// SPDX-License-Identifier: MPL-2.0
// Linux implementation of the ddbt-bench OS layer.
#ifndef _WIN32

#include "os.hpp"

#include "json.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <thread>

#include <dirent.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/sysinfo.h>
#include <sys/types.h>
#include <sys/utsname.h>
#include <sys/wait.h>
#include <unistd.h>

namespace fs = std::filesystem;

namespace ddbtb::os
{

// ------------------------------------------------------------------ helpers

std::string readFile(const std::string &path)
{
    // /proc files report size 0: read in chunks.
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0)
        return {};
    std::string s;
    char buf[8192];
    for (;;)
    {
        const ssize_t n = ::read(fd, buf, sizeof buf);
        if (n < 0 && errno == EINTR)
            continue;
        if (n <= 0)
            break;
        s.append(buf, static_cast<size_t>(n));
    }
    ::close(fd);
    return s;
}

static std::string trim(std::string s)
{
    while (!s.empty() && (s.back() == '\n' || s.back() == ' ' || s.back() == '\t' || s.back() == '\r'))
        s.pop_back();
    size_t i = 0;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t'))
        ++i;
    return s.substr(i);
}

static std::string readTrim(const std::string &path) { return trim(readFile(path)); }

// Value of "Key:   123 kB" at the start of a line of a /proc status-like text, -1 when absent.
static int64_t procField(const std::string &text, const char *key)
{
    const std::string k = std::string(key) + ":";
    size_t pos = 0;
    for (;;)
    {
        pos = text.find(k, pos);
        if (pos == std::string::npos)
            return -1;
        if (pos == 0 || text[pos - 1] == '\n')
            break;
        ++pos;
    }
    return std::strtoll(text.c_str() + pos + k.size(), nullptr, 10);
}

std::string captureCommand(const std::string &cmd)
{
    FILE *p = ::popen((cmd + " 2>/dev/null").c_str(), "r");
    if (!p)
        return {};
    std::string s;
    char buf[4096];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof buf, p)) > 0)
        s.append(buf, n);
    const int rc = ::pclose(p);
    if (rc != 0)
        return {};
    return trim(s);
}

std::string hostName()
{
    char buf[256] = {};
    ::gethostname(buf, sizeof buf - 1);
    return buf;
}

// ------------------------------------------------------------------ libraries

void *libOpen(const std::string &path, std::string &err)
{
    void *h = ::dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!h)
    {
        const char *e = ::dlerror();
        err = e ? e : "dlopen failed";
    }
    return h;
}

void *libSym(void *handle, const char *name) { return ::dlsym(handle, name); }

// ------------------------------------------------------------------ threads

bool pinCurrentThread(int cpu, std::string &err)
{
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    const int rc = ::pthread_setaffinity_np(::pthread_self(), sizeof set, &set);
    if (rc != 0)
    {
        err = std::string("pthread_setaffinity_np(") + std::to_string(cpu) + "): " + std::strerror(rc);
        return false;
    }
    return true;
}

int onlineCpuCount() { return static_cast<int>(::sysconf(_SC_NPROCESSORS_ONLN)); }

// ------------------------------------------------------------------ accounting

static ResUsage fromRusage(const struct rusage &ru)
{
    ResUsage u;
    u.user_s = static_cast<double>(ru.ru_utime.tv_sec) + static_cast<double>(ru.ru_utime.tv_usec) * 1e-6;
    u.sys_s = static_cast<double>(ru.ru_stime.tv_sec) + static_cast<double>(ru.ru_stime.tv_usec) * 1e-6;
    u.minflt = ru.ru_minflt;
    u.majflt = ru.ru_majflt;
    u.inblock = ru.ru_inblock;
    u.oublock = ru.ru_oublock;
    u.nvcsw = ru.ru_nvcsw;
    u.nivcsw = ru.ru_nivcsw;
    u.maxrss_kb = ru.ru_maxrss;
    return u;
}

ResUsage selfUsage()
{
    struct rusage ru{};
    ::getrusage(RUSAGE_SELF, &ru);
    return fromRusage(ru);
}

MemNow selfMem()
{
    const std::string s = readFile("/proc/self/status");
    MemNow m;
    m.rss_kb = procField(s, "VmRSS");
    m.hwm_kb = procField(s, "VmHWM");
    return m;
}

std::map<std::string, int64_t> vmstat()
{
    std::map<std::string, int64_t> r;
    std::istringstream in(readFile("/proc/vmstat"));
    std::string k;
    int64_t v;
    while (in >> k >> v)
        r[k] = v;
    return r;
}

// ------------------------------------------------------------------ process tree

namespace
{
// Children of pid through /proc/<pid>/task/<tid>/children (CONFIG_PROC_CHILDREN); the caller
// falls back to a full /proc scan of the ppid fields when that file does not exist.
bool procChildrenSupported()
{
    static const bool supported =
        ::access(("/proc/self/task/" + std::to_string(::gettid()) + "/children").c_str(), R_OK) == 0;
    return supported;
}

void childrenViaTask(int pid, std::vector<int> &out)
{
    const std::string taskDir = "/proc/" + std::to_string(pid) + "/task";
    DIR *d = ::opendir(taskDir.c_str());
    if (!d)
        return; // process gone
    while (dirent *e = ::readdir(d))
    {
        if (e->d_name[0] < '0' || e->d_name[0] > '9')
            continue;
        std::istringstream in(readFile(taskDir + "/" + e->d_name + "/children"));
        int c;
        while (in >> c)
            out.push_back(c);
    }
    ::closedir(d);
}

int parsePpid(const std::string &stat)
{
    const size_t rp = stat.rfind(')');
    if (rp == std::string::npos || rp + 4 >= stat.size())
        return -1;
    // ") S ppid ..."
    return std::atoi(stat.c_str() + rp + 4);
}

std::vector<int> processTree(int root)
{
    std::vector<int> tree{root};
    if (procChildrenSupported())
    {
        for (size_t i = 0; i < tree.size(); ++i)
            childrenViaTask(tree[i], tree);
        return tree;
    }
    std::multimap<int, int> kids;
    DIR *d = ::opendir("/proc");
    if (!d)
        return tree;
    while (dirent *e = ::readdir(d))
    {
        if (e->d_name[0] < '0' || e->d_name[0] > '9')
            continue;
        const int pid = std::atoi(e->d_name);
        const int ppid = parsePpid(readFile(std::string("/proc/") + e->d_name + "/stat"));
        if (ppid > 0)
            kids.emplace(ppid, pid);
    }
    ::closedir(d);
    for (size_t i = 0; i < tree.size(); ++i)
    {
        auto [a, b] = kids.equal_range(tree[i]);
        for (auto it = a; it != b; ++it)
            tree.push_back(it->second);
    }
    return tree;
}

void readSmaps(int pid, int64_t &rssKb, int64_t &pssKb)
{
    const std::string base = "/proc/" + std::to_string(pid);
    const std::string s = readFile(base + "/smaps_rollup");
    if (!s.empty())
    {
        rssKb = std::max<int64_t>(0, procField(s, "Rss"));
        pssKb = std::max<int64_t>(0, procField(s, "Pss"));
        return;
    }
    // Fallback: statm (pages)
    std::istringstream in(readFile(base + "/statm"));
    int64_t size = 0, res = 0;
    in >> size >> res;
    rssKb = res * (::sysconf(_SC_PAGESIZE) / 1024);
    pssKb = rssKb;
}

bool readIo(int pid, IoCounters &io)
{
    const std::string s = readFile("/proc/" + std::to_string(pid) + "/io");
    if (s.empty())
        return false;
    io.rchar = procField(s, "rchar");
    io.wchar = procField(s, "wchar");
    io.readBytes = procField(s, "read_bytes");
    io.writeBytes = procField(s, "write_bytes");
    io.cancelledWriteBytes = procField(s, "cancelled_write_bytes");
    return io.rchar >= 0;
}

void addIo(IoCounters &acc, const IoCounters &v)
{
    auto add = [](int64_t &a, int64_t b) {
        if (b < 0)
            return;
        a = (a < 0 ? 0 : a) + b;
    };
    add(acc.rchar, v.rchar);
    add(acc.wchar, v.wchar);
    add(acc.readBytes, v.readBytes);
    add(acc.writeBytes, v.writeBytes);
    add(acc.cancelledWriteBytes, v.cancelledWriteBytes);
}

void maxIo(IoCounters &acc, const IoCounters &v)
{
    acc.rchar = std::max(acc.rchar, v.rchar);
    acc.wchar = std::max(acc.wchar, v.wchar);
    acc.readBytes = std::max(acc.readBytes, v.readBytes);
    acc.writeBytes = std::max(acc.writeBytes, v.writeBytes);
    acc.cancelledWriteBytes = std::max(acc.cancelledWriteBytes, v.cancelledWriteBytes);
}
} // namespace

bool runMonitored(const RunOptions &opts, RunResult &res, std::string &err)
{
    if (opts.argv.empty())
    {
        err = "no command";
        return false;
    }
    const auto vmBefore = vmstat();

    int errPipe[2];
    if (::pipe2(errPipe, O_CLOEXEC) != 0)
    {
        err = std::string("pipe2: ") + std::strerror(errno);
        return false;
    }

    std::vector<char *> argv;
    for (const auto &a : opts.argv)
        argv.push_back(const_cast<char *>(a.c_str()));
    argv.push_back(nullptr);

    const auto t0 = std::chrono::steady_clock::now();
    const pid_t pid = ::fork();
    if (pid < 0)
    {
        err = std::string("fork: ") + std::strerror(errno);
        return false;
    }
    if (pid == 0)
    {
        // Child: own process group (so a timeout kills the whole tree), affinity, redirects.
        ::setpgid(0, 0);
        auto die = [&](int e) {
            const int code = e;
            [[maybe_unused]] ssize_t n = ::write(errPipe[1], &code, sizeof code);
            ::_exit(127);
        };
        if (!opts.cpus.empty())
        {
            cpu_set_t set;
            CPU_ZERO(&set);
            for (int c : opts.cpus)
                CPU_SET(c, &set);
            if (::sched_setaffinity(0, sizeof set, &set) != 0)
                die(errno);
        }
        if (!opts.cwd.empty() && ::chdir(opts.cwd.c_str()) != 0)
            die(errno);
        auto redirect = [&](const std::string &path, int target) {
            if (path.empty())
                return;
            const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
            if (fd < 0 || ::dup2(fd, target) < 0)
                die(errno);
        };
        redirect(opts.stdoutPath, STDOUT_FILENO);
        redirect(opts.stderrPath, STDERR_FILENO);
        ::execvp(argv[0], argv.data());
        die(errno);
    }
    ::setpgid(pid, pid); // also from the parent: no race with the sampler/timeout
    ::close(errPipe[1]);

    std::atomic<bool> stop{false};
    std::atomic<bool> timedOut{false};
    int64_t rssSum = 0;
    std::thread sampler([&] {
        while (!stop.load(std::memory_order_relaxed))
        {
            const auto tree = processTree(pid);
            int64_t rss = 0, pss = 0;
            IoCounters io;
            for (int p : tree)
            {
                int64_t r = 0, q = 0;
                readSmaps(p, r, q);
                rss += r;
                pss += q;
                IoCounters one;
                if (readIo(p, one))
                    addIo(io, one);
            }
            res.treeRssPeakKb = std::max(res.treeRssPeakKb, rss);
            res.treePssPeakKb = std::max(res.treePssPeakKb, pss);
            res.treeProcsMax = std::max(res.treeProcsMax, static_cast<int>(tree.size()));
            maxIo(res.ioSampledPeak, io);
            rssSum += rss;
            ++res.samples;
            if (opts.timeoutS > 0 &&
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > opts.timeoutS)
            {
                if (!timedOut.exchange(true))
                    ::kill(-pid, SIGKILL);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(opts.sampleMs));
        }
    });

    // Wait for the exit without reaping, so /proc/<pid>/io is still readable.
    siginfo_t si{};
    int rc;
    do
        rc = ::waitid(P_PID, static_cast<id_t>(pid), &si, WEXITED | WNOWAIT);
    while (rc != 0 && errno == EINTR);
    const auto t1 = std::chrono::steady_clock::now();
    stop = true;
    sampler.join();

    int status = 0;
    struct rusage ru{};
    while (::wait4(pid, &status, 0, &ru) < 0 && errno == EINTR)
    {
    }
    res.wallS = std::chrono::duration<double>(t1 - t0).count();
    res.usage = fromRusage(ru);
    // /proc/<pid>/io of a zombie is not readable (no mm for the ptrace check). Linux derives
    // ru_inblock/ru_oublock from the same accounting (read_bytes and write_bytes minus
    // cancelled_write_bytes, in 512-byte units) and wait4 includes the waited-for descendants:
    // exact storage I/O of the tree. rchar/wchar are the last sample (may miss the final
    // sampling period).
    res.io = res.ioSampledPeak;
    res.io.readBytes = res.usage.inblock * 512;
    res.io.writeBytes = res.usage.oublock * 512;
    res.io.cancelledWriteBytes = -1;
    res.ioSource = "rusage+sampled";
    res.timedOut = timedOut.load();
    res.treeRssMeanKb = res.samples ? rssSum / res.samples : 0;
    if (WIFEXITED(status))
        res.exitCode = WEXITSTATUS(status);
    else if (WIFSIGNALED(status))
    {
        res.termSignal = WTERMSIG(status);
        res.exitCode = 128 + res.termSignal;
    }

    int execErr = 0;
    if (::read(errPipe[0], &execErr, sizeof execErr) == static_cast<ssize_t>(sizeof execErr))
    {
        ::close(errPipe[0]);
        err = "cannot start " + opts.argv[0] + ": " + std::strerror(execErr);
        return false;
    }
    ::close(errPipe[0]);

    const auto vmAfter = vmstat();
    for (const char *k : {"pswpin", "pswpout", "pgmajfault", "pgpgin", "pgpgout"})
    {
        const auto a = vmBefore.find(k), b = vmAfter.find(k);
        if (a != vmBefore.end() && b != vmAfter.end())
            res.vmstatDelta[k] = b->second - a->second;
    }
    return true;
}

// ------------------------------------------------------------------ page cache

bool pageEvict(const std::string &path, std::string &err)
{
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0)
    {
        err = std::strerror(errno);
        return false;
    }
    // Dirty pages (a file just written, e.g. a COG converted into DDB_HOME) are not dropped by
    // DONTNEED: write them back first. A no-op for unmodified inputs.
    ::fdatasync(fd);
    const int rc = ::posix_fadvise(fd, 0, 0, POSIX_FADV_DONTNEED);
    ::close(fd);
    if (rc != 0)
    {
        err = std::string("posix_fadvise: ") + std::strerror(rc);
        return false;
    }
    return true;
}

bool pageWarm(const std::string &path, int64_t &bytesRead, std::string &err)
{
    bytesRead = 0;
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0)
    {
        err = std::strerror(errno);
        return false;
    }
    ::posix_fadvise(fd, 0, 0, POSIX_FADV_WILLNEED);
    std::vector<char> buf(1 << 20);
    for (;;)
    {
        const ssize_t n = ::read(fd, buf.data(), buf.size());
        if (n < 0 && errno == EINTR)
            continue;
        if (n < 0)
        {
            err = std::strerror(errno);
            ::close(fd);
            return false;
        }
        if (n == 0)
            break;
        bytesRead += n;
    }
    ::close(fd);
    return true;
}

Residency pageResidency(const std::string &path)
{
    Residency r;
    r.path = path;
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0)
    {
        r.error = std::strerror(errno);
        return r;
    }
    struct stat st{};
    ::fstat(fd, &st);
    r.size = st.st_size;
    const long page = ::sysconf(_SC_PAGESIZE);
    r.pages = (r.size + page - 1) / page;
    if (r.size > 0)
    {
        void *m = ::mmap(nullptr, static_cast<size_t>(r.size), PROT_READ, MAP_SHARED, fd, 0);
        if (m == MAP_FAILED)
            r.error = std::string("mmap: ") + std::strerror(errno);
        else
        {
            std::vector<unsigned char> vec(static_cast<size_t>(r.pages));
            if (::mincore(m, static_cast<size_t>(r.size), vec.data()) == 0)
                r.resident = std::count_if(vec.begin(), vec.end(), [](unsigned char c) { return c & 1; });
            else
                r.error = std::string("mincore: ") + std::strerror(errno);
            ::munmap(m, static_cast<size_t>(r.size));
        }
    }
    ::close(fd);
    return r;
}

// ------------------------------------------------------------------ sysinfo

namespace
{
std::vector<int> parseCpuList(const std::string &s)
{
    std::vector<int> r;
    std::stringstream ss(s);
    std::string part;
    while (std::getline(ss, part, ','))
    {
        part = trim(part);
        if (part.empty())
            continue;
        const size_t dash = part.find('-');
        if (dash == std::string::npos)
            r.push_back(std::atoi(part.c_str()));
        else
            for (int i = std::atoi(part.c_str()), e = std::atoi(part.c_str() + dash + 1); i <= e; ++i)
                r.push_back(i);
    }
    return r;
}

std::string cpuinfoField(const std::string &cpuinfo, const char *key)
{
    const std::string k = std::string("\n") + key;
    size_t pos = cpuinfo.find(k);
    if (pos == std::string::npos)
        return {};
    pos = cpuinfo.find(':', pos);
    const size_t end = cpuinfo.find('\n', pos);
    return trim(cpuinfo.substr(pos + 1, end - pos - 1));
}

void writeCpuArray(JsonWriter &w, const std::vector<int> &v)
{
    w.beginArray(true);
    for (int c : v)
        w.value(c);
    w.endArray();
}

std::string isaLevel(const std::set<std::string> &f)
{
    auto has = [&](std::initializer_list<const char *> l) {
        for (const char *x : l)
            if (!f.count(x))
                return false;
        return true;
    };
    if (!has({"cx16", "lahf_lm", "popcnt", "sse4_1", "sse4_2", "ssse3"}))
        return "x86-64";
    if (!has({"avx", "avx2", "bmi1", "bmi2", "f16c", "fma", "abm", "movbe", "xsave"}))
        return "x86-64-v2";
    if (!has({"avx512f", "avx512bw", "avx512cd", "avx512dq", "avx512vl"}))
        return "x86-64-v3";
    return "x86-64-v4";
}
} // namespace

void writeSysinfo(JsonWriter &w, const SysinfoOptions &opts)
{
    const std::string cpuinfo = "\n" + readFile("/proc/cpuinfo");
    const std::string sys = "/sys/devices/system/cpu";

    struct utsname un{};
    ::uname(&un);
    w.kv("hostname", hostName());
    w.key("kernel").beginObject(true);
    w.kv("sysname", un.sysname).kv("release", un.release).kv("version", un.version).kv("machine", un.machine);
    w.endObject();
    const std::string osRelease = readFile("/etc/os-release");
    {
        std::string pretty;
        std::istringstream in(osRelease);
        std::string line;
        while (std::getline(in, line))
            if (line.rfind("PRETTY_NAME=", 0) == 0)
            {
                pretty = line.substr(12);
                if (pretty.size() >= 2 && pretty.front() == '"')
                    pretty = pretty.substr(1, pretty.size() - 2);
            }
        w.kv("os", pretty);
    }

    // --- CPU
    w.key("cpu").beginObject();
    w.kv("model", cpuinfoField(cpuinfo, "model name"));
    w.kv("vendor", cpuinfoField(cpuinfo, "vendor_id"));
    w.kv("family", cpuinfoField(cpuinfo, "cpu family"));
    w.kv("model_id", cpuinfoField(cpuinfo, "model\t"));
    w.kv("stepping", cpuinfoField(cpuinfo, "stepping"));
    w.kv("microcode", cpuinfoField(cpuinfo, "microcode"));
    std::set<std::string> flags;
    {
        std::istringstream in(cpuinfoField(cpuinfo, "flags"));
        std::string f;
        while (in >> f)
            flags.insert(f);
    }
    w.kv("isa_level", isaLevel(flags));
    w.key("flags_of_interest").beginArray(true);
    for (const char *f : {"sse4_2", "avx", "avx2", "fma", "bmi2", "avx512f", "avx512bw", "avx512vl", "avx512dq",
                          "avx512cd", "avx512vbmi", "avx512vbmi2", "avx512_vnni", "avx512_bf16", "avx512_vp2intersect",
                          "vaes", "vpclmulqdq", "gfni", "sha_ni", "movdir64b", "avx_vnni"})
        if (flags.count(f))
            w.value(f);
    w.endArray();
    w.key("flags").beginArray(true);
    for (const auto &f : flags)
        w.value(f);
    w.endArray();

    const std::vector<int> online = parseCpuList(readTrim(sys + "/online"));
    w.kv("online", readTrim(sys + "/online"));
    w.kv("logical_cpus", static_cast<int64_t>(online.size()));

    // Per-CPU topology: core, package, L3 group (CCD on Zen), SMT siblings, prefcore ranking.
    std::vector<std::string> l3Groups;
    struct CpuRow
    {
        int cpu, core, pkg, l3, prefcore;
        std::string siblings;
        int64_t maxFreqKhz;
    };
    std::vector<CpuRow> rows;
    for (int c : online)
    {
        const std::string base = sys + "/cpu" + std::to_string(c);
        CpuRow r{};
        r.cpu = c;
        r.core = std::atoi(readTrim(base + "/topology/core_id").c_str());
        r.pkg = std::atoi(readTrim(base + "/topology/physical_package_id").c_str());
        r.siblings = readTrim(base + "/topology/thread_siblings_list");
        std::string l3 = readTrim(base + "/cache/index3/shared_cpu_list");
        auto it = std::find(l3Groups.begin(), l3Groups.end(), l3);
        if (it == l3Groups.end())
        {
            l3Groups.push_back(l3);
            r.l3 = static_cast<int>(l3Groups.size() - 1);
        }
        else
            r.l3 = static_cast<int>(it - l3Groups.begin());
        const std::string pc = readTrim(base + "/cpufreq/amd_pstate_prefcore_ranking");
        r.prefcore = pc.empty() ? -1 : std::atoi(pc.c_str());
        const std::string mf = readTrim(base + "/cpufreq/cpuinfo_max_freq");
        r.maxFreqKhz = mf.empty() ? -1 : std::atoll(mf.c_str());
        rows.push_back(r);
    }
    w.kv("l3_size", readTrim(sys + "/cpu0/cache/index3/size"));
    w.key("l3_groups").beginArray();
    for (size_t i = 0; i < l3Groups.size(); ++i)
    {
        w.beginObject(true);
        w.kv("id", static_cast<int64_t>(i));
        w.key("cpus");
        writeCpuArray(w, parseCpuList(l3Groups[i]));
        w.endObject();
    }
    w.endArray();
    w.key("cpus").beginArray();
    for (const auto &r : rows)
    {
        w.beginObject(true);
        w.kv("cpu", r.cpu).kv("core", r.core).kv("package", r.pkg).kv("l3", r.l3).kv("smt_siblings", r.siblings);
        if (r.prefcore >= 0)
            w.kv("prefcore_ranking", r.prefcore);
        if (r.maxFreqKhz >= 0)
            w.kv("max_freq_khz", r.maxFreqKhz);
        w.endObject();
    }
    w.endArray();
    w.endObject(); // cpu

    // --- frequency / power policy
    w.key("power").beginObject();
    {
        std::set<std::string> govs, epps;
        for (int c : online)
        {
            const std::string base = sys + "/cpu" + std::to_string(c) + "/cpufreq/";
            const std::string g = readTrim(base + "scaling_governor");
            if (!g.empty())
                govs.insert(g);
            const std::string e = readTrim(base + "energy_performance_preference");
            if (!e.empty())
                epps.insert(e);
        }
        w.key("governors").beginArray(true);
        for (const auto &g : govs)
            w.value(g);
        w.endArray();
        w.key("energy_performance_preference").beginArray(true);
        for (const auto &e : epps)
            w.value(e);
        w.endArray();
        w.kv("scaling_driver", readTrim(sys + "/cpu0/cpufreq/scaling_driver"));
        w.kv("amd_pstate_status", readTrim(sys + "/amd_pstate/status"));
        w.kv("amd_pstate_prefcore", readTrim(sys + "/amd_pstate/prefcore"));
        const std::string boost = readTrim(sys + "/cpufreq/boost");
        const std::string boost0 = readTrim(sys + "/cpu0/cpufreq/boost");
        w.kv("boost", !boost.empty() ? boost : boost0);
        w.kv("smt_active", readTrim(sys + "/smt/active"));
    }
    w.endObject();

    // --- memory
    {
        const std::string mi = readFile("/proc/meminfo");
        w.key("memory").beginObject(true);
        w.kv("total_kb", procField(mi, "MemTotal"))
            .kv("available_kb", procField(mi, "MemAvailable"))
            .kv("swap_total_kb", procField(mi, "SwapTotal"))
            .kv("swap_free_kb", procField(mi, "SwapFree"))
            .kv("hugepages_total", procField(mi, "HugePages_Total"))
            .kv("hugepagesize_kb", procField(mi, "Hugepagesize"));
        w.endObject();
    }
    w.key("thp").beginObject(true);
    w.kv("enabled", readTrim("/sys/kernel/mm/transparent_hugepage/enabled"))
        .kv("defrag", readTrim("/sys/kernel/mm/transparent_hugepage/defrag"));
    w.endObject();
    w.key("kernel_params").beginObject(true);
    w.kv("perf_event_paranoid", readTrim("/proc/sys/kernel/perf_event_paranoid"))
        .kv("swappiness", readTrim("/proc/sys/vm/swappiness"))
        .kv("numa_balancing", readTrim("/proc/sys/kernel/numa_balancing"));
    w.endObject();

    // --- load and temperature
    {
        std::istringstream in(readFile("/proc/loadavg"));
        double l1 = 0, l5 = 0, l15 = 0;
        in >> l1 >> l5 >> l15;
        w.key("loadavg").beginArray(true).value(l1).value(l5).value(l15).endArray();
    }
    w.key("temperatures").beginObject(true);
    std::error_code ec;
    for (const auto &e : fs::directory_iterator("/sys/class/hwmon", ec))
    {
        const std::string name = readTrim(e.path().string() + "/name");
        if (name != "k10temp" && name != "coretemp" && name != "zenpower")
            continue;
        for (int i = 1; i <= 16; ++i)
        {
            const std::string p = e.path().string() + "/temp" + std::to_string(i);
            const std::string val = readTrim(p + "_input");
            if (val.empty())
                continue;
            std::string label = readTrim(p + "_label");
            if (label.empty())
                label = "temp" + std::to_string(i);
            w.kv(name + "." + label, std::atof(val.c_str()) / 1000.0);
        }
    }
    w.endObject();

    // --- build of the driver
    w.key("driver").beginObject(true);
#if defined(__clang__)
    w.kv("compiler", std::string("clang ") + __clang_version__);
#elif defined(__GNUC__)
    w.kv("compiler", std::string("gcc ") + __VERSION__);
#else
    w.kv("compiler", "unknown");
#endif
#ifdef NDEBUG
    w.kv("ndebug", true);
#else
    w.kv("ndebug", false);
#endif
    w.kv("cplusplus", static_cast<int64_t>(__cplusplus));
    w.endObject();

    // --- repositories
    w.key("repos").beginObject();
    for (const auto &[name, path] : opts.repos)
    {
        w.key(name).beginObject(true);
        w.kv("path", path);
        const std::string q = "'" + path + "'";
        const std::string sha = captureCommand("git -C " + q + " rev-parse HEAD");
        if (sha.empty())
            w.key("sha").null();
        else
            w.kv("sha", sha);
        const std::string dirty = captureCommand("git -C " + q + " status --porcelain --untracked-files=no | wc -l");
        if (!dirty.empty())
            w.kv("dirty_files", static_cast<int64_t>(std::atoll(dirty.c_str())));
        const std::string branch = captureCommand("git -C " + q + " rev-parse --abbrev-ref HEAD");
        if (!branch.empty())
            w.kv("branch", branch);
        w.endObject();
    }
    w.endObject();

    // --- baseline
    w.key("baseline").beginObject(true);
    w.kv("dir", opts.baselineDir);
    {
        std::string manifest;
        // The snapshot script writes MANIFEST.json next to the extracted tree.
        for (const std::string &dir : {opts.baselineDir, opts.baselineDir + "/.."})
            for (const char *m : {"MANIFEST.json", "MANIFEST", "MANIFEST.sha256", "SHA256SUMS"})
            {
                const std::string p = dir + "/" + m;
                if (manifest.empty() && ::access(p.c_str(), R_OK) == 0)
                    manifest = fs::weakly_canonical(p).string();
            }
        if (manifest.empty())
            w.key("manifest").null();
        else
        {
            w.kv("manifest", manifest);
            const std::string sum = captureCommand("sha256sum '" + manifest + "'");
            w.kv("manifest_sha256", sum.substr(0, sum.find(' ')));
        }
    }
    w.endObject();
}

} // namespace ddbtb::os

#endif // !_WIN32
