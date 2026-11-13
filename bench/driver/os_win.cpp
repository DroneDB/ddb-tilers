// SPDX-License-Identifier: MPL-2.0
// Windows layer of ddbt-bench: library loading, pinning and self accounting are implemented;
// the rest are stubs that report "unsupported" so that the driver compiles with MSVC.
// TODO(windows):
//   - runMonitored: CreateProcess suspended + Job Object (JOBOBJECT_EXTENDED_LIMIT_INFORMATION.
//     PeakJobMemoryUsed, JOBOBJECT_BASIC_AND_IO_ACCOUNTING_INFORMATION), ResumeThread, sample
//     the job's processes (QueryInformationJobObject JobObjectBasicProcessIdList +
//     GetProcessMemoryInfo) every sampleMs.
//   - pageEvict: open with FILE_FLAG_NO_BUFFERING is not an eviction; use
//     SetSystemFileCacheSize / standby list purge (needs privileges) or document "unsupported".
//   - pageResidency: no mincore equivalent (QueryWorkingSetEx only covers mapped views).
//   - writeSysinfo: GetLogicalProcessorInformationEx (cores, L3 groups), registry CPU name.
#ifdef _WIN32

#include "os.hpp"

#include "json.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <psapi.h>

#include <cstdio>
#include <fstream>
#include <sstream>

namespace ddbtb::os
{

void *libOpen(const std::string &path, std::string &err)
{
    HMODULE h = ::LoadLibraryA(path.c_str());
    if (!h)
        err = "LoadLibrary failed, error " + std::to_string(::GetLastError());
    return reinterpret_cast<void *>(h);
}

void *libSym(void *handle, const char *name)
{
    return reinterpret_cast<void *>(::GetProcAddress(static_cast<HMODULE>(handle), name));
}

bool pinCurrentThread(int cpu, std::string &err)
{
    // TODO(windows): processor groups above 64 logical CPUs (SetThreadGroupAffinity).
    if (cpu >= 64 || ::SetThreadAffinityMask(::GetCurrentThread(), DWORD_PTR(1) << cpu) == 0)
    {
        err = "SetThreadAffinityMask failed for cpu " + std::to_string(cpu);
        return false;
    }
    return true;
}

int onlineCpuCount()
{
    SYSTEM_INFO si;
    ::GetSystemInfo(&si);
    return static_cast<int>(si.dwNumberOfProcessors);
}

static double ftSeconds(const FILETIME &ft)
{
    ULARGE_INTEGER u;
    u.LowPart = ft.dwLowDateTime;
    u.HighPart = ft.dwHighDateTime;
    return static_cast<double>(u.QuadPart) * 1e-7;
}

ResUsage selfUsage()
{
    ResUsage u;
    FILETIME c, e, k, us;
    if (::GetProcessTimes(::GetCurrentProcess(), &c, &e, &k, &us))
    {
        u.user_s = ftSeconds(us);
        u.sys_s = ftSeconds(k);
    }
    PROCESS_MEMORY_COUNTERS pmc{};
    if (::GetProcessMemoryInfo(::GetCurrentProcess(), &pmc, sizeof pmc))
    {
        u.minflt = pmc.PageFaultCount;
        u.maxrss_kb = static_cast<int64_t>(pmc.PeakWorkingSetSize / 1024);
    }
    return u;
}

MemNow selfMem()
{
    MemNow m;
    PROCESS_MEMORY_COUNTERS pmc{};
    if (::GetProcessMemoryInfo(::GetCurrentProcess(), &pmc, sizeof pmc))
    {
        m.rss_kb = static_cast<int64_t>(pmc.WorkingSetSize / 1024);
        m.hwm_kb = static_cast<int64_t>(pmc.PeakWorkingSetSize / 1024);
    }
    return m;
}

std::map<std::string, int64_t> vmstat() { return {}; }

bool runMonitored(const RunOptions &, RunResult &, std::string &err)
{
    err = "run: not implemented on Windows yet (TODO: Job Objects)";
    return false;
}

bool pageEvict(const std::string &, std::string &err)
{
    err = "pagecache evict: not implemented on Windows";
    return false;
}

bool pageWarm(const std::string &path, int64_t &bytesRead, std::string &err)
{
    bytesRead = 0;
    std::ifstream in(path, std::ios::binary);
    if (!in)
    {
        err = "cannot open";
        return false;
    }
    std::string buf(1 << 20, '\0');
    while (in.read(buf.data(), static_cast<std::streamsize>(buf.size())) || in.gcount() > 0)
        bytesRead += in.gcount();
    return true;
}

Residency pageResidency(const std::string &path)
{
    Residency r;
    r.path = path;
    r.error = "residency: not implemented on Windows";
    return r;
}

void writeSysinfo(JsonWriter &w, const SysinfoOptions &opts)
{
    w.kv("hostname", hostName());
    w.kv("os", "windows");
    w.key("cpu").beginObject(true);
    w.kv("logical_cpus", static_cast<int64_t>(onlineCpuCount()));
    w.endObject();
    w.kv("todo", "GetLogicalProcessorInformationEx topology, registry CPU name, memory status");
    w.key("repos").beginObject();
    for (const auto &[name, path] : opts.repos)
    {
        w.key(name).beginObject(true);
        w.kv("path", path).kv("sha", captureCommand("git -C \"" + path + "\" rev-parse HEAD"));
        w.endObject();
    }
    w.endObject();
}

std::string captureCommand(const std::string &cmd)
{
    FILE *p = ::_popen(cmd.c_str(), "r");
    if (!p)
        return {};
    std::string s;
    char buf[4096];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof buf, p)) > 0)
        s.append(buf, n);
    ::_pclose(p);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r'))
        s.pop_back();
    return s;
}

std::string readFile(const std::string &path)
{
    std::ifstream in(path, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::string hostName()
{
    char buf[256] = {};
    DWORD n = sizeof buf;
    ::GetComputerNameA(buf, &n);
    return buf;
}

} // namespace ddbtb::os

#endif // _WIN32
