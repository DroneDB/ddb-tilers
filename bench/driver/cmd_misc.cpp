// SPDX-License-Identifier: MPL-2.0
// ddbt-bench run | pagecache | sysinfo | hash
#include "common.hpp"
#include "json.hpp"
#include "os.hpp"
#include "xxh64.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

namespace ddbtb
{

// ------------------------------------------------------------------ run

int cmdRun(int argc, char **argv)
{
    Args a(argc, argv, 2, {"help"});
    if (a.has("help") || a.rest.empty())
    {
        std::puts(R"(usage: ddbt-bench run [--out file.json|-] [--cpus CPULIST] [--cwd DIR]
         [--stdout FILE] [--stderr FILE] [--timeout SECONDS] [--sample-ms 20] -- cmd args...
Runs cmd in its own process group and reports wall/user/sys time, rusage of the tree
(wait4), the peak RSS/PSS sum of the process tree (smaps_rollup every --sample-ms),
I/O bytes (/proc/<pid>/io at exit), and the /proc/vmstat swap/fault deltas.
The exit code of ddbt-bench is the exit code of cmd (125 when cmd could not be started).)");
        return a.has("help") ? 0 : 2;
    }
    a.allowOnly({"out", "cpus", "cwd", "stdout", "stderr", "timeout", "sample-ms", "help"});
    os::RunOptions o;
    o.argv = a.rest;
    o.cpus = parseCpuList(a.get("cpus"));
    o.cwd = a.get("cwd");
    o.stdoutPath = a.get("stdout");
    o.stderrPath = a.get("stderr");
    o.timeoutS = a.getDouble("timeout", 0);
    o.sampleMs = static_cast<int>(a.getInt("sample-ms", 20));
    if (o.sampleMs < 1)
        throw UsageError("--sample-ms must be >= 1");

    os::RunResult r;
    std::string err;
    const bool started = os::runMonitored(o, r, err);

    std::string out;
    JsonWriter w(out);
    w.beginObject();
    w.kv("schema", "ddbt-bench-run/1");
    w.key("cmd").beginArray(true);
    for (const auto &s : o.argv)
        w.value(s);
    w.endArray();
    w.key("cpus").beginArray(true);
    for (int c : o.cpus)
        w.value(c);
    w.endArray();
    w.kv("started", started);
    if (!started)
        w.kv("error", err);
    w.kv("exit_code", r.exitCode).kv("signal", r.termSignal).kv("timed_out", r.timedOut);
    w.kv("wall_s", r.wallS).kv("user_s", r.usage.user_s).kv("sys_s", r.usage.sys_s);
    w.kv("cpu_utilization", r.wallS > 0 ? (r.usage.user_s + r.usage.sys_s) / r.wallS : 0.0);
    w.key("rusage").beginObject(true);
    w.kv("maxrss_kb", r.usage.maxrss_kb).kv("minflt", r.usage.minflt).kv("majflt", r.usage.majflt);
    w.kv("inblock", r.usage.inblock).kv("oublock", r.usage.oublock);
    w.kv("nvcsw", r.usage.nvcsw).kv("nivcsw", r.usage.nivcsw);
    w.endObject();
    w.key("tree").beginObject(true);
    w.kv("rss_peak_kb", r.treeRssPeakKb).kv("pss_peak_kb", r.treePssPeakKb).kv("rss_mean_kb", r.treeRssMeanKb);
    w.kv("procs_max", r.treeProcsMax).kv("samples", r.samples).kv("sample_ms", o.sampleMs);
    w.endObject();
    w.key("io").beginObject(true);
    w.kv("source", r.ioSource);
    w.kv("read_bytes", r.io.readBytes).kv("write_bytes", r.io.writeBytes);
    w.kv("rchar", r.io.rchar).kv("wchar", r.io.wchar).kv("cancelled_write_bytes", r.io.cancelledWriteBytes);
    w.endObject();
    w.key("io_sampled_peak").beginObject(true);
    w.kv("read_bytes", r.ioSampledPeak.readBytes).kv("write_bytes", r.ioSampledPeak.writeBytes);
    w.kv("rchar", r.ioSampledPeak.rchar).kv("wchar", r.ioSampledPeak.wchar);
    w.endObject();
    w.key("vmstat_delta").beginObject(true);
    for (const auto &[k, v] : r.vmstatDelta)
        w.kv(k, v);
    w.endObject();
    w.endObject();
    writeOutput(a.get("out", "-"), out);
    if (!started)
    {
        std::fprintf(stderr, "ddbt-bench run: %s\n", err.c_str());
        return 125;
    }
    return r.exitCode;
}

// ------------------------------------------------------------------ pagecache

namespace
{
std::vector<std::string> expandPaths(const std::vector<std::string> &in)
{
    std::vector<std::string> out;
    for (const auto &p : in)
    {
        std::error_code ec;
        if (fs::is_directory(p, ec))
        {
            for (auto it = fs::recursive_directory_iterator(p, fs::directory_options::skip_permission_denied, ec);
                 it != fs::recursive_directory_iterator(); it.increment(ec))
            {
                if (ec)
                    break;
                if (it->is_regular_file(ec))
                    out.push_back(it->path().string());
            }
        }
        else
            out.push_back(p);
    }
    return out;
}
} // namespace

int cmdPagecache(int argc, char **argv)
{
    Args a(argc, argv, 2, {"help"});
    if (a.has("help") || a.positional.size() < 2)
    {
        std::puts(R"(usage: ddbt-bench pagecache evict|warm|resident [--out file.json|-] PATH...
  evict     posix_fadvise(DONTNEED) on each file, then report the residency left
  warm      posix_fadvise(WILLNEED) and read each file completely
  resident  mincore() residency of each file
Directories are walked recursively. Exit code 1 when a file failed (or, for evict,
when more than --max-resident-pct, default 1, stays resident).)");
        return a.has("help") ? 0 : 2;
    }
    a.allowOnly({"out", "max-resident-pct", "help"});
    const std::string action = a.positional[0];
    if (action != "evict" && action != "warm" && action != "resident")
        throw UsageError("pagecache action must be evict, warm or resident");
    const double maxPct = a.getDouble("max-resident-pct", 1.0);
    const auto files = expandPaths({a.positional.begin() + 1, a.positional.end()});

    std::string out;
    JsonWriter w(out);
    w.beginObject();
    w.kv("schema", "ddbt-bench-pagecache/1").kv("action", action);
    w.key("files").beginArray();
    int64_t tSize = 0, tPages = 0, tRes = 0, tRead = 0;
    bool failed = false;
    for (const auto &f : files)
    {
        std::string err;
        int64_t bytesRead = 0;
        bool ok = true;
        if (action == "evict")
            ok = os::pageEvict(f, err);
        else if (action == "warm")
            ok = os::pageWarm(f, bytesRead, err);
        const os::Residency r = os::pageResidency(f);
        if (err.empty())
            err = r.error;
        if (!ok || !r.error.empty())
            failed = true;
        tSize += r.size;
        tPages += r.pages;
        tRes += r.resident;
        tRead += bytesRead;
        w.beginObject(true);
        w.kv("path", f).kv("size", r.size).kv("pages", r.pages).kv("resident", r.resident);
        w.kv("resident_pct", r.pages ? 100.0 * static_cast<double>(r.resident) / static_cast<double>(r.pages) : 0.0);
        if (action == "warm")
            w.kv("bytes_read", bytesRead);
        if (!err.empty())
            w.kv("error", err);
        w.endObject();
    }
    w.endArray();
    const double pct = tPages ? 100.0 * static_cast<double>(tRes) / static_cast<double>(tPages) : 0.0;
    w.key("total").beginObject(true);
    w.kv("files", static_cast<int64_t>(files.size())).kv("size", tSize).kv("pages", tPages).kv("resident", tRes);
    w.kv("resident_pct", pct);
    if (action == "warm")
        w.kv("bytes_read", tRead);
    w.endObject();
    if (action == "evict" && pct > maxPct)
    {
        w.kv("warning", "pages still resident after DONTNEED (dirty, mapped or locked by another process)");
        failed = true;
    }
    w.endObject();
    writeOutput(a.get("out", "-"), out);
    return failed ? 1 : 0;
}

// ------------------------------------------------------------------ sysinfo

int cmdSysinfo(int argc, char **argv)
{
    Args a(argc, argv, 2, {"help"});
    if (a.has("help"))
    {
        std::puts(R"(usage: ddbt-bench sysinfo [--out file.json|-] [--repo name=path]... [--baseline DIR]
Defaults: --repo ddb-tilers=/mnt/data/repos/ddb-tilers --repo DroneDB=/mnt/data/repos/DroneDB
          --baseline $DDBT_BASELINE_DIR or /mnt/data/ddb-tilers-work/baseline/ddb-2.4.1)");
        return 0;
    }
    a.allowOnly({"out", "repo", "baseline", "help"});
    os::SysinfoOptions o;
    for (const auto &r : a.all("repo"))
    {
        const size_t eq = r.find('=');
        if (eq == std::string::npos)
            throw UsageError("--repo expects name=path");
        o.repos.emplace_back(r.substr(0, eq), r.substr(eq + 1));
    }
    if (o.repos.empty())
    {
        o.repos.emplace_back("ddb-tilers", "/mnt/data/repos/ddb-tilers");
        o.repos.emplace_back("DroneDB", "/mnt/data/repos/DroneDB");
    }
    const char *envBase = std::getenv("DDBT_BASELINE_DIR");
    o.baselineDir = a.get("baseline", envBase ? envBase : "/mnt/data/ddb-tilers-work/baseline/ddb-2.4.1");

    std::string out;
    JsonWriter w(out);
    w.beginObject();
    w.kv("schema", "ddbt-bench-sysinfo/1");
    os::writeSysinfo(w, o);
    w.endObject();
    writeOutput(a.get("out", "-"), out);
    return 0;
}

// ------------------------------------------------------------------ hash

int cmdHash(int argc, char **argv)
{
    Args a(argc, argv, 2, {"help"});
    if (a.has("help") || a.positional.empty())
    {
        std::puts("usage: ddbt-bench hash FILE...   (XXH64, seed 0, as used for tile hashes)");
        return a.has("help") ? 0 : 2;
    }
    int rc = 0;
    for (const auto &f : a.positional)
    {
        std::ifstream in(f, std::ios::binary);
        if (!in)
        {
            std::fprintf(stderr, "cannot read %s\n", f.c_str());
            rc = 1;
            continue;
        }
        const std::string data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        std::printf("%s  %s\n", JsonWriter::hex64(Xxh64::hash(data.data(), data.size())).c_str(), f.c_str());
    }
    return rc;
}

} // namespace ddbtb
