// SPDX-License-Identifier: MPL-2.0
// Operating-system layer of ddbt-bench. os_linux.cpp is the implementation; os_win.cpp holds
// the Windows stubs (TODO: LoadLibrary, SetThreadAffinityMask, Job Objects,
// GetProcessMemoryInfo, QueryProcessCycleTime) so that the driver compiles with MSVC.
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ddbtb
{
class JsonWriter;
}

namespace ddbtb::os
{

// ---- dynamic libraries
void *libOpen(const std::string &path, std::string &err);
void *libSym(void *handle, const char *name);

// ---- threads and CPUs
// Pins the calling thread to one CPU. Returns false and sets err on failure.
bool pinCurrentThread(int cpu, std::string &err);
int onlineCpuCount();

// ---- self accounting
struct ResUsage
{
    double user_s = 0, sys_s = 0;
    int64_t minflt = 0, majflt = 0, inblock = 0, oublock = 0, nvcsw = 0, nivcsw = 0, maxrss_kb = 0;
};
ResUsage selfUsage();

struct MemNow
{
    int64_t rss_kb = -1;
    int64_t hwm_kb = -1;
};
MemNow selfMem();

// ---- system counters
// pswpin, pswpout, pgmajfault, ... from /proc/vmstat (empty map when unavailable).
std::map<std::string, int64_t> vmstat();

// ---- monitored child process ("run" subcommand)
struct RunOptions
{
    std::vector<std::string> argv;
    std::vector<int> cpus; // affinity of the child (empty: inherited)
    std::string stdoutPath, stderrPath, cwd;
    double timeoutS = 0; // 0: none
    int sampleMs = 20;
};

struct IoCounters
{
    int64_t rchar = -1, wchar = -1, readBytes = -1, writeBytes = -1, cancelledWriteBytes = -1;
};

struct RunResult
{
    int exitCode = -1;
    int termSignal = 0;
    bool timedOut = false;
    double wallS = 0;
    ResUsage usage; // wait4 rusage: the child plus its waited-for descendants
    int64_t treeRssPeakKb = 0, treePssPeakKb = 0, treeRssMeanKb = 0;
    int treeProcsMax = 0;
    int64_t samples = 0;
    IoCounters io;                 // read/write_bytes exact from wait4 (tree), rchar/wchar sampled
    std::string ioSource;
    IoCounters ioSampledPeak;      // max over samples of the sum over the live tree
    std::map<std::string, int64_t> vmstatDelta;
};

bool runMonitored(const RunOptions &opts, RunResult &res, std::string &err);

// ---- page cache
struct Residency
{
    std::string path;
    int64_t size = 0, pages = 0, resident = 0;
    std::string error;
};
bool pageEvict(const std::string &path, std::string &err);
bool pageWarm(const std::string &path, int64_t &bytesRead, std::string &err);
Residency pageResidency(const std::string &path);

// ---- sysinfo
struct SysinfoOptions
{
    std::vector<std::pair<std::string, std::string>> repos; // name -> path
    std::string baselineDir;
};
void writeSysinfo(JsonWriter &w, const SysinfoOptions &opts);

// ---- misc
// Runs a shell command and returns its stdout (trailing newline stripped); empty on failure.
std::string captureCommand(const std::string &cmd);
std::string readFile(const std::string &path);
std::string hostName();

} // namespace ddbtb::os
