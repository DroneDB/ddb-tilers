// SPDX-License-Identifier: MPL-2.0
#pragma once

// CPU topology as far as scheduling cares: the logical CPUs this process may use, grouped by
// shared L3 cache (one group per CCD on Zen, one per socket on most Intel parts).
//
// Linux: /sys/devices/system/cpu/cpu*/cache/index*/{level,shared_cpu_list,size} intersected
// with sched_getaffinity. Windows: GetLogicalProcessorInformationEx(RelationCache) level 3.

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace ddbt::exec {

struct L3Domain {
    std::vector<int> cpus;   // logical CPU numbers (Windows: group * 64 + bit)
    uint64_t cacheBytes = 0; // 0 when unknown
};

struct Topology {
    /// Logical CPUs available to the process, ascending.
    std::vector<int> cpus;
    /// L3 domains covering `cpus` (a single domain when the cache layout is unknown), ordered by
    /// their lowest CPU.
    std::vector<L3Domain> l3;

    /// Detected once per process.
    static const Topology& Get();
    /// Fresh detection.
    static Topology Detect();
    /// Linux sysfs parser with an explicit root ("/sys/devices/system/cpu") and allowed CPU set
    /// (exposed for tests).
    static Topology FromSysfs(const std::string& root, const std::vector<int>& allowed);

    /// Index of the L3 domain containing `cpu`, or -1.
    int DomainOf(int cpu) const noexcept;

    /// "32 CPUs, 2 L3 domains: [0-7,16-23] 32 MiB, [8-15,24-31] 32 MiB".
    std::string ToString() const;
};

/// Parses a Linux CPU list ("0-3,8,10-11"). Returns false on malformed input.
bool ParseCpuList(std::string_view text, std::vector<int>* out);
/// Inverse of ParseCpuList (sorted, ranges collapsed).
std::string FormatCpuList(std::vector<int> cpus);

/// Logical CPUs the calling process may run on (affinity mask), ascending.
std::vector<int> AllowedCpus();

/// Restricts the calling thread to `cpus` (Windows: CPUs of the first CPU's processor group).
/// False when the OS refuses or the list is empty.
bool PinCurrentThread(const std::vector<int>& cpus);

/// Names the calling thread (visible in perf, gdb, Task Manager); best effort.
void SetCurrentThreadName(const std::string& name);

}  // namespace ddbt::exec
