// SPDX-License-Identifier: MPL-2.0
#pragma once

// Process-wide I/O counters, kept in Counters::Global() under "io.*" so they appear in the
// benchmark JSON next to the other counters.

#include <cstdint>

namespace ddbt::io {

struct IoStatsSnapshot {
    uint64_t reads = 0;           // positional reads (pread / ReadFile calls that returned data)
    uint64_t readBytes = 0;
    uint64_t mappedReads = 0;     // ByteSource::Read served by memcpy from a mapping
    uint64_t mappedReadBytes = 0;
    uint64_t callbackReads = 0;   // RangeCallbackSource calls
    uint64_t callbackReadBytes = 0;
    uint64_t writes = 0;          // pwrite / WriteFile calls
    uint64_t writeBytes = 0;
    uint64_t prefetches = 0;      // posix_fadvise / madvise WILLNEED / PrefetchVirtualMemory
    uint64_t opens = 0;
};

class IoStats {
public:
    static void AddRead(uint64_t bytes) noexcept;
    static void AddMappedRead(uint64_t bytes) noexcept;
    static void AddCallbackRead(uint64_t bytes) noexcept;
    static void AddWrite(uint64_t bytes) noexcept;
    static void AddPrefetch() noexcept;
    static void AddOpen() noexcept;

    static IoStatsSnapshot Snapshot() noexcept;
    /// Resets the io.* counters (only).
    static void Reset() noexcept;
};

/// Counter difference b - a (per field), for "I/O of this operation" measurements.
IoStatsSnapshot operator-(const IoStatsSnapshot& b, const IoStatsSnapshot& a) noexcept;

}  // namespace ddbt::io
