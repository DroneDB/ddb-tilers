// SPDX-License-Identifier: MPL-2.0
#pragma once

#include <cstdint>
#include <string>

namespace ddbt::cache {

struct CacheStats {
    uint64_t hits = 0;          // Get / GetOrLoad served from the cache
    uint64_t misses = 0;        // lookups not served from the cache (includes coalesced waiters)
    uint64_t loads = 0;         // loader runs (single-flight leaders)
    uint64_t coalesced = 0;     // GetOrLoad calls that waited for another thread's load
    uint64_t loadFailures = 0;  // loaders that threw
    uint64_t inserts = 0;
    uint64_t evictions = 0;     // entries dropped to respect the byte budget
    uint64_t rejected = 0;      // values not cached (larger than a shard, or MemBudget full)
    uint64_t entries = 0;       // current
    uint64_t bytes = 0;         // current cached bytes

    double HitRate() const noexcept {
        const uint64_t lookups = hits + misses;
        return lookups == 0 ? 0.0 : static_cast<double>(hits) / static_cast<double>(lookups);
    }
};

/// {"hits":1,"misses":2,...} for the benchmark reports.
std::string ToJson(const CacheStats& s);

}  // namespace ddbt::cache
