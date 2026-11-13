// SPDX-License-Identifier: MPL-2.0
#pragma once

// Named atomic counters (event counts, bytes, gauges), snapshotted to JSON for the benchmarks.
//
//   static Counter& reads = Counters::Global().Get("io.reads");   // look up once
//   reads.Add(1);                                                  // relaxed atomic add
//   std::string json = Counters::Global().ToJson();                // {"io.reads": 12, ...}
//
// A Counter lives as long as its registry and never moves, so references can be cached. Each
// counter has its own cache line, so unrelated hot counters do not false-share.

#include <atomic>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace ddbt {

class alignas(64) Counter {
public:
    void Add(uint64_t n = 1) noexcept { v_.fetch_add(n, std::memory_order_relaxed); }
    void Sub(uint64_t n = 1) noexcept { v_.fetch_sub(n, std::memory_order_relaxed); }
    void Set(uint64_t n) noexcept { v_.store(n, std::memory_order_relaxed); }
    /// Raises the value to `n` if it is larger (peak gauges).
    void Max(uint64_t n) noexcept {
        uint64_t cur = v_.load(std::memory_order_relaxed);
        while (n > cur && !v_.compare_exchange_weak(cur, n, std::memory_order_relaxed)) {
        }
    }
    uint64_t Load() const noexcept { return v_.load(std::memory_order_relaxed); }
    void Reset() noexcept { v_.store(0, std::memory_order_relaxed); }

private:
    std::atomic<uint64_t> v_{0};
};

class Counters {
public:
    Counters() = default;
    Counters(const Counters&) = delete;
    Counters& operator=(const Counters&) = delete;

    /// The process-wide registry (io.*, exec.*, mem.*, cache.* counters of the core).
    static Counters& Global();

    /// Counter called `name`, created at 0 on first use. Thread-safe; the reference stays valid
    /// for the lifetime of the registry.
    Counter& Get(std::string_view name);

    /// Name/value pairs sorted by name.
    std::vector<std::pair<std::string, uint64_t>> Snapshot() const;

    /// Snapshot as a JSON object {"name": value, ...} (sorted by name, no whitespace).
    std::string ToJson() const;

    /// Sets every counter to 0 (between benchmark runs).
    void ResetAll();

private:
    mutable std::mutex mutex_;
    std::map<std::string, Counter*, std::less<>> byName_;
    std::deque<Counter> storage_;  // deque: stable addresses
};

}  // namespace ddbt
