// SPDX-License-Identifier: MPL-2.0
#pragma once

#include <chrono>
#include <cstdint>

#include "core/util/counters.h"

namespace ddbt {

/// Monotonic stopwatch (steady_clock), started at construction.
class Timer {
public:
    using Clock = std::chrono::steady_clock;

    Timer() noexcept : start_(Clock::now()) {}

    void Restart() noexcept { start_ = Clock::now(); }

    uint64_t ElapsedNs() const noexcept {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start_).count());
    }
    double ElapsedUs() const noexcept { return static_cast<double>(ElapsedNs()) * 1e-3; }
    double ElapsedMs() const noexcept { return static_cast<double>(ElapsedNs()) * 1e-6; }
    double ElapsedSeconds() const noexcept { return static_cast<double>(ElapsedNs()) * 1e-9; }

    /// Elapsed nanoseconds, then restarts (lap timing).
    uint64_t LapNs() noexcept {
        const auto now = Clock::now();
        const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - start_).count();
        start_ = now;
        return static_cast<uint64_t>(ns);
    }

private:
    Clock::time_point start_;
};

/// Adds the lifetime of the scope, in nanoseconds, to a Counter.
class ScopedTimer {
public:
    explicit ScopedTimer(Counter& ns) noexcept : counter_(ns) {}
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;
    ~ScopedTimer() { counter_.Add(timer_.ElapsedNs()); }

private:
    Counter& counter_;
    Timer timer_;
};

}  // namespace ddbt
