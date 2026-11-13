// SPDX-License-Identifier: MPL-2.0
// Waiting and scheduling costs of the pool:
//  - an isolated wait (inside a cache loader) on a pool whose bulk queue holds a backlog;
//  - the latency of an interactive request while a long bulk ParallelFor keeps the workers busy;
//  - what a completion costs the rest of the pool (wake-ups of threads that were not waiting
//    for it): process CPU time per completed flight / task group.
#include <benchmark/benchmark.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/resource.h>
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <latch>
#include <memory>
#include <thread>
#include <vector>

#include "core/cache/sharded_lru.h"
#include "core/exec/parallel_for.h"
#include "core/exec/task_group.h"
#include "core/exec/thread_pool.h"
#include "core/exec/topology.h"

namespace {

using namespace ddbt::exec;
using Clock = std::chrono::steady_clock;

void SpinFor(std::chrono::nanoseconds d) {
    const auto end = Clock::now() + d;
    while (Clock::now() < end) detail::CpuRelax();
}

double ProcessCpuSeconds() {
#ifdef _WIN32
    FILETIME created, exited, kernel, user;
    if (!GetProcessTimes(GetCurrentProcess(), &created, &exited, &kernel, &user)) return 0;
    auto seconds = [](const FILETIME& t) {  // 100 ns units
        return 1e-7 * static_cast<double>((static_cast<uint64_t>(t.dwHighDateTime) << 32) | t.dwLowDateTime);
    };
    return seconds(kernel) + seconds(user);
#else
    rusage u{};
    getrusage(RUSAGE_SELF, &u);
    return static_cast<double>(u.ru_utime.tv_sec + u.ru_stime.tv_sec) +
           1e-6 * static_cast<double>(u.ru_utime.tv_usec + u.ru_stime.tv_usec);
#endif
}

// ---- isolated wait with a bulk backlog ----
//
// The pool's only worker runs a driver task for the whole benchmark, so `backlog` bulk tasks stay
// queued. Per iteration the driver runs a loader (isolated) that waits for another key, which the
// benchmark thread loads and holds for 20 us. Time per iteration = 20 us + how fast the isolated
// waiter notices the completion.
void BM_Pool_IsolatedWait_BulkBacklog(benchmark::State& state) {
    const auto backlog = static_cast<size_t>(state.range(0));
    using Cache = ddbt::cache::ShardedLru<uint64_t, uint64_t>;
    Cache cache(size_t{1} << 24, 1);
    std::atomic<uint64_t> request{0}, served{0};
    std::atomic<bool> stop{false};
    {
        ThreadPoolOptions o;
        o.threads = 1;
        ThreadPool pool(o);
        std::latch running(1);
        pool.Submit(
            [&] {
                running.count_down();
                uint64_t r = 0;
                for (;;) {
                    uint64_t want;
                    while ((want = request.load(std::memory_order_acquire)) == r) {
                        if (stop.load(std::memory_order_acquire)) return;
                        detail::CpuRelax();
                    }
                    r = want;
                    cache.GetOrLoad(2 * r, [&] { return *cache.GetOrLoad(2 * r + 1, [] { return uint64_t{0}; }); });
                    served.store(r, std::memory_order_release);
                }
            },
            Priority::kInteractive);
        running.wait();
        for (size_t i = 0; i < backlog; ++i) pool.Submit([] {}, Priority::kBulk);
        uint64_t r = 0;
        for (auto _ : state) {
            ++r;
            cache.GetOrLoad(2 * r + 1, [&] {
                request.store(r, std::memory_order_release);
                while (cache.Stats().coalesced < r) detail::CpuRelax();  // the driver waits on this key
                SpinFor(std::chrono::microseconds(20));
                return r;
            });
            while (served.load(std::memory_order_acquire) != r) detail::CpuRelax();
        }
        stop.store(true, std::memory_order_release);
    }  // the destructor runs the backlog
}
BENCHMARK(BM_Pool_IsolatedWait_BulkBacklog)->Arg(0)->Arg(100000)->UseRealTime()->Unit(benchmark::kMicrosecond);

// ---- interactive latency under a long bulk ParallelFor ----
//
// `threads - 1` workers; a background thread keeps running bulk ParallelFors (chunks of 100 us,
// about 50 ms per call). Per iteration a tile-like interactive task is submitted: it spawns 4
// interactive subtasks of 20 us into a group and waits for them. Time per iteration = submit to
// completion as seen by the (blocked) benchmark thread; p50/p99/max = submit to the end of the
// task, measured inside it (without the benchmark thread's own wake-up, which on a fully busy
// CCD is up to a scheduler slice).
void BM_Pool_InteractiveLatency_UnderBulkParallelFor(benchmark::State& state) {
    const int threads = static_cast<int>(state.range(0));
    ThreadPoolOptions o;
    o.threads = static_cast<size_t>(threads - 1);
    o.zeroWorkers = threads == 1;
    ThreadPool pool(o);
    std::atomic<bool> stop{false};
    std::atomic<uint64_t> bulkChunks{0};
    std::thread bulk([&] {
        const size_t chunks = 500 * static_cast<size_t>(threads);
        while (!stop.load(std::memory_order_acquire)) {
            ParallelFor(
                pool, 0, chunks, 1,
                [&](size_t, size_t) {
                    SpinFor(std::chrono::microseconds(100));
                    bulkChunks.fetch_add(1, std::memory_order_relaxed);
                },
                nullptr, Priority::kBulk);
        }
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));  // bulk work under way
    std::vector<double> latencies;
    const uint64_t chunks0 = bulkChunks.load();
    const auto t0 = Clock::now();
    for (auto _ : state) {
        std::atomic<uint32_t> done{0};
        Clock::time_point served{};
        const auto start = Clock::now();
        pool.Submit(
            [&] {
                TaskGroup g(pool, Priority::kInteractive);
                for (int i = 0; i < 4; ++i) g.Spawn([] { SpinFor(std::chrono::microseconds(20)); });
                g.Wait();
                served = Clock::now();
                done.store(1, std::memory_order_release);
                done.notify_one();
            },
            Priority::kInteractive);
        done.wait(0, std::memory_order_acquire);
        latencies.push_back(std::chrono::duration<double, std::micro>(served - start).count());
    }
    const double seconds = std::chrono::duration<double>(Clock::now() - t0).count();
    const uint64_t chunks1 = bulkChunks.load();
    stop.store(true, std::memory_order_release);
    bulk.join();
    std::sort(latencies.begin(), latencies.end());
    if (!latencies.empty()) {
        state.counters["p50_us"] = latencies[latencies.size() / 2];
        state.counters["p99_us"] = latencies[std::min(latencies.size() - 1, latencies.size() * 99 / 100)];
        state.counters["max_us"] = latencies.back();
    }
    // Bulk throughput while the requests ran (chunks/s, 100 us each).
    state.counters["bulk_chunks_per_s"] = static_cast<double>(chunks1 - chunks0) / seconds;
}
BENCHMARK(BM_Pool_InteractiveLatency_UnderBulkParallelFor)
    ->Arg(2)
    ->Arg(4)
    ->Arg(8)
    ->Arg(16)
    ->UseRealTime()
    ->Unit(benchmark::kMicrosecond);

// ---- wake-up cost of completions ----
//
// `workers` idle (sleeping) workers. Per iteration one worker task waits for a flight that the
// benchmark thread completes after 1 ms (by then every thread sleeps). Reported: process CPU time
// per iteration (the waiter's work is ~nothing, so this is what the wake-ups cost) and the time
// from the end of the loader to the waiter's return.
void BM_Pool_FlightCompletionWake(benchmark::State& state) {
    ThreadPoolOptions o;
    o.threads = static_cast<size_t>(state.range(0));
    ThreadPool pool(o);
    using Cache = ddbt::cache::ShardedLru<uint64_t, uint64_t>;
    Cache cache(size_t{1} << 24, 1);
    uint64_t r = 0;
    double wakeUs = 0;
    const double cpu0 = ProcessCpuSeconds();
    for (auto _ : state) {
        ++r;
        std::atomic<uint32_t> done{0};
        Clock::time_point completed{}, resumed{};
        cache.GetOrLoad(r, [&] {
            pool.Submit([&] {
                cache.GetOrLoad(r, [] { return uint64_t{0}; });
                resumed = Clock::now();
                done.store(1, std::memory_order_release);
                done.notify_one();
            });
            while (cache.Stats().coalesced < r) std::this_thread::yield();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            completed = Clock::now();
            return r;
        });
        done.wait(0, std::memory_order_acquire);
        wakeUs += std::chrono::duration<double, std::micro>(resumed - completed).count();
    }
    const double cpu = ProcessCpuSeconds() - cpu0;
    state.counters["cpu_us_per_op"] = 1e6 * cpu / static_cast<double>(state.iterations());
    state.counters["wake_us"] = wakeUs / static_cast<double>(state.iterations());
}
BENCHMARK(BM_Pool_FlightCompletionWake)->Arg(4)->Arg(15)->UseRealTime()->Unit(benchmark::kMicrosecond);

// Same for a task group: a worker task spawns one subtask, lets another worker steal it, and
// waits; the subtask sleeps 1 ms (every other thread falls asleep) and finishes.
void BM_Pool_GroupCompletionWake(benchmark::State& state) {
    ThreadPoolOptions o;
    o.threads = static_cast<size_t>(state.range(0));
    ThreadPool pool(o);
    double wakeUs = 0;
    const double cpu0 = ProcessCpuSeconds();
    for (auto _ : state) {
        std::atomic<uint32_t> done{0};
        Clock::time_point completed{}, resumed{};
        pool.Submit([&] {
            std::atomic<bool> stolen{false};
            TaskGroup g(pool);
            g.Spawn([&] {
                stolen.store(true, std::memory_order_release);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                completed = Clock::now();
            });
            while (!stolen.load(std::memory_order_acquire)) std::this_thread::yield();
            g.Wait();
            resumed = Clock::now();
            done.store(1, std::memory_order_release);
            done.notify_one();
        });
        done.wait(0, std::memory_order_acquire);
        wakeUs += std::chrono::duration<double, std::micro>(resumed - completed).count();
    }
    const double cpu = ProcessCpuSeconds() - cpu0;
    state.counters["cpu_us_per_op"] = 1e6 * cpu / static_cast<double>(state.iterations());
    state.counters["wake_us"] = wakeUs / static_cast<double>(state.iterations());
}
BENCHMARK(BM_Pool_GroupCompletionWake)->Arg(4)->Arg(15)->UseRealTime()->Unit(benchmark::kMicrosecond);

}  // namespace
