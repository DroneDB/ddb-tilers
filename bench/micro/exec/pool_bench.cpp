// SPDX-License-Identifier: MPL-2.0
// ThreadPool microbenchmarks: spawn / steal overhead against a naive mutex + condvar queue,
// fork-join, and ParallelFor scaling. Thread counts go up to the CPUs in the affinity mask
// (run under `taskset` to restrict them).
#include <benchmark/benchmark.h>

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "core/exec/parallel_for.h"
#include "core/exec/task_group.h"
#include "core/exec/thread_pool.h"
#include "core/exec/topology.h"

namespace {

using namespace ddbt::exec;

int MaxThreads() { return static_cast<int>(AllowedCpus().size()); }

// Thread counts 1, 2, 4, 8, ... up to the affinity mask, plus the maximum itself.
void ThreadArgs(benchmark::Benchmark* b) {
    const int max = MaxThreads();
    for (int t = 1; t < max; t *= 2) b->Arg(t);
    b->Arg(max);
}

// Pool for "t threads in total": t - 1 workers plus the calling thread, which helps.
std::unique_ptr<ThreadPool> MakePool(int threads) {
    ThreadPoolOptions o;
    o.threads = static_cast<size_t>(threads - 1);
    o.zeroWorkers = threads == 1;
    return std::make_unique<ThreadPool>(o);
}

// ---- naive reference: one mutex-protected std::deque<std::function>, condvar wake-ups ----

class MutexQueuePool {
public:
    explicit MutexQueuePool(size_t threads) {
        for (size_t i = 0; i < threads; ++i) {
            threads_.emplace_back([this] {
                for (;;) {
                    std::function<void()> f;
                    {
                        std::unique_lock lock(mutex_);
                        cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
                        if (queue_.empty()) return;
                        f = std::move(queue_.front());
                        queue_.pop_front();
                    }
                    f();
                }
            });
        }
    }
    ~MutexQueuePool() {
        {
            std::lock_guard lock(mutex_);
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& t : threads_) t.join();
    }
    void Submit(std::function<void()> f) {
        {
            std::lock_guard lock(mutex_);
            queue_.push_back(std::move(f));
        }
        cv_.notify_one();
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::function<void()>> queue_;
    std::vector<std::thread> threads_;
    bool stop_ = false;
};

// Tiny task body (~20 ns) so that scheduling overhead dominates.
inline void TinyWork(uint64_t seed) {
    uint64_t x = seed;
    for (int i = 0; i < 8; ++i) x = x * 6364136223846793005ULL + 1442695040888963407ULL;
    benchmark::DoNotOptimize(x);
}

constexpr int kBatch = 10000;

// External thread spawns kBatch tiny tasks (injection queue) and waits (helping).
void BM_Pool_SpawnWait_External(benchmark::State& state) {
    auto pool = MakePool(static_cast<int>(state.range(0)));
    for (auto _ : state) {
        TaskGroup g(*pool);
        for (int i = 0; i < kBatch; ++i) g.Spawn([i] { TinyWork(static_cast<uint64_t>(i)); });
        g.Wait();
    }
    state.SetItemsProcessed(state.iterations() * kBatch);
}
BENCHMARK(BM_Pool_SpawnWait_External)->Apply(ThreadArgs)->UseRealTime()->Unit(benchmark::kMicrosecond);

// A worker spawns kBatch tasks into its own deque; the others steal them.
void BM_Pool_SpawnWait_FromWorker(benchmark::State& state) {
    auto pool = MakePool(static_cast<int>(state.range(0)) + 1);  // +1: the caller only waits
    for (auto _ : state) {
        TaskGroup outer(*pool);
        outer.Spawn([&] {
            TaskGroup g(*pool);
            for (int i = 0; i < kBatch; ++i) g.Spawn([i] { TinyWork(static_cast<uint64_t>(i)); });
            g.Wait();
        });
        outer.Wait();
    }
    state.SetItemsProcessed(state.iterations() * kBatch);
}
BENCHMARK(BM_Pool_SpawnWait_FromWorker)->Apply(ThreadArgs)->UseRealTime()->Unit(benchmark::kMicrosecond);

// Same workload on the naive pool (the caller blocks; `threads` workers).
void BM_MutexQueue_SpawnWait(benchmark::State& state) {
    MutexQueuePool pool(static_cast<size_t>(state.range(0)));
    for (auto _ : state) {
        std::atomic<int> left{kBatch};
        std::mutex m;
        std::condition_variable cv;
        for (int i = 0; i < kBatch; ++i) {
            pool.Submit([&, i] {
                TinyWork(static_cast<uint64_t>(i));
                if (left.fetch_sub(1, std::memory_order_acq_rel) == 1) {
                    std::lock_guard lock(m);
                    cv.notify_one();
                }
            });
        }
        std::unique_lock lock(m);
        cv.wait(lock, [&] { return left.load(std::memory_order_acquire) == 0; });
    }
    state.SetItemsProcessed(state.iterations() * kBatch);
}
BENCHMARK(BM_MutexQueue_SpawnWait)->Apply(ThreadArgs)->UseRealTime()->Unit(benchmark::kMicrosecond);

// Recursive fork-join (binary tree of 2^17 leaves): spawn, steal and nested helping waits.
uint64_t TreeSum(ThreadPool& pool, int depth) {
    if (depth == 0) {
        TinyWork(1);
        return 1;
    }
    uint64_t a = 0, b = 0;
    TaskGroup g(pool);
    g.Spawn([&] { a = TreeSum(pool, depth - 1); });
    b = TreeSum(pool, depth - 1);
    g.Wait();
    return a + b;
}

void BM_Pool_ForkJoinTree(benchmark::State& state) {
    auto pool = MakePool(static_cast<int>(state.range(0)));
    constexpr int kDepth = 17;
    for (auto _ : state) benchmark::DoNotOptimize(TreeSum(*pool, kDepth));
    state.SetItemsProcessed(state.iterations() * ((int64_t{1} << kDepth) - 1));  // spawned tasks
}
BENCHMARK(BM_Pool_ForkJoinTree)->Apply(ThreadArgs)->UseRealTime()->Unit(benchmark::kMillisecond);

// Same tree with its root on a worker (the calling thread only waits for it).
void BM_Pool_ForkJoinTree_InWorker(benchmark::State& state) {
    auto pool = MakePool(static_cast<int>(state.range(0)) + 1);
    constexpr int kDepth = 17;
    for (auto _ : state) {
        TaskGroup outer(*pool);
        outer.Spawn([&] { benchmark::DoNotOptimize(TreeSum(*pool, kDepth)); });
        outer.Wait();
    }
    state.SetItemsProcessed(state.iterations() * ((int64_t{1} << kDepth) - 1));
}
BENCHMARK(BM_Pool_ForkJoinTree_InWorker)->Apply(ThreadArgs)->UseRealTime()->Unit(benchmark::kMillisecond);

// ---- ParallelFor scaling ----

// Compute-bound: 64 dependent FMAs per element over 4 Mi floats.
void BM_ParallelFor_Compute(benchmark::State& state) {
    auto pool = MakePool(static_cast<int>(state.range(0)));
    static std::vector<float> data(size_t{4} << 20, 1.0f);
    for (auto _ : state) {
        ParallelFor(*pool, 0, data.size(), 16384, [&](size_t lo, size_t hi) {
            for (size_t i = lo; i < hi; ++i) {
                float x = data[i];
                for (int k = 0; k < 64; ++k) x = x * 0.999f + 0.001f;
                data[i] = x;
            }
        });
        benchmark::ClobberMemory();
    }
    state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(data.size()));
}
BENCHMARK(BM_ParallelFor_Compute)->Apply(ThreadArgs)->UseRealTime()->Unit(benchmark::kMillisecond);

// Memory-bound: y = a * x + y over 2 x 128 MiB.
void BM_ParallelFor_Stream(benchmark::State& state) {
    auto pool = MakePool(static_cast<int>(state.range(0)));
    static std::vector<float> x(size_t{32} << 20, 1.0f), y(size_t{32} << 20, 2.0f);
    for (auto _ : state) {
        ParallelFor(*pool, 0, x.size(), size_t{1} << 16, [&](size_t lo, size_t hi) {
            for (size_t i = lo; i < hi; ++i) y[i] = 1.0001f * x[i] + y[i];
        });
        benchmark::ClobberMemory();
    }
    state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(x.size() * sizeof(float) * 3));
}
BENCHMARK(BM_ParallelFor_Stream)->Apply(ThreadArgs)->UseRealTime()->Unit(benchmark::kMillisecond);

// Fine-grained: 1 Mi indexes, grain 256 (4096 chunks of ~1 us).
void BM_ParallelFor_FineGrain(benchmark::State& state) {
    auto pool = MakePool(static_cast<int>(state.range(0)));
    for (auto _ : state) {
        ParallelFor(*pool, 0, size_t{1} << 20, 256, [](size_t lo, size_t hi) {
            for (size_t i = lo; i < hi; ++i) TinyWork(i);
        });
    }
    state.SetItemsProcessed(state.iterations() * (int64_t{1} << 20));
}
BENCHMARK(BM_ParallelFor_FineGrain)->Apply(ThreadArgs)->UseRealTime()->Unit(benchmark::kMillisecond);

}  // namespace
