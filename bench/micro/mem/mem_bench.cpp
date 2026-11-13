// SPDX-License-Identifier: MPL-2.0
// BufferPool vs malloc for decode / I/O sized buffers, and Arena vs malloc for small objects.
#include <benchmark/benchmark.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

#include "core/exec/topology.h"
#include "core/mem/aligned.h"
#include "core/mem/arena.h"
#include "core/mem/buffer_pool.h"

namespace {

using namespace ddbt::mem;

void SizesAndThreads(benchmark::Benchmark* b) {
    const int max = static_cast<int>(ddbt::exec::AllowedCpus().size());
    for (int64_t size : {int64_t{4} << 10, int64_t{64} << 10, int64_t{1} << 20, int64_t{8} << 20}) {
        b->Args({size});
    }
    b->Threads(1)->Threads(max);
}

// Acquire, touch the first and last cache line, release.
void BM_Mem_Malloc(benchmark::State& state) {
    const auto n = static_cast<size_t>(state.range(0));
    for (auto _ : state) {
        auto* p = static_cast<uint8_t*>(std::malloc(n));
        p[0] = 1;
        p[n - 1] = 2;
        benchmark::DoNotOptimize(p);
        std::free(p);
    }
    state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_Mem_Malloc)->Apply(SizesAndThreads)->UseRealTime();

void BM_Mem_AlignedAlloc64(benchmark::State& state) {
    const auto n = static_cast<size_t>(state.range(0));
    for (auto _ : state) {
        auto* p = static_cast<uint8_t*>(AlignedAlloc(n, 64));
        p[0] = 1;
        p[n - 1] = 2;
        benchmark::DoNotOptimize(p);
        AlignedFree(p);
    }
    state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_Mem_AlignedAlloc64)->Apply(SizesAndThreads)->UseRealTime();

BufferPool& SharedPool() {
    static BufferPool pool;
    return pool;
}

void BM_Mem_BufferPool(benchmark::State& state) {
    const auto n = static_cast<size_t>(state.range(0));
    BufferPool& pool = SharedPool();
    for (auto _ : state) {
        PooledBuffer b = pool.Acquire(n);
        b.data()[0] = 1;
        b.data()[n - 1] = 2;
        benchmark::DoNotOptimize(b.data());
    }
    state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_Mem_BufferPool)->Apply(SizesAndThreads)->UseRealTime();

// Decode-like pattern: 16 live buffers of random sizes (64 KiB .. 4 MiB); each iteration frees
// one and allocates a replacement that is fully written (one byte per 4 KiB page), so fresh
// pages from the OS cost their page faults.
template <typename Alloc, typename Free>
void MixedPattern(benchmark::State& state, Alloc alloc, Free free) {
    constexpr size_t kLive = 16;
    std::vector<std::pair<void*, size_t>> live(kLive, {nullptr, 0});
    uint64_t s = 0x9E3779B97F4A7C15ULL * (static_cast<uint64_t>(static_cast<unsigned>(state.thread_index())) + 1u);
    size_t i = 0;
    for (auto _ : state) {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        auto& slot = live[i++ % kLive];
        if (slot.first != nullptr) free(slot.first, slot.second);
        const size_t n = (size_t{64} << 10) + static_cast<size_t>(s % ((size_t{4} << 20) - (size_t{64} << 10)));
        auto* p = static_cast<uint8_t*>(alloc(n));
        for (size_t off = 0; off < n; off += 4096) p[off] = static_cast<uint8_t>(off);
        slot = {p, n};
    }
    for (auto& [p, n] : live) {
        if (p != nullptr) free(p, n);
    }
    state.SetItemsProcessed(state.iterations());
}

void BM_Mem_Mixed_Malloc(benchmark::State& state) {
    MixedPattern(state, [](size_t n) { return std::malloc(n); }, [](void* p, size_t) { std::free(p); });
}
BENCHMARK(BM_Mem_Mixed_Malloc)->Threads(1)->Threads(8)->UseRealTime();

void BM_Mem_Mixed_BufferPool(benchmark::State& state) {
    // Handles kept by pointer: the pattern frees in a different order than it allocates.
    BufferPool& pool = SharedPool();
    std::vector<std::unique_ptr<PooledBuffer>> owned;
    MixedPattern(
        state,
        [&](size_t n) {
            auto b = std::make_unique<PooledBuffer>(pool.Acquire(n));
            void* p = b->data();
            owned.push_back(std::move(b));
            return p;
        },
        [&](void* p, size_t) {
            for (auto& b : owned) {
                if (b && b->data() == p) {
                    b.reset();
                    break;
                }
            }
            std::erase(owned, nullptr);
        });
}
BENCHMARK(BM_Mem_Mixed_BufferPool)->Threads(1)->Threads(8)->UseRealTime();

// 1000 small objects (16..128 B) then free them all.
void BM_Mem_SmallObjects_Malloc(benchmark::State& state) {
    std::vector<void*> ptrs(1000);
    for (auto _ : state) {
        for (size_t i = 0; i < ptrs.size(); ++i) {
            ptrs[i] = std::malloc(16 + (i % 8) * 16);
            benchmark::DoNotOptimize(ptrs[i]);
        }
        for (void* p : ptrs) std::free(p);
    }
    state.SetItemsProcessed(state.iterations() * 1000);
}
BENCHMARK(BM_Mem_SmallObjects_Malloc);

void BM_Mem_SmallObjects_Arena(benchmark::State& state) {
    Arena arena;
    for (auto _ : state) {
        for (size_t i = 0; i < 1000; ++i) benchmark::DoNotOptimize(arena.Allocate(16 + (i % 8) * 16, 16));
        arena.Reset();
    }
    state.SetItemsProcessed(state.iterations() * 1000);
}
BENCHMARK(BM_Mem_SmallObjects_Arena);

}  // namespace
