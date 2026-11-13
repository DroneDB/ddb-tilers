// SPDX-License-Identifier: MPL-2.0
// ShardedLru microbenchmarks: hit path contention vs shard count, and mixed hit/miss
// GetOrLoad with evictions. Thread counts go up to the CPUs in the affinity mask.
#include <benchmark/benchmark.h>

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "core/cache/sharded_lru.h"
#include "core/exec/topology.h"

namespace {

using Payload = std::vector<uint8_t>;
using Cache = ddbt::cache::ShardedLru<uint64_t, Payload>;

constexpr uint64_t kHotKeys = 100000;

void ThreadCounts(benchmark::Benchmark* b) {
    const int max = static_cast<int>(ddbt::exec::AllowedCpus().size());
    for (int t = 1; t < max; t *= 2) b->Threads(t);
    b->Threads(max);
}

// One pre-filled cache per shard count, shared by all benchmark threads.
Cache& HotCache(size_t shards) {
    static std::mutex m;
    static std::map<size_t, std::unique_ptr<Cache>> caches;
    std::lock_guard lock(m);
    auto& c = caches[shards];
    if (!c) {
        c = std::make_unique<Cache>(size_t{1} << 30, shards);
        for (uint64_t k = 0; k < kHotKeys; ++k) c->Put(k, Payload(256, static_cast<uint8_t>(k)));
    }
    return *c;
}

inline uint64_t NextKey(uint64_t& s) {
    s ^= s << 13;
    s ^= s >> 7;
    s ^= s << 17;
    return s;
}

// All lookups hit (100k keys, uniform): lock + hash lookup + LRU move + shared_ptr copy.
void BM_Cache_Hit(benchmark::State& state) {
    Cache& c = HotCache(static_cast<size_t>(state.range(0)));
    uint64_t s = 0x9E3779B97F4A7C15ULL * (static_cast<uint64_t>(static_cast<unsigned>(state.thread_index())) + 1u);
    for (auto _ : state) {
        auto v = c.Get(NextKey(s) % kHotKeys);
        benchmark::DoNotOptimize(v.get());
    }
    state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_Cache_Hit)->Arg(1)->Arg(64)->Apply(ThreadCounts)->UseRealTime();

// GetOrLoad over 1M keys with room for ~25% of them: ~75% misses, a 1 KiB value built per
// miss, evictions on every insert once full.
void BM_Cache_GetOrLoadMixed(benchmark::State& state) {
    static std::unique_ptr<Cache> c;
    if (state.thread_index() == 0) c = std::make_unique<Cache>(size_t{256} << 20, 64);  // 256k x 1 KiB
    uint64_t s = 0x2545F4914F6CDD1DULL * (static_cast<uint64_t>(static_cast<unsigned>(state.thread_index())) + 1u);
    for (auto _ : state) {
        const uint64_t k = NextKey(s) % (uint64_t{1} << 20);
        auto v = c->GetOrLoad(k, [k] { return Payload(1024 - sizeof(Payload), static_cast<uint8_t>(k)); });
        benchmark::DoNotOptimize(v.get());
    }
    state.SetItemsProcessed(state.iterations());
    if (state.thread_index() == 0) {
        const auto st = c->Stats();
        state.counters["hit_rate"] = st.HitRate();
    }
}
BENCHMARK(BM_Cache_GetOrLoadMixed)->Apply(ThreadCounts)->UseRealTime();

}  // namespace
