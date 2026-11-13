// SPDX-License-Identifier: MPL-2.0
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <random>
#include <set>
#include <thread>
#include <vector>

#include "core/mem/aligned.h"
#include "core/mem/arena.h"
#include "core/mem/buffer_pool.h"
#include "core/mem/mem_budget.h"

using namespace ddbt::mem;

TEST(Aligned, AllocAlignment) {
    for (size_t align : {size_t{8}, size_t{16}, size_t{64}, size_t{4096}}) {
        for (size_t size : {size_t{0}, size_t{1}, size_t{100}, size_t{1} << 20}) {
            void* p = AlignedAlloc(size, align);
            ASSERT_NE(p, nullptr);
            EXPECT_EQ(reinterpret_cast<uintptr_t>(p) % align, 0u);
            if (size) std::memset(p, 0xAB, size);
            AlignedFree(p);
        }
    }
    auto b = MakeAlignedBytes(1000);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(b.get()) % kCacheLine, 0u);
    EXPECT_THROW(AlignedAlloc(10, 3), std::bad_alloc);
}

TEST(Aligned, LargeAllocZeroedAndHuge) {
    for (bool huge : {false, true}) {
        const size_t n = (size_t{5} << 20) + 123;
        auto* p = static_cast<uint8_t*>(LargeAlloc(n, huge));
        ASSERT_NE(p, nullptr);
        EXPECT_EQ(reinterpret_cast<uintptr_t>(p) % PageSize(), 0u);
#ifdef __linux__
        if (huge) {
            EXPECT_EQ(reinterpret_cast<uintptr_t>(p) % kHugePageSize, 0u);  // the flag's alignment is Linux's
        }
#endif
        EXPECT_EQ(p[0], 0);
        EXPECT_EQ(p[n - 1], 0);
        std::memset(p, 1, n);
        LargeFree(p, n, huge);
    }
    EXPECT_EQ(AlignUp(65, 64), 128u);
}

TEST(MemBudget, ReserveReleasePeak) {
    MemBudget b(1000);
    EXPECT_TRUE(b.TryReserve(600));
    EXPECT_FALSE(b.TryReserve(500));
    EXPECT_TRUE(b.TryReserve(400));
    EXPECT_EQ(b.Current(), 1000u);
    EXPECT_EQ(b.Available(), 0u);
    EXPECT_FALSE(b.TryReserve(1));
    b.Release(700);
    EXPECT_EQ(b.Current(), 300u);
    EXPECT_EQ(b.Peak(), 1000u);
    b.ForceReserve(2000);
    EXPECT_EQ(b.Current(), 2300u);
    EXPECT_EQ(b.Peak(), 2300u);
    EXPECT_FALSE(b.TryReserve(1));
    b.Release(2300);
    b.ResetPeak();
    EXPECT_EQ(b.Peak(), 0u);
    EXPECT_FALSE(b.TryReserve(MemBudget::kUnlimited));
    MemBudget unlimited;
    EXPECT_TRUE(unlimited.TryReserve(uint64_t{1} << 62));
}

TEST(MemBudget, ParentIsCharged) {
    MemBudget parent(1000);
    MemBudget a(800, &parent);
    MemBudget c(800, &parent);
    EXPECT_TRUE(a.TryReserve(600));
    EXPECT_EQ(parent.Current(), 600u);
    EXPECT_FALSE(c.TryReserve(500));  // child has room, parent does not
    EXPECT_EQ(c.Current(), 0u);       // rolled back
    EXPECT_TRUE(c.TryReserve(400));
    a.Release(600);
    c.Release(400);
    EXPECT_EQ(parent.Current(), 0u);
    EXPECT_EQ(parent.Peak(), 1000u);
}

TEST(MemBudget, RaiiAndConcurrency) {
    MemBudget b(1 << 20);
    {
        auto r = BudgetReservation::Try(b, 1000);
        EXPECT_TRUE(r.ok());
        auto moved = std::move(r);
        EXPECT_FALSE(r.ok());
        EXPECT_EQ(b.Current(), 1000u);
        auto fail = BudgetReservation::Try(b, 1 << 21);
        EXPECT_FALSE(fail.ok());
    }
    EXPECT_EQ(b.Current(), 0u);

    MemBudget cap(64 * 100);
    std::atomic<uint64_t> granted{0};
    std::vector<std::thread> ts;
    for (int t = 0; t < 16; ++t) {
        ts.emplace_back([&] {
            for (int i = 0; i < 2000; ++i) {
                if (cap.TryReserve(64)) {
                    const uint64_t g = granted.fetch_add(1) + 1;
                    EXPECT_LE(g, 100u);
                    granted.fetch_sub(1);
                    cap.Release(64);
                }
            }
        });
    }
    for (auto& t : ts) t.join();
    EXPECT_EQ(cap.Current(), 0u);
    EXPECT_LE(cap.Peak(), 6400u);
}

TEST(Arena, BumpAlignResetRelease) {
    MemBudget budget;
    ArenaOptions o;
    o.chunkSize = 4096;
    o.budget = &budget;
    Arena a(o);
    std::set<uintptr_t> seen;
    for (int i = 0; i < 1000; ++i) {
        const size_t align = size_t{1} << (i % 7);
        void* p = a.Allocate(static_cast<size_t>(i % 100) + 1, align);
        EXPECT_EQ(reinterpret_cast<uintptr_t>(p) % align, 0u);
        std::memset(p, i & 0xFF, static_cast<size_t>(i % 100) + 1);
        EXPECT_TRUE(seen.insert(reinterpret_cast<uintptr_t>(p)).second);
    }
    EXPECT_GT(a.ChunkCount(), 1u);
    EXPECT_EQ(budget.Current(), a.BytesReserved());
    const size_t chunks = a.ChunkCount();
    void* big = a.Allocate(100000);  // dedicated chunk
    std::memset(big, 0, 100000);
    EXPECT_EQ(a.ChunkCount(), chunks + 1);
    void* overAligned = a.Allocate(10, 4096);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(overAligned) % 4096, 0u);

    a.Reset();
    EXPECT_EQ(a.BytesUsed(), 0u);
    EXPECT_EQ(a.ChunkCount(), chunks);  // regular chunks kept
    for (int i = 0; i < 1000; ++i) a.Allocate(static_cast<size_t>(i % 100) + 1, size_t{1} << (i % 7));
    EXPECT_EQ(a.ChunkCount(), chunks);  // same pattern fits in the reused chunks

    struct Pod {
        int a;
        double b;
    };
    Pod* pod = a.New<Pod>(Pod{3, 4.5});
    EXPECT_EQ(pod->a, 3);
    int* arr = a.AllocateArray<int>(10);
    arr[9] = 1;
    const uint8_t src[3] = {1, 2, 3};
    auto copy = a.Copy(src);
    EXPECT_EQ(copy[2], 3);

    Arena moved = std::move(a);
    EXPECT_EQ(a.ChunkCount(), 0u);
    moved.Release();
    EXPECT_EQ(moved.BytesReserved(), 0u);
    EXPECT_EQ(budget.Current(), 0u);
}

TEST(Arena, HugePageChunks) {
    ArenaOptions o;
    o.chunkSize = size_t{4} << 20;
    o.hugePages = true;
    Arena a(o);
    auto* p = static_cast<uint8_t*>(a.Allocate(1 << 20));
    std::memset(p, 7, 1 << 20);
    auto* q = static_cast<uint8_t*>(a.Allocate(size_t{5} << 20));  // dedicated, >= 2 MiB
    std::memset(q, 7, size_t{5} << 20);
#ifdef __linux__
    EXPECT_EQ(reinterpret_cast<uintptr_t>(q) % kHugePageSize, 0u);  // the flag's alignment is Linux's
#endif
    EXPECT_EQ(a.BytesReserved(), size_t{9} << 20);
}

TEST(BufferPool, SizeClasses) {
    EXPECT_EQ(BufferPool::ClassOf(0), 0u);
    EXPECT_EQ(BufferPool::ClassOf(256), 0u);
    EXPECT_EQ(BufferPool::ClassSize(0), 256u);
    EXPECT_EQ(BufferPool::ClassOf(BufferPool::kMaxPooledSize), BufferPool::kClassCount - 1);
    EXPECT_EQ(BufferPool::ClassSize(BufferPool::kClassCount - 1), BufferPool::kMaxPooledSize);
    EXPECT_EQ(BufferPool::ClassOf(BufferPool::kMaxPooledSize + 1), BufferPool::kDirectClass);
    size_t prev = 0;
    for (uint32_t c = 0; c < BufferPool::kClassCount; ++c) {
        const size_t s = BufferPool::ClassSize(c);
        EXPECT_GT(s, prev);
        EXPECT_EQ(s % 64, 0u);
        EXPECT_EQ(BufferPool::ClassOf(s), c);
        EXPECT_EQ(BufferPool::ClassOf(prev + 1), c);
        if (c > 0) {
            EXPECT_LE(s - prev, s / 4);  // at most 25% slack
        }
        prev = s;
    }
}

TEST(BufferPool, RecyclesAndAligns) {
    MemBudget budget;
    BufferPoolOptions o;
    o.budget = &budget;
    BufferPool pool(o);
    uint8_t* first = nullptr;
    {
        PooledBuffer b = pool.Acquire(1000);
        ASSERT_TRUE(b);
        EXPECT_EQ(b.size(), 1000u);
        EXPECT_GE(b.capacity(), 1000u);
        EXPECT_EQ(reinterpret_cast<uintptr_t>(b.data()) % 64, 0u);
        std::memset(b.data(), 1, b.capacity());
        first = b.data();
        EXPECT_TRUE(b.Resize(b.capacity()));
        EXPECT_FALSE(b.Resize(b.capacity() + 1));
    }
    PooledBuffer again = pool.Acquire(1000);
    EXPECT_EQ(again.data(), first);  // same thread: served from its cache
    auto s = pool.Stats();
    EXPECT_EQ(s.osAllocations, 1u);
    EXPECT_EQ(s.threadHits, 1u);
    EXPECT_EQ(budget.Current(), s.bytesFromOs);

    PooledBuffer direct = pool.Acquire(BufferPool::kMaxPooledSize + 100);
    EXPECT_EQ(direct.capacity(), BufferPool::kMaxPooledSize + 100);
    direct.Reset();
    EXPECT_FALSE(direct);
    PooledBuffer moved = std::move(again);
    EXPECT_FALSE(again);
    moved.Reset();
    pool.Trim();
    EXPECT_EQ(pool.Stats().bytesFromOs, 0u);
    EXPECT_EQ(budget.Current(), 0u);
}

TEST(BufferPool, SpillsToSharedAndCapsCache) {
    BufferPoolOptions o;
    o.maxCachedBytes = size_t{8} << 20;
    BufferPool pool(o);
    {
        std::vector<PooledBuffer> held;
        for (int i = 0; i < 64; ++i) held.push_back(pool.Acquire(64 << 10));
        held.clear();  // 16 stay in the slot (1 MiB / 64 KiB), the rest spill to shared
    }
    auto s = pool.Stats();
    EXPECT_EQ(s.osAllocations, 64u);
    EXPECT_GT(s.bytesCachedShared, 0u);
    {
        std::vector<PooledBuffer> held;
        for (int i = 0; i < 64; ++i) held.push_back(pool.Acquire(64 << 10));
    }
    EXPECT_EQ(pool.Stats().osAllocations, 64u);  // all recycled

    // Class above 8 MiB: shared list only, capped at maxCachedBytes.
    {
        std::vector<PooledBuffer> held;
        for (int i = 0; i < 6; ++i) held.push_back(pool.Acquire(size_t{16} << 20));
    }
    s = pool.Stats();
    EXPECT_LE(s.bytesCachedShared, o.maxCachedBytes);
    EXPECT_GT(s.osFrees, 0u);
}

TEST(BufferPool, ConcurrentAcquireReleaseAcrossThreads) {
    BufferPool pool;
    constexpr int kThreads = 16;
    std::vector<std::thread> ts;
    std::atomic<int> errors{0};
    // Buffers are acquired on one thread and released on another (through a shared vector).
    std::mutex m;
    std::vector<PooledBuffer> handoff;
    for (int t = 0; t < kThreads; ++t) {
        ts.emplace_back([&, t] {
            std::mt19937 rng(static_cast<uint32_t>(t));
            for (int i = 0; i < 3000; ++i) {
                const size_t n = 1 + rng() % (256 << 10);
                PooledBuffer b = pool.Acquire(n);
                const auto tag = static_cast<uint8_t>(t * 31 + i);
                b.data()[0] = tag;
                b.data()[n - 1] = tag;
                if (b.data()[0] != tag || b.data()[n - 1] != tag) errors.fetch_add(1);
                if (i % 3 == 0) {
                    std::lock_guard lock(m);
                    handoff.push_back(std::move(b));
                    if (handoff.size() > 32) handoff.erase(handoff.begin(), handoff.begin() + 16);
                }
            }
        });
    }
    for (auto& t : ts) t.join();
    handoff.clear();
    EXPECT_EQ(errors.load(), 0);
    const auto s = pool.Stats();
    EXPECT_EQ(s.acquires, uint64_t{kThreads} * 3000);
    EXPECT_LT(s.osAllocations, s.acquires);
}
