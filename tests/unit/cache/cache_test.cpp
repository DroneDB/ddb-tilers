// SPDX-License-Identifier: MPL-2.0
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <latch>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <typeinfo>
#include <vector>

#include "core/cache/sharded_lru.h"
#include "core/exec/task_group.h"
#include "core/mem/mem_budget.h"
#include "../exec/test_watchdog.h"

using ddbt::cache::ShardedLru;
using ddbt::exec::TaskGroup;
using ddbt::exec::ThreadPool;
using ddbt::exec::ThreadPoolOptions;
using ddbt::test::Watchdog;
using namespace std::chrono_literals;

namespace {

using Bytes = std::vector<uint8_t>;

// Cost = payload size only, to make budgets easy to reason about.
struct PayloadCost {
    size_t operator()(const Bytes& b) const noexcept { return b.size(); }
};

using ByteCache = ShardedLru<int, Bytes, std::hash<int>, PayloadCost>;

ThreadPoolOptions Workers(size_t n) {
    ThreadPoolOptions o;
    o.threads = n;
    return o;
}

}  // namespace

TEST(ShardedLru, GetPutEraseClear) {
    ByteCache c(1000, 4);
    EXPECT_EQ(c.ShardCount(), 4u);
    EXPECT_EQ(c.Get(1), nullptr);
    auto v = c.Put(1, Bytes(10, 1));
    ASSERT_NE(v, nullptr);
    EXPECT_EQ(c.Get(1), v);
    EXPECT_TRUE(c.Contains(1));
    c.Put(1, Bytes(20, 2));  // replace
    EXPECT_EQ(c.Get(1)->size(), 20u);
    EXPECT_EQ(c.Stats().bytes, 20u);
    EXPECT_TRUE(c.Erase(1));
    EXPECT_FALSE(c.Erase(1));
    for (int i = 0; i < 10; ++i) c.Put(i, Bytes(5));
    EXPECT_EQ(c.Stats().entries, 10u);
    c.Clear();
    const auto s = c.Stats();
    EXPECT_EQ(s.entries, 0u);
    EXPECT_EQ(s.bytes, 0u);
    EXPECT_EQ(s.hits, 2u);
    EXPECT_EQ(s.misses, 1u);
    EXPECT_NE(ddbt::cache::ToJson(s).find("\"hits\":2"), std::string::npos);
    ShardedLru<int, int> rounded(100, 5);
    EXPECT_EQ(rounded.ShardCount(), 8u);
}

TEST(ShardedLru, LruEvictionByBytes) {
    ByteCache c(100, 1);  // one shard: exact LRU order
    for (int i = 0; i < 5; ++i) c.Put(i, Bytes(20));
    EXPECT_EQ(c.Stats().bytes, 100u);
    c.Get(0);                  // 0 becomes most recent; 1 is now the oldest
    c.Put(5, Bytes(30));       // needs 30: evicts 1 and 2
    EXPECT_TRUE(c.Contains(0));
    EXPECT_FALSE(c.Contains(1));
    EXPECT_FALSE(c.Contains(2));
    EXPECT_TRUE(c.Contains(3));
    EXPECT_TRUE(c.Contains(5));
    const auto s = c.Stats();
    EXPECT_EQ(s.evictions, 2u);
    EXPECT_LE(s.bytes, 100u);

    // Larger than the budget: returned, not cached, and does not flush the cache.
    auto big = c.Put(9, Bytes(101));
    EXPECT_EQ(big->size(), 101u);
    EXPECT_FALSE(c.Contains(9));
    EXPECT_TRUE(c.Contains(5));
    EXPECT_EQ(c.Stats().rejected, 1u);

    c.SetBudget(40);  // shrink: evicts down to 40 bytes
    EXPECT_LE(c.Stats().bytes, 40u);
    EXPECT_TRUE(c.Contains(5));
}

TEST(ShardedLru, BudgetHoldsAcrossShards) {
    ByteCache c(64 * 1000, 64);
    std::mt19937 rng(3);
    for (int i = 0; i < 20000; ++i) c.Put(static_cast<int>(rng() % 5000), Bytes(1 + rng() % 900));
    const auto s = c.Stats();
    EXPECT_LE(s.bytes, 64u * 1000u);
    EXPECT_GT(s.evictions, 0u);
}

// The budget is global: entries that fit it all stay cached, however the hash spreads them over
// the shards (with a budget split per shard, 16 KiB here, a shard given three of them evicted).
TEST(ShardedLru, EntriesThatFitTheBudgetStayWhateverTheirShards) {
    ByteCache c(64 * 16384, 64);
    for (int i = 0; i < 60; ++i) c.Put(i, Bytes(16384));
    for (int i = 0; i < 60; ++i) EXPECT_TRUE(c.Contains(i)) << i;
    const auto s = c.Stats();
    EXPECT_EQ(s.evictions, 0u);
    EXPECT_EQ(s.entries, 60u);
    EXPECT_EQ(s.bytes, 60u * 16384u);
}

// Over budget, the least recently used entry of all shards goes, wherever it is.
TEST(ShardedLru, EvictionIsLeastRecentlyUsedAcrossShards) {
    ByteCache c(10 * 100, 64);
    for (int i = 0; i < 10; ++i) c.Put(i, Bytes(100));
    ASSERT_NE(c.Get(0), nullptr);  // 1 is now the least recently used
    c.Put(10, Bytes(100));
    EXPECT_TRUE(c.Contains(0));
    EXPECT_FALSE(c.Contains(1));
    for (int i = 2; i <= 10; ++i) EXPECT_TRUE(c.Contains(i)) << i;
    c.Put(11, Bytes(250));  // evicts the next three oldest: 2, 3, 4
    EXPECT_FALSE(c.Contains(2));
    EXPECT_FALSE(c.Contains(3));
    EXPECT_FALSE(c.Contains(4));
    EXPECT_TRUE(c.Contains(5));
    EXPECT_TRUE(c.Contains(0));
    EXPECT_LE(c.Stats().bytes, 1000u);
    c.SetBudget(300);  // shrink: the oldest go first
    EXPECT_LE(c.Stats().bytes, 300u);
    EXPECT_TRUE(c.Contains(11));
}

// Concurrent inserts and hits on many shards end within the budget.
TEST(ShardedLru, ConcurrentInsertsEndWithinTheGlobalBudget) {
    ByteCache c(200 * 1000, 64);
    std::vector<std::thread> threads;
    for (int t = 0; t < 8; ++t) {
        threads.emplace_back([&c, t] {
            std::mt19937 rng(static_cast<unsigned>(t));
            for (int i = 0; i < 5000; ++i) {
                const int k = static_cast<int>(rng() % 2000);
                if (!c.Get(k)) c.Put(k, Bytes(1 + rng() % 1999));
            }
        });
    }
    for (auto& th : threads) th.join();
    const auto s = c.Stats();
    EXPECT_LE(s.bytes, 200u * 1000u);
    EXPECT_GT(s.evictions, 0u);
}

TEST(ShardedLru, EvictedEntryInUseSurvives) {
    ByteCache c(100, 1);
    auto held = c.Put(1, Bytes(80, 7));
    c.Put(2, Bytes(80, 8));  // evicts 1
    EXPECT_FALSE(c.Contains(1));
    ASSERT_EQ(held.use_count(), 1);
    EXPECT_EQ((*held)[79], 7);
}

TEST(ShardedLru, MemBudgetSharedBetweenCaches) {
    ddbt::mem::MemBudget budget(1000);
    ddbt::cache::ShardedLruOptions o;
    o.budgetBytes = 1000;
    o.shards = 1;
    o.memBudget = &budget;
    ByteCache a(o), b(o);
    a.Put(1, Bytes(600));
    EXPECT_EQ(budget.Current(), 600u);
    b.Put(1, Bytes(600));  // does not fit the shared budget and b has nothing to evict
    EXPECT_FALSE(b.Contains(1));
    EXPECT_EQ(b.Stats().rejected, 1u);
    b.Put(2, Bytes(300));
    EXPECT_EQ(budget.Current(), 900u);
    b.Put(3, Bytes(300));  // b evicts its own entry 2 to make room
    EXPECT_FALSE(b.Contains(2));
    EXPECT_TRUE(b.Contains(3));
    EXPECT_EQ(budget.Current(), 900u);
    a.Clear();
    b.Clear();
    EXPECT_EQ(budget.Current(), 0u);
}

// 32 threads hammer overlapping keys: every key is loaded exactly once.
TEST(ShardedLru, SingleFlight32Threads) {
    constexpr int kThreads = 32;
    constexpr int kKeys = 64;
    ByteCache c(size_t{64} << 20);
    std::vector<std::atomic<int>> loads(kKeys);
    std::latch start(kThreads);
    std::atomic<int> wrong{0};
    std::vector<std::thread> ts;
    for (int t = 0; t < kThreads; ++t) {
        ts.emplace_back([&, t] {
            std::mt19937 rng(static_cast<uint32_t>(t));
            start.arrive_and_wait();
            for (int i = 0; i < 400; ++i) {
                const int k = static_cast<int>(rng() % kKeys);
                auto v = c.GetOrLoad(k, [&, k] {
                    loads[static_cast<size_t>(k)].fetch_add(1);
                    std::this_thread::sleep_for(std::chrono::microseconds(200));
                    return Bytes(100, static_cast<uint8_t>(k));
                });
                if (!v || v->size() != 100 || (*v)[0] != static_cast<uint8_t>(k)) wrong.fetch_add(1);
            }
        });
    }
    for (auto& th : ts) th.join();
    EXPECT_EQ(wrong.load(), 0);
    for (int k = 0; k < kKeys; ++k) EXPECT_EQ(loads[static_cast<size_t>(k)].load(), 1) << "key " << k;
    const auto s = c.Stats();
    EXPECT_EQ(s.loads, static_cast<uint64_t>(kKeys));
    EXPECT_EQ(s.hits + s.misses, uint64_t{kThreads} * 400);
}

TEST(ShardedLru, LoaderExceptionReachesAllWaitersAndIsNotCached) {
    ByteCache c(1 << 20);
    std::latch entered(1);
    std::atomic<bool> release{false};
    std::atomic<int> threw{0};
    std::thread leader([&] {
        try {
            c.GetOrLoad(7, [&]() -> Bytes {
                entered.count_down();
                while (!release.load()) std::this_thread::yield();
                throw std::runtime_error("decode failed");
            });
        } catch (const std::runtime_error&) {
            threw.fetch_add(1);
        }
    });
    entered.wait();
    std::vector<std::thread> waiters;
    for (int i = 0; i < 8; ++i) {
        waiters.emplace_back([&] {
            try {
                c.GetOrLoad(7, []() -> Bytes {
                    ADD_FAILURE() << "waiter ran the loader";
                    return {};
                });
            } catch (const std::runtime_error& e) {
                if (std::string(e.what()) == "decode failed") threw.fetch_add(1);
            }
        });
    }
    while (c.Stats().coalesced < 8) std::this_thread::yield();
    release = true;
    leader.join();
    for (auto& w : waiters) w.join();
    EXPECT_EQ(threw.load(), 9);
    EXPECT_FALSE(c.Contains(7));
    EXPECT_EQ(c.Stats().loadFailures, 1u);
    // The next call loads again.
    auto v = c.GetOrLoad(7, [] { return Bytes(3, 1); });
    EXPECT_EQ(v->size(), 3u);
    EXPECT_TRUE(c.Contains(7));
}

// Each waiter gets its own exception object, of the loader's type and message.
struct CodedError : std::runtime_error, ddbt::cache::WaiterCopyable {
    CodedError(int c, const std::string& m) : std::runtime_error(m), code(c) {}
    int code;
    std::function<void()> MakeThrower() const override {
        return [c = code, m = std::string(what())] { throw CodedError(c, m); };
    }
};
struct CodedErrorNoOverride : CodedError {
    using CodedError::CodedError;
};

constexpr int kFlightWaiters = 8;

// A failed flight of key 7: a leader whose loader runs doThrow, kFlightWaiters waiters. Every
// thread runs `handle(call)`, where call() is its GetOrLoad.
template <typename Throw, typename Handle>
void RunFailedFlight(Throw doThrow, Handle handle) {
    ByteCache c(1 << 20);
    std::latch entered(1);
    std::atomic<bool> release{false};
    std::thread leader([&] {
        handle([&] {
            c.GetOrLoad(7, [&]() -> Bytes {
                entered.count_down();
                while (!release.load()) std::this_thread::yield();
                doThrow();
                return {};
            });
        });
    });
    entered.wait();
    std::vector<std::thread> ws;
    for (int i = 0; i < kFlightWaiters; ++i)
        ws.emplace_back([&] { handle([&] { c.GetOrLoad(7, []() -> Bytes { return {}; }); }); });
    while (c.Stats().coalesced < static_cast<uint64_t>(kFlightWaiters)) std::this_thread::yield();
    release = true;
    leader.join();
    for (auto& w : ws) w.join();
}

TEST(ShardedLru, WaitersGetTheirOwnExceptionCopies) {
    std::mutex m;
    std::vector<const void*> seen;
    int ok = 0;
    std::latch allCaught(kFlightWaiters + 1);
    RunFailedFlight([] { throw CodedError(42, "coded failure"); },
                    [&](auto call) {
                        try {
                            call();
                        } catch (const CodedError& e) {
                            {
                                std::lock_guard lock(m);
                                seen.push_back(&e);
                                if (typeid(e) == typeid(CodedError) && e.code == 42 &&
                                    std::string(e.what()) == "coded failure")
                                    ++ok;
                            }
                            // Every handler holds its object until all have caught theirs: distinct
                            // addresses are distinct live objects.
                            allCaught.arrive_and_wait();
                        }
                    });
    EXPECT_EQ(ok, kFlightWaiters + 1);
    std::sort(seen.begin(), seen.end());
    EXPECT_EQ(std::adjacent_find(seen.begin(), seen.end()), seen.end());

    // Standard types keep their type and message.
    int stdOk = 0;
    RunFailedFlight([] { throw std::out_of_range("past the end"); },
                    [&](auto call) {
                        try {
                            call();
                        } catch (const std::exception& e) {
                            std::lock_guard lock(m);
                            if (typeid(e) == typeid(std::out_of_range) && std::string(e.what()) == "past the end") ++stdOk;
                        }
                    });
    EXPECT_EQ(stdOk, kFlightWaiters + 1);

    // A subclass that does not override MakeThrower, or a type that is no std::exception, is
    // shared with the waiters: the type is kept.
    int subOk = 0, intOk = 0;
    RunFailedFlight([] { throw CodedErrorNoOverride(7, "sub"); },
                    [&](auto call) {
                        try {
                            call();
                        } catch (const CodedError& e) {
                            std::lock_guard lock(m);
                            if (typeid(e) == typeid(CodedErrorNoOverride) && e.code == 7) ++subOk;
                        }
                    });
    RunFailedFlight([] { throw 5; },
                    [&](auto call) {
                        try {
                            call();
                        } catch (int v) {
                            std::lock_guard lock(m);
                            if (v == 5) ++intOk;
                        }
                    });
    EXPECT_EQ(subOk, kFlightWaiters + 1);
    EXPECT_EQ(intOk, kFlightWaiters + 1);
}

// A pool worker waiting for a load keeps running other pool tasks: with a single worker,
// the independent tasks can only finish if the waiting worker runs them.
TEST(ShardedLru, WaitingWorkerHelpsThePool) {
    ThreadPool pool(Workers(1));
    ByteCache c(1 << 20);
    std::latch entered(1);
    std::atomic<bool> release{false};
    std::thread leader([&] {
        c.GetOrLoad(1, [&] {
            entered.count_down();
            while (!release.load()) std::this_thread::yield();
            return Bytes(10, 42);
        });
    });
    entered.wait();
    std::atomic<int> waiterResult{-1};
    pool.Submit([&] {
        auto v = c.GetOrLoad(1, [] { return Bytes(10, 99); });
        waiterResult = (*v)[0];
    });
    while (c.Stats().coalesced < 1) std::this_thread::yield();
    std::atomic<int> independent{0};
    for (int i = 0; i < 100; ++i) pool.Submit([&] { independent.fetch_add(1); });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (independent.load() < 100 && std::chrono::steady_clock::now() < deadline) std::this_thread::yield();
    EXPECT_EQ(independent.load(), 100);
    EXPECT_EQ(waiterResult.load(), -1);  // still waiting for the load
    release = true;
    leader.join();
    while (waiterResult.load() == -1) std::this_thread::yield();
    EXPECT_EQ(waiterResult.load(), 42);
    EXPECT_EQ(c.Stats().loads, 1u);
    EXPECT_GE(pool.Stats().helped, 100u);
}

// Loaders that spawn subtasks which read other keys (acyclic: key k depends on keys < k), from
// many nested task groups and client threads. Must finish (no deadlock) with correct values.
TEST(ShardedLru, NestedLoadsInTaskGroupsStress) {
    ThreadPool pool(Workers(6));
    ShardedLru<int, uint64_t> c(1 << 20, 8);
    std::function<uint64_t(int)> value = [&](int k) -> uint64_t {
        return *c.GetOrLoad(k, [&, k]() -> uint64_t {
            if (k < 2) return 1;
            uint64_t a = 0, b = 0;
            TaskGroup g(pool);
            g.Spawn([&] { a = value(k - 1); });
            g.Spawn([&] { b = value(k - 2); });
            g.Wait();
            return a + b;
        });
    };
    std::vector<std::thread> clients;
    std::atomic<int> wrong{0};
    for (int t = 0; t < 8; ++t) {
        clients.emplace_back([&, t] {
            for (int round = 0; round < 20; ++round) {
                TaskGroup g(pool);
                std::atomic<uint64_t> v40{0};
                for (int i = 0; i < 4; ++i) g.Spawn([&] { v40 = value(40); });
                g.Wait();
                if (v40.load() != 165580141u) wrong.fetch_add(1);  // fib(41)
                if (t == 0 && round % 5 == 4) c.Clear();
            }
        });
    }
    for (auto& th : clients) th.join();
    EXPECT_EQ(wrong.load(), 0);
}

TEST(ShardedLru, SelfDependencyIsReported) {
    ThreadPool pool(Workers(2));
    ShardedLru<int, int> c(1 << 20);
    EXPECT_THROW(c.GetOrLoad(1,
                             [&] {
                                 // Same thread, through a nested task group: detected, not a hang.
                                 int inner = 0;
                                 TaskGroup g(pool);
                                 g.Spawn([&] { inner = *c.GetOrLoad(1, [] { return 5; }); });
                                 g.Wait();
                                 return inner;
                             }),
                 std::logic_error);
    EXPECT_FALSE(c.Contains(1));
    EXPECT_EQ(*c.GetOrLoad(1, [] { return 6; }), 6);
}

// ---- review fixes ------------------------------------------------------------------------------

// F3: Erase detaches a load in flight: a later GetOrLoad loads again instead of returning the
// value of the load that started before the Erase, and the old load does not overwrite it.
TEST(ShardedLru, EraseDetachesInFlightLoad) {
    Watchdog dog(60s, "EraseDetachesInFlightLoad");
    ByteCache c(1 << 20);
    std::latch entered(1);
    std::atomic<bool> release{false};
    std::thread leader([&] {
        auto v = c.GetOrLoad(1, [&] {
            entered.count_down();
            while (!release.load()) std::this_thread::yield();
            return Bytes(1, 1);
        });
        EXPECT_EQ((*v)[0], 1);  // the loading caller still gets its own result
    });
    entered.wait();
    EXPECT_FALSE(c.Erase(1));  // nothing cached yet
    auto fresh = c.GetOrLoad(1, [] { return Bytes(1, 2); });  // a new load, not the stale one
    EXPECT_EQ((*fresh)[0], 2);
    release = true;
    leader.join();
    ASSERT_TRUE(c.Contains(1));
    EXPECT_EQ((*c.Get(1))[0], 2);
    EXPECT_EQ(c.Stats().loads, 2u);
}

TEST(ShardedLru, ClearDetachesInFlightLoads) {
    Watchdog dog(60s, "ClearDetachesInFlightLoads");
    ByteCache c(1 << 20);
    std::latch entered(1);
    std::atomic<bool> release{false};
    std::atomic<int> waiterValue{-1};
    std::thread leader([&] {
        c.GetOrLoad(5, [&] {
            entered.count_down();
            while (!release.load()) std::this_thread::yield();
            return Bytes(1, 1);
        });
    });
    entered.wait();
    // A caller that joined before Clear gets the old load's value.
    std::thread waiter([&] { waiterValue = (*c.GetOrLoad(5, [] { return Bytes(1, 9); }))[0]; });
    while (c.Stats().coalesced < 1) std::this_thread::yield();
    c.Clear();
    EXPECT_EQ((*c.GetOrLoad(5, [] { return Bytes(1, 3); }))[0], 3);
    release = true;
    leader.join();
    waiter.join();
    EXPECT_EQ(waiterValue.load(), 1);
    EXPECT_EQ((*c.Get(5))[0], 3);
}

// F4: a Put during a load wins over the (older) loaded value.
TEST(ShardedLru, PutDuringLoadIsNotOverwritten) {
    Watchdog dog(60s, "PutDuringLoadIsNotOverwritten");
    ByteCache c(1 << 20);
    std::latch entered(1);
    std::atomic<bool> release{false};
    std::thread leader([&] {
        auto v = c.GetOrLoad(4, [&] {
            entered.count_down();
            while (!release.load()) std::this_thread::yield();
            return Bytes(1, 1);
        });
        EXPECT_EQ((*v)[0], 1);
    });
    entered.wait();
    c.Put(4, Bytes(1, 5));
    EXPECT_EQ((*c.GetOrLoad(4, [] { return Bytes(1, 7); }))[0], 5);  // a hit, not the load
    release = true;
    leader.join();
    EXPECT_EQ((*c.Get(4))[0], 5);
    EXPECT_EQ(c.Stats().loads, 1u);
}

// A loader exception reaches waiters that help on a pool (parked or running other tasks),
// including one waiting inside another key's loader (an isolated wait).
TEST(ShardedLru, LoaderExceptionReachesHelpingWaiters) {
    Watchdog dog(60s, "LoaderExceptionReachesHelpingWaiters");
    ThreadPool pool(Workers(4));
    ByteCache c(1 << 20);
    std::latch entered(1);
    std::atomic<bool> release{false};
    std::atomic<int> threw{0}, finished{0};
    std::thread leader([&] {
        try {
            c.GetOrLoad(7, [&]() -> Bytes {
                entered.count_down();
                while (!release.load()) std::this_thread::yield();
                throw std::runtime_error("decode failed");
            });
        } catch (const std::runtime_error&) {
            threw.fetch_add(1);
        }
    });
    entered.wait();
    auto never = []() -> Bytes {
        ADD_FAILURE() << "a waiter ran the loader";
        return {};
    };
    for (int i = 0; i < 3; ++i) {
        pool.Submit([&] {
            try {
                c.GetOrLoad(7, never);
            } catch (const std::runtime_error& e) {
                if (std::string(e.what()) == "decode failed") threw.fetch_add(1);
            }
            finished.fetch_add(1);
        });
    }
    pool.Submit([&] {
        try {
            c.GetOrLoad(8, [&] { return *c.GetOrLoad(7, never); });
        } catch (const std::runtime_error& e) {
            if (std::string(e.what()) == "decode failed") threw.fetch_add(1);
        }
        finished.fetch_add(1);
    });
    while (c.Stats().coalesced < 4) std::this_thread::yield();
    std::this_thread::sleep_for(10ms);  // the waiters park
    release = true;
    leader.join();
    while (finished.load() < 4) std::this_thread::yield();
    EXPECT_EQ(threw.load(), 5);
    EXPECT_FALSE(c.Contains(7));
    EXPECT_FALSE(c.Contains(8));
    EXPECT_EQ(c.Stats().loadFailures, 2u);
    EXPECT_EQ(c.GetOrLoad(7, [] { return Bytes(2, 1); })->size(), 2u);
}

// F2: isolation belongs to the region. One worker; a loader submits a task S (untagged) that
// waits for another key, and runs it from its own isolated wait; meanwhile a task B that needs
// the loader's key is queued. S must wait isolated (it runs inside the loader's region): if it
// picked up B, B would wait for the loader suspended below it on the same stack (deadlock).
TEST(ShardedLru, UntaggedTaskRunByLoaderWaitStaysIsolated) {
    Watchdog dog(60s, "UntaggedTaskRunByLoaderWaitStaysIsolated");
    ThreadPool pool(Workers(1));
    ShardedLru<int, int> c(1 << 20);
    std::latch k2Entered(1);
    std::atomic<bool> releaseK2{false};
    std::thread k2Leader([&] {
        c.GetOrLoad(2, [&] {
            k2Entered.count_down();
            while (!releaseK2.load()) std::this_thread::yield();
            return 20;
        });
    });
    k2Entered.wait();
    std::atomic<bool> bQueued{false}, sWaiting{false}, sIsolated{false};
    std::atomic<int> aResult{-1}, bResult{-1};
    pool.Submit([&] {
        aResult = *c.GetOrLoad(1, [&] {
            while (!bQueued.load()) std::this_thread::yield();
            ddbt::exec::TaskGroup lg(pool);
            lg.Spawn([] {});
            pool.Submit([&] {  // S: newest in this worker's deque, so lg.Wait() runs it first
                sIsolated = ddbt::exec::detail::CurrentIsolation().node != nullptr;
                sWaiting = true;
                c.GetOrLoad(2, [] { return -2; });
            });
            lg.Wait();
            return 10;
        });
    });
    while (c.Stats().loads < 2) std::this_thread::yield();  // the loader of key 1 runs
    pool.Submit([&] { bResult = *c.GetOrLoad(1, [] { return -1; }); });
    bQueued = true;
    while (!sWaiting.load()) std::this_thread::yield();
    std::this_thread::sleep_for(20ms);  // S waits for key 2 (and, if not isolated, takes B)
    releaseK2 = true;
    k2Leader.join();
    while (aResult.load() == -1 || bResult.load() == -1) std::this_thread::yield();
    EXPECT_TRUE(sIsolated.load());
    EXPECT_EQ(aResult.load(), 10);
    EXPECT_EQ(bResult.load(), 10);
}

// F6: a completed flight wakes only the threads waiting for it, not the idle workers.
TEST(ShardedLru, FlightCompletionWakesOnlyItsWaiters) {
    Watchdog dog(60s, "FlightCompletionWakesOnlyItsWaiters");
    constexpr int kWorkers = 12;
    ThreadPoolOptions o = Workers(kWorkers);
    o.spinRounds = 4;
    ThreadPool pool(o);
    ShardedLru<int, int> c(1 << 20);
    uint64_t sleeps = 0;
    constexpr int kRounds = 10;
    for (int round = 0; round < kRounds; ++round) {
        std::this_thread::sleep_for(30ms);  // every worker blocked in its futex
        const uint64_t sleeps0 = pool.Stats().sleeps;
        std::atomic<bool> done{false};
        c.GetOrLoad(round, [&] {
            pool.Submit([&, round] {
                c.GetOrLoad(round, [] { return -1; });
                done = true;
            });
            while (c.Stats().coalesced < static_cast<uint64_t>(round) + 1) std::this_thread::yield();
            std::this_thread::sleep_for(3ms);  // the waiter parks, idle workers sleep
            return round;
        });
        while (!done.load()) std::this_thread::yield();
        std::this_thread::sleep_for(5ms);
        sleeps += pool.Stats().sleeps - sleeps0;
    }
    EXPECT_LE(sleeps, uint64_t{kRounds} * kWorkers / 2) << sleeps;
}
