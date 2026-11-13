// SPDX-License-Identifier: MPL-2.0
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <latch>
#include <mutex>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <thread>
#include <vector>

#include "core/exec/cancel_token.h"
#include "core/exec/chase_lev_deque.h"
#include "core/exec/parallel_for.h"
#include "core/exec/task_group.h"
#include "core/exec/thread_pool.h"
#include "core/exec/topology.h"
#include "test_watchdog.h"

using namespace ddbt::exec;
using ddbt::test::Watchdog;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {

ThreadPoolOptions Workers(size_t n) {
    ThreadPoolOptions o;
    o.threads = n;
    o.zeroWorkers = n == 0;
    return o;
}

// Recursive fork-join: every level waits on its own nested group.
uint64_t Fib(ThreadPool& pool, int n) {
    if (n < 12) {
        uint64_t a = 0, b = 1;
        for (int i = 0; i < n; ++i) {
            const uint64_t c = a + b;
            a = b;
            b = c;
        }
        return a;
    }
    uint64_t x = 0, y = 0;
    TaskGroup g(pool);
    g.Spawn([&] { x = Fib(pool, n - 1); });
    g.Spawn([&] { y = Fib(pool, n - 2); });
    g.Wait();
    return x + y;
}

constexpr uint64_t kFib25 = 75025;

}  // namespace

TEST(ChaseLevDeque, OwnerLifoAndGrowth) {
    detail::ChaseLevDeque<int> d(16);
    std::vector<int> items(100);
    for (int i = 0; i < 100; ++i) {
        items[static_cast<size_t>(i)] = i;
        d.Push(&items[static_cast<size_t>(i)]);
    }
    EXPECT_EQ(d.SizeApprox(), 100);
    EXPECT_EQ(*d.Steal(), 0);  // FIFO from the top
    for (int i = 99; i >= 1; --i) EXPECT_EQ(*d.Pop(), i);
    EXPECT_EQ(d.Pop(), nullptr);
    EXPECT_EQ(d.Steal(), nullptr);
    EXPECT_TRUE(d.Empty());
}

TEST(ChaseLevDeque, ConcurrentStealEachItemOnce) {
    constexpr int kItems = 50000;
    constexpr int kThieves = 4;
    detail::ChaseLevDeque<int> d(16);  // small: exercises growth under contention
    std::vector<int> items(kItems);
    std::vector<std::atomic<int>> seen(kItems);
    std::atomic<bool> done{false};
    std::atomic<int> taken{0};
    std::vector<std::thread> thieves;
    for (int t = 0; t < kThieves; ++t) {
        thieves.emplace_back([&] {
            while (!done.load(std::memory_order_acquire) || !d.Empty()) {
                if (int* p = d.Steal()) {
                    seen[static_cast<size_t>(*p)].fetch_add(1);
                    taken.fetch_add(1);
                } else {
                    std::this_thread::yield();
                }
            }
        });
    }
    std::mt19937 rng(1);
    for (int i = 0; i < kItems; ++i) {
        items[static_cast<size_t>(i)] = i;
        d.Push(&items[static_cast<size_t>(i)]);
        if (rng() % 3 == 0) {
            if (int* p = d.Pop()) {
                seen[static_cast<size_t>(*p)].fetch_add(1);
                taken.fetch_add(1);
            }
        }
    }
    while (int* p = d.Pop()) {
        seen[static_cast<size_t>(*p)].fetch_add(1);
        taken.fetch_add(1);
    }
    done.store(true, std::memory_order_release);
    for (auto& t : thieves) t.join();
    EXPECT_EQ(taken.load(), kItems);
    for (int i = 0; i < kItems; ++i) ASSERT_EQ(seen[static_cast<size_t>(i)].load(), 1) << i;
}

TEST(ThreadPool, SubmitRunsEverything) {
    std::atomic<int> count{0};
    {
        ThreadPool pool(Workers(4));
        EXPECT_EQ(pool.Size(), 4u);
        for (int i = 0; i < 10000; ++i) pool.Submit([&] { count.fetch_add(1, std::memory_order_relaxed); });
    }  // the destructor drains the queues
    EXPECT_EQ(count.load(), 10000);
}

TEST(ThreadPool, CurrentAndWorkerIndex) {
    ThreadPool pool(Workers(3));
    EXPECT_EQ(ThreadPool::Current(), nullptr);
    EXPECT_EQ(ThreadPool::CurrentWorkerIndex(), -1);
    std::mutex m;
    std::set<int> indexes;
    std::atomic<bool> wrongPool{false};
    TaskGroup g(pool);
    for (int i = 0; i < 300; ++i) {
        g.Spawn([&] {
            if (ThreadPool::Current() != &pool && ThreadPool::Current() != nullptr) wrongPool = true;
            std::lock_guard lock(m);
            indexes.insert(ThreadPool::CurrentWorkerIndex());
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        });
    }
    g.Wait();
    EXPECT_FALSE(wrongPool.load());
    for (int idx : indexes) EXPECT_TRUE(idx >= -1 && idx < 3);  // -1: run by the waiting caller
}

TEST(TaskGroup, NestedGroupsDoNotDeadlock) {
    for (size_t workers : {size_t{0}, size_t{1}, size_t{2}, size_t{8}}) {
        ThreadPool pool(Workers(workers));
        EXPECT_EQ(Fib(pool, 25), kFib25) << workers << " workers";
    }
}

TEST(TaskGroup, ExceptionPropagatesAndCancels) {
    ThreadPool pool(Workers(4));
    std::atomic<int> ran{0};
    TaskGroup g(pool);
    g.Spawn([] { throw std::runtime_error("boom"); });
    for (int i = 0; i < 2000; ++i) {
        g.Spawn([&] {
            std::this_thread::sleep_for(std::chrono::microseconds(20));
            ran.fetch_add(1);
        });
    }
    EXPECT_THROW(
        {
            try {
                g.Wait();
            } catch (const std::runtime_error& e) {
                EXPECT_STREQ(e.what(), "boom");
                throw;
            }
        },
        std::runtime_error);
    EXPECT_TRUE(g.IsCancelled());
    EXPECT_LT(ran.load(), 2000);  // the rest were skipped
    g.Wait();                     // nothing left, no second throw
}

TEST(TaskGroup, CancelSkipsUnstartedTasks) {
    ThreadPool pool(Workers(2));
    std::latch started(1);
    std::atomic<bool> release{false};
    std::atomic<int> ran{0};
    TaskGroup g(pool);
    g.Spawn([&] {
        started.count_down();
        while (!release.load()) std::this_thread::yield();
    });
    started.wait();
    for (int i = 0; i < 1000; ++i) g.Spawn([&] { ran.fetch_add(1); });
    g.Cancel();
    release = true;
    g.Wait();
    EXPECT_LE(ran.load(), 1000);
    EXPECT_TRUE(g.IsCancelled());
}

TEST(TaskGroup, ParentTokenCancelsChildren) {
    ThreadPool pool(Workers(2));
    CancelToken request;
    TaskGroup g(pool, Priority::kInteractive, &request);
    CancelToken child = g.Token().Child();
    EXPECT_FALSE(child.IsCancelled());
    request.Cancel();
    EXPECT_TRUE(g.IsCancelled());
    EXPECT_TRUE(child.IsCancelled());
    std::atomic<int> ran{0};
    for (int i = 0; i < 100; ++i) g.Spawn([&] { ran.fetch_add(1); });
    g.Wait();
    EXPECT_EQ(ran.load(), 0);

    CancelToken a;
    CancelToken copy = a;
    copy.Cancel();
    EXPECT_TRUE(a.IsCancelled());
    EXPECT_TRUE(a == copy);
}

TEST(TaskGroup, InteractiveQueuePreferredOverBulk) {
    ThreadPool pool(Workers(1));
    std::latch blocked(1);
    std::atomic<bool> release{false};
    pool.Submit([&] {
        blocked.count_down();
        while (!release.load()) std::this_thread::yield();
    });
    blocked.wait();
    std::mutex m;
    std::vector<char> order;
    // Fire-and-forget from this (external) thread: the worker is busy, so both queues fill.
    std::atomic<int> left{40};
    for (int i = 0; i < 20; ++i) {
        pool.Submit([&] {
            std::lock_guard lock(m);
            order.push_back('b');
            left.fetch_sub(1);
        }, Priority::kBulk);
    }
    for (int i = 0; i < 20; ++i) {
        pool.Submit([&] {
            std::lock_guard lock(m);
            order.push_back('i');
            left.fetch_sub(1);
        }, Priority::kInteractive);
    }
    release = true;
    while (left.load() != 0) std::this_thread::yield();
    ASSERT_EQ(order.size(), 40u);
    for (size_t k = 0; k < 20; ++k) EXPECT_EQ(order[k], 'i') << k;
}

TEST(TaskGroup, WaitingExternalThreadsHelp) {
    // No workers at all: the waiting thread has to run every task, including nested ones.
    ThreadPool pool(Workers(0));
    std::atomic<int> n{0};
    TaskGroup g(pool);
    for (int i = 0; i < 100; ++i) {
        g.Spawn([&] {
            TaskGroup inner(pool);
            for (int k = 0; k < 10; ++k) inner.Spawn([&] { n.fetch_add(1); });
            inner.Wait();
        });
    }
    g.Wait();
    EXPECT_EQ(n.load(), 1000);
    EXPECT_GE(pool.Stats().helped, 1100u);
}

// TSan stress: many client threads, nested groups, cancellation and exceptions at once.
TEST(TaskGroup, ConcurrentClientsStress) {
    ThreadPool pool(Workers(8));
    constexpr int kClients = 12;
    std::vector<std::thread> clients;
    std::atomic<int> failures{0};
    for (int c = 0; c < kClients; ++c) {
        clients.emplace_back([&, c] {
            std::mt19937 rng(static_cast<uint32_t>(c));
            for (int round = 0; round < 30; ++round) {
                const int mode = static_cast<int>(rng() % 3);
                CancelToken token;
                TaskGroup g(pool, c % 2 ? Priority::kInteractive : Priority::kBulk, &token);
                std::atomic<int> sum{0};
                for (int i = 0; i < 20; ++i) {
                    g.Spawn([&, i] {
                        TaskGroup inner(pool, Priority::kBulk, &g.Token());
                        for (int k = 0; k < 5; ++k) inner.Spawn([&] { sum.fetch_add(1, std::memory_order_relaxed); });
                        if (mode == 2 && i == 7) throw std::logic_error("x");
                        inner.Wait();
                    });
                }
                if (mode == 1) token.Cancel();
                try {
                    g.Wait();
                    if (mode == 0 && sum.load() != 100) failures.fetch_add(1);
                    if (mode == 2) failures.fetch_add(1);  // must have thrown
                } catch (const std::logic_error&) {
                    if (mode != 2) failures.fetch_add(1);
                }
            }
        });
    }
    for (auto& t : clients) t.join();
    EXPECT_EQ(failures.load(), 0);
}

TEST(ParallelFor, CoversRangeExactlyOnce) {
    ThreadPool pool(Workers(6));
    for (size_t grain : {size_t{1}, size_t{7}, size_t{1000}, size_t{100000}}) {
        std::vector<std::atomic<int>> hits(10007);
        ParallelFor(pool, 0, hits.size(), grain, [&](size_t i) { hits[i].fetch_add(1, std::memory_order_relaxed); });
        for (size_t i = 0; i < hits.size(); ++i) ASSERT_EQ(hits[i].load(), 1) << "grain " << grain << " i " << i;
    }
    std::atomic<uint64_t> sum{0};
    ParallelFor(pool, 10, 1010, 64, [&](size_t lo, size_t hi) {
        uint64_t s = 0;
        for (size_t i = lo; i < hi; ++i) s += i;
        sum.fetch_add(s);
    });
    EXPECT_EQ(sum.load(), (10 + 1009) * 1000 / 2);
    ParallelFor(pool, 5, 5, 1, [&](size_t) { FAIL(); });  // empty range
}

TEST(ParallelFor, NestedAndOnDefaultPool) {
    ThreadPool pool(Workers(4));
    std::atomic<int> n{0};
    ParallelFor(pool, 0, 16, 1, [&](size_t) { ParallelFor(pool, 0, 100, 3, [&](size_t) { n.fetch_add(1); }); });
    EXPECT_EQ(n.load(), 1600);
    std::atomic<int> m{0};
    ParallelFor(0, 1000, 10, [&](size_t) { m.fetch_add(1); });
    EXPECT_EQ(m.load(), 1000);
}

TEST(ParallelFor, ExceptionAndCancel) {
    ThreadPool pool(Workers(4));
    std::atomic<int> after{0};
    EXPECT_THROW(ParallelFor(pool, 0, 100000, 10,
                             [&](size_t i) {
                                 if (i == 50) throw std::out_of_range("bad index");
                                 after.fetch_add(1, std::memory_order_relaxed);
                             }),
                 std::out_of_range);
    EXPECT_LT(after.load(), 100000 - 1);

    CancelToken token;
    std::atomic<int> done{0};
    ParallelFor(pool, 0, 1000000, 100,
                [&](size_t lo, size_t) {
                    if (lo >= 1000) token.Cancel();
                    done.fetch_add(1);
                },
                &token);
    EXPECT_TRUE(token.IsCancelled());
    EXPECT_LT(done.load(), 10000);
}

TEST(Topology, CpuListParsing) {
    std::vector<int> v;
    ASSERT_TRUE(ParseCpuList("0-3,8,10-11\n", &v));
    EXPECT_EQ(v, (std::vector<int>{0, 1, 2, 3, 8, 10, 11}));
    EXPECT_EQ(FormatCpuList(v), "0-3,8,10-11");
    ASSERT_TRUE(ParseCpuList("", &v));
    EXPECT_TRUE(v.empty());
    EXPECT_FALSE(ParseCpuList("3-1", &v));
    EXPECT_FALSE(ParseCpuList("a", &v));
    EXPECT_FALSE(ParseCpuList("1,,2", &v));
    EXPECT_EQ(FormatCpuList({5, 1, 2, 3, 7}), "1-3,5,7");
}

TEST(Topology, SysfsTwoCcds) {
    // Fake sysfs of a 2-CCD, 8-thread part (CCD0: 0,1,4,5 / CCD1: 2,3,6,7).
    const fs::path root = fs::path(testing::TempDir()) / ("ddbt_topo_" + std::to_string(std::random_device{}()));
    for (int cpu = 0; cpu < 8; ++cpu) {
        const bool ccd0 = cpu == 0 || cpu == 1 || cpu == 4 || cpu == 5;
        for (int idx = 0; idx < 4; ++idx) {
            const fs::path dir = root / ("cpu" + std::to_string(cpu)) / "cache" / ("index" + std::to_string(idx));
            fs::create_directories(dir);
            std::ofstream(dir / "level") << (idx < 2 ? 1 : idx) << "\n";
            std::ofstream(dir / "shared_cpu_list") << (idx == 3 ? (ccd0 ? "0-1,4-5" : "2-3,6-7") : std::to_string(cpu)) << "\n";
            std::ofstream(dir / "size") << (idx == 3 ? "32768K" : "48K") << "\n";
        }
    }
    const Topology t = Topology::FromSysfs(root.string(), {0, 1, 2, 3, 4, 5, 6, 7});
    ASSERT_EQ(t.l3.size(), 2u);
    EXPECT_EQ(t.l3[0].cpus, (std::vector<int>{0, 1, 4, 5}));
    EXPECT_EQ(t.l3[1].cpus, (std::vector<int>{2, 3, 6, 7}));
    EXPECT_EQ(t.l3[0].cacheBytes, uint64_t{32} << 20);
    EXPECT_EQ(t.DomainOf(6), 1);
    EXPECT_EQ(t.DomainOf(9), -1);
    // Affinity restricted to a subset: domains are intersected.
    const Topology sub = Topology::FromSysfs(root.string(), {1, 2});
    ASSERT_EQ(sub.l3.size(), 2u);
    EXPECT_EQ(sub.l3[0].cpus, (std::vector<int>{1}));
    // No cache info: one catch-all domain.
    const Topology none = Topology::FromSysfs((root / "missing").string(), {0, 1, 2});
    ASSERT_EQ(none.l3.size(), 1u);
    EXPECT_EQ(none.l3[0].cpus.size(), 3u);
    fs::remove_all(root);
}

TEST(Topology, DetectCoversAllowedCpus) {
    const Topology& t = Topology::Get();
    ASSERT_FALSE(t.cpus.empty());
    ASSERT_FALSE(t.l3.empty());
    size_t covered = 0;
    for (const auto& d : t.l3) covered += d.cpus.size();
    EXPECT_EQ(covered, t.cpus.size());
    for (int c : t.cpus) EXPECT_GE(t.DomainOf(c), 0);
    EXPECT_FALSE(t.ToString().empty());
    std::printf("[topology] %s\n", t.ToString().c_str());
}

TEST(ThreadPool, PinnedToL3Domains) {
    ThreadPoolOptions o = Workers(4);
    o.pinToL3 = true;
    ThreadPool pool(o);
    EXPECT_EQ(Fib(pool, 22), 17711u);
}

TEST(ThreadPool, DefaultPool) {
    EXPECT_GE(DefaultThreadCount(), 1u);
    ThreadPool& p = DefaultPool();
    EXPECT_EQ(&p, &DefaultPool());
    EXPECT_GE(p.Size(), 1u);
    EXPECT_FALSE(SetDefaultPoolThreads(3));  // already created
    TaskGroup g;  // default pool
    std::atomic<int> n{0};
    for (int i = 0; i < 100; ++i) g.Spawn([&] { n.fetch_add(1); });
    g.Wait();
    EXPECT_EQ(n.load(), 100);
}

TEST(ThreadPool, IdleWorkersSleepAndWake) {
    ThreadPoolOptions o = Workers(4);
    o.spinRounds = 4;
    ThreadPool pool(o);
    for (int round = 0; round < 20; ++round) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));  // let workers fall asleep
        std::atomic<int> n{0};
        TaskGroup g(pool);
        for (int i = 0; i < 64; ++i) g.Spawn([&] { n.fetch_add(1); });
        g.Wait();
        ASSERT_EQ(n.load(), 64);
    }
    EXPECT_GT(pool.Stats().sleeps, 0u);
}

// Many nested waits: past ThreadPool::kMaxHelpDepth nested helps a thread stops picking up
// unrelated work, so stacks stay bounded (without the bound the waiting caller nested ~2800
// tasks deep here); nothing deadlocks.
TEST(TaskGroup, DeepNestingIsBounded) {
    for (size_t workers : {size_t{0}, size_t{1}, size_t{4}}) {
        ThreadPool pool(Workers(workers));
        std::atomic<int> maxDepth{0};
        std::function<void(int)> chain = [&](int level) {
            int d = ddbt::exec::detail::HelpDepth();
            int cur = maxDepth.load();
            while (d > cur && !maxDepth.compare_exchange_weak(cur, d)) {
            }
            if (level == 0) return;
            TaskGroup g(pool);
            for (int i = 0; i < 3; ++i) g.Spawn([&, level] { chain(level - 1); });
            g.Wait();
        };
        TaskGroup root(pool);
        for (int i = 0; i < 8; ++i) root.Spawn([&] { chain(6); });
        root.Wait();
        if (workers > 0) {
            EXPECT_LE(maxDepth.load(), ThreadPool::kMaxHelpDepth + 8) << workers;
        }
    }
}

// ---- review fixes ------------------------------------------------------------------------------

TEST(ChaseLevDeque, TagsPopIfStealIf) {
    detail::ChaseLevDeque<int> d(16);
    std::vector<int> items(6);
    for (int i = 0; i < 6; ++i) {
        items[static_cast<size_t>(i)] = i;
        d.Push(&items[static_cast<size_t>(i)], static_cast<unsigned>(i % 2));  // tags 0 1 0 1 0 1
    }
    EXPECT_EQ(d.TopTag(), 0);
    EXPECT_EQ(d.BottomTag(), 1);
    EXPECT_EQ(d.PopIf([](unsigned tag) { return tag == 0; }), nullptr);  // newest has tag 1
    EXPECT_EQ(d.SizeApprox(), 6);
    EXPECT_EQ(*d.PopIf([](unsigned tag) { return tag == 1; }), 5);
    EXPECT_EQ(d.StealIf([](unsigned tag) { return tag == 1; }), nullptr);  // oldest has tag 0
    EXPECT_EQ(*d.StealIf([](unsigned tag) { return tag == 0; }), 0);
    EXPECT_EQ(d.TopTag(), 1);
    EXPECT_EQ(*d.Steal(), 1);
    EXPECT_EQ(*d.Pop(), 4);  // pointers come back untagged
    EXPECT_EQ(*d.Pop(), 3);
    EXPECT_EQ(*d.Pop(), 2);
    EXPECT_EQ(d.BottomTag(), -1);
    EXPECT_EQ(d.TopTag(), -1);
}

// Growth while thieves steal, with the live range straddling the end of the ring (top and
// bottom on both sides of index 16, 32, ...) at every offset.
TEST(ChaseLevDeque, GrowUnderStealsAtRingWrap) {
    constexpr int kThieves = 3;
    constexpr int kItems = 80;  // grows 16 -> 32 -> 64 -> 128 during a round
    using Deque = detail::ChaseLevDeque<int>;
    std::vector<int> items(kItems);
    for (int i = 0; i < kItems; ++i) items[static_cast<size_t>(i)] = i;
    std::vector<std::atomic<int>> seen(kItems);
    std::atomic<Deque*> current{nullptr};
    std::atomic<int> busy{0};  // thieves inside a round's deque
    std::atomic<bool> stop{false};
    std::atomic<int> taken{0};
    std::vector<std::thread> thieves;
    for (int t = 0; t < kThieves; ++t) {
        thieves.emplace_back([&] {
            while (!stop.load(std::memory_order_acquire)) {
                busy.fetch_add(1, std::memory_order_seq_cst);
                if (Deque* d = current.load(std::memory_order_seq_cst)) {
                    if (int* p = d->Steal()) {
                        seen[static_cast<size_t>(*p)].fetch_add(1, std::memory_order_relaxed);
                        taken.fetch_add(1, std::memory_order_relaxed);
                    }
                }
                busy.fetch_sub(1, std::memory_order_seq_cst);
            }
        });
    }
    std::mt19937 rng(7);
    int dummy = -1;
    const int rounds = 600;
    for (int round = 0; round < rounds; ++round) {
        auto d = std::make_unique<Deque>(16);
        // Move top and bottom to offset (round % 16) of the ring before the thieves see it.
        for (int k = 0; k < round % 16; ++k) {
            d->Push(&dummy);
            ASSERT_EQ(d->Steal(), &dummy);
        }
        for (auto& s : seen) s.store(0, std::memory_order_relaxed);
        taken.store(0);
        current.store(d.get(), std::memory_order_seq_cst);
        for (int i = 0; i < kItems; ++i) {
            d->Push(&items[static_cast<size_t>(i)]);
            if (rng() % 5 == 0) {
                if (int* p = d->Pop()) {
                    seen[static_cast<size_t>(*p)].fetch_add(1, std::memory_order_relaxed);
                    taken.fetch_add(1, std::memory_order_relaxed);
                }
            }
        }
        while (int* p = d->Pop()) {
            seen[static_cast<size_t>(*p)].fetch_add(1, std::memory_order_relaxed);
            taken.fetch_add(1, std::memory_order_relaxed);
        }
        current.store(nullptr, std::memory_order_seq_cst);
        while (busy.load(std::memory_order_seq_cst) != 0) std::this_thread::yield();
        ASSERT_EQ(taken.load(), kItems) << "round " << round;
        for (int i = 0; i < kItems; ++i) ASSERT_EQ(seen[static_cast<size_t>(i)].load(), 1) << "round " << round << " item " << i;
    }
    stop.store(true);
    for (auto& t : thieves) t.join();
}

// F1: an isolated wait finds the tasks of its tree (also those another thread pushed) behind a
// 100k bulk backlog, and runs none of the bulk tasks.
TEST(ThreadPool, IsolatedWaitFindsItsTreeBehindABulkBacklog) {
    Watchdog dog(120s, "IsolatedWaitFindsItsTreeBehindABulkBacklog");
    ThreadPool pool(Workers(1));
    std::latch blocked(1);
    std::atomic<bool> release{false};
    pool.Submit([&] {  // keeps the only worker busy, so the backlog stays queued
        blocked.count_down();
        while (!release.load()) std::this_thread::yield();
    });
    blocked.wait();
    std::atomic<int> bulkRan{0};
    constexpr int kBacklog = 100000;
    for (int i = 0; i < kBacklog; ++i) pool.Submit([&] { bulkRan.fetch_add(1, std::memory_order_relaxed); });
    int key = 0;
    std::atomic<int> ran{0}, isolatedRuns{0};
    {
        IsolationScope scope(&key);
        TaskGroup g(pool);  // tagged with the scope
        std::atomic<bool> remoteDone{false};
        std::thread remote([&] {
            for (int i = 0; i < 10; ++i) {
                g.Spawn([&] {
                    if (IsolationScope::Inside(&key)) isolatedRuns.fetch_add(1);
                    ran.fetch_add(1);
                });
            }
            remoteDone = true;
        });
        g.Spawn([&] {
            while (!remoteDone.load()) std::this_thread::yield();
            ran.fetch_add(1);
        });
        g.Wait();
        remote.join();
    }
    EXPECT_EQ(ran.load(), 11);
    EXPECT_EQ(isolatedRuns.load(), 10);
    EXPECT_EQ(bulkRan.load(), 0);  // the isolated waiter never touched the backlog
    release = true;
    // The pool's destructor runs the backlog.
}

// F5: a thread waiting for interactive work does not run unrelated bulk tasks meanwhile.
TEST(TaskGroup, InteractiveWaitRunsNoBulkTask) {
    Watchdog dog(60s, "InteractiveWaitRunsNoBulkTask");
    ThreadPool pool(Workers(2));
    static thread_local bool inInteractiveWait = false;
    std::atomic<int> violations{0}, bulkRan{0};
    std::atomic<bool> subStarted{false}, bulkQueued{false}, tileDone{false};
    pool.Submit(
        [&] {
            TaskGroup g(pool, Priority::kInteractive);
            g.Spawn([&] {  // stolen by the other worker
                subStarted = true;
                while (!bulkQueued.load()) std::this_thread::yield();
                std::this_thread::sleep_for(20ms);
            });
            while (!subStarted.load() || !bulkQueued.load()) std::this_thread::yield();
            inInteractiveWait = true;
            g.Wait();  // nothing of its own left: must not pick up the bulk tasks
            inInteractiveWait = false;
            tileDone = true;
        },
        Priority::kInteractive);
    while (!subStarted.load()) std::this_thread::yield();
    for (int i = 0; i < 4; ++i) {
        pool.Submit([&] {
            if (inInteractiveWait) violations.fetch_add(1);
            std::this_thread::sleep_for(5ms);
            bulkRan.fetch_add(1);
        });
    }
    bulkQueued = true;
    while (!tileDone.load() || bulkRan.load() < 4) std::this_thread::yield();
    EXPECT_EQ(violations.load(), 0);
}

// F5: bulk ParallelFor helpers give way to a queued interactive request between chunks (before,
// a helper kept its worker until the whole range was done).
TEST(ParallelFor, BulkHelpersYieldToInteractiveRequests) {
    Watchdog dog(120s, "BulkHelpersYieldToInteractiveRequests");
    for (size_t workers : {size_t{1}, size_t{3}}) {
        ThreadPool pool(Workers(workers));
        constexpr size_t kChunks = 3000;
        std::vector<std::atomic<int>> hits(kChunks);
        std::atomic<size_t> chunksDone{0};
        std::thread build([&] {
            ParallelFor(pool, 0, kChunks, 1, [&](size_t i) {
                const auto end = std::chrono::steady_clock::now() + 50us;
                while (std::chrono::steady_clock::now() < end) {
                }
                hits[i].fetch_add(1, std::memory_order_relaxed);
                chunksDone.fetch_add(1, std::memory_order_relaxed);
            });
        });
        while (chunksDone.load() < 50) std::this_thread::yield();
        std::atomic<size_t> doneWhenServed{0};
        std::atomic<bool> served{false};
        pool.Submit(
            [&] {
                doneWhenServed = chunksDone.load();
                served = true;
            },
            Priority::kInteractive);
        build.join();
        while (!served.load()) std::this_thread::yield();
        EXPECT_LT(doneWhenServed.load(), kChunks / 2) << workers << " workers";
        for (size_t i = 0; i < kChunks; ++i) ASSERT_EQ(hits[i].load(), 1) << i;
    }
}

// F5: a worker that calls a bulk ParallelFor itself runs queued interactive requests between
// its chunks (a pool whose only worker is the caller).
TEST(ParallelFor, CallingWorkerServesInteractiveRequests) {
    Watchdog dog(60s, "CallingWorkerServesInteractiveRequests");
    ThreadPool pool(Workers(1));
    std::atomic<size_t> chunksDone{0}, doneWhenServed{0};
    std::atomic<bool> served{false}, buildDone{false};
    pool.Submit([&] {
        ParallelFor(pool, 0, 2000, 1, [&](size_t) {
            const auto end = std::chrono::steady_clock::now() + 50us;
            while (std::chrono::steady_clock::now() < end) {
            }
            chunksDone.fetch_add(1, std::memory_order_relaxed);
        });
        buildDone = true;
    });
    while (chunksDone.load() < 20) std::this_thread::yield();
    pool.Submit(
        [&] {
            doneWhenServed = chunksDone.load();
            served = true;
        },
        Priority::kInteractive);
    while (!buildDone.load() || !served.load()) std::this_thread::yield();
    EXPECT_LT(doneWhenServed.load(), 1000u);
    EXPECT_EQ(chunksDone.load(), 2000u);
}

// F6: a completing group wakes its waiter only, not the sleeping workers (before, every
// completion with a parked waiter woke the whole pool, and each idle worker went back to sleep).
TEST(ThreadPool, GroupCompletionWakesOnlyItsWaiter) {
    Watchdog dog(60s, "GroupCompletionWakesOnlyItsWaiter");
    constexpr int kWorkers = 12;
    ThreadPoolOptions o = Workers(kWorkers);
    o.spinRounds = 4;
    ThreadPool pool(o);
    uint64_t sleeps = 0;
    constexpr int kRounds = 10;
    for (int round = 0; round < kRounds; ++round) {
        std::this_thread::sleep_for(30ms);  // every worker blocked in its futex
        const uint64_t sleeps0 = pool.Stats().sleeps;
        std::atomic<bool> done{false};
        pool.Submit([&] {
            std::atomic<bool> stolen{false};
            TaskGroup g(pool);
            g.Spawn([&] {
                stolen = true;
                std::this_thread::sleep_for(3ms);  // the waiter parks, idle workers sleep
            });
            while (!stolen.load()) std::this_thread::yield();
            g.Wait();
            done = true;
        });
        while (!done.load()) std::this_thread::yield();
        std::this_thread::sleep_for(5ms);
        sleeps += pool.Stats().sleeps - sleeps0;
    }
    // About 2 per round (the two workers involved, more under load); a completion that wakes
    // the whole pool costs every worker a sleep.
    EXPECT_LE(sleeps, uint64_t{kRounds} * kWorkers / 2) << sleeps;
}

// A pool destroyed while all its workers sleep (no work ever, or after a burst).
TEST(ThreadPool, IdleDestructorWhileWorkersSleep) {
    Watchdog dog(60s, "IdleDestructorWhileWorkersSleep");
    for (int round = 0; round < 20; ++round) {
        ThreadPoolOptions o = Workers(4);
        o.spinRounds = 1;
        ThreadPool pool(o);
        if (round % 2 == 1) {
            std::atomic<int> n{0};
            for (int i = 0; i < 100; ++i) pool.Submit([&] { n.fetch_add(1); });
            while (n.load() < 100) std::this_thread::yield();
        }
        std::this_thread::sleep_for(3ms);  // every worker asleep
    }
}

// Depth bound without workers: several outside threads nest waits on a pool with no worker;
// past kMaxHelpDepth a thread runs its own subtasks first (wherever they are in the injection
// queue), so its stack stays bounded although the others interleave their pushes.
TEST(TaskGroup, ZeroWorkersManyClientsDepthBounded) {
    Watchdog dog(120s, "ZeroWorkersManyClientsDepthBounded");
    ThreadPool pool(Workers(0));
    std::atomic<int> maxDepth{0};
    std::atomic<int> leaves{0};
    std::function<void(int)> chain = [&](int level) {
        const int d = ddbt::exec::detail::HelpDepth();
        int cur = maxDepth.load();
        while (d > cur && !maxDepth.compare_exchange_weak(cur, d)) {
        }
        if (level == 0) {
            leaves.fetch_add(1);
            return;
        }
        TaskGroup g(pool);
        for (int i = 0; i < 3; ++i) g.Spawn([&, level] { chain(level - 1); });
        g.Wait();
    };
    std::vector<std::thread> clients;
    for (int c = 0; c < 4; ++c) {
        clients.emplace_back([&] {
            TaskGroup root(pool);
            for (int i = 0; i < 8; ++i) root.Spawn([&] { chain(6); });
            root.Wait();
        });
    }
    for (auto& t : clients) t.join();
    EXPECT_EQ(leaves.load(), 4 * 8 * 729);
    EXPECT_LE(maxDepth.load(), ThreadPool::kMaxHelpDepth + 16) << maxDepth.load();
}

// F7: the default pool can be shut down (drained and joined), repeatedly, and is created again
// on the next use.
TEST(ThreadPool, ShutdownDefaultPool) {
    std::atomic<int> n{0};
    for (int i = 0; i < 50; ++i) DefaultPool().Submit([&] { n.fetch_add(1); });
    ShutdownDefaultPool();  // runs what is queued, joins
    EXPECT_EQ(n.load(), 50);
    ShutdownDefaultPool();  // idempotent
    EXPECT_TRUE(SetDefaultPoolThreads(2));
    ThreadPool& p = DefaultPool();
    EXPECT_EQ(p.Size(), 2u);
    // Not from one of its own workers (Submit from this thread: only a worker runs it).
    std::atomic<int> outcome{0};
    p.Submit([&] {
        try {
            ShutdownDefaultPool();
            outcome = 2;
        } catch (const std::logic_error&) {
            outcome = 1;
        }
    });
    while (outcome.load() == 0) std::this_thread::yield();
    EXPECT_EQ(outcome.load(), 1);
    ParallelFor(0, 100, 1, [&](size_t) { n.fetch_add(1); });
    EXPECT_EQ(n.load(), 150);
    ShutdownDefaultPool();
    EXPECT_TRUE(SetDefaultPoolThreads(0));  // back to DefaultThreadCount() for later tests
}
