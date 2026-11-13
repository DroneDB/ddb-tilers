// SPDX-License-Identifier: MPL-2.0
#pragma once

// Work-stealing thread pool.
//
//  - One Chase-Lev deque per worker: a worker pushes and pops its own tasks at the bottom (LIFO,
//    cache-hot) and steals from the top of the others (same L3 domain first).
//  - Two injection queues for tasks submitted by threads outside the pool: interactive (tile
//    requests) and bulk (builds). A worker looks for work in this order: its own deque, the
//    interactive queue, the other workers' deques, the bulk queue.
//  - Idle workers spin briefly ("searching"), then sleep, each on its own word (std::atomic::wait:
//    futex on Linux, WaitOnAddress on Windows), listed on a stack of sleepers. A wake-up takes
//    one sleeper off the stack and changes only its word, so exactly one worker leaves its wait
//    (with one shared word, every sleeper still on its way into the wait, or spinning inside the
//    library's wait, would leave on any change). A push wakes a sleeper only when no worker is
//    searching; the last searcher to find work wakes the next one, so parallelism ramps up one
//    wake-up at a time instead of one system call per push.
//  - A waiting thread that runs out of work parks on its own word (detail::Parker): the group or
//    flight it waits for wakes exactly that thread when it completes. New work wakes an idle
//    worker first, a parked helper that may run anything only when no worker sleeps, and a
//    parked isolated helper only for a task of its own tree. No completion wakes the whole pool.
//  - Priorities: interactive tasks are picked before bulk ones (injection queue order, and a
//    worker takes a queued interactive task before a bulk task of its own deque). On Windows a
//    worker runs bulk tasks below normal OS priority, so the application's threads that submit
//    requests are not kept off the CPUs for a scheduler quantum by a busy pool. A thread
//    waiting for interactive work (an interactive group, or any wait inside an interactive task)
//    never runs unrelated bulk tasks: only its own subtasks and interactive tasks.
//  - Invariant: a worker's deque only holds tasks spawned by the frames on its own stack
//    (injected tasks are taken one at a time and run at once, never moved into a deque). The
//    isolation rules below rely on it.
//  - Waiting threads help: TaskGroup::Wait and the cache's single-flight waits run queued tasks
//    instead of blocking (HelpUntil), so a task can wait for its subtasks without deadlock even
//    on a pool with a single worker, or with no worker at all.
//  - Isolation: code inside an IsolationScope (a cache loader) and every task spawned from
//    it through a TaskGroup created inside it carry the scope's tag. Their waits only run
//    tasks of their own tree (the thread's own deque above the point where the isolated work
//    started, and tagged descendants in the injection queues), never unrelated work. Without
//    this, a thread could suspend a loader under an unrelated task that (through other
//    threads) waits for that very loader: a deadlock with no cyclic dependency in user code.
//    Isolation belongs to the region (as in TBB): an untagged task that an isolated wait runs
//    (a Submit, or a task of a group created outside the scope) runs inside the waiter's
//    isolation, so its own waits are isolated too.
//
// Use TaskGroup (task_group.h) to spawn and wait; Submit is fire-and-forget.

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "core/exec/chase_lev_deque.h"

namespace ddbt::exec {

enum class Priority : uint8_t {
    kInteractive = 0,  // latency-sensitive (tile requests): picked before bulk work
    kBulk = 1,         // throughput work (builds)
};

struct ThreadPoolOptions {
    /// Worker threads; 0 = DefaultThreadCount(). A pool may also be created with no worker
    /// at all (threads = 0 and zeroWorkers = true): then waiting threads run every task.
    size_t threads = 0;
    bool zeroWorkers = false;
    /// Pin each worker to the CPUs of one L3 domain (CCD), workers spread over the domains in
    /// proportion to their size.
    bool pinToL3 = false;
    /// Rounds of looking for work (with exponential pause back-off) before an idle worker
    /// sleeps.
    uint32_t spinRounds = 64;
    /// Thread name prefix ("ddbt-3").
    std::string name = "ddbt";
};

struct ThreadPoolStats {
    uint64_t executed = 0;   // tasks run by workers from their main loop
    uint64_t helped = 0;     // tasks run by waiting threads (TryRunOne / HelpUntil)
    uint64_t stolen = 0;     // tasks a worker took from another worker's deque
    uint64_t injected = 0;   // tasks submitted from outside the pool
    uint64_t sleeps = 0;     // times a worker went to sleep
};

/// Isolation tag: one per isolated scope (a running cache loader), linked to the enclosing
/// scope. Lives on the stack of the scope.
struct IsolationNode {
    const void* key = nullptr;
    const IsolationNode* parent = nullptr;
};

namespace detail {

/// Intrusive task header. `invoke` runs the task and destroys it; it must not throw.
struct Task {
    using InvokeFn = void (*)(Task*) noexcept;
    explicit Task(InvokeFn fn, const IsolationNode* iso = nullptr) noexcept : invoke(fn), isolation(iso) {}
    InvokeFn invoke;
    const IsolationNode* isolation;  // tree the task belongs to (nullptr: none)
    Task* next = nullptr;            // injection queue links
    Task* prev = nullptr;
    Task* snext = nullptr;           // links of the injection queue's restricted list (`restricted`)
    Task* sprev = nullptr;
    // Packed into one word (the header is 56 bytes):
    uint64_t seq : 42 = 0;      // injected tasks: the pushing thread's push number
    uint64_t origin : 20 = 0;   // injected tasks: the pushing thread (its parker id)
    uint64_t bulk : 1 = 1;      // priority (set by ThreadPool::Push)
    /// Injected tasks that a restricted wait may look for: tagged, or pushed by a thread whose
    /// own waits are restricted (isolated, or nested kMaxHelpDepth deep). Only these are
    /// scanned by isolated waits.
    uint64_t restricted : 1 = 0;

    Priority priority() const noexcept { return bulk != 0 ? Priority::kBulk : Priority::kInteractive; }
};

/// Per-thread wake-up word of a waiting thread. Parkers are never freed (a thread's parker is
/// reused by later threads), so a completing task may wake one after its waiter has left.
struct alignas(64) Parker {
    std::atomic<uint32_t> word{0};
    uint32_t id = 0;        // 1-based, < 2^kParkerIdBits
    uint32_t nextFree = 0;  // free list (registry)
    uint64_t spawnSeq = 0;  // push count of the id's last owner (the next one continues it)
    void Unpark() noexcept {
        word.fetch_add(1, std::memory_order_release);
        word.notify_one();
    }
};
inline constexpr unsigned kParkerIdBits = 20;
/// The calling thread's parker (assigned on first use, returned to the registry at thread exit).
Parker& ThisThreadParker() noexcept;
/// Parker of an id handed out by ThisThreadParker().id.
Parker& ParkerById(uint32_t id) noexcept;

/// Priority of the task the calling thread runs (kBulk outside tasks).
inline Priority& CurrentPriority() noexcept {
    thread_local Priority priority = Priority::kBulk;
    return priority;
}

/// Spin-wait hint (x86 PAUSE).
void CpuRelax() noexcept;

/// Tasks the calling thread is running from inside HelpUntil (nesting of helping waits).
inline int& HelpDepth() noexcept {
    thread_local int depth = 0;
    return depth;
}
struct HelpDepthScope {
    HelpDepthScope() noexcept { ++HelpDepth(); }
    ~HelpDepthScope() { --HelpDepth(); }
    HelpDepthScope(const HelpDepthScope&) = delete;
    HelpDepthScope& operator=(const HelpDepthScope&) = delete;
};

/// Isolation state of the calling thread.
struct IsolationState {
    const IsolationNode* node = nullptr;
    int64_t floor = 0;      // own-deque index where the current task / scope started (workers)
    uint64_t seqFloor = 0;  // this thread's injection push count at that point
};
IsolationState& CurrentIsolation() noexcept;

/// True when `node` is `ancestor` or below it.
inline bool IsWithin(const IsolationNode* node, const IsolationNode* ancestor) noexcept {
    for (; node != nullptr; node = node->parent) {
        if (node == ancestor) return true;
    }
    return false;
}

}  // namespace detail

class ThreadPool;

/// Marks the calling thread's code until the end of the scope as isolated work tagged `key`
/// (nested inside any enclosing scope). TaskGroups created inside inherit the tag.
class IsolationScope {
public:
    explicit IsolationScope(const void* key) noexcept;
    ~IsolationScope();
    IsolationScope(const IsolationScope&) = delete;
    IsolationScope& operator=(const IsolationScope&) = delete;
    const IsolationNode* node() const noexcept { return &node_; }

    /// True if the calling thread is inside a scope (here or in an ancestor task) tagged `key`.
    static bool Inside(const void* key) noexcept;

private:
    IsolationNode node_;
    detail::IsolationState saved_;
};

class ThreadPool {
public:
    ThreadPool() : ThreadPool(ThreadPoolOptions{}) {}
    explicit ThreadPool(ThreadPoolOptions options);
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    /// Runs every task still queued, then joins the workers. No TaskGroup of this pool may be
    /// alive and no other thread may submit while it runs.
    ~ThreadPool();

    /// Nesting limit of helping waits on one thread (see HelpUntil).
    static constexpr int kMaxHelpDepth = 48;

    /// Number of worker threads.
    size_t Size() const noexcept { return workers_.size(); }

    /// Fire-and-forget. `fn` must not throw (std::terminate otherwise, like std::thread).
    /// Not isolated: it may run anywhere, at any time.
    template <typename F>
    void Submit(F&& fn, Priority priority = Priority::kBulk) {
        struct Fire final : detail::Task {
            explicit Fire(F&& f) : detail::Task(&Invoke), fn(std::forward<F>(f)) {}
            static void Invoke(detail::Task* t) noexcept {
                auto* self = static_cast<Fire*>(t);
                self->fn();
                delete self;
            }
            std::decay_t<F> fn;
        };
        auto* t = new Fire(std::forward<F>(fn));
        try {
            Push(t, priority);
        } catch (...) {
            delete t;
            throw;
        }
    }

    /// Runs one queued task on the calling thread, if any (not isolated: any task). Workers of
    /// this pool look in their own deque first; other threads take from the injection queues
    /// and steal.
    bool TryRunOne();

    /// Runs one queued interactive task from the injection queue on the calling thread, if
    /// there is one and the thread may run unrelated work (it is not inside an isolated scope
    /// and not nested kMaxHelpDepth helps deep). Long bulk loops call it between chunks.
    bool TryRunInteractive();

    /// True while interactive tasks wait in the injection queue (a cheap atomic read): bulk
    /// loops yield between chunks when it is set.
    bool InteractivePending() const noexcept { return inject_[0].size.load(std::memory_order_relaxed) != 0; }

    /// Pool of the calling worker thread, or nullptr when the caller is not a pool worker.
    static ThreadPool* Current() noexcept;
    /// Index of the calling worker in its pool, or -1.
    static int CurrentWorkerIndex() noexcept;

    /// Runs tasks of this pool until `done()` is true (only tasks of the caller's own tree
    /// when it is isolated; no unrelated bulk task when `priority` or the task the caller runs
    /// is interactive). When no task is available it spins briefly, then parks until woken.
    /// Before parking it calls `arm`, which returns true if the condition already holds, else
    /// arranges a wake-up for when it becomes true:
    ///  - `arm(detail::Parker& p)`: call `p.Unpark()` then (wakes only this thread), or
    ///  - `arm()`: call WakeAll() then (wakes every parked thread; kept for compatibility).
    /// A parked caller is also woken for new work it may run. Used by TaskGroup::Wait and the
    /// cache.
    template <typename Done, typename Arm>
    void HelpUntil(Done&& done, Arm&& arm, Priority priority = Priority::kBulk);

    /// Wakes every sleeping worker and parked helper (they re-check their conditions).
    void WakeAll() noexcept;

    ThreadPoolStats Stats() const noexcept;

    /// Pushes a task: to the calling worker's deque when it is a worker of this pool, else to
    /// the injection queue of `priority`. Wakes a sleeper. Throws std::bad_alloc only.
    void Push(detail::Task* task, Priority priority);

private:
    friend class IsolationScope;

    /// How a helping wait picks tasks.
    enum class WaitMode : uint8_t {
        kAny,               // any task
        kIsolated,          // own tree / own subtasks only (isolated, or nested too deep)
        kOwnFirst,          // nested too deep on a pool without workers: own subtasks first
        kInteractive,       // own subtasks and interactive tasks only
        kInteractiveFirst,  // same, then anything (pool without workers)
    };
    struct WaitContext {
        detail::IsolationState scope;
        int self = -1;
        WaitMode mode = WaitMode::kAny;
    };
    /// A parked helper (on its stack), listed while it may sleep. Guarded by parkMutex_.
    struct ParkEntry {
        ParkEntry* prev = nullptr;
        ParkEntry* next = nullptr;
        detail::Parker* parker = nullptr;
        const IsolationNode* node = nullptr;  // isolated waiters: their scope (tagged pushes wake them)
        bool general = false;                 // runs any task: woken for new work
        bool listed = false;
        bool wokenForWork = false;
    };
    /// Unlists the entry when it leaves HelpUntil, also by exception (from `arm`).
    struct ParkGuard {
        ThreadPool* pool;
        ParkEntry* entry;
        bool released = false;
        /// Returns true if a wake-up for new work unlisted the entry.
        bool Release() noexcept {
            released = true;
            return pool->UnregisterPark(*entry);
        }
        ~ParkGuard() {
            if (!released) pool->UnregisterPark(*entry);
        }
    };

    struct alignas(64) Worker {
        detail::ChaseLevDeque<detail::Task> deque;
        std::vector<uint32_t> victims;  // steal order: same L3 domain first
        std::vector<int> cpus;          // pinning set (empty = not pinned)
        std::thread thread;
        uint64_t rng = 0;
        std::atomic<uint64_t> executed{0};
        std::atomic<uint64_t> stolen{0};
        std::atomic<uint64_t> sleeps{0};
        std::atomic<uint32_t> wake{0};  // the word this worker sleeps on (changed to wake it)
        bool idle = false;              // on the stack of sleepers (guarded by idleMutex_)
    };

    /// Doubly linked FIFO of injected tasks, plus a second list through the same tasks of the
    /// restricted ones (Task::restricted), so isolated waits never walk the bulk backlog.
    struct alignas(64) InjectionQueue {
        std::mutex mutex;
        detail::Task* head = nullptr;
        detail::Task* tail = nullptr;
        detail::Task* rhead = nullptr;  // restricted list
        detail::Task* rtail = nullptr;
        std::atomic<size_t> size{0};
        std::atomic<size_t> restrictedSize{0};

        /// Caller holds the mutex.
        void LinkLocked(detail::Task* t) noexcept;
        detail::Task* TryPop() noexcept;
        /// Like TryPop, but gives up when the lock is busy (workers).
        detail::Task* TryPopUncontended() noexcept;
        /// The newest task if it was pushed by `origin` after its push number `seqFloor`.
        detail::Task* TryPopNewestOwn(uint32_t origin, uint64_t seqFloor) noexcept;
        bool HasNewestOwn(uint32_t origin, uint64_t seqFloor) noexcept;
        void Unlink(detail::Task* t) noexcept;
        /// Newest restricted task of the caller's scope (its tree, or its own pushes since
        /// `scope.seqFloor` when `own`), or nullptr / false. O(restricted tasks), and no lock at
        /// all when there are none.
        detail::Task* TryPopFor(const detail::IsolationState& scope, uint32_t origin, bool own) noexcept;
        bool HasFor(const detail::IsolationState& scope, uint32_t origin, bool own) noexcept;
    };

    void WorkerMain(uint32_t index);
    detail::Task* FindWork(int self);
    detail::Task* TakeInjected(int queue, int self);
    detail::Task* FindIsolatedWork(int self, const detail::IsolationState& iso);
    detail::Task* FindOwnInjected(const detail::IsolationState& iso);
    detail::Task* FindInteractiveWork(int self, const detail::IsolationState& iso);
    detail::Task* StealFrom(int self, bool interactiveOnly);
    bool HasWork() const noexcept;
    bool HasIsolatedWork(int self, const detail::IsolationState& iso) noexcept;
    bool HasInteractiveWork(int self, const detail::IsolationState& iso) noexcept;
    /// Links an injected task; returns the parker of an isolated helper of its tree to wake
    /// (or nullptr).
    detail::Parker* Inject(detail::Task* t, int queue) noexcept;
    void NotifyOne() noexcept;
    /// Takes the most recent sleeper off the stack of sleepers and wakes it; false when none.
    bool WakeOneSleeper() noexcept;
    void RunTask(detail::Task* t, int self, const IsolationNode* inherit, bool interactiveRegion) noexcept;
    int SelfIndex() const noexcept;

    WaitContext BeginWait(Priority priority) const noexcept;
    bool TryHelp(const WaitContext& ctx);
    bool HasHelpWork(const WaitContext& ctx) noexcept;
    void RegisterPark(ParkEntry& entry, const WaitContext& ctx, detail::Parker& parker) noexcept;
    /// Returns true if a new-work wake-up (NotifyOne, or a task of an isolated helper's tree)
    /// unlisted the entry.
    bool UnregisterPark(ParkEntry& entry) noexcept;
    /// A helper woken for new work found its condition true instead: wake others for the work.
    void PassOnWake() noexcept;

    ThreadPoolOptions options_;
    std::vector<std::unique_ptr<Worker>> workers_;
    InjectionQueue inject_[2];
    alignas(64) std::mutex idleMutex_;
    std::vector<uint32_t> idle_;                     // sleeping workers (a stack; capacity: all)
    alignas(64) std::atomic<uint32_t> sleepers_{0};  // size of idle_ (asleep or about to be)
    std::atomic<uint32_t> searching_{0};             // workers spinning for work
    alignas(64) std::mutex parkMutex_;
    ParkEntry* parked_ = nullptr;                    // parked helpers
    std::atomic<uint32_t> parkedGeneral_{0};
    std::atomic<uint32_t> parkedIsolated_{0};
    alignas(64) std::atomic<bool> stopping_{false};
    std::atomic<uint64_t> helped_{0};
    std::atomic<uint64_t> injected_{0};
};

/// Worker count used for the default pool: $DDBT_THREADS if set (> 0), else the number of
/// CPUs in the process affinity mask.
size_t DefaultThreadCount();

/// Process-wide pool, created on first use with DefaultThreadCount() workers.
ThreadPool& DefaultPool();

/// Sets the default pool's worker count. Only before the first DefaultPool() call (or after
/// ShutdownDefaultPool()); returns false (and changes nothing) while a default pool exists.
bool SetDefaultPoolThreads(size_t threads);

/// Destroys the default pool: runs every task still queued, then joins its workers (the C API
/// teardown calls it). Idempotent; a no-op when no default pool exists. Afterwards
/// DefaultPool() creates a new pool (honouring SetDefaultPoolThreads), so a library can be
/// shut down and initialised again; references to the old pool are dangling. No task group of
/// the default pool may be alive and no thread may use it concurrently. Throws
/// std::logic_error when called from one of the default pool's own workers (it cannot join
/// itself).
void ShutdownDefaultPool();

// ---- implementation ------------------------------------------------------------------------

template <typename Done, typename Arm>
void ThreadPool::HelpUntil(Done&& done, Arm&& arm, Priority priority) {
    // Isolated callers only run tasks of their own tree. A thread nested too deep in helping
    // (kMaxHelpDepth tasks picked up by waits on its stack) is restricted to the tasks pushed
    // since its current task started (a worker's own deque above that point, or the tasks the
    // thread injected since then): its own subtasks, never unrelated work, which bounds its
    // stack depth while the workers run the rest. A pool without workers cannot restrict that
    // strictly (nobody else might run the rest): its deep helpers run their own subtasks first.
    // Interactive waits skip unrelated bulk tasks (BeginWait picks the mode).
    const WaitContext ctx = BeginWait(priority);
    uint32_t spins = 0;
    while (!done()) {
        bool ran;
        {
            detail::HelpDepthScope depth;
            ran = TryHelp(ctx);
        }
        if (ran) {
            spins = 0;
            continue;
        }
        if (spins < options_.spinRounds) {
            for (uint32_t i = 0, n = 1u << std::min<uint32_t>(spins / 8, 6); i < n; ++i) detail::CpuRelax();
            ++spins;
            continue;
        }
        spins = 0;
        // Park (Dekker-style, as idle workers): read the word, list the entry (new work that we
        // may run wakes us from then on), re-check everything, arm the completion wake-up, wait.
        detail::Parker& parker = detail::ThisThreadParker();
        ParkEntry entry;
        const uint32_t word = parker.word.load(std::memory_order_acquire);
        RegisterPark(entry, ctx, parker);
        bool woken;
        {
            ParkGuard guard{this, &entry};
            bool skip = done() || HasHelpWork(ctx);
            if (!skip) {
                if constexpr (std::is_invocable_r_v<bool, Arm&, detail::Parker&>) {
                    skip = arm(parker);
                } else {
                    skip = arm();
                }
            }
            if (!skip) parker.word.wait(word, std::memory_order_acquire);
            woken = guard.Release();
        }
        // A wake-up meant for new work may have reached us although we are done: pass it on.
        if (woken && done()) PassOnWake();
    }
}

}  // namespace ddbt::exec
