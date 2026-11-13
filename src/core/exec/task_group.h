// SPDX-License-Identifier: MPL-2.0
#pragma once

// TaskGroup: spawn tasks on a pool and wait for all of them.
//
//   TaskGroup g(pool);
//   for (auto& tile : tiles) g.Spawn([&] { Encode(tile); });
//   g.Wait();   // runs queued tasks while waiting; rethrows the first exception
//
// Wait() never just blocks while work is queued: the waiting thread runs tasks of the pool
// (its own subtasks first when it is a worker), so tasks may create nested groups and wait on
// them, at any depth, without deadlock.
//
// A group created inside an isolated scope (a cache loader, or a task of such a scope) is
// isolated too: its tasks carry the scope's tag and Wait() only runs tasks of that tree.
//
// Priority: an interactive group's tasks go before bulk work, and while Wait() waits for an
// interactive group (or is called from an interactive task) it runs no unrelated bulk task.
// A waiter that runs out of work parks; the group's last task wakes exactly that thread.
//
// The first exception thrown by a task cancels the group (tasks not yet started are skipped)
// and is rethrown by Wait(). Cancel() / a cancelled parent token also skip unstarted tasks;
// running tasks can poll IsCancelled(). Wait() returns normally after a cancellation without
// exception.

#include <atomic>
#include <cstdint>
#include <exception>
#include <mutex>
#include <type_traits>
#include <utility>

#include "core/exec/cancel_token.h"
#include "core/exec/thread_pool.h"

namespace ddbt::exec {

class TaskGroup {
public:
    explicit TaskGroup(ThreadPool& pool = DefaultPool(), Priority priority = Priority::kBulk,
                       const CancelToken* parent = nullptr)
        : pool_(&pool),
          priority_(priority),
          isolation_(detail::CurrentIsolation().node),
          token_(parent != nullptr ? parent->Child() : CancelToken()) {}
    TaskGroup(const TaskGroup&) = delete;
    TaskGroup& operator=(const TaskGroup&) = delete;
    /// Waits for the remaining tasks. An exception not collected by Wait() is dropped (a
    /// destructor cannot throw): call Wait().
    ~TaskGroup();

    /// Queues `fn` (a callable with no arguments). Thread-safe, also from inside the group's
    /// own tasks.
    template <typename F>
    void Spawn(F&& fn);

    /// Runs tasks until every spawned task has finished; rethrows the first task exception.
    /// One waiting thread at a time.
    void Wait();

    void Cancel() noexcept { token_.Cancel(); }
    bool IsCancelled() const noexcept { return token_.IsCancelled(); }
    const CancelToken& Token() const noexcept { return token_; }
    ThreadPool& Pool() const noexcept { return *pool_; }

private:
    // state_: pending task count (low 40 bits), the parker id of a parked waiter, waiter flag.
    // The last task to finish reads the waiter from the value its decrement returns, so it never
    // touches the group after the decrement (the group may be gone by then).
    static constexpr unsigned kParkerShift = 40;
    static constexpr uint64_t kWaiterBit = uint64_t{1} << 63;
    static constexpr uint64_t kCountMask = (uint64_t{1} << kParkerShift) - 1;
    static constexpr uint64_t kParkerMask = ((uint64_t{1} << detail::kParkerIdBits) - 1) << kParkerShift;
    static_assert(kParkerShift + detail::kParkerIdBits < 63);

    template <typename F>
    struct GroupTask final : detail::Task {
        GroupTask(TaskGroup* g, F&& f) : detail::Task(&Invoke, g->isolation_), group(g), fn(std::forward<F>(f)) {}
        static void Invoke(detail::Task* t) noexcept {
            auto* self = static_cast<GroupTask*>(t);
            TaskGroup* g = self->group;
            if (!g->IsCancelled()) {
                try {
                    self->fn();
                } catch (...) {
                    g->SetError(std::current_exception());
                }
            }
            delete self;  // captured state is destroyed before the group counts the task done
            g->Finish();
        }
        TaskGroup* group;
        std::decay_t<F> fn;
    };

    void SetError(std::exception_ptr e) noexcept;
    void Finish() noexcept;
    void WaitNoThrow() noexcept;

    ThreadPool* pool_;
    Priority priority_;
    const IsolationNode* isolation_;  // scope the group was created in (lives longer than it)
    CancelToken token_;
    alignas(64) std::atomic<uint64_t> state_{0};  // pending count | kWaiterBit
    std::mutex errorMutex_;
    std::exception_ptr error_;
};

template <typename F>
void TaskGroup::Spawn(F&& fn) {
    auto* t = new GroupTask<F>(this, std::forward<F>(fn));
    state_.fetch_add(1, std::memory_order_relaxed);
    try {
        pool_->Push(t, priority_);
    } catch (...) {
        delete t;
        Finish();
        throw;
    }
}

}  // namespace ddbt::exec
