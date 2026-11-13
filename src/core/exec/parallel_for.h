// SPDX-License-Identifier: MPL-2.0
#pragma once

// ParallelFor(begin, end, grain, fn): runs fn over [begin, end) in chunks of `grain` indexes.
// fn is either fn(size_t i) (called per index) or fn(size_t lo, size_t hi) (called per chunk,
// preferred for tight loops).
//
// Scheduling: chunks are claimed dynamically from a shared counter by the calling thread plus
// up to Size() helper tasks, so uneven chunk costs balance out; the caller always takes part
// (a pool with no free worker degrades to a serial loop on the caller, never a deadlock).
// Nested ParallelFor calls from inside tasks are fine.
//
// Priority: a bulk ParallelFor gives way to interactive work between chunks. While interactive
// tasks are queued (ThreadPool::InteractivePending), a helper hands the rest of its share back
// to the pool as a new helper task and returns, so its worker picks up the request first and
// resumes the loop afterwards; a calling pool worker runs the queued requests itself. Without
// this a helper that started draining the range kept its worker until the range was done.
//
// The first exception from fn stops the remaining chunks and is rethrown. A cancelled `cancel`
// token stops claiming chunks; ParallelFor then returns normally (check the token).

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <exception>
#include <new>
#include <type_traits>

#include "core/exec/task_group.h"

namespace ddbt::exec {

template <typename F>
void ParallelFor(ThreadPool& pool, size_t begin, size_t end, size_t grain, F&& fn, const CancelToken* cancel = nullptr,
                 Priority priority = Priority::kBulk) {
    if (begin >= end) return;
    if (grain == 0) grain = 1;
    const size_t n = end - begin;
    const size_t chunks = n / grain + (n % grain != 0 ? 1 : 0);

    auto runChunk = [&](size_t c) {
        const size_t lo = begin + c * grain;
        const size_t hi = c + 1 == chunks ? end : lo + grain;
        if constexpr (std::is_invocable_v<F&, size_t, size_t>) {
            fn(lo, hi);
        } else {
            for (size_t i = lo; i < hi; ++i) fn(i);
        }
    };

    const size_t helpers = std::min(chunks - 1, pool.Size());
    if (helpers == 0) {
        for (size_t c = 0; c < chunks; ++c) {
            if (cancel != nullptr && cancel->IsCancelled()) return;
            runChunk(c);
        }
        return;
    }

    TaskGroup group(pool, priority, cancel);
    std::atomic<size_t> next{0};
    using RunChunk = decltype(runChunk);
    struct Helper {
        ThreadPool& pool;
        TaskGroup& group;
        std::atomic<size_t>& next;
        RunChunk& run;
        size_t chunks;

        void operator()() const {
            while (!group.IsCancelled()) {
                const size_t c = next.fetch_add(1, std::memory_order_relaxed);
                if (c >= chunks) return;
                run(c);
                // Bulk work (not run inside an interactive region) yields to queued requests.
                if (detail::CurrentPriority() == Priority::kBulk && pool.InteractivePending() &&
                    next.load(std::memory_order_relaxed) < chunks) {
                    try {
                        group.Spawn(*this);
                        return;
                    } catch (const std::bad_alloc&) {
                        // Keep going on this thread.
                    }
                }
            }
        }
    };
    const Helper helper{pool, group, next, runChunk, chunks};
    auto body = [&] {
        const bool runRequests = priority == Priority::kBulk && ThreadPool::Current() == &pool;
        while (!group.IsCancelled()) {
            const size_t c = next.fetch_add(1, std::memory_order_relaxed);
            if (c >= chunks) return;
            runChunk(c);
            if (runRequests && detail::CurrentPriority() == Priority::kBulk) {
                while (pool.InteractivePending() && pool.TryRunInteractive()) {
                }
            }
        }
    };
    for (size_t i = 0; i < helpers; ++i) group.Spawn(helper);
    try {
        body();
    } catch (...) {
        group.Cancel();
        try {
            group.Wait();
        } catch (...) {
            // The caller's own exception wins; a helper's is dropped.
        }
        throw;
    }
    group.Wait();
}

/// ParallelFor on the default pool.
template <typename F>
void ParallelFor(size_t begin, size_t end, size_t grain, F&& fn) {
    ParallelFor(DefaultPool(), begin, end, grain, std::forward<F>(fn));
}

}  // namespace ddbt::exec
