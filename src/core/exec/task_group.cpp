// SPDX-License-Identifier: MPL-2.0
#include "core/exec/task_group.h"

namespace ddbt::exec {

TaskGroup::~TaskGroup() { WaitNoThrow(); }

void TaskGroup::SetError(std::exception_ptr e) noexcept {
    {
        std::lock_guard lock(errorMutex_);
        if (!error_) error_ = std::move(e);
    }
    token_.Cancel();
}

void TaskGroup::Finish() noexcept {
    // The group may be destroyed as soon as the count reaches zero (a waiter that is spinning
    // returns immediately): everything needed comes from the decrement's result.
    const uint64_t prev = state_.fetch_sub(1, std::memory_order_acq_rel);
    if ((prev & kCountMask) == 1 && (prev & kWaiterBit) != 0) {
        // A waiter is (or is about to be) parked; parkers are never freed, so this is safe even
        // if it has already left.
        detail::ParkerById(static_cast<uint32_t>((prev & kParkerMask) >> kParkerShift)).Unpark();
    }
}

void TaskGroup::WaitNoThrow() noexcept {
    if ((state_.load(std::memory_order_acquire) & kCountMask) != 0) {
        pool_->HelpUntil([this] { return (state_.load(std::memory_order_acquire) & kCountMask) == 0; },
                         [this](detail::Parker& parker) {
                             // Ask the last finishing task to wake this thread.
                             const uint64_t bits = kWaiterBit | (uint64_t{parker.id} << kParkerShift);
                             return (state_.fetch_or(bits, std::memory_order_seq_cst) & kCountMask) == 0;
                         },
                         priority_);
    }
    // Count is zero: no task touches state_ any more.
    state_.store(0, std::memory_order_relaxed);
}

void TaskGroup::Wait() {
    WaitNoThrow();
    std::exception_ptr e;
    {
        std::lock_guard lock(errorMutex_);
        e = std::exchange(error_, nullptr);
    }
    if (e) std::rethrow_exception(e);
}

}  // namespace ddbt::exec
