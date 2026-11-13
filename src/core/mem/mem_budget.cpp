// SPDX-License-Identifier: MPL-2.0
#include "core/mem/mem_budget.h"

namespace ddbt::mem {

void MemBudget::UpdatePeak(uint64_t value) noexcept {
    uint64_t peak = peak_.load(std::memory_order_relaxed);
    while (value > peak && !peak_.compare_exchange_weak(peak, value, std::memory_order_relaxed)) {
    }
}

bool MemBudget::TryReserveLocal(uint64_t bytes) noexcept {
    const uint64_t cap = cap_.load(std::memory_order_relaxed);
    uint64_t cur = current_.load(std::memory_order_relaxed);
    for (;;) {
        if (bytes > cap || cur > cap - bytes) return false;
        if (current_.compare_exchange_weak(cur, cur + bytes, std::memory_order_relaxed)) break;
    }
    UpdatePeak(cur + bytes);
    return true;
}

bool MemBudget::TryReserve(uint64_t bytes) noexcept {
    if (!TryReserveLocal(bytes)) return false;
    if (parent_ != nullptr && !parent_->TryReserve(bytes)) {
        current_.fetch_sub(bytes, std::memory_order_relaxed);
        return false;
    }
    return true;
}

void MemBudget::ForceReserve(uint64_t bytes) noexcept {
    const uint64_t now = current_.fetch_add(bytes, std::memory_order_relaxed) + bytes;
    UpdatePeak(now);
    if (parent_ != nullptr) parent_->ForceReserve(bytes);
}

void MemBudget::Release(uint64_t bytes) noexcept {
    current_.fetch_sub(bytes, std::memory_order_relaxed);
    if (parent_ != nullptr) parent_->Release(bytes);
}

}  // namespace ddbt::mem
