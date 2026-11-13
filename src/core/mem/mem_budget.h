// SPDX-License-Identifier: MPL-2.0
#pragma once

// MemBudget: a byte counter with a cap, shared by the caches and builders of one job.
//
//   MemBudget process(4ull << 30);            // 4 GiB for everything
//   MemBudget cache(1ull << 30, &process);    // 1 GiB of it for a cache
//   if (cache.TryReserve(n)) { ...; cache.Release(n); }
//
// A child budget charges its parent too (TryReserve succeeds only if both have room). All
// operations are lock-free.

#include <atomic>
#include <cstdint>
#include <limits>
#include <utility>

namespace ddbt::mem {

class MemBudget {
public:
    static constexpr uint64_t kUnlimited = std::numeric_limits<uint64_t>::max();

    explicit MemBudget(uint64_t cap = kUnlimited, MemBudget* parent = nullptr) noexcept
        : cap_(cap), parent_(parent) {}
    MemBudget(const MemBudget&) = delete;
    MemBudget& operator=(const MemBudget&) = delete;

    /// Reserves `bytes` if current + bytes <= cap (here and in every ancestor); false otherwise
    /// (nothing reserved).
    [[nodiscard]] bool TryReserve(uint64_t bytes) noexcept;

    /// Reserves unconditionally (may go over the cap): for memory that is already allocated or
    /// cannot be refused. Over-cap state is visible through Current() > Cap().
    void ForceReserve(uint64_t bytes) noexcept;

    /// Returns bytes reserved by TryReserve / ForceReserve.
    void Release(uint64_t bytes) noexcept;

    uint64_t Current() const noexcept { return current_.load(std::memory_order_relaxed); }
    uint64_t Peak() const noexcept { return peak_.load(std::memory_order_relaxed); }
    uint64_t Cap() const noexcept { return cap_.load(std::memory_order_relaxed); }
    /// Bytes that TryReserve could still take here (ignores the parents).
    uint64_t Available() const noexcept {
        const uint64_t cap = Cap();
        const uint64_t cur = Current();
        return cur >= cap ? 0 : cap - cur;
    }
    MemBudget* Parent() const noexcept { return parent_; }

    /// Changes the cap; existing reservations are kept even if now above it.
    void SetCap(uint64_t cap) noexcept { cap_.store(cap, std::memory_order_relaxed); }
    /// Peak := Current (start of a new measurement).
    void ResetPeak() noexcept { peak_.store(Current(), std::memory_order_relaxed); }

private:
    bool TryReserveLocal(uint64_t bytes) noexcept;
    void UpdatePeak(uint64_t value) noexcept;

    std::atomic<uint64_t> current_{0};
    std::atomic<uint64_t> peak_{0};
    std::atomic<uint64_t> cap_;
    MemBudget* const parent_;
};

/// RAII reservation: releases its bytes on destruction. Move-only.
class BudgetReservation {
public:
    BudgetReservation() noexcept = default;
    /// Tries to reserve; check ok() afterwards.
    static BudgetReservation Try(MemBudget& budget, uint64_t bytes) noexcept {
        if (!budget.TryReserve(bytes)) return {};
        return BudgetReservation(&budget, bytes);
    }
    static BudgetReservation Force(MemBudget& budget, uint64_t bytes) noexcept {
        budget.ForceReserve(bytes);
        return BudgetReservation(&budget, bytes);
    }
    BudgetReservation(BudgetReservation&& o) noexcept
        : budget_(std::exchange(o.budget_, nullptr)), bytes_(std::exchange(o.bytes_, 0)) {}
    BudgetReservation& operator=(BudgetReservation&& o) noexcept {
        if (this != &o) {
            Reset();
            budget_ = std::exchange(o.budget_, nullptr);
            bytes_ = std::exchange(o.bytes_, 0);
        }
        return *this;
    }
    BudgetReservation(const BudgetReservation&) = delete;
    BudgetReservation& operator=(const BudgetReservation&) = delete;
    ~BudgetReservation() { Reset(); }

    bool ok() const noexcept { return budget_ != nullptr; }
    uint64_t bytes() const noexcept { return bytes_; }
    void Reset() noexcept {
        if (budget_ != nullptr) budget_->Release(bytes_);
        budget_ = nullptr;
        bytes_ = 0;
    }

private:
    BudgetReservation(MemBudget* b, uint64_t n) noexcept : budget_(b), bytes_(n) {}
    MemBudget* budget_ = nullptr;
    uint64_t bytes_ = 0;
};

}  // namespace ddbt::mem
