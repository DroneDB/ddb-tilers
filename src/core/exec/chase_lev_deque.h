// SPDX-License-Identifier: MPL-2.0
#pragma once

// Chase-Lev work-stealing deque (Chase & Lev, SPAA 2005), in the C11 formulation of Lê, Pop,
// Cohen and Zappa Nardelli ("Correct and efficient work-stealing for weak memory models",
// PPoPP 2013), using seq_cst operations where the paper uses fences (ThreadSanitizer does not
// model stand-alone fences; on x86 the generated code is the same).
//
// The owner thread pushes and pops at the bottom (LIFO, cache-hot); any thread steals from the
// top (FIFO, oldest = largest work in divide-and-conquer). The ring grows when full; old rings
// are kept until the deque is destroyed, because a thief may still be reading one.
//
// Each slot carries a 1-bit tag next to the pointer (the pool stores the task priority there),
// so the owner can look at its bottom item and a thief at the top item without dereferencing a
// pointer that another thread may already have taken and freed (PopIf, StealIf).

#include <atomic>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <vector>

namespace ddbt::exec::detail {

template <typename T>
class ChaseLevDeque {
public:
    explicit ChaseLevDeque(int64_t initialCapacity = 1024) {
        int64_t cap = 16;
        while (cap < initialCapacity) cap *= 2;
        rings_.push_back(std::make_unique<Ring>(cap));
        ring_.store(rings_.back().get(), std::memory_order_relaxed);
    }
    ChaseLevDeque(const ChaseLevDeque&) = delete;
    ChaseLevDeque& operator=(const ChaseLevDeque&) = delete;

    static_assert(alignof(T) >= 2, "the low pointer bit holds the slot tag");

    /// Owner only. `tag` is 0 or 1. May throw std::bad_alloc when the ring has to grow (nothing
    /// is pushed then).
    void Push(T* item, unsigned tag = 0) {
        const int64_t b = bottom_.load(std::memory_order_relaxed);
        const int64_t t = top_.load(std::memory_order_acquire);
        Ring* r = ring_.load(std::memory_order_relaxed);
        if (b - t > r->mask) r = Grow(r, b, t);
        r->Put(b, reinterpret_cast<uintptr_t>(item) | (tag & kTagMask));
        bottom_.store(b + 1, std::memory_order_release);
    }

    /// Owner only: newest item, or nullptr when empty (or the last item was stolen meanwhile).
    T* Pop() noexcept {
        const int64_t b = bottom_.load(std::memory_order_relaxed) - 1;
        Ring* r = ring_.load(std::memory_order_relaxed);
        bottom_.store(b, std::memory_order_seq_cst);
        int64_t t = top_.load(std::memory_order_seq_cst);
        if (t <= b) {
            T* item = Untag(r->Get(b));
            if (t == b) {
                // Last item: race with thieves for it.
                if (!top_.compare_exchange_strong(t, t + 1, std::memory_order_seq_cst, std::memory_order_relaxed))
                    item = nullptr;
                bottom_.store(b + 1, std::memory_order_relaxed);
            }
            return item;
        }
        bottom_.store(b + 1, std::memory_order_relaxed);
        return nullptr;
    }

    /// Owner only: like Pop, but only items at index >= floor (pushed after BottomIndex()
    /// returned `floor`).
    T* PopAbove(int64_t floor) noexcept {
        if (bottom_.load(std::memory_order_relaxed) <= floor) return nullptr;
        return Pop();
    }

    /// Owner only: tag of the newest item, or -1 when the deque looks empty. The item may be
    /// stolen right after; the tag of a slot never changes while the item is in the deque.
    int BottomTag() const noexcept {
        const int64_t b = bottom_.load(std::memory_order_relaxed) - 1;
        const int64_t t = top_.load(std::memory_order_acquire);
        if (t > b) return -1;
        return static_cast<int>(ring_.load(std::memory_order_relaxed)->Get(b) & kTagMask);
    }

    /// Any thread: tag of the oldest item, or -1 when the deque looks empty (a snapshot).
    int TopTag() const noexcept {
        const int64_t t = top_.load(std::memory_order_seq_cst);
        const int64_t b = bottom_.load(std::memory_order_seq_cst);
        if (t >= b) return -1;
        return static_cast<int>(ring_.load(std::memory_order_acquire)->Get(t) & kTagMask);
    }

    /// Owner only: Pop, but only when the newest item's tag satisfies `accept(tag)`.
    template <typename Accept>
    T* PopIf(Accept&& accept) noexcept {
        const int tag = BottomTag();
        if (tag < 0 || !accept(static_cast<unsigned>(tag))) return nullptr;
        return Pop();
    }

    /// Owner only: index the next Push will use.
    int64_t BottomIndex() const noexcept { return bottom_.load(std::memory_order_relaxed); }

    /// Any thread: oldest item, or nullptr when empty or when it lost a race (caller may retry).
    T* Steal() noexcept {
        return StealIf([](unsigned) { return true; });
    }

    /// Any thread: Steal, but only when the oldest item's tag satisfies `accept(tag)` (the item
    /// itself is not touched before it is taken).
    template <typename Accept>
    T* StealIf(Accept&& accept) noexcept {
        int64_t t = top_.load(std::memory_order_seq_cst);
        const int64_t b = bottom_.load(std::memory_order_seq_cst);
        if (t >= b) return nullptr;
        Ring* r = ring_.load(std::memory_order_acquire);
        const uintptr_t v = r->Get(t);
        if (!accept(static_cast<unsigned>(v & kTagMask))) return nullptr;
        if (!top_.compare_exchange_strong(t, t + 1, std::memory_order_seq_cst, std::memory_order_relaxed)) return nullptr;
        return Untag(v);
    }

    /// Snapshot emptiness test (any thread). seq_cst so that a sleeping worker's re-check is
    /// ordered after its sleeper registration.
    bool Empty() const noexcept {
        const int64_t t = top_.load(std::memory_order_seq_cst);
        const int64_t b = bottom_.load(std::memory_order_seq_cst);
        return t >= b;
    }

    int64_t SizeApprox() const noexcept {
        const int64_t b = bottom_.load(std::memory_order_relaxed);
        const int64_t t = top_.load(std::memory_order_relaxed);
        return b > t ? b - t : 0;
    }

private:
    static constexpr uintptr_t kTagMask = 1;
    static T* Untag(uintptr_t v) noexcept { return reinterpret_cast<T*>(v & ~kTagMask); }

    struct Ring {
        explicit Ring(int64_t cap) : mask(cap - 1), slots(new std::atomic<uintptr_t>[static_cast<size_t>(cap)]) {}
        uintptr_t Get(int64_t i) const noexcept {
            return slots[static_cast<size_t>(i & mask)].load(std::memory_order_relaxed);
        }
        void Put(int64_t i, uintptr_t v) noexcept {
            slots[static_cast<size_t>(i & mask)].store(v, std::memory_order_relaxed);
        }
        const int64_t mask;
        std::unique_ptr<std::atomic<uintptr_t>[]> slots;
    };

    Ring* Grow(Ring* old, int64_t b, int64_t t) {
        rings_.reserve(rings_.size() + 1);
        auto bigger = std::make_unique<Ring>((old->mask + 1) * 2);
        for (int64_t i = t; i < b; ++i) bigger->Put(i, old->Get(i));
        Ring* r = bigger.get();
        rings_.push_back(std::move(bigger));
        ring_.store(r, std::memory_order_release);
        return r;
    }

    alignas(64) std::atomic<int64_t> top_{0};
    alignas(64) std::atomic<int64_t> bottom_{0};
    alignas(64) std::atomic<Ring*> ring_{nullptr};
    std::vector<std::unique_ptr<Ring>> rings_;  // owner only; every ring ever used
};

}  // namespace ddbt::exec::detail
