// SPDX-License-Identifier: MPL-2.0
#include "core/mem/buffer_pool.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <mutex>
#include <vector>

#include "core/mem/aligned.h"
#include "core/mem/mem_budget.h"

namespace ddbt::mem {

namespace {

// Classes served by the thread slots: up to 8 MiB.
constexpr uint32_t kThreadClasses = 65;
constexpr uint32_t kMaxPerThreadClass = 16;

uint32_t ThreadSlotIndex() noexcept {
    static std::atomic<uint32_t> next{0};
    thread_local const uint32_t id = next.fetch_add(1, std::memory_order_relaxed);
    return id % BufferPool::kThreadSlots;
}

}  // namespace

uint32_t BufferPool::ClassOf(size_t size) noexcept {
    if (size <= kMinClassSize) return 0;
    if (size > kMaxPooledSize) return kDirectClass;
    const size_t s = size - 1;
    const auto k = static_cast<uint32_t>(std::bit_width(s) - 1);  // 2^k <= s < 2^(k+1), k >= 8
    const auto sub = static_cast<uint32_t>((s >> (k - 2)) & 3u);
    return (k - 8) * 4 + sub + 1;
}

size_t BufferPool::ClassSize(uint32_t cls) noexcept {
    if (cls == 0) return kMinClassSize;
    const uint32_t k = 8 + (cls - 1) / 4;
    const uint32_t sub = (cls - 1) % 4;
    return (size_t{1} << k) + size_t{sub + 1} * (size_t{1} << (k - 2));
}

static_assert(BufferPool::kClassCount == 73, "classes 256 B .. 64 MiB");

namespace detail {

struct BufferPoolCore {
    explicit BufferPoolCore(BufferPoolOptions o) : options(o) {
        for (uint32_t c = 0; c < kThreadClasses; ++c) {
            const size_t perClass = options.threadCacheBytesPerClass / BufferPool::ClassSize(c);
            threadCap[c] = static_cast<uint32_t>(std::clamp<size_t>(perClass, 1, kMaxPerThreadClass));
        }
    }

    struct alignas(64) SharedList {
        std::mutex mutex;
        std::vector<void*> items;
    };

    struct alignas(64) ThreadSlot {
        std::mutex mutex;
        std::array<uint32_t, kThreadClasses> count{};
        std::array<std::array<void*, kMaxPerThreadClass>, kThreadClasses> items{};
        uint64_t hits = 0;
        uint64_t acquires = 0;
        size_t bytes = 0;  // cached in this slot
    };

    void* AllocateFromOs(size_t bytes) {
        void* p = AlignedAlloc(bytes, BufferPool::kAlignment);
        osAllocations.fetch_add(1, std::memory_order_relaxed);
        bytesFromOs.fetch_add(bytes, std::memory_order_relaxed);
        if (options.budget != nullptr) options.budget->ForceReserve(bytes);
        return p;
    }

    void FreeToOs(void* p, size_t bytes) noexcept {
        AlignedFree(p);
        osFrees.fetch_add(1, std::memory_order_relaxed);
        bytesFromOs.fetch_sub(bytes, std::memory_order_relaxed);
        if (options.budget != nullptr) options.budget->Release(bytes);
    }

    // Takes up to `max` buffers of class `cls` from the shared list into `out`; returns count.
    uint32_t TakeShared(uint32_t cls, void** out, uint32_t max) noexcept {
        SharedList& l = shared[cls];
        std::lock_guard lock(l.mutex);
        uint32_t n = 0;
        while (n < max && !l.items.empty()) {
            out[n++] = l.items.back();
            l.items.pop_back();
        }
        if (n != 0) {
            cachedShared.fetch_sub(n * BufferPool::ClassSize(cls), std::memory_order_relaxed);
            sharedHits.fetch_add(1, std::memory_order_relaxed);
        }
        return n;
    }

    // Gives buffers of class `cls` to the shared list; those over the cap are freed.
    void PutShared(uint32_t cls, void* const* in, uint32_t n) {
        const size_t bytes = BufferPool::ClassSize(cls);
        SharedList& l = shared[cls];
        uint32_t i = 0;
        {
            std::lock_guard lock(l.mutex);
            l.items.reserve(l.items.size() + n);  // the only throwing step, before any change
            for (; i < n; ++i) {
                const uint64_t cached = cachedShared.load(std::memory_order_relaxed);
                if (cached + bytes > options.maxCachedBytes) break;
                l.items.push_back(in[i]);
                cachedShared.fetch_add(bytes, std::memory_order_relaxed);
            }
        }
        for (; i < n; ++i) FreeToOs(in[i], bytes);
    }

    void* Acquire(uint32_t cls) {
        const size_t bytes = BufferPool::ClassSize(cls);
        if (cls < kThreadClasses) {
            ThreadSlot& t = slots[ThreadSlotIndex()];
            std::lock_guard lock(t.mutex);
            ++t.acquires;
            uint32_t& cnt = t.count[cls];
            if (cnt == 0) {
                // Refill half the slot from the shared list in one lock.
                cnt = TakeShared(cls, t.items[cls].data(), std::max<uint32_t>(1, threadCap[cls] / 2));
                if (cnt == 0) return AllocateFromOs(bytes);
                t.bytes += size_t{cnt} * bytes;
            } else {
                ++t.hits;
            }
            t.bytes -= bytes;
            return t.items[cls][--cnt];
        }
        acquiresLarge.fetch_add(1, std::memory_order_relaxed);
        void* p = nullptr;
        if (TakeShared(cls, &p, 1) == 1) return p;
        return AllocateFromOs(bytes);
    }

    void Release(void* p, uint32_t cls) {
        if (cls < kThreadClasses) {
            ThreadSlot& t = slots[ThreadSlotIndex()];
            std::lock_guard lock(t.mutex);
            uint32_t& cnt = t.count[cls];
            const size_t bytes = BufferPool::ClassSize(cls);
            if (cnt >= threadCap[cls]) {
                // Spill the older half to the shared list.
                const uint32_t spill = std::max<uint32_t>(1, cnt / 2);
                PutShared(cls, t.items[cls].data(), spill);
                for (uint32_t i = spill; i < cnt; ++i) t.items[cls][i - spill] = t.items[cls][i];
                cnt -= spill;
                t.bytes -= size_t{spill} * bytes;
            }
            if (t.bytes + bytes > options.threadCacheBytesPerSlot) {
                PutShared(cls, &p, 1);  // slot full: share it
                return;
            }
            t.items[cls][cnt++] = p;
            t.bytes += bytes;
            return;
        }
        PutShared(cls, &p, 1);
    }

    void Trim() {
        for (uint32_t s = 0; s < BufferPool::kThreadSlots; ++s) {
            ThreadSlot& t = slots[s];
            std::lock_guard lock(t.mutex);
            for (uint32_t c = 0; c < kThreadClasses; ++c) {
                for (uint32_t i = 0; i < t.count[c]; ++i) FreeToOs(t.items[c][i], BufferPool::ClassSize(c));
                t.count[c] = 0;
            }
            t.bytes = 0;
        }
        for (uint32_t c = 0; c < BufferPool::kClassCount; ++c) {
            SharedList& l = shared[c];
            std::lock_guard lock(l.mutex);
            const size_t bytes = BufferPool::ClassSize(c);
            for (void* p : l.items) FreeToOs(p, bytes);
            cachedShared.fetch_sub(l.items.size() * bytes, std::memory_order_relaxed);
            l.items.clear();
            l.items.shrink_to_fit();
        }
    }

    BufferPoolOptions options;
    std::array<uint32_t, kThreadClasses> threadCap{};
    std::array<SharedList, BufferPool::kClassCount> shared;
    std::array<ThreadSlot, BufferPool::kThreadSlots> slots;
    alignas(64) std::atomic<uint64_t> osAllocations{0};
    std::atomic<uint64_t> osFrees{0};
    std::atomic<uint64_t> bytesFromOs{0};
    std::atomic<uint64_t> sharedHits{0};
    std::atomic<uint64_t> acquiresLarge{0};
    alignas(64) std::atomic<uint64_t> cachedShared{0};
};

}  // namespace detail

BufferPool::BufferPool(BufferPoolOptions options) : core_(std::make_unique<detail::BufferPoolCore>(options)) {}

BufferPool::~BufferPool() {
    if (core_) core_->Trim();
}

PooledBuffer BufferPool::Acquire(size_t size) {
    const uint32_t cls = ClassOf(size);
    if (cls == kDirectClass) {
        void* p = core_->AllocateFromOs(size);
        core_->acquiresLarge.fetch_add(1, std::memory_order_relaxed);
        return PooledBuffer(core_.get(), static_cast<uint8_t*>(p), size, size, cls);
    }
    void* p = core_->Acquire(cls);
    return PooledBuffer(core_.get(), static_cast<uint8_t*>(p), size, ClassSize(cls), cls);
}

void PooledBuffer::Reset() noexcept {
    if (data_ == nullptr) return;
    if (class_ == BufferPool::kDirectClass) {
        core_->FreeToOs(data_, capacity_);
    } else {
        // PutShared may need to grow a vector; if that throws, free the buffer instead.
        try {
            core_->Release(data_, class_);
        } catch (...) {
            core_->FreeToOs(data_, capacity_);
        }
    }
    core_ = nullptr;
    data_ = nullptr;
    size_ = capacity_ = 0;
    class_ = 0;
}

void BufferPool::Trim() { core_->Trim(); }

BufferPoolStats BufferPool::Stats() const {
    BufferPoolStats s;
    for (auto& t : core_->slots) {
        std::lock_guard lock(t.mutex);
        s.threadHits += t.hits;
        s.acquires += t.acquires;
    }
    s.acquires += core_->acquiresLarge.load(std::memory_order_relaxed);
    s.sharedHits = core_->sharedHits.load(std::memory_order_relaxed);
    s.osAllocations = core_->osAllocations.load(std::memory_order_relaxed);
    s.osFrees = core_->osFrees.load(std::memory_order_relaxed);
    s.bytesFromOs = core_->bytesFromOs.load(std::memory_order_relaxed);
    s.bytesCachedShared = core_->cachedShared.load(std::memory_order_relaxed);
    return s;
}

}  // namespace ddbt::mem
