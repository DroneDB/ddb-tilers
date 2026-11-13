// SPDX-License-Identifier: MPL-2.0
#pragma once

// BufferPool: recycles 64-byte aligned byte buffers (decode targets, compressed blocks, I/O
// staging) by size class.
//
//   BufferPool pool;
//   PooledBuffer buf = pool.Acquire(n);   // capacity >= n, contents uninitialised
//   ...                                   // returned to the pool when buf is destroyed
//
// Size classes: 4 per power of two from 256 B to 64 MiB (at most 25% slack); larger requests
// are plain aligned allocations. A buffer is returned first to the calling thread's cache
// (classes up to 8 MiB, at most threadCacheBytesPerSlot bytes per thread), which spills half
// of a class to a per-class global list when full and refills from it when empty. The global
// lists hold at most maxCachedBytes.
//
// Thread caches are owned by the pool: 64 slots, a thread uses slot (thread number % 64), each
// guarded by its own (uncontended) mutex on its own cache lines. Unlike C++ thread_local
// storage this keeps every buffer owned by the pool, so destroying a pool frees everything and
// never races with exiting threads.
//
// Lifetime: the pool must outlive its PooledBuffers.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <utility>

namespace ddbt::mem {

class MemBudget;
class BufferPool;

namespace detail {
struct BufferPoolCore;
}

/// RAII handle to a pooled buffer. Move-only; empty after a move or Reset().
class PooledBuffer {
public:
    PooledBuffer() noexcept = default;
    PooledBuffer(PooledBuffer&& o) noexcept
        : core_(std::exchange(o.core_, nullptr)),
          data_(std::exchange(o.data_, nullptr)),
          size_(std::exchange(o.size_, 0)),
          capacity_(std::exchange(o.capacity_, 0)),
          class_(std::exchange(o.class_, 0)) {}
    PooledBuffer& operator=(PooledBuffer&& o) noexcept {
        if (this != &o) {
            Reset();
            core_ = std::exchange(o.core_, nullptr);
            data_ = std::exchange(o.data_, nullptr);
            size_ = std::exchange(o.size_, 0);
            capacity_ = std::exchange(o.capacity_, 0);
            class_ = std::exchange(o.class_, 0);
        }
        return *this;
    }
    PooledBuffer(const PooledBuffer&) = delete;
    PooledBuffer& operator=(const PooledBuffer&) = delete;
    ~PooledBuffer() { Reset(); }

    uint8_t* data() const noexcept { return data_; }
    /// Requested size.
    size_t size() const noexcept { return size_; }
    /// Usable size (size class).
    size_t capacity() const noexcept { return capacity_; }
    std::span<uint8_t> span() const noexcept { return {data_, size_}; }
    bool empty() const noexcept { return data_ == nullptr; }
    explicit operator bool() const noexcept { return data_ != nullptr; }

    /// Changes the logical size within the capacity (no reallocation); false if too large.
    bool Resize(size_t size) noexcept {
        if (size > capacity()) return false;
        size_ = size;
        return true;
    }

    /// Returns the buffer to its pool now.
    void Reset() noexcept;

private:
    friend class BufferPool;
    PooledBuffer(detail::BufferPoolCore* core, uint8_t* data, size_t size, size_t capacity, uint32_t cls) noexcept
        : core_(core), data_(data), size_(size), capacity_(capacity), class_(cls) {}

    detail::BufferPoolCore* core_ = nullptr;
    uint8_t* data_ = nullptr;
    size_t size_ = 0;
    size_t capacity_ = 0;
    uint32_t class_ = 0;
};

struct BufferPoolOptions {
    /// Cap on bytes kept in the global lists (excess buffers are freed).
    size_t maxCachedBytes = size_t{256} << 20;
    /// Per thread and size class, the thread cache holds up to this many bytes (at least one
    /// buffer, at most 16), for classes up to 8 MiB...
    size_t threadCacheBytesPerClass = size_t{1} << 20;
    /// ...and at most this many bytes in all (per thread slot).
    size_t threadCacheBytesPerSlot = size_t{32} << 20;
    /// Charged (ForceReserve) for every byte obtained from the OS, released when freed.
    MemBudget* budget = nullptr;
};

struct BufferPoolStats {
    uint64_t acquires = 0;       // Acquire calls
    uint64_t threadHits = 0;     // served from the calling thread's cache slot
    uint64_t sharedHits = 0;     // served from a global list
    uint64_t osAllocations = 0;  // new buffers from the OS
    uint64_t osFrees = 0;
    uint64_t bytesFromOs = 0;    // live bytes obtained from the OS (in use + cached)
    uint64_t bytesCachedShared = 0;
};

class BufferPool {
public:
    static constexpr size_t kMinClassSize = 256;
    static constexpr size_t kMaxPooledSize = size_t{64} << 20;
    static constexpr size_t kMaxThreadCachedSize = size_t{8} << 20;
    static constexpr size_t kAlignment = 64;

    BufferPool() : BufferPool(BufferPoolOptions{}) {}
    explicit BufferPool(BufferPoolOptions options);
    BufferPool(const BufferPool&) = delete;
    BufferPool& operator=(const BufferPool&) = delete;
    ~BufferPool();

    /// Buffer of at least `size` bytes, 64-byte aligned, uninitialised. Throws std::bad_alloc.
    PooledBuffer Acquire(size_t size);

    /// Frees every cached buffer (thread slots and global lists).
    void Trim();

    BufferPoolStats Stats() const;

    /// Size-class helpers (exposed for tests): class index of a size (> kMaxPooledSize maps to
    /// kDirectClass) and the byte size of a class.
    static uint32_t ClassOf(size_t size) noexcept;
    static size_t ClassSize(uint32_t cls) noexcept;
    static constexpr uint32_t kClassCount = 73;
    static constexpr uint32_t kDirectClass = 0xFFFFFFFFu;

    static constexpr uint32_t kThreadSlots = 64;

private:
    std::unique_ptr<detail::BufferPoolCore> core_;
};

}  // namespace ddbt::mem
