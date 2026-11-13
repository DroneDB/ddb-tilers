// SPDX-License-Identifier: MPL-2.0
#pragma once

// Arena: chunked bump allocator. Allocation is a pointer bump; nothing is freed individually;
// Reset() rewinds to the first chunk and reuses the chunks, Release() returns them to the OS.
// Not thread-safe: one arena per thread / per task.
//
// Destructors of objects placed in an arena are NOT run: New<T>() requires a trivially
// destructible T.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>
#include <span>
#include <type_traits>
#include <utility>
#include <vector>

namespace ddbt::mem {

class MemBudget;

struct ArenaOptions {
    /// Size of a regular chunk. Requests larger than chunkSize / 4 get a dedicated chunk.
    size_t chunkSize = size_t{256} << 10;
    /// Chunks of at least 2 MiB are mapped 2 MiB aligned with MADV_HUGEPAGE (off by default: to
    /// be turned on per use only where it measures faster).
    bool hugePages = false;
    /// Charged (ForceReserve) for every chunk held; released when chunks are freed.
    MemBudget* budget = nullptr;
};

class Arena {
public:
    Arena() : Arena(ArenaOptions{}) {}
    explicit Arena(ArenaOptions options);
    Arena(const Arena&) = delete;
    Arena& operator=(const Arena&) = delete;
    Arena(Arena&& other) noexcept;
    Arena& operator=(Arena&& other) noexcept;
    ~Arena();

    /// `size` bytes aligned to `alignment` (power of two). Never null; throws std::bad_alloc.
    void* Allocate(size_t size, size_t alignment = alignof(std::max_align_t)) {
        if (cur_ != nullptr) {
            const uintptr_t p = (reinterpret_cast<uintptr_t>(cur_) + (alignment - 1)) & ~(uintptr_t{alignment} - 1);
            const auto end = reinterpret_cast<uintptr_t>(end_);
            if (p <= end && size <= end - p) {
                cur_ = reinterpret_cast<uint8_t*>(p + size);
                used_ += size;
                return reinterpret_cast<void*>(p);
            }
        }
        return AllocateSlow(size, alignment);
    }

    /// Uninitialised array of n T (T trivially destructible).
    template <typename T>
    T* AllocateArray(size_t n) {
        static_assert(std::is_trivially_destructible_v<T>, "arena memory is never destructed");
        if (n > SIZE_MAX / sizeof(T)) throw std::bad_alloc();
        return static_cast<T*>(Allocate(n * sizeof(T), alignof(T)));
    }

    /// Constructs a T in the arena (T trivially destructible: no destructor will run).
    template <typename T, typename... Args>
    T* New(Args&&... args) {
        static_assert(std::is_trivially_destructible_v<T>, "arena memory is never destructed");
        return ::new (Allocate(sizeof(T), alignof(T))) T(std::forward<Args>(args)...);
    }

    /// Copy of `bytes` in the arena.
    std::span<uint8_t> Copy(std::span<const uint8_t> bytes) {
        auto* p = static_cast<uint8_t*>(Allocate(bytes.size(), 1));
        if (!bytes.empty()) std::memcpy(p, bytes.data(), bytes.size());
        return {p, bytes.size()};
    }

    /// Forgets every allocation; regular chunks are kept for reuse, dedicated ones are freed.
    void Reset() noexcept;
    /// Frees every chunk.
    void Release() noexcept;

    /// Bytes handed out since the last Reset (without alignment padding).
    size_t BytesUsed() const noexcept { return used_; }
    /// Bytes of chunks held (regular + dedicated).
    size_t BytesReserved() const noexcept { return reserved_; }
    size_t ChunkCount() const noexcept { return chunks_.size() + dedicated_.size(); }

private:
    struct Chunk {
        uint8_t* data;
        size_t size;
        bool large;  // LargeAlloc (mmap) rather than AlignedAlloc
    };

    void* AllocateSlow(size_t size, size_t alignment);
    Chunk NewChunk(size_t size);
    void FreeChunk(const Chunk& c) noexcept;

    ArenaOptions options_;
    std::vector<Chunk> chunks_;     // regular chunks, in use order
    std::vector<Chunk> dedicated_;  // oversized requests
    size_t current_ = 0;            // index of the chunk cur_ points into
    uint8_t* cur_ = nullptr;
    uint8_t* end_ = nullptr;
    size_t used_ = 0;
    size_t reserved_ = 0;
};

}  // namespace ddbt::mem
