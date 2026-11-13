// SPDX-License-Identifier: MPL-2.0
#pragma once

// Aligned and page-level allocation helpers.
//
// Allocation failure is not an "expected" error in the core: like operator new, these throw
// std::bad_alloc. Memory limits are enforced earlier, through MemBudget.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>

namespace ddbt::mem {

inline constexpr size_t kCacheLine = 64;
inline constexpr size_t kHugePageSize = size_t{2} << 20;  // x86-64 transparent huge page

/// Operating system page size (4096 on x86-64 Linux / Windows).
size_t PageSize() noexcept;

/// `size` bytes aligned to `alignment` (a power of two, any value >= 1). size 0 returns a valid
/// unique pointer. Throws std::bad_alloc. Free with AlignedFree.
void* AlignedAlloc(size_t size, size_t alignment = kCacheLine);
void AlignedFree(void* p) noexcept;

struct AlignedDeleter {
    void operator()(void* p) const noexcept { AlignedFree(p); }
};

/// Owning pointer to an aligned, uninitialised byte buffer.
using AlignedBytes = std::unique_ptr<uint8_t[], AlignedDeleter>;
inline AlignedBytes MakeAlignedBytes(size_t size, size_t alignment = kCacheLine) {
    return AlignedBytes(static_cast<uint8_t*>(AlignedAlloc(size, alignment)));
}

/// Large zero-filled region straight from the OS (mmap / VirtualAlloc), page aligned. With
/// `hugePages` on Linux the region is also 2 MiB aligned and advised MADV_HUGEPAGE (THP backs
/// only aligned 2 MiB extents). On Windows large pages need SeLockMemoryPrivilege, so the flag
/// is ignored there: the region is only page aligned. Throws std::bad_alloc. Free with
/// LargeFree using the same size and flag.
void* LargeAlloc(size_t size, bool hugePages = false);
void LargeFree(void* p, size_t size, bool hugePages = false) noexcept;

/// Asks the kernel to back [p, p+size) with transparent huge pages. False when unsupported.
bool AdviseHugePages(void* p, size_t size) noexcept;

/// Rounds up to a multiple of `alignment` (a power of two). No overflow check: for sizes that
/// are known to be far below SIZE_MAX.
constexpr size_t AlignUp(size_t v, size_t alignment) noexcept { return (v + alignment - 1) & ~(alignment - 1); }

}  // namespace ddbt::mem
