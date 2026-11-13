// SPDX-License-Identifier: MPL-2.0
#include "core/mem/aligned.h"

#include <cstdlib>

#ifdef _WIN32
#include <malloc.h>
#include <windows.h>
#else
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace ddbt::mem {

size_t PageSize() noexcept {
#ifdef _WIN32
    static const size_t page = [] {
        SYSTEM_INFO si;
        GetSystemInfo(&si);
        return static_cast<size_t>(si.dwPageSize);
    }();
#else
    static const size_t page = [] {
        const long v = sysconf(_SC_PAGESIZE);
        return v > 0 ? static_cast<size_t>(v) : size_t{4096};
    }();
#endif
    return page;
}

void* AlignedAlloc(size_t size, size_t alignment) {
    if (alignment == 0 || (alignment & (alignment - 1)) != 0) throw std::bad_alloc();
    if (alignment < sizeof(void*)) alignment = sizeof(void*);
    if (size == 0) size = 1;
#ifdef _WIN32
    void* p = _aligned_malloc(size, alignment);
    if (p == nullptr) throw std::bad_alloc();
    return p;
#else
    void* p = nullptr;
    if (posix_memalign(&p, alignment, size) != 0 || p == nullptr) throw std::bad_alloc();
    return p;
#endif
}

void AlignedFree(void* p) noexcept {
#ifdef _WIN32
    _aligned_free(p);
#else
    std::free(p);
#endif
}

bool AdviseHugePages(void* p, size_t size) noexcept {
#if defined(__linux__) && defined(MADV_HUGEPAGE)
    return p != nullptr && madvise(p, size, MADV_HUGEPAGE) == 0;
#else
    (void)p;
    (void)size;
    return false;
#endif
}

#ifdef _WIN32

void* LargeAlloc(size_t size, bool /*hugePages*/) {
    if (size == 0) size = 1;
    void* p = VirtualAlloc(nullptr, size, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
    if (p == nullptr) throw std::bad_alloc();
    return p;
}

void LargeFree(void* p, size_t /*size*/, bool /*hugePages*/) noexcept {
    if (p != nullptr) VirtualFree(p, 0, MEM_RELEASE);
}

#else

// With hugePages the mapping is over-allocated by 2 MiB and trimmed so the region starts on a
// 2 MiB boundary (THP only backs aligned 2 MiB extents). LargeFree recomputes the same rounded
// length, so callers pass the size and flag they allocated with.
namespace {
size_t MappedSize(size_t size, bool huge) {
    const size_t unit = huge ? kHugePageSize : PageSize();
    return AlignUp(size == 0 ? 1 : size, unit);
}
}  // namespace

void* LargeAlloc(size_t size, bool hugePages) {
    const size_t len = MappedSize(size, hugePages);
    if (!hugePages) {
        void* p = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
        if (p == MAP_FAILED) throw std::bad_alloc();
        return p;
    }
    const size_t over = len + kHugePageSize;
    void* raw = mmap(nullptr, over, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (raw == MAP_FAILED) throw std::bad_alloc();
    const auto base = reinterpret_cast<uintptr_t>(raw);
    const uintptr_t aligned = (base + kHugePageSize - 1) & ~(uintptr_t{kHugePageSize} - 1);
    const size_t head = aligned - base;
    const size_t tail = over - head - len;
    if (head != 0) munmap(raw, head);
    if (tail != 0) munmap(reinterpret_cast<void*>(aligned + len), tail);
    void* p = reinterpret_cast<void*>(aligned);
    AdviseHugePages(p, len);
    return p;
}

void LargeFree(void* p, size_t size, bool hugePages) noexcept {
    if (p != nullptr) munmap(p, MappedSize(size, hugePages));
}

#endif

}  // namespace ddbt::mem
