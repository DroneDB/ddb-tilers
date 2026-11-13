// SPDX-License-Identifier: MPL-2.0
#include "core/io/mapped_file.h"

#include <algorithm>
#include <utility>

#include "core/io/io_stats.h"
#include "core/mem/aligned.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>

#include <cerrno>
#endif

namespace ddbt::io {

MappedFile::MappedFile(MappedFile&& other) noexcept
    : data_(std::exchange(other.data_, nullptr)),
      size_(std::exchange(other.size_, 0)),
      opened_(std::exchange(other.opened_, false)),
      path_(std::move(other.path_))
#ifdef _WIN32
      ,
      mapping_(std::exchange(other.mapping_, nullptr))
#endif
{
}

MappedFile& MappedFile::operator=(MappedFile&& other) noexcept {
    if (this != &other) {
        Close();
        data_ = std::exchange(other.data_, nullptr);
        size_ = std::exchange(other.size_, 0);
        opened_ = std::exchange(other.opened_, false);
        path_ = std::move(other.path_);
#ifdef _WIN32
        mapping_ = std::exchange(other.mapping_, nullptr);
#endif
    }
    return *this;
}

MappedFile::~MappedFile() { Close(); }

void MappedFile::Close() noexcept {
#ifdef _WIN32
    if (data_ != nullptr) UnmapViewOfFile(data_);
    if (mapping_ != nullptr) ::CloseHandle(mapping_);
    mapping_ = nullptr;
#else
    if (data_ != nullptr) ::munmap(const_cast<uint8_t*>(data_), static_cast<size_t>(size_));
#endif
    data_ = nullptr;
    size_ = 0;
    opened_ = false;
}

Expected<MappedFile> MappedFile::Open(std::string_view utf8Path, AccessHint hint) {
    // The mapping keeps its own reference to the file: the descriptor is closed right after.
    DDBT_ASSIGN_OR_RETURN(File file, File::Open(utf8Path, hint));
    MappedFile m;
    m.path_ = std::string(utf8Path);
    m.size_ = file.Size();
    m.opened_ = true;
    if (m.size_ == 0) return m;
    if (m.size_ > static_cast<uint64_t>(SIZE_MAX)) return Unsupported("file too large to map: '" + m.path_ + "'");
#ifdef _WIN32
    HANDLE mapping = CreateFileMappingW(file.NativeHandle(), nullptr, PAGE_READONLY, 0, 0, nullptr);
    if (mapping == nullptr) return Win32Status("CreateFileMapping '" + m.path_ + "'", GetLastError());
    void* p = MapViewOfFile(mapping, FILE_MAP_READ, 0, 0, 0);
    if (p == nullptr) {
        const DWORD err = GetLastError();
        ::CloseHandle(mapping);
        return Win32Status("MapViewOfFile '" + m.path_ + "'", err);
    }
    m.mapping_ = mapping;
    m.data_ = static_cast<const uint8_t*>(p);
#else
    void* p = ::mmap(nullptr, static_cast<size_t>(m.size_), PROT_READ, MAP_SHARED, file.NativeHandle(), 0);
    if (p == MAP_FAILED) return ErrnoStatus("mmap '" + m.path_ + "'", errno);
    m.data_ = static_cast<const uint8_t*>(p);
    if (hint == AccessHint::kRandom) m.Advise(MapAdvice::kRandom);
    if (hint == AccessHint::kSequential) m.Advise(MapAdvice::kSequential);
#endif
    return m;
}

void MappedFile::Advise(MapAdvice advice, uint64_t offset, uint64_t length) const noexcept {
    if (data_ == nullptr || offset >= size_ || length == 0) return;
    length = std::min(length, size_ - offset);
    const uint64_t page = mem::PageSize();
    const uint64_t begin = offset & ~(page - 1);
    const uint64_t end = std::min<uint64_t>(size_, offset + length);
    void* addr = const_cast<uint8_t*>(data_ + begin);
    const auto len = static_cast<size_t>(end - begin);
#ifdef _WIN32
    if (advice == MapAdvice::kWillNeed) {
        WIN32_MEMORY_RANGE_ENTRY range{addr, len};
        PrefetchVirtualMemory(GetCurrentProcess(), 1, &range, 0);
        IoStats::AddPrefetch();
    }
#else
    int a = MADV_NORMAL;
    switch (advice) {
        case MapAdvice::kNormal: a = MADV_NORMAL; break;
        case MapAdvice::kRandom: a = MADV_RANDOM; break;
        case MapAdvice::kSequential: a = MADV_SEQUENTIAL; break;
        case MapAdvice::kWillNeed: a = MADV_WILLNEED; break;
        case MapAdvice::kDontNeed: a = MADV_DONTNEED; break;
    }
    ::madvise(addr, len, a);
    if (advice == MapAdvice::kWillNeed) IoStats::AddPrefetch();
#endif
}

}  // namespace ddbt::io
