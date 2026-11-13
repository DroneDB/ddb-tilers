// SPDX-License-Identifier: MPL-2.0
#pragma once

// MappedFile: whole file mapped read-only (mmap / CreateFileMapping + MapViewOfFile).
//
// The file must not be truncated while mapped: touching a page past the new end raises SIGBUS
// on Linux (an access violation on Windows). Inputs are treated as immutable.

#include <cstdint>
#include <span>
#include <string>
#include <string_view>

#include "core/io/file.h"
#include "core/util/status.h"

namespace ddbt::io {

enum class MapAdvice : uint8_t { kNormal, kRandom, kSequential, kWillNeed, kDontNeed };

class MappedFile {
public:
    /// Maps `utf8Path`. An empty file maps to data() == nullptr, size() == 0.
    static Expected<MappedFile> Open(std::string_view utf8Path, AccessHint hint = AccessHint::kNormal);

    MappedFile() noexcept = default;
    MappedFile(MappedFile&& other) noexcept;
    MappedFile& operator=(MappedFile&& other) noexcept;
    MappedFile(const MappedFile&) = delete;
    MappedFile& operator=(const MappedFile&) = delete;
    ~MappedFile();

    const uint8_t* data() const noexcept { return data_; }
    uint64_t size() const noexcept { return size_; }
    std::span<const uint8_t> bytes() const noexcept { return {data_, static_cast<size_t>(size_)}; }
    const std::string& Path() const noexcept { return path_; }
    bool IsOpen() const noexcept { return data_ != nullptr || opened_; }

    /// madvise on [offset, offset + length) (clamped to the file, widened to whole pages).
    /// Windows: kWillNeed -> PrefetchVirtualMemory, others are no-ops.
    void Advise(MapAdvice advice, uint64_t offset = 0, uint64_t length = UINT64_MAX) const noexcept;

    /// Advise(kWillNeed) on a range.
    void Prefetch(uint64_t offset, uint64_t length) const noexcept { Advise(MapAdvice::kWillNeed, offset, length); }

    void Close() noexcept;

private:
    const uint8_t* data_ = nullptr;
    uint64_t size_ = 0;
    bool opened_ = false;
    std::string path_;
#ifdef _WIN32
    void* mapping_ = nullptr;  // HANDLE of the file mapping object
#endif
};

}  // namespace ddbt::io
