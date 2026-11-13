// SPDX-License-Identifier: MPL-2.0
#pragma once

// File: read-only file with positional reads (pread / ReadFile + OVERLAPPED). There is no file
// position, so one File is safely shared by any number of threads. Paths are UTF-8 on every OS.

#include <cstdint>
#include <span>
#include <string>
#include <string_view>

#include "core/io/platform.h"
#include "core/util/status.h"

namespace ddbt::io {

/// Access pattern hint given to the OS at open time (posix_fadvise / FILE_FLAG_*).
enum class AccessHint : uint8_t { kNormal, kRandom, kSequential };

class File {
public:
    /// Opens `utf8Path` for reading. NotFound / IoError on failure.
    static Expected<File> Open(std::string_view utf8Path, AccessHint hint = AccessHint::kNormal);

    File() noexcept = default;
    File(File&& other) noexcept;
    File& operator=(File&& other) noexcept;
    File(const File&) = delete;
    File& operator=(const File&) = delete;
    ~File();

    bool IsOpen() const noexcept { return handle_ != detail::kInvalidHandle; }
    /// Size when opened (inputs are treated as immutable).
    uint64_t Size() const noexcept { return size_; }
    const std::string& Path() const noexcept { return path_; }
    detail::NativeHandle NativeHandle() const noexcept { return handle_; }

    /// Reads exactly out.size() bytes at `offset`; OutOfRange if the file is shorter.
    Status ReadAt(uint64_t offset, std::span<uint8_t> out) const;

    /// Reads up to out.size() bytes (fewer only at end of file); returns the count.
    Expected<size_t> ReadAtMost(uint64_t offset, std::span<uint8_t> out) const;

    /// Asks the OS to start reading [offset, offset + length) into the page cache
    /// (posix_fadvise WILLNEED; no-op on Windows, where the cache manager reads ahead itself).
    void Prefetch(uint64_t offset, uint64_t length) const noexcept;

    /// Drops the file's clean pages from the page cache (cold-cache benchmarks). posix_fadvise
    /// DONTNEED where it exists; elsewhere (Windows) a no-op that succeeds: a hint, like Prefetch.
    Status DropCache() const;

    void Close() noexcept;

private:
    bool RangeInsideFile(uint64_t offset, uint64_t length) const noexcept;

    detail::NativeHandle handle_ = detail::kInvalidHandle;
    uint64_t size_ = 0;
    std::string path_;
};

}  // namespace ddbt::io
