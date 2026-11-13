// SPDX-License-Identifier: MPL-2.0
#include "core/io/file.h"

#include <utility>

#include "core/io/io_stats.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#endif

namespace ddbt::io {

File::File(File&& other) noexcept
    : handle_(std::exchange(other.handle_, detail::kInvalidHandle)),
      size_(std::exchange(other.size_, 0)),
      path_(std::move(other.path_)) {}

File& File::operator=(File&& other) noexcept {
    if (this != &other) {
        Close();
        handle_ = std::exchange(other.handle_, detail::kInvalidHandle);
        size_ = std::exchange(other.size_, 0);
        path_ = std::move(other.path_);
    }
    return *this;
}

File::~File() { Close(); }

void File::Close() noexcept {
    if (IsOpen()) detail::CloseHandle(handle_);
    handle_ = detail::kInvalidHandle;
    size_ = 0;
}

Expected<File> File::Open(std::string_view utf8Path, AccessHint hint) {
    File f;
    f.path_ = std::string(utf8Path);
#ifdef _WIN32
    DDBT_ASSIGN_OR_RETURN(std::wstring wide, detail::ToWidePath(utf8Path));
    DWORD flags = FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED;
    if (hint == AccessHint::kRandom) flags |= FILE_FLAG_RANDOM_ACCESS;
    if (hint == AccessHint::kSequential) flags |= FILE_FLAG_SEQUENTIAL_SCAN;
    HANDLE h = CreateFileW(wide.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                           OPEN_EXISTING, flags, nullptr);
    if (h == INVALID_HANDLE_VALUE) return Win32Status("open '" + f.path_ + "'", GetLastError());
    f.handle_ = h;
#else
    int fd;
    do {
        fd = ::open(f.path_.c_str(), O_RDONLY | O_CLOEXEC);
    } while (fd < 0 && errno == EINTR);
    if (fd < 0) return ErrnoStatus("open '" + f.path_ + "'", errno);
    f.handle_ = fd;
#if defined(POSIX_FADV_RANDOM)
    if (hint == AccessHint::kRandom) ::posix_fadvise(fd, 0, 0, POSIX_FADV_RANDOM);
    if (hint == AccessHint::kSequential) ::posix_fadvise(fd, 0, 0, POSIX_FADV_SEQUENTIAL);
#else
    (void)hint;
#endif
#endif
    DDBT_ASSIGN_OR_RETURN(f.size_, detail::HandleSize(f.handle_, f.path_));
    IoStats::AddOpen();
    return f;
}

Status File::ReadAt(uint64_t offset, std::span<uint8_t> out) const {
    if (!RangeInsideFile(offset, out.size())) {
        return OutOfRange("read of " + std::to_string(out.size()) + " bytes at " + std::to_string(offset) +
                          " past the end of '" + path_ + "' (" + std::to_string(size_) + " bytes)");
    }
    size_t got = 0;
    DDBT_TRY(detail::ReadAt(handle_, offset, out.data(), out.size(), &got, path_));
    if (got != out.size()) {
        return OutOfRange("short read of '" + path_ + "' at " + std::to_string(offset) + ": " + std::to_string(got) +
                          " of " + std::to_string(out.size()) + " bytes (file truncated?)");
    }
    return Status::Ok();
}

Expected<size_t> File::ReadAtMost(uint64_t offset, std::span<uint8_t> out) const {
    if (!IsOpen()) return InvalidArgument("read from a closed File");
    size_t got = 0;
    DDBT_TRY(detail::ReadAt(handle_, offset, out.data(), out.size(), &got, path_));
    return got;
}

bool File::RangeInsideFile(uint64_t offset, uint64_t length) const noexcept {
    return IsOpen() && offset <= size_ && length <= size_ - offset;
}

void File::Prefetch(uint64_t offset, uint64_t length) const noexcept {
#if !defined(_WIN32) && defined(POSIX_FADV_WILLNEED)
    if (!IsOpen() || length == 0) return;
    ::posix_fadvise(handle_, static_cast<off_t>(offset), static_cast<off_t>(length), POSIX_FADV_WILLNEED);
    IoStats::AddPrefetch();
#else
    (void)offset;
    (void)length;
#endif
}

Status File::DropCache() const {
    if (!IsOpen()) return InvalidArgument("DropCache on a closed File");
#if !defined(_WIN32) && defined(POSIX_FADV_DONTNEED)
    const int rc = ::posix_fadvise(handle_, 0, 0, POSIX_FADV_DONTNEED);
    if (rc != 0) return ErrnoStatus("posix_fadvise(DONTNEED) '" + path_ + "'", rc);
#endif
    return Status::Ok();
}

}  // namespace ddbt::io
