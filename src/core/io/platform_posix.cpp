// SPDX-License-Identifier: MPL-2.0
#ifndef _WIN32

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#ifdef __linux__
#include <linux/falloc.h>
#endif

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <string>

#include "core/io/io_stats.h"
#include "core/io/platform.h"

namespace ddbt::io::detail {

namespace {
// Linux transfers at most 0x7ffff000 bytes per call; stay below it everywhere.
constexpr size_t kMaxChunk = size_t{1} << 30;

std::string What(const char* op, std::string_view path) {
    std::string s(op);
    s += " '";
    s += path;
    s += '\'';
    return s;
}
}  // namespace

Status ReadAt(NativeHandle fd, uint64_t offset, uint8_t* buf, size_t len, size_t* got, std::string_view path) {
    size_t done = 0;
    while (done < len) {
        const size_t want = std::min(len - done, kMaxChunk);
        if (offset + done > static_cast<uint64_t>(LLONG_MAX)) return OutOfRange("offset past the end of '" + std::string(path) + "'");
        const ssize_t n = ::pread(fd, buf + done, want, static_cast<off_t>(offset + done));
        if (n < 0) {
            if (errno == EINTR) continue;
            *got = done;
            return ErrnoStatus(What("pread", path), errno);
        }
        if (n == 0) break;  // end of file
        IoStats::AddRead(static_cast<uint64_t>(n));
        done += static_cast<size_t>(n);
    }
    *got = done;
    return Status::Ok();
}

Status WriteAt(NativeHandle fd, uint64_t offset, const uint8_t* buf, size_t len, std::string_view path) {
    size_t done = 0;
    while (done < len) {
        const size_t want = std::min(len - done, kMaxChunk);
        const ssize_t n = ::pwrite(fd, buf + done, want, static_cast<off_t>(offset + done));
        if (n < 0) {
            if (errno == EINTR) continue;
            return ErrnoStatus(What("pwrite", path), errno);
        }
        if (n == 0) return IoError(What("pwrite wrote nothing to", path));
        IoStats::AddWrite(static_cast<uint64_t>(n));
        done += static_cast<size_t>(n);
    }
    return Status::Ok();
}

Expected<uint64_t> HandleSize(NativeHandle fd, std::string_view path) {
    struct stat st {};
    if (::fstat(fd, &st) != 0) return ErrnoStatus(What("fstat", path), errno);
    return static_cast<uint64_t>(st.st_size);
}

void CloseHandle(NativeHandle fd) noexcept {
    if (fd >= 0) ::close(fd);
}

Expected<NativeHandle> CreateNew(const std::string& path, bool deleteOnClose, bool* exists) {
    *exists = false;
    for (;;) {
        const int fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
        if (fd >= 0) {
            if (deleteOnClose) ::unlink(path.c_str());
            IoStats::AddOpen();
            return fd;
        }
        const int err = errno;
        if (err == EINTR) continue;
        *exists = err == EEXIST;
        return ErrnoStatus(What("create", path), err);
    }
}

Status SetSize(NativeHandle fd, uint64_t size, std::string_view path) {
    if (size > static_cast<uint64_t>(LLONG_MAX)) return OutOfRange("size of '" + std::string(path) + "'");
    if (::ftruncate(fd, static_cast<off_t>(size)) != 0) return ErrnoStatus(What("ftruncate", path), errno);
    return Status::Ok();
}

Status SyncData(NativeHandle fd, std::string_view path) {
    if (::fdatasync(fd) != 0) return ErrnoStatus(What("fdatasync", path), errno);
    return Status::Ok();
}

Status RenameReplace(const std::string& from, const std::string& to) {
    if (std::rename(from.c_str(), to.c_str()) != 0) return ErrnoStatus("rename '" + from + "' to '" + to + "'", errno);
    return Status::Ok();
}

Status SyncParentDirectory(const std::string& path) {
    const size_t slash = path.find_last_of('/');
    const std::string dir = slash == std::string::npos ? "." : (slash == 0 ? "/" : path.substr(0, slash));
    const int fd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (fd < 0) return ErrnoStatus("open directory '" + dir + "'", errno);
    const int rc = ::fsync(fd);
    const int err = errno;
    ::close(fd);
    if (rc != 0) return ErrnoStatus("fsync directory '" + dir + "'", err);
    return Status::Ok();
}

void RemoveFile(const std::string& path) noexcept { ::unlink(path.c_str()); }

uint64_t PreferredBlockSize(NativeHandle fd) noexcept {
    struct stat st {};
    if (::fstat(fd, &st) == 0 && st.st_blksize > 0) return static_cast<uint64_t>(st.st_blksize);
    return 4096;
}

bool InsertRange(NativeHandle fd, uint64_t offset, uint64_t len) noexcept {
#ifdef __linux__
    return ::fallocate(fd, FALLOC_FL_INSERT_RANGE, static_cast<off_t>(offset), static_cast<off_t>(len)) == 0;
#else
    (void)fd;
    (void)offset;
    (void)len;
    return false;
#endif
}

void PreallocateHint(NativeHandle fd, uint64_t bytes) noexcept {
#ifdef __linux__
    // KEEP_SIZE: blocks are reserved but the size does not change.
    ::fallocate(fd, FALLOC_FL_KEEP_SIZE, 0, static_cast<off_t>(bytes));
#else
    (void)fd;
    (void)bytes;
#endif
}

void DropCacheHint(NativeHandle fd, uint64_t offset, uint64_t len) noexcept {
#ifdef POSIX_FADV_DONTNEED
    ::posix_fadvise(fd, static_cast<off_t>(offset), static_cast<off_t>(len), POSIX_FADV_DONTNEED);
#else
    (void)fd;
    (void)offset;
    (void)len;
#endif
}

unsigned long ProcessId() noexcept { return static_cast<unsigned long>(::getpid()); }

}  // namespace ddbt::io::detail

#endif  // !_WIN32
