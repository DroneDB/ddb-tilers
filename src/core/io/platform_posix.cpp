// SPDX-License-Identifier: MPL-2.0
#ifndef _WIN32

#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <climits>
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

}  // namespace ddbt::io::detail

#endif  // !_WIN32
