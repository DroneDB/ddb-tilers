// SPDX-License-Identifier: MPL-2.0
#ifdef _WIN32

#include <windows.h>

#include <algorithm>
#include <climits>
#include <string>

#include "core/io/io_stats.h"
#include "core/io/platform.h"

namespace ddbt::io::detail {

namespace {

constexpr DWORD kMaxChunk = DWORD{1} << 30;

std::string What(const char* op, std::string_view path) {
    std::string s(op);
    s += " '";
    s += path;
    s += '\'';
    return s;
}

// Handles are opened with FILE_FLAG_OVERLAPPED: each call carries its own offset, so threads
// never share a file position and the I/O manager does not serialise calls on the handle. The
// event is per thread (reused) and its low bit is set so a completion port, if one is ever
// associated, does not receive these completions.
HANDLE ThreadEvent() {
    thread_local struct Ev {
        HANDLE h = CreateEventW(nullptr, TRUE, FALSE, nullptr);
        ~Ev() {
            if (h != nullptr) ::CloseHandle(h);
        }
    } ev;
    return ev.h;
}

OVERLAPPED MakeOverlapped(uint64_t offset) {
    OVERLAPPED ov{};
    ov.Offset = static_cast<DWORD>(offset & 0xFFFFFFFFu);
    ov.OffsetHigh = static_cast<DWORD>(offset >> 32);
    ov.hEvent = reinterpret_cast<HANDLE>(reinterpret_cast<uintptr_t>(ThreadEvent()) | 1u);
    return ov;
}

}  // namespace

Expected<std::wstring> ToWidePath(std::string_view utf8) {
    if (utf8.empty()) return InvalidArgument("empty path");
    if (utf8.size() > static_cast<size_t>(INT_MAX)) return InvalidArgument("path too long");
    const int n = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, utf8.data(), static_cast<int>(utf8.size()), nullptr, 0);
    if (n <= 0) return InvalidArgument("path is not valid UTF-8: " + std::string(utf8));
    std::wstring w(static_cast<size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, utf8.data(), static_cast<int>(utf8.size()), w.data(), n);

    // Already a device / extended path: use as is.
    if (w.rfind(L"\\\\?\\", 0) == 0 || w.rfind(L"\\\\.\\", 0) == 0) return w;

    // Absolute, normalised ('/' -> '\', "." and ".." resolved): required by the \\?\ prefix,
    // which disables all normalisation.
    DWORD need = GetFullPathNameW(w.c_str(), 0, nullptr, nullptr);
    if (need == 0) return Win32Status("GetFullPathNameW '" + std::string(utf8) + "'", GetLastError());
    std::wstring full(need, L'\0');
    const DWORD got = GetFullPathNameW(w.c_str(), need, full.data(), nullptr);
    if (got == 0 || got >= need) return Win32Status("GetFullPathNameW '" + std::string(utf8) + "'", GetLastError());
    full.resize(got);

    if (full.rfind(L"\\\\", 0) == 0) return L"\\\\?\\UNC\\" + full.substr(2);  // \\server\share
    return L"\\\\?\\" + full;
}

Status ReadAt(NativeHandle h, uint64_t offset, uint8_t* buf, size_t len, size_t* got, std::string_view path) {
    size_t done = 0;
    while (done < len) {
        const DWORD want = static_cast<DWORD>(std::min<size_t>(len - done, kMaxChunk));
        OVERLAPPED ov = MakeOverlapped(offset + done);
        DWORD n = 0;
        if (!ReadFile(h, buf + done, want, nullptr, &ov)) {
            const DWORD err = GetLastError();
            if (err == ERROR_HANDLE_EOF) break;
            if (err != ERROR_IO_PENDING) {
                *got = done;
                return Win32Status(What("ReadFile", path), err);
            }
        }
        if (!GetOverlappedResult(h, &ov, &n, TRUE)) {
            const DWORD err = GetLastError();
            if (err == ERROR_HANDLE_EOF) break;
            *got = done;
            return Win32Status(What("ReadFile", path), err);
        }
        if (n == 0) break;
        IoStats::AddRead(n);
        done += n;
    }
    *got = done;
    return Status::Ok();
}

Status WriteAt(NativeHandle h, uint64_t offset, const uint8_t* buf, size_t len, std::string_view path) {
    size_t done = 0;
    while (done < len) {
        const DWORD want = static_cast<DWORD>(std::min<size_t>(len - done, kMaxChunk));
        OVERLAPPED ov = MakeOverlapped(offset + done);
        DWORD n = 0;
        if (!WriteFile(h, buf + done, want, nullptr, &ov)) {
            const DWORD err = GetLastError();
            if (err != ERROR_IO_PENDING) return Win32Status(What("WriteFile", path), err);
        }
        if (!GetOverlappedResult(h, &ov, &n, TRUE)) return Win32Status(What("WriteFile", path), GetLastError());
        if (n == 0) return IoError(What("WriteFile wrote nothing to", path));
        IoStats::AddWrite(n);
        done += n;
    }
    return Status::Ok();
}

Expected<uint64_t> HandleSize(NativeHandle h, std::string_view path) {
    LARGE_INTEGER sz{};
    if (!GetFileSizeEx(h, &sz)) return Win32Status(What("GetFileSizeEx", path), GetLastError());
    return static_cast<uint64_t>(sz.QuadPart);
}

void CloseHandle(NativeHandle h) noexcept {
    if (h != nullptr && h != INVALID_HANDLE_VALUE) ::CloseHandle(h);
}

}  // namespace ddbt::io::detail

#endif  // _WIN32
