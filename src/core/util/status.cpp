// SPDX-License-Identifier: MPL-2.0
#include "core/util/status.h"

#include <cerrno>
#include <cstring>

#ifdef _WIN32
#include <windows.h>
#endif

namespace ddbt {

const char* StatusCodeName(StatusCode code) noexcept {
    switch (code) {
        case StatusCode::kOk: return "OK";
        case StatusCode::kInvalidArgument: return "InvalidArgument";
        case StatusCode::kOutOfRange: return "OutOfRange";
        case StatusCode::kNotFound: return "NotFound";
        case StatusCode::kIoError: return "IoError";
        case StatusCode::kOverflow: return "Overflow";
        case StatusCode::kCorrupt: return "Corrupt";
        case StatusCode::kUnsupported: return "Unsupported";
        case StatusCode::kResourceExhausted: return "ResourceExhausted";
        case StatusCode::kCancelled: return "Cancelled";
        case StatusCode::kNoSpace: return "NoSpace";
        case StatusCode::kInternal: return "Internal";
    }
    return "Unknown";
}

Status::Status(StatusCode code, std::string message) {
    if (code != StatusCode::kOk) rep_ = std::make_unique<Rep>(Rep{code, std::move(message)});
}

std::string Status::ToString() const {
    if (ok()) return "OK";
    std::string s = StatusCodeName(rep_->code);
    if (!rep_->message.empty()) {
        s += ": ";
        s += rep_->message;
    }
    return s;
}

Status Status::WithContext(std::string_view context) const {
    if (ok()) return Status();
    std::string m(context);
    m += ": ";
    m += rep_->message;
    return Status(rep_->code, std::move(m));
}

Status InvalidArgument(std::string message) { return {StatusCode::kInvalidArgument, std::move(message)}; }
Status OutOfRange(std::string message) { return {StatusCode::kOutOfRange, std::move(message)}; }
Status NotFound(std::string message) { return {StatusCode::kNotFound, std::move(message)}; }
Status IoError(std::string message) { return {StatusCode::kIoError, std::move(message)}; }
Status OverflowError(std::string message) { return {StatusCode::kOverflow, std::move(message)}; }
Status Corrupt(std::string message) { return {StatusCode::kCorrupt, std::move(message)}; }
Status Unsupported(std::string message) { return {StatusCode::kUnsupported, std::move(message)}; }
Status ResourceExhausted(std::string message) { return {StatusCode::kResourceExhausted, std::move(message)}; }
Status Cancelled(std::string message) { return {StatusCode::kCancelled, std::move(message)}; }
Status InternalError(std::string message) { return {StatusCode::kInternal, std::move(message)}; }

Status ErrnoStatus(std::string_view what, int err) {
    std::string m(what);
    m += ": ";
#ifdef _WIN32
    char buf[256] = {};
    strerror_s(buf, sizeof(buf), err);
    m += buf;
#else
    // strerror_r has two incompatible signatures (GNU / XSI); strerror is thread-safe in glibc
    // for known error numbers, which is all we pass.
    m += std::strerror(err);
#endif
    m += " (errno ";
    m += std::to_string(err);
    m += ')';
    return Status(err == ENOENT   ? StatusCode::kNotFound
                  : err == ENOSPC ? StatusCode::kNoSpace
                                  : StatusCode::kIoError,
                  std::move(m));
}

#ifdef _WIN32
Status Win32Status(std::string_view what, unsigned long err) {
    std::string m(what);
    m += ": ";
    wchar_t* wbuf = nullptr;
    const DWORD n = FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                                       FORMAT_MESSAGE_IGNORE_INSERTS,
                                   nullptr, err, 0, reinterpret_cast<LPWSTR>(&wbuf), 0, nullptr);
    if (n != 0 && wbuf != nullptr) {
        const int len = WideCharToMultiByte(CP_UTF8, 0, wbuf, static_cast<int>(n), nullptr, 0, nullptr, nullptr);
        std::string text(static_cast<size_t>(len > 0 ? len : 0), '\0');
        if (len > 0) WideCharToMultiByte(CP_UTF8, 0, wbuf, static_cast<int>(n), text.data(), len, nullptr, nullptr);
        while (!text.empty() && (text.back() == '\n' || text.back() == '\r' || text.back() == ' ')) text.pop_back();
        m += text;
        LocalFree(wbuf);
    } else {
        m += "Windows error";
    }
    m += " (error ";
    m += std::to_string(err);
    m += ')';
    const bool notFound = err == ERROR_FILE_NOT_FOUND || err == ERROR_PATH_NOT_FOUND;
    const bool full = err == ERROR_DISK_FULL || err == ERROR_HANDLE_DISK_FULL;
    return Status(notFound ? StatusCode::kNotFound : full ? StatusCode::kNoSpace : StatusCode::kIoError, std::move(m));
}
#endif

}  // namespace ddbt
