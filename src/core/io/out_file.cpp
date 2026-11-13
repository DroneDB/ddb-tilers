// SPDX-License-Identifier: MPL-2.0
#include "core/io/out_file.h"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <mutex>

#include "core/io/io_stats.h"
#include "core/io/platform.h"
#include "core/mem/aligned.h"

#ifdef _WIN32
#include <process.h>
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#endif

namespace ddbt::io {

struct OutFile::State {
    detail::NativeHandle handle = detail::kInvalidHandle;
    std::string finalPath;
    std::string tempPath;
    OutFileOptions options;

    std::mutex mutex;            // guards everything below except maxEnd
    mem::AlignedBytes buffer;
    size_t bufferLen = 0;
    uint64_t appendPos = 0;      // logical end of the sequential stream (buffer included)
    Status error;                // sticky: first write error
    std::atomic<uint64_t> maxEnd{0};

    uint64_t BufferStart() const noexcept { return appendPos - bufferLen; }

    void RaiseEnd(uint64_t end) noexcept {
        uint64_t cur = maxEnd.load(std::memory_order_relaxed);
        while (end > cur && !maxEnd.compare_exchange_weak(cur, end, std::memory_order_relaxed)) {
        }
    }

    // Caller holds the mutex.
    Status FlushLocked() {
        if (!error.ok()) return error;
        if (bufferLen == 0) return Status::Ok();
        Status s = detail::WriteAt(handle, BufferStart(), buffer.get(), bufferLen, tempPath);
        if (!s.ok()) {
            error = s;
            return s;
        }
        bufferLen = 0;
        return Status::Ok();
    }

    void CloseHandle() noexcept {
        if (handle != detail::kInvalidHandle) detail::CloseHandle(handle);
        handle = detail::kInvalidHandle;
    }

    void RemoveTemp() noexcept {
        if (tempPath.empty()) return;
#ifdef _WIN32
        auto wide = detail::ToWidePath(tempPath);
        if (wide.ok()) DeleteFileW(wide->c_str());
#else
        ::unlink(tempPath.c_str());
#endif
    }
};

namespace {

std::string TempName(const std::string& finalPath) {
    static std::atomic<uint64_t> counter{0};
#ifdef _WIN32
    const auto pid = static_cast<unsigned long>(_getpid());
#else
    const auto pid = static_cast<unsigned long>(::getpid());
#endif
    return finalPath + ".tmp." + std::to_string(pid) + "." + std::to_string(counter.fetch_add(1, std::memory_order_relaxed));
}

#ifndef _WIN32
// fsync of the directory holding `path`, so the rename itself is durable.
Status SyncParentDir(const std::string& path) {
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
#endif

}  // namespace

OutFile::OutFile() noexcept = default;
OutFile::OutFile(OutFile&& other) noexcept = default;
OutFile& OutFile::operator=(OutFile&& other) noexcept {
    if (this != &other) {
        Abort();
        s_ = std::move(other.s_);
    }
    return *this;
}
OutFile::~OutFile() { Abort(); }

Expected<OutFile> OutFile::Create(std::string_view utf8Path, const OutFileOptions& options) {
    if (utf8Path.empty()) return InvalidArgument("OutFile: empty path");
    auto st = std::make_unique<State>();
    st->finalPath = std::string(utf8Path);
    st->options = options;
    if (st->options.bufferSize < 4096) st->options.bufferSize = 4096;

    for (int attempt = 0;; ++attempt) {
        st->tempPath = TempName(st->finalPath);
#ifdef _WIN32
        DDBT_ASSIGN_OR_RETURN(std::wstring wide, detail::ToWidePath(st->tempPath));
        HANDLE h = CreateFileW(wide.c_str(), GENERIC_READ | GENERIC_WRITE | DELETE, FILE_SHARE_READ, nullptr, CREATE_NEW,
                               FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, nullptr);
        if (h == INVALID_HANDLE_VALUE) {
            const DWORD err = GetLastError();
            if ((err == ERROR_FILE_EXISTS || err == ERROR_ALREADY_EXISTS) && attempt < 100) continue;
            return Win32Status("create '" + st->tempPath + "'", err);
        }
        st->handle = h;
#else
        const int fd = ::open(st->tempPath.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0666);
        if (fd < 0) {
            const int err = errno;
            if (err == EINTR || (err == EEXIST && attempt < 100)) continue;
            return ErrnoStatus("create '" + st->tempPath + "'", err);
        }
        st->handle = fd;
#endif
        break;
    }
    IoStats::AddOpen();

    if (options.preallocate > 0) {
#ifdef _WIN32
        FILE_ALLOCATION_INFO info{};
        info.AllocationSize.QuadPart = static_cast<LONGLONG>(options.preallocate);
        SetFileInformationByHandle(st->handle, FileAllocationInfo, &info, sizeof(info));  // hint only
#elif defined(__linux__)
        // KEEP_SIZE: blocks are reserved but the size stays 0; Finish() trims what is unused.
        ::fallocate(st->handle, FALLOC_FL_KEEP_SIZE, 0, static_cast<off_t>(options.preallocate));  // hint only
#endif
    }

    try {
        st->buffer = mem::MakeAlignedBytes(st->options.bufferSize, 4096);
    } catch (...) {
        st->CloseHandle();
        st->RemoveTemp();
        throw;
    }
    OutFile out;
    out.s_ = std::move(st);
    return out;
}

bool OutFile::IsOpen() const noexcept { return s_ && s_->handle != detail::kInvalidHandle; }

const std::string& OutFile::FinalPath() const noexcept {
    static const std::string empty;
    return s_ ? s_->finalPath : empty;
}

const std::string& OutFile::TempPath() const noexcept {
    static const std::string empty;
    return s_ ? s_->tempPath : empty;
}

uint64_t OutFile::Tell() const noexcept {
    if (!s_) return 0;
    std::lock_guard lock(s_->mutex);
    return s_->appendPos;
}

uint64_t OutFile::Size() const noexcept { return s_ ? s_->maxEnd.load(std::memory_order_relaxed) : 0; }

Status OutFile::Append(std::span<const uint8_t> bytes) {
    if (!IsOpen()) return InvalidArgument("Append on a closed OutFile");
    State& s = *s_;
    std::lock_guard lock(s.mutex);
    if (!s.error.ok()) return s.error;
    if (bytes.empty()) return Status::Ok();
    const size_t cap = s.options.bufferSize;
    if (s.bufferLen + bytes.size() > cap) DDBT_TRY(s.FlushLocked());
    if (bytes.size() >= cap) {
        // Larger than the buffer: written directly, no copy.
        Status st = detail::WriteAt(s.handle, s.appendPos, bytes.data(), bytes.size(), s.tempPath);
        if (!st.ok()) {
            s.error = st;
            return st;
        }
    } else {
        std::memcpy(s.buffer.get() + s.bufferLen, bytes.data(), bytes.size());
        s.bufferLen += bytes.size();
    }
    s.appendPos += bytes.size();
    s.RaiseEnd(s.appendPos);
    return Status::Ok();
}

Status OutFile::WriteAt(uint64_t offset, std::span<const uint8_t> bytes) {
    if (!IsOpen()) return InvalidArgument("WriteAt on a closed OutFile");
    if (bytes.empty()) return Status::Ok();
    if (offset > UINT64_MAX - bytes.size()) return OverflowError("WriteAt: offset + size overflows");
    State& s = *s_;
    const uint64_t end = offset + bytes.size();
    {
        std::lock_guard lock(s.mutex);
        if (!s.error.ok()) return s.error;
        const uint64_t bufStart = s.BufferStart();
        const uint64_t bufEnd = s.appendPos;
        if (s.bufferLen != 0 && offset < bufEnd && end > bufStart) {
            if (offset >= bufStart && end <= bufEnd) {
                // Entirely inside the pending buffer: patch it in memory.
                std::memcpy(s.buffer.get() + (offset - bufStart), bytes.data(), bytes.size());
                return Status::Ok();
            }
            DDBT_TRY(s.FlushLocked());
        }
    }
    Status st = detail::WriteAt(s.handle, offset, bytes.data(), bytes.size(), s.tempPath);
    if (!st.ok()) {
        std::lock_guard lock(s.mutex);
        if (s.error.ok()) s.error = st;
        return st;
    }
    s.RaiseEnd(end);
    return Status::Ok();
}

Status OutFile::Flush() {
    if (!IsOpen()) return InvalidArgument("Flush on a closed OutFile");
    std::lock_guard lock(s_->mutex);
    return s_->FlushLocked();
}

Status OutFile::Finish() {
    if (!IsOpen()) return InvalidArgument("Finish on a closed OutFile");
    State& s = *s_;
    Status st;
    {
        std::lock_guard lock(s.mutex);
        st = s.FlushLocked();
    }
    const uint64_t size = Size();
#ifdef _WIN32
    if (st.ok()) {
        FILE_END_OF_FILE_INFO eof{};
        eof.EndOfFile.QuadPart = static_cast<LONGLONG>(size);
        if (!SetFileInformationByHandle(s.handle, FileEndOfFileInfo, &eof, sizeof(eof))) {
            st = Win32Status("set end of file '" + s.tempPath + "'", GetLastError());
        }
    }
    if (st.ok() && s.options.durable && !FlushFileBuffers(s.handle)) {
        st = Win32Status("FlushFileBuffers '" + s.tempPath + "'", GetLastError());
    }
    s.CloseHandle();
    if (st.ok()) {
        auto wtmp = detail::ToWidePath(s.tempPath);
        auto wfinal = detail::ToWidePath(s.finalPath);
        if (!wtmp.ok()) st = wtmp.status();
        else if (!wfinal.ok()) st = wfinal.status();
        else if (!MoveFileExW(wtmp->c_str(), wfinal->c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
            st = Win32Status("rename '" + s.tempPath + "' to '" + s.finalPath + "'", GetLastError());
    }
#else
    // Truncating to the written size also releases blocks preallocated past it.
    if (st.ok() && ::ftruncate(s.handle, static_cast<off_t>(size)) != 0) {
        st = ErrnoStatus("ftruncate '" + s.tempPath + "'", errno);
    }
    if (st.ok() && s.options.durable && ::fdatasync(s.handle) != 0) {
        st = ErrnoStatus("fdatasync '" + s.tempPath + "'", errno);
    }
    s.CloseHandle();
    if (st.ok() && std::rename(s.tempPath.c_str(), s.finalPath.c_str()) != 0) {
        st = ErrnoStatus("rename '" + s.tempPath + "' to '" + s.finalPath + "'", errno);
    }
    if (st.ok() && s.options.durable) st = SyncParentDir(s.finalPath);
#endif
    if (st.ok()) {
        s.tempPath.clear();  // published: nothing left to delete
        return st;
    }
    s.RemoveTemp();
    s.tempPath.clear();
    return st;
}

void OutFile::Abort() noexcept {
    if (!s_) return;
    s_->CloseHandle();
    s_->RemoveTemp();
    s_->tempPath.clear();
}

}  // namespace ddbt::io
