// SPDX-License-Identifier: MPL-2.0
#include "core/io/out_file.h"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <mutex>

#include "core/io/scratch_file.h"
#include "core/mem/aligned.h"

namespace ddbt::io {

struct OutFile::State {
    ScratchFile file;
    std::string finalPath;
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
        Status s = file.WriteAt(BufferStart(), {buffer.get(), bufferLen});
        if (!s.ok()) {
            error = s;
            return s;
        }
        bufferLen = 0;
        return Status::Ok();
    }
};

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
    DDBT_ASSIGN_OR_RETURN(st->file, ScratchFile::Create(st->finalPath));
    // Blocks are reserved but the size stays 0; Finish() trims what is unused.
    st->file.Preallocate(options.preallocate);
    st->buffer = mem::MakeAlignedBytes(st->options.bufferSize, 4096);  // on a throw, ~ScratchFile deletes the file
    OutFile out;
    out.s_ = std::move(st);
    return out;
}

bool OutFile::IsOpen() const noexcept { return s_ && s_->file.IsOpen(); }

const std::string& OutFile::FinalPath() const noexcept {
    static const std::string empty;
    return s_ ? s_->finalPath : empty;
}

const std::string& OutFile::TempPath() const noexcept {
    static const std::string empty;
    return s_ ? s_->file.Path() : empty;
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
        Status st = s.file.WriteAt(s.appendPos, bytes);
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
    Status st = s.file.WriteAt(offset, bytes);
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
    if (!st.ok()) {
        s.file.Discard();
        return st;
    }
    // Setting the size to the written size also releases blocks preallocated past it.
    return s.file.Publish(s.finalPath, Size(), s.options.durable);
}

void OutFile::Abort() noexcept {
    if (s_) s_->file.Discard();
}

}  // namespace ddbt::io
