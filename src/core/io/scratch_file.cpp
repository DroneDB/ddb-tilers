// SPDX-License-Identifier: MPL-2.0
#include "core/io/scratch_file.h"

#include <atomic>
#include <utility>

namespace ddbt::io {

Expected<ScratchFile> ScratchFile::Create(std::string_view base, ScratchKind kind) {
    if (base.empty()) return InvalidArgument("ScratchFile: empty path");
    static std::atomic<uint64_t> counter{0};
    const std::string prefix = std::string(base) + "." + std::to_string(detail::ProcessId()) + ".";
    for (int attempt = 0;; ++attempt) {
        std::string path = prefix + std::to_string(counter.fetch_add(1, std::memory_order_relaxed)) + ".tmp";
        bool exists = false;
        auto h = detail::CreateNew(path, kind == ScratchKind::kAnonymous, &exists);
        if (!h.ok()) {
            if (exists && attempt < 100) continue;
            return h.status();
        }
        ScratchFile f;
        f.handle_ = *h;
        f.path_ = std::move(path);
        f.kind_ = kind;
        return f;
    }
}

ScratchFile::ScratchFile(ScratchFile&& other) noexcept
    : handle_(std::exchange(other.handle_, detail::kInvalidHandle)),
      path_(std::move(other.path_)),
      kind_(other.kind_) {
    other.path_.clear();
}

ScratchFile& ScratchFile::operator=(ScratchFile&& other) noexcept {
    if (this != &other) {
        Discard();
        handle_ = std::exchange(other.handle_, detail::kInvalidHandle);
        path_ = std::move(other.path_);
        other.path_.clear();
        kind_ = other.kind_;
    }
    return *this;
}

ScratchFile::~ScratchFile() { Discard(); }

Status ScratchFile::WriteAt(uint64_t offset, std::span<const uint8_t> bytes) {
    if (!IsOpen()) return InvalidArgument("WriteAt on a closed ScratchFile");
    return detail::WriteAt(handle_, offset, bytes.data(), bytes.size(), path_);
}

Status ScratchFile::ReadAt(uint64_t offset, std::span<uint8_t> bytes) {
    if (!IsOpen()) return InvalidArgument("ReadAt on a closed ScratchFile");
    size_t got = 0;
    DDBT_TRY(detail::ReadAt(handle_, offset, bytes.data(), bytes.size(), &got, path_));
    if (got != bytes.size()) return IoError("unexpected end of " + path_);
    return Status::Ok();
}

uint64_t ScratchFile::BlockSize() const noexcept { return IsOpen() ? detail::PreferredBlockSize(handle_) : 4096; }

bool ScratchFile::InsertRangeAtStart(uint64_t len) noexcept { return IsOpen() && detail::InsertRange(handle_, 0, len); }

void ScratchFile::Preallocate(uint64_t bytes) noexcept {
    if (IsOpen() && bytes > 0) detail::PreallocateHint(handle_, bytes);
}

void ScratchFile::DropCache(uint64_t offset, uint64_t len) noexcept {
    if (IsOpen()) detail::DropCacheHint(handle_, offset, len);
}

Status ScratchFile::Publish(const std::string& finalPath, uint64_t size, bool durable) {
    if (!IsOpen()) return InvalidArgument("Publish of a closed ScratchFile");
    if (kind_ == ScratchKind::kAnonymous) return InvalidArgument("an anonymous ScratchFile cannot be published");
    Status st = detail::SetSize(handle_, size, path_);
    if (st.ok() && durable) st = detail::SyncData(handle_, path_);
    Close();
    if (st.ok()) st = detail::RenameReplace(path_, finalPath);
    if (st.ok() && durable) st = detail::SyncParentDirectory(finalPath);
    if (st.ok()) {
        path_.clear();  // published: nothing left to delete
        return st;
    }
    Discard();
    return st;
}

void ScratchFile::Close() noexcept {
    if (handle_ != detail::kInvalidHandle) detail::CloseHandle(handle_);
    handle_ = detail::kInvalidHandle;
}

void ScratchFile::Discard() noexcept {
    Close();
    if (!path_.empty() && kind_ == ScratchKind::kNamed) detail::RemoveFile(path_);
    path_.clear();
}

}  // namespace ddbt::io
