// SPDX-License-Identifier: MPL-2.0
#pragma once

// ScratchFile: a read-write temporary file created next to a target, deleted when the object
// goes away unless Publish() renamed it onto its final path.
//
//   DDBT_ASSIGN_OR_RETURN(ScratchFile f, ScratchFile::Create(path + ".cog"));  // "<base>.<pid>.<n>.tmp"
//   DDBT_TRY(f.WriteAt(offset, bytes));
//   DDBT_TRY(f.ReadAt(offset, buffer));       // exactly buffer.size() bytes
//   DDBT_TRY(f.Publish(path, size, durable)); // set the size, sync, close, rename onto `path`
//
// The file is created exclusively (an existing file is never opened or overwritten). An
// anonymous file (ScratchKind::kAnonymous) has no name once created and cannot be published;
// it suits spill data. ReadAt / WriteAt are positional and may run on several threads at once.

#include <cstdint>
#include <span>
#include <string>
#include <string_view>

#include "core/io/platform.h"
#include "core/util/status.h"

namespace ddbt::io {

enum class ScratchKind : uint8_t {
    kNamed,      // keeps its name until Publish() or Discard()
    kAnonymous,  // unlinked at once (POSIX) / deleted on close (Windows)
};

class ScratchFile {
public:
    /// Creates "<base>.<pid>.<n>.tmp", n from a process-wide counter (retried on a collision).
    static Expected<ScratchFile> Create(std::string_view base, ScratchKind kind = ScratchKind::kNamed);

    ScratchFile() noexcept = default;
    ScratchFile(ScratchFile&& other) noexcept;
    ScratchFile& operator=(ScratchFile&& other) noexcept;
    ScratchFile(const ScratchFile&) = delete;
    ScratchFile& operator=(const ScratchFile&) = delete;
    /// Discards the file if it was not published.
    ~ScratchFile();

    Status WriteAt(uint64_t offset, std::span<const uint8_t> bytes);
    /// Reads exactly bytes.size() bytes at `offset`; a short read is an error.
    Status ReadAt(uint64_t offset, std::span<uint8_t> bytes);

    /// The file system's preferred I/O block size for this file.
    uint64_t BlockSize() const noexcept;
    /// Inserts `len` zero bytes at the start, shifting the data (Linux only, `len` a multiple of
    /// BlockSize()). False when it cannot be done; the file is then unchanged.
    bool InsertRangeAtStart(uint64_t len) noexcept;
    /// Reserves space for `bytes` (a hint).
    void Preallocate(uint64_t bytes) noexcept;
    /// [offset, offset + len) will not be read again soon (a hint).
    void DropCache(uint64_t offset, uint64_t len) noexcept;

    /// Sets the size to `size`, syncs the data and the directory entry when `durable`, closes the
    /// file and renames it onto `finalPath` (replacing it). On error the file is deleted.
    Status Publish(const std::string& finalPath, uint64_t size, bool durable);

    /// Closes and deletes the file. Idempotent.
    void Discard() noexcept;

    bool IsOpen() const noexcept { return handle_ != detail::kInvalidHandle; }
    /// The temporary name ("" once published or discarded).
    const std::string& Path() const noexcept { return path_; }

private:
    void Close() noexcept;

    detail::NativeHandle handle_ = detail::kInvalidHandle;
    std::string path_;
    ScratchKind kind_ = ScratchKind::kNamed;
};

}  // namespace ddbt::io
