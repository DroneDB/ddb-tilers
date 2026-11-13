// SPDX-License-Identifier: MPL-2.0
#pragma once

// OutFile: output file written to a temporary sibling and published atomically by rename.
//
//   DDBT_ASSIGN_OR_RETURN(OutFile out, OutFile::Create(path, {.preallocate = estimate}));
//   DDBT_TRY(out.Append(header));          // sequential, through an 8 MiB coalescing buffer
//   DDBT_TRY(out.Append(tiles));
//   DDBT_TRY(out.WriteAt(16, offsets));    // positional patch (e.g. a header field)
//   DDBT_TRY(out.Finish());                // flush, trim, rename onto `path`
//
// Until Finish() succeeds `path` is untouched (an existing file stays as it was); a destroyed or
// aborted OutFile deletes its temporary. Readers never see a half-written output.
//
// Threads: Append / Tell / Finish / Abort from one thread at a time. WriteAt may be called from
// several threads at once (positional writes, the append buffer is protected by a mutex) for
// disjoint ranges that were already appended or that Append will never cover.

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>

#include "core/util/status.h"

namespace ddbt::io {

struct OutFileOptions {
    /// Expected final size: space reserved up front (fallocate KEEP_SIZE / FileAllocationInfo)
    /// to limit fragmentation. A hint: 0 disables it, unsupported filesystems ignore it.
    uint64_t preallocate = 0;
    /// Size of the append buffer.
    size_t bufferSize = size_t{8} << 20;
    /// fsync the data (and on POSIX the directory entry) before / after the rename.
    bool durable = false;
};

class OutFile {
public:
    /// Creates the temporary file "<path>.<pid>.<n>.tmp" next to `utf8Path`.
    static Expected<OutFile> Create(std::string_view utf8Path, const OutFileOptions& options = {});

    OutFile() noexcept;
    OutFile(OutFile&& other) noexcept;
    OutFile& operator=(OutFile&& other) noexcept;
    OutFile(const OutFile&) = delete;
    OutFile& operator=(const OutFile&) = delete;
    /// Aborts (deletes the temporary) if Finish() was not called.
    ~OutFile();

    /// Appends at the current end of the sequential stream.
    Status Append(std::span<const uint8_t> bytes);

    /// Offset the next Append writes at.
    uint64_t Tell() const noexcept;

    /// Writes at an absolute offset (may extend the file; gaps read as zeros).
    Status WriteAt(uint64_t offset, std::span<const uint8_t> bytes);

    /// Writes buffered data to the file (Finish does it too).
    Status Flush();

    /// Flushes, truncates to the written size, optionally syncs, closes and renames the
    /// temporary onto the final path. On error the temporary is deleted.
    Status Finish();

    /// Closes and deletes the temporary. Idempotent.
    void Abort() noexcept;

    bool IsOpen() const noexcept;
    const std::string& FinalPath() const noexcept;
    const std::string& TempPath() const noexcept;
    /// Final size so far: max(end of appended data, end of positional writes).
    uint64_t Size() const noexcept;

private:
    struct State;
    std::unique_ptr<State> s_;
};

}  // namespace ddbt::io
