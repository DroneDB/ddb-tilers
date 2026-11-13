// SPDX-License-Identifier: MPL-2.0
#pragma once

// Internal OS layer of core/io: positional read / write loops and path conversion. Implemented
// in platform_posix.cpp and platform_win.cpp (each compiles to nothing on the other OS).

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

#include "core/util/status.h"

namespace ddbt::io::detail {

#ifdef _WIN32
using NativeHandle = void*;  // HANDLE
inline const NativeHandle kInvalidHandle = reinterpret_cast<void*>(static_cast<intptr_t>(-1));

/// UTF-8 path -> UTF-16 path for the W functions: made absolute (GetFullPathNameW, which also
/// turns '/' into '\') and prefixed with \\?\ (or \\?\UNC\) so paths over MAX_PATH work.
Expected<std::wstring> ToWidePath(std::string_view utf8);
#else
using NativeHandle = int;
inline constexpr NativeHandle kInvalidHandle = -1;
#endif

/// Reads up to `len` bytes at `offset`, retrying short reads and interruptions until `len`
/// bytes or end of file; *got receives the byte count. No shared file position is used.
Status ReadAt(NativeHandle h, uint64_t offset, uint8_t* buf, size_t len, size_t* got, std::string_view path);

/// Writes exactly `len` bytes at `offset`.
Status WriteAt(NativeHandle h, uint64_t offset, const uint8_t* buf, size_t len, std::string_view path);

/// Size of an open file.
Expected<uint64_t> HandleSize(NativeHandle h, std::string_view path);

void CloseHandle(NativeHandle h) noexcept;

/// Creates `path` for reading and writing; never opens an existing file (*exists is set when it
/// failed because the path exists). Windows handles are overlapped, as ReadAt / WriteAt expect.
/// With `deleteOnClose` the file has no name left once created (POSIX: unlinked at once;
/// Windows: FILE_FLAG_DELETE_ON_CLOSE) and disappears with the handle.
Expected<NativeHandle> CreateNew(const std::string& path, bool deleteOnClose, bool* exists);

/// Sets the size of an open file (truncates, or extends with zeros).
Status SetSize(NativeHandle h, uint64_t size, std::string_view path);

/// Writes the file's data to stable storage (fdatasync / FlushFileBuffers).
Status SyncData(NativeHandle h, std::string_view path);

/// Renames `from` onto `to`, replacing `to` if it exists.
Status RenameReplace(const std::string& from, const std::string& to);

/// Makes a rename into the directory of `path` durable (POSIX: fsync of the directory; Windows:
/// nothing to do, MoveFileEx with MOVEFILE_WRITE_THROUGH is used).
Status SyncParentDirectory(const std::string& path);

/// Deletes a file, ignoring errors.
void RemoveFile(const std::string& path) noexcept;

/// The file system's preferred I/O block size for the file (st_blksize; 4096 on Windows).
uint64_t PreferredBlockSize(NativeHandle h) noexcept;

/// Inserts `len` zero bytes at `offset`, shifting the rest of the file (Linux
/// FALLOC_FL_INSERT_RANGE; offset and len must be multiples of the block size). False when the
/// OS or the file system cannot.
bool InsertRange(NativeHandle h, uint64_t offset, uint64_t len) noexcept;

/// Reserves space for `bytes` without changing the size (a hint: errors are ignored).
void PreallocateHint(NativeHandle h, uint64_t bytes) noexcept;

/// Tells the OS that [offset, offset + len) will not be read again soon (a hint).
void DropCacheHint(NativeHandle h, uint64_t offset, uint64_t len) noexcept;

/// The id of this process.
unsigned long ProcessId() noexcept;

}  // namespace ddbt::io::detail
