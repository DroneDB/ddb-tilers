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

}  // namespace ddbt::io::detail
