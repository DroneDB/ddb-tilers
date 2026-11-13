// SPDX-License-Identifier: MPL-2.0
#pragma once

namespace ddbt {

/// "major.minor.patch"
const char* Version() noexcept;
/// Short git sha of the configure-time checkout, "unknown" when not built from a git tree.
const char* GitSha() noexcept;
/// Compiler id and version, e.g. "GNU 15.2.0".
const char* CompilerId() noexcept;
/// CMake build type, e.g. "Release".
const char* BuildType() noexcept;
/// The -march baseline the library was built for: "x86-64-v3", "native", ...
const char* IsaBaseline() noexcept;

}  // namespace ddbt
