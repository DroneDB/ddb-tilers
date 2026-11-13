// SPDX-License-Identifier: MPL-2.0
#pragma once

#include <string>

namespace ddbt {

/// Per-thread message of the last failed C API call.
void SetLastError(std::string message);
void ClearLastError() noexcept;
const char* LastError() noexcept;

}  // namespace ddbt
