// SPDX-License-Identifier: MPL-2.0
#include "core/util/last_error.h"

#include <utility>

namespace ddbt {

namespace {
thread_local std::string g_lastError;
}

void SetLastError(std::string message) { g_lastError = std::move(message); }
void ClearLastError() noexcept { g_lastError.clear(); }
const char* LastError() noexcept { return g_lastError.c_str(); }

}  // namespace ddbt
