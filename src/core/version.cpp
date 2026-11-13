// SPDX-License-Identifier: MPL-2.0
#include "core/version.h"

#include "ddbt_config.h"

namespace ddbt {

const char* Version() noexcept { return DDBT_VERSION_STRING; }
const char* GitSha() noexcept { return DDBT_GIT_SHA; }
const char* CompilerId() noexcept { return DDBT_COMPILER_ID " " DDBT_COMPILER_VERSION; }
const char* BuildType() noexcept { return DDBT_BUILD_TYPE; }
const char* IsaBaseline() noexcept { return DDBT_ISA_BASELINE_NAME; }

}  // namespace ddbt
