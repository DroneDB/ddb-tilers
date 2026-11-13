// SPDX-License-Identifier: MPL-2.0
#pragma once

// Portable test helpers for the process environment: a scoped environment variable and the
// process id (for per-process temporary names).

#include <cstdlib>
#include <optional>
#include <string>

#include "core/io/platform.h"

namespace ddbt::test {

/// Sets (or unsets, value == nullptr) an environment variable for the scope and restores the
/// previous value (or its absence) afterwards.
class ScopedEnv {
public:
    ScopedEnv(const char* name, const char* value) : name_(name) {
        if (const char* old = std::getenv(name)) old_ = old;
        Set(name_.c_str(), value);
    }
    ~ScopedEnv() { Set(name_.c_str(), old_ ? old_->c_str() : nullptr); }
    ScopedEnv(const ScopedEnv&) = delete;
    ScopedEnv& operator=(const ScopedEnv&) = delete;

    /// Sets (or unsets) a variable, without restoring it.
    static void Set(const char* name, const char* value) {
#ifdef _WIN32
        _putenv_s(name, value != nullptr ? value : "");  // "" removes the variable
#else
        if (value != nullptr) ::setenv(name, value, 1);
        else ::unsetenv(name);
#endif
    }

private:
    std::string name_;
    std::optional<std::string> old_;
};

/// The id of this process, for temporary names that do not clash between test processes.
inline std::string ProcessIdString() { return std::to_string(io::detail::ProcessId()); }

}  // namespace ddbt::test
