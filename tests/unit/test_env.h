// SPDX-License-Identifier: MPL-2.0
#pragma once

// Portable test helpers for the process environment: a scoped environment variable, the process
// id (for per-process temporary names) and a temporary directory.

#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <iterator>
#include <optional>
#include <random>
#include <string>
#include <system_error>

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

/// A directory created under DDBT_TEST_TMP (else the system's temporary directory) for the scope,
/// removed with its content afterwards.
class TempDir {
public:
    explicit TempDir(const std::string& prefix = "ddbt-test") {
        const char* base = std::getenv("DDBT_TEST_TMP");
        const std::filesystem::path root = base ? std::filesystem::path(base) : std::filesystem::temp_directory_path();
        path_ = root / (prefix + "-" + std::to_string(std::random_device{}()) + "-" + std::to_string(counter_++));
        std::filesystem::create_directories(path_);
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;

    const std::filesystem::path& path() const { return path_; }
    std::string File(const std::string& name) const { return (path_ / name).string(); }
    size_t Entries() const {
        return static_cast<size_t>(
            std::distance(std::filesystem::directory_iterator(path_), std::filesystem::directory_iterator()));
    }

private:
    static inline std::atomic<int> counter_{0};
    std::filesystem::path path_;
};

}  // namespace ddbt::test
