// SPDX-License-Identifier: MPL-2.0
// Settings read from the environment.
#pragma once

#include <cstdint>
#include <cstdlib>

namespace ddbt {

/// The unsigned decimal integer in environment variable `name`, or `fallback` when it is unset,
/// empty, not a whole number or above `max`.
inline uint64_t EnvUnsigned(const char* name, uint64_t fallback, uint64_t max) {
    const char* e = std::getenv(name);
    if (e == nullptr || *e == '\0') return fallback;
    char* end = nullptr;
    const unsigned long long v = std::strtoull(e, &end, 10);
    if (end == e || *end != '\0' || v > max) return fallback;
    return static_cast<uint64_t>(v);
}

}  // namespace ddbt
