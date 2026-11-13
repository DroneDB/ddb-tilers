// SPDX-License-Identifier: MPL-2.0
//
// Text helpers of the PLY (ASCII), XYZ and PTS readers: number parsing with std::from_chars,
// newline-aligned byte-range tasks and a newline count (the line index of every task start).
#pragma once

#include <algorithm>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

namespace ddbt::copc::readers::text {

inline bool IsSpace(char c) noexcept { return c == ' ' || c == '\t' || c == '\r' || c == '\f' || c == '\v'; }

/// Parses a double at p (after skipping separators `seps` and spaces). Accepts a leading '+'.
/// Returns false at the end of the line or on a malformed number (p then points after it).
inline bool NextDouble(const char*& p, const char* end, char sep, double& v) noexcept {
    while (p < end && (IsSpace(*p) || *p == sep)) ++p;
    if (p >= end || *p == '\n') return false;
    const char* s = p;
    if (*s == '+') ++s;
    if (s < end && *s == '"') ++s;
    auto r = std::from_chars(s, end, v);
    if (r.ec != std::errc()) {
        while (p < end && !IsSpace(*p) && *p != sep && *p != '\n') ++p;
        v = 0;
        return true;  // a field that is not a number counts as 0 (PDAL logs and uses 0)
    }
    p = r.ptr;
    if (p < end && *p == '"') ++p;
    while (p < end && !IsSpace(*p) && *p != sep && *p != '\n') ++p;  // trailing garbage of the token
    return true;
}

/// Splits [begin, end) into tasks of about `bytes` bytes that start at line starts.
/// Returns the task start offsets (relative to begin) plus `end - begin` last.
inline std::vector<uint64_t> LineAlignedBlocks(const char* begin, const char* end, uint64_t bytes) {
    std::vector<uint64_t> starts{0};
    const uint64_t size = static_cast<uint64_t>(end - begin);
    if (bytes == 0) bytes = 1;
    for (uint64_t at = bytes; at < size; at += bytes) {
        const void* nl = std::memchr(begin + at - 1, '\n', static_cast<size_t>(size - (at - 1)));
        if (nl == nullptr) break;
        const uint64_t s = static_cast<uint64_t>(static_cast<const char*>(nl) - begin) + 1;
        if (s >= size) break;
        if (s > starts.back()) starts.push_back(s);
        at = s;
    }
    starts.push_back(size);
    return starts;
}

/// Newlines in [p, p + n) (vectorised by the compiler).
inline uint64_t CountNewlines(const char* p, size_t n) noexcept {
    uint64_t c = 0;
    for (size_t i = 0; i < n; ++i) c += p[i] == '\n';
    return c;
}

inline std::string Lower(std::string_view s) {
    std::string r(s);
    for (char& c : r)
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    return r;
}

}  // namespace ddbt::copc::readers::text
