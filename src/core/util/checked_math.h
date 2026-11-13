// SPDX-License-Identifier: MPL-2.0
#pragma once

// Checked integer arithmetic for parsers: every size / offset computed from untrusted input goes
// through these, so a malicious header yields a kOverflow Status instead of a wrapped value.

#include <concepts>
#include <cstdint>
#include <limits>
#include <string>
#include <type_traits>

#include "core/util/status.h"

namespace ddbt {

template <typename T>
concept CheckedInt = std::integral<T> && !std::same_as<T, bool>;

/// *out = a + b; false (and *out unspecified) on overflow.
template <CheckedInt T>
[[nodiscard]] constexpr bool CheckedAdd(T a, T b, T* out) noexcept {
#if defined(__GNUC__) || defined(__clang__)
    return !__builtin_add_overflow(a, b, out);
#else
    if constexpr (std::is_unsigned_v<T>) {
        *out = static_cast<T>(a + b);
        return *out >= a;
    } else {
        if ((b > 0 && a > std::numeric_limits<T>::max() - b) || (b < 0 && a < std::numeric_limits<T>::min() - b))
            return false;
        *out = static_cast<T>(a + b);
        return true;
    }
#endif
}

/// *out = a - b; false on overflow (for unsigned: b > a).
template <CheckedInt T>
[[nodiscard]] constexpr bool CheckedSub(T a, T b, T* out) noexcept {
#if defined(__GNUC__) || defined(__clang__)
    return !__builtin_sub_overflow(a, b, out);
#else
    if constexpr (std::is_unsigned_v<T>) {
        if (b > a) return false;
        *out = static_cast<T>(a - b);
        return true;
    } else {
        if ((b < 0 && a > std::numeric_limits<T>::max() + b) || (b > 0 && a < std::numeric_limits<T>::min() + b))
            return false;
        *out = static_cast<T>(a - b);
        return true;
    }
#endif
}

/// *out = a * b; false on overflow.
template <CheckedInt T>
[[nodiscard]] constexpr bool CheckedMul(T a, T b, T* out) noexcept {
#if defined(__GNUC__) || defined(__clang__)
    return !__builtin_mul_overflow(a, b, out);
#else
    if constexpr (std::is_unsigned_v<T>) {
        if (a != 0 && b > std::numeric_limits<T>::max() / a) return false;
        *out = static_cast<T>(a * b);
        return true;
    } else {
        if (a == 0 || b == 0) {
            *out = 0;
            return true;
        }
        constexpr T kMax = std::numeric_limits<T>::max();
        constexpr T kMin = std::numeric_limits<T>::min();
        if (a > 0) {
            if (b > 0 ? a > kMax / b : b < kMin / a) return false;
        } else {
            if (b > 0 ? a < kMin / b : (a != 0 && b < kMax / a)) return false;
        }
        *out = static_cast<T>(a * b);
        return true;
    }
#endif
}

/// Value-preserving conversion: false when `v` does not fit in To.
template <CheckedInt To, CheckedInt From>
[[nodiscard]] constexpr bool CheckedCast(From v, To* out) noexcept {
    if (!std::in_range<To>(v)) return false;
    *out = static_cast<To>(v);
    return true;
}

/// Rounds `v` up to a multiple of `align` (a power of two); false on overflow.
template <CheckedInt T>
[[nodiscard]] constexpr bool CheckedAlignUp(T v, T align, T* out) noexcept {
    T sum{};
    if (!CheckedAdd(v, static_cast<T>(align - 1), &sum)) return false;
    *out = static_cast<T>(sum & ~static_cast<T>(align - 1));
    return true;
}

// Expected-returning forms, for parser code that propagates errors with DDBT_ASSIGN_OR_RETURN.

template <CheckedInt T>
Expected<T> AddOrError(T a, T b, const char* what = "addition") {
    T r{};
    if (!CheckedAdd(a, b, &r)) return OverflowError(std::string("integer overflow in ") + what);
    return r;
}

template <CheckedInt T>
Expected<T> SubOrError(T a, T b, const char* what = "subtraction") {
    T r{};
    if (!CheckedSub(a, b, &r)) return OverflowError(std::string("integer overflow in ") + what);
    return r;
}

template <CheckedInt T>
Expected<T> MulOrError(T a, T b, const char* what = "multiplication") {
    T r{};
    if (!CheckedMul(a, b, &r)) return OverflowError(std::string("integer overflow in ") + what);
    return r;
}

template <CheckedInt To, CheckedInt From>
Expected<To> CastOrError(From v, const char* what = "conversion") {
    To r{};
    if (!CheckedCast(v, &r)) return OverflowError(std::string("value out of range in ") + what);
    return r;
}

/// a * b * c, for "width * height * bytesPerPixel" style sizes.
template <CheckedInt T>
Expected<T> Mul3OrError(T a, T b, T c, const char* what = "multiplication") {
    T r{};
    if (!CheckedMul(a, b, &r) || !CheckedMul(r, c, &r)) return OverflowError(std::string("integer overflow in ") + what);
    return r;
}

/// True when [offset, offset + length) lies inside [0, size) without overflow.
[[nodiscard]] constexpr bool RangeInside(uint64_t offset, uint64_t length, uint64_t size) noexcept {
    return offset <= size && length <= size - offset;
}

}  // namespace ddbt
