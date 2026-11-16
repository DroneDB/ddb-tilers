// SPDX-License-Identifier: MPL-2.0
//
// IEEE 754 half-precision samples (TIFF SampleFormat 3, 16 bits; GDAL's Float16) to and from float,
// as GDAL's CPLHalfToFloat and CPLFloatToHalf convert them: subnormals, infinities and NaN payloads
// kept; a float is truncated to a half, a value beyond the halves is an infinity. DoubleToHalf rounds a
// double to the nearest half (ties to even), as GDAL's GFloat16 does with the compiler's _Float16
// (DDBT_HAVE__FLOAT16, the test GDAL's build makes: GCC and Clang on x86-64). A GDAL built without
// _Float16 (MSVC) rounds only the halves it converts eight at a time (GDALCopyWords) and truncates the
// others, depending on where a sample falls in a buffer: ddb-tilers rounds them all, on every platform.
#pragma once

#include <cstdint>
#include <cstring>

namespace ddbt::util {

inline float HalfToFloat(uint16_t half) noexcept {
    const uint32_t sign = static_cast<uint32_t>(half & 0x8000u) << 16;
    uint32_t exponent = (half >> 10) & 0x1Fu;
    uint32_t mantissa = half & 0x3FFu;
    uint32_t bits = 0;
    if (exponent == 0) {
        if (mantissa == 0) {
            bits = sign;  // +-0
        } else {
            // Subnormal: normalised in float
            exponent = 127 - 15 + 1;
            while ((mantissa & 0x400u) == 0) {
                mantissa <<= 1;
                --exponent;
            }
            bits = sign | (exponent << 23) | ((mantissa & 0x3FFu) << 13);
        }
    } else if (exponent == 31) {
        bits = sign | 0x7F800000u | (mantissa << 13);  // infinity or NaN
    } else {
        bits = sign | ((exponent + 127 - 15) << 23) | (mantissa << 13);
    }
    float f = 0;
    std::memcpy(&f, &bits, sizeof f);
    return f;
}

inline uint16_t FloatToHalf(float value) noexcept {
    uint32_t f = 0;
    std::memcpy(&f, &value, sizeof f);
    const uint32_t sign = (f >> 31) & 1u;
    uint32_t exponent = (f >> 23) & 0xFFu;
    uint32_t mantissa = f & 0x7FFFFFu;
    if (exponent == 255) {
        if (mantissa == 0) return static_cast<uint16_t>((sign << 15) | 0x7C00u);  // infinity
        if ((mantissa >> 13) != 0) return static_cast<uint16_t>((sign << 15) | 0x7C00u | (mantissa >> 13));
        return static_cast<uint16_t>((sign << 15) | 0x7E00u);  // a NaN whose payload a half cannot hold
    }
    if (exponent <= 127 - 15) {
        // Zero, or too small for a normal half: a subnormal half or zero
        const uint32_t shift = 13 + 1 + 127 - 15 - exponent;
        if (shift >= 32) return static_cast<uint16_t>(sign << 15);
        return static_cast<uint16_t>((sign << 15) | ((mantissa | 0x800000u) >> shift));
    }
    if (exponent - (127 - 15) >= 31) return static_cast<uint16_t>((sign << 15) | 0x7C00u);  // beyond: infinity
    exponent -= 127 - 15;
    mantissa >>= 13;
    return static_cast<uint16_t>((sign << 15) | (exponent << 10) | mantissa);
}

/// A double rounded once to the nearest half, ties to even (IEEE 754), without _Float16: subnormal
/// halves, an infinity past 65504 (65520 and above), a NaN quiet with the top of its payload
inline uint16_t DoubleToHalfNearest(double value) noexcept {
    uint64_t d = 0;
    std::memcpy(&d, &value, sizeof d);
    const auto sign = static_cast<uint16_t>((d >> 48) & 0x8000u);
    const auto exponent = static_cast<int>((d >> 52) & 0x7FFu);
    const uint64_t mantissa = d & ((uint64_t{1} << 52) - 1);
    if (exponent == 0x7FF) {
        if (mantissa == 0) return static_cast<uint16_t>(sign | 0x7C00u);  // infinity
        return static_cast<uint16_t>(sign | 0x7E00u | ((mantissa >> 42) & 0x3FFu));  // quiet NaN
    }
    if (exponent == 0) return sign;  // zero, or a double subnormal: far below the halves
    const uint64_t m = mantissa | (uint64_t{1} << 52);
    // Rounds q + rem / 2^shift to the nearest integer, ties to even
    const auto round = [](uint64_t q, uint64_t rem, int shift) {
        const uint64_t half = uint64_t{1} << (shift - 1);
        return (rem > half || (rem == half && (q & 1u) != 0)) ? q + 1 : q;
    };
    const int e = exponent - 1023 + 15;  // the exponent of a normal half
    if (e <= 0) {
        // A subnormal half: the value in units of 2^-24 is m / 2^(1051 - exponent)
        const int shift = 1051 - exponent;
        if (shift >= 54) return sign;  // below half of the smallest subnormal
        const uint64_t q = round(m >> shift, m & ((uint64_t{1} << shift) - 1), shift);
        return static_cast<uint16_t>(sign | q);  // 0x400 is the smallest normal half
    }
    uint64_t q = round(mantissa >> 42, mantissa & ((uint64_t{1} << 42) - 1), 42);
    int halfExponent = e;
    if (q == 0x400u) {  // rounded up into the next binade
        q = 0;
        ++halfExponent;
    }
    if (halfExponent >= 31) return static_cast<uint16_t>(sign | 0x7C00u);  // infinity
    return static_cast<uint16_t>(sign | (static_cast<unsigned>(halfExponent) << 10) | q);
}

inline uint16_t DoubleToHalf(double value) noexcept {
#if defined(DDBT_HAVE__FLOAT16)
    const _Float16 h = static_cast<_Float16>(value);
    uint16_t bits = 0;
    std::memcpy(&bits, &h, sizeof bits);
    return bits;
#else
    return DoubleToHalfNearest(value);
#endif
}

/// A half as a sample of the warp's templates (GDAL's GFloat16): its bits.
struct Half {
    uint16_t bits = 0;
};

inline constexpr uint16_t kHalfMaxBits = 0x7BFF;  // 65504, GDAL's NumericLimits<GFloat16>::max()

/// GDAL's nextafter of two GFloat16: one step of the bits towards `toward`.
inline uint16_t HalfNextAfter(uint16_t from, uint16_t toward) noexcept {
    const float x = HalfToFloat(from), y = HalfToFloat(toward);
    if (x != x) return from;    // NaN
    if (y != y) return toward;  // NaN
    if (x == y) return toward;
    if (x != 0.0f) return static_cast<uint16_t>((x < y) == (x > 0.0f) ? from + 1 : from - 1);
    return static_cast<uint16_t>(((toward >> 15) << 15) | 0x0001u);
}

}  // namespace ddbt::util
