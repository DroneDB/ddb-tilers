// SPDX-License-Identifier: MPL-2.0
//
// IEEE 754 half-precision samples (TIFF SampleFormat 3, 16 bits; GDAL's Float16) to and from float,
// as GDAL's CPLHalfToFloat and CPLFloatToHalf convert them: subnormals, infinities and NaN payloads
// kept; a float is truncated to a half, a value beyond the halves is an infinity. DoubleToHalf is
// GDAL's GFloat16 from a double, which depends on the build: the compiler's _Float16 (correctly
// rounded) where it has one (DDBT_HAVE__FLOAT16, the test GDAL's build makes: GCC and Clang on
// x86-64, an MSVC that has it), else CPLFloatToHalf of the float.
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

inline uint16_t DoubleToHalf(double value) noexcept {
#if defined(DDBT_HAVE__FLOAT16)
    const _Float16 h = static_cast<_Float16>(value);
    uint16_t bits = 0;
    std::memcpy(&bits, &h, sizeof bits);
    return bits;
#else
    return FloatToHalf(static_cast<float>(value));
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
