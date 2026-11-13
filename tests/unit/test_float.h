// SPDX-License-Identifier: MPL-2.0
#pragma once

// Comparing doubles with reference values computed by GDAL / Python on Linux: with glibc's libm
// the code must give the same bits; other C runtimes (MSVC's) round exp, log, atan, tan... in
// the last bits differently, so the same formulas land a few units in the last place away
// (at most 4 on the reference data with MSVC's runtime). The bound below allows twice that.

#include <bit>
#include <cmath>
#include <cstdint>

namespace ddbt::test {

/// Units in the last place between two finite doubles (0: the same bits; +0 and -0 are 0 apart).
inline uint64_t UlpDistance(double a, double b) {
    auto ordered = [](double v) {
        const auto bits = std::bit_cast<int64_t>(v);
        return bits < 0 ? INT64_MIN - bits : bits;  // monotonic over the doubles
    };
    const int64_t x = ordered(a), y = ordered(b);
    return x > y ? static_cast<uint64_t>(x) - static_cast<uint64_t>(y) : static_cast<uint64_t>(y) - static_cast<uint64_t>(x);
}

/// How far a libm-based result may be from a glibc reference value.
#if defined(__GLIBC__)
inline constexpr uint64_t kLibmUlpBound = 0;
#else
inline constexpr uint64_t kLibmUlpBound = 8;
#endif

}  // namespace ddbt::test

/// a equals the glibc reference b: the same bits with glibc, within kLibmUlpBound elsewhere.
#define EXPECT_LIBM_EQ(a, b)                                                       \
    EXPECT_LE(::ddbt::test::UlpDistance((a), (b)), ::ddbt::test::kLibmUlpBound) \
        << #a " = " << (a) << ", " #b " = " << (b)
