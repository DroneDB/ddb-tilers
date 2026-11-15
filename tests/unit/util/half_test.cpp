// SPDX-License-Identifier: MPL-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>

#include "core/util/half.h"

namespace ddbt::util {
namespace {

uint32_t BitsOf(float f) {
    uint32_t b = 0;
    std::memcpy(&b, &f, 4);
    return b;
}

TEST(Half, everyHalfIsReadExactly) {
    EXPECT_EQ(HalfToFloat(0x3C00), 1.0f);
    EXPECT_EQ(HalfToFloat(0xC000), -2.0f);
    EXPECT_EQ(HalfToFloat(0x7BFF), 65504.0f);                    // the largest half
    EXPECT_EQ(HalfToFloat(0x0001), std::ldexp(1.0f, -24));      // the smallest subnormal
    EXPECT_EQ(HalfToFloat(0x03FF), std::ldexp(1023.0f, -24));   // the largest subnormal
    EXPECT_EQ(BitsOf(HalfToFloat(0x8000)), 0x80000000u);         // -0
    EXPECT_EQ(HalfToFloat(0x7C00), std::numeric_limits<float>::infinity());
    EXPECT_EQ(BitsOf(HalfToFloat(0x7E01)), 0x7FC02000u);         // a NaN keeps its payload
    // Every half goes back to itself (NaNs with a payload a half holds)
    for (uint32_t h = 0; h <= 0xFFFF; ++h)
        EXPECT_EQ(FloatToHalf(HalfToFloat(static_cast<uint16_t>(h))), h) << h;
}

TEST(Half, aFloatIsTruncatedAndADoubleRoundedAsGdalsBuildDoes) {
    // CPLFloatToHalf truncates: 1 + 2^-11 + 2^-12 (between two halves, nearer the upper one) is 1
    const float between = 1.0f + std::ldexp(1.0f, -11) + std::ldexp(1.0f, -12);
    EXPECT_EQ(FloatToHalf(between), 0x3C00);
    EXPECT_EQ(FloatToHalf(1e6f), 0x7C00);   // beyond the halves: infinity
    EXPECT_EQ(FloatToHalf(1e-9f), 0x0000);  // below them: zero
#if defined(__FLT16_MAX__)
    // With the compiler's _Float16, GDAL's GFloat16 rounds the double to nearest
    EXPECT_EQ(DoubleToHalf(static_cast<double>(between)), 0x3C01);
    EXPECT_EQ(DoubleToHalf(-9999.0), 0xF0E2);  // -10000: the nearest half
#else
    EXPECT_EQ(DoubleToHalf(static_cast<double>(between)), 0x3C00);
#endif
}

}  // namespace
}  // namespace ddbt::util
