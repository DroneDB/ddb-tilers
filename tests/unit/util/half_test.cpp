// SPDX-License-Identifier: MPL-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>

#include "cog/tiff/tiff_reader.h"
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

TEST(Half, aFloatIsTruncatedAndADoubleRoundedToTheNearestHalf) {
    // CPLFloatToHalf truncates: 1 + 2^-11 + 2^-12 (between two halves, nearer the upper one) is 1
    const float between = 1.0f + std::ldexp(1.0f, -11) + std::ldexp(1.0f, -12);
    EXPECT_EQ(FloatToHalf(between), 0x3C00);
    EXPECT_EQ(FloatToHalf(1e6f), 0x7C00);   // beyond the halves: infinity
    EXPECT_EQ(FloatToHalf(1e-9f), 0x0000);  // below them: zero
    // A double is rounded to the nearest half on every platform, with _Float16 or without
    for (const auto convert : {&DoubleToHalf, &DoubleToHalfNearest}) {
        EXPECT_EQ(convert(static_cast<double>(between)), 0x3C01);
        EXPECT_EQ(convert(-9999.0), 0xF0E2);                      // -10000: the nearest half
        EXPECT_EQ(convert(1.0 + std::ldexp(1.0, -11)), 0x3C00);   // a tie: to the even half
        EXPECT_EQ(convert(1.0 + 3 * std::ldexp(1.0, -11)), 0x3C02);
        EXPECT_EQ(convert(65504.0), 0x7BFF);
        EXPECT_EQ(convert(65519.99), 0x7BFF);
        EXPECT_EQ(convert(65520.0), 0x7C00);                      // the tie above the largest: infinity
        EXPECT_EQ(convert(-1e300), 0xFC00);
        EXPECT_EQ(convert(std::ldexp(1.0, -24)), 0x0001);         // the smallest subnormal
        EXPECT_EQ(convert(std::ldexp(1.0, -25)), 0x0000);         // a tie with zero: to zero
        EXPECT_EQ(convert(std::ldexp(1.5, -25)), 0x0001);
        EXPECT_EQ(convert(std::ldexp(1023.5, -24)), 0x0400);      // the largest subnormal rounds up into the normals
        EXPECT_EQ(convert(-0.0), 0x8000);
        EXPECT_EQ(convert(1e-320), 0x0000);                       // a double subnormal
    }
}

#if defined(DDBT_HAVE__FLOAT16)
// Without _Float16 a double is rounded by DoubleToHalfNearest: the same half as the compiler's, bit for bit
TEST(Half, theRoundingWithoutFloat16IsTheCompilers) {
    uint64_t state = 0x9E3779B97F4A7C15u;
    const auto next = [&state] {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return state;
    };
    const auto check = [](double v) {
        const _Float16 h = static_cast<_Float16>(v);
        uint16_t bits = 0;
        std::memcpy(&bits, &h, sizeof bits);
        if (v != v) return;  // NaN payloads: libgcc's are its own
        ASSERT_EQ(DoubleToHalfNearest(v), bits) << v;
    };
    // Every half, the doubles around it and the ties between it and the next
    for (uint32_t h = 0; h < 0x7C00; ++h) {
        const double a = HalfToFloat(static_cast<uint16_t>(h));
        const double b = HalfToFloat(static_cast<uint16_t>(h + 1));
        for (const double v : {a, (a + b) / 2, std::nextafter(a, b), std::nextafter((a + b) / 2, a),
                               std::nextafter((a + b) / 2, b)}) {
            check(v);
            check(-v);
        }
    }
    // Doubles of any exponent near the halves
    for (int i = 0; i < 2000000; ++i) {
        const uint64_t r = next();
        const double v = std::ldexp(static_cast<double>(r >> 11) / 9007199254740992.0, static_cast<int>(r % 64) - 40);
        check((r & 1u) != 0 ? -v : v);
    }
}
#endif

TEST(Half, nextAfterStepsTheBitsAsGdalDoes) {
    EXPECT_EQ(HalfNextAfter(0x3C00, kHalfMaxBits), 0x3C01);  // up from 1
    EXPECT_EQ(HalfNextAfter(0xBC00, kHalfMaxBits), 0xBBFF);  // up from -1: towards 0
    EXPECT_EQ(HalfNextAfter(kHalfMaxBits, 0x0000), 0x7BFE);  // down from the largest
    EXPECT_EQ(HalfNextAfter(0x0000, kHalfMaxBits), 0x0001);  // from 0: the smallest subnormal
    EXPECT_EQ(HalfNextAfter(0x3C00, 0x3C00), 0x3C00);
}

// GDALDataTypeUnion never gives Float16 ("not supported everywhere"): GDAL warps halves in Float32
TEST(Half, theUnionOfAHalfIsAFloat) {
    using cog::tiff::SampleType;
    using cog::tiff::SampleTypeUnion;
    EXPECT_EQ(SampleTypeUnion(SampleType::kF16, SampleType::kF16), SampleType::kF32);
    EXPECT_EQ(SampleTypeUnion(SampleType::kU8, SampleType::kF16), SampleType::kF32);
    EXPECT_EQ(SampleTypeUnion(SampleType::kI32, SampleType::kF16), SampleType::kF64);  // max(16, 2 x 32)
    EXPECT_EQ(SampleTypeUnion(SampleType::kI64, SampleType::kI64), SampleType::kI64);
    EXPECT_EQ(SampleTypeUnion(SampleType::kU64, SampleType::kF32), SampleType::kF64);
}

}  // namespace
}  // namespace ddbt::util
