// SPDX-License-Identifier: MPL-2.0
//
// GDAL's AVERAGE of 8-bit samples (pyramid/average.h): the source pixels and weights of each
// destination pixel, the rounding, the mask and the nodata rules.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "cog/pyramid/average.h"

namespace {

using namespace ddbt::cog::pyramid;

// 5 -> 3: destination i covers [5i/3, 5(i+1)/3); its first and last pixels weigh what it covers
TEST(Average, anAxisOfAnOddRatioWeighsThePixelsItCoversInPart) {
    const AverageTaps t = AverageAxis(5, 3, 0, 3);
    ASSERT_EQ(t.start, (std::vector<int>{0, 1, 3}));
    ASSERT_EQ(t.count, (std::vector<int>{2, 3, 2}));
    const double third = 5.0 / 3.0;
    EXPECT_DOUBLE_EQ(t.first[0], 1.0);
    EXPECT_DOUBLE_EQ(t.last[0], 1 - (2 - third));
    EXPECT_DOUBLE_EQ(t.first[1], 1 - (third - 1));
    EXPECT_DOUBLE_EQ(t.last[1], 1 - (4 - 2 * third));
    EXPECT_DOUBLE_EQ(t.total[1], t.first[1] + 1 + t.last[1]);
    EXPECT_DOUBLE_EQ(t.last[2], 1.0);
    EXPECT_EQ(t.SrcBegin(), 0);
    EXPECT_EQ(t.SrcEnd(), 5);
}

DownsampleSpec Spec(MaskMode mask, int bands = 1) {
    DownsampleSpec s;
    s.type = ddbt::cog::tiff::SampleType::kU8;
    s.bands = bands;
    s.mask = mask;
    return s;
}

uint8_t AverageOf2x2(const DownsampleSpec& spec, std::vector<uint8_t> src, const uint8_t* mask = nullptr) {
    const AverageTaps axis = AverageAxis(2, 1, 0, 1);
    uint8_t out = 0;
    Average(spec, src.data(), 2, 0, 0, 2, 2, mask, axis, axis, 1, 1, &out, 1);
    return out;
}

TEST(Average, fourSamplesRoundHalfUpAsGdalsFastPath) {
    EXPECT_EQ(AverageOf2x2(Spec(MaskMode::kNone), {1, 2, 3, 4}), 3);      // (10 + 2) / 4
    EXPECT_EQ(AverageOf2x2(Spec(MaskMode::kNone), {1, 2, 3, 3}), 2);      // (9 + 2) / 4
    EXPECT_EQ(AverageOf2x2(Spec(MaskMode::kNone), {255, 255, 255, 254}), 255);
}

TEST(Average, aMaskLeavesItsZeroPixelsOutAndAPixelWithoutAnyIsTheNodata) {
    const uint8_t mask[4] = {255, 255, 255, 0};
    EXPECT_EQ(AverageOf2x2(Spec(MaskMode::kExternal), {10, 20, 30, 200}, mask), 20);
    const uint8_t none[4] = {0, 0, 0, 0};
    EXPECT_EQ(AverageOf2x2(Spec(MaskMode::kExternal), {10, 20, 30, 200}, none), 0);
    // No mask under kExternal: every pixel (the all-valid mask of an alpha band)
    EXPECT_EQ(AverageOf2x2(Spec(MaskMode::kExternal), {10, 20, 30, 200}, nullptr), 65);
}

TEST(Average, theNodataValueIsLeftOutAndAComputedNodataIsReplaced) {
    DownsampleSpec spec = Spec(MaskMode::kNoData);
    spec.hasNoData = true;
    spec.noData = 7;
    EXPECT_EQ(AverageOf2x2(spec, {7, 20, 30, 40}), 30);  // 7 left out
    EXPECT_EQ(AverageOf2x2(spec, {7, 7, 7, 7}), 7);      // no value: the nodata
    EXPECT_EQ(AverageOf2x2(spec, {6, 8, 7, 7}), 8);      // 7 computed: replaced by 8
    spec.noData = 255;
    EXPECT_EQ(AverageOf2x2(spec, {254, 255, 255, 255}), 254);
    EXPECT_EQ(AverageOf2x2(spec, {255, 254, 255, 255}), 254);
}

}  // namespace
