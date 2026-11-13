// SPDX-License-Identifier: MPL-2.0
//
// Predictors and byte swapping against the scalar specification, on every Highway target the
// CPU supports, for every sample width, a range of samples per pixel (including more samples
// than vector lanes) and odd widths (vector tails).

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "cog/codecs/predictor.h"
#include "core/mem/aligned.h"
#include "core/simd/simd.h"

namespace ddbt::cog::codecs {
namespace {

struct ForcedTargetGuard {
    ~ForcedTargetGuard() { simd::ForceTargetForTest(0); }
};

std::vector<uint8_t> RandomBytes(size_t n, uint32_t seed) {
    std::mt19937 rng(seed);
    std::vector<uint8_t> v(n);
    for (auto& b : v) b = static_cast<uint8_t>(rng());
    return v;
}

class PredictorTargets : public ::testing::TestWithParam<int64_t> {};

TEST_P(PredictorTargets, HorizontalMatchesReference) {
    ForcedTargetGuard guard;
    simd::ForceTargetForTest(GetParam());
    for (size_t bps : {size_t{1}, size_t{2}, size_t{4}, size_t{8}}) {
        for (size_t spp : std::vector<size_t>{1, 2, 3, 4, 5, 7, 8, 16, 33, 52, 70}) {
            for (size_t width : std::vector<size_t>{1, 2, 3, 15, 16, 17, 63, 64, 65, 255, 512}) {
                const size_t rows = 3;
                const size_t rowBytes = width * spp * bps;
                const size_t stride = rowBytes + 8 * bps;  // padding between rows
                auto data = RandomBytes(stride * rows, static_cast<uint32_t>(bps * 1000003 + spp * 1009 + width));
                mem::AlignedBytes a = mem::MakeAlignedBytes(data.size());
                mem::AlignedBytes b = mem::MakeAlignedBytes(data.size());
                std::memcpy(a.get(), data.data(), data.size());
                std::memcpy(b.get(), data.data(), data.size());
                UndoHorizontalPredictor(a.get(), rows, stride, width, spp, bps);
                reference::UndoHorizontalPredictor(b.get(), rows, stride, width, spp, bps);
                ASSERT_EQ(0, std::memcmp(a.get(), b.get(), data.size()))
                    << simd::TargetNameOf(GetParam()) << " bps " << bps << " spp " << spp << " width " << width;
            }
        }
    }
}

TEST_P(PredictorTargets, FloatingPointMatchesReference) {
    ForcedTargetGuard guard;
    simd::ForceTargetForTest(GetParam());
    for (size_t bps : std::vector<size_t>{2, 4, 8}) {
        for (size_t spp : std::vector<size_t>{1, 2, 3, 4, 52}) {
            for (size_t width : std::vector<size_t>{1, 7, 16, 31, 64, 100, 256}) {
                const size_t rows = 2;
                const size_t stride = width * spp * bps;
                auto data = RandomBytes(stride * rows, static_cast<uint32_t>(bps * 7 + spp * 131 + width));
                mem::AlignedBytes a = mem::MakeAlignedBytes(data.size());
                mem::AlignedBytes b = mem::MakeAlignedBytes(data.size());
                std::memcpy(a.get(), data.data(), data.size());
                std::memcpy(b.get(), data.data(), data.size());
                UndoFloatingPointPredictor(a.get(), rows, stride, width, spp, bps);
                reference::UndoFloatingPointPredictor(b.get(), rows, stride, width, spp, bps);
                ASSERT_EQ(0, std::memcmp(a.get(), b.get(), data.size()))
                    << simd::TargetNameOf(GetParam()) << " bps " << bps << " spp " << spp << " width " << width;
            }
        }
    }
}

TEST_P(PredictorTargets, ByteSwapMatchesReference) {
    ForcedTargetGuard guard;
    simd::ForceTargetForTest(GetParam());
    for (size_t bps : {size_t{1}, size_t{2}, size_t{4}, size_t{8}}) {
        for (size_t count : std::vector<size_t>{0, 1, 3, 31, 32, 33, 1000}) {
            auto data = RandomBytes(count * bps, static_cast<uint32_t>(count + bps));
            mem::AlignedBytes a = mem::MakeAlignedBytes(data.size() + 1);
            if (!data.empty()) std::memcpy(a.get(), data.data(), data.size());
            ByteSwapSamples(a.get(), count, bps);
            reference::ByteSwapSamples(data.data(), count, bps);
            if (!data.empty()) {
                ASSERT_EQ(0, std::memcmp(a.get(), data.data(), data.size())) << bps << " " << count;
            }
        }
    }
}

std::vector<int64_t> Targets() { return simd::SupportedTargetBits(); }

INSTANTIATE_TEST_SUITE_P(AllTargets, PredictorTargets, ::testing::ValuesIn(Targets()),
                         [](const ::testing::TestParamInfo<int64_t>& p) { return simd::TargetNameOf(p.param); });

// Known values: libtiff's horizontal differencing of a 16-bit RGB row and the floating-point
// predictor of two float32 samples.
TEST(Predictor, KnownValues) {
    alignas(8) uint16_t row[6] = {100, 200, 300, 5, 65535, 1};  // pixel 2 is pixel 1 + deltas
    UndoHorizontalPredictor(reinterpret_cast<uint8_t*>(row), 1, sizeof(row), 2, 3, 2);
    EXPECT_EQ(row[3], 105);
    EXPECT_EQ(row[4], 199);  // 200 + 65535 wraps
    EXPECT_EQ(row[5], 301);

    // Two float32 samples 1.0f (3f800000) and -2.5f (c0200000), one sample per pixel:
    // byte planes MSB first: [3f c0] [80 20] [00 00] [00 00], then differenced with stride 1.
    const uint8_t planes[8] = {0x3f, 0xc0, 0x80, 0x20, 0x00, 0x00, 0x00, 0x00};
    alignas(4) uint8_t enc[8];
    enc[0] = planes[0];
    for (int i = 1; i < 8; ++i) enc[i] = static_cast<uint8_t>(planes[i] - planes[i - 1]);
    UndoFloatingPointPredictor(enc, 1, 8, 2, 1, 4);
    float f[2];
    std::memcpy(f, enc, 8);
    EXPECT_EQ(f[0], 1.0f);
    EXPECT_EQ(f[1], -2.5f);
}

}  // namespace
}  // namespace ddbt::cog::codecs
