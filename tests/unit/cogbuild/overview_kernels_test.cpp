// SPDX-License-Identifier: MPL-2.0
//
// The SIMD overview passes against the scalar reference, on every Highway target the CPU runs:
// the destination bytes must be identical for every sample type, band count, mask mode (none,
// an external mask with partial values, all valid, all invalid, nodata per band including NaN
// and the type's extremes), special floating-point samples (NaN, infinities, signed zeros,
// subnormals), factor-two, odd, rounded and factor-three sizes, windows inside the image and at
// its edges, and widths and heights that are not multiples of the SIMD blocks.

#include <gtest/gtest.h>

// Index arithmetic of the synthetic images mixes int and size_t freely.
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wconversion"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "cog/pyramid/overview.h"
#include "core/simd/simd.h"

namespace ddbt::cog {
namespace {

using pyramid::MaskMode;
using tiff::SampleType;

struct ForcedTargetGuard {
    ~ForcedTargetGuard() { ddbt::simd::ForceTargetForTest(0); }
};

size_t SizeOf(SampleType t) {
    switch (t) {
        case SampleType::kU8:
        case SampleType::kI8: return 1;
        case SampleType::kU16:
        case SampleType::kI16: return 2;
        case SampleType::kU32:
        case SampleType::kI32:
        case SampleType::kF32: return 4;
        default: return 8;
    }
}

bool IsFloat(SampleType t) { return t == SampleType::kF32 || t == SampleType::kF64; }

bool IsNaNSample(SampleType t, const uint8_t* p) {
    if (t == SampleType::kF32) {
        float v;
        std::memcpy(&v, p, sizeof v);
        return std::isnan(v);
    }
    if (t == SampleType::kF64) {
        double v;
        std::memcpy(&v, p, sizeof v);
        return std::isnan(v);
    }
    return false;
}

template <typename T>
void Put(uint8_t* p, T v) {
    std::memcpy(p, &v, sizeof(T));
}

// Writes `v` (a value of the sample type's range) as a sample of type t.
void PutValue(SampleType t, uint8_t* p, double v) {
    switch (t) {
        case SampleType::kU8: Put(p, static_cast<uint8_t>(v)); break;
        case SampleType::kI8: Put(p, static_cast<int8_t>(v)); break;
        case SampleType::kU16: Put(p, static_cast<uint16_t>(v)); break;
        case SampleType::kI16: Put(p, static_cast<int16_t>(v)); break;
        case SampleType::kU32: Put(p, static_cast<uint32_t>(v)); break;
        case SampleType::kI32: Put(p, static_cast<int32_t>(v)); break;
        case SampleType::kF32: Put(p, static_cast<float>(v)); break;
        default: Put(p, v); break;
    }
}

std::pair<double, double> Range(SampleType t) {
    switch (t) {
        case SampleType::kU8: return {0, 255};
        case SampleType::kI8: return {-128, 127};
        case SampleType::kU16: return {0, 65535};
        case SampleType::kI16: return {-32768, 32767};
        case SampleType::kU32: return {0, 4294967295.0};
        case SampleType::kI32: return {-2147483648.0, 2147483647.0};
        case SampleType::kF32: return {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max()};
        default: return {std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max()};
    }
}

// A sample: mostly smooth values (an orthophoto's), with the extremes, the nodata value and,
// for floating point, NaN, infinities, signed zeros and subnormals.
double SampleValue(SampleType t, std::mt19937& rng, int x, int y, int b, bool specials, double nd) {
    const auto [lo, hi] = Range(t);
    const uint32_t pick = rng() % 100;
    if (pick < 3) return lo;
    if (pick < 6) return hi;
    if (pick < 9 && !std::isnan(nd)) return nd;
    if (specials && IsFloat(t)) {
        if (pick < 11) return std::numeric_limits<double>::quiet_NaN();
        if (pick < 12) return std::numeric_limits<double>::infinity();
        if (pick < 13) return -std::numeric_limits<double>::infinity();
        if (pick < 14) return -0.0;
        if (pick < 15) return t == SampleType::kF32 ? 1e-40 : 1e-310;
    }
    if (pick < 40) {
        // Random over the whole range (integers: exact; floats: wide magnitudes).
        if (IsFloat(t)) return (static_cast<double>(rng()) / 4294967296.0 - 0.5) * std::pow(10.0, rng() % 12);
        return std::floor(lo + (hi - lo) * (static_cast<double>(rng()) / 4294967296.0));
    }
    // Smooth: a gradient with small noise, inside the range.
    const double span = IsFloat(t) ? 1000.0 : std::min(hi - lo, 60000.0);
    const double base = IsFloat(t) ? 0.0 : lo;
    const double v = base + std::fmod(x * 7.3 + y * 3.1 + b * 41.0 + (rng() % 9), span);
    return IsFloat(t) ? v + 0.37 : std::floor(v);
}

struct Geometry {
    int srcSize[2];  // full image width, height
    int dstSize[2];
    int dst0[2], dst1[2];  // destination window
    int margin;            // source columns / rows beyond the taps in the window (left / top)
};

enum class MaskKind { kNone, kPartial, kAllValid, kAllInvalid, kNoData, kNoDataNan, kNoDataMax, kNoDataLowest };

struct Case {
    SampleType type;
    int bands;
    MaskKind mask;
    Geometry g;
    uint32_t seed;
};

std::string Describe(const Case& c) {
    return "type " + std::to_string(static_cast<int>(c.type)) + " bands " + std::to_string(c.bands) + " mask " +
           std::to_string(static_cast<int>(c.mask)) + " src " + std::to_string(c.g.srcSize[0]) + "x" +
           std::to_string(c.g.srcSize[1]) + " dst " + std::to_string(c.g.dstSize[0]) + "x" +
           std::to_string(c.g.dstSize[1]) + " window " + std::to_string(c.g.dst0[0]) + "," +
           std::to_string(c.g.dst0[1]) + "-" + std::to_string(c.g.dst1[0]) + "," + std::to_string(c.g.dst1[1]);
}

// Runs one case through Downsample and reference::Downsample (and the masks' own level through
// DownsampleMask and its reference) and compares every destination byte.
void CheckCase(const Case& c) {
    SCOPED_TRACE(Describe(c));
    const Geometry& g = c.g;
    const auto cols = pyramid::CubicTaps(g.srcSize[0], g.dstSize[0], g.dst0[0], g.dst1[0]);
    const auto rows = pyramid::CubicTaps(g.srcSize[1], g.dstSize[1], g.dst0[1], g.dst1[1]);
    const int x0 = std::max(0, cols.SrcBegin() - g.margin), y0 = std::max(0, rows.SrcBegin() - g.margin);
    const int sw = cols.SrcEnd() - x0, sh = rows.SrcEnd() - y0;
    const size_t ss = SizeOf(c.type);
    const size_t px = ss * static_cast<size_t>(c.bands);
    const size_t stride = static_cast<size_t>(sw) * px + 3;  // rows not packed

    pyramid::DownsampleSpec spec;
    spec.type = c.type;
    spec.bands = c.bands;
    const auto [lo, hi] = Range(c.type);
    double nd = std::numeric_limits<double>::quiet_NaN();  // no nodata value among the samples
    switch (c.mask) {
        case MaskKind::kNone: spec.mask = MaskMode::kNone; break;
        case MaskKind::kPartial:
        case MaskKind::kAllValid:
        case MaskKind::kAllInvalid: spec.mask = MaskMode::kExternal; break;
        case MaskKind::kNoData: nd = IsFloat(c.type) ? -9999.5 : std::floor((lo + hi) / 3); break;
        case MaskKind::kNoDataNan: nd = std::numeric_limits<double>::quiet_NaN(); break;
        case MaskKind::kNoDataMax: nd = hi; break;
        case MaskKind::kNoDataLowest: nd = lo; break;
    }
    if (c.mask >= MaskKind::kNoData) {
        spec.mask = MaskMode::kNoData;
        spec.hasNoData = true;
        spec.noData = nd;
    }

    std::mt19937 rng(c.seed);
    std::vector<uint8_t> src(stride * static_cast<size_t>(sh), 0xAB);
    const bool specials = c.mask != MaskKind::kNone || c.seed % 2 == 0;
    // A region of constant samples (a uniform area, where a computed value can equal nodata).
    for (int y = 0; y < sh; ++y)
        for (int x = 0; x < sw; ++x)
            for (int b = 0; b < c.bands; ++b) {
                const bool flat = x < sw / 4 && y < sh / 3;
                const double v =
                    flat ? (std::isnan(nd) ? lo + 1 : nd) : SampleValue(c.type, rng, x, y, b, specials, nd);
                PutValue(c.type, src.data() + static_cast<size_t>(y) * stride + static_cast<size_t>(x) * px + ss * b,
                         v);
            }
    std::vector<uint8_t> mask(static_cast<size_t>(sw) * static_cast<size_t>(sh), 255);
    for (int y = 0; y < sh; ++y)
        for (int x = 0; x < sw; ++x) {
            uint8_t& m = mask[static_cast<size_t>(y) * sw + x];
            if (c.mask == MaskKind::kAllInvalid) {
                m = 0;
            } else if (c.mask == MaskKind::kPartial) {
                // A valid band on the left, a ragged edge with partial values and holes, a few
                // valid pixels, then nothing valid.
                const int edge = sw / 2 + static_cast<int>(rng() % 7) - 3;
                const uint32_t r = rng() % 20;
                if (x < sw / 3) m = 255;
                else if (x <= edge) m = r < 4 ? 0 : r < 8 ? static_cast<uint8_t>(1 + rng() % 254) : 255;
                else if (x <= edge + 9) m = r < 4 ? 255 : 0;
                else m = 0;
            }
        }

    const int dw = g.dst1[0] - g.dst0[0], dh = g.dst1[1] - g.dst0[1];
    const size_t dstStride = static_cast<size_t>(dw) * px + 5;
    std::vector<uint8_t> got(dstStride * static_cast<size_t>(dh), 0xCD), want(got);
    const uint8_t* m = spec.mask == MaskMode::kExternal ? mask.data() : nullptr;
    pyramid::Downsample(spec, src.data(), stride, x0, y0, sw, sh, m, cols, rows, dw, dh, got.data(), dstStride);
    pyramid::reference::Downsample(spec, src.data(), stride, x0, y0, sw, sh, m, cols, rows, dw, dh, want.data(),
                                   dstStride);
    // Byte for byte, with one exception: without a mask, a floating-point sum that meets two
    // different NaNs (a NaN sample and the NaN of inf - inf, or NaNs of both signs) returns the
    // NaN of whichever operand the compiler placed first, and the scalar code does not place
    // them consistently (no fixed operand order reproduces it), so only the NaN-ness of such a
    // sample is defined. Under a mask NaN samples weigh nothing and every NaN is the default
    // one, so masked outputs are compared exactly, NaNs included.
    const bool nanPayloadFree = spec.mask == MaskMode::kNone && IsFloat(c.type);
    for (size_t y = 0; y < static_cast<size_t>(dh); ++y) {
        for (size_t x = 0; x < static_cast<size_t>(dw); ++x)
            for (size_t b = 0; b < static_cast<size_t>(c.bands); ++b) {
                const size_t at = y * dstStride + x * px + b * ss;
                if (std::memcmp(&got[at], &want[at], ss) == 0) continue;
                if (nanPayloadFree && IsNaNSample(c.type, &got[at]) && IsNaNSample(c.type, &want[at])) continue;
                std::string bytes;
                for (size_t k = 0; k < ss; ++k)
                    bytes += " " + std::to_string(got[at + k]) + "/" + std::to_string(want[at + k]);
                FAIL() << "first difference at destination pixel " << x << "," << y << " band " << b
                       << " (bytes got/want:" << bytes << ")";
            }
        // Nothing written past the row.
        const size_t tail = y * dstStride + static_cast<size_t>(dw) * px;
        ASSERT_EQ(0, std::memcmp(&got[tail], &want[tail], dstStride - static_cast<size_t>(dw) * px)) << "row " << y;
    }
    if (spec.mask == MaskMode::kExternal && c.bands == 1) {
        std::vector<uint8_t> gotMask(static_cast<size_t>(dw) * dh + 7, 0xCD), wantMask(gotMask);
        pyramid::DownsampleMask(mask.data(), x0, y0, sw, sh, cols, rows, dw, dh, gotMask.data(),
                                static_cast<size_t>(dw));
        pyramid::reference::DownsampleMask(mask.data(), x0, y0, sw, sh, cols, rows, dw, dh, wantMask.data(),
                                           static_cast<size_t>(dw));
        EXPECT_EQ(gotMask, wantMask) << "mask level";
    }
}

std::vector<Geometry> Geometries() {
    return {
        // Whole image, exact factor two, sizes that are not multiples of any SIMD block.
        {{74, 58}, {37, 29}, {0, 0}, {37, 29}, 0},
        // An interior window with spare source columns and rows on its left and top.
        {{300, 260}, {150, 130}, {41, 33}, {94, 70}, 3},
        // A window at the right and bottom edges (clipped taps).
        {{300, 260}, {150, 130}, {111, 97}, {150, 130}, 0},
        // Rounded halving of an odd size (a tile matrix grid): the ratio is just under two, so
        // tap counts and steps vary along the axis.
        {{257, 131}, {129, 66}, {0, 0}, {129, 66}, 0},
        // Truncated halving of an odd size (an own grid).
        {{131, 97}, {65, 48}, {0, 0}, {65, 48}, 1},
        // Factor three: twelve-tap kernels.
        {{90, 75}, {30, 25}, {0, 0}, {30, 25}, 0},
        // Single destination column / row.
        {{40, 40}, {20, 20}, {7, 0}, {8, 20}, 0},
        {{40, 40}, {20, 20}, {0, 19}, {20, 20}, 0},
    };
}

void CheckEverything() {
    const SampleType types[] = {SampleType::kU8,  SampleType::kI8,  SampleType::kU16, SampleType::kI16,
                                SampleType::kU32, SampleType::kI32, SampleType::kF32, SampleType::kF64};
    const MaskKind masks[] = {MaskKind::kNone,      MaskKind::kPartial,   MaskKind::kAllValid,
                              MaskKind::kAllInvalid, MaskKind::kNoData,    MaskKind::kNoDataNan,
                              MaskKind::kNoDataMax,  MaskKind::kNoDataLowest};
    uint32_t seed = 1;
    for (const SampleType t : types)
        for (const int bands : {1, 2, 3, 4, 6})
            for (const MaskKind mk : masks) {
                if (mk == MaskKind::kNoDataNan && !IsFloat(t)) continue;
                for (const Geometry& g : Geometries()) {
                    CheckCase({t, bands, mk, g, seed++});
                    if (::testing::Test::HasFatalFailure()) return;
                }
            }
}

TEST(OverviewKernels, DownsampleMatchesTheScalarReferenceOnEveryTarget) {
    ForcedTargetGuard guard;
    for (const int64_t bit : ddbt::simd::SupportedTargetBits()) {
        ddbt::simd::ForceTargetForTest(bit);
        SCOPED_TRACE(ddbt::simd::TargetNameOf(bit));
        CheckEverything();
        if (HasFatalFailure()) return;
    }
}

// An orthophoto tile as the builder downsamples it: RGB with a mask that is valid almost
// everywhere (the all-valid path) and a 0..255 border from the previous level, at the size of a
// 512 x 512 destination tile, so the block and column loops run many times.
TEST(OverviewKernels, OrthophotoTileWithMaskMatchesTheScalarReferenceOnEveryTarget) {
    ForcedTargetGuard guard;
    const auto cols = pyramid::CubicTaps(2 * 1024 - 1, 1024, 512, 1024);
    const auto rows = pyramid::CubicTaps(2 * 1024 - 1, 1024, 0, 512);
    const int x0 = cols.SrcBegin(), y0 = rows.SrcBegin();
    const int sw = cols.SrcEnd() - x0, sh = rows.SrcEnd() - y0;
    std::mt19937 rng(7);
    std::vector<uint8_t> px(static_cast<size_t>(sw) * sh * 3), mask(static_cast<size_t>(sw) * sh, 255);
    for (size_t i = 0; i < px.size(); ++i) px[i] = static_cast<uint8_t>((i * 37 / 101 + rng() % 5) & 0xFF);
    for (int y = 0; y < sh; ++y)
        for (int x = sw - 40 - y / 9; x < sw; ++x)
            mask[static_cast<size_t>(y) * sw + x] = x < sw - 36 - y / 9 ? static_cast<uint8_t>(60 * (x % 4)) : 0;
    pyramid::DownsampleSpec spec;
    spec.bands = 3;
    for (const int64_t bit : ddbt::simd::SupportedTargetBits()) {
        ddbt::simd::ForceTargetForTest(bit);
        SCOPED_TRACE(ddbt::simd::TargetNameOf(bit));
        for (const MaskMode mode : {MaskMode::kNone, MaskMode::kExternal}) {
            spec.mask = mode;
            std::vector<uint8_t> got(512 * 512 * 3), want(got.size());
            const uint8_t* m = mode == MaskMode::kExternal ? mask.data() : nullptr;
            pyramid::Downsample(spec, px.data(), static_cast<size_t>(sw) * 3, x0, y0, sw, sh, m, cols, rows, 512, 512,
                                got.data(), 512 * 3);
            pyramid::reference::Downsample(spec, px.data(), static_cast<size_t>(sw) * 3, x0, y0, sw, sh, m, cols, rows,
                                           512, 512, want.data(), 512 * 3);
            EXPECT_TRUE(got == want) << "mask mode " << static_cast<int>(mode);
        }
        std::vector<uint8_t> got(512 * 512), want(got.size());
        pyramid::DownsampleMask(mask.data(), x0, y0, sw, sh, cols, rows, 512, 512, got.data(), 512);
        pyramid::reference::DownsampleMask(mask.data(), x0, y0, sw, sh, cols, rows, 512, 512, want.data(), 512);
        EXPECT_TRUE(got == want) << "mask level";
    }
}

}  // namespace
}  // namespace ddbt::cog
