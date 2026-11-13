// SPDX-License-Identifier: MPL-2.0
//
// Overview resampling as GDAL's COG driver computes it: GDALRegenerateOverviewsMultiBand with
// CUBIC (gcore/overview.cpp, GDALResampleChunk_ConvolutionT), each level from the previous one.
//
//   - Separable convolution with the Catmull-Rom cubic kernel (a = -0.5, radius 2), the kernel
//     stretched by the ratio source / destination size (8 taps at a factor of 2); destination
//     pixel i is centred on source (i + 0.5) * ratio; taps clipped at the image edges and the
//     weights normalised over the taps that remain. Horizontal pass in double, vertical pass in
//     double, then the work type (float for 8 / 16-bit and Float32 data, double otherwise), then
//     the sample type (rounded half away from zero and clamped for integers).
//   - With a mask (an internal mask, an alpha band, or the band's nodata): each tap's weight is
//     multiplied by its mask value (0..255), the result divided by the masked weight sum; a
//     destination pixel whose taps have no run of valid taps at least half as long as the taps is
//     invalid (the rule for kernels with negative weights), as is one without valid tap: it gets
//     the nodata value (0 without nodata). A computed value equal to the nodata value is replaced
//     (GDALGetNoDataReplacementValue). NaN samples weigh 0 under a mask.
//   - The mask of the next level is the same resampling of the mask (as an 8-bit band) with
//     itself as its weights: values stay 0..255 (written as 1 bit: non-zero = valid).
//
// Work is done per destination window, from a source window that holds every tap: the result
// does not depend on the window split (GDAL's chunks do not either).
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "cog/tiff/tiff_reader.h"

namespace ddbt::cog::pyramid {

/// Cubic taps of destination indices [dst0, dst1) of an axis of `srcSize` -> `dstSize` pixels:
/// absolute source start and count per destination index, raw kernel weights (not normalised).
struct Taps {
    int dst0 = 0;
    int maxTaps = 0;
    std::vector<int> start, count;
    std::vector<double> weights;      // maxTaps per destination index
    std::vector<double> invSum;       // 1 / sum of the weights (0 when the sum is 0)
    int SrcBegin() const;             // first source index any tap reads
    int SrcEnd() const;               // one past the last
};
Taps CubicTaps(int srcSize, int dstSize, int dst0, int dst1);

/// The catmull-rom kernel GDAL uses (gdalresamplingkernels.h CubicKernel).
double CubicKernel(double x);

enum class MaskMode : uint8_t {
    kNone,      // every pixel valid
    kExternal,  // a per-pixel 8-bit mask (0..255) given with the source window
    kNoData,    // per band: valid when the sample is not the nodata value (NaN nodata: not NaN)
};

struct DownsampleSpec {
    tiff::SampleType type = tiff::SampleType::kU8;
    int bands = 1;            // samples per pixel (interleaved)
    MaskMode mask = MaskMode::kNone;
    bool hasNoData = false;
    double noData = 0;
};

/// One destination window [cols.dst0, +dstW) x [rows.dst0, +dstH) of every band.
/// src: the source window, interleaved samples, its top-left source pixel at (srcX0, srcY0),
/// srcW x srcH pixels, rows srcStride bytes apart; it must cover cols.SrcBegin()..SrcEnd() and
/// rows.SrcBegin()..SrcEnd(). srcMask: srcW x srcH bytes (rows of srcW) for MaskMode::kExternal.
/// dst: interleaved, rows dstStride bytes apart.
void Downsample(const DownsampleSpec& spec, const uint8_t* src, size_t srcStride, int srcX0, int srcY0, int srcW,
                int srcH, const uint8_t* srcMask, const Taps& cols, const Taps& rows, int dstW, int dstH, uint8_t* dst,
                size_t dstStride);

/// The next level of an 8-bit mask (its own weights), same window conventions; dst rows of
/// dstStride bytes.
void DownsampleMask(const uint8_t* src, int srcX0, int srcY0, int srcW, int srcH, const Taps& cols, const Taps& rows,
                    int dstW, int dstH, uint8_t* dst, size_t dstStride);

/// The scalar implementation of Downsample and DownsampleMask: one destination pixel at a time,
/// in the order of GDAL's loops. Downsample computes the same bits with SIMD passes
/// (overview_kernels.h); this one is kept as the reference the tests hold them to.
namespace reference {
void Downsample(const DownsampleSpec& spec, const uint8_t* src, size_t srcStride, int srcX0, int srcY0, int srcW,
                int srcH, const uint8_t* srcMask, const Taps& cols, const Taps& rows, int dstW, int dstH, uint8_t* dst,
                size_t dstStride);
void DownsampleMask(const uint8_t* src, int srcX0, int srcY0, int srcW, int srcH, const Taps& cols, const Taps& rows,
                    int dstW, int dstH, uint8_t* dst, size_t dstStride);
}  // namespace reference

/// GDAL's COG overview sizes. Tile-matrix grids (a warp to GoogleMapsCompatible): halved with
/// rounding (int(n / 2 + 0.5)), while either side is larger than the block size. Own grids
/// (OVERVIEWS=IGNORE_EXISTING of a COG re-encoded at its grid): halved with truncation.
std::vector<std::pair<uint32_t, uint32_t>> OverviewSizes(uint32_t width, uint32_t height, uint32_t blockSize,
                                                         bool tileMatrix);

}  // namespace ddbt::cog::pyramid
