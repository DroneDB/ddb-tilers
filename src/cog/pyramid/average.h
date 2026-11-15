// SPDX-License-Identifier: MPL-2.0
//
// Overview resampling by GDAL's AVERAGE of 8-bit samples (gcore/overview.cpp,
// GDALResampleChunk_AverageOrRMS_T with UInt8 work data), each level from the previous one as
// GDALRegenerateOverviewsMultiBand computes it:
//
//   - Destination pixel i of an axis of srcSize -> dstSize pixels covers the source
//     [i * ratio, (i + 1) * ratio) (ratio = srcSize / dstSize): its first and last source pixels
//     weigh the part of them it covers (1 when it covers one pixel), the others 1.
//   - Without a mask: the weighted sum of the samples (per row: first, middle, last pixel; the
//     rows weighted the same way) over the product of the weights of the axes, + 0.5, truncated.
//   - With a mask (an alpha band or a dataset mask, a band's nodata): only the pixels whose mask
//     is not 0 count, the weights of the counted pixels summed as they are counted; a destination
//     pixel that counts none gets the nodata value (0 without one).
//   - A computed value equal to the nodata value is replaced (+ 1, 254 for 255).
//   - GDAL's fast path, without a mask: where the columns are a half of the source (2 whole pixels
//     each) and a destination row covers 2 source rows, the four samples average to
//     (sum + 2) / 4, the weights of the rows left aside, and the nodata value is not replaced.
//
// The nodata value is GDAL's cast of it to 8 bits (static_cast of a double: a value outside 0..255
// wraps as an int32 truncated to its low byte), which a band whose nodata Byte cannot hold also
// has (its mask is all valid: MaskMode::kNone, the replacement still applies).
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "cog/pyramid/overview.h"

namespace ddbt::cog::pyramid {

/// The source pixels of destination indices [dst0, dst1) of an axis: start and count, and the
/// weights GDAL gives them (the first, the last, and the sum of all of them).
struct AverageTaps {
    int dst0 = 0;
    bool half = false;  // the source is twice the destination: every destination pixel covers 2 whole pixels
    std::vector<int> start, count;
    std::vector<double> first, last, total;
    int SrcBegin() const;  // first source index any destination reads
    int SrcEnd() const;    // one past the last
};
AverageTaps AverageAxis(int srcSize, int dstSize, int dst0, int dst1);

/// GDAL's cast of a nodata value to an 8-bit sample (static_cast<GByte> of the double, as x86 does it)
uint8_t CastNoData(double nodata);

/// One destination window of every band of 8-bit samples, with the window conventions of
/// Downsample (overview.h): spec.type must be kU8. spec.mask: kNone (every pixel valid),
/// kExternal (srcMask, srcW x srcH bytes: the pixels whose value is not 0; nullptr: all of them,
/// as GDAL's all-valid mask band of an alpha band), kNoData (per band: the samples that are not
/// the nodata value).
void Average(const DownsampleSpec& spec, const uint8_t* src, size_t srcStride, int srcX0, int srcY0, int srcW,
             int srcH, const uint8_t* srcMask, const AverageTaps& cols, const AverageTaps& rows, int dstW, int dstH,
             uint8_t* dst, size_t dstStride);

}  // namespace ddbt::cog::pyramid
