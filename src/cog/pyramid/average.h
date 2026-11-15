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
//
// The result of GDAL's 2 x 2 fast path is the same: the four samples average to (sum + 2) / 4.
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
    std::vector<int> start, count;
    std::vector<double> first, last, total;
    int SrcBegin() const;  // first source index any destination reads
    int SrcEnd() const;    // one past the last
};
AverageTaps AverageAxis(int srcSize, int dstSize, int dst0, int dst1);

/// One destination window of every band of 8-bit samples, with the window conventions of
/// Downsample (overview.h): spec.type must be kU8. spec.mask: kNone (every pixel valid),
/// kExternal (srcMask, srcW x srcH bytes: the pixels whose value is not 0; nullptr: all of them,
/// as GDAL's all-valid mask band of an alpha band), kNoData (per band: the samples that are not
/// the nodata value).
void Average(const DownsampleSpec& spec, const uint8_t* src, size_t srcStride, int srcX0, int srcY0, int srcW,
             int srcH, const uint8_t* srcMask, const AverageTaps& cols, const AverageTaps& rows, int dstW, int dstH,
             uint8_t* dst, size_t dstStride);

}  // namespace ddbt::cog::pyramid
