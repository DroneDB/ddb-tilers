// SPDX-License-Identifier: MPL-2.0
//
// The stretch of non-Byte bands to bytes (DroneDB vis_render.h stretchToByte / stretchRange).
#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>

#include "cog/tiler/cog_source.h"
#include "cog/tiler/stats.h"

namespace ddbt::cog::tiler {

/// stretchToByte: v clipped to [lo, hi] (NaN gives hi: 255), 255 * (c - lo) / (hi - lo) truncated.
uint8_t StretchToByte(double v, double lo, double hi);

/// Whether the values of a band of `type` from lo to hi are one value as far as the arithmetic
/// can tell: equal, or, for floating-point samples, apart by at most 2^-42 of their magnitude
/// (about a thousand units in the last place of a double). Resampling a constant raster (the warp
/// and the overviews of a COG, as GDAL's kernels do it) leaves its values a few dozen units in
/// the last place apart, and stretching that span to 0-255 draws noise. Real data is never that
/// flat: two different Float32 values are at least 2^-24 of their magnitude apart, a DEM that
/// spans a centimetre at 9000 m spans 1e-6 of it. Integer samples are whole numbers (a span of 1
/// is real data), so for them only equal values are one value.
bool IsConstantSpan(double lo, double hi, PixelType type);

/// stretchRange: mean +- 2.33 sigma clamped to [min, max] (min..max when that is empty), or
/// min to min + 1 when the band is constant (IsConstantSpan of its min and max).
std::pair<double, double> StretchRange(const BandStatistics& s, PixelType type);

/// Stretches n native samples (PixelTypeSize(type) bytes each) to bytes at dst, every `stride`
/// bytes. 8 / 16-bit samples go through a per-thread 2^bits LUT of StretchToByte (same values);
/// viaFloat: the samples were read as Float32 first (DroneDB readSelectedBands), so 32-bit
/// integers round to float before the stretch.
void StretchSamples(PixelType type, const uint8_t* src, size_t n, double lo, double hi, bool viaFloat, uint8_t* dst,
                    size_t stride);

}  // namespace ddbt::cog::tiler
