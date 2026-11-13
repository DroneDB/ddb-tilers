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

/// stretchRange: mean +- 2.33 sigma clamped to [min, max] (min..max when that is empty, then
/// lo + 1 when still empty).
std::pair<double, double> StretchRange(const BandStatistics& s);

/// Stretches n native samples (PixelTypeSize(type) bytes each) to bytes at dst, every `stride`
/// bytes. 8 / 16-bit samples go through a per-thread 2^bits LUT of StretchToByte (same values);
/// viaFloat: the samples were read as Float32 first (DroneDB readSelectedBands), so 32-bit
/// integers round to float before the stretch.
void StretchSamples(PixelType type, const uint8_t* src, size_t n, double lo, double hi, bool viaFloat, uint8_t* dst,
                    size_t stride);

}  // namespace ddbt::cog::tiler
