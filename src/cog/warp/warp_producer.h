// SPDX-License-Identifier: MPL-2.0
//
// The warped COG of a source in any CRS: the target grid (GDAL's GDALSuggestedWarpOutput2 and
// the COG driver's GoogleMapsCompatible zoom / extent, ZOOM_LEVEL_STRATEGY=UPPER), the output
// plan (cog.cpp's transparency and compression rules) and the producer of its full-resolution
// strips (GDAL's bilinear warp kernel).
#pragma once

#include <cstdint>
#include <memory>

#include "cog/build/pipeline.h"
#include "cog/build/source.h"
#include "core/util/status.h"

namespace ddbt::cog::warp {

struct WarpSettings {
    encode::Codec losslessCodec = encode::Codec::kLzw;
    int level = 0;
    int jpegQuality = 75;
    bool noJpeg = false;
    bool evenTileOrigin = false;
};

/// The plan of the warped COG of `source` (everything but the GDAL metadata).
Expected<build::Plan> WarpPlan(const build::SourceRaster& source, const WarpSettings& settings);

/// The producer of the plan's full-resolution strips.
Expected<std::unique_ptr<build::TileProducer>> MakeWarpProducer(const build::SourceRaster& source, const build::Plan& plan,
                                                                uint64_t memoryBudget);

}  // namespace ddbt::cog::warp
