// SPDX-License-Identifier: MPL-2.0
//
// The warped COG of a source in any CRS: the target grid (GDAL's GDALSuggestedWarpOutput2 and
// the COG driver's GoogleMapsCompatible zoom / extent, ZOOM_LEVEL_STRATEGY=UPPER), the output
// plan (cog.cpp's transparency and compression rules) and the producer of its full-resolution
// strips (GDAL's bilinear warp kernel).
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "cog/build/pipeline.h"
#include "cog/build/source.h"
#include "cog/warp/warp_chunks.h"
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
Expected<build::Plan> WarpPlan(const build::RasterSource& source, const WarpSettings& settings);

/// The producer of the plan's full-resolution strips.
Expected<std::unique_ptr<build::TileProducer>> MakeWarpProducer(const build::RasterSource& source, const build::Plan& plan,
                                                                uint64_t memoryBudget);

/// The chunks of GDAL's warper the producer follows for a source georeferenced by GCPs (none for a
/// source with a geotransform): for tests.
Expected<std::vector<WarpChunk>> GdalWarpChunks(const build::RasterSource& source, const build::Plan& plan);

}  // namespace ddbt::cog::warp
