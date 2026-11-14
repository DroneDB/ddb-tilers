// SPDX-License-Identifier: MPL-2.0
//
// The warped COG of a source in any CRS: the target grid (GDAL's GDALSuggestedWarpOutput2 and
// the COG driver's GoogleMapsCompatible zoom / extent, ZOOM_LEVEL_STRATEGY=UPPER; or gdalwarp's
// grid in the source's CRS), the output plan (cog.cpp's transparency and compression rules, or
// gdalwarp -dstalpha's) and the producer of its full-resolution strips (GDAL's bilinear or
// nearest neighbour warp kernel).
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "cog/build/pipeline.h"
#include "cog/build/source.h"
#include "cog/warp/warp_chunks.h"
#include "core/util/status.h"

namespace ddbt::cog::warp {

/// The grid of the output.
enum class Target : uint8_t {
    kWebMercator,  ///< the GoogleMapsCompatible grid of EPSG:3857 (DroneDB's COGs)
    kSourceCrs,    ///< gdalwarp's grid in the source's own CRS (no -t_srs)
    /// The EPSG:3857 grid of the COG of the source's warp into its own CRS: the grid of libddb's
    /// COG of a photo (two warps), the pixels warped once from the source.
    kWebMercatorThroughSourceCrs,
};

enum class Resampling : uint8_t { kBilinear, kNearest };

struct WarpSettings {
    encode::Codec losslessCodec = encode::Codec::kLzw;
    int level = 0;
    int jpegQuality = 75;
    bool noJpeg = false;
    bool evenTileOrigin = false;
    Target target = Target::kWebMercator;
    Resampling resampling = Resampling::kBilinear;
    /// gdalwarp -dstalpha, as libddb warps a photo: every output has an alpha band (a mask for
    /// JPEG) for the pixels outside the source, its alpha, mask or nodata; a source nodata is not
    /// the output's and applies to each band as well as to the pixel (no UNIFIED_SRC_NODATA, no
    /// -dstnodata); JPEG for 8-bit RGB sources, whatever their transparency. Otherwise DroneDB's
    /// COG rules (cog_builder.h).
    bool dstAlpha = false;
    /// gdalwarp -wm (bytes): the warp memory that splits the chunks of a source georeferenced by
    /// GCPs (512 MB: libddb's COG builds; gdalwarp's own default is 64 MB).
    double warpMemory = 512.0 * 1024 * 1024;
};

/// The plan of the warped COG of `source` (everything but the GDAL metadata).
Expected<build::Plan> WarpPlan(const build::RasterSource& source, const WarpSettings& settings);

/// The producer of the plan's full-resolution strips.
Expected<std::unique_ptr<build::TileProducer>> MakeWarpProducer(const build::RasterSource& source, const build::Plan& plan,
                                                                const WarpSettings& settings, uint64_t memoryBudget);

/// The chunks of GDAL's warper the producer follows for a source georeferenced by GCPs (none for a
/// source with a geotransform): for tests.
Expected<std::vector<WarpChunk>> GdalWarpChunks(const build::RasterSource& source, const build::Plan& plan,
                                                const WarpSettings& settings = {});

}  // namespace ddbt::cog::warp
