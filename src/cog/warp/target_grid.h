// SPDX-License-Identifier: MPL-2.0
//
// The EPSG:3857 grid of the COG gdalwarp -of COG -t_srs EPSG:3857 writes with
// TILING_SCHEME=GoogleMapsCompatible and ZOOM_LEVEL_STRATEGY=UPPER (DroneDB's cogWarpArgs):
//
//   - GDALSuggestedWarpOutput2 (alg/gdaltransformer.cpp) on GDAL's general image transformer
//     (source pixel -> source CRS through the geotransform or the GCPs, SourceGeoreference, then
//     to EPSG:3857; for a geotransform with PROJ's area of interest of its extent, which GDAL does
//     not compute for GCPs): the source edges sampled with
//     nSteps = clamp(round(min(W, H) / 50), 20, 100) points each, transformed; the pixel size is
//     the diagonal of the transformed corners (0, 0) and (W, H) over the source's pixel diagonal;
//     the extent is the bounding box of the successful points, rounded to whole pixels.
//   - gdalwarp's check that a 21 x 21 grid of the extent goes to source pixels and back within a
//     target pixel, else the suggestion again with CHECK_WITH_INVERT_PROJ; a geographic source
//     with a north-up geotransform that crosses Mercator's latitude limits gets gdalwarp's
//     clamped extent instead (GDAL does that for geotransforms only).
//   - COGGetWarpingCharacteristics (frmts/gtiff/cogdriver.cpp): the first zoom level whose
//     resolution is at least as fine as that pixel size (1e-8 relative tolerance), the extent
//     snapped outwards to that level's 256 px tiles with a 0.499 px tolerance, clamped to the
//     tile matrix; size = extent / resolution.
// Same arithmetic as the tiler's WarpModel (tested equal to libddb's COGs). The option
// evenTileOrigin moves the origin to an even tile column / row (a 512 px retina tile of the
// next zoom is then one block): not GDAL's grid, off by default.
#pragma once

#include <array>
#include <memory>

#include "core/crs/crs.h"
#include "core/crs/transform.h"
#include "cog/warp/source_georef.h"
#include "core/util/status.h"

namespace ddbt::cog::warp {

struct TargetGrid {
    std::array<double, 6> geoTransform{};
    uint32_t width = 0, height = 0;
    int zoom = 0;
    double suggestedResolution = 0;  // GDALSuggestedWarpOutput2's pixel size
    std::shared_ptr<const crs::Transform> transform;  // source CRS -> EPSG:3857
};

/// srcCrs: the CRS of the source (of its GCPs for a GCP source); W x H its size.
Expected<TargetGrid> ComputeTargetGrid(const crs::Crs& srcCrs, const SourceGeoreference& georef, uint32_t W,
                                       uint32_t H, bool evenTileOrigin);

/// gdalwarp's output grid for a source warped into its own CRS (no -t_srs: the CRS of its GCPs),
/// without a COG tiling: GDALSuggestedWarpOutput2's extent and resolution, the size rounded from
/// them (gdalwarp's GDALWarpCreateOutput).
/// `transform` is the identity.
Expected<TargetGrid> ComputeSourceCrsGrid(const crs::Crs& srcCrs, const SourceGeoreference& georef, uint32_t W,
                                          uint32_t H);

/// The EPSG:3857 grid of ComputeTargetGrid for the raster gdalwarp writes of the source in its
/// own CRS (ComputeSourceCrsGrid): the grid of libddb's COG of a photo, which warps it into the
/// CRS of its GCPs first. `transform` is the source's to EPSG:3857.
Expected<TargetGrid> ComputeTargetGridThroughSourceCrs(const crs::Crs& srcCrs, const SourceGeoreference& georef,
                                                       uint32_t W, uint32_t H, bool evenTileOrigin);

/// GDALSuggestedWarpOutput2's extent (min x, min y, max x, max y) of the source in the pixels and
/// lines of an output of geotransform dstGt: gdalwarp warps a source with RPC metadata only
/// within it (AdjustOutputExtentForRPC). t: source CRS -> the output's CRS.
Expected<std::array<double, 4>> SuggestedPixelExtent(const crs::Transform& t, const SourceGeoreference& georef,
                                                     uint32_t W, uint32_t H, const std::array<double, 6>& dstGt);

}  // namespace ddbt::cog::warp
