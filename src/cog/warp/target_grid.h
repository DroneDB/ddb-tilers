// SPDX-License-Identifier: MPL-2.0
//
// The EPSG:3857 grid of the COG gdalwarp -of COG -t_srs EPSG:3857 writes with
// TILING_SCHEME=GoogleMapsCompatible and ZOOM_LEVEL_STRATEGY=UPPER (DroneDB's cogWarpArgs):
//
//   - GDALSuggestedWarpOutput2 (alg/gdaltransformer.cpp): the source edges sampled with
//     nSteps = clamp(round(min(W, H) / 50), 20, 100) points each, transformed; the pixel size is
//     the diagonal of the transformed corners (0, 0) and (W, H) over the source's pixel diagonal;
//     the extent is the bounding box of the successful points, rounded to whole pixels.
//   - COGGetWarpingCharacteristics (frmts/gtiff/cogdriver.cpp): the first zoom level whose
//     resolution is at least as fine as that pixel size (1e-8 relative tolerance), the extent
//     snapped outwards to that level's 256 px tiles with a 0.499 px tolerance, clamped to the
//     tile matrix; size = extent / resolution.
//
// Same arithmetic as the tiler's WarpModel (tested equal to libddb's COGs). The option
// evenTileOrigin moves the origin to an even tile column / row (a 512 px retina tile of the
// next zoom is then one block): not GDAL's grid, off by default.
#pragma once

#include <array>
#include <memory>

#include "core/crs/crs.h"
#include "core/crs/transform.h"
#include "core/util/status.h"

namespace ddbt::cog::warp {

struct TargetGrid {
    std::array<double, 6> geoTransform{};
    uint32_t width = 0, height = 0;
    int zoom = 0;
    double suggestedResolution = 0;  // GDALSuggestedWarpOutput2's pixel size
    std::shared_ptr<const crs::Transform> transform;  // source CRS -> EPSG:3857
};

/// srcGt: the source geotransform; W x H its size.
Expected<TargetGrid> ComputeTargetGrid(const crs::Crs& srcCrs, const std::array<double, 6>& srcGt, uint32_t W,
                                       uint32_t H, bool evenTileOrigin);

}  // namespace ddbt::cog::warp
