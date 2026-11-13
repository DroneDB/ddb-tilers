// SPDX-License-Identifier: MPL-2.0
//
// WarpModel: how the tiler serves a GeoTIFF that is not in EPSG:3857.
//
// DroneDB 2.4.1 never tiles such a file directly: it builds an EPSG:3857 COG first (gdalwarp
// -of COG -t_srs EPSG:3857 -r bilinear, TILING_SCHEME=GoogleMapsCompatible,
// ZOOM_LEVEL_STRATEGY=UPPER) and tiles that. The tile set, the bounds checks ("Out of bounds"),
// the zoom range and every geoQuery window are those of that COG's grid. This model computes the
// same grid without building it (the "virtual grid"): GDALSuggestedWarpOutput2's resolution and
// extent (edge sampling of the source, corner diagonal), the COG driver's zoom level for it
// (first level at least as fine, 1e-8 relative tolerance; UPPER) and its extent snapped to the
// 256 px tiles of that zoom (0.499 px tolerance), plus the COG's overview pyramid (sizes halved,
// rounded up, until a level fits one 256 px tile).
//
// A pixel of the virtual grid (or of one of its overviews) is read as the nearest source pixel
// to its centre: the centre is moved to the source CRS through a GridApprox of the inverse
// transform (0.001 source pixel bound, as GDAL's warped VRT error threshold), then to a source
// pixel with the inverse geotransform, at the source level GDAL's overview rule picks for the
// level's resolution. The baseline's COG is bilinear at full resolution and its overviews are
// averaged, so values differ from it by the resampling only; the geometry is the same.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <utility>
#include <vector>

#include "core/crs/transform.h"

namespace ddbt::cog::tiler {

class CogSource;

class WarpModel {
public:
    /// Throws TileError when the source cannot be transformed to EPSG:3857.
    static std::unique_ptr<WarpModel> Create(const CogSource& source);

    /// The virtual grid in EPSG:3857 (north up, square pixels).
    const std::array<double, 6>& GeoTransform() const noexcept { return gt_; }
    int Width() const noexcept { return levels_[0].first; }
    int Height() const noexcept { return levels_[0].second; }
    /// Virtual pyramid: level 0 = the grid, then the COG overviews.
    const std::vector<std::pair<int, int>>& Levels() const noexcept { return levels_; }
    /// The zoom level of the grid (GoogleMapsCompatible, 256 px tiles).
    int Zoom() const noexcept { return zoom_; }
    /// GDALSuggestedWarpOutput2's pixel size (before the COG snapping).
    double SuggestedResolution() const noexcept { return suggestedRes_; }

    /// Source samples of the view pixels cols x rows of virtual level `level` (a -1 entry is a
    /// pixel outside the window: it maps to -1). srcLevel: the index of CogSource::Levels() the
    /// samples are in; sx / sy (cols.size() * rows.size(), row-major): pixel indices in that
    /// level, -1 when the centre falls outside the source raster or does not transform.
    void Map(const CogSource& source, int level, std::span<const int32_t> cols, std::span<const int32_t> rows,
             int& srcLevel, std::vector<int32_t>& sx, std::vector<int32_t>& sy) const;

    /// Source pixels (full resolution, fractional) of points of the grid's CRS (EPSG:3857), in
    /// place; failures give NaN. For tests and diagnostics.
    void ToSourcePixels(double* xs, double* ys, size_t n) const;

private:
    WarpModel() = default;

    std::shared_ptr<const crs::Transform> transform_;  // source CRS -> EPSG:3857
    std::array<double, 6> srcGt_{};
    std::array<double, 6> srcInvGt_{};
    double srcPixel_ = 1.0;  // source pixel size (source units)
    std::array<double, 6> gt_{};
    std::vector<std::pair<int, int>> levels_;
    int zoom_ = 0;
    double suggestedRes_ = 0;
};

}  // namespace ddbt::cog::tiler
