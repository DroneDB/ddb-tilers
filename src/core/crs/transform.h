// SPDX-License-Identifier: MPL-2.0
#pragma once

// Coordinate transformation between two CRSs, prepared as GDAL prepares the one of a raster
// warp (GDALCreateGenImgProjTransformer2 -> GDALCreateReprojectionTransformerEx -> OGRProjCT):
//   - PROJ chooses the operation(s): proj_create_crs_to_crs_from_pj(src, dst, area, options)
//     with the area of interest = the source extent in lon/lat (GDALComputeAreaOfInterest) and
//     GDAL's default options (ballpark allowed, ONLY_BEST left to PROJ / PROJ_ONLY_BEST_DEFAULT,
//     no accuracy); PROJ then picks per point among the candidate operations by area of use;
//   - axis order: traditional GIS order (proj_normalize_for_visualization: longitude first for
//     geographic CRSs, easting first), which is what GDAL's OAMS_TRADITIONAL_GIS_ORDER gives;
//   - the inverse is the same prepared operation run backwards (OGRProjCT::GetInverse clones it);
//   - source equivalent to target: identity (OGRProjCT bNoTransform);
//   - EPSG:4326 <-> EPSG:3857: closed form, the same bits as PROJ / GDAL (web_mercator_analytic.h).
// Thread safety: a Transform is immutable and can be shared; each thread transforms with its
// own clone of the prepared PJ (proj_clone, made on first use) in its own PJ_CONTEXT.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "core/crs/crs.h"

namespace ddbt::crs {

struct TransformOptions {
    /// Area of interest in degrees. Unset: none (PROJ considers every operation and picks per
    /// point). Use AreaOfInterestFor() to get GDAL's for a source extent.
    std::optional<LonLatBox> areaOfInterest;
    /// ALLOW_BALLPARK (GDAL default: allowed).
    bool allowBallpark = true;
    /// ONLY_BEST: unset leaves PROJ's default (PROJ_ONLY_BEST_DEFAULT / proj.ini).
    std::optional<bool> onlyBest;
    /// ACCURACY in metres (unset: none).
    std::optional<double> accuracy;
    /// Closed form for EPSG:4326 <-> EPSG:3857 (same results as PROJ / GDAL, faster).
    bool analyticWebMercator = true;

    /// Cache key text.
    std::string Key() const;
};

enum class Direction { Forward, Inverse };

class Transform {
public:
    enum class Kind { Identity, WebMercatorFromLonLat, WebMercatorToLonLatGdal, Proj };

    /// Prepares src -> dst. Throws CrsError when PROJ has no operation. Prefer
    /// TransformCache::Global().Get(), which shares prepared transforms.
    static std::shared_ptr<const Transform> Create(const Crs& src, const Crs& dst, const TransformOptions& options = {});

    /// GDALComputeAreaOfInterest: the four corners of a source extent, in the source's own
    /// geographic CRS (the horizontal part of a compound CRS; degrees, lon/lat), min / max of the
    /// corners that transform; west > east when the top corners cross the antimeridian.
    /// nullopt when no corner transforms or the result is out of [-180, 180] x [-90, 90].
    static std::optional<LonLatBox> AreaOfInterestFor(const Crs& src, double minX, double minY, double maxX,
                                                      double maxY);

    ~Transform();
    Transform(const Transform&) = delete;
    Transform& operator=(const Transform&) = delete;

    const Crs& Source() const noexcept { return src_; }
    const Crs& Target() const noexcept { return dst_; }
    const TransformOptions& Options() const noexcept { return options_; }
    Kind GetKind() const noexcept { return kind_; }

    /// Transforms n points in place (traditional GIS order: lon/lat in degrees, x/y in the CRS
    /// units). zs may be null (z = 0, not written). Returns the number of points that failed;
    /// their x and y are set to HUGE_VAL (as GDAL does). Batched through proj_trans_generic.
    size_t TransformPoints(double* xs, double* ys, double* zs, size_t n, Direction dir = Direction::Forward) const;
    /// One point; false if it failed.
    bool TransformPoint(double& x, double& y, double* z = nullptr, Direction dir = Direction::Forward) const {
        return TransformPoints(&x, &y, z, 1, dir) == 0;
    }

    /// The PROJ definition of the prepared operation ("" for identity, the analytic forms, or a
    /// set of candidate operations PROJ picks from per point: see LastUsedOperation()).
    std::string Definition() const;
    /// The operation PROJ used for the last point this thread transformed (PROJ string), "" if
    /// none or not PROJ.
    std::string LastUsedOperation() const;
    struct Impl;

private:
    Transform(Crs src, Crs dst, TransformOptions options);
    void* ThreadPj() const;  // PJ* of this thread

    Crs src_, dst_;
    TransformOptions options_;
    Kind kind_ = Kind::Proj;
    uint64_t id_ = 0;
    std::shared_ptr<const void> alive_;
    std::unique_ptr<Impl> impl_;
};

/// Process-wide cache of prepared transforms, keyed by (source, target, options). Concurrent
/// requests for the same key prepare it once (the others wait).
class TransformCache {
public:
    static TransformCache& Global();

    std::shared_ptr<const Transform> Get(const Crs& src, const Crs& dst, const TransformOptions& options = {});
    /// GDAL's warp preparation: options.areaOfInterest = AreaOfInterestFor(src, extent).
    std::shared_ptr<const Transform> GetForExtent(const Crs& src, const Crs& dst, double minX, double minY,
                                                  double maxX, double maxY, TransformOptions options = {});
    size_t Size() const;
    void Clear();

private:
    struct Slot;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, std::shared_ptr<Slot>> map_;
};

}  // namespace ddbt::crs
