// SPDX-License-Identifier: MPL-2.0
//
// The source side of the warp: source pixel / line <-> source CRS, the first step of GDAL's
// general image transformer (GDALGenImgProjTransform) from source pixels and its last step back
// to them. A raster is georeferenced either by a geotransform (affine) or, without one, by ground
// control points (GDALCreateGenImgProjTransformer2's GCP part with its default options: a
// homography for 4 or 5 GCPs, polynomials otherwise, crs::GcpTransform). The target grid and the
// warp producer go through this class and never through a geotransform directly, so both kinds
// are warped the same way; for a geotransform the arithmetic is GDAL's (x = gt[0] + p * gt[1] +
// l * gt[2], and the inverse geotransform likewise), the same operations in the same order.
//
// Immutable: one SourceGeoreference can be shared between threads.
#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <span>

#include "cog/tiff/geotiff.h"
#include "core/crs/crs.h"
#include "core/crs/gcp_transform.h"
#include "core/util/status.h"

namespace ddbt::cog::warp {

class SourceGeoreference {
public:
    /// A geotransform; fails when it cannot be inverted (GDAL: "Cannot invert geotransform").
    static Expected<SourceGeoreference> FromGeoTransform(const std::array<double, 6>& gt);
    /// GCPs in gcpCrs (which only decides the antimeridian unwrap); fails where GDAL cannot build
    /// the transform (too few GCPs, degenerate GCPs: GcpTransform::Create).
    static Expected<SourceGeoreference> FromGcps(std::span<const crs::Gcp> gcps, const crs::Crs& gcpCrs);
    /// What gdalwarp uses for a GeoTIFF: its geotransform, else its GCPs (in the CRS of its
    /// GeoKeys). Fails when it has neither.
    static Expected<SourceGeoreference> Of(const tiff::Georeferencing& georef);

    /// The geotransform, when the raster has one (nullopt for GCPs).
    const std::optional<std::array<double, 6>>& GeoTransform() const noexcept { return gt_; }

    /// Pixel / line -> source CRS, n points in place. A point whose x or y is HUGE_VAL (an earlier
    /// step failed it) or that the GCP transform cannot map fails and comes out as HUGE_VAL.
    /// Returns the number of points that failed.
    size_t PixelToCrs(double* xs, double* ys, size_t n) const noexcept;
    /// Source CRS -> pixel / line, likewise.
    size_t CrsToPixel(double* xs, double* ys, size_t n) const noexcept;
    bool PixelToCrs(double& x, double& y) const noexcept { return PixelToCrs(&x, &y, 1) == 0; }
    bool CrsToPixel(double& x, double& y) const noexcept { return CrsToPixel(&x, &y, 1) == 0; }

    /// The affine part of CrsToPixel for vectorised callers: the inverse geotransform, x' = a[0] +
    /// x * a[1] + y * a[2], y' = a[3] + x * a[4] + y * a[5]; the identity for GCPs (their callers
    /// compute source pixels themselves).
    const std::array<double, 6>& PixelAffine() const noexcept { return affine_; }

private:
    SourceGeoreference() = default;

    std::optional<std::array<double, 6>> gt_;
    std::optional<crs::GcpTransform> gcp_;
    std::array<double, 6> affine_{0, 1, 0, 0, 0, 1};  // inverse geotransform, or identity
};

/// The inverse of a geotransform by its determinant: false when it is not invertible. It is not
/// GDALInvGeoTransform to the last bit (GDAL takes 1 / gt[1] and 1 / gt[5] for a north-up
/// geotransform), and the warp of a raster with a geotransform keeps it as it is.
bool InvertGeoTransform(const std::array<double, 6>& gt, std::array<double, 6>& inv);

/// GDALInvGeoTransform, to the last bit (all zeros when it is not invertible).
std::array<double, 6> GdalInvGeoTransform(const std::array<double, 6>& gt);

}  // namespace ddbt::cog::warp
