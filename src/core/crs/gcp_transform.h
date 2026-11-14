// SPDX-License-Identifier: MPL-2.0
#pragma once

// The transform between the pixel / line coordinates of a raster georeferenced only by ground
// control points (GCPs, no geotransform) and the coordinates of the GCPs' CRS: the one gdalwarp
// uses for such a raster, GDALCreateGenImgProjTransformer2 with its default options (no METHOD,
// GCPS_OK=YES, no REFINE_TOLERANCE, GCP_ANTIMERIDIAN_UNWRAP=AUTO) and MAX_GCP_ORDER (gdalwarp
// -order) as an option:
//   - 4 or 5 GCPs: a homography fitted by least squares (GDALCreateHomographyTransformerFromGCPs);
//   - otherwise: polynomials fitted by least squares (GDALCreateGCPTransformer), of the requested
//     order, or, with order 0, of order 2 from 6 GCPs on and of order 1 below; the inverse
//     (georeferenced -> pixel / line) is a second polynomial fitted separately, not the inverse of
//     the first, so a round trip is exact only where the GCPs fit exactly; order 1 with 2 GCPs
//     takes them as opposite corners of a rectangle (a third GCP is made at a third corner);
//   - GCPs in a geographic CRS in degrees whose longitudes are all within [-180, -170] or
//     [170, 180] are first unwrapped: a negative longitude gains 360 (so the georeferenced side,
//     both ways, is then in [170, 190] for those rasters).
// The arithmetic is GDAL 3.13.3's, in the same order (see gcp_transform.cpp), so the results are
// GDAL's to the last bit for the polynomials and with GDAL's built-in linear solver (the one a
// GDAL built without Armadillo uses) for the homography.
//
// A GcpTransform is immutable: it can be shared between threads.

#include <array>
#include <cstddef>
#include <span>
#include <vector>

#include "core/crs/crs.h"
#include "core/crs/transform.h"
#include "core/util/status.h"

namespace ddbt::crs {

/// A ground control point (GDAL_GCP): the pixel / line position on the raster (0, 0 = top left
/// corner of the top left pixel) and the georeferenced position in the GCPs' CRS (x = easting or
/// longitude, y = northing or latitude, traditional GIS order). z is kept for completeness: the
/// transforms do not use it (as GDAL's do not).
struct Gcp {
    double pixel = 0, line = 0;
    double x = 0, y = 0, z = 0;
};

enum class GcpMethod {
    /// gdalwarp's default (no METHOD): homography for 4 or 5 GCPs, polynomial otherwise.
    Auto,
    /// METHOD=GCP_HOMOGRAPHY: homography whatever the number of GCPs (at least 4).
    Homography,
    /// METHOD=GCP_POLYNOMIAL: polynomial whatever the number of GCPs.
    Polynomial,
};

struct GcpTransformOptions {
    GcpMethod method = GcpMethod::Auto;
    /// MAX_GCP_ORDER (gdalwarp -order): 0 chooses (2 from 6 GCPs on, else 1), 1 to 3 asks that
    /// order; the homography of GcpMethod::Auto with 4 or 5 GCPs ignores it, as GDAL does. A
    /// negative order (GDAL's thin plate spline) is not supported, except with
    /// GcpMethod::Polynomial, where GDAL takes it as 0.
    int maxOrder = 0;
};

class GcpTransform {
public:
    enum class Kind { Homography, Polynomial };

    /// Fits the transform to the GCPs. gcpCrs (may be empty) only decides the longitude unwrap.
    /// Fails, with GDAL's reason, where GDAL fails: too few GCPs for the order, GCPs on a line or
    /// otherwise degenerate (singular system), a homography that does not map the GCPs' pixel
    /// extent to a convex quadrilateral; and, where GDAL would build a transform that gives
    /// NaN for every point (a homography whose coefficients are not finite), fails as well.
    static Expected<GcpTransform> Create(std::span<const Gcp> gcps, const Crs& gcpCrs,
                                         const GcpTransformOptions& options = {});

    Kind GetKind() const noexcept { return kind_; }
    /// Order of the polynomials (1 to 3), 0 for a homography.
    int Order() const noexcept { return order_; }
    /// The GCPs' longitudes were unwrapped across the antimeridian.
    bool LongitudesUnwrapped() const noexcept { return unwrapped_; }

    /// Transforms n points in place: Direction::Forward from pixel / line to the GCPs' CRS,
    /// Direction::Inverse back (GDALGCPTransform / GDALHomographyTransform with bDstToSrc).
    /// success (may be null) receives 1 or 0 per point. A point fails when x or y is HUGE_VAL
    /// (a point an earlier step failed), or, for a homography, when it maps to infinity; its x
    /// and y are then set to HUGE_VAL. Returns the number of points that failed.
    size_t TransformPoints(double* xs, double* ys, size_t n, Direction dir = Direction::Forward,
                           int* success = nullptr) const noexcept;
    /// One point; false if it failed.
    bool TransformPoint(double& x, double& y, Direction dir = Direction::Forward) const noexcept {
        return TransformPoints(&x, &y, 1, dir) == 0;
    }

    /// The homography, pixel / line -> georeferenced (GDAL's nine coefficients: x = (h0 + h1 p +
    /// h2 l) / (h6 + h7 p + h8 l), y = (h3 + h4 p + h5 l) / (h6 + h7 p + h8 l)), and its inverse.
    /// Zero for a polynomial transform.
    const std::array<double, 9>& Homography() const noexcept { return forward_; }
    const std::array<double, 9>& InverseHomography() const noexcept { return inverse_; }

private:
    GcpTransform() = default;

    void PolynomialPoints(double* xs, double* ys, size_t n, Direction dir, int* success, size_t& failed) const noexcept;
    void HomographyPoints(double* xs, double* ys, size_t n, Direction dir, int* success, size_t& failed) const noexcept;

    Kind kind_ = Kind::Polynomial;
    int order_ = 0;
    bool unwrapped_ = false;
    // Polynomial: coefficients of x and y in terms of (pixel - pixel mean, line - line mean)
    // (toGeo) and of (x - x mean, y - y mean) (fromGeo), GDAL's term order 1, e, n, e^2, en, n^2,
    // e^3, e^2n, en^2, n^3.
    std::array<double, 10> toGeoX_{}, toGeoY_{}, fromGeoX_{}, fromGeoY_{};
    double pixelMean_ = 0, lineMean_ = 0, xMean_ = 0, yMean_ = 0;
    // Homography.
    std::array<double, 9> forward_{}, inverse_{};
};

}  // namespace ddbt::crs
