// SPDX-License-Identifier: MPL-2.0
#pragma once

// Closed-form EPSG:4326 <-> EPSG:3857 (spherical "pseudo" Mercator, R = 6378137 m), the same
// operations as the code the baseline runs, so the results are the same bits:
//   - LonLatToWebMercator: PROJ's pipeline for EPSG:4326 -> EPSG:3857 (unitconvert deg -> rad,
//     webmerc: x = a * lam, y = a * asinh(tan(phi))), including its checks;
//   - WebMercatorToLonLatProj: the inverse of that operation (PJ_INV of the same pipeline:
//     phi = atan(sinh(y / a)), lam = x / a);
//   - WebMercatorToLonLatGdal: GDAL's own shortcut for a EPSG:3857 -> EPSG:4326 transformation
//     (OGRProjCT, bWebMercatorToWGS84LongLat): lat = pi/2 - 2 atan(exp(-y / R)), longitudes
//     wrapped into [-180, 180].
// GDAL uses PROJ for 4326 -> 3857 and for the inverse of that, and its shortcut only for a
// transformation created as 3857 -> 4326 (and PROJ for its inverse): Transform picks the same.
//
// Arrays: structure of arrays, in place allowed (out may alias in), one tight loop per call with
// no allocation and no PROJ dispatch. The transcendental functions are libm's, as in PROJ and
// GDAL: a vector math library would be faster but change the last bits (Highway's contrib/math
// is not in the vcpkg build). Lon/lat in degrees, x/y in metres. Returns the number of failed
// points, which are set to HUGE_VAL (x and y).

#include <cstddef>

namespace ddbt::crs::analytic {

inline constexpr double kWebMercatorRadius = 6378137.0;

size_t LonLatToWebMercator(const double* lon, const double* lat, double* x, double* y, size_t n) noexcept;
size_t WebMercatorToLonLatProj(const double* x, const double* y, double* lon, double* lat, size_t n) noexcept;
size_t WebMercatorToLonLatGdal(const double* x, const double* y, double* lon, double* lat, size_t n) noexcept;

}  // namespace ddbt::crs::analytic
