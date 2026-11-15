// SPDX-License-Identifier: MPL-2.0
#pragma once
// The bounds of a box of one CRS in another, as GDAL computes them for gdalwarp's -te_srs
// (OGRCoordinateTransformation::TransformBounds with 21 points of densification, through the
// transformation gdalwarp prepares for the box: Transform::AreaOfInterestForBox):
//   - the 4 sides of the box, each densified into densify + 1 points, transformed as one ring;
//     a geographic box whose east is under its west crosses the antimeridian;
//   - to a projected or other non-geographic CRS: the min and max of the points that transform,
//     then, for a projected CRS, the poles at its central meridian (85.0511287798066 degrees for a
//     Mercator 1SP with a central meridian of 0, else 90, less 1e-8) and the points 180 degrees from
//     the central meridian (at the latitude of origin when the meridian is not 0, and at the pole
//     when a pole is in the box), each added when it falls in the box;
//   - to a geographic CRS in degrees: a pole of it in the box gives the whole circle of longitudes
//     to that pole; else the longitudes are bounded across the antimeridian where the ring
//     crosses it (west > east).
// The axes in traditional GIS order (longitude first). nullopt when no point transforms.
#include <optional>

#include "core/crs/crs.h"

namespace ddbt::crs {

struct Bounds {
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
};

std::optional<Bounds> TransformBounds(const Crs& from, const Crs& to, const Bounds& box, int densify = 21);

}  // namespace ddbt::crs
