// SPDX-License-Identifier: MPL-2.0
//
// The transform of the points of one input of a COPC build to the build's target CRS, on the
// interleaved X / Y / Z doubles the readers give, before they are quantised.
//
// The transform is prepared once (crs::TransformCache: GDAL's choice of operation, with the
// area of interest of the input's header bounds). It is 3-D (Z transformed with X and Y, by
// crs::Transform::TransformPoints: proj_trans_generic, each thread on its own copy of the
// operation) when the source is geocentric, either CRS has ellipsoidal heights (3-D geographic
// or projected) or the heights change vertical CRS. Otherwise the heights are kept (a compound
// source to a target of the same vertical CRS or of none; two vertical CRSs are the same when
// PROJ finds them equivalent, so an "unknown" vertical CRS copied from the source into the
// target keeps its heights) and X / Y transform between the horizontal parts, for an input
// whose bounds are known through a grid approximation over those bounds (crs::GridApprox:
// bilinear between exact nodes, refined until it is within `maxError` of PROJ): PROJ's
// transverse Mercator costs about 0.5 us a point, 4 s more on 160 million points and 30
// threads; the grid a few nanoseconds.
#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "core/crs/crs.h"
#include "core/crs/grid_approx.h"
#include "core/crs/transform.h"
#include "core/util/status.h"

namespace ddbt::copc::build {

class PointTransform {
public:
    /// The transform from `source` to `target`. `bounds` (min x, min y, max x, max y in the
    /// source CRS): the area the points cover, when known (points outside go through PROJ).
    /// `maxError`: the largest error of the grid approximation, in target units. InvalidArgument
    /// when PROJ has no operation between the CRSs.
    static Expected<std::shared_ptr<const PointTransform>> Create(const crs::Crs& source, const crs::Crs& target,
                                                                  const std::optional<std::array<double, 4>>& bounds,
                                                                  double maxError);

    /// Transforms `n` points (xyz[3 i .. 3 i + 2]) into `out` (3 n doubles; it may be xyz).
    /// Thread-safe. InvalidArgument when a point does not transform (its coordinates in the message).
    Status Apply(const double* xyz, size_t n, double* out) const;

    /// The bounds of the points of the box [min, max] (source CRS) once transformed: the points of
    /// its surface, `segments` + 1 a side on each face, through Apply (the extremes of a
    /// continuous one-to-one transform lie on the surface of the box; between two samples a side
    /// bends by less than the margin a caller adds). nullopt when a sample does not transform.
    std::optional<std::pair<std::array<double, 3>, std::array<double, 3>>> BoxBounds(const std::array<double, 3>& min,
                                                                                      const std::array<double, 3>& max,
                                                                                      int segments = 32) const;

    bool ThreeD() const noexcept { return threeD_; }
    /// The 2-D transform runs through a grid approximation.
    bool Approximated() const noexcept { return grid_.has_value(); }

    /// How a point is transformed, from the cheapest: not at all (the target is the source CRS),
    /// through the grid approximation, through PROJ (3-D, or 2-D without bounds to grid over).
    enum class Path { Identity, Grid, Exact };
    Path GetPath() const noexcept;

private:
    PointTransform(std::shared_ptr<const crs::Transform> t, bool threeD, std::optional<crs::GridApprox> grid)
        : t_(std::move(t)), threeD_(threeD), grid_(std::move(grid)) {}
    std::shared_ptr<const crs::Transform> t_;
    bool threeD_;
    std::optional<crs::GridApprox> grid_;
};

/// The WKT a LAS WKT VLR holds for `crs`: WKT1 GDAL when it expresses the CRS (read back as an
/// equivalent CRS), else WKT2 2019 (a 3-D projected CRS, a geoid model WKT1 has no place for).
/// Throws crs::CrsError when PROJ cannot write either.
std::string LasWkt(const crs::Crs& crs);

}  // namespace ddbt::copc::build
