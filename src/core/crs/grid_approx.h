// SPDX-License-Identifier: MPL-2.0
#pragma once

// GridApprox: a transform over a rectangle of the source CRS, approximated by bilinear
// interpolation between exact PROJ results at the nodes of a grid, refined where needed.
//
//  - PROJ is evaluated exactly on an N x N grid of nodes (default 9 x 9, i.e. 8 x 8 cells);
//  - each cell is checked against PROJ at its centre and the middles of its four edges (the
//    points where bilinear interpolation of a smooth mapping is worst); a cell whose error
//    there exceeds maxError (target units) is split in four (quadtree), reusing those five
//    exact points as the new nodes, down to maxDepth (and within a cell budget);
//  - a cell that still fails the bound at maxDepth or past the budget, or with a node or check
//    point PROJ could not transform, is marked exact: its points go through PROJ (batched);
//  - points outside the rectangle go through PROJ too.
// The bound is guaranteed at the check points; between them it holds for transforms that are
// smooth at the cell scale (every projection away from its singularities). The unit tests
// measure it on random points. For pixels: maxError = tolerance_px * target pixel size (GDAL's
// warper uses 0.125 px by default).
//
// Built once per (transform, rectangle), then immutable and thread-safe (the exact fallback uses
// the Transform's per-thread PROJ objects).

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "core/crs/transform.h"

namespace ddbt::crs {

struct GridApproxOptions {
    int gridSize = 9;       ///< nodes per side of the initial grid (>= 2)
    double maxError = 0.0;  ///< target units; 0: exact everywhere (no approximation)
    int maxDepth = 8;       ///< quadtree levels below the initial cells
    /// Cell budget: once the quadtree holds this many cells, cells that still fail the bound
    /// become exact instead of splitting (bounds memory and build time when maxError is too
    /// tight for the area: 65536 cells are ~5 MB).
    size_t maxCells = 65536;
    Direction direction = Direction::Forward;
};

struct GridApproxStats {
    size_t initialCells = 0;
    size_t leafCells = 0;
    size_t exactCells = 0;     ///< leaves whose points fall back to PROJ
    size_t projEvaluations = 0;  ///< points PROJ transformed while building
    double maxCheckError = 0;    ///< max error measured at the check points of accepted cells
    int maxDepthReached = 0;
};

class GridApprox {
public:
    /// Builds the approximation of `transform` over [minX, maxX] x [minY, maxY] (source units, in
    /// the given direction's source). Throws std::invalid_argument for an empty rectangle or
    /// gridSize < 2.
    static GridApprox Build(std::shared_ptr<const Transform> transform, double minX, double minY, double maxX,
                            double maxY, const GridApproxOptions& options);

    /// Transforms n points in place (as Transform::TransformPoints, without z). Returns the
    /// number of failed points (set to HUGE_VAL). Groups of consecutive points in one cell are
    /// interpolated with SIMD (grid_approx_kernels.h), with the bits of the scalar code.
    size_t TransformPoints(double* xs, double* ys, size_t n) const;

    /// TransformPoints point by point, without SIMD: the reference of the tests.
    size_t TransformPointsScalar(double* xs, double* ys, size_t n) const;

    const GridApproxStats& Stats() const noexcept { return stats_; }
    const GridApproxOptions& Options() const noexcept { return options_; }

    struct Node {
        // Corner values (target x, y) at (x0,y0) (x1,y0) (x0,y1) (x1,y1).
        double cx[4], cy[4];
        int32_t child = -1;  ///< index of the first of 4 children (order: SW, SE, NW, NE), -1: leaf
        uint8_t exact = 0;   ///< leaf whose points go through PROJ
    };

private:
    GridApprox() = default;

    // Interpolates one point; false when it needs the exact transform (outside the rectangle,
    // NaN, or in an exact cell).
    bool Interpolate(double& x, double& y) const;
    // The exact transform of the points at `idx`; returns the number that failed.
    size_t TransformExactly(double* xs, double* ys, const std::vector<size_t>& idx) const;

    std::shared_ptr<const Transform> transform_;
    GridApproxOptions options_;
    GridApproxStats stats_;
    double minX_ = 0, minY_ = 0, maxX_ = 0, maxY_ = 0;
    double cellW_ = 0, cellH_ = 0;  // initial cell size
    double invW_ = 0, invH_ = 0;    // 1 / cell size
    int cells_ = 0;                 // initial cells per side
    std::vector<Node> nodes_;       // first cells_ * cells_ are the initial cells (row-major from minY)
};

}  // namespace ddbt::crs
