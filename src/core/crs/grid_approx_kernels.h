// SPDX-License-Identifier: MPL-2.0
//
// The interpolation of GridApprox::TransformPoints, compiled per CPU target (runtime dispatch, see
// grid_approx_kernels.cpp). Internal API: grid_approx.cpp and the unit tests use it.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "core/crs/grid_approx.h"

namespace ddbt::crs::kernels {

/// The initial cells of a GridApprox.
struct CellGrid {
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    double cellW = 0, cellH = 0, invW = 0, invH = 0;
    int cells = 0;
    const GridApprox::Node* nodes = nullptr;  ///< the first cells * cells are the initial cells, row-major
};

/// One point by GridApprox's operations; false when it needs the exact transform (outside the
/// rectangle, NaN, or in an exact cell). The multiply-adds are fused (std::fma), whatever the
/// compiler's contraction: the SIMD path of InterpolatePoints reproduces these bits, and they are
/// the ones GCC's default contraction gave on x86-64-v3.
inline bool InterpolatePoint(const CellGrid& g, double& x, double& y) noexcept {
    const double px = x, py = y;
    if (!(px >= g.minX && px <= g.maxX && py >= g.minY && py <= g.maxY)) return false;  // outside or NaN
    const int ci = std::min(g.cells - 1, static_cast<int>((px - g.minX) * g.invW));
    const int cj = std::min(g.cells - 1, static_cast<int>((py - g.minY) * g.invH));
    double x0 = std::fma(static_cast<double>(ci), g.cellW, g.minX), y0 = std::fma(static_cast<double>(cj), g.cellH, g.minY);
    double x1 = ci + 1 == g.cells ? g.maxX : x0 + g.cellW;
    double y1 = cj + 1 == g.cells ? g.maxY : y0 + g.cellH;
    const GridApprox::Node* nd = &g.nodes[static_cast<size_t>(cj) * static_cast<size_t>(g.cells) + static_cast<size_t>(ci)];
    while (nd->child >= 0) {
        const double xm = 0.5 * (x0 + x1), ym = 0.5 * (y0 + y1);
        int q = 0;
        if (px >= xm) {
            q |= 1;
            x0 = xm;
        } else {
            x1 = xm;
        }
        if (py >= ym) {
            q |= 2;
            y0 = ym;
        } else {
            y1 = ym;
        }
        nd = &g.nodes[static_cast<size_t>(nd->child + q)];
    }
    if (nd->exact) return false;
    const double s = (px - x0) / (x1 - x0), t = (py - y0) / (y1 - y0);
    const double bx = std::fma(s, nd->cx[1] - nd->cx[0], nd->cx[0]);
    const double tx = std::fma(s, nd->cx[3] - nd->cx[2], nd->cx[2]);
    const double by = std::fma(s, nd->cy[1] - nd->cy[0], nd->cy[0]);
    const double ty = std::fma(s, nd->cy[3] - nd->cy[2], nd->cy[2]);
    x = std::fma(t, tx - bx, bx);
    y = std::fma(t, ty - by, by);
    return true;
}

/// Interpolates every point it can, compiled for the running CPU: each group of SIMD-width
/// consecutive points in one initial cell that is an accepted leaf (no refinement, not exact) with
/// vector operations (points are dense along rows, so most groups are), the others one by one
/// with InterpolatePoint; the same bits either way. Writes the indices of the points that need
/// the exact transform to `exact` (n entries at most) and returns their number.
size_t InterpolatePoints(const CellGrid& g, double* xs, double* ys, size_t n, uint32_t* exact);

}  // namespace ddbt::crs::kernels
