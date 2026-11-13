// SPDX-License-Identifier: MPL-2.0
//
// Highway SIMD kernel of GridApprox::TransformPoints (runtime dispatch, see
// grid_approx_kernels.cpp). Internal API: grid_approx.cpp and the unit tests use it.
#pragma once

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

/// Interpolates, with GridApprox's operations (two FMAs per coordinate, as the scalar code
/// writes them), every group of SIMD-width consecutive points that lies in one initial cell that
/// is an accepted leaf (no refinement, not exact): points are dense along rows, so most groups
/// do. Writes the indices of the other points to `rest` (n entries at most) and returns their
/// number; the caller transforms those point by point. On a target without FMA, every point.
size_t InterpolateSharedCells(const CellGrid& g, double* xs, double* ys, size_t n, uint32_t* rest);

/// Whether the target InterpolateSharedCells dispatches to has a native FMA, i.e. interpolates
/// at all (SSE2 to SSE4 do not: their points all go to the scalar code).
bool SharedCellKernelInterpolates();

}  // namespace ddbt::crs::kernels
