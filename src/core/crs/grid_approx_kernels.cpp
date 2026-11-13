// SPDX-License-Identifier: MPL-2.0
//
// Highway SIMD kernel of GridApprox::TransformPoints. Compiled once per enabled target
// (foreach_target.h re-includes this file); HWY_DYNAMIC_DISPATCH picks the best target at runtime.
//
// Bit-identical to GridApprox's scalar code: the cell's corner differences, the subtraction and
// division of the cell coordinates, and the interpolations as fused multiply-adds (std::fma there,
// MulAdd here: one rounding). Targets without a native FMA would emulate MulAdd with two
// roundings, so they leave every point to the scalar code. The file is compiled with
// -ffp-contract=off: no other multiply and add may be fused.

#include "core/crs/grid_approx_kernels.h"

#include <cmath>

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "core/crs/grid_approx_kernels.cpp"
#include "hwy/foreach_target.h"  // IWYU pragma: keep, must come before highway.h
#include "hwy/highway.h"

HWY_BEFORE_NAMESPACE();
namespace ddbt::crs::kernels {
namespace HWY_NAMESPACE {

namespace hn = hwy::HWY_NAMESPACE;

void InterpolateSharedCellsImpl(const CellGrid& g, double* HWY_RESTRICT xs, double* HWY_RESTRICT ys, size_t n,
                                uint32_t* HWY_RESTRICT rest, size_t* restCount) {
    size_t k = 0;
    size_t i = 0;
#if HWY_NATIVE_FMA
    const hn::ScalableTag<double> d;
    const size_t N = hn::Lanes(d);
    const auto minX = hn::Set(d, g.minX), maxX = hn::Set(d, g.maxX);
    const auto minY = hn::Set(d, g.minY), maxY = hn::Set(d, g.maxY);
    const auto invW = hn::Set(d, g.invW), invH = hn::Set(d, g.invH);
    const auto lastCell = hn::Set(d, g.cells - 1);
    for (; i + N <= n; i += N) {
        const auto px = hn::LoadU(d, xs + i), py = hn::LoadU(d, ys + i);
        // Outside (or NaN): the exact transform.
        const auto inRect = hn::And(hn::And(hn::Ge(px, minX), hn::Le(px, maxX)), hn::And(hn::Ge(py, minY), hn::Le(py, maxY)));
        bool shared = hn::AllTrue(d, inRect);
        int ci = 0, cj = 0;
        if (shared) {
            const auto cx = hn::Min(lastCell, hn::Trunc(hn::Mul(hn::Sub(px, minX), invW)));
            const auto cy = hn::Min(lastCell, hn::Trunc(hn::Mul(hn::Sub(py, minY), invH)));
            const double cx0 = hn::GetLane(cx), cy0 = hn::GetLane(cy);
            shared = hn::AllTrue(d, hn::And(hn::Eq(cx, hn::Set(d, cx0)), hn::Eq(cy, hn::Set(d, cy0))));
            ci = static_cast<int>(cx0);
            cj = static_cast<int>(cy0);
        }
        const GridApprox::Node* nd = shared ? &g.nodes[static_cast<size_t>(cj) * static_cast<size_t>(g.cells) +
                                                       static_cast<size_t>(ci)]
                                            : nullptr;
        if (nd == nullptr || nd->child >= 0 || nd->exact) {
            for (size_t l = 0; l < N; ++l) rest[k++] = static_cast<uint32_t>(i + l);
            continue;
        }
        const double x0 = std::fma(static_cast<double>(ci), g.cellW, g.minX);
        const double y0 = std::fma(static_cast<double>(cj), g.cellH, g.minY);
        const double x1 = ci + 1 == g.cells ? g.maxX : x0 + g.cellW;
        const double y1 = cj + 1 == g.cells ? g.maxY : y0 + g.cellH;
        const auto s = hn::Div(hn::Sub(px, hn::Set(d, x0)), hn::Set(d, x1 - x0));
        const auto t = hn::Div(hn::Sub(py, hn::Set(d, y0)), hn::Set(d, y1 - y0));
        const auto bx = hn::MulAdd(s, hn::Set(d, nd->cx[1] - nd->cx[0]), hn::Set(d, nd->cx[0]));
        const auto tx = hn::MulAdd(s, hn::Set(d, nd->cx[3] - nd->cx[2]), hn::Set(d, nd->cx[2]));
        const auto by = hn::MulAdd(s, hn::Set(d, nd->cy[1] - nd->cy[0]), hn::Set(d, nd->cy[0]));
        const auto ty = hn::MulAdd(s, hn::Set(d, nd->cy[3] - nd->cy[2]), hn::Set(d, nd->cy[2]));
        hn::StoreU(hn::MulAdd(t, hn::Sub(tx, bx), bx), d, xs + i);
        hn::StoreU(hn::MulAdd(t, hn::Sub(ty, by), by), d, ys + i);
    }
#endif
    for (; i < n; ++i) rest[k++] = static_cast<uint32_t>(i);
    *restCount = k;
}

}  // namespace HWY_NAMESPACE
}  // namespace ddbt::crs::kernels
HWY_AFTER_NAMESPACE();

#if HWY_ONCE
namespace ddbt::crs::kernels {

HWY_EXPORT(InterpolateSharedCellsImpl);

size_t InterpolateSharedCells(const CellGrid& g, double* xs, double* ys, size_t n, uint32_t* rest) {
    size_t count = 0;
    HWY_DYNAMIC_DISPATCH(InterpolateSharedCellsImpl)(g, xs, ys, n, rest, &count);
    return count;
}

}  // namespace ddbt::crs::kernels
#endif  // HWY_ONCE
