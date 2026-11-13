// SPDX-License-Identifier: MPL-2.0
//
// GridApprox (error bound on random points, exact fallback) and the polynomial fits.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

#include "core/crs/crs.h"
#include "core/crs/grid_approx.h"
#include "core/crs/grid_approx_kernels.h"
#include "core/crs/polyfit.h"
#include "core/crs/transform.h"
#include "core/simd/simd.h"

namespace {

using ddbt::crs::Crs;
using ddbt::crs::Direction;
using ddbt::crs::FitPolynomial;
using ddbt::crs::FitPolynomialAdaptive;
using ddbt::crs::GridApprox;
using ddbt::crs::GridApproxOptions;
using ddbt::crs::Transform;
using ddbt::crs::TransformCache;
using ddbt::crs::TransformOptions;
namespace kernels = ddbt::crs::kernels;
namespace simd = ddbt::simd;

struct Area {
    const char* name;
    Crs src, dst;
    double minX, minY, maxX, maxY;
    double maxError;
    Direction dir;
    bool viaProj;  // force PROJ for 4326 <-> 3857
};

// Max distance between GridApprox and the exact transform on random points of the rectangle.
double MeasureError(const GridApprox& g, const Transform& t, const Area& a, size_t n, uint64_t seed) {
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> ux(a.minX, a.maxX), uy(a.minY, a.maxY);
    std::vector<double> x(n), y(n);
    for (size_t i = 0; i < n; ++i) {
        x[i] = ux(rng);
        y[i] = uy(rng);
    }
    std::vector<double> ex = x, ey = y;
    EXPECT_EQ(g.TransformPoints(x.data(), y.data(), n), 0u);
    EXPECT_EQ(t.TransformPoints(ex.data(), ey.data(), nullptr, n, a.dir), 0u);
    double m = 0;
    for (size_t i = 0; i < n; ++i) m = std::max(m, std::hypot(x[i] - ex[i], y[i] - ey[i]));
    return m;
}

TEST(GridApprox, ErrorBoundOnRandomPoints) {
    // A tile-sized and a raster-sized area per CRS; bounds in target units (0.125 px of the
    // target resolution: z18 is 0.597 m/px, so 0.075 m; and much tighter ones).
    const Crs wgs = Crs::FromEpsg(4326), merc = Crs::FromEpsg(3857);
    const std::vector<Area> areas = {
        {"31370->3857 20 km", Crs::FromEpsg(31370), merc, 140000, 160000, 160000, 180000, 0.075, Direction::Forward, false},
        {"31370->3857 20 km tight", Crs::FromEpsg(31370), merc, 140000, 160000, 160000, 180000, 0.001, Direction::Forward, false},
        {"3857->31370 inverse z14 tile", Crs::FromEpsg(31370), merc, 485000, 6592000, 487446, 6594446, 0.001, Direction::Inverse, false},
        {"2154->3857 50 km", Crs::FromEpsg(2154), merc, 630000, 6840000, 680000, 6890000, 0.01, Direction::Forward, false},
        {"32617->3857 100 km", Crs::FromEpsg(32617), merc, 300000, 4400000, 400000, 4500000, 0.05, Direction::Forward, false},
        {"6599 ftUS->3857", Crs::FromEpsg(6599), merc, 1500000, 300000, 1700000, 600000, 0.01, Direction::Forward, false},
        // A continent at z6 (2446 m/px at 256 px): 0.125 px = 300 m; and a z12 tile area at 1 cm.
        {"4326->3857 continent z6", wgs, merc, -10, 35, 30, 70, 300, Direction::Forward, true},
        {"4326->3857 z12 area 1 cm", wgs, merc, 4.30, 50.80, 4.40, 50.90, 0.01, Direction::Forward, true},
        {"3857->4326 (inverse) z8 tile", wgs, merc, 0, 6261721, 156543, 6418264, 1e-7, Direction::Inverse, true},
    };
    for (const auto& a : areas) {
        SCOPED_TRACE(a.name);
        TransformOptions o;
        o.analyticWebMercator = !a.viaProj;
        const auto t = TransformCache::Global().Get(a.src, a.dst, o);
        GridApproxOptions go;
        go.maxError = a.maxError;
        go.direction = a.dir;
        const GridApprox g = GridApprox::Build(t, a.minX, a.minY, a.maxX, a.maxY, go);
        const double err = MeasureError(g, *t, a, 20000, 99);
        const auto& s = g.Stats();
        std::printf("[ gridapx  ] %-32s bound %-8g measured %.3g  leaves %zu exact %zu depth %d PROJ evals %zu\n", a.name,
                    a.maxError, err, s.leafCells, s.exactCells, s.maxDepthReached, s.projEvaluations);
        EXPECT_LE(err, a.maxError);
        EXPECT_LE(s.maxCheckError, a.maxError);
    }
}

TEST(GridApprox, ExactFallbackWherePROJFails) {
    // 4326 -> 3857 through PROJ over latitudes beyond the pole: cells touching |lat| > 90 are
    // exact, their invalid points fail exactly as PROJ fails them, the others are approximated.
    TransformOptions o;
    o.analyticWebMercator = false;
    const auto t = Transform::Create(Crs::FromEpsg(4326), Crs::FromEpsg(3857), o);
    GridApproxOptions go;
    go.maxError = 0.5;
    const GridApprox g = GridApprox::Build(t, -10, 60, 10, 95, go);
    EXPECT_GT(g.Stats().exactCells, 0u);
    std::vector<double> x = {0, 5, 0, -3}, y = {70, 89.5, 92, 91};
    std::vector<double> ex = x, ey = y;
    EXPECT_EQ(g.TransformPoints(x.data(), y.data(), x.size()), 2u);
    EXPECT_EQ(t->TransformPoints(ex.data(), ey.data(), nullptr, ex.size()), 2u);
    EXPECT_NEAR(x[0], ex[0], 0.5);
    EXPECT_NEAR(y[0], ey[0], 0.5);
    EXPECT_NEAR(y[1], ey[1], 0.5);
    EXPECT_EQ(x[2], HUGE_VAL);
    EXPECT_EQ(y[3], HUGE_VAL);
    // Outside the rectangle: exact
    double ox = 50, oy = 10, px = 50, py = 10;
    EXPECT_EQ(g.TransformPoints(&ox, &oy, 1), 0u);
    ASSERT_TRUE(t->TransformPoint(px, py));
    EXPECT_EQ(ox, px);
    EXPECT_EQ(oy, py);
}

// Points as the warper and the tiler ask for them: rows of pixel centres over the rectangle
// (and a pixel beyond each edge), then random points, points outside and NaN.
void WarpLikePoints(double minX, double minY, double maxX, double maxY, int cols, int rows, uint64_t seed,
                    std::vector<double>& x, std::vector<double>& y) {
    const double dx = (maxX - minX) / (cols - 2), dy = (maxY - minY) / (rows - 2);
    for (int j = 0; j < rows; ++j)
        for (int i = 0; i < cols; ++i) {
            x.push_back(minX + (i - 0.5) * dx);
            y.push_back(maxY - (j - 0.5) * dy);
        }
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> ux(minX, maxX), uy(minY, maxY);
    for (int k = 0; k < 997; ++k) {
        x.push_back(ux(rng));
        y.push_back(uy(rng));
        if (k % 13 == 0) x.back() = maxX + (maxX - minX) * 0.01;
        if (k % 17 == 0) y.back() = std::numeric_limits<double>::quiet_NaN();
        if (k % 19 == 0) x.back() = minX;
        if (k % 23 == 0) y.back() = maxY;
    }
}

TEST(GridApprox, SimdMatchesTheScalarCodeOnEveryTarget) {
    struct Case {
        const char* name;
        Crs src, dst;
        double minX, minY, maxX, maxY, maxError;
        Direction dir;
        bool viaProj;
    };
    const Crs wgs = Crs::FromEpsg(4326), merc = Crs::FromEpsg(3857);
    const std::vector<Case> cases = {
        // A 512 px output tile of a UTM orthophoto (z21, 0.125 of a 3 cm source pixel): no refinement.
        {"3857->32630 inverse z21 tile", Crs::FromEpsg(32630), merc, -281000, 7370000, -280961.8, 7370038.2, 0.00375,
         Direction::Inverse, false},
        // Large unrefined cells (a bound that accepts every cell): interpolations far from
        // linear, where a fused and an unfused multiply-add round differently.
        {"4326->3857 continent, unrefined", wgs, merc, -10, 35, 30, 70, 1e12, Direction::Forward, true},
        {"32617->3857 100 km, unrefined", Crs::FromEpsg(32617), merc, 300000, 4400000, 400000, 4500000, 1e6,
         Direction::Forward, false},
        {"3857->2154 (inverse) France, unrefined", Crs::FromEpsg(2154), merc, -500000, 5000000, 1000000, 6600000, 1e9,
         Direction::Inverse, false},
        // Refined cells (a quadtree several levels deep).
        {"31370->3857 20 km tight", Crs::FromEpsg(31370), merc, 140000, 160000, 160000, 180000, 0.001, Direction::Forward,
         false},
        {"4326->3857 continent z6", wgs, merc, -10, 35, 30, 70, 300, Direction::Forward, true},
        // Exact cells where PROJ fails (latitudes beyond the pole).
        {"4326->3857 beyond the pole", wgs, merc, -10, 60, 10, 95, 0.5, Direction::Forward, true},
    };
    simd::ForceTargetForTest(0);
    for (const auto& c : cases) {
        SCOPED_TRACE(c.name);
        TransformOptions o;
        o.analyticWebMercator = !c.viaProj;
        const auto t = Transform::Create(c.src, c.dst, o);
        GridApproxOptions go;
        go.maxError = c.maxError;
        go.direction = c.dir;
        const GridApprox g = GridApprox::Build(t, c.minX, c.minY, c.maxX, c.maxY, go);
        for (const int cols : {514, 37, 3}) {
            std::vector<double> x, y;
            WarpLikePoints(c.minX, c.minY, c.maxX, c.maxY, cols, cols == 514 ? 514 : 41, 5, x, y);
            std::vector<double> rx = x, ry = y;
            const size_t refFailed = g.TransformPointsScalar(rx.data(), ry.data(), rx.size());
            for (const int64_t bit : simd::SupportedTargetBits()) {
                simd::ForceTargetForTest(bit);
                std::vector<double> sx = x, sy = y;
                EXPECT_EQ(g.TransformPoints(sx.data(), sy.data(), sx.size()), refFailed);
                for (size_t i = 0; i < x.size(); ++i) {
                    ASSERT_EQ(0, std::memcmp(&sx[i], &rx[i], sizeof(double)))
                        << simd::TargetNameOf(bit) << " cols " << cols << " point " << i << " (" << x[i] << ", " << y[i]
                        << ")";
                    ASSERT_EQ(0, std::memcmp(&sy[i], &ry[i], sizeof(double)))
                        << simd::TargetNameOf(bit) << " cols " << cols << " point " << i;
                }
            }
            simd::ForceTargetForTest(0);
        }
    }
}

TEST(GridApprox, SharedCellKernelInterpolatesWholeGroups) {
    // One accepted cell: every full SIMD group is interpolated by the kernel (fused multiply-adds,
    // as the scalar code); only the tail is left. An exact cell and a refined cell are left whole,
    // and so is every point on a target without a native FMA.
    GridApprox::Node nodes[3];
    const double cx[4] = {1000.25, 1100.5, 990.125, 1105.75}, cy[4] = {-50.5, -48.25, 40.125, 45.0};
    for (int k = 0; k < 4; ++k) {
        nodes[0].cx[k] = nodes[1].cx[k] = nodes[2].cx[k] = cx[k];
        nodes[0].cy[k] = nodes[1].cy[k] = nodes[2].cy[k] = cy[k];
    }
    nodes[1].exact = 1;
    nodes[2].child = 0;
    for (int which = 0; which < 3; ++which) {
        kernels::CellGrid grid;
        grid.minX = 10;
        grid.minY = 20;
        grid.maxX = 30;
        grid.maxY = 50;
        grid.cellW = 20;
        grid.cellH = 30;
        grid.invW = 1.0 / 20;
        grid.invH = 1.0 / 30;
        grid.cells = 1;
        grid.nodes = &nodes[which];
        for (const int64_t bit : simd::SupportedTargetBits()) {
            simd::ForceTargetForTest(bit);
            const size_t n = 203;
            std::vector<double> x(n), y(n);
            for (size_t i = 0; i < n; ++i) {
                x[i] = 10 + 20.0 * static_cast<double>(i) / (n - 1);
                y[i] = 20 + 30.0 * static_cast<double>(i % 7) / 6;
            }
            const std::vector<double> x0 = x, y0 = y;
            std::vector<uint32_t> rest(n);
            const size_t r = kernels::InterpolateSharedCells(grid, x.data(), y.data(), n, rest.data());
            std::vector<bool> left(n, false);
            for (size_t k = 0; k < r; ++k) left[rest[k]] = true;
            if (which != 0 || !kernels::SharedCellKernelInterpolates()) {
                EXPECT_EQ(r, n) << simd::TargetNameOf(bit);
                continue;
            }
            EXPECT_LT(r, size_t{64}) << simd::TargetNameOf(bit);
            for (size_t i = 0; i < n; ++i) {
                if (left[i]) continue;
                const double s = (x0[i] - std::fma(0.0, 20.0, 10.0)) / (30.0 - 10.0);
                const double t = (y0[i] - std::fma(0.0, 30.0, 20.0)) / (50.0 - 20.0);
                const double bx = std::fma(s, cx[1] - cx[0], cx[0]), tx = std::fma(s, cx[3] - cx[2], cx[2]);
                const double by = std::fma(s, cy[1] - cy[0], cy[0]), ty = std::fma(s, cy[3] - cy[2], cy[2]);
                EXPECT_EQ(x[i], std::fma(t, tx - bx, bx)) << simd::TargetNameOf(bit) << " point " << i;
                EXPECT_EQ(y[i], std::fma(t, ty - by, by)) << simd::TargetNameOf(bit) << " point " << i;
            }
        }
        simd::ForceTargetForTest(0);
    }
}

TEST(GridApprox, ExactModeAndArguments) {
    const auto t = TransformCache::Global().Get(Crs::FromEpsg(2154), Crs::FromEpsg(3857));
    const GridApprox g = GridApprox::Build(t, 0, 0, 1, 1, GridApproxOptions{});  // maxError 0: exact
    double x = 652000, y = 6862000, ex = x, ey = y;
    EXPECT_EQ(g.TransformPoints(&x, &y, 1), 0u);
    ASSERT_TRUE(t->TransformPoint(ex, ey));
    EXPECT_EQ(x, ex);
    EXPECT_EQ(y, ey);
    GridApproxOptions bad;
    bad.gridSize = 1;
    bad.maxError = 1;
    EXPECT_THROW(GridApprox::Build(t, 0, 0, 1, 1, bad), std::invalid_argument);
    EXPECT_THROW(GridApprox::Build(t, 1, 0, 1, 1, GridApproxOptions{}), std::invalid_argument);
}

TEST(PolyFit, ExactForPolynomials) {
    std::mt19937_64 rng(5);
    std::uniform_real_distribution<double> u(-1000, 1000);
    std::vector<double> x(64), y(64), a(64), b(64);
    for (size_t i = 0; i < x.size(); ++i) {
        x[i] = 500000 + u(rng);
        y[i] = 6500000 + u(rng);
        const double X = x[i] - 500000, Y = y[i] - 6500000;
        a[i] = 3 + 2 * X - 0.5 * Y + 1e-4 * X * X - 2e-5 * X * Y + 3e-9 * X * X * Y;
        b[i] = -7 + 0.1 * X + 1.5 * Y + 5e-6 * Y * Y - 1e-9 * Y * Y * Y;
    }
    for (int d = 1; d <= 3; ++d) {
        const auto f = FitPolynomial(x.data(), y.data(), a.data(), b.data(), x.size(), d);
        ASSERT_TRUE(f.has_value());
        EXPECT_EQ(f->Degree(), d);
        if (d == 3) {
            EXPECT_LT(f->MaxResidual(), 1e-8);
            double ua = 0, ub = 0;
            f->Eval(x[3], y[3], ua, ub);
            EXPECT_NEAR(ua, a[3], 1e-8);
            EXPECT_NEAR(ub, b[3], 1e-8);
            std::vector<double> ea(x.size()), eb(x.size());
            f->Eval(x.data(), y.data(), ea.data(), eb.data(), x.size());
            for (size_t i = 0; i < x.size(); ++i) EXPECT_NEAR(ea[i], a[i], 1e-8);
        } else {
            EXPECT_GT(f->MaxResidual(), 1e-6);
        }
    }
    // Too few / degenerate samples
    EXPECT_FALSE(FitPolynomial(x.data(), y.data(), a.data(), b.data(), 5, 2).has_value());
    std::vector<double> line(16, 1.0);
    EXPECT_FALSE(FitPolynomial(line.data(), y.data(), a.data(), b.data(), 16, 1).has_value());
    EXPECT_FALSE(FitPolynomial(x.data(), y.data(), a.data(), b.data(), x.size(), 4).has_value());
}

TEST(PolyFit, AdaptiveOnProjections) {
    // A COPC tile-sized window (300 m) of real projections: a cubic is within 1 mm.
    struct W {
        int epsg;
        double x, y;
    };
    for (const W w : {W{31370, 150000, 170000}, W{2154, 652000, 6862000}, W{32617, 320000, 4460000}}) {
        const auto t = TransformCache::Global().Get(Crs::FromEpsg(w.epsg), Crs::FromEpsg(3857));
        const auto f = FitPolynomialAdaptive(*t, w.x, w.y, w.x + 300, w.y + 300, 0.001, 8);
        ASSERT_TRUE(f.has_value()) << w.epsg;
        EXPECT_LE(f->MaxResidual(), 0.001);
        // Check off the samples
        std::mt19937_64 rng(1);
        std::uniform_real_distribution<double> d(0, 300);
        double maxErr = 0;
        for (int i = 0; i < 1000; ++i) {
            double px = w.x + d(rng), py = w.y + d(rng), ex = px, ey = py;
            double u = 0, v = 0;
            f->Eval(px, py, u, v);
            ASSERT_TRUE(t->TransformPoint(ex, ey));
            maxErr = std::max(maxErr, std::hypot(u - ex, v - ey));
        }
        std::printf("[ polyfit  ] EPSG:%d 300 m window: degree %d, residual %.3g m, off-sample max %.3g m\n", w.epsg,
                    f->Degree(), f->MaxResidual(), maxErr);
        EXPECT_LE(maxErr, 0.002);
    }
}

}  // namespace
