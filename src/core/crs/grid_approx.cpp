// SPDX-License-Identifier: MPL-2.0

#include "core/crs/grid_approx.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "core/crs/grid_approx_kernels.h"

namespace ddbt::crs {

namespace {

struct Pending {
    int32_t node;
    double x0, y0, x1, y1;
    int depth;
};

inline bool Ok(double x, double y) { return x != HUGE_VAL && y != HUGE_VAL && std::isfinite(x) && std::isfinite(y); }

}  // namespace

GridApprox GridApprox::Build(std::shared_ptr<const Transform> transform, double minX, double minY, double maxX,
                             double maxY, const GridApproxOptions& options) {
    if (!transform) throw std::invalid_argument("GridApprox: no transform");
    if (!(maxX > minX) || !(maxY > minY)) throw std::invalid_argument("GridApprox: empty rectangle");
    if (options.gridSize < 2) throw std::invalid_argument("GridApprox: gridSize < 2");
    GridApprox g;
    g.transform_ = std::move(transform);
    g.options_ = options;
    g.minX_ = minX;
    g.minY_ = minY;
    g.maxX_ = maxX;
    g.maxY_ = maxY;
    if (!(options.maxError > 0)) return g;  // exact mode

    const int n = options.gridSize;
    const int cells = n - 1;
    g.cells_ = cells;
    g.cellW_ = (maxX - minX) / cells;
    g.cellH_ = (maxY - minY) / cells;
    g.invW_ = 1.0 / g.cellW_;
    g.invH_ = 1.0 / g.cellH_;
    const auto nodeX = [&](int i) { return i == cells ? maxX : minX + i * g.cellW_; };
    const auto nodeY = [&](int j) { return j == cells ? maxY : minY + j * g.cellH_; };

    // Exact values at the grid nodes.
    std::vector<double> gx(static_cast<size_t>(n) * static_cast<size_t>(n)), gy(gx.size());
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            gx[static_cast<size_t>(j * n + i)] = nodeX(i);
            gy[static_cast<size_t>(j * n + i)] = nodeY(j);
        }
    g.transform_->TransformPoints(gx.data(), gy.data(), nullptr, gx.size(), options.direction);
    g.stats_.projEvaluations += gx.size();

    g.nodes_.resize(static_cast<size_t>(cells) * static_cast<size_t>(cells));
    std::vector<Pending> level;
    level.reserve(g.nodes_.size());
    for (int j = 0; j < cells; ++j)
        for (int i = 0; i < cells; ++i) {
            Node& nd = g.nodes_[static_cast<size_t>(j * cells + i)];
            const size_t c[4] = {static_cast<size_t>(j * n + i), static_cast<size_t>(j * n + i + 1),
                                 static_cast<size_t>((j + 1) * n + i), static_cast<size_t>((j + 1) * n + i + 1)};
            for (int k = 0; k < 4; ++k) {
                nd.cx[k] = gx[c[k]];
                nd.cy[k] = gy[c[k]];
            }
            level.push_back({j * cells + i, nodeX(i), nodeY(j), nodeX(i + 1), nodeY(j + 1), 0});
        }
    g.stats_.initialCells = g.nodes_.size();

    // Refine level by level, one PROJ batch per level.
    std::vector<double> qx, qy;
    std::vector<Pending> next;
    while (!level.empty()) {
        // Check points of the cells whose corners all transformed: centre, south, north, west, east.
        qx.clear();
        qy.clear();
        std::vector<int> base(level.size(), -1);
        for (size_t k = 0; k < level.size(); ++k) {
            const Pending& p = level[k];
            const Node& nd = g.nodes_[static_cast<size_t>(p.node)];
            bool cornersOk = true;
            for (int c = 0; c < 4; ++c) cornersOk = cornersOk && Ok(nd.cx[c], nd.cy[c]);
            if (!cornersOk) continue;
            base[k] = static_cast<int>(qx.size());
            const double xm = 0.5 * (p.x0 + p.x1), ym = 0.5 * (p.y0 + p.y1);
            const double px[5] = {xm, xm, xm, p.x0, p.x1};
            const double py[5] = {ym, p.y0, p.y1, ym, ym};
            qx.insert(qx.end(), px, px + 5);
            qy.insert(qy.end(), py, py + 5);
        }
        if (!qx.empty()) {
            g.transform_->TransformPoints(qx.data(), qy.data(), nullptr, qx.size(), options.direction);
            g.stats_.projEvaluations += qx.size();
        }
        next.clear();
        for (size_t k = 0; k < level.size(); ++k) {
            const Pending p = level[k];
            g.stats_.maxDepthReached = std::max(g.stats_.maxDepthReached, p.depth);
            if (base[k] < 0) {  // a corner failed: exact
                Node& nd = g.nodes_[static_cast<size_t>(p.node)];
                nd.exact = 1;
                continue;
            }
            const size_t b = static_cast<size_t>(base[k]);
            const double* ex = &qx[b];
            const double* ey = &qy[b];
            bool checksOk = true;
            for (int c = 0; c < 5; ++c) checksOk = checksOk && Ok(ex[c], ey[c]);
            double err = 0;
            {
                const Node& nd = g.nodes_[static_cast<size_t>(p.node)];
                if (checksOk) {
                    // Bilinear predictions: centre = mean of 4 corners, edge middles = mean of 2.
                    const double ax[5] = {0.25 * (nd.cx[0] + nd.cx[1] + nd.cx[2] + nd.cx[3]), 0.5 * (nd.cx[0] + nd.cx[1]),
                                          0.5 * (nd.cx[2] + nd.cx[3]), 0.5 * (nd.cx[0] + nd.cx[2]),
                                          0.5 * (nd.cx[1] + nd.cx[3])};
                    const double ay[5] = {0.25 * (nd.cy[0] + nd.cy[1] + nd.cy[2] + nd.cy[3]), 0.5 * (nd.cy[0] + nd.cy[1]),
                                          0.5 * (nd.cy[2] + nd.cy[3]), 0.5 * (nd.cy[0] + nd.cy[2]),
                                          0.5 * (nd.cy[1] + nd.cy[3])};
                    for (int c = 0; c < 5; ++c) err = std::max(err, std::hypot(ax[c] - ex[c], ay[c] - ey[c]));
                }
            }
            if (checksOk && err <= options.maxError) {
                g.stats_.maxCheckError = std::max(g.stats_.maxCheckError, err);
                continue;  // accepted leaf
            }
            if (p.depth >= options.maxDepth || g.nodes_.size() + 4 > options.maxCells) {
                g.nodes_[static_cast<size_t>(p.node)].exact = 1;
                continue;
            }
            // Split in four; the check points are the new nodes.
            const Node parent = g.nodes_[static_cast<size_t>(p.node)];
            const int32_t first = static_cast<int32_t>(g.nodes_.size());
            g.nodes_[static_cast<size_t>(p.node)].child = first;
            // Values: corners c0 (x0,y0) c1 (x1,y0) c2 (x0,y1) c3 (x1,y1); checks m (centre), s, n, w, e.
            const double vx[9] = {parent.cx[0], ex[1], parent.cx[1], ex[3], ex[0], ex[4], parent.cx[2], ex[2], parent.cx[3]};
            const double vy[9] = {parent.cy[0], ey[1], parent.cy[1], ey[3], ey[0], ey[4], parent.cy[2], ey[2], parent.cy[3]};
            // 3 x 3 lattice indices (row-major from y0): 0 1 2 / 3 4 5 / 6 7 8
            const int quad[4][4] = {{0, 1, 3, 4}, {1, 2, 4, 5}, {3, 4, 6, 7}, {4, 5, 7, 8}};
            const double xm = 0.5 * (p.x0 + p.x1), ym = 0.5 * (p.y0 + p.y1);
            const double bx[4][2] = {{p.x0, xm}, {xm, p.x1}, {p.x0, xm}, {xm, p.x1}};
            const double by[4][2] = {{p.y0, ym}, {p.y0, ym}, {ym, p.y1}, {ym, p.y1}};
            for (int q = 0; q < 4; ++q) {
                Node child;
                for (int c = 0; c < 4; ++c) {
                    child.cx[c] = vx[quad[q][c]];
                    child.cy[c] = vy[quad[q][c]];
                }
                g.nodes_.push_back(child);
                next.push_back({first + q, bx[q][0], by[q][0], bx[q][1], by[q][1], p.depth + 1});
            }
        }
        level.swap(next);
    }
    for (const Node& nd : g.nodes_) {
        if (nd.child < 0) {
            ++g.stats_.leafCells;
            if (nd.exact) ++g.stats_.exactCells;
        }
    }
    return g;
}

bool GridApprox::Interpolate(double& x, double& y) const {
    const double px = x, py = y;
    if (!(px >= minX_ && px <= maxX_ && py >= minY_ && py <= maxY_)) return false;  // outside or NaN
    const int ci = std::min(cells_ - 1, static_cast<int>((px - minX_) * invW_));
    const int cj = std::min(cells_ - 1, static_cast<int>((py - minY_) * invH_));
    // The multiply-adds are fused (std::fma), whatever the compiler's contraction: the SIMD kernel
    // reproduces these bits, and they are the ones GCC's default contraction gave on x86-64-v3.
    double x0 = std::fma(static_cast<double>(ci), cellW_, minX_), y0 = std::fma(static_cast<double>(cj), cellH_, minY_);
    double x1 = ci + 1 == cells_ ? maxX_ : x0 + cellW_;
    double y1 = cj + 1 == cells_ ? maxY_ : y0 + cellH_;
    const Node* nd = &nodes_[static_cast<size_t>(cj * cells_ + ci)];
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
        nd = &nodes_[static_cast<size_t>(nd->child + q)];
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

size_t GridApprox::TransformExactly(double* xs, double* ys, const std::vector<size_t>& idx) const {
    if (idx.empty()) return 0;
    std::vector<double> ex(idx.size()), ey(idx.size());
    for (size_t k = 0; k < idx.size(); ++k) {
        ex[k] = xs[idx[k]];
        ey[k] = ys[idx[k]];
    }
    const size_t failed = transform_->TransformPoints(ex.data(), ey.data(), nullptr, ex.size(), options_.direction);
    for (size_t k = 0; k < idx.size(); ++k) {
        xs[idx[k]] = ex[k];
        ys[idx[k]] = ey[k];
    }
    return failed;
}

size_t GridApprox::TransformPoints(double* xs, double* ys, size_t n) const {
    if (nodes_.empty()) return transform_->TransformPoints(xs, ys, nullptr, n, options_.direction);
    kernels::CellGrid grid;
    grid.minX = minX_;
    grid.minY = minY_;
    grid.maxX = maxX_;
    grid.maxY = maxY_;
    grid.cellW = cellW_;
    grid.cellH = cellH_;
    grid.invW = invW_;
    grid.invH = invH_;
    grid.cells = cells_;
    grid.nodes = nodes_.data();
    std::vector<size_t> exactIdx;
    constexpr size_t kBlock = 1024;
    uint32_t rest[kBlock];
    for (size_t b = 0; b < n; b += kBlock) {
        const size_t m = std::min(kBlock, n - b);
        const size_t r = kernels::InterpolateSharedCells(grid, xs + b, ys + b, m, rest);
        for (size_t k = 0; k < r; ++k) {
            const size_t i = b + rest[k];
            if (!Interpolate(xs[i], ys[i])) exactIdx.push_back(i);
        }
    }
    return TransformExactly(xs, ys, exactIdx);
}

size_t GridApprox::TransformPointsScalar(double* xs, double* ys, size_t n) const {
    if (nodes_.empty()) return transform_->TransformPoints(xs, ys, nullptr, n, options_.direction);
    std::vector<size_t> exactIdx;
    for (size_t i = 0; i < n; ++i)
        if (!Interpolate(xs[i], ys[i])) exactIdx.push_back(i);
    return TransformExactly(xs, ys, exactIdx);
}

}  // namespace ddbt::crs
