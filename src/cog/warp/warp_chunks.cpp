// SPDX-License-Identifier: MPL-2.0
//
// Ported from GDAL 3.13.3, MIT licence: GDALWarpOperation::CollectChunkListInternal,
// ComputeSourceWindow, ComputeSourceWindowTransformPoints and
// ComputeSourceWindowStartingFromSource of alg/gdalwarpoperation.cpp. The copyright notices and the permission notice of the ported code:
//
//   alg/gdalwarpoperation.cpp:
//     Copyright (c) 2003, Frank Warmerdam <warmerdam@pobox.com>
//     Copyright (c) 2007-2012, Even Rouault <even dot rouault at spatialys.com>
//
//   Permission is hereby granted, free of charge, to any person obtaining a copy of this
//   software and associated documentation files (the "Software"), to deal in the Software
//   without restriction, including without limitation the rights to use, copy, modify, merge,
//   publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons
//   to whom the Software is furnished to do so, subject to the following conditions:
//
//   The above copyright notice and this permission notice shall be included in all copies or
//   substantial portions of the Software.
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//   INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
//   PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
//   FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
//   OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//   DEALINGS IN THE SOFTWARE.
//
// Compiled without floating-point contraction (sources.cmake): the windows must round as GDAL's.
#include "cog/warp/warp_chunks.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>
#include <limits>
#include <optional>

namespace ddbt::cog::warp {

namespace {

constexpr int kStepCount = 21;  // DEFAULT_STEP_COUNT

struct Bounds {
    double minX = std::numeric_limits<double>::infinity(), minY = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity(), maxY = -std::numeric_limits<double>::infinity();
    void Add(double x, double y) {
        minX = std::min(minX, x);
        minY = std::min(minY, y);
        maxX = std::max(maxX, x);
        maxY = std::max(maxY, y);
    }
};

class ChunkCollector {
public:
    ChunkCollector(int srcW, int srcH, const PixelTransformer& t, const WarpChunkOptions& o)
        : srcW_(srcW), srcH_(srcH), t_(t), o_(o) {}

    // CollectChunkListInternal.
    void Collect(int dx, int dy, int dw, int dh) {
        WarpChunk c{dx, dy, dw, dh, 0, 0, 0, 0};
        double fill = 0;
        SourceWindow(c, fill);  // a failure leaves an empty source window, which is skipped
        if (c.sw == 0 || c.sh == 0) return;
        const double memory = (static_cast<double>(o_.srcPixelBits) * c.sw * c.sh +
                               static_cast<double>(o_.dstPixelBits) * dw * dh) /
                              8.0;
        if ((memory > o_.memoryLimit && (dw > 2 || dh > 2)) || (fill > 0 && fill < 0.5 && (dw > 100 || dh > 100))) {
            const int bx = o_.dstBlockWidth, by = o_.dstBlockHeight;
            const bool optimizeSize =
                c.sw > 0 && dh > 0 &&
                (dw > dh ? std::fabs(double(dw) / dh - double(c.sw) / c.sh) < 5 * double(dw) / dh
                         : std::fabs(double(dh) / dw - double(c.sh) / c.sw) < 5 * double(dh) / dw) &&
                dw / 2 >= bx && dh / 2 >= by;
            if (dw > dh && (!optimizeSize || dw / 2 >= bx || dh == 1)) {
                int c1 = dw / 2;
                if (optimizeSize && c1 > bx) c1 = (c1 / bx) * bx;
                Collect(dx, dy, c1, dh);
                Collect(dx + c1, dy, dw - c1, dh);
            } else {
                int c1 = dh / 2;
                if (optimizeSize && c1 > by) c1 = (c1 / by) * by;
                Collect(dx, dy, dw, c1);
                Collect(dx, dy + c1, dw, dh - c1);
            }
            return;
        }
        chunks_.push_back(c);
    }

    std::vector<WarpChunk> Take() { return std::move(chunks_); }

private:
    // ComputeSourceWindowTransformPoints: the bounds accumulate over the calls, as in GDAL.
    void TransformPoints(int dx, int dy, int dw, int dh, bool grid, bool all, Bounds& b, int& samples,
                         int& failed) const {
        std::vector<double> xs, ys;
        const double step = all ? 0 : 1.0 / (kStepCount - 1);
        if (grid) {
            for (int iY = 0; iY < kStepCount + 2; iY++) {
                // GDAL takes 0.5 / the window's width for the first and last rows as well.
                const double ry = (iY == 0) ? 0.5 / dw : (iY <= kStepCount) ? (iY - 1) * step : 1 - 0.5 / dw;
                for (int iX = 0; iX < kStepCount + 2; iX++) {
                    const double rx = (iX == 0) ? 0.5 / dw : (iX <= kStepCount) ? (iX - 1) * step : 1 - 0.5 / dw;
                    xs.push_back(rx * dw + dx);
                    ys.push_back(ry * dh + dy);
                }
            }
        } else if (all) {
            for (int iX = 0; iX <= dw; ++iX) {
                xs.push_back(dx + iX), ys.push_back(dy);
                xs.push_back(dx + iX), ys.push_back(dy + dh);
            }
            for (int iY = 1; iY < dh; ++iY) {
                xs.push_back(dx), ys.push_back(dy + iY);
                xs.push_back(dx + dw), ys.push_back(dy + iY);
            }
        } else {
            for (double r = 0.0; r <= 1.0 + step * 0.5; r += step) {
                xs.push_back(r * dw + dx), ys.push_back(dy);
                xs.push_back(r * dw + dx), ys.push_back(dy + dh);
                xs.push_back(dx), ys.push_back(r * dh + dy);
                xs.push_back(dw + dx), ys.push_back(r * dh + dy);
            }
        }
        std::vector<char> ok(xs.size(), 0);
        t_(true, xs.data(), ys.data(), xs.size(), ok.data());
        samples = static_cast<int>(xs.size());
        failed = 0;
        for (size_t i = 0; i < xs.size(); ++i) {
            if (!ok[i] || std::isnan(xs[i]) || std::isnan(ys[i])) {
                ++failed;
                continue;
            }
            b.Add(xs[i], ys[i]);
        }
    }

    // ComputeSourceWindowStartingFromSource: a grid of the source sent forward; the source
    // positions that land in the destination window extend the bounds.
    void StartingFromSource(int dx, int dy, int dw, int dh, Bounds& b) {
        const double step = 1.0 / (kStepCount - 1);
        auto ratioX = [&](int i) {
            return i == 0 ? 0.5 / srcW_ : i <= kStepCount ? (i - 1) * step : 1 - 0.5 / srcW_;
        };
        auto ratioY = [&](int i) {
            return i == 0 ? 0.5 / srcH_ : i <= kStepCount ? (i - 1) * step : 1 - 0.5 / srcH_;
        };
        if (!forward_) {
            Forward f;
            for (int iY = 0; iY < kStepCount + 2; iY++)
                for (int iX = 0; iX < kStepCount + 2; iX++) {
                    f.xs.push_back(ratioX(iX) * srcW_);
                    f.ys.push_back(ratioY(iY) * srcH_);
                }
            f.ok.assign(f.xs.size(), 0);
            t_(false, f.xs.data(), f.ys.data(), f.xs.size(), f.ok.data());
            forward_ = std::move(f);
        }
        size_t i = 0;
        for (int iY = 0; iY < kStepCount + 2; iY++)
            for (int iX = 0; iX < kStepCount + 2; iX++, i++) {
                const double x = forward_->xs[i], y = forward_->ys[i];
                if (forward_->ok[i] && x >= dx && x <= dx + dw && y >= dy && y <= dy + dh)
                    b.Add(ratioX(iX) * srcW_, ratioY(iY) * srcH_);
            }
    }

    // ComputeSourceWindow without SAMPLE_GRID / SAMPLE_STEPS. GDAL samples again with
    // CHECK_WITH_INVERT_PROJ when the window is beyond a million pixels of the raster: that
    // check does not change a transform that goes back to where it came from, and the bounds
    // accumulate, so the window is the same; it is not repeated here.
    void SourceWindow(WarpChunk& c, double& fill) {
        const int dx = c.dx, dy = c.dy, dw = c.dw, dh = c.dh;
        bool all = false, grid = false;
        {
            double cx[4] = {double(dx), double(dx + dw), double(dx), double(dx + dw)};
            double cy[4] = {double(dy), double(dy), double(dy + dh), double(dy + dh)};
            char ok[4] = {0, 0, 0, 0};
            t_(true, cx, cy, 4, ok);
            all = !ok[0] || !ok[1] || !ok[2] || !ok[3];
        }
        Bounds b;
        int samples = 0, failed = 0;
        TransformPoints(dx, dy, dw, dh, grid, all, b, samples, failed);
        if (o_.specialPointInOutput) {
            grid = true;
            all = false;
            TransformPoints(dx, dy, dw, dh, grid, all, b, samples, failed);
        }
        if (!grid && failed > 0) {
            grid = true;
            all = false;
            TransformPoints(dx, dy, dw, dh, grid, all, b, samples, failed);
        }
        if (failed > samples - 5) return;  // "Too many points failed to transform": skipped
        if (grid) StartingFromSource(dx, dy, dw, dh, b);
        if (b.minX > srcW_ || b.maxX < 0 || b.minY > srcH_ || b.maxY < 0) return;
        auto roundIfCloseEnough = [](double v) {
            const double r = std::round(v);
            return std::fabs(r - v) < 1e-6 ? r : v;
        };
        const double minX = roundIfCloseEnough(b.minX), minY = roundIfCloseEnough(b.minY);
        const double maxX = roundIfCloseEnough(b.maxX), maxY = roundIfCloseEnough(b.maxY);
        const int res = o_.filterRadius;
        const double xScale = std::max(1e-3, static_cast<double>(dw) / (maxX - minX));
        const double yScale = std::max(1e-3, static_cast<double>(dh) / (maxY - minY));
        int xRadius = xScale < 0.95 ? static_cast<int>(std::ceil(res / xScale)) : res;
        int yRadius = yScale < 0.95 ? static_cast<int>(std::ceil(res / yScale)) : res;
        if (o_.sourceExtra >= 0) {
            xRadius += o_.sourceExtra;
            yRadius += o_.sourceExtra;
        } else if (failed > 0) {
            xRadius += 10;
            yRadius += 10;
        }
        const int minXc = static_cast<int>(std::max(0.0, minX));
        const int minYc = static_cast<int>(std::max(0.0, minY));
        const int maxXc = static_cast<int>(std::min(std::ceil(maxX), static_cast<double>(srcW_)));
        const int maxYc = static_cast<int>(std::min(std::ceil(maxY), static_cast<double>(srcH_)));
        if (maxXc - minXc > 0.9 * srcW_) {
            c.sx = 0;
            c.sw = srcW_;
        } else {
            c.sx = std::max(0, std::min(minXc - xRadius, srcW_));
            c.sw = std::max(0, std::min(srcW_ - c.sx, maxXc - c.sx + xRadius));
        }
        if (maxYc - minYc > 0.9 * srcH_) {
            c.sy = 0;
            c.sh = srcH_;
        } else {
            c.sy = std::max(0, std::min(minYc - yRadius, srcH_));
            c.sh = std::max(0, std::min(srcH_ - c.sy, maxYc - c.sy + yRadius));
        }
        fill = static_cast<double>(c.sw) * c.sh /
               std::max(1.0, (maxX - minX + 2 * xRadius) * (maxY - minY + 2 * yRadius));
    }

    struct Forward {
        std::vector<double> xs, ys;
        std::vector<char> ok;
    };

    int srcW_, srcH_;
    const PixelTransformer& t_;
    const WarpChunkOptions& o_;
    std::optional<Forward> forward_;  // the source grid sent forward, computed once (as GDAL caches it)
    std::vector<WarpChunk> chunks_;
};

}  // namespace

ChunkScale WarpChunkScale(int dw, int dh, const PixelTransformer& t) {
    const int nx = std::min(10, dw), ny = std::min(10, dh);
    std::vector<double> xs, ys;
    xs.reserve(static_cast<size_t>(4 * nx * ny));
    ys.reserve(static_cast<size_t>(4 * nx * ny));
    for (int iy = 0; iy < ny; ++iy) {
        const double y = ((iy > 0 && iy == ny - 1) ? -1.0 : 0.0) +
                         (ny == 1 ? 0.0 : static_cast<double>(iy) * dh / (ny - 1));
        for (int ix = 0; ix < nx; ++ix) {
            const double x = ((ix > 0 && ix == nx - 1) ? -1.0 : 0.0) +
                             (nx == 1 ? 0.0 : static_cast<double>(ix) * dw / (nx - 1));
            for (const auto [ox, oy] : {std::pair{0, 0}, {1, 0}, {0, 1}, {1, 1}}) {
                xs.push_back(x + ox);
                ys.push_back(y + oy);
            }
        }
    }
    std::vector<char> ok(xs.size(), 0);
    t(true, xs.data(), ys.data(), xs.size(), ok.data());
    std::vector<std::pair<double, double>> scales;
    const auto sq = [](double v) { return v * v; };
    for (size_t i = 0; i + 3 < xs.size(); i += 4) {
        if (!ok[i] || !ok[i + 1] || !ok[i + 2] || !ok[i + 3]) continue;
        const double vx01 = xs[i + 1] - xs[i], vy01 = ys[i + 1] - ys[i];
        const double vx23 = xs[i + 3] - xs[i + 2], vy23 = ys[i + 3] - ys[i + 2];
        const double vx02 = xs[i + 2] - xs[i], vy02 = ys[i + 2] - ys[i];
        const double vx13 = xs[i + 3] - xs[i + 1], vy13 = ys[i + 3] - ys[i + 1];
        const double l01 = sq(vx01) + sq(vy01), l23 = sq(vx23) + sq(vy23);
        const double l02 = sq(vx02) + sq(vy02), l13 = sq(vx13) + sq(vy13);
        constexpr double kTanModestAngle = 0.35, kLengthTolerance = 0.1, kEpsilon = 1e-10;
        if (std::fabs(vy01) < kTanModestAngle * vx01 && std::fabs(vy23) < kTanModestAngle * vx23 &&
            std::fabs(l01 - l23) < kLengthTolerance * std::fabs(l01) &&
            std::fabs(l02 - l13) < kLengthTolerance * std::fabs(l02)) {
            const double lx = std::sqrt(std::sqrt(l01 * l23)), ly = std::sqrt(std::sqrt(l02 * l13));
            if (lx > kEpsilon && ly > kEpsilon) scales.emplace_back(1.0 / lx, 1.0 / ly);
        } else {
            // The area of the square (corners 0, 1, 3, 2), as GDAL's getArea sums it
            const double px[4] = {xs[i], xs[i + 1], xs[i + 3], xs[i + 2]};
            const double py[4] = {ys[i], ys[i + 1], ys[i + 3], ys[i + 2]};
            double sum = px[0] * (py[1] - py[3]);
            for (int k = 1; k < 3; ++k) sum += px[k] * (py[k + 1] - py[k - 1]);
            sum += px[3] * (py[0] - py[2]);
            const double factor = std::sqrt(0.5 * std::fabs(sum));
            if (factor > kEpsilon) scales.emplace_back(1.0 / factor, 1.0 / factor);
        }
    }
    ChunkScale out{0.0, 0.0};
    if (!scales.empty()) {
        std::sort(scales.begin(), scales.end(),
                  [](const auto& a, const auto& b) { return a.first * a.second < b.first * b.second; });
        double xMax = 0, yMax = 0;
        for (const auto& [x, y] : scales) {
            xMax = std::max(xMax, x);
            yMax = std::max(yMax, y);
        }
        int n = 0;
        for (const auto& [x, y] : scales) {
            if (x > 0.1 * xMax && y > 0.1 * yMax) {
                ++n;
                const double dx = x - out.x, dy = y - out.y, inv = 1.0 / n;
                out.x += dx * inv;
                out.y += dy * inv;
            }
        }
    }
    const auto snap = [](double scale) {
        if (scale <= 0) scale = 1.0;
        if (scale < 1.0) {
            const double r = 1.0 / scale;
            const int n = static_cast<int>(r + 0.5);
            if (std::fabs(r - n) < 0.05) scale = 1.0 / n;
        }
        return scale;
    };
    out.x = snap(out.x);
    out.y = snap(out.y);
    return out;
}

std::vector<WarpChunk> CollectWarpChunks(int x, int y, int w, int h, int srcW, int srcH, const PixelTransformer& t,
                                         const WarpChunkOptions& options) {
    ChunkCollector c(srcW, srcH, t, options);
    c.Collect(x, y, w, h);
    std::vector<WarpChunk> chunks = c.Take();
    // GDAL's order: top to bottom, then left to right.
    std::sort(chunks.begin(), chunks.end(),
              [](const WarpChunk& a, const WarpChunk& b) { return a.dy != b.dy ? a.dy < b.dy : a.dx < b.dx; });
    return chunks;
}

}  // namespace ddbt::cog::warp
