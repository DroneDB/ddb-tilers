// SPDX-License-Identifier: MPL-2.0
// Compiled without floating-point contraction (sources.cmake): the grid must round as GDAL's.
#include "cog/warp/target_grid.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>
#include <vector>

namespace ddbt::cog::warp {

namespace {

constexpr double kHalf = 6378137 * std::numbers::pi;  // half the EPSG:3857 world

bool Ok(double x, double y) { return std::isfinite(x) && std::isfinite(y) && x != HUGE_VAL && y != HUGE_VAL; }

// Source pixel -> EPSG:3857 (GDAL's GenImgProj transformer, forward), optionally with OGR's
// CHECK_WITH_INVERT_PROJ: the point is transformed back and rejected when it moves by more than
// the threshold (0.1 for geographic sources, 10000 otherwise).
struct Forward {
    const crs::Transform& t;
    const std::array<double, 6>& gt;
    bool invertCheck = false;
    double threshold = 10000;

    void operator()(std::vector<double>& xs, std::vector<double>& ys, std::vector<char>& ok) const {
        const size_t n = xs.size();
        std::vector<double> gx(n), gy(n);
        for (size_t i = 0; i < n; ++i) {
            gx[i] = gt[0] + xs[i] * gt[1] + ys[i] * gt[2];
            gy[i] = gt[3] + xs[i] * gt[4] + ys[i] * gt[5];
        }
        xs = gx;
        ys = gy;
        t.TransformPoints(xs.data(), ys.data(), nullptr, n);
        ok.assign(n, 1);
        for (size_t i = 0; i < n; ++i) ok[i] = Ok(xs[i], ys[i]);
        if (!invertCheck) return;
        std::vector<double> bx = xs, by = ys;
        t.TransformPoints(bx.data(), by.data(), nullptr, n, crs::Direction::Inverse);
        for (size_t i = 0; i < n; ++i) {
            if (!ok[i]) continue;
            if (!Ok(bx[i], by[i]) || std::fabs(bx[i] - gx[i]) > threshold || std::fabs(by[i] - gy[i]) > threshold) {
                ok[i] = 0;
                xs[i] = ys[i] = HUGE_VAL;
            }
        }
    }
    bool One(double& x, double& y) const {
        std::vector<double> xs{x}, ys{y};
        std::vector<char> ok;
        (*this)(xs, ys, ok);
        x = xs[0];
        y = ys[0];
        return ok[0];
    }
};

struct Suggested {
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    double pixelSize = 0;
    int pixels = 0, lines = 0;
};

// GDALSuggestedWarpOutput2 (alg/gdaltransformer.cpp) for a north-up source to EPSG:3857, without
// its sub-pixel right / bottom border adjustments and the TransformBounds() union: neither changed
// the grid of any raster of the test corpus.
Expected<Suggested> SuggestWarpOutput(const crs::Transform& t, const std::array<double, 6>& gt,
                                      const std::array<double, 6>& invGt, int W, int H, bool invertCheck,
                                      double threshold) {
    const Forward fwd{t, gt, invertCheck, threshold};
    int nSteps = static_cast<int>(static_cast<double>(std::min(W, H)) / 50 + 0.5);
    nSteps = std::clamp(nSteps, 20, 100);
    const double step = 1.0 / nSteps;
    const int n1 = nSteps + 1;
    std::vector<double> xs, ys;
    for (int i = 0; i <= nSteps; ++i) {
        const double r = (i == nSteps) ? 1.0 : i * step;
        xs.push_back(r * W), ys.push_back(0.0);
    }
    for (int i = 0; i <= nSteps; ++i) {
        const double r = (i == nSteps) ? 1.0 : i * step;
        xs.push_back(r * W), ys.push_back(H);
    }
    for (int i = 0; i <= nSteps; ++i) {
        const double r = (i == nSteps) ? 1.0 : i * step;
        xs.push_back(0.0), ys.push_back(r * H);
    }
    for (int i = 0; i <= nSteps; ++i) {
        const double r = (i == nSteps) ? 1.0 : i * step;
        xs.push_back(W), ys.push_back(r * H);
    }
    std::vector<double> inX = xs, inY = ys;
    std::vector<char> ok;
    fwd(xs, ys, ok);
    int failed = 0;
    for (char o : ok) failed += !o;
    if (failed == 0) {
        // The reverse check: back to source pixels, within W / nSteps and H / nSteps.
        std::vector<double> rx = xs, ry = ys;
        t.TransformPoints(rx.data(), ry.data(), nullptr, rx.size(), crs::Direction::Inverse);
        for (size_t i = 0; i < rx.size() && failed == 0; ++i) {
            if (!Ok(rx[i], ry[i])) {
                ++failed;
                break;
            }
            if (invertCheck) {
                // The inverse of the check transform rejects what moves too much as well.
                double fx = rx[i], fy = ry[i];
                t.TransformPoints(&fx, &fy, nullptr, 1);
                if (!Ok(fx, fy)) {
                    ++failed;
                    break;
                }
            }
            const double px = invGt[0] + rx[i] * invGt[1] + ry[i] * invGt[2];
            const double py = invGt[3] + rx[i] * invGt[4] + ry[i] * invGt[5];
            if (std::fabs(px - inX[i]) > W / static_cast<double>(nSteps) ||
                std::fabs(py - inY[i]) > H / static_cast<double>(nSteps))
                ++failed;
        }
    }
    const bool grid = failed != 0;
    if (grid) {
        xs.clear();
        ys.clear();
        for (int i = 0; i <= nSteps; ++i) {
            const double r = (i == nSteps) ? 1.0 : i * step;
            for (int j = 0; j <= nSteps; ++j) {
                const double r2 = (j == nSteps) ? 1.0 : j * step;
                xs.push_back(r2 * W);
                ys.push_back(r * H);
            }
        }
        fwd(xs, ys, ok);
    }
    const int nSamples = static_cast<int>(xs.size());
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    bool got = false;
    auto add = [&](double x, double y) {
        if (!got) {
            minX = maxX = x;
            minY = maxY = y;
            got = true;
        } else {
            minX = std::min(minX, x);
            maxX = std::max(maxX, x);
            minY = std::min(minY, y);
            maxY = std::max(maxY, y);
        }
    };
    failed = 0;
    for (int i = 0; i < nSamples; ++i) {
        const size_t k = static_cast<size_t>(i);
        int xi = 0, yi = 0;
        if (grid) {
            xi = i % n1;
            yi = i / n1;
        } else if (i < 2 * n1) {
            xi = i % n1;
            yi = i < n1 ? 0 : nSteps;
        }
        if (xi > 0 && (ok[k - 1] || ok[k])) {
            // Dichotomy towards a discontinuity (sign change of x, or a failed neighbour).
            double xBefore = xs[k - 1], xAfter = xs[k];
            double inBefore = static_cast<double>(xi - 1) * W / nSteps, inAfter = static_cast<double>(xi) * W / nSteps;
            bool invBefore = !ok[k - 1], invAfter = !ok[k];
            int iter = 0;
            while ((invBefore || invAfter || xBefore * xAfter < 0.0) && iter < 16) {
                double x = (inBefore + inAfter) / 2.0;
                double y = static_cast<double>(yi) * H / nSteps;
                if (fwd.One(x, y)) {
                    add(x, y);
                    if (invBefore || xBefore * x < 0) {
                        invAfter = false;
                        inAfter = (inBefore + inAfter) / 2.0;
                        xAfter = x;
                    } else {
                        invBefore = false;
                        xBefore = x;
                        inBefore = (inBefore + inAfter) / 2.0;
                    }
                } else {
                    if (invBefore)
                        inBefore = (inBefore + inAfter) / 2.0;
                    else if (invAfter)
                        inAfter = (inBefore + inAfter) / 2.0;
                    else
                        break;
                }
                ++iter;
            }
        }
        if (!ok[k]) {
            ++failed;
            continue;
        }
        add(xs[k], ys[k]);
    }
    if (failed > nSamples - 10 || !got)
        return Status(StatusCode::kInvalidArgument, "Too many points failed to transform, unable to compute output bounds");
    double dx = 0, dy = 0;
    if (ok[0] && ok[static_cast<size_t>(nSamples - 1)]) {
        dx = xs[static_cast<size_t>(nSamples - 1)] - xs[0];
        dy = ys[static_cast<size_t>(nSamples - 1)] - ys[0];
    }
    if (dx == 0.0 || dy == 0.0) {
        dx = maxX - minX;
        dy = maxY - minY;
    }
    Suggested s;
    const double diag = std::sqrt(dx * dx + dy * dy);
    s.pixelSize = diag / std::sqrt(static_cast<double>(W) * W + static_cast<double>(H) * H);
    s.pixels = static_cast<int>((maxX - minX) / s.pixelSize + 0.5);
    s.lines = static_cast<int>((maxY - minY) / s.pixelSize + 0.5);
    s.minX = minX;
    s.maxY = maxY;
    s.maxX = minX + s.pixels * s.pixelSize;
    s.minY = maxY - s.lines * s.pixelSize;
    return s;
}

// gdalwarp's check of a suggested extent (GDALWarpCreateOutput): a 21 x 21 grid of target points
// goes to source pixels and back within one target pixel.
bool ExtentRoundTrips(const crs::Transform& t, const std::array<double, 6>& gt, const std::array<double, 6>& invGt,
                      const Suggested& s) {
    constexpr int kSteps = 20;
    for (int i = 0; i <= kSteps; ++i) {
        for (int j = 0; j <= kSteps; ++j) {
            const double ri = i * 1.0 / kSteps, rj = j * 1.0 / kSteps;
            const double ex = (1 - ri) * s.minX + ri * s.maxX, ey = (1 - rj) * s.minY + rj * s.maxY;
            double x = ex, y = ey;
            if (!t.TransformPoint(x, y, nullptr, crs::Direction::Inverse) || !Ok(x, y)) return false;
            const double px = invGt[0] + x * invGt[1] + y * invGt[2], py = invGt[3] + x * invGt[4] + y * invGt[5];
            double gx = gt[0] + px * gt[1] + py * gt[2], gy = gt[3] + px * gt[4] + py * gt[5];
            if (!t.TransformPoint(gx, gy) || !Ok(gx, gy)) return false;
            if (std::fabs(gx - ex) > (s.maxX - s.minX) / s.pixels || std::fabs(gy - ey) > (s.maxY - s.minY) / s.lines)
                return false;
        }
    }
    return true;
}

bool InvertGt(const std::array<double, 6>& gt, std::array<double, 6>& inv) {
    const double det = gt[1] * gt[5] - gt[2] * gt[4];
    if (std::abs(det) < 1e-15) return false;
    const double invDet = 1.0 / det;
    inv[1] = gt[5] * invDet;
    inv[4] = -gt[4] * invDet;
    inv[2] = -gt[2] * invDet;
    inv[5] = gt[1] * invDet;
    inv[0] = (gt[2] * gt[3] - gt[0] * gt[5]) * invDet;
    inv[3] = (-gt[1] * gt[3] + gt[0] * gt[4]) * invDet;
    return true;
}

// gdalwarp's resolution for a known target extent (bKnownTargetExtentButNotResolution): the
// local scale at a 10 x 10 grid of target points, outliers removed, the smallest kept.
double ResolutionForExtent(const crs::Transform& t, const std::array<double, 6>& gt, double minX, double minY,
                           double maxX, double maxY) {
    constexpr int nX = 10, nY = 10;
    const double eps = std::min(maxX - minX, std::abs(maxY - minY)) / 1000;
    std::vector<double> xs, ys;
    for (int iy = 0; iy < nY; ++iy) {
        for (int ix = 0; ix < nX; ++ix) {
            const double x = minX + static_cast<double>(ix) * (maxX - minX) / (nX - 1);
            const double y = minY + static_cast<double>(iy) * (maxY - minY) / (nY - 1);
            xs.push_back(x), ys.push_back(y);
            xs.push_back(ix == nX - 1 ? x - eps : x + eps), ys.push_back(y);
            xs.push_back(x), ys.push_back(iy == nY - 1 ? y - eps : y + eps);
        }
    }
    // The target extent's corners in the source CRS give the origin of the "pixel" coordinates.
    double ox = minX, oy = minY;
    if (!t.TransformPoint(ox, oy, nullptr, crs::Direction::Inverse)) return 0;
    t.TransformPoints(xs.data(), ys.data(), nullptr, xs.size(), crs::Direction::Inverse);
    std::vector<std::pair<double, double>> pairs;
    for (size_t i = 0; i < xs.size(); i += 3) {
        bool good = true;
        for (size_t k = i; k < i + 3; ++k) good = good && Ok(xs[k], ys[k]);
        if (!good) continue;
        auto px = [&](size_t k) { return (xs[k] - ox) / gt[1]; };
        auto py = [&](size_t k) { return std::abs((ys[k] - oy) / gt[5]); };
        const double r1 = std::abs(eps) / std::hypot(px(i + 1) - px(i), py(i + 1) - py(i));
        const double r2 = std::abs(eps) / std::hypot(px(i + 2) - px(i), py(i + 2) - py(i));
        if (std::isfinite(r1) && std::isfinite(r2)) pairs.emplace_back(r1, r2);
    }
    if (pairs.empty()) return 0;
    std::sort(pairs.begin(), pairs.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    const double m1 = pairs[pairs.size() / 2].first;
    std::vector<std::pair<double, double>> kept;
    for (const auto& p : pairs)
        if (p.first > m1 / 10) kept.push_back(p);
    std::sort(kept.begin(), kept.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
    if (kept.empty()) return 0;
    const double m2 = kept[kept.size() / 2].second;
    std::vector<double> res;
    for (const auto& p : kept)
        if (p.second > m2 / 10) res.push_back((p.first + p.second) / 2);
    std::sort(res.begin(), res.end());
    if (res.empty()) return 0;
    const double med = res[res.size() / 2];
    for (const double r : res)
        if (r > med / 10) return r;
    return 0;
}

}  // namespace

Expected<TargetGrid> ComputeTargetGrid(const crs::Crs& srcCrs, const std::array<double, 6>& gt, uint32_t uW,
                                       uint32_t uH, bool evenTileOrigin) {
    TargetGrid g;
    const int W = static_cast<int>(uW), H = static_cast<int>(uH);
    const double ex[4] = {gt[0], gt[0] + W * gt[1] + H * gt[2], gt[3], gt[3] + W * gt[4] + H * gt[5]};
    try {
        static const crs::Crs webMercator = crs::Crs::FromEpsg(3857);
        g.transform = crs::TransformCache::Global().GetForExtent(srcCrs, webMercator, std::min(ex[0], ex[1]),
                                                                std::min(ex[2], ex[3]), std::max(ex[0], ex[1]),
                                                                std::max(ex[2], ex[3]));
    } catch (const std::exception& e) {
        return Status(StatusCode::kInvalidArgument, std::string("Cannot transform to EPSG:3857: ") + e.what());
    }
    std::array<double, 6> invGt{};
    if (!InvertGt(gt, invGt)) return Status(StatusCode::kInvalidArgument, "Cannot invert the source geotransform");
    const crs::Transform& t = *g.transform;
    const bool geographic = srcCrs.Type() == crs::CrsType::Geographic2D || srcCrs.Type() == crs::CrsType::Geographic3D;

    // gdalwarp's output (GDALWarpCreateOutput), then the COG driver's grid from it.
    double minX, minY, maxX, maxY, res;
    constexpr double kMaxLat = 85.0511287798066;
    const double minLon = gt[0], maxLon = gt[0] + gt[1] * W;
    const double maxLat = gt[3], minLat = gt[3] + gt[5] * H;
    if (geographic && gt[2] == 0 && gt[4] == 0 && gt[5] < 0 && minLon >= -180 - 1e-3 && maxLon <= 180 + 1e-3 &&
        ((maxLat > kMaxLat && minLat < kMaxLat) || (maxLat > -kMaxLat && minLat < -kMaxLat))) {
        // Geographic source beyond Mercator's latitudes: the output bounds clamped to them, the
        // resolution from the local scale over that extent.
        double xLL = std::max(minLon, -180.0), yLL = std::max(minLat, -kMaxLat);
        double xUR = std::min(maxLon, 180.0), yUR = std::min(maxLat, kMaxLat);
        if (!t.TransformPoint(xLL, yLL) || !t.TransformPoint(xUR, yUR))
            return Status(StatusCode::kInvalidArgument, "Cannot transform the raster's bounds to EPSG:3857");
        minX = xLL, minY = yLL, maxX = xUR, maxY = yUR;
        res = ResolutionForExtent(t, gt, minX, minY, maxX, maxY);
        if (!(res > 0)) return Status(StatusCode::kInvalidArgument, "Cannot compute the output resolution");
    } else {
        auto s = SuggestWarpOutput(t, gt, invGt, W, H, false, 0);
        if (!s.ok() || !ExtentRoundTrips(t, gt, invGt, *s)) {
            // gdalwarp retries with CHECK_WITH_INVERT_PROJ=TRUE.
            s = SuggestWarpOutput(t, gt, invGt, W, H, true, geographic ? 0.1 : 10000);
        }
        if (!s.ok()) return s.status();
        minX = s->minX, minY = s->minY, maxX = s->maxX, maxY = s->maxY, res = s->pixelSize;
    }
    g.suggestedResolution = res;

    // COGGetWarpingCharacteristics, GoogleMapsCompatible, ZOOM_LEVEL_STRATEGY=UPPER.
    int zoom = 0;
    double zres = 0;
    for (; zoom <= 30; ++zoom) {
        zres = 2 * kHalf / 256 / static_cast<double>(1u << zoom);
        if (res > zres || std::abs(res - zres) / zres <= 1e-8) break;
    }
    if (zoom > 30) return Status(StatusCode::kInvalidArgument, "Could not find an appropriate zoom level");
    zres = 2 * kHalf / 256 / static_cast<double>(1u << zoom);
    const double oriX = -kHalf, oriY = kHalf;
    const double tileExtent = zres * 256;
    const double eps = 0.499 * zres;
    long tlx = static_cast<long>(std::floor((minX - oriX + eps) / tileExtent));
    long tly = static_cast<long>(std::floor((oriY - maxY + eps) / tileExtent));
    long brx = static_cast<long>(std::ceil((maxX - oriX - eps) / tileExtent));
    long bry = static_cast<long>(std::ceil((oriY - minY - eps) / tileExtent));
    const long matrix = 1L << zoom;
    tlx = std::max(0L, tlx);
    tly = std::max(0L, tly);
    brx = std::min(matrix, brx);
    bry = std::min(matrix, bry);
    if (evenTileOrigin) {
        tlx &= ~1L;
        tly &= ~1L;
    }
    const double gMinX = oriX + static_cast<double>(tlx) * tileExtent, gMinY = oriY - static_cast<double>(bry) * tileExtent;
    const double gMaxX = oriX + static_cast<double>(brx) * tileExtent, gMaxY = oriY - static_cast<double>(tly) * tileExtent;
    const long gw = std::lround((gMaxX - gMinX) / zres);
    const long gh = std::lround((gMaxY - gMinY) / zres);
    if (gw <= 0 || gh <= 0 || gw > INT32_MAX || gh > INT32_MAX)
        return Status(StatusCode::kInvalidArgument, "Cannot build COG: empty or oversized output grid");
    g.zoom = zoom;
    g.width = static_cast<uint32_t>(gw);
    g.height = static_cast<uint32_t>(gh);
    // gdalwarp's output geotransform from the extent and the size.
    g.geoTransform = {gMinX, (gMaxX - gMinX) / static_cast<double>(gw), 0.0, gMaxY, 0.0,
                      -(gMaxY - gMinY) / static_cast<double>(gh)};
    return g;
}

}  // namespace ddbt::cog::warp
