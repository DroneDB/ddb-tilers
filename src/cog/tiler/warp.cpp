// SPDX-License-Identifier: MPL-2.0
#include "cog/tiler/warp.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include "cog/tiler/cog_source.h"
#include "cog/tiler/kernels.h"
#include "cog/tiler/raster_io.h"
#include "cog/tiler/tile_error.h"
#include "core/crs/crs.h"
#include "core/crs/grid_approx.h"

namespace ddbt::cog::tiler {

namespace {

bool InvertGeoTransform(const std::array<double, 6>& gt, std::array<double, 6>& inv) {
    // GDALInvGeoTransform (no rotation fast path kept general).
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

}  // namespace

std::unique_ptr<WarpModel> WarpModel::Create(const CogSource& source) {
    std::unique_ptr<WarpModel> m(new WarpModel());
    m->srcGt_ = source.GeoTransform();
    if (!InvertGeoTransform(m->srcGt_, m->srcInvGt_)) Fail("Cannot invert the geotransform of " + source.Path());
    m->srcPixel_ = std::hypot(m->srcGt_[1], m->srcGt_[4]);

    const int W = source.Width(), H = source.Height();
    const auto& gt = m->srcGt_;
    const double ex[4] = {gt[0], gt[0] + W * gt[1] + H * gt[2], gt[3], gt[3] + W * gt[4] + H * gt[5]};
    static const crs::Crs webMercator = crs::Crs::FromEpsg(3857);
    try {
        m->transform_ = crs::TransformCache::Global().GetForExtent(
            *source.Georef().crs, webMercator, std::min(ex[0], ex[1]), std::min(ex[2], ex[3]), std::max(ex[0], ex[1]),
            std::max(ex[2], ex[3]));
    } catch (const std::exception& e) {
        Fail("Cannot create warped VRT of " + source.Path() + ": " + e.what());
    }

    // GDALSuggestedWarpOutput2: the edges, nSteps + 1 points each, in pixel coordinates moved
    // through the source geotransform and the transform.
    int nSteps = static_cast<int>(static_cast<double>(std::min(W, H)) / 50 + 0.5);
    nSteps = std::clamp(nSteps, 20, 100);
    const double step = 1.0 / nSteps;
    const int n1 = nSteps + 1;
    std::vector<double> xs(static_cast<size_t>(4 * n1)), ys(static_cast<size_t>(4 * n1));
    std::vector<double> px(static_cast<size_t>(4 * n1)), py(static_cast<size_t>(4 * n1));
    for (int i = 0; i <= nSteps; ++i) {
        const double r = (i == nSteps) ? 1.0 : i * step;
        px[static_cast<size_t>(i)] = r * W, py[static_cast<size_t>(i)] = 0.0;
        px[static_cast<size_t>(n1 + i)] = r * W, py[static_cast<size_t>(n1 + i)] = H;
        px[static_cast<size_t>(2 * n1 + i)] = 0.0, py[static_cast<size_t>(2 * n1 + i)] = r * H;
        px[static_cast<size_t>(3 * n1 + i)] = W, py[static_cast<size_t>(3 * n1 + i)] = r * H;
    }
    for (size_t i = 0; i < xs.size(); ++i) {
        xs[i] = gt[0] + px[i] * gt[1] + py[i] * gt[2];
        ys[i] = gt[3] + px[i] * gt[4] + py[i] * gt[5];
    }
    m->transform_->TransformPoints(xs.data(), ys.data(), nullptr, xs.size());
    double minX = std::numeric_limits<double>::max(), minY = minX;
    double maxX = -minX, maxY = -minX;
    size_t ok = 0;
    for (size_t i = 0; i < xs.size(); ++i) {
        if (!std::isfinite(xs[i]) || !std::isfinite(ys[i]) || xs[i] == HUGE_VAL) continue;
        ++ok;
        minX = std::min(minX, xs[i]);
        maxX = std::max(maxX, xs[i]);
        minY = std::min(minY, ys[i]);
        maxY = std::max(maxY, ys[i]);
    }
    if (ok + 10 < xs.size() || ok == 0)
        Fail("Cannot create warped VRT of " + source.Path() + ": too many points failed to transform");
    const size_t last = xs.size() - 1;  // (W, H): the last point of the right edge
    double dx = xs[last] - xs[0], dy = ys[last] - ys[0];
    if (dx == 0.0 || dy == 0.0) {
        dx = maxX - minX;
        dy = maxY - minY;
    }
    const double diag = std::sqrt(dx * dx + dy * dy);
    const double pixelSize = diag / std::sqrt(static_cast<double>(W) * W + static_cast<double>(H) * H);
    m->suggestedRes_ = pixelSize;

    // COGGetWarpingCharacteristics, TILING_SCHEME=GoogleMapsCompatible, ZOOM_LEVEL_STRATEGY=UPPER.
    constexpr double kHalf = 6378137 * std::numbers::pi;
    int zoom = 0;
    double res = 0;
    for (; zoom <= 30; ++zoom) {
        res = 2 * kHalf / 256 / (1 << zoom);
        if (pixelSize > res || std::abs(pixelSize - res) / res <= 1e-8) break;
    }
    if (zoom > 30) Fail("Could not find an appropriate zoom level for " + source.Path());
    res = 2 * kHalf / 256 / (1 << zoom);
    const double oriX = -kHalf, oriY = kHalf;
    const double tileExtent = res * 256;
    const double eps = 0.499 * res;
    // The extent of GDALSuggestedWarpOutput2 (without its sub-pixel border adjustment).
    const int nPixels = static_cast<int>((maxX - minX) / pixelSize + 0.5);
    const int nLines = static_cast<int>((maxY - minY) / pixelSize + 0.5);
    const double eMinX = minX, eMaxY = maxY;
    const double eMaxX = minX + nPixels * pixelSize, eMinY = maxY - nLines * pixelSize;
    int tlx = static_cast<int>(std::floor((eMinX - oriX + eps) / tileExtent));
    int tly = static_cast<int>(std::floor((oriY - eMaxY + eps) / tileExtent));
    int brx = static_cast<int>(std::ceil((eMaxX - oriX - eps) / tileExtent));
    int bry = static_cast<int>(std::ceil((oriY - eMinY - eps) / tileExtent));
    const int matrix = 1 << zoom;
    tlx = std::max(0, tlx);
    tly = std::max(0, tly);
    brx = std::min(matrix, brx);
    bry = std::min(matrix, bry);
    const double gMinX = oriX + tlx * tileExtent, gMinY = oriY - bry * tileExtent;
    const double gMaxX = oriX + brx * tileExtent, gMaxY = oriY - tly * tileExtent;
    const int gw = static_cast<int>(std::round((gMaxX - gMinX) / res));
    const int gh = static_cast<int>(std::round((gMaxY - gMinY) / res));
    if (gw <= 0 || gh <= 0) Fail("Cannot create warped VRT of " + source.Path() + ": empty output");
    m->zoom_ = zoom;
    // gdalwarp's output geotransform from the extent and the size.
    m->gt_ = {gMinX, (gMaxX - gMinX) / gw, 0.0, gMaxY, 0.0, -(gMaxY - gMinY) / gh};
    // The COG driver's overviews: halved (floor) while larger than a 512 px block.
    m->levels_.emplace_back(gw, gh);
    int ow = gw, oh = gh;
    while (ow > 512 || oh > 512) {
        ow = std::max(1, ow / 2);
        oh = std::max(1, oh / 2);
        m->levels_.emplace_back(ow, oh);
    }
    return m;
}

void WarpModel::ToSourcePixels(double* xs, double* ys, size_t n) const {
    const size_t failed = transform_->TransformPoints(xs, ys, nullptr, n, crs::Direction::Inverse);
    for (size_t i = 0; i < n; ++i) {
        if (failed != 0 && (xs[i] == HUGE_VAL || !std::isfinite(xs[i]))) {
            xs[i] = ys[i] = std::numeric_limits<double>::quiet_NaN();
            continue;
        }
        const double x = xs[i], y = ys[i];
        xs[i] = srcInvGt_[0] + x * srcInvGt_[1] + y * srcInvGt_[2];
        ys[i] = srcInvGt_[3] + x * srcInvGt_[4] + y * srcInvGt_[5];
    }
}

void WarpModel::Map(const CogSource& source, int level, std::span<const int32_t> cols,
                    std::span<const int32_t> rows, int& srcLevel, std::vector<int32_t>& sx,
                    std::vector<int32_t>& sy) const {
    const size_t nc = cols.size(), nr = rows.size();
    sx.assign(nc * nr, -1);
    sy.assign(nc * nr, -1);
    const auto& lv = levels_[static_cast<size_t>(level)];
    const double resX = gt_[1] * levels_[0].first / lv.first;
    const double resY = gt_[5] * levels_[0].second / lv.second;

    // The source level whose resolution matches this level (GDAL's overview rule).
    std::vector<std::pair<int, int>> srcLevels;
    for (const auto& l : source.Levels()) srcLevels.emplace_back(static_cast<int>(l.width), static_cast<int>(l.height));
    srcLevel = BestLevel(srcLevels, std::abs(resX) / suggestedRes_, 1.2);
    const double scaleX = static_cast<double>(srcLevels[static_cast<size_t>(srcLevel)].first) / srcLevels[0].first;
    const double scaleY = static_cast<double>(srcLevels[static_cast<size_t>(srcLevel)].second) / srcLevels[0].second;
    const int lw = srcLevels[static_cast<size_t>(srcLevel)].first;
    const int lh = srcLevels[static_cast<size_t>(srcLevel)].second;

    // Pixel centres (EPSG:3857) of the pixels with a sample.
    std::vector<double> xs, ys;
    std::vector<size_t> where;
    xs.reserve(nc * nr);
    ys.reserve(nc * nr);
    where.reserve(nc * nr);
    double bMinX = std::numeric_limits<double>::max(), bMinY = bMinX, bMaxX = -bMinX, bMaxY = -bMinX;
    for (size_t j = 0; j < nr; ++j) {
        if (rows[j] < 0) continue;
        const double y = gt_[3] + (rows[j] + 0.5) * resY;
        for (size_t i = 0; i < nc; ++i) {
            if (cols[i] < 0) continue;
            const double x = gt_[0] + (cols[i] + 0.5) * resX;
            xs.push_back(x);
            ys.push_back(y);
            where.push_back(j * nc + i);
            bMinX = std::min(bMinX, x);
            bMaxX = std::max(bMaxX, x);
            bMinY = std::min(bMinY, y);
            bMaxY = std::max(bMaxY, y);
        }
    }
    if (xs.empty()) return;
    const double padX = std::abs(resX), padY = std::abs(resY);
    crs::GridApproxOptions opt;
    opt.gridSize = 9;
    opt.maxError = 0.001 * srcPixel_;
    opt.direction = crs::Direction::Inverse;
    try {
        const crs::GridApprox approx = crs::GridApprox::Build(transform_, bMinX - padX, bMinY - padY, bMaxX + padX,
                                                              bMaxY + padY, opt);
        approx.TransformPoints(xs.data(), ys.data(), xs.size());
    } catch (const std::exception& e) {
        Fail(std::string("Cannot warp: ") + e.what());
    }
    kernels::SourcePixelsJob job;
    job.xs = xs.data();
    job.ys = ys.data();
    job.where = where.data();
    job.n = xs.size();
    job.invGt = srcInvGt_.data();
    job.scaleX = scaleX;
    job.scaleY = scaleY;
    job.width = lw;
    job.height = lh;
    job.sx = sx.data();
    job.sy = sy.data();
    kernels::SourcePixels(job);
}

}  // namespace ddbt::cog::tiler
