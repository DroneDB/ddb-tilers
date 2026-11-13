// SPDX-License-Identifier: MPL-2.0
//
// Line-by-line ports of DroneDB 2.4.1 (git cbc4bffb): src/library/tiler.cpp,
// src/library/gdaltiler.cpp, src/library/pctiler.cpp. Keep the expressions as they are there:
// the point of this file is to produce the same bits. Compiled with -ffp-contract=off (see
// sources.cmake) so that no multiply-add is fused where the baseline rounds twice.

#include "core/tile/compat.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <numbers>

namespace ddbt::tile::compat {

namespace {
constexpr double kPi = std::numbers::pi;  // ddb::kPi

// The baseline's double -> int casts are undefined behaviour for NaN and values beyond int. Where
// the value fits they are kept as they are (same bits); otherwise:

// the window cannot be represented (GeoQuery gives no window: a blank tile). Excludes INT_MIN too,
// so std::abs of the result is defined.
bool FitsInt(double v) noexcept { return v > -2147483648.0 && v < 2147483648.0; }

// tile numbers and zooms saturate (NaN gives INT_MIN).
int SaturateInt(double v) noexcept {
    if (!(v > -2147483649.0)) return std::numeric_limits<int>::min();
    if (!(v < 2147483648.0)) return std::numeric_limits<int>::max();
    return static_cast<int>(v);
}
}  // namespace

GlobalMercator::GlobalMercator(int tileSize) : tileSize_(tileSize) {
    originShift_ = 2.0 * kPi * 6378137.0 / 2.0;
    initialResolution_ = 2.0 * kPi * 6378137.0 / static_cast<double>(tileSize);
    maxZoomLevel_ = 99;
}

Box<Point> GlobalMercator::tileBounds(int tx, int ty, int zoom) const noexcept {
    // Pyramid pixel coords overflow int from z24 (256 px) / z23 (512 px)
    const double size = static_cast<double>(tileSize_);
    const Point min = pixelsToMeters(tx * size, ty * size, zoom);
    const Point max = pixelsToMeters((tx + 1.0) * size, (ty + 1.0) * size, zoom);
    return {min, max};
}

PointI GlobalMercator::metersToTile(double mx, double my, int zoom) const noexcept {
    const Point p = metersToPixels(mx, my, zoom);
    return pixelsToTile(p.x, p.y);
}

Point GlobalMercator::pixelsToMeters(double px, double py, int zoom) const noexcept {
    const double res = resolution(zoom);
    return {px * res - originShift_, py * res - originShift_};
}

Point GlobalMercator::metersToPixels(double mx, double my, int zoom) const noexcept {
    const double res = resolution(zoom);
    return {(mx + originShift_) / res, (my + originShift_) / res};
}

PointI GlobalMercator::pixelsToTile(double px, double py) const noexcept {
    return {SaturateInt(std::ceil(px / static_cast<double>(tileSize_)) - 1),
            SaturateInt(std::ceil(py / static_cast<double>(tileSize_)) - 1)};
}

double GlobalMercator::resolution(int zoom) const noexcept { return initialResolution_ / std::pow(2, zoom); }

int GlobalMercator::zoomForLength(double meterLength) const noexcept {
    return SaturateInt(std::round(std::log(initialResolution_ / meterLength * tileSize_) / std::log(2)));
}

int GlobalMercator::zoomForPixelSize(double pixelSize) const noexcept {
    for (int i = 0; i < maxZoomLevel_; i++) {
        if (pixelSize > resolution(i)) {
            return i - 1;
        }
    }
    return 0;  // "Exceeded max zoom level"
}

Box<PointI> MinMaxCoordsForZ(const GlobalMercator& m, double oMinX, double oMinY, double oMaxX, double oMaxY,
                             int tz) noexcept {
    Box<PointI> b{m.metersToTile(oMinX, oMinY, tz), m.metersToTile(oMaxX, oMaxY, tz)};
    // Crop tiles extending world limits (+-180,+-90)
    b.min.x = std::max<int>(0, b.min.x);
    b.max.x = std::min<int>(SaturateInt(std::pow(2, tz) - 1), b.max.x);
    // (rows are not cropped in the baseline)
    return b;
}

int FlipRow(int ty, int tz) noexcept {
    const int64_t row = static_cast<int64_t>(SaturateInt(std::pow(2, tz) - 1)) - ty;
    return static_cast<int>(std::clamp<int64_t>(row, std::numeric_limits<int>::min(), std::numeric_limits<int>::max()));
}

int InternalRow(int ty, int tz, bool tms) noexcept { return tms ? FlipRow(ty, tz) : ty; }

RasterBounds RasterBoundsOf(const double gt[6], int rasterXSize, int rasterYSize) noexcept {
    RasterBounds b;
    b.oMinX = gt[0];
    b.oMaxX = gt[0] + rasterXSize * gt[1];
    b.oMaxY = gt[3];
    b.oMinY = gt[3] - rasterYSize * gt[1];
    return b;
}

ZoomRange RasterZoomRange(const GlobalMercator& m, const double gt[6], int rasterXSize, int rasterYSize) noexcept {
    ZoomRange z;
    z.max = m.zoomForPixelSize(gt[1]);
    z.min = m.zoomForPixelSize(gt[1] * std::max(rasterXSize, rasterYSize) / m.tileSize());
    return z;
}

std::optional<GQResult> GeoQuery(const double geo[6], int rasterXSize, int rasterYSize, double ulx, double uly,
                                 double lrx, double lry, int querySize) noexcept {
    GQResult o;
    if (std::abs(geo[1]) < std::numeric_limits<double>::epsilon() ||
        std::abs(geo[5]) < std::numeric_limits<double>::epsilon()) {
        return std::nullopt;  // "Invalid geotransform: pixel size is zero"
    }

    const double rx = (ulx - geo[0]) / geo[1] + 0.001;
    const double ry = (uly - geo[3]) / geo[5] + 0.001;
    const double rxsize = (lrx - ulx) / geo[1] + 0.5;
    const double rysize = (lry - uly) / geo[5] + 0.5;
    if (!FitsInt(rx) || !FitsInt(ry) || !FitsInt(rxsize) || !FitsInt(rysize)) return std::nullopt;
    o.r.x = static_cast<int>(rx);
    o.r.y = static_cast<int>(ry);
    o.r.xsize = static_cast<int>(rxsize);
    o.r.ysize = static_cast<int>(rysize);

    if (querySize == 0) {
        o.w.xsize = o.r.xsize;
        o.w.ysize = o.r.ysize;
    } else {
        o.w.xsize = querySize;
        o.w.ysize = querySize;
    }

    o.w.x = 0;
    if (o.r.x < 0) {
        const int rxShift = std::abs(o.r.x);
        if (o.r.xsize > 0) {
            const double wx = o.w.xsize * (static_cast<double>(rxShift) / static_cast<double>(o.r.xsize));
            const double cut = o.r.xsize * (static_cast<double>(rxShift) / static_cast<double>(o.r.xsize));
            if (!FitsInt(wx) || !FitsInt(cut)) return std::nullopt;
            o.w.x = static_cast<int>(wx);
            o.w.xsize = o.w.xsize - o.w.x;
            o.r.xsize = o.r.xsize - static_cast<int>(cut);
        }
        o.r.x = 0;
    }

    if (static_cast<int64_t>(o.r.x) + o.r.xsize > rasterXSize) {
        if (o.r.xsize > 0) {
            const double wxsize = o.w.xsize * (static_cast<double>(rasterXSize) - static_cast<double>(o.r.x)) /
                                  static_cast<double>(o.r.xsize);
            if (!FitsInt(wxsize)) return std::nullopt;
            o.w.xsize = static_cast<int>(wxsize);
        }
        o.r.xsize = rasterXSize - o.r.x;
    }

    o.w.y = 0;
    if (o.r.y < 0) {
        const int ryShift = std::abs(o.r.y);
        if (o.r.ysize > 0) {
            const double wy = o.w.ysize * (static_cast<double>(ryShift) / static_cast<double>(o.r.ysize));
            const double cut = o.r.ysize * (static_cast<double>(ryShift) / static_cast<double>(o.r.ysize));
            if (!FitsInt(wy) || !FitsInt(cut)) return std::nullopt;
            o.w.y = static_cast<int>(wy);
            o.w.ysize = o.w.ysize - o.w.y;
            o.r.ysize = o.r.ysize - static_cast<int>(cut);
        }
        o.r.y = 0;
    }

    if (static_cast<int64_t>(o.r.y) + o.r.ysize > rasterYSize) {
        if (o.r.ysize > 0) {
            const double wysize = o.w.ysize * (static_cast<double>(rasterYSize) - static_cast<double>(o.r.y)) /
                                  static_cast<double>(o.r.ysize);
            if (!FitsInt(wysize)) return std::nullopt;
            o.w.ysize = static_cast<int>(wysize);
        }
        o.r.ysize = rasterYSize - o.r.y;
    }

    return o;
}

RasterTiling::RasterTiling(int tileSize, const double geoTransform[6], int xsize, int ysize)
    : mercator(tileSize), rasterXSize(xsize), rasterYSize(ysize) {
    std::copy(geoTransform, geoTransform + 6, gt);
    bounds = RasterBoundsOf(gt, xsize, ysize);
    zooms = RasterZoomRange(mercator, gt, xsize, ysize);
}

bool TileWindow(const RasterTiling& t, int tz, int tx, int ty, GQResult& g) noexcept {
    const Box<PointI> tMinMax =
        MinMaxCoordsForZ(t.mercator, t.bounds.oMinX, t.bounds.oMinY, t.bounds.oMaxX, t.bounds.oMaxY, tz);
    if (!tMinMax.Contains(tx, ty)) return false;

    const Box<Point> b = t.mercator.tileBounds(tx, ty, tz);
    const auto q = GeoQuery(t.gt, t.rasterXSize, t.rasterYSize, b.min.x, b.max.y, b.max.x, b.min.y,
                            t.mercator.tileSize());
    if (!q) return false;
    g = *q;
    return g.r.xsize > 0 && g.r.ysize > 0 && g.w.xsize > 0 && g.w.ysize > 0;
}

ZoomRange PointCloudZoomRange(const GlobalMercator& m, double oMinX, double oMinY, double oMaxX, double oMaxY,
                              int span) noexcept {
    ZoomRange z;
    z.min = m.zoomForLength(std::min(oMaxX - oMinX, oMaxY - oMinY));
    z.max = z.min + static_cast<int>(std::round(std::log(static_cast<double>(span) / 4.0) / std::log(2)));
    return z;
}

int PointCloudBoundsBuffer(const GlobalMercator& m, int tz) noexcept {
    const double tileResolution = m.resolution(tz);
    // resolution (m / px) * border (px) --> buffer (m), truncated to int as in the baseline
    // float / double: the float is promoted, as in the baseline (written explicitly here)
    return static_cast<int>(tileResolution * std::ceil(static_cast<double>(static_cast<float>(m.tileSize())) / 40.0));
}

PixelMapping PointCloudPixelMapping(const GlobalMercator& m, const Box<Point>& tileBounds, int tz) noexcept {
    const double tileResolution = m.resolution(tz);
    const int pointRadius = kPointRadius;
    const double pointRadiusMeters = pointRadius * tileResolution;
    const int paddedTileSize = m.tileSize() + pointRadius * 2;

    PixelMapping mapping;
    mapping.originX = tileBounds.min.x;
    mapping.shiftX = -pointRadiusMeters;
    mapping.scaleX = paddedTileSize / (tileBounds.max.x - tileBounds.min.x + pointRadiusMeters * 2.0);
    mapping.originY = tileBounds.min.y;
    mapping.shiftY = pointRadiusMeters;
    mapping.scaleY = paddedTileSize / (tileBounds.max.y - tileBounds.min.y + pointRadiusMeters * 2.0);
    mapping.width = mapping.height = paddedTileSize;
    return mapping;
}

}  // namespace ddbt::tile::compat
