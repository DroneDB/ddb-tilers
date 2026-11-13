// SPDX-License-Identifier: MPL-2.0

#include "core/tile/web_mercator.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ddbt::tile {

namespace {

constexpr double kPi = std::numbers::pi;

// Clamps a real tile index to [0, 2^z - 1]; NaN goes to 0.
int64_t ClampIndex(double v, int z) noexcept {
    const double last = std::ldexp(1.0, z) - 1.0;
    if (!(v > 0.0)) return 0;
    if (v >= last) return static_cast<int64_t>(last);
    return static_cast<int64_t>(v);
}

}  // namespace

WebMercatorQuad::WebMercatorQuad(int tileSize) : tileSize_(tileSize), sizeShift_(0) {
    if (tileSize < 1 || tileSize > 4096 || (tileSize & (tileSize - 1)) != 0)
        throw std::invalid_argument("tile size must be a power of two in [1, 4096]");
    int shift = 0;
    for (int s = tileSize; s > 1; s >>= 1) ++shift;
    sizeShift_ = shift - 8;
    // Same expression and evaluation order as GlobalMercator (DroneDB) and gdal2tiles.
    initialResolution_ = 2.0 * kPi * 6378137.0 / static_cast<double>(tileSize);
}

double WebMercatorQuad::Resolution(int z) const noexcept {
    // Division by an exact power of two: identical to initialResolution / std::pow(2, z).
    return std::ldexp(initialResolution_, -z);
}

double WebMercatorQuad::TileSpan(int z) noexcept { return std::ldexp(2.0 * kOriginShift, -z); }

int WebMercatorQuad::ZoomForResolution(double res, ZoomRounding rounding, int maxZoom) const noexcept {
    // GDAL's COG driver rule (COGGetWarpingCharacteristics): the first zoom whose resolution is
    // finer than res, or equal within 1e-8 relative; then LOWER / UPPER / AUTO.
    if (!(res > 0.0)) return 0;
    int z = 0;
    double prev = 0.0;
    double cur = Resolution(0);
    for (; z <= maxZoom; ++z) {
        cur = Resolution(z);
        if (res > cur || std::fabs(res - cur) / cur <= 1e-8) break;
        prev = cur;
    }
    if (z > maxZoom) return maxZoom;
    if (z > 0 && std::fabs(res - cur) / cur > 1e-8) {
        switch (rounding) {
            case ZoomRounding::Down: --z; break;
            case ZoomRounding::Up: break;
            case ZoomRounding::Nearest:
                if (prev / res < res / cur) --z;
                break;
        }
    }
    return z;
}

Bounds WebMercatorQuad::TileBounds(int z, int64_t x, int64_t y, Scheme scheme) const noexcept {
    // GlobalMercator works with TMS rows: pixelsToMeters(tx * size, ty * size) etc.
    const int64_t tmsY = scheme == Scheme::TMS ? y : ((int64_t{1} << z) - 1 - y);
    const double size = static_cast<double>(tileSize_);
    const double res = Resolution(z);
    Bounds b;
    b.minX = static_cast<double>(x) * size * res - kOriginShift;
    b.minY = static_cast<double>(tmsY) * size * res - kOriginShift;
    b.maxX = (static_cast<double>(x) + 1.0) * size * res - kOriginShift;
    b.maxY = (static_cast<double>(tmsY) + 1.0) * size * res - kOriginShift;
    return b;
}

Bounds WebMercatorQuad::TileLonLatBounds(int z, int64_t x, int64_t y, Scheme scheme) const noexcept {
    const Bounds m = TileBounds(z, x, y, scheme);
    return {MetersToLon(m.minX), MetersToLat(m.minY), MetersToLon(m.maxX), MetersToLat(m.maxY)};
}

TileKey WebMercatorQuad::MetersToTile(double mx, double my, int z) const noexcept {
    const double span = TileSpan(z);
    const int64_t x = ClampIndex(std::floor((mx + kOriginShift) / span), z);
    const int64_t y = ClampIndex(std::floor((kOriginShift - my) / span), z);
    return TileKey(z, static_cast<uint32_t>(x), static_cast<uint32_t>(y));
}

TileKey WebMercatorQuad::LonLatToTile(double lon, double lat, int z) noexcept {
    // mercantile.tile(): x = lng / 360 + 0.5, y = 0.5 - 0.25 * ln((1 + sin) / (1 - sin)) / pi,
    // tile = floor((v + 1e-14) * 2^z), v <= 0 -> 0, v >= 1 -> 2^z - 1.
    constexpr double kEpsilon = 1e-14;
    const double fx = lon / 360.0 + 0.5;
    const double sinLat = std::sin(lat * (kPi / 180.0));
    double fy = 0.5 - 0.25 * std::log((1.0 + sinLat) / (1.0 - sinLat)) / kPi;
    if (lat >= 90.0) fy = 0.0;
    if (lat <= -90.0) fy = 1.0;
    const double z2 = std::ldexp(1.0, z);
    const auto index = [&](double f) -> uint32_t {
        if (!(f > 0.0)) return 0;
        if (f >= 1.0) return static_cast<uint32_t>(z2 - 1.0);
        return static_cast<uint32_t>(std::min(z2 - 1.0, std::floor((f + kEpsilon) * z2)));
    };
    return TileKey(z, index(fx), index(fy));
}

TileRange WebMercatorQuad::TilesForBounds(const Bounds& m, int z, Scheme scheme) const noexcept {
    TileRange r;
    r.z = z;
    if (!(m.maxX > m.minX) || !(m.maxY > m.minY)) return r;  // empty (also NaN)
    const double span = TileSpan(z);
    const double last = std::ldexp(1.0, z) - 1.0;
    // A box whose edge lies on a tile edge up to rounding (1e-9 tile: 0.04 mm at z0) does not
    // reach into the neighbouring tile.
    constexpr double kEdgeTolerance = 1e-9;
    const auto lo = [&](double v) {
        return static_cast<int64_t>(std::clamp(std::floor(v + kEdgeTolerance), 0.0, last));
    };
    const auto hi = [&](double v) {
        return static_cast<int64_t>(std::clamp(std::ceil(v - kEdgeTolerance) - 1.0, -1.0, last));
    };
    r.minX = lo((m.minX + kOriginShift) / span);
    r.maxX = hi((m.maxX + kOriginShift) / span);
    // XYZ rows from the north
    r.minY = lo((kOriginShift - m.maxY) / span);
    r.maxY = hi((kOriginShift - m.minY) / span);
    if (r.maxX < r.minX || r.maxY < r.minY) return TileRange{z, 0, 0, -1, -1};
    if (scheme == Scheme::TMS) {
        const int64_t n = (int64_t{1} << z) - 1;
        const int64_t a = n - r.maxY, b = n - r.minY;
        r.minY = a;
        r.maxY = b;
    }
    return r;
}

TileRange WebMercatorQuad::TilesForLonLatBounds(const Bounds& ll, int z, Scheme scheme) const noexcept {
    const auto clampLat = [](double lat) { return std::clamp(lat, -kMaxLatitude, kMaxLatitude); };
    const auto clampM = [](double v) { return std::clamp(v, -kOriginShift, kOriginShift); };
    const Bounds m{clampM(LonToMeters(ll.minX)), clampM(LatToMeters(clampLat(ll.minY))),
                   clampM(LonToMeters(ll.maxX)), clampM(LatToMeters(clampLat(ll.maxY)))};
    return TilesForBounds(m, z, scheme);
}

// gdal2tiles GlobalMercator.LatLonToMeters / MetersToLatLon, same operation order.
double LonToMeters(double lon) noexcept { return lon * kOriginShift / 180.0; }

double LatToMeters(double lat) noexcept {
    const double my = std::log(std::tan((90.0 + lat) * kPi / 360.0)) / (kPi / 180.0);
    return my * kOriginShift / 180.0;
}

double MetersToLon(double mx) noexcept { return (mx / kOriginShift) * 180.0; }

double MetersToLat(double my) noexcept {
    const double lat = (my / kOriginShift) * 180.0;
    return 180.0 / kPi * (2.0 * std::atan(std::exp(lat * kPi / 180.0)) - kPi / 2.0);
}

}  // namespace ddbt::tile
