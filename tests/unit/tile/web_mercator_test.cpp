// SPDX-License-Identifier: MPL-2.0
//
// core/tile/web_mercator (clean API) and the baseline quirks of core/tile/compat: known tiles,
// mercantile's formulas restated here, poles, the antimeridian, XYZ/TMS, 256/512 px, ranges.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <numbers>
#include <random>
#include <unordered_set>

#include "core/tile/compat.h"
#include "core/tile/web_mercator.h"

namespace {

using ddbt::tile::Bounds;
using ddbt::tile::Scheme;
using ddbt::tile::TileKey;
using ddbt::tile::TileRange;
using ddbt::tile::WebMercatorQuad;
namespace compat = ddbt::tile::compat;

constexpr double kPi = std::numbers::pi;

// mercantile 1.2 tile(lng, lat, zoom), restated from its source.
TileKey MercantileTile(double lng, double lat, int zoom) {
    const double x = lng / 360.0 + 0.5;
    const double sinlat = std::sin(lat * kPi / 180.0);
    const double y = 0.5 - 0.25 * std::log((1.0 + sinlat) / (1.0 - sinlat)) / kPi;
    const double z2 = std::pow(2.0, zoom);
    const auto idx = [&](double v) -> uint32_t {
        if (v <= 0) return 0;
        if (v >= 1) return static_cast<uint32_t>(z2 - 1);
        return static_cast<uint32_t>(std::floor((v + 1e-14) * z2));
    };
    return TileKey(zoom, idx(x), idx(y));
}

// mercantile ul(): north-west corner of an XYZ tile, degrees.
void MercantileUl(uint32_t x, uint32_t y, int z, double& lon, double& lat) {
    const double z2 = std::pow(2.0, z);
    lon = x / z2 * 360.0 - 180.0;
    lat = std::atan(std::sinh(kPi * (1 - 2 * y / z2))) * 180.0 / kPi;
}

TEST(TileKey, PackRoundTrip) {
    const TileKey k(22, 3000000, 1234567);
    EXPECT_EQ(k.z(), 22);
    EXPECT_EQ(k.x(), 3000000u);
    EXPECT_EQ(k.y(), 1234567u);
    EXPECT_TRUE(k.Valid());
    EXPECT_EQ(TileKey::FromPacked(k.packed()), k);
    const TileKey deepest(29, (1u << 29) - 1, (1u << 29) - 1);
    EXPECT_EQ(deepest.z(), 29);
    EXPECT_EQ(deepest.x(), (1u << 29) - 1);
    EXPECT_EQ(deepest.y(), (1u << 29) - 1);
    EXPECT_FALSE(TileKey(3, 8, 0).Valid());
    EXPECT_EQ(TileKey::From(3, 2, 1, Scheme::TMS).y(), 6u);
    EXPECT_EQ(TileKey(3, 2, 6).Row(Scheme::TMS), 1u);
    EXPECT_EQ(TileKey(5, 10, 11).Child(3), TileKey(6, 21, 23));
    EXPECT_EQ(TileKey(6, 21, 23).Parent(), TileKey(5, 10, 11));
    EXPECT_LT(TileKey(4, 15, 15), TileKey(5, 0, 0));
    std::unordered_set<TileKey, ddbt::tile::TileKeyHash> set{TileKey(1, 0, 0), TileKey(1, 0, 1), TileKey(1, 0, 0)};
    EXPECT_EQ(set.size(), 2u);
}

TEST(WebMercatorQuad, Constants) {
    EXPECT_DOUBLE_EQ(ddbt::tile::kOriginShift, 20037508.342789244);
    const WebMercatorQuad q256(256), q512(512);
    EXPECT_DOUBLE_EQ(q256.Resolution(0), 156543.03392804097);
    EXPECT_EQ(q512.sizeShift(), 1);
    EXPECT_EQ(q512.EffectiveZoom(17), 18);
    // A 512 px tile at z has the resolution of a 256 px tile at z + 1 and the extent of one at z.
    for (int z = 0; z < 25; ++z) {
        EXPECT_EQ(q512.Resolution(z), q256.Resolution(z + 1));
        EXPECT_EQ(q512.TileBounds(z, 3, 1, Scheme::XYZ), q256.TileBounds(z, 3, 1, Scheme::XYZ));
    }
    EXPECT_THROW(WebMercatorQuad(300), std::invalid_argument);
    EXPECT_THROW(WebMercatorQuad(0), std::invalid_argument);
}

TEST(WebMercatorQuad, KnownTiles) {
    struct K {
        double lon, lat;
        int z;
        uint32_t x, y;
    };
    // Reference rows (XYZ) as given by mercantile / the OSM slippy-map formulas.
    const K known[] = {
        {-0.1278, 51.5074, 10, 511, 340},         // London
        {13.405, 52.52, 15, 17604, 10746},        // Berlin
        {151.2093, -33.8688, 12, 3768, 2457},     // Sydney
        {-122.4194, 37.7749, 18, 41928, 101324},  // San Francisco
        {0.0, 0.0, 1, 1, 1},
        {179.9999, 0.0, 5, 31, 16},  // antimeridian, east side
        {-180.0, 0.0, 5, 0, 16},     // antimeridian, west side
    };
    for (const auto& k : known) {
        EXPECT_EQ(WebMercatorQuad::LonLatToTile(k.lon, k.lat, k.z), TileKey(k.z, k.x, k.y)) << k.lon << "," << k.lat;
    }
}

TEST(WebMercatorQuad, PolesAndAntimeridian) {
    for (int z : {0, 1, 5, 12, 22}) {
        const uint32_t last = (1u << z) - 1;
        EXPECT_EQ(WebMercatorQuad::LonLatToTile(0, 90, z).y(), 0u);
        EXPECT_EQ(WebMercatorQuad::LonLatToTile(0, 89.9999, z).y(), 0u);
        EXPECT_EQ(WebMercatorQuad::LonLatToTile(0, -90, z).y(), last);
        EXPECT_EQ(WebMercatorQuad::LonLatToTile(0, -89.9999, z).y(), last);
        EXPECT_EQ(WebMercatorQuad::LonLatToTile(180, 0, z).x(), last);
        EXPECT_EQ(WebMercatorQuad::LonLatToTile(-180, 0, z).x(), 0u);
        EXPECT_EQ(WebMercatorQuad::LonLatToTile(181, 0, z).x(), last);  // clamped
        EXPECT_EQ(WebMercatorQuad::LonLatToTile(-181, 0, z).x(), 0u);
    }
}

TEST(WebMercatorQuad, LonLatToTileMatchesMercantile) {
    std::mt19937_64 rng(7);
    std::uniform_real_distribution<double> lon(-180.0, 180.0), lat(-85.05, 85.05);
    std::uniform_int_distribution<int> zd(0, 24);
    for (int i = 0; i < 20000; ++i) {
        const double lo = lon(rng), la = lat(rng);
        const int z = zd(rng);
        ASSERT_EQ(WebMercatorQuad::LonLatToTile(lo, la, z), MercantileTile(lo, la, z)) << lo << "," << la << " z" << z;
    }
}

TEST(WebMercatorQuad, TileLonLatBoundsMatchMercantileUl) {
    const WebMercatorQuad q(256);
    std::mt19937_64 rng(11);
    for (int z : {0, 1, 3, 8, 14, 20}) {
        std::uniform_int_distribution<uint32_t> d(0, (1u << z) - 1);
        for (int i = 0; i < 50; ++i) {
            const uint32_t x = d(rng), y = d(rng);
            const Bounds b = q.TileLonLatBounds(z, x, y, Scheme::XYZ);
            double lon = 0, lat = 0;
            MercantileUl(x, y, z, lon, lat);
            EXPECT_NEAR(b.minX, lon, 1e-9);
            EXPECT_NEAR(b.maxY, lat, 1e-9);
            // A point inside the tile maps back to it.
            const double cx = 0.5 * (b.minX + b.maxX);
            const double cy = ddbt::tile::MetersToLat(0.5 * (ddbt::tile::LatToMeters(b.minY) + ddbt::tile::LatToMeters(b.maxY)));
            EXPECT_EQ(WebMercatorQuad::LonLatToTile(cx, cy, z), TileKey(z, x, y));
        }
    }
}

TEST(WebMercatorQuad, MetersToTileAndBack) {
    for (int ts : {256, 512}) {
        const WebMercatorQuad q(ts);
        std::mt19937_64 rng(3);
        std::uniform_real_distribution<double> m(-ddbt::tile::kOriginShift, ddbt::tile::kOriginShift);
        for (int i = 0; i < 5000; ++i) {
            const int z = i % 23;
            const double mx = m(rng), my = m(rng);
            const TileKey k = q.MetersToTile(mx, my, z);
            const Bounds b = q.TileBounds(k);
            EXPECT_LE(b.minX, mx);
            EXPECT_GE(b.maxX, mx);
            EXPECT_LE(b.minY, my);
            EXPECT_GE(b.maxY, my);
        }
        // Edges: the world corners clamp, a tile edge goes east / south (XYZ).
        EXPECT_EQ(q.MetersToTile(-ddbt::tile::kOriginShift, ddbt::tile::kOriginShift, 4), TileKey(4, 0, 0));
        EXPECT_EQ(q.MetersToTile(ddbt::tile::kOriginShift, -ddbt::tile::kOriginShift, 4), TileKey(4, 15, 15));
        EXPECT_EQ(q.MetersToTile(0.0, 0.0, 1), TileKey(1, 1, 1));
    }
}

TEST(WebMercatorQuad, TilesForBounds) {
    const WebMercatorQuad q(256);
    // Exactly one tile's bounds -> that tile only.
    const Bounds one = q.TileBounds(10, 511, 340, Scheme::XYZ);
    TileRange r = q.TilesForBounds(one, 10, Scheme::XYZ);
    EXPECT_EQ(r.Count(), 1);
    EXPECT_TRUE(r.Contains(511, 340));
    r = q.TilesForBounds(one, 10, Scheme::TMS);
    EXPECT_TRUE(r.Contains(511, 1023 - 340));
    // Same box one level deeper: 2 x 2.
    EXPECT_EQ(q.TilesForBounds(one, 11, Scheme::XYZ), (TileRange{11, 1022, 680, 1023, 681}));
    // The whole world, and beyond it: clamped.
    const Bounds world{-3e7, -3e7, 3e7, 3e7};
    EXPECT_EQ(q.TilesForBounds(world, 3, Scheme::XYZ), (TileRange{3, 0, 0, 7, 7}));
    // Empty / degenerate boxes.
    EXPECT_TRUE(q.TilesForBounds(Bounds{1, 1, 1, 2}, 5, Scheme::XYZ).Empty());
    EXPECT_TRUE(q.TilesForBounds(Bounds{NAN, 1, 2, 2}, 5, Scheme::XYZ).Empty());
    // Lon/lat: the poles clamp to the square world.
    EXPECT_EQ(q.TilesForLonLatBounds(Bounds{-180, -90, 180, 90}, 2, Scheme::XYZ), (TileRange{2, 0, 0, 3, 3}));
    const TileRange london = q.TilesForLonLatBounds(Bounds{-0.13, 51.50, -0.12, 51.51}, 10, Scheme::XYZ);
    EXPECT_TRUE(london.Contains(511, 340));
}

TEST(WebMercatorQuad, ZoomForResolution) {
    const WebMercatorQuad q(256);
    using ddbt::tile::ZoomRounding;
    // Exactly a level's resolution: that level whatever the rounding.
    for (int z = 0; z < 25; ++z) {
        for (auto r : {ZoomRounding::Up, ZoomRounding::Down, ZoomRounding::Nearest})
            EXPECT_EQ(q.ZoomForResolution(q.Resolution(z), r), z);
    }
    // R1 pixel size 0.02986 m: z22 has 0.0373 m, z23 0.0187 m.
    EXPECT_EQ(q.ZoomForResolution(0.0298582141, ZoomRounding::Up), 23);
    EXPECT_EQ(q.ZoomForResolution(0.0298582141, ZoomRounding::Down), 22);
    EXPECT_EQ(q.ZoomForResolution(0.0298582141, ZoomRounding::Nearest), 22);
    const WebMercatorQuad q512(512);
    EXPECT_EQ(q512.ZoomForResolution(0.0298582141, ZoomRounding::Up), 22);
}

// ---- compat: behaviours of the baseline that differ from the clean API ---------------------

TEST(TileCompat, InvertedTmsFlag) {
    // tms == true: the caller's row is XYZ and is flipped; tms == false: used as is (TMS).
    EXPECT_EQ(compat::InternalRow(340, 10, true), 1023 - 340);
    EXPECT_EQ(compat::InternalRow(683, 10, false), 683);
    EXPECT_EQ(compat::FlipRow(compat::FlipRow(77, 9), 9), 77);
    // With the flip, compat bounds equal the clean API's bounds of the XYZ tile.
    const compat::GlobalMercator m(256);
    const WebMercatorQuad q(256);
    const auto b = m.tileBounds(511, compat::InternalRow(340, 10, true), 10);
    const Bounds c = q.TileBounds(10, 511, 340, Scheme::XYZ);
    EXPECT_EQ(b.min.x, c.minX);
    EXPECT_EQ(b.min.y, c.minY);
    EXPECT_EQ(b.max.x, c.maxX);
    EXPECT_EQ(b.max.y, c.maxY);
}

TEST(TileCompat, MetersToTileCeilSemantics) {
    // ceil(px / T) - 1: a point on a tile edge belongs to the tile WEST / SOUTH of it, and the
    // world origin gives -1 (the clean API floors and clamps).
    const compat::GlobalMercator m(256);
    const auto t = m.metersToTile(-ddbt::tile::kOriginShift, -ddbt::tile::kOriginShift, 3);
    EXPECT_EQ(t.x, -1);
    EXPECT_EQ(t.y, -1);
    const auto e = m.metersToTile(0.0, 0.0, 1);
    EXPECT_EQ(e.x, 0);
    EXPECT_EQ(e.y, 0);
}

TEST(TileCompat, MinMaxCoordsCropsColumnsOnly) {
    const compat::GlobalMercator m(256);
    const double os = ddbt::tile::kOriginShift;
    const auto b = compat::MinMaxCoordsForZ(m, -os * 1.3, -os * 1.3, os * 1.3, os * 1.3, 2);
    EXPECT_EQ(b.min.x, 0);
    EXPECT_EQ(b.max.x, 3);
    EXPECT_EQ(b.min.y, -1);  // rows not cropped
    EXPECT_EQ(b.max.y, 4);
    EXPECT_TRUE(b.Contains(0, -1));
    EXPECT_TRUE(b.Contains(3, 4));
    EXPECT_FALSE(b.Contains(4, 0));
}

TEST(TileCompat, ZoomForPixelSizeAndLength) {
    const compat::GlobalMercator m(256);
    EXPECT_EQ(m.zoomForPixelSize(200000.0), -1);  // coarser than z0: -1 (gdal2tiles: 0)
    EXPECT_EQ(m.zoomForPixelSize(m.resolution(18)), 18);
    EXPECT_EQ(m.zoomForPixelSize(m.resolution(18) * 0.999), 18);
    EXPECT_EQ(m.zoomForPixelSize(m.resolution(18) * 1.001), 17);
    EXPECT_EQ(m.zoomForPixelSize(1e-30), 0);  // beyond 99 levels
    // round(log2(initialRes / L * 256)): a 155 m wide cloud (P1) -> z18
    EXPECT_EQ(m.zoomForLength(155.0), 18);
    EXPECT_EQ(m.zoomForLength(1527.0), 15);  // P3
    const auto pc = compat::PointCloudZoomRange(m, 0, 0, 155.0, 142.0, 147);
    EXPECT_EQ(pc.min, 18);
    EXPECT_EQ(pc.max, 18 + 5);  // round(log2(147 / 4)) = round(5.2)
}

TEST(TileCompat, RasterBoundsQuirk) {
    // oMinY uses the pixel width (gt[1]), not the height: non-square pixels shift the south bound.
    const double gt[6] = {1000.0, 2.0, 0.0, 5000.0, 0.0, -1.0};
    const auto b = compat::RasterBoundsOf(gt, 100, 100);
    EXPECT_EQ(b.oMinX, 1000.0);
    EXPECT_EQ(b.oMaxX, 1200.0);
    EXPECT_EQ(b.oMaxY, 5000.0);
    EXPECT_EQ(b.oMinY, 4800.0);  // gdal2tiles: 4900
}

TEST(TileCompat, PointCloudPaddingTruncation) {
    // int(res(z) * ceil(T / 40)): 7 px at 256, 13 px at 512; truncated to 0 m from z21 at 256.
    const compat::GlobalMercator m256(256), m512(512);
    EXPECT_EQ(compat::PointCloudBoundsBuffer(m256, 18), static_cast<int>(m256.resolution(18) * 7.0));
    EXPECT_EQ(compat::PointCloudBoundsBuffer(m256, 18), 4);
    EXPECT_EQ(compat::PointCloudBoundsBuffer(m256, 20), 1);
    EXPECT_EQ(compat::PointCloudBoundsBuffer(m256, 21), 0);
    EXPECT_EQ(compat::PointCloudBoundsBuffer(m256, 22), 0);
    EXPECT_EQ(compat::PointCloudBoundsBuffer(m512, 20), static_cast<int>(m512.resolution(20) * 13.0));
    EXPECT_EQ(compat::PointCloudBoundsBuffer(m512, 21), 0);

    const auto tb = m256.tileBounds(100, 200, 10);
    const auto pm = compat::PointCloudPixelMapping(m256, tb, 10);
    EXPECT_EQ(pm.width, 260);
    EXPECT_EQ(pm.height, 260);
    EXPECT_DOUBLE_EQ(pm.shiftX, -2 * m256.resolution(10));
    EXPECT_DOUBLE_EQ(pm.shiftY, 2 * m256.resolution(10));
    EXPECT_NEAR(pm.scaleX * m256.resolution(10), 1.0, 1e-12);
    EXPECT_EQ(pm.originX, tb.min.x);
}

TEST(TileCompat, TileWindowEdges) {
    // A 3857 raster of 1000 x 800 px of 1 m; its tiles at z17 (1.19 m/px at 256).
    const double gt[6] = {1000.0, 1.0, 0.0, 9000.0, 0.0, -1.0};
    const compat::RasterTiling t(256, gt, 1000, 800);
    EXPECT_EQ(t.zooms.max, 17);
    const auto range = compat::MinMaxCoordsForZ(t.mercator, t.bounds.oMinX, t.bounds.oMinY, t.bounds.oMaxX,
                                                t.bounds.oMaxY, 17);
    int nonEmpty = 0;
    for (int tx = range.min.x - 1; tx <= range.max.x + 1; ++tx) {
        for (int ty = range.min.y - 1; ty <= range.max.y + 1; ++ty) {
            compat::GQResult g;
            const bool ok = compat::TileWindow(t, 17, tx, ty, g);
            if (!range.Contains(tx, ty)) {
                EXPECT_FALSE(ok);
                continue;
            }
            if (!ok) continue;
            ++nonEmpty;
            EXPECT_GE(g.r.x, 0);
            EXPECT_GE(g.r.y, 0);
            EXPECT_LE(g.r.x + g.r.xsize, 1000);
            EXPECT_LE(g.r.y + g.r.ysize, 800);
            EXPECT_GE(g.w.x, 0);
            EXPECT_LE(g.w.x + g.w.xsize, 256);
            EXPECT_LE(g.w.y + g.w.ysize, 256);
        }
    }
    EXPECT_GT(nonEmpty, 0);
    // A zero pixel size is refused (the baseline throws).
    const double bad[6] = {0, 0.0, 0, 0, 0, -1};
    EXPECT_FALSE(compat::GeoQuery(bad, 10, 10, 0, 0, 1, 1, 256).has_value());
}

}  // namespace
