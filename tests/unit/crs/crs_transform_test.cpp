// SPDX-License-Identifier: MPL-2.0
//
// core/crs: CRS parsing, transforms against GDAL (osr with GDAL's warp area of interest) and
// cs2cs (data/gen_reference.py), the analytic Web Mercator forms against PROJ bit for bit, the
// inverse direction, the transform cache.

#include <gtest/gtest.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

#include "core/crs/crs.h"
#include "core/crs/transform.h"
#include "core/crs/web_mercator_analytic.h"

namespace {

struct CrsCase {
    const char* name;
    const char* src;
    const char* dst;
    int hasExtent;
    double extent[4];  // minX, minY, maxX, maxY
    int hasAoi;
    double aoi[4];  // GDALComputeAreaOfInterest: west, south, east, north
    int n;
    const double* in;     // x, y, z triplets
    const double* gdal;   // GDAL osr result
    const double* cs2cs;  // cs2cs result (12 decimals)
};

#include "data/crs_reference.inc"

using ddbt::crs::Crs;
using ddbt::crs::CrsError;
using ddbt::crs::CrsType;
using ddbt::crs::Direction;
using ddbt::crs::Transform;
using ddbt::crs::TransformCache;
using ddbt::crs::TransformOptions;

Crs Parse(const char* text) {
    const std::string s(text);
    return s.find('[') != std::string::npos ? Crs::FromWkt(s) : Crs::FromUserInput(s);
}

TEST(Crs, Parse) {
    const Crs bd72 = Crs::FromEpsg(31370);
    EXPECT_EQ(bd72.Definition(), "EPSG:31370");
    EXPECT_EQ(bd72.Name(), "BD72 / Belgian Lambert 72");
    EXPECT_EQ(bd72.Type(), CrsType::Projected);
    EXPECT_EQ(bd72.Epsg(), 31370);
    EXPECT_DOUBLE_EQ(bd72.HorizontalUnitFactor(), 1.0);
    ASSERT_TRUE(bd72.AreaOfUse().has_value());
    EXPECT_LT(bd72.AreaOfUse()->west, 3.0);

    EXPECT_EQ(Crs::FromEpsg(4326).Type(), CrsType::Geographic2D);
    EXPECT_EQ(Crs::FromEpsg(4979).Type(), CrsType::Geographic3D);
    EXPECT_EQ(Crs::FromEpsg(4978).Type(), CrsType::Geocentric);
    EXPECT_EQ(Crs::FromEpsg(7415).Type(), CrsType::Compound);
    EXPECT_EQ(Crs::FromEpsg(5703).Type(), CrsType::Vertical);
    EXPECT_NEAR(Crs::FromEpsg(6599).HorizontalUnitFactor(), 0.30480060960121924, 1e-15);
    EXPECT_NEAR(Crs::FromEpsg(4326).HorizontalUnitFactor(), 0.017453292519943295, 1e-15);

    EXPECT_THROW(Crs::FromEpsg(1), CrsError);
    EXPECT_THROW(Crs::FromUserInput("not a crs"), CrsError);
    EXPECT_THROW(Crs::FromWkt("PROJCS[\"broken\""), CrsError);
    EXPECT_FALSE(Crs().Valid());
}

TEST(Crs, OfficialDefinitionPreferred) {
    // A WKT1 with AUTHORITY["EPSG","3857"] equivalent to the database: the definition becomes the
    // code (GDAL's OGR_CT_PREFER_OFFICIAL_SRS_DEF), so the analytic path and the area of use apply.
    const Crs official = Crs::FromEpsg(3857);
    const Crs fromWkt = Crs::FromWkt(official.ToWkt(ddbt::crs::WktFlavor::Wkt1Gdal));
    EXPECT_EQ(fromWkt.Definition(), "EPSG:3857");
    EXPECT_EQ(fromWkt.Epsg(), 3857);
    EXPECT_TRUE(fromWkt.Equivalent(official));
    // PROJJSON round trip
    const Crs json = Crs::FromProjJson(Crs::FromEpsg(2154).ToProjJson());
    EXPECT_EQ(json.Definition(), "EPSG:2154");
    EXPECT_FALSE(Crs::FromEpsg(2154).Equivalent(Crs::FromEpsg(31370)));
}

TEST(Crs, WktOnlyLasCrs) {
    // The LAS WKT VLR of the corpus: no identifier, kept as PROJJSON.
    const CrsCase* las = nullptr;
    for (const auto& c : kCrsCases)
        if (std::string(c.name) == "wkt_oblique_stereographic_las") las = &c;
    ASSERT_NE(las, nullptr);
    const Crs c = Crs::FromWkt(las->src);
    EXPECT_EQ(c.Type(), CrsType::Projected);
    EXPECT_FALSE(c.Epsg().has_value());
    EXPECT_EQ(c.Definition().front(), '{');
    EXPECT_EQ(c.Name(), "Oblique_Stereographic");
    EXPECT_TRUE(Crs::FromProjJson(c.Definition()).Equivalent(c));
}

TEST(Crs, ProjDataDirIsTheBuildOne) {
    // The build bakes in the proj.db of the PROJ it links (unless DDBT_PROJ_DATA overrides it).
    EXPECT_FALSE(ddbt::crs::ProjDataDir().empty());
}

double MaxAbs(const std::vector<double>& a, const double* b, size_t n, size_t stride, size_t offset) {
    double m = 0;
    for (size_t i = 0; i < n; ++i) m = std::max(m, std::fabs(a[i] - b[i * stride + offset]));
    return m;
}

TEST(CrsTransform, MatchesGdalAndCs2cs) {
    for (const auto& c : kCrsCases) {
        SCOPED_TRACE(c.name);
        const Crs src = Parse(c.src), dst = Parse(c.dst);
        TransformOptions opt;
        if (c.hasExtent) {
            const auto aoi = Transform::AreaOfInterestFor(src, c.extent[0], c.extent[1], c.extent[2], c.extent[3]);
            ASSERT_EQ(aoi.has_value(), c.hasAoi != 0);
            if (aoi) {
                // GDALComputeAreaOfInterest, recomputed: same corners, same operation
                EXPECT_NEAR(aoi->west, c.aoi[0], 1e-9);
                EXPECT_NEAR(aoi->south, c.aoi[1], 1e-9);
                EXPECT_NEAR(aoi->east, c.aoi[2], 1e-9);
                EXPECT_NEAR(aoi->north, c.aoi[3], 1e-9);
            }
            opt.areaOfInterest = aoi;
        }
        const auto t = TransformCache::Global().Get(src, dst, opt);
        const size_t n = static_cast<size_t>(c.n);
        std::vector<double> x(n), y(n), z(n);
        for (size_t i = 0; i < n; ++i) {
            x[i] = c.in[3 * i];
            y[i] = c.in[3 * i + 1];
            z[i] = c.in[3 * i + 2];
        }
        EXPECT_EQ(t->TransformPoints(x.data(), y.data(), z.data(), n), 0u);
        // Tolerances: 1 um in metres, 1e-11 degree (1 um) in degrees; the measured differences
        // are printed (0 means bit-identical).
        const double tolXY = dst.IsGeographic() ? 1e-11 : 1e-6;
        const double dxg = MaxAbs(x, c.gdal, n, 3, 0), dyg = MaxAbs(y, c.gdal, n, 3, 1), dzg = MaxAbs(z, c.gdal, n, 3, 2);
        EXPECT_LE(dxg, tolXY);
        EXPECT_LE(dyg, tolXY);
        EXPECT_LE(dzg, 1e-6);
        // cs2cs prints 12 decimals
        const double dxc = MaxAbs(x, c.cs2cs, n, 3, 0), dyc = MaxAbs(y, c.cs2cs, n, 3, 1);
        EXPECT_LE(dxc, tolXY + 1e-12);
        EXPECT_LE(dyc, tolXY + 1e-12);
        std::printf("[ parity   ] %-34s vs GDAL max |dx| %.3g |dy| %.3g |dz| %.3g, vs cs2cs %.3g %.3g  (%s)\n", c.name,
                    dxg, dyg, dzg, dxc, dyc,
                    t->GetKind() == Transform::Kind::Proj ? t->Definition().substr(0, 60).c_str() : "analytic/identity");

        // Inverse (the same prepared operation run backwards, as GDAL's GetInverse): back to the
        // input. With a datum shift the round trip is not exact: PROJ inverts a 7-parameter
        // Helmert approximately (sign-flipped parameters), 0.1 to 0.5 mm here; 1 mm allowed.
        EXPECT_EQ(t->TransformPoints(x.data(), y.data(), z.data(), n, Direction::Inverse), 0u);
        const double tolBack = src.IsGeographic() ? 1e-9 : 1e-3;
        double back = 0;
        for (size_t i = 0; i < n; ++i) {
            back = std::max(back, std::max(std::fabs(x[i] - c.in[3 * i]), std::fabs(y[i] - c.in[3 * i + 1])));
        }
        EXPECT_LE(back, tolBack);
        std::printf("[ parity   ] %-34s round trip max %.3g\n", c.name, back);
    }
}

TEST(CrsTransform, SinglePointAndFailures) {
    const auto t = TransformCache::Global().Get(Crs::FromEpsg(31370), Crs::FromEpsg(3857));
    double x = 150000, y = 170000;
    ASSERT_TRUE(t->TransformPoint(x, y));
    EXPECT_GT(x, 400000);
    // Non-finite input fails (HUGE_VAL), as in GDAL
    double xs[3] = {150000, NAN, 150000}, ys[3] = {170000, 170000, HUGE_VAL};
    EXPECT_EQ(t->TransformPoints(xs, ys, nullptr, 3), 2u);
    EXPECT_EQ(xs[1], HUGE_VAL);
    EXPECT_EQ(ys[2], HUGE_VAL);
    EXPECT_TRUE(std::isfinite(xs[0]));
    double nx = NAN, ny = 0;
    EXPECT_FALSE(t->TransformPoint(nx, ny));
    // Identity
    const auto id = TransformCache::Global().Get(Crs::FromEpsg(2154), Crs::FromEpsg(2154));
    EXPECT_EQ(id->GetKind(), Transform::Kind::Identity);
    double ix = 1, iy = 2;
    EXPECT_TRUE(id->TransformPoint(ix, iy));
    EXPECT_EQ(ix, 1);
    EXPECT_EQ(iy, 2);
    // No operation: a local engineering CRS (a site grid) has none to EPSG:3857
    const Crs site = Crs::FromWkt(
        "ENGCRS[\"local\",EDATUM[\"site\"],CS[Cartesian,2],AXIS[\"x\",east,LENGTHUNIT[\"metre\",1]],"
        "AXIS[\"y\",north,LENGTHUNIT[\"metre\",1]]]");
    EXPECT_EQ(site.Type(), CrsType::Engineering);
    EXPECT_THROW(Transform::Create(site, Crs::FromEpsg(3857)), CrsError);
    EXPECT_THROW(TransformCache::Global().Get(site, Crs::FromEpsg(3857)), CrsError);
    EXPECT_THROW(TransformCache::Global().Get(site, Crs::FromEpsg(3857)), CrsError);  // not cached
}

uint64_t Bits(double v) { return std::bit_cast<uint64_t>(v); }

TEST(CrsTransform, AnalyticWebMercatorIsPROJBitForBit) {
    TransformOptions viaProj;
    viaProj.analyticWebMercator = false;
    const Crs wgs = Crs::FromEpsg(4326), merc = Crs::FromEpsg(3857);
    const auto analytic = Transform::Create(wgs, merc);
    const auto proj = Transform::Create(wgs, merc, viaProj);
    ASSERT_EQ(analytic->GetKind(), Transform::Kind::WebMercatorFromLonLat);
    ASSERT_EQ(proj->GetKind(), Transform::Kind::Proj);

    std::mt19937_64 rng(42);
    std::uniform_real_distribution<double> lon(-180, 180), lat(-89.999, 89.999);
    const size_t n = 200000;
    std::vector<double> ax(n), ay(n);
    for (size_t i = 0; i < n; ++i) {
        ax[i] = lon(rng);
        ay[i] = lat(rng);
    }
    // Edge values
    const double edges[][2] = {{0, 0}, {-0.0, -0.0}, {180, 90}, {-180, -90}, {179.9999999, 85.0511287798066}, {1e-300, -1e-300}};
    for (size_t i = 0; i < std::size(edges); ++i) {
        ax[i] = edges[i][0];
        ay[i] = edges[i][1];
    }
    std::vector<double> px = ax, py = ay;
    EXPECT_EQ(analytic->TransformPoints(ax.data(), ay.data(), nullptr, n), 0u);
    EXPECT_EQ(proj->TransformPoints(px.data(), py.data(), nullptr, n), 0u);
    size_t diff = 0;
    for (size_t i = 0; i < n; ++i) diff += (Bits(ax[i]) != Bits(px[i])) || (Bits(ay[i]) != Bits(py[i]));
    EXPECT_EQ(diff, 0u) << "forward: points differing from PROJ";

    // Inverse (PJ_INV of the 4326 -> 3857 operation)
    std::vector<double> bx = ax, by = ay;
    EXPECT_EQ(analytic->TransformPoints(ax.data(), ay.data(), nullptr, n, Direction::Inverse), 0u);
    EXPECT_EQ(proj->TransformPoints(bx.data(), by.data(), nullptr, n, Direction::Inverse), 0u);
    diff = 0;
    for (size_t i = 0; i < n; ++i) diff += (Bits(ax[i]) != Bits(bx[i])) || (Bits(ay[i]) != Bits(by[i]));
    EXPECT_EQ(diff, 0u) << "inverse: points differing from PROJ";

    // Out of domain: PROJ's failures are the analytic path's failures.
    double fx[4] = {0, 0, 1000, 0}, fy[4] = {90.0000001, -90.0000001, 0, NAN};
    double gx[4], gy[4];
    std::copy(fx, fx + 4, gx);
    std::copy(fy, fy + 4, gy);
    const size_t fa = analytic->TransformPoints(fx, fy, nullptr, 4);
    const size_t fp = proj->TransformPoints(gx, gy, nullptr, 4);
    EXPECT_EQ(fa, fp);
    for (int i = 0; i < 4; ++i) EXPECT_EQ(Bits(fx[i]), Bits(gx[i])) << i;
}

TEST(CrsTransform, WebMercatorToLonLatIsGdal) {
    // GDAL's 3857 -> 4326 shortcut is what the reference (osr) ran: bit for bit.
    for (const auto& c : kCrsCases) {
        if (std::string(c.name) != "webmercator_to_wgs84") continue;
        const auto t = Transform::Create(Crs::FromEpsg(3857), Crs::FromEpsg(4326));
        ASSERT_EQ(t->GetKind(), Transform::Kind::WebMercatorToLonLatGdal);
        for (int i = 0; i < c.n; ++i) {
            double x = c.in[3 * i], y = c.in[3 * i + 1];
            ASSERT_TRUE(t->TransformPoint(x, y));
            EXPECT_EQ(Bits(x), Bits(c.gdal[3 * i])) << i;
            EXPECT_EQ(Bits(y), Bits(c.gdal[3 * i + 1])) << i;
        }
    }
    // Longitudes beyond the world wrap (GDAL), the inverse is PROJ's forward.
    double x = 3.0 * 20037508.342789244, y = 0;
    const auto t = Transform::Create(Crs::FromEpsg(3857), Crs::FromEpsg(4326));
    ASSERT_TRUE(t->TransformPoint(x, y));
    EXPECT_NEAR(x, 180.0, 1e-9);
}

TEST(CrsTransform, CacheSharesPreparedTransforms) {
    TransformCache cache;
    const Crs a = Crs::FromEpsg(2154), b = Crs::FromEpsg(3857);
    const auto t1 = cache.Get(a, b);
    const auto t2 = cache.Get(Crs::FromUserInput("EPSG:2154"), b);
    EXPECT_EQ(t1.get(), t2.get());
    TransformOptions o;
    o.areaOfInterest = ddbt::crs::LonLatBox{2, 48, 3, 49};
    const auto t3 = cache.Get(a, b, o);
    EXPECT_NE(t1.get(), t3.get());
    EXPECT_EQ(cache.Size(), 2u);
    const auto t4 = cache.GetForExtent(a, b, 640000, 6850000, 665000, 6875000);
    ASSERT_TRUE(t4->Options().areaOfInterest.has_value());
    EXPECT_NEAR(t4->Options().areaOfInterest->west, 2.2, 0.2);
    cache.Clear();
    EXPECT_EQ(cache.Size(), 0u);
    // The transform outlives the cache entry.
    double x = 652000, y = 6862000;
    EXPECT_TRUE(t1->TransformPoint(x, y));
}

#ifdef __linux__
TEST(CrsTransform, PreparedTransformsHoldNoDatabaseConnection) {
    // Each prepared transform used to own a PJ_CONTEXT (one proj.db connection and file
    // descriptor each); the prototype now lives in no context of its own.
    const auto fds = [] {
        size_t n = 0;
        for ([[maybe_unused]] const auto& e : std::filesystem::directory_iterator("/proc/self/fd")) ++n;
        return n;
    };
    TransformCache cache;
    const Crs src = Crs::FromEpsg(31370), dst = Crs::FromEpsg(3857);
    cache.Get(src, dst);
    const size_t before = fds();
    std::vector<std::shared_ptr<const Transform>> keep;
    for (int i = 0; i < 100; ++i) {
        TransformOptions o;
        o.areaOfInterest = ddbt::crs::LonLatBox{4.0 + i * 1e-3, 50.5, 4.5, 51.0};
        keep.push_back(cache.Get(src, dst, o));
        double x = 150000, y = 170000;
        ASSERT_TRUE(keep.back()->TransformPoint(x, y));
    }
    EXPECT_LT(fds(), before + 5);
}
#endif

}  // namespace
