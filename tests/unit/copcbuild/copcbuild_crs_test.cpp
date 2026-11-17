// SPDX-License-Identifier: MPL-2.0
//
// COPC builder: builds into a target CRS. A cloud in longitude / latitude with heights in metres
// (as Metashape exports one) gets a cube sized by its height range in "degrees", so its octree
// never splits horizontally; with a target CRS every point is transformed before it is quantised
// and the octree is cut in metres. Each case checks the point count, a round trip of the points
// back to the source within a millimetre, the CRS written, and the octree report.
#include <gtest/gtest.h>

#include <proj.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "cog/tiler/json_lite.h"
#include "copc/build/copc_builder.h"
#include "copc/build/copc_check.h"
#include "copc/build/point_transform.h"
#include "copc/build/sidecar.h"
#include "copc/copc/copc_file.h"
#include "copc/copc/node_decoder.h"
#include "copc/laz/chunk_decoder.h"
#include "core/crs/crs.h"
#include "core/crs/proj_context.h"
#include "core/crs/transform.h"
#include "core/util/endian.h"
#include "ddbt/ddbt.h"
#include "../test_env.h"
#include "synth_las.h"

namespace ddbt::copc::build {
namespace {

namespace fs = std::filesystem;
namespace json = ddbt::cog::tiler::json;
using copc::testing::BuildLasBytes;
using copc::testing::FillRandom;
using copc::testing::ReadBytes;
using copc::testing::SynthLas;
using copc::testing::WriteBytes;

constexpr size_t kGpsAt = 22;  // GPS time of a format 6 record: the tag of a point

std::string TempDir(const std::string& name) {
    const fs::path p = fs::temp_directory_path() / ("ddbt-copcbuild-crs-" + ddbt::test::ProcessIdString() + "-" + name);
    fs::create_directories(p);
    return p.string();
}

std::string Wkt1(const std::string& crs) { return crs::Crs::FromUserInput(crs).ToWkt(crs::WktFlavor::Wkt1Gdal); }

/// The coordinates of a point.
struct Xyz {
    double x = 0, y = 0, z = 0;
};

/// A format 6 cloud of `n` points around `centre` (x, y, z in the units of the scale), each
/// point tagged by its index in its GPS time (plus `firstTag`).
SynthLas TaggedCloud(uint64_t n, uint32_t seed, const std::array<double, 3>& scale, const std::array<double, 3>& centre,
                     const std::array<int32_t, 3>& extent, double firstTag = 0) {
    SynthLas s;
    s.format = 6;
    s.minor = 4;
    s.laz = true;
    s.scale = scale;
    s.offset = centre;
    FillRandom(s, n, seed, extent);
    for (uint64_t i = 0; i < n; ++i) StoreLE<double>(s.records.data() + i * s.RecordSize() + kGpsAt, firstTag + static_cast<double>(i));
    return s;
}

/// The coordinates of the points of a synthetic cloud, by tag.
std::vector<Xyz> PointsOf(const SynthLas& s) {
    std::vector<Xyz> out(s.count);
    for (uint64_t i = 0; i < s.count; ++i) {
        const uint8_t* r = s.records.data() + i * s.RecordSize();
        out[i] = {LoadLE<int32_t>(r) * s.scale[0] + s.offset[0], LoadLE<int32_t>(r + 4) * s.scale[1] + s.offset[1],
                  LoadLE<int32_t>(r + 8) * s.scale[2] + s.offset[2]};
    }
    return out;
}

/// Replaces the coordinates of a cloud's points with `xyz`, quantised to 1 mm around their centre.
void SetPoints(SynthLas& s, const std::vector<Xyz>& xyz) {
    std::array<double, 3> lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
    for (const Xyz& p : xyz) {
        const double v[3] = {p.x, p.y, p.z};
        for (size_t a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], v[a]);
            hi[a] = std::max(hi[a], v[a]);
        }
    }
    for (size_t a = 0; a < 3; ++a) {
        s.scale[a] = 0.001;
        s.offset[a] = std::round((lo[a] + hi[a]) / 2);
    }
    for (uint64_t i = 0; i < s.count; ++i) {
        uint8_t* r = s.records.data() + i * s.RecordSize();
        const double v[3] = {xyz[i].x, xyz[i].y, xyz[i].z};
        for (size_t a = 0; a < 3; ++a)
            StoreLE<int32_t>(r + 4 * a, static_cast<int32_t>(std::lround((v[a] - s.offset[a]) / s.scale[a])));
    }
}

/// A point of a COPC file: its coordinates and its tag.
struct TaggedPoint {
    Xyz p;
    double tag = 0;
};

std::vector<TaggedPoint> ReadCopc(const std::string& path) {
    std::vector<TaggedPoint> out;
    auto file = CopcFile::Open(path);
    EXPECT_TRUE(file.ok()) << file.status().ToString();
    if (!file.ok()) return out;
    const las::LasHeader& h = (*file)->header();
    auto all = (*file)->hierarchy().All();
    EXPECT_TRUE(all.ok());
    auto dec = laz::ChunkDecoder::Create((*file)->pointFormat(), (*file)->extraBytes(), laz::kLayersAll);
    EXPECT_TRUE(dec.ok());
    const size_t rs = (*dec)->recordSize();
    std::vector<uint8_t> scratch, recs;
    for (const HierarchyNode* node : all->nodes) {
        auto chunk = NodeChunk(**file, *node, scratch);
        EXPECT_TRUE(chunk.ok());
        recs.resize(static_cast<size_t>(node->pointCount) * rs);
        EXPECT_TRUE((*dec)->DecodeRecords(*chunk, static_cast<uint32_t>(node->pointCount), recs).ok());
        for (int32_t i = 0; i < node->pointCount; ++i) {
            const uint8_t* r = recs.data() + static_cast<size_t>(i) * rs;
            out.push_back({{LoadLE<int32_t>(r) * h.scale[0] + h.offset[0], LoadLE<int32_t>(r + 4) * h.scale[1] + h.offset[1],
                            LoadLE<int32_t>(r + 8) * h.scale[2] + h.offset[2]},
                           LoadLE<double>(r + kGpsAt)});
        }
    }
    return out;
}

/// The distance in metres of two points of `crs` (longitude / latitude in degrees, or metres).
double Distance(const crs::Crs& crs, const Xyz& a, const Xyz& b) {
    double dx = a.x - b.x, dy = a.y - b.y;
    if (crs.IsHorizontallyGeographic()) {
        constexpr double kDegree = 3.14159265358979323846 / 180;
        dx *= 6378137.0 * kDegree * std::cos(a.y * kDegree);
        dy *= 6378137.0 * kDegree;
    }
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// Every source point is in the COPC once, and every 7th, transformed back from the COPC's CRS to
/// its source's, is within a millimetre of where it was. `sources` holds the points by tag.
void ExpectRoundTrip(const std::string& copc, const std::vector<std::pair<crs::Crs, std::vector<Xyz>>>& sources,
                     const std::vector<double>& firstTags, bool threeD) {
    auto file = CopcFile::Open(copc);
    ASSERT_TRUE(file.ok());
    auto target = (*file)->Crs();
    ASSERT_TRUE(target.ok());
    const std::vector<TaggedPoint> points = ReadCopc(copc);
    size_t expected = 0;
    for (const auto& s : sources) expected += s.second.size();
    ASSERT_EQ(points.size(), expected);
    std::vector<std::vector<uint8_t>> seen(sources.size());
    for (size_t k = 0; k < sources.size(); ++k) seen[k].assign(sources[k].second.size(), 0);
    double worst = 0;
    size_t checked = 0;
    for (const TaggedPoint& t : points) {
        size_t k = sources.size();
        for (size_t j = 0; j < sources.size(); ++j)
            if (t.tag >= firstTags[j] && t.tag < firstTags[j] + static_cast<double>(sources[j].second.size())) k = j;
        ASSERT_LT(k, sources.size()) << "a point with an unknown tag " << t.tag;
        const size_t i = static_cast<size_t>(t.tag - firstTags[k]);
        ASSERT_EQ(seen[k][i], 0) << "a point written twice";
        seen[k][i] = 1;
        if (i % 7 != 0) continue;
        const crs::Crs& source = sources[k].first;
        auto back = crs::TransformCache::Global().Get(*target, source);
        Xyz p = t.p;
        ASSERT_TRUE(back->TransformPoint(p.x, p.y, threeD ? &p.z : nullptr));
        worst = std::max(worst, Distance(source, sources[k].second[i], p));
        ++checked;
    }
    EXPECT_GT(checked, 100u);
    EXPECT_LT(worst, 0.001) << "a point moved by more than a millimetre";
}

bool HasWarning(const CopcBuildReport& r, const std::string& text) {
    return std::any_of(r.warnings.begin(), r.warnings.end(), [&](const std::string& w) { return w.find(text) != std::string::npos; });
}

/// The depth histogram adds up to the nodes and the points of the build, and its largest node is
/// the one reported.
void ExpectConsistentHistogram(const CopcBuildReport& r) {
    uint64_t nodes = 0, points = 0;
    int last = -1;
    for (const OctreeLevel& l : r.depthHistogram) {
        EXPECT_GT(l.depth, last);
        last = l.depth;
        nodes += l.nodes;
        points += l.points;
    }
    EXPECT_EQ(nodes, r.nodes);
    EXPECT_EQ(points, r.points);
    EXPECT_EQ(last, r.maxDepth);
    EXPECT_GT(r.maxNodePoints, 0u);
    EXPECT_LE(r.maxNodePoints, r.points);
}

// A geographic cloud as Metashape exports one: terrain 0.014 degrees wide rising 147 m from west
// to east (a thin sheet: every column of points within half a metre), at 1e-7 degrees and 1 cm.
SynthLas GeographicCloud(double lon, double lat, uint64_t n, uint32_t seed) {
    constexpr int32_t kHalfWidth = 70000, kHalfRise = 7350;
    SynthLas s = TaggedCloud(n, seed, {1e-7, 1e-7, 0.01}, {lon, lat, 650}, {kHalfWidth, kHalfWidth, kHalfRise});
    for (uint64_t i = 0; i < n; ++i) {
        uint8_t* r = s.records.data() + i * s.RecordSize();
        const int32_t x = LoadLE<int32_t>(r);
        const auto noise = static_cast<int32_t>((i * 2654435761u) % 51) - 25;
        StoreLE<int32_t>(r + 8, static_cast<int32_t>(int64_t{x} * kHalfRise / kHalfWidth) + noise);
    }
    s.wkt = Wkt1("EPSG:4326");
    return s;
}

constexpr uint32_t kLeafMax = 200;

// Leaves of 200 points (100,000 by default) and node grids scaled down alike (148 and 222 cells by
// default: about 0.2 leaves on a surface), so a few hundred thousand points make an octree of the
// shape of a cloud of a hundred million.
CopcBuildOptions Options(const std::string& targetCrs) {
    CopcBuildOptions o;
    o.targetCrs = targetCrs;
    o.lod.leafMax = kLeafMax;
    o.lod.hoistMin = 4;
    o.lod.hoistTotal = 20;
    o.lod.gridCells = 6;
    o.lod.rootGridCells = 10;
    o.threads = 4;
    return o;
}

void ExpectGeographicToUtm(double lon, double lat, const std::string& utm, const std::string& name) {
    const std::string dir = TempDir(name);
    const SynthLas s = GeographicCloud(lon, lat, 400000, 7);
    const std::string in = dir + "/geo.laz", out = dir + "/utm.copc.laz", flat = dir + "/geo.copc.laz";
    ASSERT_TRUE(WriteBytes(in, BuildLasBytes(s)));

    // As the source CRS: one cube in "degrees", sized by the height range.
    CopcBuildReport geo;
    Status st = BuildCopc(in, flat, Options(""), &geo);
    ASSERT_TRUE(st.ok()) << st.ToString();
    EXPECT_FALSE(geo.transformed);
    EXPECT_EQ(geo.crs, geo.sourceCrs);
    EXPECT_GT(geo.maxNodePoints, uint64_t{kLeafMax} * 16);
    EXPECT_TRUE(HasWarning(geo, "degenerate octree"));
    ExpectConsistentHistogram(geo);

    CopcBuildReport rep;
    st = BuildCopc(in, out, Options(utm), &rep);
    ASSERT_TRUE(st.ok()) << st.ToString();
    EXPECT_TRUE(rep.transformed);
    EXPECT_EQ(rep.transformPath, "grid");
    // Planned from the header box transformed: the input is read once, and the cube holds it.
    EXPECT_FALSE(rep.boundsPass);
    EXPECT_FALSE(rep.restarted);
    EXPECT_EQ(rep.points, s.count);
    EXPECT_EQ(rep.sourceCrs, s.wkt);
    EXPECT_TRUE(crs::Crs::FromWkt(rep.crs).Equivalent(crs::Crs::FromUserInput(utm)));
    EXPECT_EQ(rep.crs.rfind("PROJCS[", 0), 0u) << "WKT1 when it can express the CRS: " << rep.crs;
    for (size_t a = 0; a < 3; ++a) {
        EXPECT_EQ(rep.scale[a], 0.001);
        EXPECT_EQ(rep.offset[a], std::round(rep.offset[a]));
    }
    // Cut in metres: no node far above the leaf size, and many more nodes.
    EXPECT_LE(rep.maxNodePoints, uint64_t{kLeafMax} * 16);
    EXPECT_FALSE(HasWarning(rep, "degenerate octree"));
    EXPECT_GT(rep.nodes, 2 * geo.nodes);
    ExpectConsistentHistogram(rep);

    auto file = CopcFile::Open(out);
    ASSERT_TRUE(file.ok());
    ASSERT_TRUE((*file)->metadata().wkt.has_value());
    EXPECT_EQ(*(*file)->metadata().wkt, rep.crs);
    auto chk = CheckCopc(out, 2);
    ASSERT_TRUE(chk.ok());
    EXPECT_TRUE(chk->ok()) << (chk->errors.empty() ? "" : chk->errors[0]);
    EXPECT_EQ(chk->outside, 0u);
    ExpectRoundTrip(out, {{crs::Crs::FromWkt(s.wkt), PointsOf(s)}}, {0}, false);

    // The sidecar describes the COPC: its CRS, and a footprint around the source's centre.
    const std::vector<uint8_t> meta = ReadBytes(out + ".meta.json");
    const auto j = json::Parse(std::string(meta.begin(), meta.end()));
    ASSERT_TRUE(j && j->IsObject());
    ASSERT_NE(j->Find("projection"), nullptr);
    EXPECT_EQ(j->Find("projection")->str, rep.crs);
    const json::Value* centroid = j->Find("centroid");
    ASSERT_NE(centroid, nullptr);
    const json::Value* c = centroid->Find("coordinates");
    ASSERT_NE(c, nullptr);
    ASSERT_GE(c->arr.size(), 2u);
    EXPECT_NEAR(c->arr[0].num, lon, 0.01);
    EXPECT_NEAR(c->arr[1].num, lat, 0.01);
    fs::remove_all(dir);
}

TEST(CopcBuildCrs, GeographicNorthToUtmKeepsEveryPointWithinAMillimetre) {
    ExpectGeographicToUtm(13.84, 46.68, "EPSG:32633", "north");
}

TEST(CopcBuildCrs, GeographicSouthToUtmSouthKeepsEveryPointWithinAMillimetre) {
    ExpectGeographicToUtm(18.42, -33.92, "EPSG:32734", "south");
}

// A 3-D projected CRS: UTM 33N with ellipsoidal heights (no EPSG code names it).
std::string Utm33With3dWkt() {
    PJ_CONTEXT* ctx = crs::detail::NewContext();
    crs::detail::PjPtr utm(proj_create(ctx, "EPSG:32633"));
    EXPECT_TRUE(utm);
    crs::detail::PjPtr utm3d(proj_crs_promote_to_3D(ctx, nullptr, utm.get()));
    EXPECT_TRUE(utm3d);
    const char* wkt = utm3d ? proj_as_wkt(ctx, utm3d.get(), PJ_WKT2_2019, nullptr) : nullptr;
    std::string out = wkt != nullptr ? wkt : "";
    utm3d.reset();
    utm.reset();
    proj_context_destroy(ctx);
    return out;
}

TEST(CopcBuildCrs, GeocentricToUtmWithEllipsoidalHeightsIsThreeDimensional) {
    const std::string dir = TempDir("ecef");
    // Points around 46.68 N 13.84 E, 580-720 m above the ellipsoid, in earth-centred metres.
    SynthLas s = TaggedCloud(200000, 11, {1e-7, 1e-7, 0.01}, {13.84, 46.68, 650}, {70000, 70000, 7000});
    std::vector<Xyz> lonLatH = PointsOf(s);
    std::vector<Xyz> ecef = lonLatH;
    auto toEcef = crs::TransformCache::Global().Get(crs::Crs::FromEpsg(4979), crs::Crs::FromEpsg(4978));
    for (Xyz& p : ecef) ASSERT_TRUE(toEcef->TransformPoint(p.x, p.y, &p.z));
    SetPoints(s, ecef);
    s.wkt = Wkt1("EPSG:4978");
    const std::string in = dir + "/ecef.laz", out = dir + "/utm.copc.laz";
    ASSERT_TRUE(WriteBytes(in, BuildLasBytes(s)));

    const std::string target = Utm33With3dWkt();
    ASSERT_FALSE(target.empty());
    CopcBuildReport rep;
    const Status st = BuildCopc(in, out, Options(target), &rep);
    ASSERT_TRUE(st.ok()) << st.ToString();
    EXPECT_TRUE(rep.transformed);
    EXPECT_EQ(rep.transformPath, "exact");
    EXPECT_FALSE(rep.boundsPass);
    EXPECT_FALSE(rep.restarted);
    EXPECT_EQ(rep.points, s.count);
    // WKT1 has no 3-D projected CRS: WKT2.
    EXPECT_EQ(rep.crs.rfind("PROJCRS[", 0), 0u) << rep.crs;
    EXPECT_TRUE(crs::Crs::FromWkt(rep.crs).Equivalent(crs::Crs::FromUserInput(target)));
    EXPECT_LE(rep.maxNodePoints, uint64_t{kLeafMax} * 16);
    ExpectConsistentHistogram(rep);
    ExpectRoundTrip(out, {{crs::Crs::FromWkt(s.wkt), PointsOf(s)}}, {0}, true);
    // Z is the height above the ellipsoid (both quantisations: within 2 mm).
    double worst = 0;
    for (const TaggedPoint& t : ReadCopc(out))
        worst = std::max(worst, std::abs(t.p.z - lonLatH[static_cast<size_t>(t.tag)].z));
    EXPECT_LT(worst, 0.002);
    EXPECT_GT(rep.depthHistogram.size(), 3u);
    // A host reads the COPC back: its summary states the 3-D CRS, in WKT2 as WKT1 cannot
    char* info = nullptr;
    ASSERT_EQ(ddbtPointCloudInfo(out.c_str(), true, &info), DDBT_ERR_NONE) << ddbtLastError();
    const std::string summary(info);
    ddbtFree(info);
    const auto parsed = json::Parse(summary);
    ASSERT_TRUE(parsed && parsed->IsObject()) << summary;
    const std::string wkt = parsed->obj.at("wkt").str;
    EXPECT_EQ(wkt.rfind("PROJCRS[", 0), 0u) << wkt;
    EXPECT_TRUE(crs::Crs::FromWkt(wkt).Equivalent(crs::Crs::FromUserInput(target)));
    fs::remove_all(dir);
}

// A cloud whose CRS does not fit its coordinates (a header that says EPSG:4978 over a site grid of a
// few hundred metres): the host asks for the CRS to be dropped. The points are those of the build
// that keeps it, record for record, and the COPC states no CRS, so no reader places it on the earth.
TEST(CopcBuildCrs, ADroppedCrsKeepsEveryPointAndWritesNone) {
    const std::string dir = TempDir("drop");
    SynthLas s = TaggedCloud(20000, 13, {0.001, 0.001, 0.001}, {62, 47, 14}, {150000, 142000, 26000});
    s.wkt = Wkt1("EPSG:4978");
    const std::string in = dir + "/site.laz", kept = dir + "/kept.copc.laz", dropped = dir + "/dropped.copc.laz";
    ASSERT_TRUE(WriteBytes(in, BuildLasBytes(s)));

    CopcBuildReport keptReport, droppedReport;
    ASSERT_TRUE(BuildCopc(in, kept, Options(""), &keptReport).ok());
    CopcBuildOptions drop = Options("");
    drop.dropCrs = true;
    const Status st = BuildCopc(in, dropped, drop, &droppedReport);
    ASSERT_TRUE(st.ok()) << st.ToString();

    EXPECT_FALSE(keptReport.crsDropped);
    EXPECT_TRUE(droppedReport.crsDropped);
    EXPECT_FALSE(droppedReport.transformed);
    EXPECT_EQ(droppedReport.sourceCrs, s.wkt);
    EXPECT_EQ(droppedReport.crs, "");
    EXPECT_EQ(droppedReport.scale, keptReport.scale);
    EXPECT_EQ(droppedReport.offset, keptReport.offset);
    EXPECT_EQ(droppedReport.points, s.count);

    // The file states no CRS
    auto file = CopcFile::Open(dropped);
    ASSERT_TRUE(file.ok()) << file.status().ToString();
    EXPECT_FALSE((*file)->metadata().wkt.has_value() && !(*file)->metadata().wkt->empty());
    char* info = nullptr;
    ASSERT_EQ(ddbtPointCloudInfo(dropped.c_str(), false, &info), DDBT_ERR_NONE) << ddbtLastError();
    const auto summary = json::Parse(std::string(info));
    ddbtFree(info);
    ASSERT_TRUE(summary && summary->IsObject());
    EXPECT_EQ(summary->obj.at("wkt").str, "");

    // The same points, at the same quantisation, as the COPC that keeps the CRS
    auto byTag = [](std::vector<TaggedPoint> points) {
        std::sort(points.begin(), points.end(), [](const TaggedPoint& a, const TaggedPoint& b) { return a.tag < b.tag; });
        return points;
    };
    const auto a = byTag(ReadCopc(kept));
    const auto b = byTag(ReadCopc(dropped));
    ASSERT_EQ(a.size(), s.count);
    ASSERT_EQ(b.size(), a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        ASSERT_EQ(a[i].tag, b[i].tag);
        ASSERT_EQ(a[i].p.x, b[i].p.x) << i;
        ASSERT_EQ(a[i].p.y, b[i].p.y) << i;
        ASSERT_EQ(a[i].p.z, b[i].p.z) << i;
    }

    // The sidecar has no CRS and no footprint
    const std::vector<uint8_t> meta = ReadBytes(dropped + ".meta.json");
    const auto j = json::Parse(std::string(meta.begin(), meta.end()));
    ASSERT_TRUE(j && j->IsObject());
    EXPECT_EQ(j->Find("polyBounds"), nullptr);
    EXPECT_EQ(j->Find("centroid"), nullptr);
    const json::Value* projection = j->Find("projection");
    EXPECT_TRUE(projection == nullptr || projection->str.empty());

    // A target and a dropped CRS ask for two different things
    CopcBuildOptions both = Options("EPSG:32633");
    both.dropCrs = true;
    const std::string none = dir + "/none.copc.laz";
    EXPECT_EQ(BuildCopc(in, none, both, nullptr).code(), StatusCode::kInvalidArgument);
    EXPECT_FALSE(fs::exists(none));
    fs::remove_all(dir);
}

TEST(CopcBuildCrs, CompoundKeepsItsVerticalCrs) {
    const std::string dir = TempDir("compound");
    // NAD83(2011) + NAVD88 height, as a Florida survey exports it, to NAD83(2011) / UTM 17N + NAVD88.
    SynthLas s = TaggedCloud(200000, 13, {1e-7, 1e-7, 0.01}, {-82.59, 27.53, 10}, {50000, 50000, 2000});
    s.wkt = Wkt1("EPSG:6318+5703");
    const std::string in = dir + "/geo.laz", out = dir + "/utm.copc.laz";
    ASSERT_TRUE(WriteBytes(in, BuildLasBytes(s)));
    CopcBuildReport rep;
    const Status st = BuildCopc(in, out, Options("EPSG:6346+5703"), &rep);
    ASSERT_TRUE(st.ok()) << st.ToString();
    EXPECT_EQ(rep.transformPath, "grid");
    EXPECT_EQ(rep.crs.rfind("COMPD_CS[", 0), 0u) << rep.crs;
    EXPECT_TRUE(crs::Crs::FromWkt(rep.crs).Equivalent(crs::Crs::FromUserInput("EPSG:6346+5703")));
    ExpectRoundTrip(out, {{crs::Crs::FromWkt(s.wkt), PointsOf(s)}}, {0}, true);
    // The same vertical CRS: heights as they were.
    const std::vector<Xyz> src = PointsOf(s);
    double worst = 0;
    for (const TaggedPoint& t : ReadCopc(out)) worst = std::max(worst, std::abs(t.p.z - src[static_cast<size_t>(t.tag)].z));
    EXPECT_LT(worst, 1e-6);
    fs::remove_all(dir);
}

// Heights in US survey feet to a vertical CRS in metres (NAVD88 height (ftUS) to NAVD88 height):
// the heights change, so every point goes through PROJ in three dimensions.
TEST(CopcBuildCrs, ACompoundToAnotherVerticalUnitIsThreeDimensional) {
    const std::string dir = TempDir("ftus");
    SynthLas s = TaggedCloud(100000, 43, {1e-7, 1e-7, 0.01}, {-82.59, 27.53, 30}, {50000, 50000, 2000});
    s.wkt = Wkt1("EPSG:6318+6360");
    const std::string in = dir + "/geo.laz", out = dir + "/utm.copc.laz";
    ASSERT_TRUE(WriteBytes(in, BuildLasBytes(s)));
    CopcBuildReport rep;
    const Status st = BuildCopc(in, out, Options("EPSG:6346+5703"), &rep);
    ASSERT_TRUE(st.ok()) << st.ToString();
    EXPECT_EQ(rep.transformPath, "exact");
    EXPECT_FALSE(rep.boundsPass);
    // Every point once, where PROJ puts it horizontally, its height in metres (both quantisations:
    // within 2 mm)
    constexpr double kUsFoot = 1200.0 / 3937.0;
    const std::vector<Xyz> src = PointsOf(s);
    auto horizontal = crs::TransformCache::Global().Get(crs::Crs::FromEpsg(6318), crs::Crs::FromEpsg(6346));
    const std::vector<TaggedPoint> points = ReadCopc(out);
    ASSERT_EQ(points.size(), src.size());
    std::vector<uint8_t> seen(src.size(), 0);
    double worstXy = 0, worstZ = 0;
    for (const TaggedPoint& t : points) {
        const size_t i = static_cast<size_t>(t.tag);
        ASSERT_LT(i, src.size());
        ASSERT_EQ(seen[i]++, 0) << "a point written twice";
        double x = src[i].x, y = src[i].y;
        ASSERT_TRUE(horizontal->TransformPoint(x, y));
        worstXy = std::max({worstXy, std::abs(t.p.x - x), std::abs(t.p.y - y)});
        worstZ = std::max(worstZ, std::abs(t.p.z - src[i].z * kUsFoot));
    }
    EXPECT_LT(worstXy, 0.001);
    EXPECT_LT(worstZ, 0.002);
    fs::remove_all(dir);
}

// The transform of the points: a grid approximation within a tenth of a millimetre of PROJ where
// the heights are kept, PROJ in 3-D where they change.
TEST(CopcBuildCrs, PointTransformApproximatesTwoDimensionsAndRunsThreeExactly) {
    const std::array<double, 4> box{13.833, 46.673, 13.847, 46.687};
    const crs::Crs geo = crs::Crs::FromEpsg(4326), utm = crs::Crs::FromEpsg(32633);
    auto t = PointTransform::Create(geo, utm, box, 1e-4);
    ASSERT_TRUE(t.ok()) << t.status().ToString();
    EXPECT_TRUE((*t)->Approximated());
    EXPECT_FALSE((*t)->ThreeD());
    std::vector<double> xyz;
    for (int i = 0; i < 1000; ++i) {
        // Inside the box, and a few points outside it (exact).
        xyz.push_back(13.83 + 0.02 * ((i * 37) % 1000) / 1000.0);
        xyz.push_back(46.67 + 0.02 * ((i * 91) % 1000) / 1000.0);
        xyz.push_back(600 + i);
    }
    std::vector<double> out(xyz.size());
    ASSERT_TRUE((*t)->Apply(xyz.data(), 1000, out.data()).ok());
    auto exact = crs::TransformCache::Global().Get(geo, utm);
    double worst = 0;
    for (size_t i = 0; i < 1000; ++i) {
        double x = xyz[3 * i], y = xyz[3 * i + 1];
        ASSERT_TRUE(exact->TransformPoint(x, y));
        worst = std::max({worst, std::abs(x - out[3 * i]), std::abs(y - out[3 * i + 1])});
        EXPECT_EQ(out[3 * i + 2], xyz[3 * i + 2]);
    }
    EXPECT_LT(worst, 1e-4);

    auto same = PointTransform::Create(crs::Crs::FromUserInput("EPSG:4326+5773"), crs::Crs::FromUserInput("EPSG:32633+5773"), box, 1e-4);
    ASSERT_TRUE(same.ok());
    EXPECT_FALSE((*same)->ThreeD());
    EXPECT_TRUE((*same)->Approximated());
    auto ecef = PointTransform::Create(crs::Crs::FromEpsg(4978), utm, std::nullopt, 1e-4);
    ASSERT_TRUE(ecef.ok());
    EXPECT_TRUE((*ecef)->ThreeD());
    EXPECT_FALSE((*ecef)->Approximated());
    auto ellipsoidal = PointTransform::Create(crs::Crs::FromEpsg(4979), utm, box, 1e-4);
    ASSERT_TRUE(ellipsoidal.ok());
    EXPECT_TRUE((*ellipsoidal)->ThreeD());
    EXPECT_EQ((*t)->GetPath(), PointTransform::Path::Grid);
    EXPECT_EQ((*same)->GetPath(), PointTransform::Path::Grid);
    EXPECT_EQ((*ecef)->GetPath(), PointTransform::Path::Exact);
    // The bounds of the box transformed: those of a dense lattice of the box, to within the bend
    // of a side between two samples (well under a millimetre over 1.5 km).
    const auto bounds = (*t)->BoxBounds({box[0], box[1], 580}, {box[2], box[3], 720});
    ASSERT_TRUE(bounds.has_value());
    std::array<double, 3> lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
    for (int i = 0; i <= 200; ++i)
        for (int j = 0; j <= 200; ++j) {
            double x = box[0] + (box[2] - box[0]) * i / 200, y = box[1] + (box[3] - box[1]) * j / 200;
            ASSERT_TRUE(exact->TransformPoint(x, y));
            lo = {std::min(lo[0], x), std::min(lo[1], y), 580};
            hi = {std::max(hi[0], x), std::max(hi[1], y), 720};
        }
    for (size_t a = 0; a < 3; ++a) {
        EXPECT_NEAR(bounds->first[a], lo[a], 1e-3) << a;
        EXPECT_NEAR(bounds->second[a], hi[a], 1e-3) << a;
    }
    EXPECT_FALSE((*t)->BoxBounds({13.84, 46.68, 0}, {13.85, 95.0, 1}).has_value());
    // A point PROJ cannot transform is an error, not a point dropped.
    const double far[3] = {13.84, 95.0, 0};
    double farOut[3];
    EXPECT_EQ((*t)->Apply(far, 1, farOut).code(), StatusCode::kInvalidArgument);
}

// A geographic CRS on coordinates it cannot hold (UTM metres labelled EPSG:4326): no footprint,
// and the reason; the same coordinates in degrees have one.
TEST(CopcBuildCrs, ASidecarHasNoFootprintForCoordinatesItsCrsCannotHold) {
    SidecarInput in;
    in.points = 10;
    in.wkt = Wkt1("EPSG:4326");
    in.min = {410393, 5169914, 582};
    in.max = {411433, 5171161, 729};
    auto j = json::Parse(SidecarJson(in));
    ASSERT_TRUE(j && j->IsObject());
    EXPECT_EQ(j->Find("polyBounds"), nullptr);
    ASSERT_NE(j->Find("footprintWarning"), nullptr);
    EXPECT_EQ(j->Find("footprintWarning")->str, "the coordinates do not fit WGS 84");
    in.min = {13.83, 46.67, 582};
    in.max = {13.85, 46.69, 729};
    j = json::Parse(SidecarJson(in));
    ASSERT_TRUE(j && j->IsObject());
    EXPECT_NE(j->Find("polyBounds"), nullptr);
    EXPECT_EQ(j->Find("footprintWarning"), nullptr);
    // Only a geographic CRS bounds its coordinates.
    EXPECT_EQ(crs::BoundsMisfit(crs::Crs::FromEpsg(32633), 410393, 5169914, 411433, 5171161), "");
    EXPECT_EQ(crs::BoundsMisfit(crs::Crs::FromUserInput("EPSG:4326+5773"), -180, -90, 180, 90), "");
    EXPECT_NE(crs::BoundsMisfit(crs::Crs::FromUserInput("EPSG:4326+5773"), -181, 0, 0, 0), "");
}

TEST(CopcBuildCrs, AnUnknownVerticalCrsKeepsItsHeights) {
    const std::string dir = TempDir("unknown-vertical");
    // As Metashape states longitude / latitude with heights: WGS 84 + a vertical CRS it cannot name.
    SynthLas s = GeographicCloud(13.84, 46.68, 100000, 41);
    s.wkt = "COMPD_CS[\"WGS 84\"," + Wkt1("EPSG:4326") +
            ",VERT_CS[\"unknown\",VERT_DATUM[\"unknown\",2005],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Up\",UP]]]";
    const std::string in = dir + "/geo.laz", out = dir + "/utm.copc.laz";
    ASSERT_TRUE(WriteBytes(in, BuildLasBytes(s)));
    CopcBuildReport rep;
    const Status st = BuildCopc(in, out, Options("EPSG:32633"), &rep);
    ASSERT_TRUE(st.ok()) << st.ToString();
    EXPECT_EQ(rep.transformPath, "grid");
    EXPECT_EQ(rep.sourceCrs, s.wkt);
    EXPECT_TRUE(crs::Crs::FromWkt(rep.crs).Equivalent(crs::Crs::FromEpsg(32633)));
    EXPECT_FALSE(HasWarning(rep, "degenerate octree"));
    ExpectRoundTrip(out, {{crs::Crs::FromEpsg(4326), PointsOf(s)}}, {0}, false);
    const std::vector<Xyz> src = PointsOf(s);
    double worst = 0;
    for (const TaggedPoint& t : ReadCopc(out)) worst = std::max(worst, std::abs(t.p.z - src[static_cast<size_t>(t.tag)].z));
    EXPECT_LT(worst, 1e-6);
    fs::remove_all(dir);
}

// The vertical CRS Metashape cannot name, kept by a host in the target it asks for (UTM + the
// source's vertical part, in WKT1 as the source states it and in WKT2 as GDAL writes a compound
// CRS it has no code for): the heights do not change, so X / Y go through the grid.
TEST(CopcBuildCrs, ATargetWithTheSourcesUnknownVerticalCrsKeepsHeightsOnTheGrid) {
    const std::string dir = TempDir("unknown-vertical-kept");
    const std::string vertical =
        "VERT_CS[\"unknown\",VERT_DATUM[\"unknown\",2005],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Up\",UP]]";
    SynthLas s = GeographicCloud(13.84, 46.68, 100000, 47);
    s.wkt = "COMPD_CS[\"WGS 84\"," + Wkt1("EPSG:4326") + "," + vertical + "]";
    const std::string in = dir + "/geo.laz";
    ASSERT_TRUE(WriteBytes(in, BuildLasBytes(s)));
    const std::string wkt1 = "COMPD_CS[\"WGS 84 / UTM zone 33N + unknown\"," + Wkt1("EPSG:32633") + "," + vertical + "]";
    const std::string wkt2 = crs::Crs::FromWkt(wkt1).ToWkt(crs::WktFlavor::Wkt2_2019);
    ASSERT_EQ(wkt2.rfind("COMPOUNDCRS[", 0), 0u) << wkt2;
    const std::vector<Xyz> src = PointsOf(s);
    for (const std::string& target : {wkt1, wkt2}) {
        const std::string out = dir + "/utm.copc.laz";
        CopcBuildReport rep;
        const Status st = BuildCopc(in, out, Options(target), &rep);
        ASSERT_TRUE(st.ok()) << st.ToString();
        EXPECT_EQ(rep.transformPath, "grid") << target;
        EXPECT_FALSE(rep.boundsPass);
        EXPECT_TRUE(crs::Crs::FromWkt(rep.crs).Equivalent(crs::Crs::FromWkt(wkt1))) << rep.crs;
        EXPECT_FALSE(HasWarning(rep, "degenerate octree"));
        ExpectRoundTrip(out, {{crs::Crs::FromEpsg(4326), PointsOf(s)}}, {0}, false);
        double worst = 0;
        for (const TaggedPoint& t : ReadCopc(out)) worst = std::max(worst, std::abs(t.p.z - src[static_cast<size_t>(t.tag)].z));
        EXPECT_LT(worst, 1e-6);
        fs::remove_all(out);
    }
    fs::remove_all(dir);
}

/// Builds `s` to UTM 33N: every point once, within a millimetre of where it was, a valid COPC
/// whose cube holds every point.
CopcBuildReport ExpectUtmBuildOf(const SynthLas& s, const std::string& name) {
    const std::string dir = TempDir(name);
    const std::string in = dir + "/geo.laz", out = dir + "/utm.copc.laz";
    EXPECT_TRUE(WriteBytes(in, BuildLasBytes(s)));
    CopcBuildReport rep;
    const Status st = BuildCopc(in, out, Options("EPSG:32633"), &rep);
    EXPECT_TRUE(st.ok()) << st.ToString();
    if (st.ok()) {
        EXPECT_EQ(rep.points, s.count);
        auto chk = CheckCopc(out, 2);
        EXPECT_TRUE(chk.ok());
        if (chk.ok()) {
            EXPECT_TRUE(chk->ok()) << (chk->errors.empty() ? "" : chk->errors[0]);
            EXPECT_EQ(chk->outside, 0u);
        }
        ExpectRoundTrip(out, {{crs::Crs::FromWkt(s.wkt), PointsOf(s)}}, {0}, false);
    }
    fs::remove_all(dir);
    return rep;
}

// A header box smaller than the cloud (the middle half of it): the cube planned from it misses
// points, so the build plans again from the bounds of the points it read.
TEST(CopcBuildCrs, AHeaderBoxThatMissesPointsIsPlannedAgain) {
    SynthLas s = GeographicCloud(13.84, 46.68, 100000, 53);
    s.overrideBounds = true;
    s.min = {13.84 - 0.0035, 46.68 - 0.0035, 650 - 36};
    s.max = {13.84 + 0.0035, 46.68 + 0.0035, 650 + 36};
    const CopcBuildReport rep = ExpectUtmBuildOf(s, "small-header");
    EXPECT_TRUE(rep.restarted);
    EXPECT_FALSE(rep.boundsPass);
}

// A header box far from the cloud (around 0, 0): the transformed points do not fit a quantisation
// around its centre, so the build plans from a bounds pass over them.
TEST(CopcBuildCrs, AHeaderBoxFarFromThePointsIsPlannedFromABoundsPass) {
    SynthLas s = GeographicCloud(13.84, 46.68, 50000, 59);
    s.overrideBounds = true;
    s.min = {0, 0, 0};
    s.max = {0.001, 0.001, 1};
    const CopcBuildReport rep = ExpectUtmBuildOf(s, "far-header");
    EXPECT_TRUE(rep.restarted);
    EXPECT_TRUE(rep.boundsPass);
}

// No header bounds (minimum over maximum): a bounds pass plans the build, and with no box to grid
// over every point goes through PROJ.
TEST(CopcBuildCrs, AnInputWithoutHeaderBoundsIsPlannedFromABoundsPass) {
    SynthLas s = GeographicCloud(13.84, 46.68, 50000, 61);
    s.overrideBounds = true;
    s.min = {1, 1, 1};
    s.max = {0, 0, 0};
    const CopcBuildReport rep = ExpectUtmBuildOf(s, "no-header");
    EXPECT_TRUE(rep.boundsPass);
    EXPECT_FALSE(rep.restarted);
    EXPECT_EQ(rep.transformPath, "exact");
}

TEST(CopcBuildCrs, InputsInDifferentCrssNeedATarget) {
    const std::string dir = TempDir("mixed");
    // The same area in longitude / latitude and in UTM 33N.
    const SynthLas geo = GeographicCloud(13.84, 46.68, 60000, 17);
    SynthLas utm = TaggedCloud(50000, 19, {0.001, 0.001, 0.01}, {411400, 5170500, 650}, {500000, 500000, 7000}, 1e6);
    utm.wkt = Wkt1("EPSG:32633");
    ASSERT_TRUE(WriteBytes(dir + "/geo.laz", BuildLasBytes(geo)));
    ASSERT_TRUE(WriteBytes(dir + "/utm.laz", BuildLasBytes(utm)));

    CopcBuildOptions o = Options("");
    o.extraInputs = {dir + "/utm.laz"};
    const Status refused = BuildCopc(dir + "/geo.laz", dir + "/mixed.copc.laz", o, nullptr);
    EXPECT_EQ(refused.code(), StatusCode::kInvalidArgument);
    EXPECT_NE(std::string(refused.message()).find("different coordinate systems"), std::string::npos) << refused.ToString();
    EXPECT_FALSE(fs::exists(dir + "/mixed.copc.laz"));

    o.targetCrs = "EPSG:32633";
    CopcBuildReport rep;
    const Status st = BuildCopc(dir + "/geo.laz", dir + "/mixed.copc.laz", o, &rep);
    ASSERT_TRUE(st.ok()) << st.ToString();
    EXPECT_EQ(rep.points, geo.count + utm.count);
    ExpectRoundTrip(dir + "/mixed.copc.laz",
                    {{crs::Crs::FromWkt(geo.wkt), PointsOf(geo)}, {crs::Crs::FromWkt(utm.wkt), PointsOf(utm)}}, {0, 1e6},
                    false);
    fs::remove_all(dir);
}

TEST(CopcBuildCrs, InputsInOneCrsWrittenTwoWaysMerge) {
    const std::string dir = TempDir("same");
    SynthLas a = GeographicCloud(13.84, 46.68, 20000, 23);
    SynthLas b = GeographicCloud(13.84, 46.68, 20000, 29);
    b.wkt = crs::Crs::FromEpsg(4326).ToWkt(crs::WktFlavor::Wkt2_2019);
    ASSERT_TRUE(WriteBytes(dir + "/a.laz", BuildLasBytes(a)));
    ASSERT_TRUE(WriteBytes(dir + "/b.laz", BuildLasBytes(b)));
    CopcBuildOptions o = Options("");
    o.extraInputs = {dir + "/b.laz"};
    CopcBuildReport rep;
    const Status st = BuildCopc(dir + "/a.laz", dir + "/ab.copc.laz", o, &rep);
    ASSERT_TRUE(st.ok()) << st.ToString();
    EXPECT_EQ(rep.points, 40000u);
    EXPECT_EQ(rep.crs, a.wkt);
    fs::remove_all(dir);
}

TEST(CopcBuildCrs, ATargetNeedsAValidCrsAndAnInputCrs) {
    const std::string dir = TempDir("errors");
    SynthLas s = GeographicCloud(13.84, 46.68, 5000, 31);
    ASSERT_TRUE(WriteBytes(dir + "/geo.laz", BuildLasBytes(s)));
    for (const char* bad : {"EPSG:999999", "not a crs"}) {
        const Status st = BuildCopc(dir + "/geo.laz", dir + "/out.copc.laz", Options(bad), nullptr);
        EXPECT_EQ(st.code(), StatusCode::kInvalidArgument) << bad << ": " << st.ToString();
    }
    s.wkt.clear();
    ASSERT_TRUE(WriteBytes(dir + "/nocrs.laz", BuildLasBytes(s)));
    const Status st = BuildCopc(dir + "/nocrs.laz", dir + "/out.copc.laz", Options("EPSG:32633"), nullptr);
    EXPECT_EQ(st.code(), StatusCode::kInvalidArgument) << st.ToString();
    EXPECT_FALSE(fs::exists(dir + "/out.copc.laz"));
    fs::remove_all(dir);
}

TEST(CopcBuildCrs, AWktTooLongForItsRecordIsAnError) {
    const std::string dir = TempDir("longwkt");
    const SynthLas s = GeographicCloud(13.84, 46.68, 5000, 37);
    ASSERT_TRUE(WriteBytes(dir + "/geo.laz", BuildLasBytes(s)));
    // UTM 33N under a name of 70,000 characters: its WKT does not fit the 65,535 bytes of a VLR.
    // Without its EPSG code, which would make it the official definition (and its name).
    std::string wkt = Wkt1("EPSG:32633");
    const std::string code = ",AUTHORITY[\"EPSG\",\"32633\"]]";
    ASSERT_EQ(wkt.rfind(code), wkt.size() - code.size());
    wkt.replace(wkt.size() - code.size(), code.size(), "]");
    const size_t name = wkt.find('"') + 1;
    wkt.replace(name, wkt.find('"', name) - name, std::string(70000, 'n'));
    const Status st = BuildCopc(dir + "/geo.laz", dir + "/out.copc.laz", Options(wkt), nullptr);
    EXPECT_EQ(st.code(), StatusCode::kInvalidArgument) << st.ToString();
    EXPECT_FALSE(fs::exists(dir + "/out.copc.laz"));
    fs::remove_all(dir);
}

}  // namespace
}  // namespace ddbt::copc::build
