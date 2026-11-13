// SPDX-License-Identifier: MPL-2.0
// Synthetic COPC clouds for the tiler tests: random points in a cube, distributed over a
// 3-level octree (each point in the node of its position), written with the COPC test builder.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <vector>


#include "../copc/copc_test_util.h"
#include "../test_env.h"
#include "copc/las/point_format.h"

namespace ddbt::copc::tiler::test {

inline const char* kUtm33Wkt =
    "PROJCS[\"WGS 84 / UTM zone 33N\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563]],"
    "PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433]],PROJECTION[\"Transverse_Mercator\"],"
    "PARAMETER[\"latitude_of_origin\",0],PARAMETER[\"central_meridian\",15],PARAMETER[\"scale_factor\",0.9996],"
    "PARAMETER[\"false_easting\",500000],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1],AUTHORITY[\"EPSG\",\"32633\"]]";

inline const char* kEcefWkt =
    "GEOCCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],"
    "AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"metre\",1,"
    "AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Geocentric X\",OTHER],AXIS[\"Geocentric Y\",OTHER],AXIS[\"Geocentric Z\",NORTH],"
    "AUTHORITY[\"EPSG\",\"4978\"]]";

struct SynthPoint {
    int32_t x = 0, y = 0, z = 0;
    uint16_t r = 0, g = 0, b = 0;
};

struct CloudSpec {
    int format = 7;           // 6 (no colour), 7 (RGB)
    bool sixteenBit = false;  // colours up to 65535
    size_t points = 60000;
    uint32_t seed = 3;
    double center[3] = {500050, 5000050, 50};
    double halfSize = 50;
    double scale = 0.01;
    double offset[3] = {500000, 5000000, 0};
    double zSpan = 40;  // points have z in [center - halfSize, center - halfSize + zSpan]
    int pageDepth = 0;  // > 0: child hierarchy pages from this depth
    std::string wkt = kUtm33Wkt;
    double spacing = 0;  // 0: 2 * halfSize / 128
    // Dark points: all colour components below 256 in the 16-bit case for this fraction.
    double darkFraction = 0;
};

inline std::vector<uint8_t> RecordsOf(int format, const std::vector<SynthPoint>& pts) {
    const las::PointLayout l = las::LayoutOf(format);
    std::vector<uint8_t> v(pts.size() * l.baseSize, 0);
    for (size_t i = 0; i < pts.size(); ++i) {
        uint8_t* p = v.data() + i * l.baseSize;
        std::memcpy(p + l.x, &pts[i].x, 4);
        std::memcpy(p + l.y, &pts[i].y, 4);
        std::memcpy(p + l.z, &pts[i].z, 4);
        p[l.returns] = 0x11;
        if (l.HasRgb()) {
            std::memcpy(p + l.rgb, &pts[i].r, 2);
            std::memcpy(p + l.rgb + 2, &pts[i].g, 2);
            std::memcpy(p + l.rgb + 4, &pts[i].b, 2);
        }
    }
    return v;
}

/// Builds the cloud: 1/16 of the points at the root, 1/4 at depth 1, the rest at depth 2.
inline std::vector<uint8_t> BuildCloud(const CloudSpec& spec) {
    std::mt19937 rng(spec.seed);
    const double lo[3] = {spec.center[0] - spec.halfSize, spec.center[1] - spec.halfSize,
                          spec.center[2] - spec.halfSize};
    std::uniform_real_distribution<double> ux(0.0, 2 * spec.halfSize), uz(0.0, spec.zSpan);
    std::uniform_int_distribution<int> c8(0, 255), c16(0, 65535), u01(0, 999);
    std::vector<SynthPoint> pts(spec.points);
    std::vector<double> pos(spec.points * 3);
    for (size_t i = 0; i < spec.points; ++i) {
        const double p[3] = {lo[0] + ux(rng), lo[1] + ux(rng), lo[2] + uz(rng)};
        for (int a = 0; a < 3; ++a) pos[i * 3 + static_cast<size_t>(a)] = p[a];
        pts[i].x = static_cast<int32_t>(std::llround((p[0] - spec.offset[0]) / spec.scale));
        pts[i].y = static_cast<int32_t>(std::llround((p[1] - spec.offset[1]) / spec.scale));
        pts[i].z = static_cast<int32_t>(std::llround((p[2] - spec.offset[2]) / spec.scale));
        const bool dark = u01(rng) < spec.darkFraction * 1000;
        auto col = [&] { return static_cast<uint16_t>(spec.sixteenBit && !dark ? c16(rng) : c8(rng)); };
        pts[i].r = col();
        pts[i].g = col();
        pts[i].b = col();
    }
    // Octree assignment by position.
    std::map<std::tuple<int, int, int, int>, std::vector<SynthPoint>> nodes;
    for (size_t i = 0; i < spec.points; ++i) {
        const int depth = i % 16 == 0 ? 0 : (i % 4 == 0 ? 1 : 2);
        const int cells = 1 << depth;
        int k[3];
        for (int a = 0; a < 3; ++a) {
            const double t = (pos[i * 3 + static_cast<size_t>(a)] - lo[a]) / (2 * spec.halfSize);
            k[a] = std::clamp(static_cast<int>(t * cells), 0, cells - 1);
        }
        nodes[{depth, k[0], k[1], k[2]}].push_back(pts[i]);
    }
    copc::testing::SynthCopc s;
    s.format = spec.format;
    for (int a = 0; a < 3; ++a) {
        s.scale[a] = spec.scale;
        s.offset[a] = spec.offset[a];
        s.center[a] = spec.center[a];
    }
    s.halfSize = spec.halfSize;
    s.spacing = spec.spacing > 0 ? spec.spacing : 2 * spec.halfSize / 128;
    s.pageDepth = spec.pageDepth;
    s.wkt = spec.wkt;
    // Every ancestor exists (with or without points), parents first.
    std::map<std::tuple<int, int, int, int>, int> keys;
    for (const auto& [k, v] : nodes) {
        auto [d, x, y, z] = k;
        while (true) {
            keys.emplace(std::make_tuple(d, x, y, z), 0);
            if (d == 0) break;
            --d;
            x >>= 1;
            y >>= 1;
            z >>= 1;
        }
    }
    for (const auto& [k, unused] : keys) {
        (void)unused;
        copc::testing::SynthNode n;
        n.key = {std::get<0>(k), std::get<1>(k), std::get<2>(k), std::get<3>(k)};
        const auto it = nodes.find(k);
        if (it != nodes.end()) {
            n.records = RecordsOf(spec.format, it->second);
            n.count = static_cast<uint32_t>(it->second.size());
        }
        s.nodes.push_back(std::move(n));
    }
    return copc::testing::BuildCopcBytes(s);
}

/// A scratch directory under the work tree (not /tmp: tmpfs).
inline std::filesystem::path ScratchDir(const std::string& name) {
    const char* base = std::getenv("DDBT_TEST_TMP");
    std::filesystem::path d = base ? base : "/mnt/data/ddb-tilers-work/tmp";
    if (!std::filesystem::exists(d)) d = std::filesystem::temp_directory_path();
    d /= "copctiler-test-" + ddbt::test::ProcessIdString() + "-" + name;
    std::filesystem::create_directories(d);
    return d;
}

inline std::string WriteCloud(const std::filesystem::path& dir, const std::string& name, const CloudSpec& spec) {
    const auto bytes = BuildCloud(spec);
    const std::filesystem::path p = dir / (name + ".copc.laz");
    std::ofstream f(p, std::ios::binary);
    f.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    return p.string();
}

}  // namespace ddbt::copc::tiler::test
