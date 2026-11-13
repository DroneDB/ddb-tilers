// SPDX-License-Identifier: MPL-2.0
// The COPC tiler on synthetic clouds: compat against an independent re-implementation of
// DroneDB 2.4.1's PointCloudTiler::tile (linear node scan with pow(), CopcNodeReader's node order,
// readers.copc's node order, per-point exact transforms, PointRasterizer); the hybrid mode (the
// default) against compat (the same stamps moved onto the point); the invariants of the hybrid and
// modern modes (determinism across threads, no seams, 512 px = four 256 px tiles); modes, errors
// and blank tiles, and the C API entry.
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <numbers>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <thread>
#include <unordered_set>

#include "api/copc_tile.h"
#include "cog/tiler/tile_error.h"
#include "copc/cache/node_cache.h"
#include "copc/copc/copc_file.h"
#include "copc/copc/node_decoder.h"
#include "copc/laz/chunk_decoder.h"
#include "copc/tiler/cloud.h"
#include "copc/tiler/copc_tiler.h"
#include "copc/tiler/kernels.h"
#include "copc/tiler/rasterizer.h"
#include "core/crs/transform.h"
#include "core/tile/compat.h"
#include "core/util/last_error.h"
#include "synth_cloud.h"

namespace {

using namespace ddbt::copc;
using namespace ddbt::copc::tiler;
using ddbt::copc::tiler::test::CloudSpec;
namespace compat = ddbt::tile::compat;

class CopcTilerTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { dir_ = new std::filesystem::path(test::ScratchDir("tiler")); }
    static void TearDownTestSuite() {
        std::error_code ec;
        std::filesystem::remove_all(*dir_, ec);
        delete dir_;
    }
    static std::string Cloud(const std::string& name, const CloudSpec& spec) {
        const auto p = *dir_ / (name + ".copc.laz");
        if (!std::filesystem::exists(p)) return test::WriteCloud(*dir_, name, spec);
        return p.string();
    }
    static std::filesystem::path* dir_;
};
std::filesystem::path* CopcTilerTest::dir_ = nullptr;

// ------------------------------------------------------------------------------------------------
// The reference: PointCloudTiler::tile written out again from the baseline's sources.

struct RefKey {
    int d, x, y, z;
    bool operator==(const RefKey&) const = default;
};
struct RefKeyHash {
    size_t operator()(const RefKey& k) const noexcept {
        std::hash<uint64_t> h;
        const uint64_t k1 = (static_cast<uint64_t>(k.d) << 32) | static_cast<uint64_t>(k.x);
        const uint64_t k2 = (static_cast<uint64_t>(k.y) << 32) | static_cast<uint64_t>(k.z);
        return h(k1) ^ (h(k2) << 1);
    }
};

struct RefEntry {
    RefKey key;
    uint64_t offset;
    int32_t bytes, count;
};

std::vector<uint8_t> ReferenceTile(const std::string& path, int tz, int tx, int ty, int tileSize, bool tms) {
    auto file = CopcFile::Open(path).value();
    const auto& h = file->header();
    const auto crs = file->Crs().value();
    const auto merc = ddbt::crs::Crs::FromEpsg(3857);
    const auto toCloud = ddbt::crs::Transform::Create(merc, crs);
    const auto toMercator = ddbt::crs::Transform::Create(crs, merc);

    // readCopcIndex: pages from a stack.
    std::vector<RefEntry> nodes;
    std::map<std::tuple<int, int, int, int>, RefEntry> byKey;
    std::vector<std::pair<uint64_t, uint64_t>> pages{{file->info().rootHierOffset, file->info().rootHierSize}};
    while (!pages.empty()) {
        auto [off, size] = pages.back();
        pages.pop_back();
        const auto page = file->source().ReadVector(off, size).value();
        for (size_t e = 0; e < page.size(); e += 32) {
            RefEntry n;
            std::memcpy(&n.key, page.data() + e, 16);
            std::memcpy(&n.offset, page.data() + e + 16, 8);
            std::memcpy(&n.bytes, page.data() + e + 24, 4);
            std::memcpy(&n.count, page.data() + e + 28, 4);
            if (n.count == -1)
                pages.emplace_back(n.offset, static_cast<uint64_t>(n.bytes));
            else {
                byKey[{n.key.d, n.key.x, n.key.y, n.key.z}] = n;
                if (n.count > 0) nodes.push_back(n);
            }
        }
    }

    const compat::GlobalMercator mercator(tileSize);
    const int xyzY = tms ? static_cast<int>(std::pow(2, tz) - 1) - ty : ty;
    const auto tileBounds = mercator.tileBounds(tx, xyzY, tz);
    auto bounds = tileBounds;
    const double tileResolution = mercator.resolution(tz);
    const int boundsBufSize = static_cast<int>(tileResolution * std::ceil(static_cast<double>(static_cast<float>(tileSize)) / 40.0));
    bounds.min.x -= boundsBufSize;
    bounds.max.x += boundsBufSize;
    bounds.min.y -= boundsBufSize;
    bounds.max.y += boundsBufSize;
    EXPECT_TRUE(toCloud->TransformPoint(bounds.min.x, bounds.min.y));
    EXPECT_TRUE(toCloud->TransformPoint(bounds.max.x, bounds.max.y));
    const double minX = bounds.min.x, maxX = bounds.max.x, minY = bounds.min.y, maxY = bounds.max.y;
    const double copcResolution = mercator.resolution(tz - 2);

    const int pointRadius = 2;
    const double pointRadiusMeters = pointRadius * tileResolution;
    const int paddedTileSize = tileSize + pointRadius * 2;
    CompatMapping mapping;
    mapping.originX = tileBounds.min.x;
    mapping.shiftX = -pointRadiusMeters;
    mapping.scaleX = paddedTileSize / (tileBounds.max.x - tileBounds.min.x + pointRadiusMeters * 2.0);
    mapping.originY = tileBounds.min.y;
    mapping.shiftY = pointRadiusMeters;
    mapping.scaleY = paddedTileSize / (tileBounds.max.y - tileBounds.min.y + pointRadiusMeters * 2.0);
    mapping.width = mapping.height = paddedTileSize;
    CompatRasterizer rasterizer;
    rasterizer.Reset(tileSize, mapping, pointRadius, pointRadius);

    const double cx = file->info().centerX, cy = file->info().centerY, hs = file->info().halfSize;
    auto nodeBounds = [&](const RefKey& k, double& nMinX, double& nMinY, double& nMaxX, double& nMaxY) {
        const double rootMinX = cx - hs, rootMaxX = cx + hs, rootMinY = cy - hs, rootMaxY = cy + hs;
        const int width = static_cast<int>(std::pow(2, k.d));
        const double cellWidth = (rootMaxX - rootMinX) / width;
        volatile double p;
        p = cellWidth * k.x;
        nMinX = k.x == 0 ? rootMinX : rootMinX + p;
        p = cellWidth * (k.x + 1);
        nMaxX = k.x == width - 1 ? rootMaxX : rootMinX + p;
        p = cellWidth * k.y;
        nMinY = k.y == 0 ? rootMinY : rootMinY + p;
        p = cellWidth * (k.y + 1);
        nMaxY = k.y == width - 1 ? rootMaxY : rootMinY + p;
    };
    auto unfused = [](int32_t v, double s, double o) {
        volatile double p = v * s;
        return p + o;
    };
    std::vector<double> bx, by, bz;
    std::vector<uint16_t> br, bg, bb;
    auto flush = [&] {
        toMercator->TransformPoints(bx.data(), by.data(), nullptr, bx.size());
        rasterizer.Add(bx.data(), by.data(), bz.data(), br.data(), bg.data(), bb.data(), bx.size());
        bx.clear(), by.clear(), bz.clear(), br.clear(), bg.clear(), bb.clear();
    };
    auto readNode = [&](const RefEntry& n, const std::function<void(double, double, double, uint16_t, uint16_t,
                                                                     uint16_t)>& sink) {
        HierarchyNode hn;
        hn.key = {n.key.d, n.key.x, n.key.y, n.key.z};
        hn.offset = n.offset;
        hn.byteSize = n.bytes;
        hn.pointCount = n.count;
        RawPoints raw;
        EXPECT_TRUE(DecodeNodeRaw(*file, hn, laz::kLayersXyzRgb, raw).ok());
        for (size_t i = 0; i < raw.size(); i++) {
            const double x = unfused(raw.x[i], h.scale[0], h.offset[0]);
            const double y = unfused(raw.y[i], h.scale[1], h.offset[1]);
            const double z = unfused(raw.z[i], h.scale[2], h.offset[2]);
            if (x < minX || x > maxX || y < minY || y > maxY) continue;
            sink(x, y, z, raw.r.empty() ? 0 : raw.r[i], raw.g.empty() ? 0 : raw.g[i], raw.b.empty() ? 0 : raw.b[i]);
        }
    };
    auto push = [&](double x, double y, double z, uint16_t r, uint16_t g, uint16_t b) {
        bx.push_back(x), by.push_back(y), bz.push_back(z), br.push_back(r), bg.push_back(g), bb.push_back(b);
        if (bx.size() == 16384) flush();
    };

    if (file->hasRgb()) {
        // CopcNodeReader::read
        const int depthEnd =
            std::max(1, static_cast<int>(std::ceil(std::log2(file->spacing() / copcResolution))) + 1);
        for (const RefEntry& n : nodes) {
            if (n.key.d >= depthEnd) continue;
            double a, b, c, d;
            nodeBounds(n.key, a, b, c, d);
            if (a <= maxX && c >= minX && b <= maxY && d >= minY) readNode(n, push);
        }
    } else {
        // readers.copc (resolution through its option text) + filters.colorinterp.
        char buf[64];
        std::snprintf(buf, sizeof buf, "%.15g", copcResolution);
        const double resolution = std::strtod(buf, nullptr);
        const int depthEnd = std::max(1, static_cast<int>(std::ceil(std::log2(file->spacing() / resolution))) + 1);
        std::snprintf(buf, sizeof buf, "%.15g", h.min[2]);
        const double rmin = std::strtod(buf, nullptr);
        std::snprintf(buf, sizeof buf, "%.15g", h.max[2]);
        const double rmax = std::strtod(buf, nullptr);
        std::unordered_set<RefKey, RefKeyHash> hierarchy;
        std::function<void(const RefKey&)> load = [&](const RefKey& k) {
            const auto it = byKey.find({k.d, k.x, k.y, k.z});
            if (it == byKey.end()) return;
            if (it->second.count) hierarchy.insert(k);
            for (int i = 0; i < 8; ++i) {
                const RefKey c{k.d + 1, (k.x << 1) | (i & 1), (k.y << 1) | ((i >> 1) & 1), (k.z << 1) | ((i >> 2) & 1)};
                double a, b, cc, d;
                nodeBounds(c, a, b, cc, d);
                if (c.d < depthEnd && a <= maxX && cc >= minX && b <= maxY && d >= minY) load(c);
            }
        };
        load({0, 0, 0, 0});
        // Loads submitted in the set's order to 4 threads, tiles streamed as they complete (the
        // tiler's model of the completion order: 500 us + 0.34 us per point).
        std::vector<std::pair<double, RefKey>> done;
        double freeAt[4] = {};
        for (const RefKey& k : hierarchy) {
            double* w = std::min_element(freeAt, freeAt + 4);
            *w += 500.0 + 0.34 * byKey[{k.d, k.x, k.y, k.z}].count;
            done.emplace_back(*w, k);
        }
        std::stable_sort(done.begin(), done.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
        for (const auto& [t, k] : done) {
            readNode(byKey[{k.d, k.x, k.y, k.z}], [&](double x, double y, double z, uint16_t, uint16_t, uint16_t) {
                const uint32_t c = ddbt::copc::tiler::kernels::RampTable()[kernels::RampIndexScalar(z, rmin, rmax)];
                push(x, y, z, c & 0xFF, (c >> 8) & 0xFF, (c >> 16) & 0xFF);
            });
        }
    }
    if (!bx.empty()) flush();
    std::vector<uint8_t> rgba(static_cast<size_t>(tileSize) * static_cast<size_t>(tileSize) * 4);
    rasterizer.ToRgba(rgba.data());
    return rgba;
}

// The XYZ tiles of zoom z that cover the cloud's footprint (with a ring).
std::vector<std::array<int, 2>> TilesAround(const tiler::Cloud& c, int z, int tileSize, int ring = 1) {
    const compat::GlobalMercator m(tileSize);
    const auto& f = *c.compatFootprint();
    const auto b = compat::MinMaxCoordsForZ(m, f.minX, f.minY, f.maxX, f.maxY, z);
    std::vector<std::array<int, 2>> out;
    for (int y = b.min.y - ring; y <= b.max.y + ring; ++y)
        for (int x = b.min.x - ring; x <= b.max.x + ring; ++x) out.push_back({x, y});
    return out;
}

bool InRange(const tiler::Cloud& c, int z, int x, int tmsRow, int tileSize) {
    const compat::GlobalMercator m(tileSize);
    const auto& f = *c.compatFootprint();
    return compat::MinMaxCoordsForZ(m, f.minX, f.minY, f.maxX, f.maxY, z).Contains(x, tmsRow);
}

size_t Opaque(const std::vector<uint8_t>& rgba) {
    size_t n = 0;
    for (size_t i = 3; i < rgba.size(); i += 4) n += rgba[i] != 0;
    return n;
}

void ExpectCompatEqualsReference(const std::string& path, const std::vector<int>& zooms, int tileSize,
                                 bool expectRawFallback = false) {
    auto cloud = tiler::Cloud::Open(path);
    ASSERT_TRUE(cloud->compatFootprint().has_value()) << cloud->footprintError();
    size_t compared = 0, nonBlank = 0, fallbacks = 0;
    for (const int z : zooms) {
        for (const auto& [x, tmsRow] : TilesAround(*cloud, z, tileSize, 0)) {
            if (!InRange(*cloud, z, x, tmsRow, tileSize)) continue;
            for (const bool tms : {false, true}) {
                const int y = tms ? compat::FlipRow(tmsRow, z) : tmsRow;
                TileRequest r;
                r.path = path;
                r.z = z, r.x = x, r.y = y, r.tileSize = tileSize, r.tms = tms;
                r.mode = Mode::kCompat;
                std::vector<uint8_t> got;
                RenderInfo info;
                RenderRgba(*cloud, r, got, &info);
                const auto expect = ReferenceTile(path, z, x, y, tileSize, tms);
                ASSERT_EQ(got, expect) << path << " z" << z << " x" << x << " y" << y << " tms " << tms;
                ++compared;
                nonBlank += Opaque(got) > 0;
                fallbacks += info.rawFallback;
            }
        }
    }
    EXPECT_GT(compared, 8u);
    EXPECT_GT(nonBlank, compared / 3);
    if (expectRawFallback) {
        EXPECT_GT(fallbacks, 0u);
    }
}

TEST_F(CopcTilerTest, CompatEqualsTheBaselineAlgorithmColored8Bit) {
    CloudSpec s;
    ExpectCompatEqualsReference(Cloud("rgb8", s), {17, 18, 19, 21}, 256);
    ExpectCompatEqualsReference(Cloud("rgb8", s), {18, 20}, 512);
}

TEST_F(CopcTilerTest, CompatEqualsTheBaselineAlgorithmColored16Bit) {
    CloudSpec s;
    s.sixteenBit = true;
    s.darkFraction = 0.995;  // most points below 256: some tiles take the raw fallback
    s.seed = 9;
    ExpectCompatEqualsReference(Cloud("rgb16dark", s), {18, 20, 22}, 256, true);
    CloudSpec t;
    t.sixteenBit = true;
    ExpectCompatEqualsReference(Cloud("rgb16", t), {18, 20}, 256);
}

TEST_F(CopcTilerTest, CompatEqualsTheBaselineAlgorithmColourlessRamp) {
    CloudSpec s;
    s.format = 6;
    s.seed = 4;
    ExpectCompatEqualsReference(Cloud("ramp", s), {17, 18, 20}, 256);
}

TEST_F(CopcTilerTest, CompatEqualsTheBaselineAlgorithmWithChildPages) {
    CloudSpec s;
    s.pageDepth = 1;
    s.seed = 12;
    ExpectCompatEqualsReference(Cloud("paged", s), {18, 19}, 256);
}

// ------------------------------------------------------------------------------------------------
// hybrid and modern modes

// 300 000 points on 50 x 50 m: tiles of z20-z22 are mostly covered.
CloudSpec DenseSpec() {
    CloudSpec s;
    s.points = 300000;
    s.seed = 21;
    s.center[0] = 500025;
    s.center[1] = 5000025;
    s.center[2] = 25;
    s.halfSize = 25;
    s.zSpan = 20;
    return s;
}

std::vector<uint8_t> Render(const std::string& path, int z, int x, int y, int size, Mode mode,
                            RenderInfo* info = nullptr) {
    auto cloud = tiler::Cloud::Get(path);
    TileRequest r;
    r.path = path;
    r.z = z, r.x = x, r.y = y, r.tileSize = size, r.tms = true;
    r.mode = mode;
    std::vector<uint8_t> rgba;
    RenderRgba(*cloud, r, rgba, info);
    return rgba;
}

TEST_F(CopcTilerTest, SplatModesAreDeterministicAcrossThreadsAndCacheStates) {
    CloudSpec s;
    const std::string path = Cloud("rgb8", s);
    auto cloud = tiler::Cloud::Get(path);
    struct T {
        int z, x, y;
    };
    std::vector<T> tiles;
    for (const int z : {17, 18, 19, 20, 21}) {
        for (const auto& [x, tmsRow] : TilesAround(*cloud, z, 256, 0))
            if (InRange(*cloud, z, x, tmsRow, 256)) tiles.push_back({z, x, compat::FlipRow(tmsRow, z)});
    }
    ASSERT_GT(tiles.size(), 10u);
    for (const Mode mode : {Mode::kHybrid, Mode::kModern}) {
        cache::NodeCache::Global().Clear();
        std::vector<std::vector<uint8_t>> serial;
        for (const auto& t : tiles) serial.push_back(Render(path, t.z, t.x, t.y, 256, mode));
        for (const int threads : {1, 8, 16}) {
            cache::NodeCache::Global().Clear();
            std::atomic<size_t> next{0}, bad{0};
            std::vector<std::thread> pool;
            for (int t = 0; t < threads; ++t)
                pool.emplace_back([&] {
                    for (size_t i; (i = next.fetch_add(1)) < tiles.size() * 2;) {
                        const auto& tl = tiles[i % tiles.size()];
                        if (Render(path, tl.z, tl.x, tl.y, 256, mode) != serial[i % tiles.size()]) ++bad;
                    }
                });
            for (auto& th : pool) th.join();
            EXPECT_EQ(bad.load(), 0u) << ModeName(mode) << " " << threads << " threads";
        }
    }
}

TEST_F(CopcTilerTest, SplatModesHaveNoSeamsA512TileIsFour256Tiles) {
    // A 512 px tile at z covers the four 256 px tiles of z + 1 at the same resolution and reads
    // the same octree depth: without seams, the mosaic of the four equals the big tile (up to
    // rare 1 px differences of a point on a pixel edge). The baseline's mosaic misses discs along
    // the tile edges from z21 (its padding is truncated to 0 m).
    const std::string path = Cloud("dense", DenseSpec());
    auto cloud = tiler::Cloud::Get(path);
    for (const Mode mode : {Mode::kHybrid, Mode::kModern, Mode::kCompat}) {
        size_t worstDiff = 0, tilesCompared = 0;
        size_t edgeMissing = 0;
        for (const int z : {20, 21}) {
            for (const auto& [x, tmsRow] : TilesAround(*cloud, z, 512, 0)) {
                if (!InRange(*cloud, z, x, tmsRow, 512)) continue;
                const int y = compat::FlipRow(tmsRow, z);
                const auto big = Render(path, z, x, y, 512, mode);
                if (Opaque(big) < 512 * 512 / 2) continue;  // only tiles well inside the cloud
                std::vector<uint8_t> mosaic(big.size(), 0);
                bool ok = true;
                for (int q = 0; q < 4; ++q) {
                    const int cx = 2 * x + (q & 1), cy = 2 * y + (q >> 1);
                    const int cz = z + 1;
                    if (!InRange(*cloud, cz, cx, compat::FlipRow(cy, cz), 256)) {
                        ok = false;
                        break;
                    }
                    const auto small = Render(path, cz, cx, cy, 256, mode);
                    for (int row = 0; row < 256; ++row)
                        std::memcpy(mosaic.data() + ((static_cast<size_t>(q >> 1) * 256 + static_cast<size_t>(row)) * 512 + static_cast<size_t>(q & 1) * 256) * 4,
                                    small.data() + static_cast<size_t>(row) * 256 * 4, 256 * 4);
                }
                if (!ok) continue;
                ++tilesCompared;
                size_t diff = 0;
                for (size_t i = 0; i < big.size(); i += 4)
                    diff += std::memcmp(big.data() + i, mosaic.data() + i, 4) != 0;
                worstDiff = std::max(worstDiff, diff);
                // Transparent pixels of the mosaic next to the inner seams where the big tile is opaque.
                for (int k = 0; k < 512; ++k)
                    for (const int seam : {255, 256}) {
                        const size_t a = (static_cast<size_t>(k) * 512 + static_cast<size_t>(seam)) * 4 + 3;
                        const size_t b = (static_cast<size_t>(seam) * 512 + static_cast<size_t>(k)) * 4 + 3;
                        edgeMissing += static_cast<size_t>(big[a] != 0 && mosaic[a] == 0) + static_cast<size_t>(big[b] != 0 && mosaic[b] == 0);
                    }
            }
        }
        ASSERT_GT(tilesCompared, 0u);
        if (mode != Mode::kCompat) {
            EXPECT_LE(worstDiff, 512u * 512u / 1000u)
                << ModeName(mode) << ": pixels differing between a 512 tile and its 4 tiles";
            EXPECT_EQ(edgeMissing, 0u) << ModeName(mode) << ": holes along the seams";
        } else {
            EXPECT_GT(edgeMissing, 0u) << "compat keeps the baseline's seams at z21+";
        }
    }
}

// A against B moved (dx east, dy south), 8 px margins: alpha IoU, and the fraction of the pixels
// opaque in both with the same colour.
struct ShiftedMatch {
    size_t inter = 0, uni = 0, sameColour = 0;
    double Iou() const { return uni ? static_cast<double>(inter) / static_cast<double>(uni) : 1.0; }
    double Same() const { return inter ? static_cast<double>(sameColour) / static_cast<double>(inter) : 1.0; }
};
void MatchShifted(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b, int size, int dx, int dy,
                  ShiftedMatch& m) {
    for (int row = 8; row < size - 8; ++row)
        for (int col = 8; col < size - 8; ++col) {
            const uint8_t* pa = &a[(static_cast<size_t>(row) * static_cast<size_t>(size) + static_cast<size_t>(col)) * 4];
            const uint8_t* pb = &b[(static_cast<size_t>(row + dy) * static_cast<size_t>(size) +
                                    static_cast<size_t>(col + dx)) *
                                   4];
            const bool oa = pa[3] != 0, ob = pb[3] != 0;
            m.inter += oa && ob;
            m.uni += oa || ob;
            m.sameColour += oa && ob && std::memcmp(pa, pb, 4) == 0;
        }
}

TEST_F(CopcTilerTest, HybridIsCompatDrawnOnThePoint) {
    // Hybrid keeps the baseline's stamp, depth limit and colours; it anchors the stamp on the
    // pixel corner nearest the point where the baseline anchors it 4 px west and 1 px north. Its
    // coverage is compat's moved by (4, 1) px (up to a point on a rounding edge: the fit, or
    // exact PROJ in another operation order); where stamps overlap, the highest point wins instead
    // of the last drawn.
    CloudSpec rgb16;
    rgb16.sixteenBit = true;
    CloudSpec ramp;
    ramp.format = 6;
    ramp.seed = 4;
    const std::vector<std::pair<std::string, CloudSpec>> clouds{
        {"rgb8", CloudSpec{}}, {"rgb16", rgb16}, {"ramp", ramp}, {"dense", DenseSpec()}};
    for (const auto& [name, spec] : clouds) {
        const std::string path = Cloud(name, spec);
        auto cloud = tiler::Cloud::Get(path);
        ShiftedMatch all;
        for (const int z : {17, 18, 19, 20, 21}) {
            for (const auto& [x, tmsRow] : TilesAround(*cloud, z, 256, 0)) {
                if (!InRange(*cloud, z, x, tmsRow, 256)) continue;
                const int y = compat::FlipRow(tmsRow, z);
                RenderInfo hi, ci;
                const auto h = Render(path, z, x, y, 256, Mode::kHybrid, &hi);
                const auto c = Render(path, z, x, y, 256, Mode::kCompat, &ci);
                EXPECT_EQ(hi.depthEnd, ci.depthEnd) << name << " z" << z;
                MatchShifted(c, h, 256, 4, 1, all);
            }
        }
        std::printf("%s: alpha IoU %.5f, same colour %.4f\n", name.c_str(), all.Iou(), all.Same());
        ASSERT_GT(all.uni, 10000u) << name;
        EXPECT_GE(all.Iou(), 0.999) << name;
    }

    // Sparse clouds (stamps seldom overlap at z21): the same pixels in the same colours, the
    // ramp included (filters.colorinterp's range).
    for (const int format : {6, 7}) {
        CloudSpec sparse;
        sparse.format = format;
        sparse.points = 4000;
        sparse.seed = 31;
        const std::string path = Cloud("sparse" + std::to_string(format), sparse);
        auto cloud = tiler::Cloud::Get(path);
        ShiftedMatch all;
        for (const auto& [x, tmsRow] : TilesAround(*cloud, 21, 256, 0)) {
            if (!InRange(*cloud, 21, x, tmsRow, 256)) continue;
            const int y = compat::FlipRow(tmsRow, 21);
            MatchShifted(Render(path, 21, x, y, 256, Mode::kCompat), Render(path, 21, x, y, 256, Mode::kHybrid), 256,
                         4, 1, all);
        }
        std::printf("sparse, format %d: alpha IoU %.5f, same colour %.4f\n", format, all.Iou(), all.Same());
        ASSERT_GT(all.uni, 10000u) << format;
        EXPECT_GE(all.Iou(), 0.999) << format;
        EXPECT_GE(all.Same(), 0.98) << format;
    }
}

TEST_F(CopcTilerTest, ModernCoversWhatCompatCoversUpToTheWestShift) {
    // The baseline draws an 11 px disc 4 px west of the point; modern draws the 13 px disc on the
    // point, reads at least as deep here (metric CRS at 45 N: projected spacing 1.4 x) and draws
    // every point. Shifted back by 4 px, what compat covers modern mostly covers: the baseline's
    // disc is asymmetric (offsets -2..1, centroid -0.18 px) and placed with round(), so its centre
    // sits 0.32 px east and 0.68 px north of the point on average after the shift, and a row or a
    // column of it pokes out of the symmetric disc for about a tenth of the pixels.
    const std::string path = Cloud("dense", DenseSpec());
    auto cloud = tiler::Cloud::Get(path);
    size_t compatOpaque = 0, both = 0;
    for (const int z : {19, 20, 21}) {
        for (const auto& [x, tmsRow] : TilesAround(*cloud, z, 256, 0)) {
            if (!InRange(*cloud, z, x, tmsRow, 256)) continue;
            const int y = compat::FlipRow(tmsRow, z);
            const auto c = Render(path, z, x, y, 256, Mode::kCompat);
            const auto d = Render(path, z, x, y, 256, Mode::kModern);
            for (int row = 0; row < 256; ++row)
                for (int col = 8; col < 248; ++col) {
                    const bool a = c[(static_cast<size_t>(row) * 256 + static_cast<size_t>(col - 4)) * 4 + 3] != 0;
                    const bool b = d[(static_cast<size_t>(row) * 256 + static_cast<size_t>(col)) * 4 + 3] != 0;
                    compatOpaque += a;
                    both += a && b;
                }
        }
    }
    ASSERT_GT(compatOpaque, 10000u);
    EXPECT_GT(static_cast<double>(both) / static_cast<double>(compatOpaque), 0.85);
}

TEST_F(CopcTilerTest, SplatModesUseAFitAndStayWithinTheResidual) {
    CloudSpec s;
    const std::string path = Cloud("rgb8", s);
    auto cloud = tiler::Cloud::Get(path);
    const auto [x, tmsRow] = TilesAround(*cloud, 18, 256, 0).front();
    for (const Mode mode : {Mode::kHybrid, Mode::kModern}) {
        RenderInfo info;
        Render(path, 18, x, compat::FlipRow(tmsRow, 18), 256, mode, &info);
        EXPECT_TRUE(info.transform == RenderInfo::Transform::kAffine ||
                    info.transform == RenderInfo::Transform::kQuadratic ||
                    info.transform == RenderInfo::Transform::kCubic)
            << ModeName(mode);
        EXPECT_LE(info.fitResidual, 0.01);
        EXPECT_GT(info.opaque, 0u);
    }
}

TEST_F(CopcTilerTest, GeocentricCloudsTileInHybridAndModernModes) {
    // A cloud in EPSG:4978 near (lon 15, lat 45): the baseline has no footprint (heights beyond
    // +-30 km), the hybrid and modern modes tile it.
    CloudSpec s;
    s.wkt = test::kEcefWkt;
    const double lat = 45 * std::numbers::pi / 180, lon = 15 * std::numbers::pi / 180, R = 6378137.0;
    s.center[0] = R * std::cos(lat) * std::cos(lon);
    s.center[1] = R * std::cos(lat) * std::sin(lon);
    s.center[2] = R * std::sin(lat);
    for (int a = 0; a < 3; ++a) s.offset[a] = std::round(s.center[a]);
    s.halfSize = 50;
    s.zSpan = 100;
    const std::string path = Cloud("ecef", s);
    auto cloud = tiler::Cloud::Get(path);
    EXPECT_TRUE(cloud->geocentric());
    EXPECT_FALSE(cloud->compatFootprint().has_value());
    ASSERT_TRUE(cloud->footprint().has_value());
    const compat::GlobalMercator m(256);
    const auto& f = *cloud->footprint();
    const auto range = compat::MinMaxCoordsForZ(m, f.minX, f.minY, f.maxX, f.maxY, 17);
    size_t opaque = 0, hybridOpaque = 0;
    for (int ty = range.min.y; ty <= range.max.y; ++ty)
        for (int tx = range.min.x; tx <= range.max.x; ++tx) {
            TileRequest r;
            r.path = path;
            r.z = 17, r.x = tx, r.y = ty, r.tileSize = 256, r.tms = false, r.mode = Mode::kModern;
            std::vector<uint8_t> rgba;
            RenderRgba(*cloud, r, rgba);
            opaque += Opaque(rgba);
            r.mode = Mode::kHybrid;
            RenderRgba(*cloud, r, rgba);
            hybridOpaque += Opaque(rgba);
            r.mode = Mode::kCompat;
            try {
                RenderRgba(*cloud, r, rgba);
                ADD_FAILURE() << "compat tiled a geocentric cloud";
            } catch (const ddbt::cog::tiler::TileError& e) {
                EXPECT_NE(std::string(e.what()).find("footprint unavailable"), std::string::npos) << e.what();
            }
        }
    EXPECT_GT(opaque, 1000u);
    EXPECT_GT(hybridOpaque, 1000u);
}

// ------------------------------------------------------------------------------------------------
// errors, blank tiles, the C API

TEST_F(CopcTilerTest, ErrorsAreTheBaselines) {
    CloudSpec s;
    const std::string path = Cloud("rgb8", s);
    auto cloud = tiler::Cloud::Get(path);
    const compat::GlobalMercator m(256);
    const auto& f = *cloud->compatFootprint();
    const auto range = compat::MinMaxCoordsForZ(m, f.minX, f.minY, f.maxX, f.maxY, 18);
    TileRequest r;
    r.path = path;
    r.z = 18, r.x = range.max.x + 1, r.y = range.min.y, r.tms = false;
    for (const Mode mode : {Mode::kCompat, Mode::kHybrid, Mode::kModern}) {
        r.mode = mode;
        try {
            RenderTile(r);
            ADD_FAILURE() << "no error";
        } catch (const ddbt::cog::tiler::TileError& e) {
            const std::string expect = "Out of bounds [(" + std::to_string(range.min.x) + "; " +
                                       std::to_string(range.min.y) + ") - (" + std::to_string(range.max.x) + "; " +
                                       std::to_string(range.max.y) + ")]";
            EXPECT_EQ(e.what(), expect);
            EXPECT_EQ(e.code(), ddbt::cog::tiler::TileErrorCode::kException);
        }
    }

    r.path = (*dir_ / "missing.copc.laz").string();
    try {
        RenderTile(r);
        ADD_FAILURE();
    } catch (const ddbt::cog::tiler::TileError& e) {
        EXPECT_EQ(e.code(), ddbt::cog::tiler::TileErrorCode::kNotFound);
        EXPECT_EQ(std::string(e.what()), r.path + " does not exist");
    }

    const auto notCopc = *dir_ / "bad.copc.laz";
    {
        std::ofstream(notCopc) << "not a copc file";
    }
    r.path = notCopc.string();
    try {
        RenderTile(r);
        ADD_FAILURE();
    } catch (const ddbt::cog::tiler::TileError& e) {
        EXPECT_EQ(std::string(e.what()), "Cannot get COPC info for " + r.path);
    }

    CloudSpec nowkt;
    nowkt.wkt.clear();
    r.path = Cloud("nowkt", nowkt);
    try {
        RenderTile(r);
        ADD_FAILURE();
    } catch (const ddbt::cog::tiler::TileError& e) {
        EXPECT_EQ(std::string(e.what()), "COPC file has no WKT SRS: " + r.path);
    }
}

TEST_F(CopcTilerTest, AnyZoomInRangeIsAcceptedAndEmptyTilesAreBlankBytes) {
    CloudSpec s;
    const std::string path = Cloud("rgb8", s);
    auto cloud = tiler::Cloud::Get(path);
    for (const int z : {3, 10, 14, 24}) {
        const auto [x, tmsRow] = TilesAround(*cloud, z, 256, 0).front();
        for (const Mode mode : {Mode::kCompat, Mode::kHybrid, Mode::kModern}) {
            TileRequest r;
            r.path = path;
            r.z = z, r.x = x, r.y = tmsRow, r.tms = false, r.mode = mode, r.noCache = true;
            EXPECT_NO_THROW(RenderTile(r)) << "z" << z;
        }
    }
    // The corner tile of z22: the footprint's range includes tiles the cloud does not reach.
    const compat::GlobalMercator m(256);
    const auto& f = *cloud->compatFootprint();
    const auto range = compat::MinMaxCoordsForZ(m, f.minX, f.minY, f.maxX, f.maxY, 24);
    TileRequest r;
    r.path = path;
    r.z = 24, r.x = range.min.x, r.y = range.max.y, r.tms = false, r.noCache = true;
    for (const Mode mode : {Mode::kHybrid, Mode::kModern}) {
        r.mode = mode;
        std::vector<uint8_t> rgba;
        RenderInfo info;
        RenderRgba(*cloud, r, rgba, &info);
        if (info.blank) {
            const auto bytes = RenderTile(r);
            ddbt::encode::EncodeOptions o;
            const auto& blank = ddbt::encode::BlankTileBytes(256, 256, 3, o);
            EXPECT_EQ(std::vector<uint8_t>(bytes.data(), bytes.data() + bytes.size()), blank);
        }
    }
}

TEST_F(CopcTilerTest, ModesParseAndHybridIsTheDefault) {
    EXPECT_EQ(ParseMode("compat"), Mode::kCompat);
    EXPECT_EQ(ParseMode("hybrid"), Mode::kHybrid);
    EXPECT_EQ(ParseMode("modern"), Mode::kModern);
    EXPECT_EQ(ParseMode("default"), Mode::kHybrid);  // DDBT_TILER_MODE=default (the COG tiler's name)
    EXPECT_FALSE(ParseMode("Hybrid").has_value());
    EXPECT_FALSE(ParseMode("").has_value());
    for (const Mode m : {Mode::kCompat, Mode::kHybrid, Mode::kModern}) EXPECT_EQ(ParseMode(ModeName(m)), m);
    if (std::getenv("DDBT_COPC_TILER_MODE") == nullptr && std::getenv("DDBT_TILER_MODE") == nullptr) {
        EXPECT_EQ(GetMode(), Mode::kHybrid);
    }
    const Mode before = GetMode();
    SetMode(Mode::kModern);
    EXPECT_EQ(GetMode(), Mode::kModern);
    SetMode(before);
}

TEST_F(CopcTilerTest, EncodedTileCacheReturnsTheSameBytes) {
    CloudSpec s;
    const std::string path = Cloud("rgb8", s);
    auto cloud = tiler::Cloud::Get(path);
    const auto [x, tmsRow] = TilesAround(*cloud, 19, 256, 0)[4];
    TileRequest r;
    r.path = path;
    r.z = 19, r.x = x, r.y = tmsRow, r.tms = false, r.format = ddbt::encode::TileFormat::kWebp;
    ClearTileCache();
    const auto before = TileCacheStats();
    const auto a = RenderTile(r);
    const auto b = RenderTile(r);
    r.noCache = true;
    const auto c = RenderTile(r);
    const auto after = TileCacheStats();
    EXPECT_EQ(std::vector<uint8_t>(a.data(), a.data() + a.size()), std::vector<uint8_t>(b.data(), b.data() + b.size()));
    EXPECT_EQ(std::vector<uint8_t>(a.data(), a.data() + a.size()), std::vector<uint8_t>(c.data(), c.data() + c.size()));
    EXPECT_EQ(after.hits - before.hits, 1u);

    // The mode is part of the key: each mode renders its own tile.
    r.noCache = false;
    r.format = ddbt::encode::TileFormat::kPng;
    std::vector<std::vector<uint8_t>> byMode;
    for (const Mode mode : {Mode::kCompat, Mode::kHybrid, Mode::kModern}) {
        r.mode = mode;
        const auto cached = RenderTile(r);
        r.noCache = true;
        const auto fresh = RenderTile(r);
        r.noCache = false;
        byMode.emplace_back(cached.data(), cached.data() + cached.size());
        EXPECT_EQ(byMode.back(), std::vector<uint8_t>(fresh.data(), fresh.data() + fresh.size())) << ModeName(mode);
    }
    EXPECT_NE(byMode[0], byMode[1]);
    EXPECT_NE(byMode[1], byMode[2]);
}

TEST_F(CopcTilerTest, CApiEntry) {
    CloudSpec s;
    const std::string path = Cloud("rgb8", s);
    EXPECT_TRUE(ddbt::api::IsCopcPath(path.c_str()));
    EXPECT_TRUE(ddbt::api::IsCopcPath("/a/B.COPC.LAZ"));
    EXPECT_FALSE(ddbt::api::IsCopcPath("/a/b.laz"));
    auto cloud = tiler::Cloud::Get(path);
    const auto [x, tmsRow] = TilesAround(*cloud, 19, 256, 0)[4];
    uint8_t* buf = nullptr;
    int size = 0;
    ASSERT_EQ(ddbt::api::CopcMemoryTile(path.c_str(), 19, x, compat::FlipRow(tmsRow, 19), &buf, &size, 256, true, false,
                                        nullptr, "png"),
              DDBT_ERR_NONE)
        << ddbt::LastError();
    ASSERT_GT(size, 8);
    EXPECT_EQ(std::memcmp(buf, "\x89PNG\r\n\x1a\n", 8), 0);
    std::free(buf);
    EXPECT_EQ(ddbt::api::CopcMemoryTile(path.c_str(), 19, x, tmsRow, &buf, &size, 300, true, false, nullptr, "png"),
              DDBT_ERR_INVALIDARGS);
    EXPECT_STREQ(ddbt::LastError(), "Tile size must be a power of 2 greater than 0");
    EXPECT_EQ(ddbt::api::CopcMemoryTile(path.c_str(), 19, x, tmsRow, &buf, &size, 256, true, false, nullptr, "gif"),
              DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbt::api::CopcMemoryTile("/nope/x.copc.laz", 19, x, tmsRow, &buf, &size, 256, true, false, nullptr,
                                        "png"),
              DDBT_ERR_NOTFOUND);
    EXPECT_EQ(ddbt::api::CopcMemoryTile(path.c_str(), 19, 0, 0, &buf, &size, 256, true, false, nullptr, "png"),
              DDBT_ERR_EXCEPTION);
    EXPECT_EQ(std::string(ddbt::LastError()).rfind("Out of bounds [(", 0), 0u);
}

}  // namespace
