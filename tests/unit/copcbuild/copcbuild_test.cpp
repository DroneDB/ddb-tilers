// SPDX-License-Identifier: MPL-2.0
//
// COPC builder: Morton keys, the LOD kernel, partition independence, conversions of formats
// 0-10, end-to-end builds of synthetic LAS / LAZ files (every point kept, structure checks,
// byte-identical outputs across threads, partitionings and the spill path, wrong header bounds).
#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <cstdio>
#include <filesystem>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "copc/build/copc_builder.h"
#include "copc/build/copc_check.h"
#include "copc/build/key_mapper.h"
#include "copc/build/lod.h"
#include "copc/build/morton.h"
#include "copc/copc/copc_file.h"
#include "copc/copc/node_decoder.h"
#include "copc/laz/chunk_decoder.h"
#include "copc/readers/convert.h"
#include "copc/readers/las_reader.h"
#include "core/mem/system_memory.h"
#include "core/simd/simd.h"
#include "../test_env.h"
#include "synth_las.h"

namespace ddbt::copc::build {
namespace {

namespace fs = std::filesystem;
using copc::testing::BuildLasBytes;
using copc::testing::FillRandom;
using copc::testing::MultisetHash;
using copc::testing::ReadBytes;
using copc::testing::SynthLas;
using copc::testing::WriteBytes;

std::string TempDir() {
    const fs::path p = fs::temp_directory_path() / ("ddbt-copcbuild-" + ddbt::test::ProcessIdString());
    fs::create_directories(p);
    return p.string();
}

TEST(CopcBuildMorton, EncodersMatch) {
    std::mt19937_64 rng(1);
    for (int i = 0; i < 100000; ++i) {
        const uint32_t x = static_cast<uint32_t>(rng()) & (kKeyCells - 1), y = static_cast<uint32_t>(rng()) & (kKeyCells - 1),
                       z = static_cast<uint32_t>(rng()) & (kKeyCells - 1);
        const uint64_t k = MortonEncodePortable(x, y, z);
        ASSERT_EQ(MortonEncode(x, y, z), k);
        uint32_t a, b, c;
        MortonDecode(k, a, b, c);
        ASSERT_EQ(a, x);
        ASSERT_EQ(b, y);
        ASSERT_EQ(c, z);
        MortonDecodePortable(k, a, b, c);
        ASSERT_EQ(a, x);
        ASSERT_EQ(b, y);
        ASSERT_EQ(c, z);
#if DDBT_MORTON_PDEP
        if (simd::HasAvx2()) {
            ASSERT_EQ(MortonEncodePdep(x, y, z), k);
            MortonDecodePdep(k, a, b, c);
            ASSERT_EQ(a, x);
            ASSERT_EQ(b, y);
            ASSERT_EQ(c, z);
        }
#endif
    }
    // Child direction = the 3 bits below the prefix: x bit 0, y bit 1, z bit 2.
    EXPECT_EQ(MortonEncode(1, 0, 0), 1u);
    EXPECT_EQ(MortonEncode(0, 1, 0), 2u);
    EXPECT_EQ(MortonEncode(0, 0, 1), 4u);
    EXPECT_EQ(PrefixAt(MortonEncode(kKeyCells - 1, 0, 0), 1), 1u);
}

// The per-target loops (portable encoder below AVX2, PDEP on AVX2 and better when the CPU runs it
// fast) give the keys of KeyOfRecord and the same winners on every target the CPU runs.
TEST(CopcBuildKernels, KeysAndWinnersOnEveryTarget) {
    struct Guard {
        ~Guard() { simd::ForceTargetForTest(0); }
    } guard;
    Cube cube;
    cube.center = {1000.0, -2000.0, 50.0};
    cube.halfSize = 600.0;
    const KeyMapper m(cube, {0.01, 0.01, 0.001}, {1000.0, -2000.0, 0.0});
    const size_t rs = 20, n = 50001;
    std::mt19937_64 rng(7);
    std::vector<uint8_t> recs(n * rs);
    for (size_t i = 0; i < n; ++i) {
        // Mostly inside the cube, a few points beyond each face.
        const int32_t v[3] = {static_cast<int32_t>(rng() % 130001) - 65000, static_cast<int32_t>(rng() % 130001) - 65000,
                              static_cast<int32_t>(rng() % 1300001) - 650000};
        std::memcpy(recs.data() + i * rs, v, sizeof v);
    }
    std::vector<uint64_t> want(n);
    bool wantOutside = false;
    for (size_t i = 0; i < n; ++i) want[i] = m.KeyOfRecord(recs.data() + i * rs, wantOutside);
    ASSERT_TRUE(wantOutside);
    bool insideOutside = false;
    std::vector<uint64_t> inside(1);
    m.KeysOfRecords(recs.data(), rs, 0, inside.data(), insideOutside);
    EXPECT_FALSE(insideOutside);

    // The keys of one node of depth 9: its prefix and random low bits, sorted.
    const int depth = 9;
    const uint64_t prefix = uint64_t{0123} << (3 * (kKeyBits - depth));
    std::vector<uint64_t> nodeKeys(20000);
    for (auto& k : nodeKeys) k = prefix | (rng() & ((uint64_t{1} << (3 * (kKeyBits - depth))) - 1));
    std::sort(nodeKeys.begin(), nodeKeys.end());
    std::vector<uint8_t> wantWin;

    std::vector<int64_t> bits = simd::SupportedTargetBits();
    std::reverse(bits.begin(), bits.end());  // the least capable first: the reference winners
    for (const int64_t bit : bits) {
        simd::ForceTargetForTest(bit);
        const std::string target = simd::TargetNameOf(bit) + (simd::HasFastPdep() ? " (PDEP)" : "");
        std::vector<uint64_t> got(n);
        bool outside = false;
        m.KeysOfRecords(recs.data(), rs, n, got.data(), outside);
        EXPECT_EQ(got, want) << target;
        EXPECT_EQ(outside, wantOutside) << target;
        for (const int cells : {3, 64, 128}) {
            std::vector<uint32_t> scratch;
            std::vector<uint8_t> win(nodeKeys.size(), 0);
            SelectWinners(nodeKeys.data(), nodeKeys.size(), depth, cells, win.data(), scratch);
            if (cells == 128) {
                if (wantWin.empty()) wantWin = win;
                EXPECT_EQ(win, wantWin) << target;
            }
            for (uint32_t v : scratch) ASSERT_EQ(v, ~uint32_t{0}) << target;
            EXPECT_GT(std::count(win.begin(), win.end(), uint8_t{1}), 0) << target << " " << cells;
        }
    }
}

TEST(CopcBuildLod, WinnerIsClosestToTheCellCentreThenFirst) {
    std::vector<uint32_t> scratch;
    // Node of depth 13 (8 key bits per axis), 128 cells: 2 key cells per grid cell. A point at
    // local (1,1,1) or (0,0,0) of its cell is as close as any other corner, so the first wins.
    std::vector<uint64_t> keys = {MortonEncode(4, 4, 4), MortonEncode(5, 5, 5), MortonEncode(6, 4, 4)};
    std::sort(keys.begin(), keys.end());
    std::vector<uint8_t> win(keys.size(), 0);
    SelectWinners(keys.data(), keys.size(), 13, 128, win.data(), scratch);
    EXPECT_EQ(win[0], 1);  // (4,4,4) and (5,5,5) share the cell (2,2,2): the first wins
    EXPECT_EQ(win[1], 0);
    EXPECT_EQ(win[2], 1);  // (6,4,4) alone in (3,2,2)
    // Depth 11 (10 bits), 128 cells of 8 key cells: local 3 or 4 is the centre row;
    // (3,4,3) and (4,4,4) are both closest, the smaller key wins.
    keys = {MortonEncode(0, 0, 0), MortonEncode(3, 4, 3), MortonEncode(7, 7, 7), MortonEncode(4, 4, 4)};
    std::sort(keys.begin(), keys.end());
    win.assign(keys.size(), 0);
    SelectWinners(keys.data(), keys.size(), 11, 128, win.data(), scratch);
    int winners = 0;
    for (size_t i = 0; i < keys.size(); ++i) {
        winners += win[i];
        if (win[i]) {
            EXPECT_EQ(keys[i], std::min(MortonEncode(3, 4, 3), MortonEncode(4, 4, 4)));
        }
    }
    EXPECT_EQ(winners, 1);
    // The scratch grid is left empty for the next call.
    for (uint32_t v : scratch) ASSERT_EQ(v, ~uint32_t{0});
}

TEST(CopcBuildLod, WinnerOnAGridThatDoesNotDivideTheKeyCells) {
    // Depth 14 (7 bits: 128 key cells per axis), 3 grid cells per axis: cell 1 holds key cells
    // 43..85, its centre is at 64 key units. Key cells 63 and 64 (centres 63.5, 64.5) are both
    // 0.5 away on x: the smaller key wins; 60 is further.
    std::vector<uint32_t> scratch;
    const uint64_t base = uint64_t{5} << (3 * (kKeyBits - 14));  // a node prefix, not 0
    std::vector<uint64_t> keys = {base | MortonEncode(60, 64, 64), base | MortonEncode(63, 63, 64),
                                  base | MortonEncode(64, 63, 64), base | MortonEncode(42, 0, 0),
                                  base | MortonEncode(43, 0, 0)};
    std::sort(keys.begin(), keys.end());
    std::vector<uint8_t> win(keys.size(), 0);
    SelectWinners(keys.data(), keys.size(), 14, 3, win.data(), scratch);
    std::vector<uint64_t> got;
    for (size_t i = 0; i < keys.size(); ++i)
        if (win[i]) got.push_back(keys[i]);
    std::vector<uint64_t> want = {base | MortonEncode(42, 0, 0), base | MortonEncode(43, 0, 0),
                                  base | std::min(MortonEncode(63, 63, 64), MortonEncode(64, 63, 64))};
    std::sort(want.begin(), want.end());
    EXPECT_EQ(got, want);  // 42 and 43 lie in cells 0 and 1 of x
}

TEST(CopcBuildLod, LeafLevelAndRegionLeaves) {
    // Untwine's level: the first level halves, the others divide by 8.
    EXPECT_EQ(LeafLevel(50000, 100000), 1);
    EXPECT_EQ(LeafLevel(200000, 100000), 1);
    EXPECT_EQ(LeafLevel(200001, 100000), 1);  // integer halving, as Untwine
    EXPECT_EQ(LeafLevel(200002, 100000), 2);
    EXPECT_EQ(LeafLevel(903713, 100000), 2);      // P1
    EXPECT_EQ(LeafLevel(4086063, 100000), 3);     // E5
    EXPECT_EQ(LeafLevel(88777694, 100000), 4);    // P4
    EXPECT_EQ(LeafLevel(669397297, 100000), 5);   // P5
    EXPECT_EQ(LeafLevel(1330712980, 100000), 6);  // P6
    LodParams p;
    p.leafLevel = 3;
    EXPECT_EQ(RegionLeafDepth(100000, p), 3);
    EXPECT_EQ(RegionLeafDepth(100001, p), 4);
    EXPECT_EQ(RegionLeafDepth(400000, p), 4);
    EXPECT_EQ(RegionLeafDepth(400001, p), 5);
    EXPECT_FALSE(IsLeaf(2, 10, -1, p));  // above the leaf level: never
    EXPECT_TRUE(IsLeaf(3, 100000, -1, p));
    EXPECT_FALSE(IsLeaf(3, 100001, -1, p));
    EXPECT_FALSE(IsLeaf(4, 5, 5, p));  // inside a region cut at depth 5
    EXPECT_TRUE(IsLeaf(5, 500000, 5, p));
    p.leafLevel = 13;
    EXPECT_EQ(RegionLeafDepth(uint64_t{1} << 40, p), p.maxDepth);
    EXPECT_TRUE(IsLeaf(p.maxDepth, uint64_t{1} << 40, 13, p));
}

struct Node {
    int depth;
    uint64_t prefix;
    std::vector<uint8_t> recs;
    bool operator==(const Node&) const = default;
};

// Builds a random cloud of `n` points (keys, 16-byte records) either as one partition at the
// root, or cut at depth `cut` into partitions joined by top nodes (a leaf of the LOD above
// `cut` is a partition, as in the builder); returns every emitted node.
std::vector<Node> BuildSynthetic(size_t n, int cut, const LodParams& p, uint32_t seed) {
    std::mt19937_64 rng(seed);
    const size_t rs = 16;
    std::vector<uint8_t> recs(n * rs);
    std::vector<uint64_t> keys(n);
    std::normal_distribution<double> g(0.5, 0.12);
    for (size_t i = 0; i < n; ++i) {
        uint32_t c[3];
        for (auto& v : c) v = static_cast<uint32_t>(std::clamp(g(rng), 0.0, 0.999999) * kKeyCells);
        if (i % 50 == 3 && i > 0) {  // exact duplicates of the previous point
            std::memcpy(recs.data() + i * rs, recs.data() + (i - 1) * rs, rs);
            keys[i] = keys[i - 1];
            continue;
        }
        keys[i] = MortonEncode(c[0], c[1], c[2]);
        for (size_t k = 0; k < rs; ++k) recs[i * rs + k] = static_cast<uint8_t>(rng());
    }
    std::vector<Node> out;
    EmitFn emit = [&](int d, uint64_t prefix, const uint8_t* r, uint32_t count) -> Status {
        out.push_back({d, prefix, std::vector<uint8_t>(r, r + size_t{count} * rs)});
        return Status();
    };
    std::vector<std::pair<uint64_t, size_t>> byKey(n);
    for (size_t i = 0; i < n; ++i) byKey[i] = {keys[i], i};
    std::sort(byKey.begin(), byKey.end());
    auto entriesOf = [&](uint64_t prefix, int depth) {
        const int shift = 3 * (kKeyBits - depth);
        const auto lo = std::lower_bound(byKey.begin(), byKey.end(), std::make_pair(prefix << shift, size_t{0}));
        const auto hi = std::lower_bound(byKey.begin(), byKey.end(), std::make_pair((prefix + 1) << shift, size_t{0}));
        std::vector<SortEntry> e;
        for (auto it = lo; it != hi; ++it) e.push_back({it->first, recs.data() + it->second * rs});
        std::shuffle(e.begin(), e.end(), rng);  // arrival order must not matter
        return e;
    };
    if (cut == 0) {
        auto e = entriesOf(0, 0);
        NodePoints root;
        EXPECT_TRUE(BuildPartition(0, 0, -1, e, rs, p, emit, root).ok());
        EXPECT_TRUE(emit(0, 0, root.records.data(), static_cast<uint32_t>(root.size())).ok());
        return out;
    }
    // Partitions at depth `cut` in Morton order, top nodes above them in post-order.
    std::map<std::pair<int, uint64_t>, NodePoints> pts;
    std::function<void(int, uint64_t, int)> rec = [&](int d, uint64_t prefix, int regionLeaf) {
        auto e = entriesOf(prefix, d);
        if (e.empty()) return;
        if (d == p.leafLevel) regionLeaf = RegionLeafDepth(e.size(), p);
        if (d == cut || IsLeaf(d, e.size(), regionLeaf, p)) {
            NodePoints np;
            EXPECT_TRUE(BuildPartition(d, prefix, d > p.leafLevel ? regionLeaf : -1, e, rs, p, emit, np).ok());
            pts[{d, prefix}] = std::move(np);
            return;
        }
        for (uint64_t c = 0; c < 8; ++c) rec(d + 1, prefix * 8 + c, regionLeaf);
        std::array<NodePoints*, 8> ch{};
        bool any = false;
        for (uint64_t c = 0; c < 8; ++c) {
            auto it = pts.find({d + 1, prefix * 8 + c});
            if (it != pts.end()) {
                ch[c] = &it->second;
                any = true;
            }
        }
        if (!any) return;
        NodePoints np;
        EXPECT_TRUE(BuildTopNode(d, prefix, ch, rs, p, emit, np).ok());
        pts[{d, prefix}] = std::move(np);
    };
    rec(0, 0, -1);
    NodePoints& root = pts[{0, 0}];
    EXPECT_TRUE(emit(0, 0, root.records.data(), static_cast<uint32_t>(root.size())).ok());
    return out;
}

TEST(CopcBuildLod, OutputDoesNotDependOnThePartitioning) {
    // leafMax 300: leaf level 4, regions cut deeper below it, cuts above it; leafMax 3000: leaf
    // level 2 (points / 2 / 8 <= 3000), cuts 3 and 4 inside the regions.
    for (uint32_t leafMax : {300u, 3000u}) {
        LodParams p;
        p.leafMax = leafMax;
        p.hoistMin = 20;
        p.hoistTotal = 300;
        p.gridCells = 20;      // coarse grids, so the nodes thin out at these small counts
        p.rootGridCells = 30;
        p.leafLevel = LeafLevel(40000, leafMax);
        for (uint32_t seed : {1u, 2u, 3u}) {
            const auto whole = BuildSynthetic(40000, 0, p, seed);
            ASSERT_GT(whole.size(), 20u);
            for (int cut : {1, 2, 3, 4}) {
                const auto parts = BuildSynthetic(40000, cut, p, seed);
                ASSERT_EQ(parts.size(), whole.size()) << "cut " << cut;
                for (size_t i = 0; i < whole.size(); ++i)
                    ASSERT_TRUE(parts[i] == whole[i]) << "node " << i << " cut " << cut << " leafMax " << leafMax;
            }
            size_t total = 0, root = 0;
            for (const Node& n : whole) total += n.recs.size() / 16;
            root = whole.back().recs.size() / 16;
            EXPECT_EQ(total, 40000u);
            EXPECT_GT(root, 0u);
            EXPECT_LE(root, 30u * 30u * 30u);
        }
    }
}

TEST(CopcBuildLod, HoistingRules) {
    // A node with 8 children: two of them under hoistMin give all their points; the others give
    // one point per occupied cell. Then all children together under hoistTotal give everything.
    LodParams p;
    p.leafMax = 1000;
    p.leafLevel = 1;
    p.hoistMin = 10;
    p.hoistTotal = 60;
    p.gridCells = 2;  // one cell per child
    p.rootGridCells = 2;
    const size_t rs = 16;
    auto build = [&](const std::vector<size_t>& perChild) {
        std::vector<NodePoints> kids(8);
        std::array<NodePoints*, 8> ch{};
        for (size_t c = 0; c < 8; ++c) {
            if (perChild[c] == 0) continue;
            kids[c].depth = 1;
            kids[c].prefix = c;
            for (size_t i = 0; i < perChild[c]; ++i) {
                kids[c].keys.push_back((uint64_t{c} << (3 * (kKeyBits - 1))) + i);
                for (size_t b = 0; b < rs; ++b) kids[c].records.push_back(static_cast<uint8_t>(c * 16 + i + b));
            }
            ch[c] = &kids[c];
        }
        std::vector<std::pair<uint64_t, uint32_t>> emitted;
        EmitFn emit = [&](int, uint64_t prefix, const uint8_t*, uint32_t count) -> Status {
            emitted.push_back({prefix, count});
            return Status();
        };
        NodePoints out;
        EXPECT_TRUE(BuildTopNode(0, 0, ch, rs, p, emit, out).ok());
        return std::make_pair(out.size(), emitted);
    };
    auto [up, em] = build({5, 20, 0, 3, 20, 20, 20, 20});
    EXPECT_EQ(up, 5u + 3u + 5u);  // two hoisted children + one winner from each of the 5 others
    ASSERT_EQ(em.size(), 5u);      // hoisted children are gone; the others keep 19 points
    for (auto& e : em) EXPECT_EQ(e.second, 19u);
    auto [up2, em2] = build({5, 20, 0, 3, 20, 0, 0, 0});
    EXPECT_EQ(up2, 48u);  // 48 < hoistTotal: everything goes up
    EXPECT_TRUE(em2.empty());
}

TEST(CopcBuildConvert, LegacyFieldsAsPdalReadsThem) {
    uint8_t out[3];
    // return 3 of 5, scan direction 1, edge 0; class 12 (legacy overlap) + synthetic + withheld
    readers::LegacyReturnsAndClass(static_cast<uint8_t>(3 | (5 << 3) | (1 << 6)), static_cast<uint8_t>(12 | (1 << 5) | (1 << 7)), out);
    EXPECT_EQ(out[0], 3 | (5 << 4));
    EXPECT_EQ(out[1], 1 | (1 << 2) | (1 << 3) | (1 << 6));
    EXPECT_EQ(out[2], 0);
    readers::LegacyReturnsAndClass(static_cast<uint8_t>(1 | (1 << 3) | (1 << 7)), static_cast<uint8_t>(2 | (1 << 6)), out);
    EXPECT_EQ(out[0], 1 | (1 << 4));
    EXPECT_EQ(out[1], (1 << 1) | (1 << 7));
    EXPECT_EQ(out[2], 2);
    for (int deg = -90; deg <= 90; ++deg)
        EXPECT_EQ(readers::ScanAngleFromDegrees(static_cast<float>(deg)), static_cast<int16_t>(std::round(static_cast<float>(deg) / .006f)));
    EXPECT_EQ(readers::ScanAngleFromDegrees(1), 167);
    EXPECT_EQ(readers::ScanAngleFromDegrees(-90), -15000);
    EXPECT_EQ(readers::Color8To16(255), 65535);
    EXPECT_EQ(readers::Color8To16(1), 257);
}

// The records the builder must keep: the reader's conversion of every input point.
uint64_t ConvertedInputHash(const std::string& path, int format, uint16_t eb, const std::array<double, 3>& scale,
                            const std::array<double, 3>& offset, uint64_t* count) {
    auto r = readers::LasReader::Open(path);
    EXPECT_TRUE(r.ok()) << r.status().ToString();
    readers::OutputSpec spec;
    spec.format = format;
    spec.extraBytes = eb;
    spec.scale = scale;
    spec.offset = offset;
    struct Sink final : readers::BatchSink {
        const readers::OutputSpec* spec;
        uint64_t sum = 0, n = 0;
        Status Consume(readers::Batch& b) override {
            if (b.xyz) DDBT_TRY(readers::QuantizeXyz(*spec, b.xyz, b.count, b.records));
            sum += MultisetHash(b.records, b.count, spec->RecordSize());
            n += b.count;
            return Status();
        }
    } sink;
    sink.spec = &spec;
    for (size_t t = 0; t < (*r)->TaskCount(); ++t) EXPECT_TRUE((*r)->Read(t, spec, sink).ok());
    *count = sink.n;
    return sink.sum;
}

uint64_t CopcHash(const std::string& path, uint64_t* count) {
    auto file = CopcFile::Open(path);
    EXPECT_TRUE(file.ok()) << file.status().ToString();
    auto all = (*file)->hierarchy().All();
    EXPECT_TRUE(all.ok());
    uint64_t sum = 0, n = 0;
    auto dec = laz::ChunkDecoder::Create((*file)->pointFormat(), (*file)->extraBytes(), laz::kLayersAll);
    EXPECT_TRUE(dec.ok());
    std::vector<uint8_t> scratch, recs;
    for (const HierarchyNode* node : all->nodes) {
        auto chunk = NodeChunk(**file, *node, scratch);
        EXPECT_TRUE(chunk.ok());
        recs.resize(static_cast<size_t>(node->pointCount) * (*dec)->recordSize());
        EXPECT_TRUE((*dec)->DecodeRecords(*chunk, static_cast<uint32_t>(node->pointCount), recs).ok());
        sum += MultisetHash(recs.data(), static_cast<size_t>(node->pointCount), (*dec)->recordSize());
        n += static_cast<uint64_t>(node->pointCount);
    }
    *count = n;
    return sum;
}

struct BuildCase {
    int format;
    uint16_t eb;
    bool laz;
    int minor;
};

TEST(CopcBuildEndToEnd, EveryFormatKeepsEveryPointAndIsDeterministic) {
    const std::string dir = TempDir();
    const std::vector<BuildCase> cases = {{0, 0, false, 2}, {1, 4, true, 2},  {2, 0, true, 2},  {3, 0, true, 2},
                                          {4, 0, false, 3}, {5, 2, false, 3}, {6, 0, true, 4},  {7, 3, true, 4},
                                          {8, 5, true, 4},  {9, 0, false, 4}, {10, 1, false, 4}};
    for (const BuildCase& c : cases) {
        SCOPED_TRACE("format " + std::to_string(c.format));
        SynthLas s;
        s.format = c.format;
        s.extraBytes = c.eb;
        s.laz = c.laz;
        s.minor = c.minor;
        s.chunkTableMinusOne = c.format == 3;
        s.wkt = "PROJCS[\"WGS 84 / UTM zone 15N\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563]],"
                "PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433]],PROJECTION[\"Transverse_Mercator\"],"
                "PARAMETER[\"latitude_of_origin\",0],PARAMETER[\"central_meridian\",-93],PARAMETER[\"scale_factor\",0.9996],"
                "PARAMETER[\"false_easting\",500000],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1],AUTHORITY[\"EPSG\",\"32615\"]]";
        FillRandom(s, 60000 + static_cast<uint64_t>(c.format) * 1000, static_cast<uint32_t>(c.format + 10));
        const std::string in = dir + "/in" + std::to_string(c.format) + (c.laz ? ".laz" : ".las");
        ASSERT_TRUE(WriteBytes(in, BuildLasBytes(s)));

        CopcBuildOptions base;
        base.lod.leafMax = 2000;
        base.lod.hoistMin = 30;
        base.metadata = false;
        std::vector<std::vector<uint8_t>> outputs;
        int variant = 0;
        for (auto tweak : std::vector<std::function<void(CopcBuildOptions&)>>{
                 [](CopcBuildOptions& o) { o.threads = 1; o.maxPartitionPoints = 1u << 30; },
                 [](CopcBuildOptions& o) { o.threads = 4; o.maxPartitionPoints = 3000; },
                 [](CopcBuildOptions& o) { o.threads = 3; o.maxPartitionPoints = 2000; o.forceSpill = true; },
                 [](CopcBuildOptions& o) { o.threads = 8; },
                 // Spilling with half of a tiny budget's share in memory: some bins in memory,
                 // the others spilled, partitions admitted one or two at a time.
                 [](CopcBuildOptions& o) {
                     o.threads = 3;
                     o.forceSpill = true;
                     o.spillMemoryFraction = 0.5;
                     o.memoryBudget = uint64_t{64 + 3 * 12 + 4} << 20;
                     o.maxPartitionPoints = 2500;
                 }}) {
            CopcBuildOptions o = base;
            tweak(o);
            o.tempDir = dir;
            const std::string out = dir + "/out" + std::to_string(c.format) + "_" + std::to_string(variant++) + ".copc.laz";
            CopcBuildReport rep;
            const Status st = BuildCopc(in, out, o, &rep);
            ASSERT_TRUE(st.ok()) << st.ToString();
            EXPECT_EQ(rep.points, s.count);
            EXPECT_EQ(rep.spilled, o.forceSpill);
            if (o.spillMemoryFraction > 0) {
                EXPECT_GT(rep.spillBytes, 0u);
                EXPECT_LT(rep.spillBytes, rep.points * 30u);  // not every record (>= 30 bytes each)
            }
            outputs.push_back(ReadBytes(out));
            if (variant == 2) {
                EXPECT_GT(rep.partitions, 4u);
            }
            if (variant == 1) {
                auto chk = CheckCopc(out, 2);
                ASSERT_TRUE(chk.ok()) << chk.status().ToString();
                for (const auto& e : chk->errors) ADD_FAILURE() << e;
                EXPECT_TRUE(chk->chunkTableMatches);
                EXPECT_EQ(chk->outside, 0u);  // inside its node by PDAL's arithmetic, not only within tolerance
                EXPECT_EQ(chk->decodedPoints, s.count);
                EXPECT_GT(chk->maxDepth, 0);
                const int expectFormat = (c.format == 8 || c.format == 10) ? 8 : (c.format == 2 || c.format == 3 || c.format == 5 || c.format == 7) ? 7 : 6;
                EXPECT_EQ(rep.format, expectFormat);
                EXPECT_EQ(rep.extraBytes, c.eb);
                EXPECT_EQ(rep.scale, s.scale);
                EXPECT_EQ(rep.offset, s.offset);
                uint64_t nIn = 0, nOut = 0;
                const uint64_t hIn = ConvertedInputHash(in, rep.format, c.eb, rep.scale, rep.offset, &nIn);
                const uint64_t hOut = CopcHash(out, &nOut);
                EXPECT_EQ(nIn, s.count);
                EXPECT_EQ(nOut, s.count);
                EXPECT_EQ(hIn, hOut);
            }
            fs::remove(out);
        }
        for (size_t i = 1; i < outputs.size(); ++i) EXPECT_TRUE(outputs[i] == outputs[0]) << "variant " << i << " differs";
        fs::remove(in);
    }
    fs::remove_all(dir);
}

TEST(CopcBuildMemory, DefaultBudgetFromTheEnvironment) {
    ddbt::test::ScopedEnv restore("DDBT_COPC_BUILD_MEM_MB", nullptr);
    auto set = [](const char* v) { ddbt::test::ScopedEnv::Set("DDBT_COPC_BUILD_MEM_MB", v); };
    set("512");
    auto b = DefaultCopcMemoryBudget();
    ASSERT_TRUE(b.ok());
    EXPECT_EQ(*b, uint64_t{512} << 20);
    for (const char* bad : {"abc", "0", "12x", "-5"}) {
        set(bad);
        EXPECT_EQ(DefaultCopcMemoryBudget().status().code(), StatusCode::kInvalidArgument) << bad;
    }
    set(nullptr);
    b = DefaultCopcMemoryBudget();
    ASSERT_TRUE(b.ok());
    const uint64_t phys = mem::PhysicalMemoryBytes();
    EXPECT_EQ(*b, phys ? std::min(uint64_t{4096} << 20, phys / 2) : uint64_t{4096} << 20);
}

TEST(CopcBuildEndToEnd, WrongHeaderBoundsArePlannedAgain) {
    const std::string dir = TempDir();
    SynthLas s;
    s.format = 6;
    s.minor = 4;
    FillRandom(s, 20000, 99);
    s.overrideBounds = true;
    s.min = {s.offset[0] - 10, s.offset[1] - 10, -10};
    s.max = {s.offset[0] + 10, s.offset[1] + 10, 10};
    const std::string in = dir + "/bad.las", out = dir + "/bad.copc.laz";
    ASSERT_TRUE(WriteBytes(in, BuildLasBytes(s)));
    CopcBuildOptions o;
    o.lod.leafMax = 1000;
    o.metadata = false;
    CopcBuildReport rep;
    const Status st = BuildCopc(in, out, o, &rep);
    ASSERT_TRUE(st.ok()) << st.ToString();
    EXPECT_TRUE(rep.restarted);
    auto chk = CheckCopc(out, 2);
    ASSERT_TRUE(chk.ok());
    EXPECT_TRUE(chk->ok()) << (chk->errors.empty() ? "" : chk->errors[0]);
    EXPECT_EQ(chk->outside, 0u);
    fs::remove_all(dir);
}

TEST(CopcBuildEndToEnd, MergedInputsTakeTheFinestScale) {
    const std::string dir = TempDir();
    SynthLas a, b;
    a.format = 1;
    a.scale = {0.01, 0.01, 0.01};
    FillRandom(a, 7000, 21);
    b.format = 7;
    b.minor = 4;
    b.scale = {0.001, 0.001, 0.005};
    b.offset = {500100, 4000100, 5};
    FillRandom(b, 9000, 22, 50000);
    ASSERT_TRUE(WriteBytes(dir + "/a.las", BuildLasBytes(a)));
    ASSERT_TRUE(WriteBytes(dir + "/b.las", BuildLasBytes(b)));
    CopcBuildOptions o;
    o.extraInputs = {dir + "/b.las"};
    o.metadata = false;
    o.lod.leafMax = 1000;
    CopcBuildReport rep;
    const Status st = BuildCopc(dir + "/a.las", dir + "/ab.copc.laz", o, &rep);
    ASSERT_TRUE(st.ok()) << st.ToString();
    EXPECT_EQ(rep.points, 16000u);
    EXPECT_EQ(rep.format, 7);  // one input has colour
    EXPECT_EQ(rep.scale[0], 0.001);
    EXPECT_EQ(rep.scale[2], 0.005);
    auto chk = CheckCopc(dir + "/ab.copc.laz", 2);
    ASSERT_TRUE(chk.ok());
    EXPECT_TRUE(chk->ok()) << (chk->errors.empty() ? "" : chk->errors[0]);
    // Every point of both inputs, requantised.
    uint64_t na = 0, nb = 0, nout = 0;
    const uint64_t ha = ConvertedInputHash(dir + "/a.las", 7, 0, rep.scale, rep.offset, &na);
    const uint64_t hb = ConvertedInputHash(dir + "/b.las", 7, 0, rep.scale, rep.offset, &nb);
    EXPECT_EQ(ha + hb, CopcHash(dir + "/ab.copc.laz", &nout));
    EXPECT_EQ(nout, 16000u);
    fs::remove_all(dir);
}

TEST(CopcBuildEndToEnd, SidecarAndErrors) {
    const std::string dir = TempDir();
    SynthLas s;
    s.format = 3;
    FillRandom(s, 5000, 5, 20000, false);
    s.wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563]],PRIMEM[\"Greenwich\",0],"
            "UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";
    s.scale = {1e-7, 1e-7, 0.01};
    s.offset = {12.0, 45.0, 0};
    const std::string in = dir + "/geo.las", out = dir + "/geo.copc.laz";
    ASSERT_TRUE(WriteBytes(in, BuildLasBytes(s)));
    CopcBuildOptions o;
    CopcBuildReport rep;
    ASSERT_TRUE(BuildCopc(in, out, o, &rep).ok());
    EXPECT_FALSE(rep.color16);
    const auto meta = ReadBytes(out + ".meta.json");
    const std::string text(meta.begin(), meta.end());
    EXPECT_NE(text.find("\"pointCount\": 5000"), std::string::npos) << text;
    EXPECT_NE(text.find("\"color16\": false"), std::string::npos);
    EXPECT_NE(text.find("\"polyBounds\""), std::string::npos) << text;
    EXPECT_NE(text.find("\"Red\""), std::string::npos);
    EXPECT_EQ(BuildCopc(in, in, o, nullptr).code(), StatusCode::kInvalidArgument);
    EXPECT_FALSE(BuildCopc(dir + "/missing.las", out, o, nullptr).ok());
    fs::remove_all(dir);
}

}  // namespace
}  // namespace ddbt::copc::build
