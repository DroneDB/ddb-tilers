// SPDX-License-Identifier: MPL-2.0
//
// CopcFile and NodeDecoder: validation (non-COPC, truncated and wrong-format files), raw and
// compact node decoding against the records that were encoded (synthetic) and against the
// upstream decode (corpus), the per-cloud 16-bit colour decision, concurrent decoding, and
// compact-node memory.

#include <gtest/gtest.h>

#include <atomic>
#include <filesystem>
#include <iostream>
#include <thread>
#include <vector>

#include "copc/copc/copc_file.h"
#include "copc/copc/node_decoder.h"
#include "copc/las/point_format.h"
#include "copc_test_util.h"
#include "core/exec/parallel_for.h"
#include "core/io/byte_source.h"

namespace ddbt::copc {
namespace {

using testing::BuildCopcBytes;
using testing::Get;
using testing::SynthCopc;
using testing::SynthNode;

SynthCopc SmallCloud(int format, int eb, bool color16, uint32_t seed = 1) {
    SynthCopc s;
    s.format = format;
    s.extraBytes = eb;
    s.center[0] = s.center[1] = s.center[2] = 0;
    s.halfSize = 200;
    s.spacing = 3;
    s.wkt = crs::Crs::FromEpsg(32633).ToWkt(crs::WktFlavor::Wkt1Gdal);
    const NodeKey keys[] = {{0, 0, 0, 0}, {1, 0, 0, 0}, {1, 1, 0, 0}, {1, 1, 1, 1}, {2, 3, 3, 2}, {3, 6, 7, 5}};
    for (const NodeKey& k : keys) {
        SynthNode n;
        n.key = k;
        if (k.d == 2) {
            n.count = 0;  // a structural node
        } else {
            testing::SynthOptions o;
            o.format = format;
            o.extraBytes = eb;
            o.seed = seed++;
            o.color16 = color16 && k.d == 3;  // 16-bit values only deep: the decision ignores depth 3
            o.base[0] = o.base[1] = o.base[2] = 0;
            o.spread = k.d == 0 ? 19000 : 9000;
            n.count = 500 + 100 * static_cast<uint32_t>(k.d);
            n.records = testing::SynthRecords(n.count, o);
        }
        s.nodes.push_back(std::move(n));
    }
    return s;
}

TEST(CopcFile, OpensSyntheticFilesOfEachFormat) {
    for (int format : {6, 7, 8}) {
        const SynthCopc spec = SmallCloud(format, format == 8 ? 9 : 0, false);
        auto file = CopcFile::Open(std::make_shared<io::MemorySource>(BuildCopcBytes(spec), "synthetic"));
        ASSERT_TRUE(file.ok()) << file.status().ToString();
        const CopcFile& f = **file;
        EXPECT_EQ(f.pointFormat(), format);
        EXPECT_EQ(f.extraBytes(), format == 8 ? 9 : 0);
        EXPECT_EQ(f.hasRgb(), format >= 7);
        EXPECT_EQ(f.hasNir(), format == 8);
        EXPECT_DOUBLE_EQ(f.spacing(), 3);
        EXPECT_DOUBLE_EQ(f.RootCube().minX, -200);
        EXPECT_DOUBLE_EQ(f.RootCube().maxZ, 200);
        auto crs = f.Crs();
        ASSERT_TRUE(crs.ok()) << crs.status().ToString();
        EXPECT_EQ(crs->Epsg().value_or(0), 32633);
        auto all = f.hierarchy().All();
        ASSERT_TRUE(all.ok());
        EXPECT_EQ(all->nodes.size(), 5u);
        EXPECT_EQ(all->points, f.header().pointCount);
    }
    auto a = CopcFile::Open(std::make_shared<io::MemorySource>(BuildCopcBytes(SmallCloud(7, 0, false)), "a"));
    auto b = CopcFile::Open(std::make_shared<io::MemorySource>(BuildCopcBytes(SmallCloud(7, 0, false)), "b"));
    ASSERT_TRUE(a.ok() && b.ok());
    EXPECT_NE((*a)->id(), (*b)->id());
}

TEST(CopcFile, RejectsWhatReadersCopcRejects) {
    const std::vector<uint8_t> good = BuildCopcBytes(SmallCloud(7, 0, false));
    auto open = [](std::vector<uint8_t> b) {
        return CopcFile::Open(std::make_shared<io::MemorySource>(std::move(b), "bad"));
    };
    {
        auto b = good;
        const char user[] = "notcopc";
        for (size_t i = 0; i < sizeof(user); ++i) b.at(375 + 2 + i) = static_cast<uint8_t>(user[i]);
        EXPECT_EQ(open(b).code(), StatusCode::kUnsupported) << "no COPC info VLR";
    }
    {
        auto b = good;
        b[104] = 3 | 0x80;
        testing::Put<uint16_t>(b, 105, 34);
        EXPECT_EQ(open(b).code(), StatusCode::kUnsupported) << "format 3";
    }
    {
        auto b = good;
        testing::Put<double>(b, 375 + 54 + 32, 0.0);
        EXPECT_EQ(open(b).code(), StatusCode::kCorrupt) << "spacing 0";
    }
    {
        auto b = good;
        b.resize(b.size() / 2);
        EXPECT_FALSE(open(b).ok()) << "truncated";
    }
    {
        auto b = good;
        testing::Put<uint16_t>(b, 375 + 54 + 160 + 54, 2);  // LAZ compressor 2
        EXPECT_EQ(open(b).code(), StatusCode::kUnsupported);
    }
    const char* plain = "/mnt/data/repos/test_data/point-clouds/point-cloud-complex.laz";
    if (std::filesystem::exists(plain)) {
        EXPECT_EQ(CopcFile::Open(plain).code(), StatusCode::kUnsupported);
    }
    EXPECT_EQ(CopcFile::Open("/nonexistent/file.copc.laz").code(), StatusCode::kNotFound);
}

TEST(CopcFile, RawAndCompactNodesEqualTheEncodedRecords) {
    for (int format : {6, 7, 8}) {
        for (bool color16 : {false, true}) {
            const SynthCopc spec = SmallCloud(format, 3, color16, 10 + static_cast<uint32_t>(format));
            auto file = CopcFile::Open(std::make_shared<io::MemorySource>(BuildCopcBytes(spec), "synthetic"));
            ASSERT_TRUE(file.ok());
            NodeDecoder dec(*file);
            auto sixteen = (*file)->Uses16BitColor();
            ASSERT_TRUE(sixteen.ok());
            EXPECT_FALSE(*sixteen) << "16-bit values only at depth 3: the decision reads depths 0-2";
            const las::PointLayout l = las::LayoutOf(format);
            const size_t rs = l.baseSize + 3u;
            for (const SynthNode& sn : spec.nodes) {
                if (sn.count == 0) continue;
                auto sel = (*file)->hierarchy().Select({Box3::Xy(-1e9, -1e9, 1e9, 1e9), 0});
                const HierarchyNode* node = nullptr;
                for (const HierarchyNode* n : sel->nodes)
                    if (n->key == sn.key) node = n;
                ASSERT_NE(node, nullptr);
                RawPoints raw;
                ASSERT_TRUE(dec.DecodeRaw(*node, laz::kLayersXyzRgb | laz::kLayerNir, raw).ok());
                ASSERT_EQ(raw.size(), sn.count);
                EXPECT_EQ(raw.r.empty(), format == 6);
                EXPECT_EQ(raw.nir.empty(), format != 8);
                auto compact = dec.DecodeCompact(*node);
                ASSERT_TRUE(compact.ok()) << compact.status().ToString();
                ASSERT_EQ(compact->size(), sn.count);
                EXPECT_EQ(compact->hasColor(), format != 6);
                for (size_t i = 0; i < sn.count; ++i) {
                    const uint8_t* p = sn.records.data() + i * rs;
                    ASSERT_EQ(raw.x[i], Get<int32_t>(p));
                    ASSERT_EQ(raw.y[i], Get<int32_t>(p + 4));
                    ASSERT_EQ(raw.z[i], Get<int32_t>(p + 8));
                    for (int a = 0; a < 3; ++a) ASSERT_EQ(compact->Coord(a, i), Get<int32_t>(p + 4 * a));
                    if (format != 6) {
                        uint16_t c[3];
                        std::memcpy(c, p + l.rgb, 6);
                        ASSERT_EQ(raw.r[i], c[0]);
                        ASSERT_EQ(raw.g[i], c[1]);
                        ASSERT_EQ(raw.b[i], c[2]);
                        uint32_t expect = 0xFF000000u;
                        for (int k = 0; k < 3; ++k) expect |= std::min<uint32_t>(c[k], 255u) << (8 * k);
                        ASSERT_EQ(compact->rgba()[i], expect);
                    }
                    if (format == 8) {
                        ASSERT_EQ(raw.nir[i], Get<uint16_t>(p + l.nir));
                    }
                }
                // Masked decode leaves the other arrays empty.
                ASSERT_TRUE(dec.DecodeRaw(*node, laz::kLayerXY, raw).ok());
                EXPECT_TRUE(raw.z.empty());
                EXPECT_TRUE(raw.r.empty());
            }
        }
    }
}

TEST(CopcFile, SixteenBitDecisionAndWideAxes) {
    // 16-bit colours in the root node: the cloud is 16-bit and colours are reduced with >> 8.
    SynthCopc spec = SmallCloud(7, 0, false);
    testing::SynthOptions o;
    o.format = 7;
    o.color16 = true;
    o.seed = 99;
    o.base[0] = o.base[1] = o.base[2] = 0;
    o.spread = 19900;  // the root node spans 39800 units: still 16-bit offsets
    spec.nodes[0].records = testing::SynthRecords(spec.nodes[0].count, o);
    auto file = CopcFile::Open(std::make_shared<io::MemorySource>(BuildCopcBytes(spec), "16"));
    ASSERT_TRUE(file.ok());
    auto sixteen = (*file)->Uses16BitColor();
    ASSERT_TRUE(sixteen.ok());
    EXPECT_TRUE(*sixteen);
    NodeDecoder dec(*file);
    auto all = (*file)->hierarchy().All();
    const HierarchyNode* root = all->nodes.front();
    ASSERT_EQ(root->key.d, 0);
    auto c = dec.DecodeCompact(*root);
    ASSERT_TRUE(c.ok());
    RawPoints raw;
    ASSERT_TRUE(dec.DecodeRaw(*root, laz::kLayersXyzRgb, raw).ok());
    for (size_t i = 0; i < raw.size(); ++i) {
        const uint32_t e = (raw.r[i] >> 8) | ((raw.g[i] >> 8) << 8) | ((raw.b[i] >> 8) << 16) | 0xFF000000u;
        ASSERT_EQ(c->rgba()[i], e);
    }
    EXPECT_FALSE(c->wide(0) || c->wide(1) || c->wide(2));
    EXPECT_EQ(c->Bytes(), sizeof(CompactNode) + mem::AlignUp(raw.size() * 2, 64) * 3 + raw.size() * 4);

    // Wide axes: a span of more than 65535 units stores int32.
    RawPoints wide;
    wide.x = {0, 70000, -5};
    wide.y = {1, 2, 3};
    wide.z = {-2000000000, 2000000000, 0};
    const CompactNode w = CompactNode::FromRaw(wide, false);
    EXPECT_TRUE(w.wide(0));
    EXPECT_FALSE(w.wide(1));
    EXPECT_TRUE(w.wide(2));
    EXPECT_FALSE(w.hasColor());
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_EQ(w.Coord(0, i), wide.x[i]);
        EXPECT_EQ(w.Coord(1, i), wide.y[i]);
        EXPECT_EQ(w.Coord(2, i), wide.z[i]);
    }
    EXPECT_EQ(w.base()[1], 1);
}

// The decision reads whole depths from the root down until they hold 4096 points. Here every
// node has 200 points, so depths 0-2 (73 nodes) are all read, in parallel: wherever the one
// 16-bit node sits in file order (first, middle, last, at depth 0, 1 or 2), the cloud is 16-bit,
// every time; with no 16-bit node it is 8-bit.
TEST(CopcFile, SixteenBitDecisionInParallelIsTheSerialAnswer) {
    auto cloud = [](int sixteenAt, uint32_t rootPoints = 200) {
        SynthCopc s;
        s.format = 7;
        s.center[0] = s.center[1] = s.center[2] = 0;
        s.halfSize = 200;
        s.spacing = 3;
        s.wkt = crs::Crs::FromEpsg(32633).ToWkt(crs::WktFlavor::Wkt1Gdal);
        std::vector<NodeKey> keys{{0, 0, 0, 0}};
        for (int d = 1; d <= 2; ++d)
            for (int x = 0; x < (1 << d); ++x)
                for (int y = 0; y < (1 << d); ++y)
                    for (int z = 0; z < (1 << d); ++z) keys.push_back({d, x, y, z});
        uint32_t seed = 7;
        for (size_t i = 0; i < keys.size(); ++i) {
            SynthNode n;
            n.key = keys[i];
            testing::SynthOptions o;
            o.format = 7;
            o.seed = seed++;
            o.color16 = static_cast<int>(i) == sixteenAt;
            o.base[0] = o.base[1] = o.base[2] = 0;
            o.spread = 4000;
            n.count = i == 0 ? rootPoints : 200;
            n.records = testing::SynthRecords(n.count, o);
            s.nodes.push_back(std::move(n));
        }
        return s;
    };
    for (int at : {-1, 0, 4, 9, 40, 72}) {
        const std::vector<uint8_t> bytes = BuildCopcBytes(cloud(at));
        for (int rep = 0; rep < 20; ++rep) {
            auto file = CopcFile::Open(std::make_shared<io::MemorySource>(bytes, "decision"));
            ASSERT_TRUE(file.ok()) << file.status().ToString();
            ASSERT_EQ((*file)->hierarchy().All()->nodes.size(), 73u);
            auto sixteen = (*file)->Uses16BitColor();
            ASSERT_TRUE(sixteen.ok()) << sixteen.status().ToString();
            EXPECT_EQ(*sixteen, at >= 0) << "16-bit node at " << at;
        }
    }
    // A root of 4096 points or more is the whole sample: 16-bit values only deeper are not seen
    // (the cloud is taken as 8-bit and they are clamped), 16-bit values in the root are.
    for (int at : {0, 4, 40}) {
        auto file = CopcFile::Open(std::make_shared<io::MemorySource>(BuildCopcBytes(cloud(at, 5000)), "root"));
        ASSERT_TRUE(file.ok()) << file.status().ToString();
        auto sixteen = (*file)->Uses16BitColor();
        ASSERT_TRUE(sixteen.ok());
        EXPECT_EQ(*sixteen, at == 0) << "16-bit node at " << at;
    }
}

TEST(CopcFile, ConcurrentDecodesAgree) {
    const SynthCopc spec = SmallCloud(8, 2, true);
    auto file = CopcFile::Open(std::make_shared<io::MemorySource>(BuildCopcBytes(spec), "synthetic"));
    ASSERT_TRUE(file.ok());
    NodeDecoder dec(*file);
    auto all = (*file)->hierarchy().All();
    ASSERT_TRUE(all.ok());
    auto hashAll = [&] {
        testing::MultisetHash h;
        RawPoints raw;
        for (const HierarchyNode* n : all->nodes) {
            if (!dec.DecodeRaw(*n, laz::kLayersXyzRgb, raw).ok()) return testing::MultisetHash{};
            for (size_t i = 0; i < raw.size(); ++i) h.Add(raw.x[i], raw.y[i], raw.z[i], raw.r[i], raw.g[i], raw.b[i]);
        }
        return h;
    };
    const testing::MultisetHash ref = hashAll();
    EXPECT_EQ(ref.count, all->points);
    std::atomic<int> bad{0};
    std::vector<std::thread> threads;
    for (int t = 0; t < 8; ++t)
        threads.emplace_back([&] {
            for (int k = 0; k < 20; ++k)
                if (!(hashAll() == ref)) bad.fetch_add(1);
        });
    for (auto& t : threads) t.join();
    EXPECT_EQ(bad.load(), 0);
}

// Corpus: compact nodes against the upstream decode, the colour decision, and bytes per point.
TEST(CopcFile, CorpusCompactNodes) {
    const auto files = testing::CorpusCopcFiles();
    if (files.empty()) GTEST_SKIP() << "no corpus COPC files";
    for (const std::string& path : files) {
        auto file = CopcFile::Open(path);
        ASSERT_TRUE(file.ok()) << file.status().ToString();
        const CopcFile& f = **file;
        auto sixteen = f.Uses16BitColor();
        ASSERT_TRUE(sixteen.ok());
        auto all = f.hierarchy().All();
        ASSERT_TRUE(all.ok());
        NodeDecoder dec(*file);
        const size_t rs = f.header().pointRecordLength;
        const las::PointLayout l = f.header().Layout();
        std::atomic<uint64_t> bytes{0}, points{0}, wideNodes{0}, failures{0}, over255{0};
        exec::ParallelFor(0, all->nodes.size(), 1, [&](size_t i) {
            const HierarchyNode& n = *all->nodes[i];
            auto c = dec.DecodeCompact(n);
            std::vector<uint8_t> scratch, ref(static_cast<size_t>(n.pointCount) * rs);
            auto chunk = NodeChunk(f, n, scratch);
            if (!c.ok() || !chunk.ok() ||
                !laz::DecodeChunkReference(f.pointFormat(), f.extraBytes(), *chunk, static_cast<uint32_t>(n.pointCount), ref)
                     .ok()) {
                failures.fetch_add(1);
                return;
            }
            uint64_t o255 = 0;
            for (size_t k = 0; k < c->size(); ++k) {
                const uint8_t* p = ref.data() + k * rs;
                for (int a = 0; a < 3; ++a)
                    if (c->Coord(a, k) != Get<int32_t>(p + 4 * a)) {
                        failures.fetch_add(1);
                        return;
                    }
                if (l.HasRgb()) {
                    uint32_t e = 0xFF000000u;
                    for (int ch = 0; ch < 3; ++ch) {
                        const uint16_t v = Get<uint16_t>(p + l.rgb + 2 * ch);
                        o255 += v > 255;
                        e |= (*sixteen ? (v >> 8) : std::min<uint32_t>(v, 255u)) << (8 * ch);
                    }
                    if (c->rgba()[k] != e) {
                        failures.fetch_add(1);
                        return;
                    }
                }
            }
            over255.fetch_add(o255);
            bytes.fetch_add(c->Bytes());
            points.fetch_add(c->size());
            if (c->wide(0) || c->wide(1) || c->wide(2)) wideNodes.fetch_add(1);
        });
        EXPECT_EQ(failures.load(), 0u) << path;
        EXPECT_EQ(points.load(), all->points);
        if (!*sixteen) {
            EXPECT_EQ(over255.load(), 0u) << path << ": 8-bit decision but values above 255 deeper down";
        }
        std::cout << "[ compact ] " << path << ": colour " << (f.hasRgb() ? (*sixteen ? "16-bit" : "8-bit") : "none")
                  << ", " << static_cast<double>(bytes.load()) / static_cast<double>(points.load()) << " B/pt, "
                  << wideNodes.load() << "/" << all->nodes.size() << " nodes with an int32 axis\n";
    }
}

}  // namespace
}  // namespace ddbt::copc
