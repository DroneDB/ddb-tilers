// SPDX-License-Identifier: MPL-2.0
//
// The laz-perf fork's layered decoder against the upstream decode path:
//  - synthetic chunks of formats 6-8 (with and without extra bytes, one and several scanner
//    channels, 1 to 5000 points): compress -> upstream decode is lossless; the layered decoder
//    with every layer gives the same records; with any layer mask (the XYZ / XYZ+RGB(+NIR)
//    masks of the tiler and random ones) every decoded field equals the full decode;
//  - every node of the corpus COPC files: XYZ+RGB(+NIR), XYZ and XY+RGB masks into arrays and
//    full layered records, against upstream's chunk_decompressor;
//  - decoder reuse across chunks and formats, truncated / lying / corrupted chunks.

#include <gtest/gtest.h>

#include <atomic>
#include <cstring>
#include <iostream>
#include <random>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "../copc/copc_test_util.h"
#include "copc/copc/copc_file.h"
#include "copc/copc/node_decoder.h"
#include "copc/las/point_format.h"
#include "copc/laz/chunk_decoder.h"
#include "core/exec/parallel_for.h"

namespace ddbt::copc::laz {
namespace {

using copc::testing::SynthOptions;
using copc::testing::SynthRecords;

// Byte ranges of a format 6-10 record owned by each layer (the flags byte is shared: the
// scanner channel bits belong to XY, the others to the flags layer).
struct FieldBytes {
    uint32_t layer;
    size_t at, size;
    uint8_t mask = 0xFF;
};

std::vector<FieldBytes> Fields(int format, int extraBytes) {
    const las::PointLayout l = las::LayoutOf(format);
    std::vector<FieldBytes> f = {
        {kLayerXY, 0, 8},           {kLayerXY, 14, 1},           {kLayerXY, 15, 1, 0x30},
        {kLayerZ, 8, 4},            {kLayerIntensity, 12, 2},    {kLayerFlags, 15, 1, 0xCF},
        {kLayerClassification, 16, 1}, {kLayerUserData, 17, 1}, {kLayerScanAngle, 18, 2},
        {kLayerPointSource, 20, 2}, {kLayerGpsTime, 22, 8},
    };
    if (l.HasRgb()) f.push_back({kLayerRgb, static_cast<size_t>(l.rgb), 6});
    if (l.HasNir()) f.push_back({kLayerNir, static_cast<size_t>(l.nir), 2});
    if (extraBytes) f.push_back({kLayerExtraBytes, l.baseSize, static_cast<size_t>(extraBytes)});
    return f;
}

// Compares the fields of `layers` between two record arrays; returns the first mismatch.
std::string CompareFields(int format, int extraBytes, uint32_t layers, const uint8_t* a, const uint8_t* b, size_t n) {
    const size_t rs = las::BaseRecordSize(format) + static_cast<size_t>(extraBytes);
    for (const FieldBytes& f : Fields(format, extraBytes)) {
        if (!(f.layer & (layers | kLayerXY))) continue;
        for (size_t i = 0; i < n; ++i)
            for (size_t k = 0; k < f.size; ++k) {
                const uint8_t x = a[i * rs + f.at + k] & f.mask, y = b[i * rs + f.at + k] & f.mask;
                if (x != y)
                    return "point " + std::to_string(i) + " byte " + std::to_string(f.at + k) + " (layer bit " +
                           std::to_string(f.layer) + "): " + std::to_string(x) + " != " + std::to_string(y);
            }
    }
    return {};
}

// SoA output of `layers` against reference records; returns the first mismatch.
std::string CompareSoa(int format, int extraBytes, uint32_t layers, const std::vector<uint8_t>& ref, size_t n,
                       const std::vector<int32_t>& x, const std::vector<int32_t>& y, const std::vector<int32_t>& z,
                       const std::vector<uint16_t>& r, const std::vector<uint16_t>& g, const std::vector<uint16_t>& b,
                       const std::vector<uint16_t>& nir) {
    using copc::testing::Get;
    const las::PointLayout l = las::LayoutOf(format);
    const size_t rs = l.baseSize + static_cast<size_t>(extraBytes);
    for (size_t i = 0; i < n; ++i) {
        const uint8_t* p = ref.data() + i * rs;
        const std::string at = "point " + std::to_string(i) + ": ";
        if (x[i] != Get<int32_t>(p) || y[i] != Get<int32_t>(p + 4)) return at + "x/y";
        if ((layers & kLayerZ) && z[i] != Get<int32_t>(p + 8)) return at + "z";
        if ((layers & kLayerRgb) && l.HasRgb() &&
            (r[i] != Get<uint16_t>(p + l.rgb) || g[i] != Get<uint16_t>(p + l.rgb + 2) ||
             b[i] != Get<uint16_t>(p + l.rgb + 4)))
            return at + "rgb";
        if ((layers & kLayerNir) && l.HasNir() && nir[i] != Get<uint16_t>(p + l.nir)) return at + "nir";
    }
    return {};
}

struct SoaBuffers {
    std::vector<int32_t> x, y, z;
    std::vector<uint16_t> r, g, b, nir;
    explicit SoaBuffers(size_t n) : x(n), y(n), z(n), r(n), g(n), b(n), nir(n) {}
    SoaOut Out(uint32_t layers) {
        SoaOut o;
        o.x = x.data();
        o.y = y.data();
        if (layers & kLayerZ) o.z = z.data();
        if (layers & kLayerRgb) {
            o.r = r.data();
            o.g = g.data();
            o.b = b.data();
        }
        if (layers & kLayerNir) o.nir = nir.data();
        return o;
    }
};

const uint32_t kTilerMasks[] = {kLayersXyzRgb | kLayerNir, kLayersXyzRgb, kLayersXyz, kLayerXY | kLayerRgb, kLayerXY,
                                kLayersAll & ~kLayerGpsTime, kLayersAll & ~(kLayerClassification | kLayerFlags)};

TEST(LayeredDecode, SyntheticChunksEveryMaskMatchesFullDecode) {
    std::mt19937 rng(12345);
    size_t cases = 0;
    for (int format : {6, 7, 8}) {
        for (int eb : {0, 3, 26}) {
            for (bool multi : {false, true}) {
                for (size_t n : {size_t{1}, size_t{2}, size_t{3}, size_t{50}, size_t{5000}}) {
                    SynthOptions o;
                    o.format = format;
                    o.extraBytes = eb;
                    o.multiChannel = multi;
                    o.seed = static_cast<uint32_t>(rng());
                    o.color16 = (rng() & 1) != 0;
                    const std::vector<uint8_t> records = SynthRecords(n, o);
                    const size_t rs = las::BaseRecordSize(format) + static_cast<size_t>(eb);
                    auto chunk = CompressChunk(format, eb, records, static_cast<uint32_t>(n));
                    ASSERT_TRUE(chunk.ok()) << chunk.status().ToString();
                    SCOPED_TRACE("format " + std::to_string(format) + " eb " + std::to_string(eb) + " multi " +
                                 std::to_string(multi) + " n " + std::to_string(n));

                    std::vector<uint8_t> ref(n * rs);
                    ASSERT_TRUE(DecodeChunkReference(format, eb, *chunk, static_cast<uint32_t>(n), ref).ok());
                    ASSERT_EQ(ref, records) << "upstream compress / decompress round trip";

                    std::vector<uint32_t> masks(std::begin(kTilerMasks), std::end(kTilerMasks));
                    masks.push_back(kLayersAll);
                    for (int k = 0; k < 24; ++k) masks.push_back(static_cast<uint32_t>(rng()) & kLayersAll);
                    for (uint32_t m : masks) {
                        SCOPED_TRACE("mask " + std::to_string(m));
                        auto dec = ChunkDecoder::Create(format, eb, m);
                        ASSERT_TRUE(dec.ok());
                        std::vector<uint8_t> rec(n * rs, 0xAB);
                        ASSERT_TRUE((*dec)->DecodeRecords(*chunk, static_cast<uint32_t>(n), rec).ok());
                        if (m == kLayersAll) {
                            ASSERT_EQ(rec, ref);
                        }
                        ASSERT_EQ(CompareFields(format, eb, m, rec.data(), ref.data(), n), "");
                        SoaBuffers s(n);
                        ASSERT_TRUE((*dec)->DecodeSoa(*chunk, static_cast<uint32_t>(n), s.Out(m)).ok());
                        ASSERT_EQ(CompareSoa(format, eb, m, ref, n, s.x, s.y, s.z, s.r, s.g, s.b, s.nir), "");
                        ++cases;
                    }
                }
            }
        }
    }
    std::cout << "[ layered ] " << cases << " synthetic chunk x mask cases\n";
}

TEST(LayeredDecode, ReusedDecoderMatchesFreshDecoders) {
    // One decoder per mask decodes chunks of different contents and sizes, in a shuffled
    // order and twice each: the reset between chunks must leave no state behind.
    for (int format : {6, 7, 8}) {
        std::vector<std::vector<uint8_t>> chunks, refs;
        std::vector<uint32_t> counts;
        for (uint32_t seed = 1; seed <= 6; ++seed) {
            SynthOptions o;
            o.format = format;
            o.extraBytes = 4;
            o.seed = seed;
            o.multiChannel = seed % 2 == 0;
            const uint32_t n = seed * 701;
            auto rec = SynthRecords(n, o);
            auto c = CompressChunk(format, 4, rec, n);
            ASSERT_TRUE(c.ok());
            chunks.push_back(*c);
            refs.push_back(rec);
            counts.push_back(n);
        }
        for (uint32_t m : {kLayersAll, kLayersXyzRgb | kLayerNir, kLayersXyz}) {
            auto dec = ChunkDecoder::Create(format, 4, m);
            ASSERT_TRUE(dec.ok());
            const size_t rs = (*dec)->recordSize();
            for (int pass = 0; pass < 2; ++pass)
                for (size_t k : {3u, 0u, 5u, 1u, 4u, 2u}) {
                    std::vector<uint8_t> out(counts[k] * rs, 0);
                    ASSERT_TRUE((*dec)->DecodeRecords(chunks[k], counts[k], out).ok());
                    ASSERT_EQ(CompareFields(format, 4, m, out.data(), refs[k].data(), counts[k]), "")
                        << "format " << format << " mask " << m << " chunk " << k << " pass " << pass;
                }
            EXPECT_EQ((*dec)->chunksDecoded(), 12u);
        }
    }
}

TEST(LayeredDecode, ThreadDecoderIsPerThreadAndKeyed) {
    auto a = ThreadDecoder(7, 0, kLayersXyzRgb);
    auto b = ThreadDecoder(7, 0, kLayersXyzRgb);
    auto c = ThreadDecoder(7, 0, kLayersXyz);
    auto d = ThreadDecoder(6, 0, kLayersXyzRgb);
    ASSERT_TRUE(a.ok() && b.ok() && c.ok() && d.ok());
    EXPECT_EQ(*a, *b);
    EXPECT_NE(*a, *c);
    EXPECT_NE(*a, *d);
    ChunkDecoder* other = nullptr;
    std::thread t([&] { other = *ThreadDecoder(7, 0, kLayersXyzRgb); });
    t.join();
    EXPECT_NE(other, *a);
    EXPECT_FALSE(ThreadDecoder(3, 0, kLayersAll).ok()) << "the layered path is for formats 6-8";
}

TEST(LayeredDecode, TruncatedLyingAndCorruptChunksFailCleanly) {
    SynthOptions o;
    o.format = 8;
    o.extraBytes = 5;
    const uint32_t n = 300;
    auto rec = SynthRecords(n, o);
    auto chunk = CompressChunk(8, 5, rec, n);
    ASSERT_TRUE(chunk.ok());
    auto dec = ChunkDecoder::Create(8, 5, kLayersAll);
    ASSERT_TRUE(dec.ok());
    SoaBuffers s(n + 10);
    // The header region (first point, count, layer sizes) and the layers: every truncation
    // either decodes (reads past a layer's end give zeros) or reports Corrupt; never reads out
    // of bounds (ASan builds check this).
    const std::vector<uint8_t>& c = *chunk;
    for (size_t len = 0; len < c.size(); len += (len < 200 ? 1 : 97)) {
        std::vector<uint8_t> cut(c.begin(), c.begin() + static_cast<ptrdiff_t>(len));
        Status st = (*dec)->DecodeSoa(cut, n, s.Out(kLayersAll));
        if (len < 38 + 5 + 4) {
            EXPECT_EQ(st.code(), StatusCode::kCorrupt) << len;
        }
    }
    // A count other than the chunk's.
    EXPECT_EQ((*dec)->DecodeSoa(c, n + 1, s.Out(kLayersAll)).code(), StatusCode::kCorrupt);
    EXPECT_EQ((*dec)->DecodeSoa(c, n - 1, s.Out(kLayersAll)).code(), StatusCode::kCorrupt);
    // Still fine afterwards.
    ASSERT_TRUE((*dec)->DecodeSoa(c, n, s.Out(kLayersAll)).ok());
    // Random corruption: any status, no crash.
    std::mt19937 rng(7);
    for (int k = 0; k < 300; ++k) {
        std::vector<uint8_t> bad = c;
        for (int f = 0; f < 4; ++f) bad[rng() % bad.size()] ^= static_cast<uint8_t>(1u << (rng() % 8));
        (*dec)->DecodeSoa(bad, n, s.Out(kLayersAll)).IgnoreError();
    }
    ASSERT_TRUE((*dec)->DecodeSoa(c, n, s.Out(kLayersAll)).ok());
    EXPECT_EQ(CompareSoa(8, 5, kLayersAll, rec, n, s.x, s.y, s.z, s.r, s.g, s.b, s.nir), "");
}

// Every node of every corpus COPC file: masked and full layered decodes against upstream.
TEST(LayeredDecode, CorpusEveryNodeMatchesUpstream) {
    const auto files = copc::testing::CorpusCopcFiles();
    if (files.empty()) GTEST_SKIP() << "no corpus COPC files on this machine";
    for (const std::string& path : files) {
        auto file = CopcFile::Open(path);
        ASSERT_TRUE(file.ok()) << path << ": " << file.status().ToString();
        const CopcFile& f = **file;
        auto all = f.hierarchy().All(NodeOrder::kFileOrder);
        ASSERT_TRUE(all.ok());
        const int format = f.pointFormat(), eb = f.extraBytes();
        const size_t rs = f.header().pointRecordLength;
        ASSERT_EQ(rs, las::BaseRecordSize(format) + static_cast<size_t>(eb));
        std::vector<uint32_t> masks = {kLayersXyzRgb | kLayerNir, kLayersXyz, kLayerXY | kLayerRgb};
        std::atomic<uint64_t> points{0}, nodes{0}, failures{0};
        std::mutex firstMutex;
        std::string first;
        exec::ParallelFor(0, all->nodes.size(), 1, [&](size_t i) {
            const HierarchyNode& node = *all->nodes[i];
            auto fail = [&](const std::string& what) {
                failures.fetch_add(1);
                std::lock_guard<std::mutex> lock(firstMutex);
                if (first.empty())
                    first = std::to_string(node.key.d) + "-" + std::to_string(node.key.x) + "-" +
                            std::to_string(node.key.y) + "-" + std::to_string(node.key.z) + ": " + what;
            };
            std::vector<uint8_t> scratch;
            auto chunk = NodeChunk(f, node, scratch);
            if (!chunk.ok()) return fail(chunk.status().ToString());
            const uint32_t n = static_cast<uint32_t>(node.pointCount);
            std::vector<uint8_t> ref(n * rs);
            if (Status s = DecodeChunkReference(format, eb, *chunk, n, ref); !s.ok()) return fail(s.ToString());
            auto full = ThreadDecoder(format, eb, kLayersAll);
            std::vector<uint8_t> rec(n * rs);
            if (Status s = (*full)->DecodeRecords(*chunk, n, rec); !s.ok()) return fail(s.ToString());
            if (rec != ref) return fail("full layered records differ");
            SoaBuffers soa(n);
            for (uint32_t m : masks) {
                auto dec = ThreadDecoder(format, eb, m);
                if (Status s = (*dec)->DecodeSoa(*chunk, n, soa.Out(m)); !s.ok()) return fail(s.ToString());
                const std::string d = CompareSoa(format, eb, m, ref, n, soa.x, soa.y, soa.z, soa.r, soa.g, soa.b, soa.nir);
                if (!d.empty()) return fail("mask " + std::to_string(m) + ": " + d);
            }
            points.fetch_add(n);
            nodes.fetch_add(1);
        });
        EXPECT_EQ(failures.load(), 0u) << path << ": first failure at node " << first;
        EXPECT_EQ(points.load(), all->points) << path;
        std::cout << "[ layered ] " << path << ": PDRF " << format << " + " << eb << " EB, " << nodes.load()
                  << " nodes, " << points.load() << " points: XYZ+RGB(+NIR), XYZ, XY+RGB and full records equal "
                  << "upstream\n";
    }
}

}  // namespace
}  // namespace ddbt::copc::laz
