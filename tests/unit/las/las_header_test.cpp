// SPDX-License-Identifier: MPL-2.0
//
// LAS structures: point formats 0-10, headers 1.0-1.4 (synthetic and corpus), VLRs and EVLRs,
// extra-bytes descriptors, LAZ and COPC info VLRs, malformed headers, and the CRS rules
// (WKT / GeoTIFF keys) on synthetic files and the corpus.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "../copc/copc_test_util.h"
#include "copc/las/las_crs.h"
#include "copc/las/las_header.h"
#include "copc/las/point_format.h"
#include "core/io/byte_source.h"

namespace ddbt::copc::las {
namespace {

using copc::testing::Append;
using copc::testing::Put;

TEST(LasPointFormat, LayoutsOfFormats0To10) {
    const uint16_t sizes[] = {20, 28, 26, 34, 57, 63, 30, 36, 38, 59, 67};
    for (int f = 0; f <= 10; ++f) EXPECT_EQ(LayoutOf(f).baseSize, sizes[f]) << f;
    EXPECT_EQ(LayoutOf(1).gpsTime, 20);
    EXPECT_EQ(LayoutOf(2).rgb, 20);
    EXPECT_EQ(LayoutOf(3).rgb, 28);
    EXPECT_EQ(LayoutOf(4).wavePacket, 28);
    EXPECT_EQ(LayoutOf(5).wavePacket, 34);
    EXPECT_FALSE(LayoutOf(0).extended);
    const PointLayout l6 = LayoutOf(6);
    EXPECT_TRUE(l6.extended);
    EXPECT_EQ(l6.flags, 15);
    EXPECT_EQ(l6.classification, 16);
    EXPECT_EQ(l6.scanAngle, 18);
    EXPECT_EQ(l6.pointSourceId, 20);
    EXPECT_EQ(l6.gpsTime, 22);
    EXPECT_FALSE(l6.HasRgb());
    EXPECT_EQ(LayoutOf(7).rgb, 30);
    EXPECT_EQ(LayoutOf(8).nir, 36);
    EXPECT_EQ(LayoutOf(9).wavePacket, 30);
    EXPECT_EQ(LayoutOf(10).rgb, 30);
    EXPECT_EQ(LayoutOf(10).nir, 36);
    EXPECT_EQ(LayoutOf(10).wavePacket, 38);
}

// A minimal header of version 1.minor (all counts 0) followed by nothing.
std::vector<uint8_t> MakeHeader(int minor, int format, uint16_t recordLength) {
    const size_t size = minor >= 4 ? 375 : minor == 3 ? 235 : 227;
    std::vector<uint8_t> h(size, 0);
    std::memcpy(h.data(), "LASF", 4);
    Put<uint16_t>(h, 4, 42);
    Put<uint16_t>(h, 6, minor >= 4 ? 16 : 0);
    h[24] = 1;
    h[25] = static_cast<uint8_t>(minor);
    std::memcpy(h.data() + 26, "SYS ID  ", 8);
    std::memcpy(h.data() + 58, "writer 1.0", 10);
    Put<uint16_t>(h, 90, 300);
    Put<uint16_t>(h, 92, 2024);
    Put<uint16_t>(h, 94, static_cast<uint16_t>(size));
    Put<uint32_t>(h, 96, static_cast<uint32_t>(size));
    h[104] = static_cast<uint8_t>(format);
    Put<uint16_t>(h, 105, recordLength);
    Put<uint32_t>(h, 107, 1234);
    Put<uint32_t>(h, 111, 1000);
    const double scale[3] = {0.01, 0.001, 0.1}, offset[3] = {100, -200, 0};
    for (size_t i = 0; i < 3; ++i) {
        Put<double>(h, 131 + 8 * i, scale[i]);
        Put<double>(h, 155 + 8 * i, offset[i]);
        Put<double>(h, 179 + 16 * i, 10.0 * static_cast<double>(i + 1));
        Put<double>(h, 187 + 16 * i, -10.0 * static_cast<double>(i + 1));
    }
    if (minor >= 4) {
        Put<uint64_t>(h, 247, 5000000000ull);
        Put<uint64_t>(h, 255, 4000000000ull);
    }
    return h;
}

TEST(LasHeader, ParsesVersions10To14) {
    for (int minor = 0; minor <= 4; ++minor) {
        const int format = minor >= 4 ? 7 : 3;
        auto h = ParseHeader(MakeHeader(minor, format, static_cast<uint16_t>(BaseRecordSize(format) + 4)));
        ASSERT_TRUE(h.ok()) << minor << ": " << h.status().ToString();
        EXPECT_EQ(h->versionMinor, minor);
        EXPECT_EQ(h->pointFormat, format);
        EXPECT_FALSE(h->compressed);
        EXPECT_EQ(h->ExtraBytesPerPoint(), 4);
        EXPECT_EQ(h->systemIdentifier, "SYS ID");
        EXPECT_EQ(h->generatingSoftware, "writer 1.0");
        EXPECT_EQ(h->creationYear, 2024);
        EXPECT_EQ(h->fileSourceId, 42);
        EXPECT_DOUBLE_EQ(h->scale[1], 0.001);
        EXPECT_DOUBLE_EQ(h->offset[1], -200);
        EXPECT_DOUBLE_EQ(h->max[2], 30);
        EXPECT_DOUBLE_EQ(h->min[2], -30);
        if (minor >= 4) {
            EXPECT_EQ(h->pointCount, 5000000000ull) << "1.4: the 64-bit count";
            EXPECT_EQ(h->pointsByReturn[0], 4000000000ull);
        } else {
            EXPECT_EQ(h->pointCount, 1234u);
            EXPECT_EQ(h->pointsByReturn[0], 1000u);
        }
    }
}

TEST(LasHeader, CompressionBitsAndErrors) {
    auto laz = MakeHeader(4, 6, 30);
    laz[104] = 6 | 0x80;
    auto h = ParseHeader(laz);
    ASSERT_TRUE(h.ok());
    EXPECT_TRUE(h->compressed);
    EXPECT_EQ(h->pointFormat, 6);
    auto old = MakeHeader(2, 3, 34);
    old[104] = 3 | 0x40;  // old LASzip marker
    auto ho = ParseHeader(old);
    ASSERT_TRUE(ho.ok());
    EXPECT_TRUE(ho->compressed);
    EXPECT_EQ(ho->pointFormat, 3);

    auto bad = MakeHeader(4, 6, 30);
    bad[0] = 'X';
    EXPECT_EQ(ParseHeader(bad).code(), StatusCode::kCorrupt);
    bad = MakeHeader(4, 6, 30);
    bad[24] = 2;
    EXPECT_EQ(ParseHeader(bad).code(), StatusCode::kUnsupported);
    bad = MakeHeader(4, 6, 30);
    bad[25] = 5;
    EXPECT_EQ(ParseHeader(bad).code(), StatusCode::kUnsupported);
    bad = MakeHeader(4, 6, 30);
    Put<uint16_t>(bad, 94, 227);  // too small for 1.4
    EXPECT_EQ(ParseHeader(bad).code(), StatusCode::kCorrupt);
    bad = MakeHeader(4, 11, 80);
    EXPECT_EQ(ParseHeader(bad).code(), StatusCode::kUnsupported);
    bad = MakeHeader(4, 7, 35);  // < 36
    EXPECT_EQ(ParseHeader(bad).code(), StatusCode::kCorrupt);
    bad = MakeHeader(2, 1, 28);
    Put<double>(bad, 139, 0.0);  // y scale 0
    EXPECT_EQ(ParseHeader(bad).code(), StatusCode::kCorrupt);
    bad = MakeHeader(2, 1, 28);
    Put<uint32_t>(bad, 96, 100);  // points inside the header
    EXPECT_EQ(ParseHeader(bad).code(), StatusCode::kCorrupt);
    bad = MakeHeader(4, 6, 30);
    bad.resize(300);
    EXPECT_EQ(ParseHeader(bad).code(), StatusCode::kCorrupt);
    EXPECT_EQ(ParseHeader(std::vector<uint8_t>(10, 0)).code(), StatusCode::kCorrupt);
}

std::vector<uint8_t> Descriptor(uint8_t type, uint8_t options, const char* name, double scale = 1, double offset = 0) {
    std::vector<uint8_t> d(192, 0);
    d[2] = type;
    d[3] = options;
    std::strncpy(reinterpret_cast<char*>(d.data() + 4), name, 32);
    Put<int64_t>(d, 40, -9999);  // no data (element 0)
    Put<double>(d, 112, scale);
    Put<double>(d, 136, offset);
    std::strncpy(reinterpret_cast<char*>(d.data() + 160), "desc", 32);
    return d;
}

TEST(LasExtraBytes, DescriptorsTypesSizesAndOffsets) {
    std::vector<uint8_t> p;
    for (auto& d : {Descriptor(6, 1 | 8, "i32", 0.5), Descriptor(0, 3, "raw3"), Descriptor(10, 0, "f64"),
                    Descriptor(13, 0, "u16x2"), Descriptor(30, 0, "f64x3")})
        p.insert(p.end(), d.begin(), d.end());
    auto eb = ParseExtraBytes(p, 4 + 3 + 8 + 4 + 24);
    ASSERT_TRUE(eb.ok()) << eb.status().ToString();
    ASSERT_EQ(eb->size(), 5u);
    EXPECT_EQ((*eb)[0].name, "i32");
    EXPECT_EQ((*eb)[0].size, 4);
    EXPECT_EQ((*eb)[0].byteOffset, 0);
    EXPECT_TRUE((*eb)[0].HasNoData());
    EXPECT_TRUE((*eb)[0].HasScale());
    EXPECT_DOUBLE_EQ((*eb)[0].scale[0], 0.5);
    EXPECT_DOUBLE_EQ((*eb)[0].AnyAsDouble((*eb)[0].noData), -9999);
    EXPECT_EQ((*eb)[0].description, "desc");
    EXPECT_EQ((*eb)[1].size, 3);
    EXPECT_EQ((*eb)[1].byteOffset, 4);
    EXPECT_EQ((*eb)[2].size, 8);
    EXPECT_EQ((*eb)[2].byteOffset, 7);
    EXPECT_EQ((*eb)[3].size, 4);
    EXPECT_EQ((*eb)[3].elements, 2);
    EXPECT_EQ((*eb)[4].size, 24);
    EXPECT_EQ((*eb)[4].elements, 3);
    EXPECT_EQ((*eb)[4].byteOffset, 19);

    EXPECT_EQ(ParseExtraBytes(p, 42).code(), StatusCode::kCorrupt) << "descriptors beyond the record";
    p.push_back(0);
    EXPECT_EQ(ParseExtraBytes(p, 100).code(), StatusCode::kCorrupt) << "not a multiple of 192";
    EXPECT_EQ(ParseExtraBytes(Descriptor(31, 0, "x"), 100).code(), StatusCode::kCorrupt);
}

// A LAS 1.4 file in memory with VLRs (WKT, GeoKeys, extra bytes, LAZ) and EVLRs.
struct FileBuilder {
    std::vector<uint8_t> f;
    uint32_t vlrs = 0;
    explicit FileBuilder(int format, uint16_t eb) { f = MakeHeader(4, format, static_cast<uint16_t>(BaseRecordSize(format) + eb)); }
    void Vlr(const char* user, uint16_t record, const std::vector<uint8_t>& data) {
        const size_t at = f.size();
        f.resize(at + 54, 0);
        std::strncpy(reinterpret_cast<char*>(f.data() + at + 2), user, 16);
        Put<uint16_t>(f, at + 18, record);
        Put<uint16_t>(f, at + 20, static_cast<uint16_t>(data.size()));
        std::strncpy(reinterpret_cast<char*>(f.data() + at + 22), "a description", 32);
        f.insert(f.end(), data.begin(), data.end());
        ++vlrs;
    }
    void EndVlrs(size_t pointBytes) {
        Put<uint32_t>(f, 100, vlrs);
        Put<uint32_t>(f, 96, static_cast<uint32_t>(f.size()));
        f.resize(f.size() + pointBytes, 0);
    }
    uint32_t evlrs = 0;
    uint64_t firstEvlr = 0;
    void Evlr(const char* user, uint16_t record, const std::vector<uint8_t>& data) {
        if (evlrs++ == 0) firstEvlr = f.size();
        const size_t at = f.size();
        f.resize(at + 60, 0);
        std::strncpy(reinterpret_cast<char*>(f.data() + at + 2), user, 16);
        Put<uint16_t>(f, at + 18, record);
        Put<uint64_t>(f, at + 20, data.size());
        f.insert(f.end(), data.begin(), data.end());
        Put<uint64_t>(f, 235, firstEvlr);
        Put<uint32_t>(f, 243, evlrs);
    }
};

std::vector<uint8_t> Bytes(const std::string& s, bool nul = true) {
    std::vector<uint8_t> v(s.begin(), s.end());
    if (nul) v.push_back(0);
    return v;
}

TEST(LasMetadata, VlrsAndEvlrs) {
    FileBuilder b(7, 7);
    b.Vlr("LASF_Spec", 4, [] {
        auto d = Descriptor(5, 0, "u32");
        auto e = Descriptor(0, 3, "raw");
        d.insert(d.end(), e.begin(), e.end());
        return d;
    }());
    std::vector<uint8_t> laz;
    for (uint16_t v : {uint16_t{3}, uint16_t{0}}) Append<uint16_t>(laz, v);
    laz.push_back(3);
    laz.push_back(4);
    Append<uint16_t>(laz, 3);
    Append<uint32_t>(laz, 0);
    Append<uint32_t>(laz, 50000);
    Append<int64_t>(laz, -1);
    Append<int64_t>(laz, -1);
    Append<uint16_t>(laz, 2);
    for (int v : {10, 30, 3, 11, 6, 3}) Append<uint16_t>(laz, static_cast<uint16_t>(v));
    b.Vlr("laszip encoded", 22204, laz);
    b.Vlr("other", 77, std::vector<uint8_t>(10, 9));
    b.EndVlrs(1000);
    b.Evlr("LASF_Projection", 2112, Bytes("PROJCS[\"x\"]"));
    b.Evlr("copc", 1000, std::vector<uint8_t>(64, 0));
    b.Evlr("big", 1, std::vector<uint8_t>(70000, 1));

    io::MemorySource src(b.f);
    auto m = ReadMetadata(src);
    ASSERT_TRUE(m.ok()) << m.status().ToString();
    ASSERT_EQ(m->vlrs.size(), 3u);
    EXPECT_EQ(m->vlrs[0].userId, "LASF_Spec");
    EXPECT_EQ(m->vlrs[0].description, "a description");
    EXPECT_EQ(m->vlrs[0].headerOffset, 375u);
    EXPECT_EQ(m->vlrs[0].dataOffset, 375u + 54u);
    ASSERT_EQ(m->extraBytes.size(), 2u);
    EXPECT_EQ(m->extraBytes[1].byteOffset, 4);
    ASSERT_TRUE(m->laz.has_value());
    EXPECT_EQ(m->laz->compressor, 3);
    EXPECT_EQ(m->laz->chunkSize, 50000u);
    EXPECT_FALSE(m->laz->VariableChunks());
    ASSERT_EQ(m->laz->items.size(), 2u);
    EXPECT_EQ(m->laz->items[1].type, 11);
    EXPECT_EQ(m->laz->items[1].size, 6);
    ASSERT_EQ(m->evlrs.size(), 3u);
    EXPECT_TRUE(m->evlrs[0].extended);
    ASSERT_TRUE(m->wkt.has_value());
    EXPECT_EQ(*m->wkt, "PROJCS[\"x\"]");
    ASSERT_TRUE(m->copcHierarchy.has_value());
    EXPECT_FALSE(m->copcHierarchy->payloadKept) << "the hierarchy is never read here";
    EXPECT_EQ(m->copcHierarchy->dataSize, 64u);
    EXPECT_FALSE(m->evlrs[2].payloadKept) << "above maxKeptPayload";
    EXPECT_FALSE(m->IsCopc());
    EXPECT_NE(m->FindVlr("other", 77), nullptr);
    EXPECT_EQ(m->FindVlr("other", 78), nullptr);
}

TEST(LasMetadata, MalformedVlrsAndEvlrs) {
    {
        FileBuilder b(6, 0);
        b.Vlr("x", 1, std::vector<uint8_t>(20, 0));
        b.EndVlrs(100);
        Put<uint16_t>(b.f, 375 + 20, 500);  // payload runs into the points
        io::MemorySource src(b.f);
        EXPECT_EQ(ReadMetadata(src).code(), StatusCode::kCorrupt);
    }
    {
        FileBuilder b(6, 0);
        b.Vlr("x", 1, std::vector<uint8_t>(20, 0));
        b.EndVlrs(100);
        Put<uint32_t>(b.f, 100, 2);  // claims two VLRs
        io::MemorySource src(b.f);
        EXPECT_EQ(ReadMetadata(src).code(), StatusCode::kCorrupt);
    }
    {
        FileBuilder b(6, 0);
        b.EndVlrs(100);
        b.Evlr("x", 1, std::vector<uint8_t>(20, 0));
        Put<uint64_t>(b.f, b.firstEvlr + 20, 1u << 30);  // past the end
        io::MemorySource src(b.f);
        EXPECT_EQ(ReadMetadata(src).code(), StatusCode::kCorrupt);
    }
    {
        FileBuilder b(6, 0);
        b.EndVlrs(100);
        b.Evlr("x", 1, std::vector<uint8_t>(20, 0));
        Put<uint32_t>(b.f, 243, 3);  // more EVLRs than present
        io::MemorySource src(b.f);
        EXPECT_EQ(ReadMetadata(src).code(), StatusCode::kCorrupt);
    }
    {
        // A bad extra-bytes VLR is a warning (PDAL ignores it), not an error.
        FileBuilder b(6, 2);
        b.Vlr("LASF_Spec", 4, Descriptor(6, 0, "too big"));
        b.EndVlrs(100);
        io::MemorySource src(b.f);
        auto m = ReadMetadata(src);
        ASSERT_TRUE(m.ok());
        EXPECT_TRUE(m->extraBytes.empty());
        EXPECT_EQ(m->warnings.size(), 1u);
    }
    {
        // The COPC info VLR must be the first one.
        FileBuilder b(6, 0);
        b.Vlr("x", 1, std::vector<uint8_t>(4, 0));
        b.Vlr("copc", 1, std::vector<uint8_t>(160, 0));
        b.EndVlrs(100);
        io::MemorySource src(b.f);
        auto m = ReadMetadata(src);
        ASSERT_TRUE(m.ok());
        EXPECT_FALSE(m->IsCopc());
        EXPECT_EQ(m->warnings.size(), 1u);
    }
}

// GeoKey directory: version 1.1.0 + keys (id, location, count, value).
std::vector<uint8_t> GeoKeyDirectory(const std::vector<std::array<uint16_t, 4>>& keys) {
    std::vector<uint8_t> v;
    for (uint16_t s : {uint16_t{1}, uint16_t{1}, uint16_t{0}, static_cast<uint16_t>(keys.size())}) Append<uint16_t>(v, s);
    for (const auto& k : keys)
        for (uint16_t s : k) Append<uint16_t>(v, s);
    return v;
}

TEST(LasCrs, GeoKeysAndWktRules) {
    // LAS 1.2 format 3 with GeoTIFF keys only: projected EPSG:32615.
    {
        std::vector<uint8_t> f = MakeHeader(2, 3, 34);
        const auto dir = GeoKeyDirectory({{1024, 0, 1, 1}, {3072, 0, 1, 32615}, {3076, 0, 1, 9001}});
        const size_t at = f.size();
        f.resize(at + 54, 0);
        std::strncpy(reinterpret_cast<char*>(f.data() + at + 2), "LASF_Projection", 16);
        Put<uint16_t>(f, at + 18, 34735);
        Put<uint16_t>(f, at + 20, static_cast<uint16_t>(dir.size()));
        f.insert(f.end(), dir.begin(), dir.end());
        Put<uint32_t>(f, 100, 1);
        Put<uint32_t>(f, 96, static_cast<uint32_t>(f.size()));
        io::MemorySource src(f);
        auto m = ReadMetadata(src);
        ASSERT_TRUE(m.ok()) << m.status().ToString();
        auto d = FindCrsDefinition(*m);
        ASSERT_TRUE(d.ok()) << d.status().ToString();
        EXPECT_EQ(d->source, CrsSource::kGeoKeys);
        EXPECT_EQ(d->text, "EPSG:32615");
        auto crs = ResolveCrs(*m);
        ASSERT_TRUE(crs.ok()) << crs.status().ToString();
        EXPECT_EQ(crs->Epsg().value_or(0), 32615);
    }
    // 1.4 with both: WKT wins (WKT bit / format >= 6); WKT2 and PROJJSON win over WKT.
    {
        FileBuilder b(6, 0);
        b.Vlr("LASF_Projection", 34735, GeoKeyDirectory({{1024, 0, 1, 1}, {3072, 0, 1, 32615}}));
        b.Vlr("LASF_Projection", 2112, Bytes(crs::Crs::FromEpsg(2154).ToWkt(crs::WktFlavor::Wkt1Gdal)));
        b.EndVlrs(0);
        io::MemorySource src(b.f);
        auto m = ReadMetadata(src);
        ASSERT_TRUE(m.ok());
        auto d = FindCrsDefinition(*m);
        ASSERT_TRUE(d.ok());
        EXPECT_EQ(d->source, CrsSource::kWkt);
        auto crs = ResolveCrs(*m);
        ASSERT_TRUE(crs.ok()) << crs.status().ToString();
        EXPECT_EQ(crs->Epsg().value_or(0), 2154);
    }
    {
        LasMetadata m;
        m.header.pointFormat = 3;
        EXPECT_EQ(FindCrsDefinition(m)->source, CrsSource::kNone);
        auto none = ResolveCrs(m);
        ASSERT_TRUE(none.ok());
        EXPECT_FALSE(none->Valid());
        m.wkt = "not a crs";
        EXPECT_EQ(FindCrsDefinition(m)->source, CrsSource::kWkt) << "fallback when the keys are absent";
        EXPECT_FALSE(ResolveCrs(m).ok());
        m.wkt2 = "x";
        EXPECT_EQ(FindCrsDefinition(m)->source, CrsSource::kWkt2);
    }
}

// Corpus files: header fields, COPC info and CRS against what pdal info reports.
TEST(LasMetadata, CorpusCopcFiles) {
    struct Expect {
        const char* path;
        int format;
        uint64_t count;
        double centerX, halfSize, spacing;
        uint64_t rootOffset, rootSize;
        int epsg;  // 0: no EPSG check
    };
    const Expect expects[] = {
        {"/mnt/data/datasets/point-clouds/test-crash/autzen-classified.copc.laz", 7, 10653336, 637905.545, 2327.755,
         36.37117188, 81114146, 8896, 0},
        {"/mnt/data/datasets/point-clouds/test-crash/LHD_FXX_0658_6859_PTS_O_LAMB93_IGN69.copc.laz", 6, 29932350,
         658500, 500, 6.802721088, 204452937, 46976, 0},
        {"/mnt/data/datasets/point-clouds/test-crash/20160402_11SMS46053676.copc.laz", 6, 9801822, 1020593.72,
         792.9545836, 10.78849774, 41083040, 12544, 0},
    };
    int seen = 0;
    for (const Expect& e : expects) {
        if (!std::filesystem::exists(e.path)) continue;
        ++seen;
        auto src = io::OpenByteSource(e.path);
        ASSERT_TRUE(src.ok());
        auto m = ReadMetadata(**src);
        ASSERT_TRUE(m.ok()) << e.path << ": " << m.status().ToString();
        EXPECT_EQ(m->header.versionMinor, 4);
        EXPECT_EQ(m->header.pointFormat, e.format);
        EXPECT_TRUE(m->header.compressed);
        EXPECT_EQ(m->header.pointCount, e.count);
        ASSERT_TRUE(m->copcInfo.has_value());
        // pdal info prints 10 significant digits.
        EXPECT_NEAR(m->copcInfo->centerX, e.centerX, 1e-3);
        EXPECT_NEAR(m->copcInfo->halfSize, e.halfSize, 1e-6);
        EXPECT_NEAR(m->copcInfo->spacing, e.spacing, 1e-8);
        EXPECT_EQ(m->copcInfo->rootHierOffset, e.rootOffset);
        EXPECT_EQ(m->copcInfo->rootHierSize, e.rootSize);
        ASSERT_TRUE(m->laz.has_value());
        EXPECT_EQ(m->laz->compressor, 3);
        EXPECT_TRUE(m->copcHierarchy.has_value());
        auto crs = ResolveCrs(*m);
        ASSERT_TRUE(crs.ok()) << e.path << ": " << crs.status().ToString();
        EXPECT_TRUE(crs->Valid());
        EXPECT_TRUE(crs->IsProjected() || crs->IsCompound()) << crs->Name();
    }
    if (seen == 0) GTEST_SKIP() << "no corpus files";
}

TEST(LasMetadata, CorpusPlainLazWithExtraBytes) {
    const char* path = "/mnt/data/repos/test_data/point-clouds/point-cloud-complex.laz";
    if (!std::filesystem::exists(path)) GTEST_SKIP() << path;
    auto src = io::OpenByteSource(path);
    ASSERT_TRUE(src.ok());
    auto m = ReadMetadata(**src);
    ASSERT_TRUE(m.ok()) << m.status().ToString();
    EXPECT_EQ(m->header.pointFormat, 8);
    EXPECT_EQ(m->header.pointRecordLength, 46);
    EXPECT_EQ(m->header.ExtraBytesPerPoint(), 8);
    EXPECT_FALSE(m->IsCopc());
    ASSERT_FALSE(m->extraBytes.empty());
    uint16_t total = 0;
    for (const auto& e : m->extraBytes) total = static_cast<uint16_t>(total + e.size);
    EXPECT_LE(total, 8);
    ASSERT_TRUE(m->laz.has_value());
    EXPECT_TRUE(m->geoKeys.has_value());
    EXPECT_TRUE(m->wkt.has_value());
    auto d = FindCrsDefinition(*m);
    ASSERT_TRUE(d.ok());
    EXPECT_EQ(d->source, CrsSource::kWkt) << "format 8: WKT first";
    EXPECT_TRUE(ResolveCrs(*m).ok());
}

}  // namespace
}  // namespace ddbt::copc::las
