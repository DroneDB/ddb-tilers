// SPDX-License-Identifier: MPL-2.0
//
// Readers of the COPC builder: LAZ point formats 4 / 5 / 9 / 10 (the fork's WAVEPACKET13 item,
// the skipped WAVEPACKET14 layer) and the chunk-table offset -1, PLY (ASCII, binary LE / BE),
// XYZ / PTS detection and conversions, E57 (synthetic files: CRC-checked pages, bitpacked
// integers split across packets, scaled integers, spherical coordinates, poses, limits), the
// XML parser and CRC-32C.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "../copcbuild/synth_las.h"
#include "../test_env.h"
#include "copc/readers/convert.h"
#include "copc/readers/e57_reader.h"
#include "copc/readers/las_reader.h"
#include "copc/readers/point_reader.h"
#include "core/simd/simd.h"
#include "core/util/endian.h"

namespace ddbt::copc::readers {
namespace {

namespace fs = std::filesystem;
using copc::testing::BuildLasBytes;
using copc::testing::FillRandom;
using copc::testing::SynthLas;
using copc::testing::WriteBytes;

std::string TempDir() {
    const fs::path p = fs::temp_directory_path() / ("ddbt-readers-" + ddbt::test::ProcessIdString());
    fs::create_directories(p);
    return p.string();
}

void WriteText(const std::string& path, const std::string& s) {
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f << s;
}

/// Every point of a reader as records of `spec` (X / Y / Z quantised), in task order.
std::vector<uint8_t> ReadAll(const PointReader& r, const OutputSpec& spec) {
    struct Sink final : BatchSink {
        const OutputSpec* spec;
        std::vector<uint8_t> out;
        Status Consume(Batch& b) override {
            if (b.xyz) DDBT_TRY(QuantizeXyz(*spec, b.xyz, b.count, b.records));
            out.insert(out.end(), b.records, b.records + b.count * spec->RecordSize());
            return Status();
        }
    } sink;
    sink.spec = &spec;
    for (size_t t = 0; t < r.TaskCount(); ++t) EXPECT_TRUE(r.Read(t, spec, sink).ok()) << "task " << t;
    return std::move(sink.out);
}

OutputSpec Spec(int format, double scale = 0.001) {
    OutputSpec s;
    s.format = format;
    s.scale = {scale, scale, scale};
    s.offset = {0, 0, 0};
    return s;
}

int32_t X(const std::vector<uint8_t>& recs, const OutputSpec& s, size_t i, int axis) {
    return LoadLE<int32_t>(recs.data() + i * s.RecordSize() + 4 * static_cast<size_t>(axis));
}
uint16_t U16(const std::vector<uint8_t>& recs, const OutputSpec& s, size_t i, size_t off) {
    return LoadLE<uint16_t>(recs.data() + i * s.RecordSize() + off);
}

TEST(CopcReaders, LazWaveformFormatsDecodeAsTheirLasRecords) {
    const std::string dir = TempDir();
    for (int f : {4, 5, 9, 10}) {
        SCOPED_TRACE("format " + std::to_string(f));
        SynthLas s;
        s.format = f;
        s.minor = f >= 9 ? 4 : 3;
        s.extraBytes = f == 5 || f == 10 ? 3 : 0;
        s.chunkSize = 4000;
        FillRandom(s, 13000, static_cast<uint32_t>(100 + f));
        const size_t rs = s.RecordSize();
        const std::string las = dir + "/w.las", laz = dir + "/w.laz";
        ASSERT_TRUE(WriteBytes(las, BuildLasBytes(s)));
        s.laz = true;
        s.chunkTableMinusOne = f == 5;
        ASSERT_TRUE(WriteBytes(laz, BuildLasBytes(s)));
        auto rz = LasReader::Open(laz);
        ASSERT_TRUE(rz.ok()) << rz.status().ToString();
        ASSERT_EQ((*rz)->TaskCount(), 4u);
        // Raw records: equal to the originals (format 9 / 10: wave packets are zeros).
        const las::PointLayout L = las::LayoutOf(f);
        uint64_t at = 0;
        for (size_t t = 0; t < (*rz)->TaskCount(); ++t) {
            std::vector<uint8_t> raw;
            const auto n = static_cast<uint32_t>((*rz)->TaskPoints(t));
            ASSERT_TRUE((*rz)->ReadRawRecords(t, n, raw).ok());
            for (uint32_t i = 0; i < n; ++i, ++at) {
                std::vector<uint8_t> want(s.records.begin() + static_cast<ptrdiff_t>(at * rs),
                                          s.records.begin() + static_cast<ptrdiff_t>((at + 1) * rs));
                if (f >= 9) std::memset(want.data() + L.wavePacket, 0, 29);
                if (std::memcmp(raw.data() + size_t{i} * rs, want.data(), rs) != 0) {
                    std::string diff;
                    for (size_t k = 0; k < rs; ++k)
                        if (raw[size_t{i} * rs + k] != want[k]) diff += " " + std::to_string(k);
                    FAIL() << "point " << at << " differs at bytes" << diff;
                }
            }
        }
        // Converted records: the same from LAS and LAZ.
        auto rl = LasReader::Open(las);
        ASSERT_TRUE(rl.ok());
        OutputSpec spec;
        spec.format = L.HasNir() ? 8 : L.HasRgb() ? 7 : 6;
        spec.extraBytes = s.extraBytes;
        spec.scale = s.scale;
        spec.offset = s.offset;
        EXPECT_TRUE(ReadAll(**rl, spec) == ReadAll(**rz, spec));
    }
    fs::remove_all(dir);
}

TEST(CopcReaders, PlyAsciiAndBinary) {
    const std::string dir = TempDir();
    const std::string header =
        "element vertex 3\nproperty double x\nproperty float y\nproperty int z\nproperty uchar red\n"
        "property uchar green\nproperty uchar blue\nproperty ushort intensity\nproperty float nx\n";
    // ASCII with a face element after the vertices and CRLF line ends.
    WriteText(dir + "/a.ply", "ply\r\nformat ascii 1.0\r\ncomment x\r\n" + header +
                                  "element face 1\nproperty list uchar int vertex_indices\nend_header\r\n"
                                  "1.5 2.25 3 255 0 1 1000 0.5\r\n-1 -2 -3 10 20 30 7 0\n4 5 6 0 0 0 0 0\n3 0 1 2\n");
    for (const bool big : {false, true}) {
        std::vector<uint8_t> b;
        const std::string h = std::string("ply\nformat ") + (big ? "binary_big_endian" : "binary_little_endian") +
                              " 1.0\n" + header + "end_header\n";
        b.insert(b.end(), h.begin(), h.end());
        const double xs[3] = {1.5, -1, 4};
        const float ys[3] = {2.25f, -2, 5};
        const int32_t zs[3] = {3, -3, 6};
        const uint8_t col[3][3] = {{255, 0, 1}, {10, 20, 30}, {0, 0, 0}};
        const uint16_t in[3] = {1000, 7, 0};
        for (int i = 0; i < 3; ++i) {
            uint8_t v[8 + 4 + 4 + 3 + 2 + 4];
            if (big) {
                StoreBE<double>(v, xs[i]);
                StoreBE<float>(v + 8, ys[i]);
                StoreBE<int32_t>(v + 12, zs[i]);
                StoreBE<uint16_t>(v + 19, in[i]);
            } else {
                StoreLE<double>(v, xs[i]);
                StoreLE<float>(v + 8, ys[i]);
                StoreLE<int32_t>(v + 12, zs[i]);
                StoreLE<uint16_t>(v + 19, in[i]);
            }
            std::memcpy(v + 16, col[i], 3);
            StoreLE<float>(v + 21, 0.5f);
            b.insert(b.end(), v, v + sizeof(v));
        }
        ASSERT_TRUE(WriteBytes(dir + (big ? "/be.ply" : "/le.ply"), b));
    }
    for (const char* name : {"/a.ply", "/le.ply", "/be.ply"}) {
        SCOPED_TRACE(name);
        auto r = OpenReader(dir + name);
        ASSERT_TRUE(r.ok()) << r.status().ToString();
        EXPECT_EQ((*r)->Info().pointCount, 3u);
        EXPECT_TRUE((*r)->Info().hasRgb);
        const OutputSpec spec = Spec(7);
        const auto recs = ReadAll(**r, spec);
        ASSERT_EQ(recs.size(), 3 * spec.RecordSize());
        EXPECT_EQ(X(recs, spec, 0, 0), 1500);
        EXPECT_EQ(X(recs, spec, 0, 1), 2250);
        EXPECT_EQ(X(recs, spec, 0, 2), 3000);
        EXPECT_EQ(X(recs, spec, 1, 2), -3000);
        EXPECT_EQ(U16(recs, spec, 0, rec::kRgb), 65535);
        EXPECT_EQ(U16(recs, spec, 0, rec::kRgb + 4), 257);
        EXPECT_EQ(U16(recs, spec, 1, rec::kRgb + 2), 20 * 257);
        EXPECT_EQ(U16(recs, spec, 0, rec::kIntensity), 1000);
        EXPECT_EQ(recs[rec::kReturns], 0x11);  // return 1 of 1
    }
    fs::remove_all(dir);
}

TEST(CopcReaders, XyzDetectionAsDroneDb) {
    const std::string dir = TempDir();
    struct Case {
        std::string name, text;
        size_t points;
        std::vector<int32_t> firstXyz;
        int format;
        uint16_t firstRed;
    };
    const std::vector<Case> cases = {
        {"cc.xyz", "//Generated\n//X Y Z R G B Return_Number Number_Of_Returns User_Data\n1.5 2 3 95 116 77 1 2 3\n4 5 6 1 2 3 2 2 9\n", 2,
         {1500, 2000, 3000}, 7, 95 * 257},
        {"noh.xyz", "1 2 3 100 150 200\n4 5 6 110 160 210\n", 2, {1000, 2000, 3000}, 7, 100 * 257},
        {"count.xyz", "2\n1 2 3\n4 5 6\n", 2, {1000, 2000, 3000}, 6, 0},
        {"tab.xyz", "X\tY\tZ\n1.0\t2.0\t3.0\n4.0\t5.0\t6.0\n", 2, {1000, 2000, 3000}, 6, 0},
        {"semi.xyz", "X;Y;Z\n1.0;2.0;3.0\n4.0;5.0;6.0\n", 2, {1000, 2000, 3000}, 6, 0},
        {"comma.xyz", "X,Y,Z\n1, 1, 5\n6, 2, 10\n", 2, {1000, 1000, 5000}, 6, 0},
        {"quoted.xyz", "\"X\",\"Y\",\"Z\",\"Red\",\"Green\",\"Blue\"\n1,2,3,40000,0,0\n", 1, {1000, 2000, 3000}, 7, 40000},
        {"eu.xyz", "500102,364;4600056,835;3,445\n500190,093;4600001,960;2,658\n", 2, {500102364, 2147483647, 3445}, 6, 0},
        {"unit.xyz", "//X Y Z R G B\n1 2 3 1.0 0.5 0\n2 3 4 0.25 0 0\n", 2, {1000, 2000, 3000}, 7, 65535},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.name);
        WriteText(dir + "/" + c.name, c.text);
        auto r = OpenReader(dir + "/" + c.name);
        ASSERT_TRUE(r.ok()) << r.status().ToString();
        EXPECT_EQ((*r)->Info().hasRgb, c.format == 7);
        OutputSpec spec = Spec(c.format);
        if (c.name == "eu.xyz") spec.offset = {0, 4600000, 0};
        const auto recs = ReadAll(**r, spec);
        ASSERT_EQ(recs.size() / spec.RecordSize(), c.points);
        if (c.name == "eu.xyz") {
            EXPECT_EQ(X(recs, spec, 0, 0), 500102364);
            EXPECT_EQ(X(recs, spec, 0, 1), 56835);
            EXPECT_EQ(X(recs, spec, 0, 2), 3445);
        } else {
            for (int a = 0; a < 3; ++a) EXPECT_EQ(X(recs, spec, 0, a), c.firstXyz[static_cast<size_t>(a)]) << a;
        }
        if (c.format == 7) {
            EXPECT_EQ(U16(recs, spec, 0, rec::kRgb), c.firstRed);
        }
        if (c.name == "cc.xyz") {
            EXPECT_EQ(recs[spec.RecordSize() + rec::kReturns], 2 | (2 << 4));
            EXPECT_EQ(recs[spec.RecordSize() + rec::kUserData], 9);
        }
        if (c.name == "unit.xyz") {
            EXPECT_EQ(U16(recs, spec, 0, rec::kRgb + 2), 32768);
        }
    }
    fs::remove_all(dir);
}

TEST(CopcReaders, PtsCountsScansAndIntensity) {
    const std::string dir = TempDir();
    // PDAL reads the announced count: the extra line is not a point.
    WriteText(dir + "/a.pts", "2\n1 2 3 -2048\n4 5 6 2047\n7 8 9 0\n");
    // Two scans, 7 values (X Y Z I R G B).
    WriteText(dir + "/b.pts", "1\n1 2 3 0 10 20 30\n2\n4 5 6 -255 1 2 3\n7 8 9 100 4 5 6\n");
    auto a = OpenReader(dir + "/a.pts");
    ASSERT_TRUE(a.ok()) << a.status().ToString();
    OutputSpec s6 = Spec(6);
    auto ra = ReadAll(**a, s6);
    ASSERT_EQ(ra.size() / s6.RecordSize(), 2u);
    EXPECT_EQ(U16(ra, s6, 0, rec::kIntensity), 0);
    EXPECT_EQ(U16(ra, s6, 1, rec::kIntensity), 65520);
    auto b = OpenReader(dir + "/b.pts");
    ASSERT_TRUE(b.ok()) << b.status().ToString();
    OutputSpec s7 = Spec(7);
    auto rb = ReadAll(**b, s7);
    ASSERT_EQ(rb.size() / s7.RecordSize(), 3u);
    EXPECT_EQ(U16(rb, s7, 1, rec::kIntensity), (2048 - 255) * 16);
    EXPECT_EQ(U16(rb, s7, 0, rec::kRgb), 10 * 257);
    EXPECT_EQ(X(rb, s7, 2, 2), 9000);
    fs::remove_all(dir);
}

// CRC-32C on every target the CPU runs (the SSE 4.2 instruction from SSE4 on, the table below):
// the bitwise definition, for every length up to 70 and every alignment of the start.
TEST(CopcReaders, Crc32cOnEveryTarget) {
    const auto reference = [](const uint8_t* p, size_t n) {
        uint32_t crc = 0xFFFFFFFFu;
        for (size_t i = 0; i < n; ++i) {
            crc ^= p[i];
            for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ ((crc & 1u) ? 0x82F63B78u : 0u);
        }
        return crc ^ 0xFFFFFFFFu;
    };
    std::vector<uint8_t> bytes(80);
    for (size_t i = 0; i < bytes.size(); ++i) bytes[i] = static_cast<uint8_t>(i * 131u + 7u);
    struct Guard {
        ~Guard() { ddbt::simd::ForceTargetForTest(0); }
    } guard;
    for (const int64_t bit : ddbt::simd::SupportedTargetBits()) {
        ddbt::simd::ForceTargetForTest(bit);
        for (size_t start = 0; start < 8; ++start)
            for (size_t n = 0; n + start <= 70; ++n)
                ASSERT_EQ(Crc32c(bytes.data() + start, n), reference(bytes.data() + start, n))
                    << ddbt::simd::TargetNameOf(bit) << " start " << start << " length " << n;
    }
}

TEST(CopcReaders, Crc32cAndXml) {
    const char* t = "123456789";
    EXPECT_EQ(Crc32c(reinterpret_cast<const uint8_t*>(t), 9), 0xE3069283u);
    auto x = ParseXml("<?xml version=\"1.0\"?><!-- c --><root a='1' b=\"x&amp;y\"><c type=\"Float\">1.5</c>"
                      "<d><![CDATA[<raw>]]></d><e/>t&lt;&#65;</root>");
    ASSERT_TRUE(x.ok()) << x.status().ToString();
    EXPECT_EQ(x->name, "root");
    EXPECT_EQ(*x->Attribute("b"), "x&y");
    EXPECT_EQ(x->Child("c")->text, "1.5");
    EXPECT_EQ(x->Child("d")->text, "<raw>");
    EXPECT_NE(x->Child("e"), nullptr);
    EXPECT_EQ(x->text, "t<A");
    EXPECT_FALSE(ParseXml("<a><b></a>").ok());
}

// ---------------------------------------------------------------- synthetic E57

class BitWriter {
public:
    void Put(uint64_t v, int w) {
        for (int i = 0; i < w; ++i) {
            if (bit_ % 8 == 0) bytes.push_back(0);
            if ((v >> i) & 1u) bytes.back() |= static_cast<uint8_t>(1u << (bit_ % 8));
            ++bit_;
        }
    }
    std::vector<uint8_t> bytes;

private:
    uint64_t bit_ = 0;
};

struct E57Field {
    std::string xml;  // prototype element
    BitWriter stream;
};

/// An E57 file of scans, each given as prototype fields with their encoded streams, the scan
/// XML around the points (pose, limits), and the record count. Streams are cut into packets
/// of `perPacket` bytes per stream.
std::vector<uint8_t> BuildE57(std::vector<std::pair<std::vector<E57Field>, std::string>> scans,
                              const std::vector<uint64_t>& counts, size_t perPacket) {
    std::vector<uint8_t> L(48, 0);  // logical bytes; the header is rewritten at the end
    auto phys = [](uint64_t l) { return l / 1020 * 1024 + l % 1020; };
    auto put16 = [&](uint16_t v) { L.push_back(static_cast<uint8_t>(v)); L.push_back(static_cast<uint8_t>(v >> 8)); };
    std::string data3D;
    for (size_t s = 0; s < scans.size(); ++s) {
        auto& fields = scans[s].first;
        const uint64_t secStart = L.size();
        L.resize(L.size() + 32, 0);
        const uint64_t dataStart = L.size();
        std::vector<size_t> pos(fields.size(), 0);
        for (;;) {
            bool more = false;
            for (size_t f = 0; f < fields.size(); ++f) more = more || pos[f] < fields[f].stream.bytes.size();
            if (!more) break;
            const uint64_t pk = L.size();
            L.push_back(1);
            L.push_back(0);
            put16(0);
            put16(static_cast<uint16_t>(fields.size()));
            std::vector<size_t> take(fields.size());
            for (size_t f = 0; f < fields.size(); ++f) {
                take[f] = std::min(perPacket + f * 3, fields[f].stream.bytes.size() - pos[f]);
                put16(static_cast<uint16_t>(take[f]));
            }
            for (size_t f = 0; f < fields.size(); ++f) {
                L.insert(L.end(), fields[f].stream.bytes.begin() + static_cast<ptrdiff_t>(pos[f]),
                         fields[f].stream.bytes.begin() + static_cast<ptrdiff_t>(pos[f] + take[f]));
                pos[f] += take[f];
            }
            while ((L.size() - pk) % 4) L.push_back(0);
            StoreLE<uint16_t>(L.data() + pk + 2, static_cast<uint16_t>(L.size() - pk - 1));
        }
        L[secStart] = 1;
        StoreLE<uint64_t>(L.data() + secStart + 8, L.size() - secStart);
        StoreLE<uint64_t>(L.data() + secStart + 16, phys(dataStart));
        std::string proto;
        for (const auto& f : fields) proto += f.xml;
        data3D += "<vectorChild type=\"Structure\">" + scans[s].second + "<points type=\"CompressedVector\" fileOffset=\"" +
                  std::to_string(phys(secStart)) + "\" recordCount=\"" + std::to_string(counts[s]) +
                  "\"><prototype type=\"Structure\">" + proto +
                  "</prototype><codecs type=\"Vector\" allowHeterogeneousChildren=\"1\"></codecs></points></vectorChild>";
    }
    const std::string xml = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<e57Root type=\"Structure\" "
                            "xmlns=\"http://www.astm.org/COMMIT/E57/2010-e57-v1.0\"><data3D type=\"Vector\">" +
                            data3D + "</data3D></e57Root>\n";
    const uint64_t xmlStart = L.size();
    L.insert(L.end(), xml.begin(), xml.end());
    while (L.size() % 1020) L.push_back(0);
    const uint64_t physLen = L.size() / 1020 * 1024;
    std::memcpy(L.data(), "ASTM-E57", 8);
    StoreLE<uint32_t>(L.data() + 8, 1);
    StoreLE<uint32_t>(L.data() + 12, 0);
    StoreLE<uint64_t>(L.data() + 16, physLen);
    StoreLE<uint64_t>(L.data() + 24, phys(xmlStart));
    StoreLE<uint64_t>(L.data() + 32, xml.size());
    StoreLE<uint64_t>(L.data() + 40, 1024);
    std::vector<uint8_t> P;
    for (size_t p = 0; p < L.size(); p += 1020) {
        P.insert(P.end(), L.begin() + static_cast<ptrdiff_t>(p), L.begin() + static_cast<ptrdiff_t>(p + 1020));
        uint8_t c[4];
        StoreBE<uint32_t>(c, Crc32c(L.data() + p, 1020));
        P.insert(P.end(), c, c + 4);
    }
    return P;
}

TEST(CopcReaders, E57ScansDecodeLikePdal) {
    const std::string dir = TempDir();
    std::mt19937_64 rng(7);
    const uint64_t n1 = 20000, n2 = 9000;
    // Scan 1: cartesian doubles, colour 0-255, intensity ScaledInteger 0-1000 (10 bits), a pose
    // rotating 90 degrees about z and translating.
    std::vector<E57Field> f1(7);
    f1[0].xml = "<cartesianX type=\"Float\"/>";
    f1[1].xml = "<cartesianY type=\"Float\"/>";
    f1[2].xml = "<cartesianZ type=\"Float\" precision=\"single\"/>";
    f1[3].xml = "<colorRed type=\"Integer\" minimum=\"0\" maximum=\"255\"/>";
    f1[4].xml = "<colorGreen type=\"Integer\" minimum=\"0\" maximum=\"255\"/>";
    f1[5].xml = "<colorBlue type=\"Integer\" minimum=\"0\" maximum=\"255\"/>";
    f1[6].xml = "<intensity type=\"ScaledInteger\" minimum=\"-500\" maximum=\"500\" scale=\"0.001\" offset=\"0.5\"/>";
    std::vector<std::array<double, 7>> want1(n1);
    for (uint64_t i = 0; i < n1; ++i) {
        const double x = static_cast<double>(rng() % 100000) / 1000.0, y = static_cast<double>(rng() % 100000) / 1000.0;
        const float z = static_cast<float>(rng() % 5000) / 100.0f;
        const uint64_t r = rng() % 256, g = rng() % 256, b = rng() % 256;
        const int64_t it = static_cast<int64_t>(rng() % 1001) - 500;
        f1[0].stream.Put(std::bit_cast<uint64_t>(x), 64);
        f1[1].stream.Put(std::bit_cast<uint64_t>(y), 64);
        f1[2].stream.Put(std::bit_cast<uint32_t>(z), 32);
        f1[3].stream.Put(r, 8);
        f1[4].stream.Put(g, 8);
        f1[5].stream.Put(b, 8);
        f1[6].stream.Put(static_cast<uint64_t>(it + 500), 10);
        want1[i] = {x, y, z, double(r), double(g), double(b), static_cast<double>(it) * 0.001 + 0.5};
    }
    const std::string pose1 =
        "<pose type=\"Structure\"><rotation type=\"Structure\"><w type=\"Float\">0.70710678118654757</w><x type=\"Float\"/>"
        "<y type=\"Float\"/><z type=\"Float\">0.70710678118654757</z></rotation><translation type=\"Structure\">"
        "<x type=\"Float\">1000</x><y type=\"Float\">2000</y><z type=\"Float\">5</z></translation></pose>"
        "<intensityLimits type=\"Structure\"><intensityMinimum type=\"Float\">0</intensityMinimum>"
        "<intensityMaximum type=\"Float\">1</intensityMaximum></intensityLimits>";
    // Scan 2: spherical (range ScaledInteger, angles double), 16-bit colour limits, 3-bit
    // integers (returnIndex) packed across byte and packet boundaries.
    std::vector<E57Field> f2(5);
    f2[0].xml = "<sphericalRange type=\"ScaledInteger\" minimum=\"0\" maximum=\"100000\" scale=\"0.001\"/>";
    f2[1].xml = "<sphericalAzimuth type=\"Float\"/>";
    f2[2].xml = "<sphericalElevation type=\"Float\"/>";
    f2[3].xml = "<returnIndex type=\"Integer\" minimum=\"0\" maximum=\"5\"/>";
    f2[4].xml = "<colorRed type=\"Integer\" minimum=\"0\" maximum=\"4095\"/>";
    std::vector<std::array<double, 5>> want2(n2);
    for (uint64_t i = 0; i < n2; ++i) {
        const uint64_t range = rng() % 100001;
        const double az = static_cast<double>(rng() % 6283) / 1000.0, el = static_cast<double>(rng() % 3000) / 2000.0 - 0.75;
        const uint64_t ri = rng() % 6, red = rng() % 4096;
        f2[0].stream.Put(range, 17);
        f2[1].stream.Put(std::bit_cast<uint64_t>(az), 64);
        f2[2].stream.Put(std::bit_cast<uint64_t>(el), 64);
        f2[3].stream.Put(ri, 3);
        f2[4].stream.Put(red, 12);
        want2[i] = {static_cast<double>(range) * 0.001, az, el, double(ri), double(red)};
    }
    const std::vector<uint8_t> file = BuildE57({{f1, pose1}, {f2, ""}}, {n1, n2}, 977);
    const std::string path = dir + "/s.e57";
    ASSERT_TRUE(WriteBytes(path, file));
    ReaderOptions ro;
    ro.pointsPerTask = 4096;  // tasks start mid-packet and mid-byte
    auto r = E57Reader::Open(path, ro);
    ASSERT_TRUE(r.ok()) << r.status().ToString();
    EXPECT_EQ((*r)->ScanCount(), 2u);
    EXPECT_EQ((*r)->Info().pointCount, n1 + n2);
    EXPECT_GT((*r)->TaskCount(), 6u);
    OutputSpec spec = Spec(7, 0.0001);
    spec.offset = {1000, 2000, 0};
    const auto recs = ReadAll(**r, spec);
    ASSERT_EQ(recs.size() / spec.RecordSize(), n1 + n2);
    auto q = [&](double v, int a) { return static_cast<int32_t>(std::round((v - spec.offset[static_cast<size_t>(a)]) / 0.0001)); };
    const double c = 0.70710678118654757;
    const double r00 = 1 - 2.0 * (c * c), r01 = 2.0 * (0 - c * c), r10 = 2.0 * (0 + c * c);
    for (uint64_t i = 0; i < n1; i += 37) {
        const auto& w = want1[i];
        const double x = w[0] * r00 + w[1] * r01 + w[2] * 0 + 1000, y = w[0] * r10 + w[1] * r00 + w[2] * 0 + 2000,
                     z = w[0] * 0 + w[1] * 0 + w[2] * (1 - 2.0 * (0 + 0)) + 5;
        ASSERT_NEAR(X(recs, spec, i, 0), q(x, 0), 1) << i;
        ASSERT_NEAR(X(recs, spec, i, 1), q(y, 1), 1) << i;
        ASSERT_EQ(X(recs, spec, i, 2), q(z, 2)) << i;
        ASSERT_EQ(U16(recs, spec, i, rec::kRgb), static_cast<uint16_t>(w[3] * 257)) << i;
        ASSERT_EQ(U16(recs, spec, i, rec::kRgb + 4), static_cast<uint16_t>(w[5] * 257)) << i;
        ASSERT_EQ(U16(recs, spec, i, rec::kIntensity), ClampU16(w[6] * 65535.0)) << i;
    }
    const double redFactor = static_cast<double>(static_cast<float>(65535.0 / 4095.0));
    for (uint64_t i = 0; i < n2; i += 29) {
        const auto& w = want2[i];
        const size_t k = static_cast<size_t>(n1 + i);
        const double x = w[0] * std::cos(w[2]) * std::cos(w[1]), y = w[0] * std::cos(w[2]) * std::sin(w[1]),
                     z = w[0] * std::sin(w[2]);
        ASSERT_NEAR(X(recs, spec, k, 0), q(x, 0), 1) << i;
        ASSERT_NEAR(X(recs, spec, k, 1), q(y, 1), 1) << i;
        ASSERT_NEAR(X(recs, spec, k, 2), q(z, 2), 1) << i;
        ASSERT_EQ(recs[k * spec.RecordSize() + rec::kReturns] & 0x0F, static_cast<int>(w[3]) + 1) << i;
        ASSERT_EQ(U16(recs, spec, k, rec::kRgb), ClampU16(w[4] * redFactor)) << i;
    }
    // A flipped bit fails the page's CRC.
    std::vector<uint8_t> bad = file;
    bad[3000] ^= 0x10;
    ASSERT_TRUE(WriteBytes(dir + "/bad.e57", bad));
    auto rb = E57Reader::Open(dir + "/bad.e57", ro);
    bool failed = !rb.ok();
    if (rb.ok()) {
        struct Null final : BatchSink {
            Status Consume(Batch&) override { return Status(); }
        } sink;
        for (size_t t = 0; t < (*rb)->TaskCount(); ++t) failed = failed || !(*rb)->Read(t, spec, sink).ok();
    }
    EXPECT_TRUE(failed);
    fs::remove_all(dir);
}

}  // namespace
}  // namespace ddbt::copc::readers
