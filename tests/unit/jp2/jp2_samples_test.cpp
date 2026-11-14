// SPDX-License-Identifier: MPL-2.0
//
// JPEG 2000 files as GDAL's JP2OpenJPEG driver reads them (tests/fixtures/jp2, expected.tsv
// written by expected.py with the system GDAL): the size, band type, every band's samples
// (CRC-32 of band.ReadRaster()), colour interpretations and NBITS, the colour table, the
// geotransform or GCPs, the CRS and RPC presence, and the files GDAL refuses. The same check runs
// on any folder with DDBT_JP2_PARITY_DIR (the files) and DDBT_JP2_PARITY_TSV (expected.py's
// output for it), e.g. on all of GDAL's own JPEG 2000 samples.

#include <gtest/gtest.h>

#include <libdeflate.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "cog/build/jp2_source.h"
#include "cog/build/source.h"
#include "core/crs/crs.h"

namespace ddbt::cog::build {
namespace {

std::vector<std::vector<std::string>> ReadTsv(const std::string& path) {
    std::ifstream in(path);
    std::vector<std::vector<std::string>> out;
    std::string line;
    while (std::getline(in, line)) {
        std::vector<std::string> f;
        std::stringstream s(line);
        std::string field;
        while (std::getline(s, field, '\t')) f.push_back(field);
        if (!f.empty()) out.push_back(std::move(f));
    }
    return out;
}

const char* TypeName(tiff::SampleType t) {
    switch (t) {
        case tiff::SampleType::kU8: return "Byte";
        case tiff::SampleType::kI16: return "Int16";
        case tiff::SampleType::kU16: return "UInt16";
        case tiff::SampleType::kI32: return "Int32";
        case tiff::SampleType::kU32: return "UInt32";
        default: return "?";
    }
}

// Band b of [x, x + w) x [y, y + h) of the source, rows of w samples (host order).
std::vector<uint8_t> ReadBand(const RasterSource& s, int b, uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
    const size_t ss = s.SampleSize(), pb = s.PixelBytes();
    std::vector<uint8_t> out(static_cast<size_t>(w) * h * ss);
    std::vector<uint8_t> block(s.BlockBytes());
    const uint32_t bw = s.BlockWidth(), bh = s.BlockHeight();
    for (uint32_t by = y / bh; by <= (y + h - 1) / bh; ++by) {
        for (uint32_t bx = x / bw; bx <= (x + w - 1) / bw; ++bx) {
            auto rows = s.ReadBlock(bx, by, block.data());
            EXPECT_TRUE(rows.ok()) << rows.status().message();
            if (!rows.ok()) return {};
            const uint32_t x0 = std::max(x, bx * bw), x1 = std::min(x + w, (bx + 1) * bw);
            const uint32_t y0 = std::max(y, by * bh), y1 = std::min(y + h, by * bh + *rows);
            for (uint32_t yy = y0; yy < y1; ++yy)
                for (uint32_t xx = x0; xx < x1; ++xx)
                    std::memcpy(out.data() + (static_cast<size_t>(yy - y) * w + (xx - x)) * ss,
                                block.data() + (static_cast<size_t>(yy - by * bh) * bw + (xx - bx * bw)) * pb +
                                    static_cast<size_t>(b) * ss,
                                ss);
        }
    }
    return out;
}

std::string Crc(const std::vector<uint8_t>& v) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%08x", libdeflate_crc32(0, v.data(), v.size()));
    return buf;
}

double D(const std::string& s) { return std::strtod(s.c_str(), nullptr); }

// expected.py's escapes of a field: \\, \n, \t.
std::string Unescape(const std::string& s) {
    std::string out;
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '\\' && i + 1 < s.size()) {
            const char c = s[++i];
            out += c == 'n' ? '\n' : c == 't' ? '\t' : c;
        } else {
            out += s[i];
        }
    }
    return out;
}

// Checks the expectations of one file.
void CheckFile(const std::string& dir, const std::string& name, const std::vector<std::vector<std::string>>& lines) {
    SCOPED_TRACE(name);
    {
        auto src = OpenRasterSource(dir + "/" + name);
        if (lines[0][0] == "refused") {
            // GDAL cannot open or decode the file: neither can the source (opening or reading).
            if (!src.ok()) return;
            std::vector<uint8_t> block((*src)->BlockBytes());
            bool failed = false;
            for (uint32_t by = 0; by < (*src)->BlocksDown() && !failed; ++by)
                for (uint32_t bx = 0; bx < (*src)->BlocksAcross() && !failed; ++bx)
                    failed = !(*src)->ReadBlock(bx, by, block.data()).ok();
            EXPECT_TRUE(failed) << "GDAL refuses " << name;
            return;
        }
        ASSERT_TRUE(src.ok()) << src.status().message();
        const RasterSource& s = **src;
        const auto* jp2 = dynamic_cast<const Jp2Source*>(&s);
        ASSERT_NE(jp2, nullptr);
        std::vector<std::array<double, 5>> gcps;
        bool hasGt = false, hasCrs = false, rpc = false;
        for (const auto& l : lines) {
            const std::string& kind = l[0];
            if (kind == "size") {
                EXPECT_EQ(std::to_string(s.Width()), l[2]);
                EXPECT_EQ(std::to_string(s.Height()), l[3]);
                EXPECT_EQ(std::to_string(s.Bands()), l[4]);
                EXPECT_EQ(TypeName(s.Type()), l[5]);
            } else if (kind == "band") {
                const int b = std::stoi(l[2]) - 1;
                SCOPED_TRACE("band " + l[2]);
                EXPECT_EQ(s.ColorInterp(b), l[3]);
                const auto nbits = jp2->Nbits(b);
                EXPECT_EQ(nbits ? std::to_string(*nbits) : "-", l[4]);
                if (l[5] != "-") {
                    EXPECT_EQ(Crc(ReadBand(s, b, 0, 0, s.Width(), s.Height())), l[5]);
                }
            } else if (kind == "window") {
                SCOPED_TRACE("window " + l[3] + "," + l[4]);
                const auto v = ReadBand(s, std::stoi(l[2]) - 1, static_cast<uint32_t>(std::stoul(l[3])),
                                        static_cast<uint32_t>(std::stoul(l[4])), static_cast<uint32_t>(std::stoul(l[5])),
                                        static_cast<uint32_t>(std::stoul(l[6])));
                EXPECT_EQ(Crc(v), l[7]);
            } else if (kind == "ct") {
                const auto t = s.ColorTable();
                ASSERT_TRUE(t.has_value());
                EXPECT_EQ(std::to_string(t->size()), l[2]);
                std::vector<uint8_t> rgb;
                for (const auto& e : *t) rgb.insert(rgb.end(), e.begin(), e.end());
                EXPECT_EQ(Crc(rgb), l[3]);
            } else if (kind == "gt") {
                hasGt = true;
                ASSERT_TRUE(s.Georef().geoTransform.has_value());
                for (size_t i = 0; i < 6; ++i) EXPECT_EQ((*s.Georef().geoTransform)[i], D(l[2 + i])) << "gt " << i;
            } else if (kind == "gcp") {
                gcps.push_back({D(l[2]), D(l[3]), D(l[4]), D(l[5]), D(l[6])});
            } else if (kind == "crs") {
                hasCrs = true;
                ASSERT_TRUE(s.Georef().crs.has_value()) << s.Georef().crsError;
                EXPECT_TRUE(s.Georef().crs->Equivalent(crs::Crs::FromWkt(Unescape(l[2]))))
                    << s.Georef().crs->ToWkt() << "\nGDAL: " << l[2];
            } else if (kind == "rpc") {
                rpc = true;
            }
        }
        EXPECT_EQ(s.Georef().geoTransform.has_value(), hasGt);
        EXPECT_EQ(s.Georef().crs.has_value(), hasCrs);
        EXPECT_EQ(s.HasRpc(), rpc);
        ASSERT_EQ(s.Georef().gcps.size(), gcps.size());
        for (size_t i = 0; i < gcps.size(); ++i) {
            const auto& p = s.Georef().gcps[i];
            EXPECT_EQ((std::array<double, 5>{p.pixel, p.line, p.x, p.y, p.z}), gcps[i]) << "gcp " << i;
        }
    }
}

// Checks every expectation of `tsv` on the files of `dir`; returns the number of files checked.
size_t CheckFolder(const std::string& dir, const std::string& tsv) {
    const auto rows = ReadTsv(tsv);
    EXPECT_FALSE(rows.empty()) << tsv;
    std::map<std::string, std::vector<std::vector<std::string>>> byFile;
    std::vector<std::string> order;
    for (const auto& r : rows) {
        if (r.size() < 2) continue;
        if (!byFile.count(r[1])) order.push_back(r[1]);
        byFile[r[1]].push_back(r);
    }
    for (const std::string& name : order) CheckFile(dir, name, byFile[name]);
    return order.size();
}

TEST(Jp2GdalSamples, EveryFileIsWhatGdalReads) {
    const std::string dir = std::string(DDBT_TEST_FIXTURES) + "/jp2";
    EXPECT_GE(CheckFolder(dir, dir + "/expected.tsv"), 20u);
}

TEST(Jp2GdalSamples, FilesFromTheEnvironment) {
    const char* dir = std::getenv("DDBT_JP2_PARITY_DIR");
    const char* tsv = std::getenv("DDBT_JP2_PARITY_TSV");
    if (dir == nullptr || tsv == nullptr) GTEST_SKIP() << "DDBT_JP2_PARITY_DIR / DDBT_JP2_PARITY_TSV not set";
    EXPECT_GT(CheckFolder(dir, tsv), 0u);
}

}  // namespace
}  // namespace ddbt::cog::build
