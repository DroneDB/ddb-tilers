// SPDX-License-Identifier: MPL-2.0
#pragma once

// The parity check of the raster readers: a folder of files and what GDAL reads in each, as the
// tab-separated lines of tests/fixtures/tools/gdal_expected (built against the GDAL libddb
// ships), compared with what OpenRasterSource reads: the size, band type, every band's samples
// (CRC-32, whole or on windows), colour interpretations, NBITS, nodata, the per-dataset mask, the
// colour table, the geotransform or GCPs, the CRS, RPC presence, and the files GDAL refuses. A
// band with a sample grid (a lossy file decoded in floating point, JPEG 2000's 9/7 wavelet, whose
// rounding depends on the build of its library: MSVC's OpenJPEG decodes 1 level apart in a few
// pixels) has its CRC checked on Linux only, where the expectations are made, and its grid within
// 1 of GDAL's everywhere.

#include <gtest/gtest.h>

#include <libdeflate.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "cog/build/source.h"
#include "core/crs/crs.h"

namespace ddbt::test {

/// An extra check of each opened file (e.g. which reader opened it).
using SourceCheck = std::function<void(const cog::build::RasterSource&)>;

namespace parity_detail {

inline std::vector<std::vector<std::string>> ReadTsv(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
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

inline const char* TypeName(cog::tiff::SampleType t) {
    using cog::tiff::SampleType;
    switch (t) {
        case SampleType::kU8: return "Byte";
        case SampleType::kI8: return "Int8";
        case SampleType::kI16: return "Int16";
        case SampleType::kU16: return "UInt16";
        case SampleType::kI32: return "Int32";
        case SampleType::kU32: return "UInt32";
        case SampleType::kF32: return "Float32";
        case SampleType::kF64: return "Float64";
        default: return "?";
    }
}

/// Band b (or the mask, b == -1) of [x, x + w) x [y, y + h) of the source, rows of w samples.
inline std::vector<uint8_t> ReadBand(const cog::build::RasterSource& s, int b, uint32_t x, uint32_t y, uint32_t w,
                                     uint32_t h) {
    const bool mask = b < 0;
    const size_t ss = mask ? 1 : s.SampleSize(), pb = mask ? 1 : s.PixelBytes();
    std::vector<uint8_t> out(static_cast<size_t>(w) * h * ss);
    const uint32_t bw = s.BlockWidth(), bh = s.BlockHeight();
    std::vector<uint8_t> block(static_cast<size_t>(bw) * bh * pb);
    for (uint32_t by = y / bh; by <= (y + h - 1) / bh; ++by) {
        for (uint32_t bx = x / bw; bx <= (x + w - 1) / bw; ++bx) {
            uint32_t rows = std::min(bh, s.Height() - by * bh);
            if (mask) {
                const Status st = s.ReadMaskBlock(bx, by, block.data());
                EXPECT_TRUE(st.ok()) << st.message();
                if (!st.ok()) return {};
            } else {
                auto r = s.ReadBlock(bx, by, block.data());
                EXPECT_TRUE(r.ok()) << r.status().message();
                if (!r.ok()) return {};
                rows = *r;
            }
            const uint32_t x0 = std::max(x, bx * bw), x1 = std::min(x + w, (bx + 1) * bw);
            const uint32_t y0 = std::max(y, by * bh), y1 = std::min(y + h, by * bh + rows);
            for (uint32_t yy = y0; yy < y1; ++yy)
                for (uint32_t xx = x0; xx < x1; ++xx)
                    std::memcpy(out.data() + (static_cast<size_t>(yy - y) * w + (xx - x)) * ss,
                                block.data() + (static_cast<size_t>(yy - by * bh) * bw + (xx - bx * bw)) * pb +
                                    (mask ? 0 : static_cast<size_t>(b) * ss),
                                ss);
        }
    }
    return out;
}

inline std::string Crc(const std::vector<uint8_t>& v) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%08x", libdeflate_crc32(0, v.data(), v.size()));
    return buf;
}

inline double D(const std::string& s) { return std::strtod(s.c_str(), nullptr); }

// Sample i of a band read by ReadBand, as a number.
inline double SampleAt(const std::vector<uint8_t>& v, cog::tiff::SampleType t, size_t i) {
    using cog::tiff::SampleType;
    const auto get = [&](auto tag) {
        decltype(tag) x;
        std::memcpy(&x, v.data() + i * sizeof(x), sizeof(x));
        return static_cast<double>(x);
    };
    switch (t) {
        case SampleType::kU8: return get(uint8_t{});
        case SampleType::kI8: return get(int8_t{});
        case SampleType::kI16: return get(int16_t{});
        case SampleType::kU16: return get(uint16_t{});
        case SampleType::kI32: return get(int32_t{});
        case SampleType::kU32: return get(uint32_t{});
        case SampleType::kF32: return get(float{});
        case SampleType::kF64: return get(double{});
        default: ADD_FAILURE() << "sample type"; return 0;
    }
}

inline std::vector<uint8_t> FromHex(const std::string& hex) {
    std::vector<uint8_t> out(hex.size() / 2);
    for (size_t i = 0; i < out.size(); ++i) out[i] = static_cast<uint8_t>(std::stoul(hex.substr(2 * i, 2), nullptr, 16));
    return out;
}

// The samples of `band` (width x height) at every step-th column and row are within 1 of `expected`.
inline void ExpectGridWithinOne(const std::vector<uint8_t>& band, uint32_t width, uint32_t height,
                                cog::tiff::SampleType t, uint32_t step, const std::vector<uint8_t>& expected) {
    size_t k = 0, off = 0;
    double worst = 0;
    for (uint32_t y = 0; y < height; y += step) {
        for (uint32_t x = 0; x < width; x += step, ++k) {
            const double d = std::abs(SampleAt(band, t, static_cast<size_t>(y) * width + x) - SampleAt(expected, t, k));
            if (d > 1) {
                ++off;
                worst = std::max(worst, d);
            }
        }
    }
    EXPECT_EQ(k * cog::tiff::SampleTypeSize(t), expected.size()) << "grid size";
    EXPECT_EQ(off, 0u) << "samples more than 1 apart (worst " << worst << ")";
}

// gdal_expected's escapes of a field: \\, \n, \t.
inline std::string Unescape(const std::string& s) {
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
inline void CheckFile(const std::string& dir, const std::string& name, const std::vector<std::vector<std::string>>& lines,
                      const SourceCheck& extra) {
    SCOPED_TRACE(name);
    auto src = cog::build::OpenRasterSource(dir + "/" + name);
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
    const cog::build::RasterSource& s = **src;
    if (extra) extra(s);
    std::vector<std::array<double, 5>> gcps;
    bool hasGt = false, hasCrs = false, rpc = false, mask = false;
    std::optional<std::string> nodata;
    // The sample grids of the bands of a lossy file, by band number.
    std::map<std::string, const std::vector<std::string>*> grids;
    for (const auto& l : lines)
        if (l[0] == "grid") grids[l[2]] = &l;
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
            const auto nbits = s.Nbits(b);
            EXPECT_EQ(nbits ? std::to_string(*nbits) : "-", l[4]);
            if (l[5] != "-") {
                const auto v = ReadBand(s, b, 0, 0, s.Width(), s.Height());
                const auto grid = grids.find(l[2]);
                if (grid == grids.end()) {
                    EXPECT_EQ(Crc(v), l[5]);
                } else {
#if defined(__linux__)
                    EXPECT_EQ(Crc(v), l[5]);
#endif
                    const auto& g = *grid->second;
                    ExpectGridWithinOne(v, s.Width(), s.Height(), s.Type(), static_cast<uint32_t>(std::stoul(g[3])),
                                        FromHex(g[4]));
                }
            }
        } else if (kind == "window") {
            SCOPED_TRACE("window " + l[3] + "," + l[4]);
            const auto v = ReadBand(s, std::stoi(l[2]) - 1, static_cast<uint32_t>(std::stoul(l[3])),
                                    static_cast<uint32_t>(std::stoul(l[4])), static_cast<uint32_t>(std::stoul(l[5])),
                                    static_cast<uint32_t>(std::stoul(l[6])));
            EXPECT_EQ(Crc(v), l[7]);
        } else if (kind == "nodata") {
            // One nodata value for every band.
            if (nodata) {
                EXPECT_EQ(*nodata, l[3]) << "band " << l[2];
            }
            nodata = l[3];
            ASSERT_TRUE(s.NoData().has_value());
            EXPECT_EQ(*s.NoData(), D(l[3]));
        } else if (kind == "mask") {
            mask = true;
            ASSERT_TRUE(s.HasMask());
            EXPECT_EQ(Crc(ReadBand(s, -1, 0, 0, s.Width(), s.Height())), l[2]);
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
    EXPECT_EQ(s.NoData().has_value(), nodata.has_value());
    EXPECT_EQ(s.HasMask(), mask);
    EXPECT_EQ(s.Georef().geoTransform.has_value(), hasGt);
    EXPECT_EQ(s.Georef().crs.has_value(), hasCrs);
    EXPECT_EQ(s.HasRpc(), rpc);
    ASSERT_EQ(s.Georef().gcps.size(), gcps.size());
    for (size_t i = 0; i < gcps.size(); ++i) {
        const auto& p = s.Georef().gcps[i];
        EXPECT_EQ((std::array<double, 5>{p.pixel, p.line, p.x, p.y, p.z}), gcps[i]) << "gcp " << i;
    }
}

}  // namespace parity_detail

/// Checks every expectation of `tsv` on the files of `dir`; returns the number of files checked.
inline size_t CheckGdalParity(const std::string& dir, const std::string& tsv, const SourceCheck& extra = {}) {
    const auto rows = parity_detail::ReadTsv(tsv);
    EXPECT_FALSE(rows.empty()) << tsv;
    std::map<std::string, std::vector<std::vector<std::string>>> byFile;
    std::vector<std::string> order;
    for (const auto& r : rows) {
        if (r.size() < 2) continue;
        if (!byFile.count(r[1])) order.push_back(r[1]);
        byFile[r[1]].push_back(r);
    }
    for (const std::string& name : order) parity_detail::CheckFile(dir, name, byFile[name], extra);
    return order.size();
}

}  // namespace ddbt::test
