// SPDX-License-Identifier: MPL-2.0
#include "cog/build/source.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "cog/build/gif_source.h"
#include "cog/build/jp2_source.h"
#include "cog/build/jpeg_source.h"
#include "cog/build/png_source.h"
#include "cog/build/tiff_source.h"
#include "cog/build/webp_source.h"
#include "cog/jp2/jp2_file.h"
#include "core/io/byte_source.h"
#include "core/util/xml_lite.h"

namespace ddbt::cog::build {

Expected<std::unique_ptr<RasterSource>> OpenRasterSource(const std::string& path, uint64_t memoryBudget) {
    // The reader of the format the first bytes tell, as GDAL's drivers identify them.
    auto open = [&](auto opener) -> Expected<std::unique_ptr<RasterSource>> {
        auto s = opener(path, memoryBudget);
        if (!s.ok()) return s.status();
        return std::unique_ptr<RasterSource>(std::move(*s));
    };
    {
        auto bytes = io::OpenByteSource(path, io::SourceKind::kPread);
        if (!bytes.ok()) return bytes.status();
        const io::ByteSource& b = **bytes;
        if (jp2::IsJpeg2000(b)) return open(Jp2Source::Open);
        if (IsJpegFile(b)) return open(JpegSource::Open);
        if (IsPngFile(b)) return open(PngSource::Open);
        if (IsGifFile(b)) return open(GifSource::Open);
        if (IsWebpFile(b)) return open(WebpSource::Open);
    }
    return open([](const std::string& p, uint64_t) { return TiffSource::Open(p); });
}

writer::RawTag AsciiTag(uint16_t tag, const std::string& value) {
    writer::RawTag t{tag, 2, value.size() + 1, {}};
    t.data.assign(value.begin(), value.end());
    t.data.push_back(0);
    return t;
}

writer::RawTag ShortTag(uint16_t tag, uint16_t value) {
    return writer::RawTag{tag, 3, 1, {static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8)}};
}

namespace {

// libtiff 4.7's DoubleToRational and ToRationalEuclideanGCD (libtiff/tif_dirwrite.c), the rational
// libtiff writes for a double: the best of two continued-fraction approximations. Their notice:
//
//   Copyright (c) 1988-1997 Sam Leffler
//   Copyright (c) 1991-1997 Silicon Graphics, Inc.
//
//   Permission to use, copy, modify, distribute, and sell this software and its documentation for
//   any purpose is hereby granted without fee, provided that (i) the above copyright notices and
//   this permission notice appear in all copies of the software and related documentation, and
//   (ii) the names of Sam Leffler and Silicon Graphics may not be used in any advertising or
//   publicity relating to the software without the specific, prior written permission of Sam
//   Leffler and Silicon Graphics.
//
//   THE SOFTWARE IS PROVIDED "AS-IS" AND WITHOUT WARRANTY OF ANY KIND, EXPRESS, IMPLIED OR
//   OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY OR FITNESS FOR A
//   PARTICULAR PURPOSE.
//
//   IN NO EVENT SHALL SAM LEFFLER OR SILICON GRAPHICS BE LIABLE FOR ANY SPECIAL, INCIDENTAL,
//   INDIRECT OR CONSEQUENTIAL DAMAGES OF ANY KIND, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
//   OF USE, DATA OR PROFITS, WHETHER OR NOT ADVISED OF THE POSSIBILITY OF DAMAGE, AND ON ANY
//   THEORY OF LIABILITY, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
//   SOFTWARE.
void ToRationalEuclideanGcd(double value, bool smallRange, uint64_t& num, uint64_t& den) {
    uint64_t numSum[3] = {0, 1, 0}, denomSum[3] = {1, 0, 0};
    const uint64_t nMax = smallRange ? uint64_t{(2147483647 - 1) / 2} : uint64_t{(9223372036854775807 - 1) / 2};
    const double fMax = static_cast<double>(nMax);
    constexpr uint64_t kMaxDenom = 0xFFFFFFFFu, kReturnLimit = 0xFFFFFFFFu;
    uint64_t bigDenom = 1;
    while (value != std::floor(value) && value < fMax && bigDenom < nMax) {
        bigDenom <<= 1;
        value *= 2;
    }
    uint64_t bigNum = static_cast<uint64_t>(value);
    constexpr int kMaxIterations = 64;
    for (int i = 0; i < kMaxIterations; ++i) {
        if (bigDenom == 0) break;
        const uint64_t val = bigNum / bigDenom;
        uint64_t aux = bigNum;
        bigNum = bigDenom;
        bigDenom = aux % bigDenom;
        aux = val;
        if (denomSum[1] * val + denomSum[0] >= kMaxDenom) {
            aux = (kMaxDenom - denomSum[0]) / denomSum[1];
            if (aux * 2 >= val || denomSum[1] >= kMaxDenom) i = kMaxIterations + 1;
            else break;
        }
        numSum[2] = aux * numSum[1] + numSum[0];
        numSum[0] = numSum[1];
        numSum[1] = numSum[2];
        denomSum[2] = aux * denomSum[1] + denomSum[0];
        denomSum[0] = denomSum[1];
        denomSum[1] = denomSum[2];
    }
    while (numSum[1] > kReturnLimit || denomSum[1] > kReturnLimit) {
        numSum[1] /= 2;
        denomSum[1] /= 2;
    }
    num = numSum[1];
    den = denomSum[1];
}

void DoubleToRational(double value, uint32_t& num, uint32_t& den) {
    if (!(value >= 0)) {
        num = den = 0;
        return;
    }
    if (value > 4294967295.0) {
        num = 0xFFFFFFFFu;
        den = 0;
        return;
    }
    if (value == static_cast<double>(static_cast<uint32_t>(value))) {
        num = static_cast<uint32_t>(value);
        den = 1;
        return;
    }
    if (value < 1.0 / 4294967295.0) {
        num = 0;
        den = 0xFFFFFFFFu;
        return;
    }
    uint64_t n1, d1, n2, d2;
    ToRationalEuclideanGcd(value, false, n1, d1);
    ToRationalEuclideanGcd(value, true, n2, d2);
    const double diff1 = std::fabs(value - static_cast<double>(n1) / static_cast<double>(d1));
    const double diff2 = std::fabs(value - static_cast<double>(n2) / static_cast<double>(d2));
    num = static_cast<uint32_t>(diff1 < diff2 ? n1 : n2);
    den = static_cast<uint32_t>(diff1 < diff2 ? d1 : d2);
}

}  // namespace

double GdalFloatTag(double value) {
    char text[32];
    std::snprintf(text, sizeof(text), "%.8g", static_cast<double>(static_cast<float>(value)));
    return std::strtod(text, nullptr);
}

writer::RawTag RationalTag(uint16_t tag, double value) {
    // libtiff keeps X / YResolution as floats and writes them as the rational of that float.
    uint32_t num = 0, den = 1;
    DoubleToRational(static_cast<double>(static_cast<float>(value)), num, den);
    writer::RawTag t{tag, 5, 1, std::vector<uint8_t>(8)};
    for (int i = 0; i < 4; ++i) {
        t.data[static_cast<size_t>(i)] = static_cast<uint8_t>(num >> (8 * i));
        t.data[static_cast<size_t>(4 + i)] = static_cast<uint8_t>(den >> (8 * i));
    }
    return t;
}

void GtiffMetadata(const std::vector<std::pair<std::string, std::string>>& items, std::vector<writer::RawTag>& tags,
                   std::string& gdalMetadata) {
    // GTiffDataset's asTIFFTags (the DGIWG GEO_METADATA / TIFF_RSID are left out).
    enum class Kind : uint8_t { kString, kFloat, kShort, kUnit };
    struct Known {
        const char* name;
        uint16_t tag;
        Kind kind;
    };
    static constexpr Known kKnown[] = {
        {"TIFFTAG_DOCUMENTNAME", 269, Kind::kString},    {"TIFFTAG_IMAGEDESCRIPTION", 270, Kind::kString},
        {"TIFFTAG_SOFTWARE", 305, Kind::kString},        {"TIFFTAG_DATETIME", 306, Kind::kString},
        {"TIFFTAG_ARTIST", 315, Kind::kString},          {"TIFFTAG_HOSTCOMPUTER", 316, Kind::kString},
        {"TIFFTAG_COPYRIGHT", 33432, Kind::kString},     {"TIFFTAG_XRESOLUTION", 282, Kind::kFloat},
        {"TIFFTAG_YRESOLUTION", 283, Kind::kFloat},      {"TIFFTAG_RESOLUTIONUNIT", 296, Kind::kUnit},
        {"TIFFTAG_MINSAMPLEVALUE", 280, Kind::kShort},   {"TIFFTAG_MAXSAMPLEVALUE", 281, Kind::kShort},
    };
    auto upper = [](std::string s) {
        for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        return s;
    };
    std::string doc;
    for (const auto& [name, value] : items) {
        const std::string key = upper(name);
        if (key.rfind("TIFFTAG_", 0) == 0) {
            for (const Known& k : kKnown) {
                if (key != k.name) continue;
                switch (k.kind) {
                    case Kind::kString: tags.push_back(AsciiTag(k.tag, value)); break;
                    case Kind::kFloat: tags.push_back(RationalTag(k.tag, std::strtod(value.c_str(), nullptr))); break;
                    case Kind::kShort:
                        tags.push_back(ShortTag(k.tag, static_cast<uint16_t>(std::atoi(value.c_str()))));
                        break;
                    case Kind::kUnit: {
                        // A ResolutionUnit cannot be 0 (what atoi gives for text): 1, no unit.
                        const int v = std::atoi(value.c_str());
                        tags.push_back(ShortTag(k.tag, static_cast<uint16_t>(v == 0 ? 1 : v)));
                        break;
                    }
                }
            }
            continue;
        }
        if (key == "AREA_OR_POINT") continue;
        // GDAL escapes the value, then the XML serialization escapes it again (its GDAL_METADATA
        // readers unescape twice).
        doc += "  <Item name=\"" + xml::Escape(name, true) + "\">" + xml::Escape(xml::Escape(value, true), false) +
               "</Item>\n";
    }
    gdalMetadata = doc.empty() ? std::string() : "<GDALMetadata>\n" + doc + "</GDALMetadata>\n";
}

}  // namespace ddbt::cog::build
