// SPDX-License-Identifier: MPL-2.0
#include "cog/build/source.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>

#include "cog/build/jp2_source.h"
#include "cog/build/tiff_source.h"
#include "cog/jp2/jp2_file.h"
#include "core/io/byte_source.h"
#include "core/util/xml_lite.h"

namespace ddbt::cog::build {

Expected<std::unique_ptr<RasterSource>> OpenRasterSource(const std::string& path, uint64_t memoryBudget) {
    {
        auto bytes = io::OpenByteSource(path, io::SourceKind::kPread);
        if (!bytes.ok()) return bytes.status();
        if (jp2::IsJpeg2000(**bytes)) {
            auto j = Jp2Source::Open(path, memoryBudget);
            if (!j.ok()) return j.status();
            return std::unique_ptr<RasterSource>(std::move(*j));
        }
    }
    auto t = TiffSource::Open(path);
    if (!t.ok()) return t.status();
    return std::unique_ptr<RasterSource>(std::move(*t));
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

writer::RawTag RationalTag(uint16_t tag, double value) {
    // libtiff stores the float GDAL sets as a rational: integers as n / 1.
    uint32_t num, den = 1;
    if (value >= 0 && value == std::floor(value) && value < 4294967296.0) {
        num = static_cast<uint32_t>(value);
    } else {
        den = 1000000;
        num = static_cast<uint32_t>(std::max(0.0, std::min(4294967295.0, std::round(value * den))));
    }
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
        doc += "  <Item name=\"" + xml::Escape(name, true) + "\">" + xml::Escape(value, false) + "</Item>\n";
    }
    gdalMetadata = doc.empty() ? std::string() : "<GDALMetadata>\n" + doc + "</GDALMetadata>\n";
}

}  // namespace ddbt::cog::build
