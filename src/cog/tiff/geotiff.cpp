// SPDX-License-Identifier: MPL-2.0
//
// GeoKeys, geotransform / GCPs (GDAL's GTiffDataset::LoadGeoreferencingAndPamIfNeeded), world
// files and .prj sidecars. The keys -> CRS conversion is in geotiff_crs.cpp.

#include "cog/tiff/geotiff.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "core/util/checked_math.h"

namespace ddbt::cog::tiff {

std::optional<int> GeoKeyDirectory::Short(uint16_t id) const {
    auto it = keys.find(id);
    if (it == keys.end()) return std::nullopt;
    if (!it->second.shorts.empty()) return it->second.shorts[0];
    if (!it->second.doubles.empty() && std::isfinite(it->second.doubles[0]) && std::fabs(it->second.doubles[0]) < 1e9) {
        return static_cast<int>(it->second.doubles[0]);
    }
    return std::nullopt;
}

std::optional<double> GeoKeyDirectory::Double(uint16_t id, size_t index) const {
    auto it = keys.find(id);
    if (it == keys.end()) return std::nullopt;
    if (index < it->second.doubles.size()) return it->second.doubles[index];
    if (index < it->second.shorts.size()) return static_cast<double>(it->second.shorts[index]);
    return std::nullopt;
}

std::vector<double> GeoKeyDirectory::Doubles(uint16_t id) const {
    auto it = keys.find(id);
    if (it == keys.end()) return {};
    if (!it->second.doubles.empty()) return it->second.doubles;
    std::vector<double> v;
    for (uint16_t s : it->second.shorts) v.push_back(s);
    return v;
}

std::optional<std::string> GeoKeyDirectory::Ascii(uint16_t id) const {
    auto it = keys.find(id);
    if (it == keys.end() || it->second.location != tag::kGeoAsciiParams) return std::nullopt;
    return it->second.ascii;
}

Expected<GeoKeyDirectory> ReadGeoKeys(const TiffReader& reader, uint32_t ifd) {
    GeoKeyDirectory dir;
    const IfdInfo& info = reader.Ifd(ifd);
    const TagEntry* dirTag = info.FindTag(tag::kGeoKeyDirectory);
    if (dirTag == nullptr) return dir;
    auto shortsOr = reader.ReadTagUInts(*dirTag, 65536 * 4 + 4);
    if (!shortsOr.ok()) return shortsOr.status().WithContext("GeoKeyDirectory");
    const std::vector<uint64_t>& s = *shortsOr;
    if (s.size() < 4) return Corrupt("GeoKeyDirectory: shorter than its header");
    dir.version = static_cast<uint16_t>(s[0]);
    dir.revision = static_cast<uint16_t>(s[1]);
    dir.minorRevision = static_cast<uint16_t>(s[2]);
    const uint64_t n = s[3];
    if (n > (s.size() - 4) / 4) return Corrupt("GeoKeyDirectory: more keys than entries");

    std::vector<double> doubles;
    if (const TagEntry* e = info.FindTag(tag::kGeoDoubleParams)) {
        auto d = reader.ReadTagDoubles(*e, 1 << 20);
        if (d.ok()) doubles = std::move(*d);
    }
    std::string ascii;
    if (const TagEntry* e = info.FindTag(tag::kGeoAsciiParams)) {
        auto b = reader.ReadTagBytes(*e, 1 << 20);
        if (b.ok()) ascii.assign(b->begin(), b->end());
    }
    for (uint64_t k = 0; k < n; ++k) {
        const uint64_t* e = s.data() + 4 + k * 4;
        GeoKey key;
        key.id = static_cast<uint16_t>(e[0]);
        key.location = static_cast<uint16_t>(e[1]);
        const uint64_t count = e[2];
        const uint64_t off = e[3];
        if (key.location == 0) {
            key.shorts.push_back(static_cast<uint16_t>(off));
        } else if (key.location == tag::kGeoKeyDirectory) {
            if (!RangeInside(off, count, s.size())) continue;
            for (uint64_t i = 0; i < count; ++i) key.shorts.push_back(static_cast<uint16_t>(s[off + i]));
        } else if (key.location == tag::kGeoDoubleParams) {
            if (!RangeInside(off, count, doubles.size())) continue;
            key.doubles.assign(doubles.begin() + static_cast<ptrdiff_t>(off), doubles.begin() + static_cast<ptrdiff_t>(off + count));
        } else if (key.location == tag::kGeoAsciiParams) {
            if (!RangeInside(off, count, ascii.size())) continue;
            std::string v = ascii.substr(static_cast<size_t>(off), static_cast<size_t>(count));
            // libgeotiff: the '|' terminator (or a NUL) ends the value.
            if (!v.empty() && (v.back() == '|' || v.back() == '\0')) v.pop_back();
            const size_t nul = v.find('\0');
            if (nul != std::string::npos) v.resize(nul);
            key.ascii = std::move(v);
        } else {
            continue;  // unknown location: ignored like libgeotiff
        }
        dir.keys.emplace(key.id, std::move(key));  // first occurrence wins
    }
    return dir;
}

// ---------------------------------------------------------------------------------------------
// World files and .prj

std::optional<std::array<double, 6>> ParseWorldFile(std::string_view text) {
    // GDALLoadWorldFile: the first six non-empty lines, A D B E C F (C, F: centre of the
    // upper-left pixel); A and E must not be zero.
    std::array<double, 6> v{};
    size_t got = 0;
    size_t pos = 0;
    while (got < 6 && pos < text.size()) {
        size_t end = text.find_first_of("\r\n", pos);
        if (end == std::string_view::npos) end = text.size();
        std::string line(text.substr(pos, end - pos));
        pos = end + 1;
        const size_t first = line.find_first_not_of(" \t");
        if (first == std::string::npos) continue;
        char* stop = nullptr;
        v[got++] = std::strtod(line.c_str() + first, &stop);
        if (stop == line.c_str() + first) return std::nullopt;
    }
    if (got < 6 || v[0] == 0.0 || v[3] == 0.0) return std::nullopt;
    std::array<double, 6> gt{};
    gt[1] = v[0];
    gt[4] = v[1];
    gt[2] = v[2];
    gt[5] = v[3];
    gt[0] = v[4] - 0.5 * v[0] - 0.5 * v[2];
    gt[3] = v[5] - 0.5 * v[1] - 0.5 * v[3];
    return gt;
}

namespace {

std::optional<std::string> ReadSmallFile(const std::filesystem::path& p, size_t limit) {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(p, ec)) return std::nullopt;
    const auto size = std::filesystem::file_size(p, ec);
    if (ec || size > limit) return std::nullopt;
    std::ifstream f(p, std::ios::binary);
    if (!f) return std::nullopt;
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::filesystem::path FindCaseInsensitive(const std::filesystem::path& p) {
    std::error_code ec;
    if (std::filesystem::exists(p, ec)) return p;
    // Same name with an upper-case extension (GDAL also tries both cases).
    std::string ext = p.extension().string();
    for (char& c : ext) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    std::filesystem::path q = p;
    q.replace_extension(ext);
    if (std::filesystem::exists(q, ec)) return q;
    return {};
}

}  // namespace

std::string FindWorldFile(std::string_view rasterPath) {
    if (rasterPath.empty()) return {};
    const std::filesystem::path p{std::string(rasterPath)};
    std::string ext = p.extension().string();
    if (!ext.empty() && ext[0] == '.') ext.erase(0, 1);
    std::vector<std::string> candidates;
    if (ext.size() >= 2) {
        // GDALReadWorldFile2 with no extension: first + last letter + 'w' (tif -> tfw), then
        // the extension + 'w' (tif -> tifw); then "wld".
        candidates.push_back(std::string{ext.front(), ext.back(), 'w'});
        candidates.push_back(ext + "w");
    }
    candidates.push_back("wld");
    for (const std::string& c : candidates) {
        std::string lc = "." + c;
        for (char& ch : lc) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        std::filesystem::path q = p;
        q.replace_extension(lc);
        auto found = FindCaseInsensitive(q);
        if (!found.empty()) return found.string();
    }
    return {};
}

Expected<crs::Crs> ReadPrjFile(std::string_view prjPath) {
    auto text = ReadSmallFile(std::filesystem::path(std::string(prjPath)), 1 << 20);
    if (!text) return NotFound("cannot read " + std::string(prjPath));
    try {
        return crs::Crs::FromWkt(*text);
    } catch (const std::exception& e) {
        try {
            return crs::Crs::FromUserInput(*text);
        } catch (const std::exception&) {
            return Corrupt(std::string(prjPath) + ": " + e.what());
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Georeferencing

namespace {

// The GeoKeys of the main image, and whether it is PixelIsPoint.
Georeferencing KeysOf(const TiffReader& reader) {
    Georeferencing g;
    auto keys = ReadGeoKeys(reader, reader.MainImage());
    if (keys.ok()) g.keys = std::move(*keys);
    else g.crsError = keys.status().ToString();
    if (auto rt = g.keys.Short(geokey::kGTRasterType); rt && *rt == 2) g.pixelIsPoint = true;
    return g;
}

// A DOUBLE tag of the main image (empty when absent or unreadable).
std::vector<double> DoublesTag(const TiffReader& reader, uint16_t tag) {
    const TagEntry* e = reader.Ifd(reader.MainImage()).FindTag(tag);
    if (e == nullptr) return {};
    auto v = reader.ReadTagDoubles(*e, 1 << 22);
    return v.ok() ? std::move(*v) : std::vector<double>{};
}

// GDAL's shift of a PixelIsPoint geotransform to the corner of the first pixel.
void ShiftPixelIsPoint(std::array<double, 6>& gt) {
    gt[0] -= gt[1] * 0.5 + gt[2] * 0.5;
    gt[3] -= gt[4] * 0.5 + gt[5] * 0.5;
}

// The CRS the GeoKeys describe (GTIFGetDefn + GTIFGetOGISDefnAsOSR).
void ResolveCrs(Georeferencing& g) {
    if (g.keys.empty()) return;
    auto def = GeoKeysToCrsDefinition(g.keys);
    if (!def.ok()) {
        g.crsError = def.status().ToString();
    } else if (!def->empty()) {
        g.crsDefinition = *def;
        try {
            if (def->rfind("EPSG:", 0) == 0) g.crs = crs::Crs::FromUserInput(*def);
            else g.crs = crs::Crs::FromWkt(*def);
        } catch (const std::exception& e) {
            g.crsError = e.what();
        }
    }
}

}  // namespace

Expected<Georeferencing> ReadGeoreferencing(const TiffReader& reader, std::string_view path) {
    Georeferencing g = KeysOf(reader);
    const std::vector<double> scale = DoublesTag(reader, tag::kModelPixelScale);
    const std::vector<double> ties = DoublesTag(reader, tag::kModelTiepoint);
    const std::vector<double> matrix = DoublesTag(reader, tag::kModelTransformation);

    std::array<double, 6> gt{0, 1, 0, 0, 0, 1};
    if (scale.size() >= 2 && scale[0] != 0.0 && scale[1] != 0.0) {
        gt = {0, 0, 0, 0, 0, 0};
        gt[1] = scale[0];
        // GDAL assumes north-up files even with a negative ScaleY (GTIFF_HONOUR_NEGATIVE_SCALEY
        // unset).
        gt[5] = scale[1] < 0 ? scale[1] : -scale[1];
        if (ties.size() >= 6) {
            gt[0] = ties[3] - ties[0] * gt[1];
            gt[3] = ties[4] - ties[1] * gt[5];
            if (g.pixelIsPoint) ShiftPixelIsPoint(gt);
            g.geoTransform = gt;
            g.source = GeoTransformSource::kInternal;
        }
    } else if (matrix.size() == 16) {
        gt = {matrix[3], matrix[0], matrix[1], matrix[7], matrix[4], matrix[5]};
        if (g.pixelIsPoint) ShiftPixelIsPoint(gt);
        g.geoTransform = gt;
        g.source = GeoTransformSource::kInternal;
    }
    if (!g.geoTransform && !path.empty()) {
        if (auto w = ReadWorldFile(path)) {
            g.geoTransform = w->first;
            g.source = GeoTransformSource::kWorldFile;
            g.worldFile = w->second;
        }
    }
    if (!g.geoTransform && ties.size() >= 6) {
        for (size_t i = 0; i + 6 <= ties.size(); i += 6) {
            GroundControlPoint p{ties[i], ties[i + 1], ties[i + 3], ties[i + 4], ties[i + 5]};
            if (g.pixelIsPoint) {
                p.pixel += 0.5;
                p.line += 0.5;
            }
            g.gcps.push_back(p);
        }
    }
    ResolveCrs(g);
    return g;
}

Georeferencing ReadEmbeddedGeoreferencing(const TiffReader& reader) {
    Georeferencing g = KeysOf(reader);
    const std::vector<double> scale = DoublesTag(reader, tag::kModelPixelScale);
    const std::vector<double> ties = DoublesTag(reader, tag::kModelTiepoint);
    const std::vector<double> matrix = DoublesTag(reader, tag::kModelTransformation);
    constexpr std::array<double, 6> kDefault{0, 1, 0, 0, 0, 1};
    std::array<double, 6> gt = kDefault;
    if (scale.size() >= 2) {
        gt[1] = scale[0];
        gt[5] = -std::abs(scale[1]);
        if (ties.size() >= 6) {
            gt[0] = ties[3] - ties[0] * gt[1];
            gt[3] = ties[4] - ties[1] * gt[5];
            if (g.pixelIsPoint) ShiftPixelIsPoint(gt);
        }
    } else if (ties.size() >= 6) {
        for (size_t i = 0; i + 6 <= ties.size(); i += 6)
            g.gcps.push_back(GroundControlPoint{ties[i], ties[i + 1], ties[i + 3], ties[i + 4], ties[i + 5]});
    } else if (matrix.size() == 16) {
        gt = {matrix[3], matrix[0], matrix[1], matrix[7], matrix[4], matrix[5]};
    }
    // A geotransform is reported when it is not the default one.
    if (gt != kDefault) {
        g.geoTransform = gt;
        g.source = GeoTransformSource::kInternal;
    }
    ResolveCrs(g);
    return g;
}

std::optional<std::pair<std::array<double, 6>, std::string>> ReadWorldFile(std::string_view rasterPath) {
    const std::string wf = FindWorldFile(rasterPath);
    if (wf.empty()) return std::nullopt;
    auto text = ReadSmallFile(wf, 1 << 16);
    if (!text) return std::nullopt;
    auto w = ParseWorldFile(*text);
    if (!w) return std::nullopt;
    return std::make_pair(*w, wf);
}

}  // namespace ddbt::cog::tiff
