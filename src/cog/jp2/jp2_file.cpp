// SPDX-License-Identifier: MPL-2.0
//
// Ported from GDAL 3.13.3, MIT licence: the box reader of gcore/gdaljp2box.cpp (GDALJP2Box), the
// box collection and georeferencing of gcore/gdaljp2metadata.cpp (ReadBoxes, CollectGMLData,
// ParseJP2GeoTIFF, ParseGMLCoverageDesc, ParseMSIG) and their priority in
// gcore/gdaljp2abstractdataset.cpp (LoadJP2Metadata), the colour boxes of
// frmts/opjlike/jp2opjlikedataset.cpp (JP2OPJLikeDataset::Open), and OGRFastAtof of
// ogr/ogrutils.cpp. The copyright notices and the permission notice of the ported code:
//
//   gcore/gdaljp2box.cpp, gcore/gdaljp2metadata.cpp, gcore/gdaljp2abstractdataset.cpp:
//     Copyright (c) 2002, 2005, Frank Warmerdam <warmerdam@pobox.com>
//     Copyright (c) 2010-2015, Even Rouault <even dot rouault at spatialys dot com>
//     Copyright (c) 2015, European Union Satellite Centre
//   frmts/opjlike/jp2opjlikedataset.cpp:
//     Copyright (c) 2010-2014, Even Rouault <even dot rouault at spatialys dot com>
//     Copyright (c) 2015, European Union (European Environment Agency)
//     Copyright (c) 2023, Grok Image Compression Inc.
//   ogr/ogrutils.cpp:
//     Copyright (c) 1999, Frank Warmerdam
//     Copyright (c) 2008-2014, Even Rouault <even dot rouault at spatialys.com>
//
//   Permission is hereby granted, free of charge, to any person obtaining a copy of this
//   software and associated documentation files (the "Software"), to deal in the Software
//   without restriction, including without limitation the rights to use, copy, modify, merge,
//   publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons
//   to whom the Software is furnished to do so, subject to the following conditions:
//
//   The above copyright notice and this permission notice shall be included in all copies or
//   substantial portions of the Software.
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//   INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
//   PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
//   FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
//   OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//   DEALINGS IN THE SOFTWARE.
#include "cog/jp2/jp2_file.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <limits>
#include <memory>

#include "cog/tiff/tiff_reader.h"
#include "core/util/xml_lite.h"

namespace ddbt::cog::jp2 {

namespace {

constexpr uint64_t kMaxBoxData = uint64_t{100} << 20;  // GDALJP2Box::ReadBoxData's limit
constexpr size_t kMaxBoxes = 1 << 16;
constexpr size_t kMaxGeoTiffBoxes = 2;  // MAX_JP2GEOTIFF_BOXES

// UUIDs of the boxes GDAL reads.
constexpr std::array<uint8_t, 16> kGeoTiffUuid{0xb1, 0x4b, 0xf8, 0xbd, 0x08, 0x3d, 0x4b, 0x43,
                                              0xa5, 0xae, 0x8c, 0xd7, 0xd5, 0xa6, 0xce, 0x03};
constexpr std::array<uint8_t, 16> kMsigUuid{0x96, 0xa9, 0xf1, 0xf1, 0xdc, 0x98, 0x40, 0x2d,
                                           0xa7, 0xae, 0xd6, 0x8e, 0x34, 0x45, 0x18, 0x09};

bool EqualNoCase(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i)
        if (std::tolower(static_cast<unsigned char>(a[i])) != std::tolower(static_cast<unsigned char>(b[i])))
            return false;
    return true;
}

bool StartsWithNoCase(std::string_view s, std::string_view prefix) {
    return s.size() >= prefix.size() && EqualNoCase(s.substr(0, prefix.size()), prefix);
}

uint32_t Be32(const uint8_t* p) {
    return (uint32_t{p[0]} << 24) | (uint32_t{p[1]} << 16) | (uint32_t{p[2]} << 8) | uint32_t{p[3]};
}

// GDALJP2Box::ReadBox at `offset`; false when the header cannot be read or is invalid.
bool ReadBoxAt(const io::ByteSource& src, uint64_t offset, Box& box) {
    uint8_t h[16];
    if (offset > src.Size() || src.Size() - offset < 8 || !src.Read(offset, {h, 8}).ok()) return false;
    box.offset = offset;
    box.type.assign(reinterpret_cast<const char*>(h + 4), 4);
    const uint32_t lbox = Be32(h);
    if (lbox != 1) {
        box.length = lbox;
        box.dataOffset = offset + 8;
    } else {
        if (src.Size() - offset < 16 || !src.Read(offset + 8, {h + 8, 8}).ok()) return false;
        const uint64_t xl = (uint64_t{Be32(h + 8)} << 32) | Be32(h + 12);
        if (xl > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) return false;
        box.length = xl;
        box.dataOffset = offset + 16;
    }
    if (box.length == 0) box.length = src.Size() - offset;
    if (box.type == "uuid") {
        if (src.Size() - box.dataOffset < 16 || box.dataOffset > src.Size() ||
            !src.Read(box.dataOffset, {box.uuid.data(), 16}).ok())
            return false;
        box.dataOffset += 16;
    }
    return box.length >= box.dataOffset - box.offset;
}

// The data of a box (GDALJP2Box::ReadBoxData): empty when too big or unreadable.
std::vector<uint8_t> BoxData(const io::ByteSource& src, const Box& b) {
    const uint64_t n = b.DataLength();
    if (n > kMaxBoxData) return {};
    auto v = src.ReadVector(b.dataOffset, static_cast<size_t>(n));
    return v.ok() ? std::move(*v) : std::vector<uint8_t>{};
}

std::string BoxText(const io::ByteSource& src, const Box& b) {
    const std::vector<uint8_t> d = BoxData(src, b);
    return std::string(d.begin(), d.end());
}

// The colr / pclr / cdef boxes of a jp2h box (JP2OPJLikeDataset::Open).
void ReadHeaderBoxes(const io::ByteSource& src, const Box& jp2h, Jp2File& f) {
    for (const Box& b : Children(src, jp2h)) {
        const uint64_t n = b.DataLength();
        if (!f.palette && EqualNoCase(b.type, "pclr") && n >= 3 && n <= 2 + 1 + 4 + 4 * 256) {
            const std::vector<uint8_t> d = BoxData(src, b);
            if (d.size() != n) continue;
            const size_t entries = (size_t{d[0]} << 8) | d[1];
            const size_t components = d[2];
            if (entries > 256 || (components != 3 && components != 4)) continue;
            // 8-bit unsigned components (bit depth byte 7) only
            bool eightBit = true;
            for (size_t c = 0; c < components; ++c) eightBit = eightBit && d[3 + c] == 7;
            if (!eightBit || n != 3 + components + components * entries) continue;
            std::vector<std::array<uint8_t, 4>> table(entries);
            const size_t base = 3 + components;
            for (size_t i = 0; i < entries; ++i)
                table[i] = {d[base + components * i], d[base + components * i + 1], d[base + components * i + 2],
                            components == 4 ? d[base + components * i + 3] : uint8_t{255}};
            f.palette = std::move(table);
        } else if (EqualNoCase(b.type, "colr") && n == 7) {
            const std::vector<uint8_t> d = BoxData(src, b);
            if (d.size() != 7 || d[0] != 1) continue;  // enumerated colour space only
            switch (Be32(d.data() + 3)) {
                case 16:  // sRGB
                case 20:  // e-sRGB
                case 21:  // ROMM-RGB
                    f.colourSpace = ColourSpace::kSrgb;
                    break;
                case 17: f.colourSpace = ColourSpace::kGray; break;
                case 18: f.colourSpace = ColourSpace::kSycc; break;
                default: f.colourSpace = ColourSpace::kUnknown; break;
            }
        } else if (EqualNoCase(b.type, "cdef") && n >= 2 && n <= 2 + 6 * 16384) {
            f.channelDefinitions.push_back(BoxData(src, b));
        }
    }
}

// GDALJP2Metadata::CollectGMLData: the labelled XML documents of a gml.data association.
void CollectGmlData(const io::ByteSource& src, const Box& gmlData, Jp2File& f) {
    for (const Box& child : Children(src, gmlData)) {
        if (!EqualNoCase(child.type, "asoc")) continue;
        std::optional<std::string> label, xml;
        for (const Box& sub : Children(src, child)) {
            if (EqualNoCase(sub.type, "lbl ") && !label) {
                std::string l = BoxText(src, sub);
                l.resize(std::strlen(l.c_str()));  // a C string
                label = std::move(l);
            } else if (EqualNoCase(sub.type, "xml ") && !xml) {
                std::string x = BoxText(src, sub);
                // Some GML data holds NUL characters: trailing ones are dropped; one inside the
                // document ends it, unless the part before it is not a document, in which case
                // they become new lines.
                while (!x.empty() && x.back() == '\0') x.pop_back();
                const size_t nul = x.find('\0');
                if (nul != std::string::npos) {
                    if (xml::Parse(std::string_view(x).substr(0, nul))) {
                        x.resize(nul);
                    } else {
                        std::replace(x.begin(), x.end(), '\0', '\n');
                    }
                }
                xml = std::move(x);
            }
        }
        if (label && xml) f.labelledXml.emplace_back(*label, *xml);
    }
}

// GDALJP2Metadata::ReadBox for one top-level (or JUMBF) box.
void ReadMetadataBox(const io::ByteSource& src, const Box& b, Jp2File& f, int depth) {
    if (b.type == "uuid" && b.uuid == kGeoTiffUuid) {
        if (f.geoTiffBoxes.size() < kMaxGeoTiffBoxes) {
            std::vector<uint8_t> d = BoxData(src, b);
            if (!d.empty()) f.geoTiffBoxes.push_back(std::move(d));
        }
    } else if (b.type == "uuid" && b.uuid == kMsigUuid) {
        if (f.msig.empty()) {
            std::vector<uint8_t> d = BoxData(src, b);
            if (d.size() >= 70 && std::memcmp(d.data(), "MSIG/", 5) == 0) f.msig = std::move(d);
        }
    } else if (EqualNoCase(b.type, "asoc")) {
        const std::vector<Box> children = Children(src, b);
        if (!children.empty() && EqualNoCase(children[0].type, "lbl ")) {
            std::string label = BoxText(src, children[0]);
            label.resize(std::strlen(label.c_str()));
            if (EqualNoCase(label, "gml.data")) CollectGmlData(src, b, f);
        }
    } else if (EqualNoCase(b.type, "xml ")) {
        std::string x = BoxText(src, b);
        x.resize(std::strlen(x.c_str()));
        if (x.rfind("<GDALMultiDomainMetadata>", 0) == 0 && f.gdalMultiDomainMetadata.empty())
            f.gdalMultiDomainMetadata = std::move(x);
    } else if (EqualNoCase(b.type, "jp2h")) {
        for (const Box& sub : Children(src, b)) {
            if (!EqualNoCase(sub.type, "res ")) continue;
            const std::vector<Box> res = Children(src, sub);
            // The resc or resd box, whichever comes first.
            if (res.empty() || res[0].DataLength() != 10) continue;
            const std::vector<uint8_t> d = BoxData(src, res[0]);
            if (d.size() != 10) continue;
            const int vNum = d[0] * 256 + d[1], vDen = d[2] * 256 + d[3];
            const int hNum = d[4] * 256 + d[5], hDen = d[6] * 256 + d[7];
            const int vExp = d[8], hExp = d[9];
            // pixels per centimetre
            const double v = (vNum / static_cast<double>(vDen)) * std::pow(10.0, vExp) / 100;
            const double h = (hNum / static_cast<double>(hDen)) * std::pow(10.0, hExp) / 100;
            f.resolution = std::array<double, 2>{h, v};
        }
    } else if (EqualNoCase(b.type, "jumb") && depth < 8) {
        for (const Box& sub : Children(src, b)) ReadMetadataBox(src, sub, f, depth + 1);
    }
}

// GDAL's CPLAtof: the leading number of the text (0 when there is none).
double Atof(const std::string& s) { return std::strtod(s.c_str(), nullptr); }

// OGRFastAtof (GDAL 3.13.3, ogr/ogrutils.cpp, MIT licence: Copyright (c) 1999, Frank Warmerdam,
// Copyright (c) 2008-2014, Even Rouault), which GDAL's GML reader parses coordinates with: digits
// accumulated as a double and divided by a power of ten, which is not always the nearest double
// of the text (4895767.000000000000000 is 4895767.0000000009); exponents and long fractions go to
// CPLAtof.
double FastAtof(const char* str) {
    constexpr double kTenPower[] = {1e0,  1e1,  1e2,  1e3,  1e4,  1e5,  1e6,  1e7,  1e8,  1e9,  1e10,
                                    1e11, 1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19, 1e20, 1e21,
                                    1e22, 1e23, 1e24, 1e25, 1e26, 1e27, 1e28, 1e29, 1e30, 1e31};
    // OGRCallAtofOnShortString: the number alone, through CPLAtof.
    auto shortString = [](const char* s) {
        while (*s == ' ' || *s == '\t') ++s;
        std::string t;
        while (*s == '+' || *s == '-' || (*s >= '0' && *s <= '9') || *s == '.' || *s == 'e' || *s == 'E' ||
               *s == 'd' || *s == 'D') {
            t += *s++;
            if (t.size() == 127) return std::strtod(s, nullptr);
        }
        return std::strtod(t.c_str(), nullptr);
    };
    double value = 0, sign = 1.0;
    const char* p = str;
    while (*p == ' ' || *p == '\t') ++p;
    if (*p == '+') {
        ++p;
    } else if (*p == '-') {
        sign = -1.0;
        ++p;
    }
    for (;;) {
        if (*p >= '0' && *p <= '9') {
            value = value * 10.0 + (*p - '0');
            ++p;
        } else if (*p == '.') {
            ++p;
            break;
        } else if (*p == 'e' || *p == 'E' || *p == 'd' || *p == 'D') {
            return shortString(str);
        } else {
            return sign * value;
        }
    }
    unsigned fraction = 0;
    for (;;) {
        if (*p >= '0' && *p <= '9') {
            value = value * 10.0 + (*p - '0');
            ++fraction;
            ++p;
        } else if (*p == 'e' || *p == 'E' || *p == 'd' || *p == 'D') {
            return shortString(str);
        } else {
            if (fraction < std::size(kTenPower)) return sign * (value / kTenPower[fraction]);
            return shortString(str);
        }
    }
}

// CSLTokenizeStringComplex(text, " ,", FALSE, FALSE): non-empty tokens between spaces / commas.
std::vector<std::string> Tokens(std::string_view s) {
    std::vector<std::string> out;
    std::string cur;
    for (const char c : s) {
        if (c == ' ' || c == ',') {
            if (!cur.empty()) out.push_back(std::move(cur));
            cur.clear();
        } else {
            cur += c;
        }
    }
    if (!cur.empty()) out.push_back(std::move(cur));
    return out;
}

// The x, y of a GML Point (OGR_G_CreateFromGMLTree): pos, coordinates or coord.
std::optional<std::array<double, 2>> GmlPoint(const xml::Element& point) {
    auto numbers = [](std::string_view text, bool commas) {
        std::vector<double> v;
        std::string cur;
        auto flush = [&] {
            if (!cur.empty()) v.push_back(FastAtof(cur.c_str()));
            cur.clear();
        };
        for (const char c : text) {
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r' || (commas && c == ',')) flush();
            else cur += c;
        }
        flush();
        return v;
    };
    if (const xml::Element* pos = point.Child("pos")) {
        const auto v = numbers(pos->text, false);
        if (v.size() >= 2) return std::array<double, 2>{v[0], v[1]};
        return std::nullopt;
    }
    if (const xml::Element* c = point.Child("coordinates")) {
        const auto v = numbers(c->text, true);
        if (v.size() >= 2) return std::array<double, 2>{v[0], v[1]};
        return std::nullopt;
    }
    if (const xml::Element* c = point.Child("coord")) {
        const xml::Element* x = c->Child("X");
        const xml::Element* y = c->Child("Y");
        if (x != nullptr && y != nullptr) return std::array<double, 2>{FastAtof(x->text.c_str()), FastAtof(y->text.c_str())};
    }
    return std::nullopt;
}

// GDALJP2Metadata::ParseJP2GeoTIFF: the best of the GeoTIFF boxes.
bool ParseGeoJp2(const Jp2File& f, tiff::Georeferencing& g, bool& rpc) {
    struct Candidate {
        tiff::Georeferencing geo;
        bool rpc = false;
        bool valid = false;  // a CRS
    };
    std::vector<Candidate> c;
    for (const std::vector<uint8_t>& box : f.geoTiffBoxes) {
        Candidate k;
        auto reader = tiff::TiffReader::Open(std::make_shared<io::MemorySource>(box, "GeoJP2 box"));
        if (reader.ok()) {
            k.geo = tiff::ReadEmbeddedGeoreferencing(**reader);
            constexpr uint16_t kRpcCoefficientTag = 50844;
            const tiff::TagEntry* t = (*reader)->Ifd((*reader)->MainImage()).FindTag(kRpcCoefficientTag);
            k.rpc = t != nullptr && t->count == 92;
            k.valid = k.geo.crs.has_value();
        }
        c.push_back(std::move(k));
    }
    int best = -1;
    for (size_t i = 0; i < c.size(); ++i) {
        if (!c[i].valid) continue;
        // Anything else than a local CRS is better.
        if (best < 0 || c[static_cast<size_t>(best)].geo.crs->Type() == crs::CrsType::Engineering)
            best = static_cast<int>(i);
    }
    if (best < 0) {
        for (size_t i = 0; i < c.size(); ++i)
            if (c[i].geo.geoTransform || !c[i].geo.gcps.empty() || c[i].rpc) best = static_cast<int>(i);
    }
    if (best < 0) return false;
    g = std::move(c[static_cast<size_t>(best)].geo);
    rpc = c[static_cast<size_t>(best)].rpc;
    return true;
}

// GDALJP2Metadata::ParseGMLCoverageDesc: the RectifiedGrid of the gml.root-instance document.
// Sets the geotransform when the grid has one even when no CRS is found (and then returns false).
bool ParseGml(const Jp2File& f, tiff::Georeferencing& g) {
    const std::string* coverage = nullptr;
    for (const auto& [label, text] : f.labelledXml)
        if (EqualNoCase(label, "gml.root-instance")) {
            coverage = &text;
            break;
        }
    if (coverage == nullptr) return false;
    const std::optional<xml::Element> root = xml::Parse(*coverage);
    if (!root) return false;
    const xml::Element* rg = root->Find("RectifiedGrid");
    const xml::Element* origin = rg != nullptr ? rg->Path("origin.Point") : nullptr;
    const xml::Element* offset1 = nullptr;
    const xml::Element* offset2 = nullptr;
    if (rg != nullptr) {
        for (size_t i = 0; i < rg->children.size(); ++i) {
            if (rg->children[i].name != "offsetVector") continue;
            offset1 = &rg->children[i];
            // The second offset vector is the element right after the first.
            if (i + 1 < rg->children.size() && rg->children[i + 1].name == "offsetVector") offset2 = &rg->children[i + 1];
            break;
        }
    }
    if (origin == nullptr || offset1 == nullptr || offset2 == nullptr) return false;
    const auto point = GmlPoint(*origin);
    const std::vector<std::string> o1 = Tokens(offset1->text), o2 = Tokens(offset2->text);
    bool success = false;
    std::array<double, 6> gt{};
    if (o1.size() >= 2 && o2.size() >= 2 && point) {
        gt[0] = (*point)[0];
        gt[1] = Atof(o1[0]);
        gt[2] = Atof(o2[0]);
        gt[3] = (*point)[1];
        gt[4] = Atof(o1[1]);
        gt[5] = Atof(o2[1]);
        // From the centre of the first pixel to its corner
        gt[0] -= gt[1] * 0.5;
        gt[0] -= gt[2] * 0.5;
        gt[3] -= gt[4] * 0.5;
        gt[3] -= gt[5] * 0.5;
        success = true;
        g.geoTransform = gt;
        g.source = tiff::GeoTransformSource::kInternal;
    }
    // The CRS: the origin's srsName, else the FeatureCollection's envelope, else the grid's.
    const std::string* srsName = origin->Attribute("srsName");
    if (srsName == nullptr && root->name == "FeatureCollection")
        if (const xml::Element* env = root->Path("boundedBy.Envelope")) srsName = env->Attribute("srsName");
    if (srsName == nullptr) srsName = rg->Attribute("srsName");
    bool flip = false;
    if (success && srsName != nullptr && !g.crs) {
        const std::string& n = *srsName;
        try {
            if (StartsWithNoCase(n, "epsg:")) {
                g.crs = crs::Crs::FromUserInput(n);
            } else if ((StartsWithNoCase(n, "urn:") && n.find(":def:") != std::string::npos) ||
                       StartsWithNoCase(n, "http://www.opengis.net/def/crs/")) {
                g.crs = crs::Crs::FromUserInput(n);
                flip = g.crs->NorthingFirst();
            } else {
                // A CRS in a dictionary of the file (GML CRS definitions) is not read.
                g.crsError = "unsupported GMLJP2 srsName " + n;
            }
            if (g.crs) g.crsDefinition = n;
        } catch (const std::exception& e) {
            g.crs.reset();
            g.crsError = e.what();
        }
    }
    // Some files name their axes easting / longitude then northing / latitude explicitly, which
    // overrides the CRS's order.
    if (flip) {
        int axes = 0;
        bool eastFirst = false, northSecond = false;
        for (const xml::Element& c : rg->children) {
            if (c.name != "axisName" || c.text.empty()) continue;
            if (axes == 0 && (StartsWithNoCase(c.text, "EAST") || StartsWithNoCase(c.text, "LONG"))) eastFirst = true;
            else if (axes == 1 && (StartsWithNoCase(c.text, "NORTH") || StartsWithNoCase(c.text, "LAT"))) northSecond = true;
            ++axes;
        }
        if (eastFirst && northSecond) flip = false;
    }
    if (flip && g.geoTransform) {
        auto& t = *g.geoTransform;
        std::swap(t[0], t[3]);
        // GDAL_JP2K_ALT_OFFSETVECTOR_ORDER=TRUE in a comment of the document: the other order of
        // the offset vectors.
        if (coverage->find("GDAL_JP2K_ALT_OFFSETVECTOR_ORDER=TRUE") != std::string::npos) {
            std::swap(t[1], t[5]);
            std::swap(t[2], t[4]);
        } else {
            std::swap(t[1], t[4]);
            std::swap(t[2], t[5]);
        }
    }
    return g.crs.has_value() && success;
}

// GDALJP2Metadata::ParseMSIG: the world file of the MSIG box (little-endian doubles).
bool ParseMsig(const Jp2File& f, tiff::Georeferencing& g) {
    if (f.msig.size() < 70) return false;
    double v[6];
    for (int i = 0; i < 6; ++i) {
        uint64_t bits = 0;
        for (int k = 7; k >= 0; --k) bits = (bits << 8) | f.msig[static_cast<size_t>(22 + 8 * i + k)];
        std::memcpy(&v[i], &bits, 8);
    }
    std::array<double, 6> gt{v[4], v[0], v[2], v[5], v[1], v[3]};
    gt[0] -= 0.5 * gt[1];
    gt[0] -= 0.5 * gt[2];
    gt[3] -= 0.5 * gt[4];
    gt[3] -= 0.5 * gt[5];
    g.geoTransform = gt;
    g.source = tiff::GeoTransformSource::kInternal;
    return true;
}

}  // namespace

bool Box::IsSuperBox() const noexcept {
    return EqualNoCase(type, "asoc") || EqualNoCase(type, "jp2h") || EqualNoCase(type, "res ") ||
           EqualNoCase(type, "jumb");
}

std::vector<Box> ReadBoxes(const io::ByteSource& src, uint64_t begin, uint64_t end) {
    std::vector<Box> out;
    uint64_t offset = begin;
    Box b;
    while (out.size() < kMaxBoxes && offset < end && ReadBoxAt(src, offset, b) && b.type[0] != '\0') {
        out.push_back(b);
        if (b.length == 0) break;
        offset = b.offset + b.length;
    }
    return out;
}

std::vector<Box> Children(const io::ByteSource& src, const Box& parent) {
    if (!parent.IsSuperBox()) return {};
    return ReadBoxes(src, parent.dataOffset, parent.offset + parent.length);
}

bool IsJpeg2000(const io::ByteSource& src) {
    uint8_t h[16];
    if (src.Size() < 16 || !src.Read(0, {h, 16}).ok()) return false;
    static constexpr uint8_t kSocSiz[4] = {0xff, 0x4f, 0xff, 0x51};
    return std::memcmp(h, kSocSiz, 4) == 0 || std::memcmp(h + 4, "jP  ", 4) == 0;
}

Expected<Jp2File> ReadJp2File(const io::ByteSource& src) {
    if (!IsJpeg2000(src)) return Status(StatusCode::kCorrupt, "No JPEG 2000 stream detected in " + std::string(src.Name()));
    Jp2File f;
    uint8_t h[4];
    DDBT_TRY(src.Read(0, {h, 4}));
    if (h[0] == 0xff && h[1] == 0x4f) {
        f.codestreamOffset = 0;
        f.codestreamLength = src.Size();
        return f;
    }
    f.jp2 = true;
    const std::vector<Box> boxes = ReadBoxes(src, 0, src.Size());
    bool found = false;
    for (const Box& b : boxes) {
        if (!found && EqualNoCase(b.type, "jp2c")) {
            f.codestreamOffset = b.dataOffset;
            f.codestreamLength = b.DataLength();
            found = true;
        }
        if (EqualNoCase(b.type, "jp2h")) ReadHeaderBoxes(src, b, f);
        ReadMetadataBox(src, b, f, 0);
    }
    if (!found || (f.codestreamOffset == 0 && f.codestreamLength == 0))
        return Status(StatusCode::kCorrupt, "No code-stream in JP2 file " + std::string(src.Name()));
    return f;
}

ChannelRoles ReadChannelDefinitions(const std::vector<std::vector<uint8_t>>& boxes, int bands) {
    ChannelRoles r;
    for (const std::vector<uint8_t>& cdef : boxes) {
        if (bands <= 0 || cdef.size() != 2 + static_cast<size_t>(bands) * 6) continue;
        const int entries = (cdef[0] << 8) | cdef[1];
        if (entries != bands) continue;
        r.red = r.green = r.blue = -1;
        for (int i = 0; i < bands; ++i) {
            const uint8_t* e = cdef.data() + 2 + 6 * i;
            const int cn = (e[0] << 8) | e[1];
            const int typ = (e[2] << 8) | e[3];
            const int asoc = (e[4] << 8) | e[5];
            if (cn >= bands) break;  // GDAL stops at an invalid entry
            if (typ == 0) {
                if (asoc == 1) r.red = cn;
                else if (asoc == 2) r.green = cn;
                else if (asoc == 3) r.blue = cn;
                else if (asoc > bands && asoc != 65535) break;
            } else if (typ == 1) {
                r.alpha = cn;
            }
        }
    }
    return r;
}

tiff::Georeferencing ReadJp2Georeferencing(const Jp2File& file, std::string_view path, bool* rpc) {
    tiff::Georeferencing g;
    bool hasRpc = false;
    if (file.jp2) {
        tiff::Georeferencing geo;
        if (ParseGeoJp2(file, geo, hasRpc)) {
            g = std::move(geo);
        } else {
            // A GML grid without a CRS keeps its geotransform unless an MSIG box replaces it.
            if (!ParseGml(file, g)) ParseMsig(file, g);
        }
    }
    if (!g.geoTransform && !path.empty()) {
        if (auto w = tiff::ReadWorldFile(path)) {
            g.geoTransform = w->first;
            g.source = tiff::GeoTransformSource::kWorldFile;
            g.worldFile = w->second;
            g.pixelIsPoint = false;
        }
    }
    if (rpc != nullptr) *rpc = hasRpc;
    return g;
}

std::vector<std::pair<std::string, std::string>> Jp2Metadata(const Jp2File& file) {
    std::vector<std::pair<std::string, std::string>> items;
    auto set = [&](const std::string& key, const std::string& value) {
        for (auto& [k, v] : items)
            if (EqualNoCase(k, key)) {
                v = value;
                return;
            }
        items.emplace_back(key, value);
    };
    if (!file.gdalMultiDomainMetadata.empty()) {
        if (const auto root = xml::Parse(file.gdalMultiDomainMetadata)) {
            for (const xml::Element& md : root->children) {
                if (md.name != "Metadata") continue;
                const std::string* domain = md.Attribute("domain");
                const std::string* format = md.Attribute("format");
                if ((domain != nullptr && !domain->empty()) || format != nullptr) continue;  // the default domain
                for (const xml::Element& item : md.children) {
                    const std::string* key = item.Attribute("key");
                    if (item.name == "MDI" && key != nullptr) set(*key, item.text);
                }
            }
        }
    }
    if (file.resolution) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%g", (*file.resolution)[0]);
        set("TIFFTAG_XRESOLUTION", buf);
        std::snprintf(buf, sizeof(buf), "%g", (*file.resolution)[1]);
        set("TIFFTAG_YRESOLUTION", buf);
        set("TIFFTAG_RESOLUTIONUNIT", "3 (pixels/cm)");
    }
    return items;
}

}  // namespace ddbt::cog::jp2
