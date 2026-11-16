// SPDX-License-Identifier: MPL-2.0
//
// Follows GDAL 3.13's GTiffDataset (gtiffdataset_read.cpp: OpenOffset, GetMetadataDomainList,
// GetMetadata, LoadMDAreaOrPoint, LoadICCProfile, LoadMetadata, CheckCOGLayout), the JPEG quality
// guess of gtiffdataset_write.cpp, GDALMultiDomainMetadata and CPLStringList.
#include "cog/tiff/gdal_metadata.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "cog/tiff/geotiff.h"
#include "cog/tiff/tiff_reader.h"
#include "cog/tiff/tiff_tags.h"
#include "core/util/xml_lite.h"

namespace ddbt::cog::tiff {

static char Upper(char c) { return c >= 'a' && c <= 'z' ? static_cast<char>(c - 'a' + 'A') : c; }

bool EqualNoCase(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i)
        if (Upper(a[i]) != Upper(b[i])) return false;
    return true;
}

namespace {

namespace tags {
constexpr uint16_t kDocumentName = 269;
constexpr uint16_t kXResolution = 282;
constexpr uint16_t kYResolution = 283;
constexpr uint16_t kMinSampleValue = 280;
constexpr uint16_t kMaxSampleValue = 281;
constexpr uint16_t kResolutionUnit = 296;
constexpr uint16_t kTransferFunction = 301;
constexpr uint16_t kArtist = 315;
constexpr uint16_t kHostComputer = 316;
constexpr uint16_t kWhitePoint = 318;
constexpr uint16_t kPrimaryChromaticities = 319;
constexpr uint16_t kTransferRange = 342;
constexpr uint16_t kXmlPacket = 700;
constexpr uint16_t kCopyright = 33432;
constexpr uint16_t kIccProfile = 34675;
constexpr uint16_t kRpcCoefficient = 50844;
constexpr uint16_t kTiffRsid = 50908;
constexpr uint16_t kGeoMetadata = 50909;
}  // namespace tags

constexpr uint16_t kCompressionNone = 1;
constexpr uint16_t kCompressionJpeg = 7;
constexpr uint16_t kCompressionAdobeDeflate = 8;
constexpr uint16_t kCompressionLzw = 5;
constexpr uint16_t kCompressionLerc = 34887;
constexpr uint16_t kCompressionZstd = 50000;
constexpr uint16_t kCompressionWebp = 50001;

constexpr std::string_view kImageStructure = "IMAGE_STRUCTURE";


bool StartsWithNoCase(std::string_view s, std::string_view prefix) {
    return s.size() >= prefix.size() && EqualNoCase(s.substr(0, prefix.size()), prefix);
}

bool IsDocumentDomain(std::string_view domain) {
    return StartsWithNoCase(domain, "xml:") || StartsWithNoCase(domain, "json:");
}

// CPLCompareKeyValueString: the keys (up to '=' or the end) compared without case.
int CompareKeys(std::string_view a, std::string_view b) {
    for (size_t i = 0;; ++i) {
        const char ca = i < a.size() ? a[i] : '\0';
        const char cb = i < b.size() ? b[i] : '\0';
        const bool endA = ca == '=' || ca == '\0';
        const bool endB = cb == '=' || cb == '\0';
        if (endA) return endB ? 0 : -1;
        if (endB) return 1;
        if (Upper(ca) != Upper(cb)) return Upper(ca) < Upper(cb) ? -1 : 1;
    }
}

// CSLFindName: the entry "key=..." or "key:..." (key without case).
std::vector<std::string>::iterator FindName(std::vector<std::string>& items, std::string_view key) {
    return std::find_if(items.begin(), items.end(), [&](const std::string& item) {
        return item.size() > key.size() && StartsWithNoCase(item, key) &&
               (item[key.size()] == '=' || item[key.size()] == ':');
    });
}

// printf's %.<precision>g
std::string FormatG(double v, int precision) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.*g", precision, v);
    return buf;
}

// CPLBase64Encode
std::string Base64(std::span<const uint8_t> bytes) {
    static constexpr char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((bytes.size() + 2) / 3 * 4);
    size_t i = 0;
    for (; i + 3 <= bytes.size(); i += 3) {
        const uint32_t v = (uint32_t{bytes[i]} << 16) | (uint32_t{bytes[i + 1]} << 8) | bytes[i + 2];
        out += kAlphabet[(v >> 18) & 63];
        out += kAlphabet[(v >> 12) & 63];
        out += kAlphabet[(v >> 6) & 63];
        out += kAlphabet[v & 63];
    }
    if (i < bytes.size()) {
        const uint32_t v = (uint32_t{bytes[i]} << 16) | (i + 1 < bytes.size() ? uint32_t{bytes[i + 1]} << 8 : 0u);
        out += kAlphabet[(v >> 18) & 63];
        out += kAlphabet[(v >> 12) & 63];
        out += i + 1 < bytes.size() ? kAlphabet[(v >> 6) & 63] : '=';
        out += '=';
    }
    return out;
}

// The values libtiff hands over for a tag, as GDAL then reads them -------------------------------

const TagEntry* TagOf(const IfdInfo& ifd, uint16_t tag) {
    const TagEntry* e = ifd.FindTag(tag);
    return e != nullptr && e->count > 0 ? e : nullptr;
}

std::optional<std::string> StringTag(const TiffReader& r, const IfdInfo& ifd, uint16_t tag) {
    const TagEntry* e = TagOf(ifd, tag);
    if (e == nullptr) return std::nullopt;
    auto s = r.ReadTagString(*e, r.Options().maxTagBytes);
    if (!s.ok()) return std::nullopt;
    return std::move(*s);
}

std::optional<std::vector<uint8_t>> BytesTag(const TiffReader& r, const IfdInfo& ifd, uint16_t tag) {
    const TagEntry* e = TagOf(ifd, tag);
    if (e == nullptr) return std::nullopt;
    auto b = r.ReadTagBytes(*e, r.Options().maxTagBytes);
    if (!b.ok()) return std::nullopt;
    return std::move(*b);
}

std::optional<std::vector<uint64_t>> UIntsTag(const TiffReader& r, const IfdInfo& ifd, uint16_t tag) {
    const TagEntry* e = TagOf(ifd, tag);
    if (e == nullptr) return std::nullopt;
    auto v = r.ReadTagUInts(*e, uint64_t{1} << 20);
    if (!v.ok() || v->empty()) return std::nullopt;
    return std::move(*v);
}

std::optional<std::vector<double>> DoublesTag(const TiffReader& r, const IfdInfo& ifd, uint16_t tag) {
    const TagEntry* e = TagOf(ifd, tag);
    if (e == nullptr) return std::nullopt;
    auto v = r.ReadTagDoubles(*e, uint64_t{1} << 20);
    if (!v.ok() || v->empty()) return std::nullopt;
    return std::move(*v);
}

// A C string (GDAL copies the bytes of a tag, then reads them up to the first NUL)
std::string CString(const std::vector<uint8_t>& bytes) {
    const auto nul = std::find(bytes.begin(), bytes.end(), uint8_t{0});
    return std::string(bytes.begin(), nul);
}

// GDAL_METADATA ----------------------------------------------------------------------------------

// What GDAL keeps of an item of the IMAGE_STRUCTURE domain (the others describe how GDAL wrote
// the blocks, which the GTiff reader takes from the tags); `dataset` also gets the side effects.
bool KeepsImageStructureItem(const GdalMetadataItem& it, uint16_t compression, MetadataDomains& target) {
    if (EqualNoCase(it.name, "OVERVIEW_RESAMPLING") || EqualNoCase(it.name, "INTERLEAVE")) return true;
    if (compression == kCompressionWebp && EqualNoCase(it.name, "COMPRESSION_REVERSIBILITY")) return true;
    if (compression == kCompressionWebp && EqualNoCase(it.name, "WEBP_LEVEL")) {
        const int level = std::atoi(it.value.c_str());
        if (level >= 1 && level <= 100) target.SetItem(kImageStructure, "COMPRESSION_REVERSIBILITY", "LOSSY");
        return true;
    }
    return compression == kCompressionLerc &&
           (EqualNoCase(it.name, "MAX_Z_ERROR") || EqualNoCase(it.name, "MAX_Z_ERROR_OVERVIEW"));
}

void SetValue(MetadataDomains& domains, const GdalMetadataItem& it) {
    if (IsDocumentDomain(it.domain)) domains.SetItems(it.domain, {it.value});
    else domains.SetItem(it.domain, it.name, it.value);
}

// The items of GDAL_METADATA applied as the GTiff driver applies them, in their order.
void ApplyGdalMetadata(std::string_view xml, uint16_t compression, GdalMetadata& md) {
    const int bandCount = static_cast<int>(md.bands.size());
    for (const GdalMetadataItem& it : ParseGdalMetadata(xml)) {
        if (it.sample < -1 || it.sample > bandCount - 1) continue;
        const int band = it.sample + 1;  // 0: the dataset
        MetadataDomains& target = band == 0 ? md.dataset : md.bands[static_cast<size_t>(band - 1)].domains;
        if (EqualNoCase(it.domain, kImageStructure) && !KeepsImageStructureItem(it, compression, md.dataset)) continue;
        if (band == 0) {
            SetValue(target, it);
            continue;
        }
        GdalBandRoles& roles = md.bands[static_cast<size_t>(band - 1)].roles;
        if (EqualNoCase(it.role, "scale")) {
            roles.hasOffsetScale = true;
            roles.scale = std::atof(it.value.c_str());
        } else if (EqualNoCase(it.role, "offset")) {
            roles.hasOffsetScale = true;
            roles.offset = std::atof(it.value.c_str());
        } else if (EqualNoCase(it.role, "unittype")) {
            roles.unitType = it.value;
        } else if (EqualNoCase(it.role, "description")) {
            roles.description = it.value;
        } else if (EqualNoCase(it.role, "colorinterp")) {
            roles.colorInterp = EqualNoCase(it.value, "undefined") ? ColorInterp::kUndefined : ColorInterpByName(it.value);
            // A name GDAL does not know stays a band item
            if (*roles.colorInterp == ColorInterp::kUndefined && !EqualNoCase(it.value, "undefined"))
                target.SetItem("", "COLOR_INTERPRETATION", it.value);
        } else {
            SetValue(target, it);
        }
    }
}

// IMAGE_STRUCTURE --------------------------------------------------------------------------------

// GTIFFFindNextTable: the next marker segment `marker` of JPEG tables (from its length bytes) and
// its length, or an empty span.
std::span<const uint8_t> NextJpegTable(std::span<const uint8_t> t, uint8_t marker) {
    const size_t n = t.size();
    for (size_t i = 0; i + 1 < n;) {
        if (t[i] != 0xFF) return {};
        ++i;
        if (t[i] == 0xD8) {
            ++i;
            continue;
        }
        if (i + 2 >= n) return {};
        const size_t len = size_t{t[i + 1]} * 256 + t[i + 2];
        if (i + 1 + len >= n) return {};
        if (t[i] == marker) return t.subspan(i + 1, len);
        i += 1 + len;
    }
    return {};
}

constexpr uint8_t kMarkerHuffman = 0xC4;
constexpr uint8_t kMarkerQuant = 0xDB;

// The quantization segments of JPEG tables, one after the other (what GDAL hashes or compares)
std::vector<uint8_t> QuantSegments(std::span<const uint8_t> tables) {
    std::vector<uint8_t> out;
    std::span<const uint8_t> rest = tables;
    for (;;) {
        const std::span<const uint8_t> seg = NextJpegTable(rest, kMarkerQuant);
        if (seg.empty()) return out;
        out.insert(out.end(), seg.begin(), seg.end());
        rest = rest.subspan(static_cast<size_t>(seg.data() + seg.size() - rest.data()));
    }
}

// libjpeg's quantization segment (emit_dqt) of a standard table at `quality`
// (jpeg_set_quality without forcing baseline, as libtiff asks).
void AppendQuantSegment(std::vector<uint8_t>& out, const std::array<uint16_t, 64>& basic, int index, int quality) {
    static constexpr std::array<uint8_t, 64> kNaturalOrder = {
        0,  1,  8,  16, 9,  2,  3,  10, 17, 24, 32, 25, 18, 11, 4,  5,  12, 19, 26, 33, 40, 48,
        41, 34, 27, 20, 13, 6,  7,  14, 21, 28, 35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23,
        30, 37, 44, 51, 58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63};
    const long scale = quality < 50 ? 5000 / quality : 200 - quality * 2;
    std::array<uint16_t, 64> q{};
    bool wide = false;
    for (size_t i = 0; i < 64; ++i) {
        long v = (static_cast<long>(basic[i]) * scale + 50L) / 100L;
        v = std::clamp(v, 1L, 32767L);
        q[i] = static_cast<uint16_t>(v);
        wide = wide || v > 255;
    }
    const size_t length = wide ? 64 * 2 + 1 + 2 : 64 + 1 + 2;
    out.push_back(static_cast<uint8_t>(length >> 8));
    out.push_back(static_cast<uint8_t>(length & 0xFF));
    out.push_back(static_cast<uint8_t>(index + (wide ? 16 : 0)));
    for (const uint8_t natural : kNaturalOrder) {
        if (wide) out.push_back(static_cast<uint8_t>(q[natural] >> 8));
        out.push_back(static_cast<uint8_t>(q[natural] & 0xFF));
    }
}

}  // namespace

std::vector<uint8_t> StandardJpegQuantTables(int quality, bool ycbcr) {
    static constexpr std::array<uint16_t, 64> kLuminance = {
        16, 11, 10, 16, 24,  40,  51,  61,  12, 12, 14, 19, 26,  58,  60,  55,  14, 13, 16, 24, 40, 57,
        69, 56, 14, 17, 22,  29,  51,  87,  80, 62, 18, 22, 37,  56,  68,  109, 103, 77, 24, 35, 55, 64,
        81, 104, 113, 92, 49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99};
    static constexpr std::array<uint16_t, 64> kChrominance = {
        17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99, 24, 26, 56, 99, 99, 99,
        99, 99, 47, 66, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
        99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99};
    std::vector<uint8_t> out;
    AppendQuantSegment(out, kLuminance, 0, quality);
    if (ycbcr) AppendQuantSegment(out, kChrominance, 1, quality);
    return out;
}

namespace {

// The colour space GDAL reads the samples from when it converts them to RGB(A)
const char* SourceColorSpace(const IfdInfo& main) {
    if (main.compression == kCompressionJpeg && main.photometric == photometric::kYCbCr) return "YCbCr";
    if (!main.viaRgba) return nullptr;
    switch (main.photometric) {
        case photometric::kCieLab: return "CIELAB";
        case photometric::kLogL: return "LOGL";
        case photometric::kLogLuv: return "LOGLUV";
        case photometric::kSeparated: return "CMYK";
        case photometric::kYCbCr: return "YCbCr";
        default: return nullptr;
    }
}

// The bits of a sample when GDAL's type for it has more (NBITS): the odd sizes GDAL reads into
// UInt8, UInt16 or UInt32, and 24-bit floats it reads into Float32
std::optional<int> NBits(const IfdInfo& main) {
    const int bits = main.bitsPerSample;
    const bool odd = main.sampleFormat == 3 ? bits == 24 : bits < 32 && bits != 8 && bits != 16;
    return odd ? std::optional<int>(bits) : std::nullopt;
}

// GTiffDataset::CheckCOGLayout: tiled, the directories first and in order of decreasing size,
// the data of the smallest overview first.
bool HasCogLayout(const TiffReader& r, const IfdInfo& main, const std::vector<uint32_t>& overviews) {
    const auto firstBlock = [&](uint32_t ifd) -> std::optional<uint64_t> {
        auto loc = r.Locate(ifd, 0);
        if (!loc.ok() || loc->size == 0) return std::nullopt;
        return loc->offset;
    };
    bool ok = main.tiled;
    const bool needsOverviews = main.width > 512 || main.height > 512;
    ok = ok && !(overviews.empty() && needsOverviews);
    const uint64_t firstIfd = r.IsBigTiff() ? 16 : 8;
    ok = ok && main.offset == firstIfd;
    uint64_t lastIfd = firstIfd;
    uint32_t lastW = main.width, lastH = main.height;
    for (const uint32_t o : overviews) {
        const IfdInfo& ov = r.Ifd(o);
        ok = ok && ov.offset > lastIfd && ov.width <= lastW && ov.height <= lastH && ov.tiled;
        lastW = ov.width;
        lastH = ov.height;
        lastIfd = ov.offset;
    }
    uint64_t dataOffset = lastIfd;
    for (auto it = overviews.rbegin(); ok && it != overviews.rend(); ++it) {
        ok = r.Ifd(*it).tiled;
        if (const auto off = firstBlock(*it)) {
            ok = ok && *off > dataOffset;
            dataOffset = *off;
        }
    }
    if (!overviews.empty())
        if (const auto off = firstBlock(main.index)) ok = ok && *off > dataOffset;
    return ok;
}

// The COG layout GDAL's ghost header promises (the blocks GDAL can then read without the index)
bool GhostSaysCog(const GhostHeader& g) {
    if (!g.present) return false;
    const std::string& t = g.text;
    const auto has = [&](std::string_view s) { return t.find(s) != std::string::npos; };
    const bool bandOrTile = !has("INTERLEAVE=") || has("INTERLEAVE=BAND") || has("INTERLEAVE=TILE");
    return !has("KNOWN_INCOMPATIBLE_EDITION=YES") && has("LAYOUT=IFDS_BEFORE_DATA") && has("BLOCK_ORDER=ROW_MAJOR") &&
           has("BLOCK_LEADER=SIZE_AS_UINT4") && has("BLOCK_TRAILER=LAST_4_BYTES_REPEATED") && bandOrTile;
}

void ReadImageStructure(const IfdInfo& main, GdalMetadata& md) {
    MetadataDomains& d = md.dataset;
    if (main.compression != kCompressionNone) {
        if (const char* name = GdalCompressionName(main.compression)) {
            d.SetItem(kImageStructure, "COMPRESSION", name);
        } else {
            d.SetItem("", "COMPRESSION", std::to_string(main.compression));  // GDAL puts it there
        }
    }
    if (main.compression == kCompressionJpeg && main.photometric == photometric::kYCbCr) {
        d.SetItem(kImageStructure, "COMPRESSION", "YCbCr JPEG");
    } else if (main.compression == kCompressionLerc) {
        // libtiff's LERC_ADD_COMPRESSION and LERC_VERSION come from LercParameters (2.4 without it)
        const uint32_t version = main.hasLercParameters ? main.lercParameters[0] : 4;
        const uint32_t add = main.hasLercParameters ? main.lercParameters[1] : 0;
        if (add == 1) d.SetItem(kImageStructure, "COMPRESSION", "LERC_DEFLATE");
        else if (add == 2) d.SetItem(kImageStructure, "COMPRESSION", "LERC_ZSTD");
        if (version == 4) d.SetItem(kImageStructure, "LERC_VERSION", "2.4");
    }
    const int bands = static_cast<int>(md.bands.size());
    d.SetItem(kImageStructure, "INTERLEAVE", main.planarConfig == 1 && bands != 1 ? "PIXEL" : "BAND");
    if (const auto nbits = NBits(main))
        for (GdalBandMetadata& b : md.bands) b.domains.SetItem(kImageStructure, "NBITS", std::to_string(*nbits));
    if (main.photometric == photometric::kMinIsWhite && main.hasPhotometric)
        d.SetItem(kImageStructure, "MINISWHITE", "YES");
}

void ReadJpegAndPredictor(const IfdInfo& main, MetadataDomains& d) {
    if (main.compression == kCompressionJpeg && !main.jpegTables.empty()) {
        const std::span<const uint8_t> tables(main.jpegTables);
        if (!NextJpegTable(tables, kMarkerQuant).empty()) {
            if (const auto quality = GuessJpegQuality(tables, main.photometric == photometric::kYCbCr)) {
                d.SetItem(kImageStructure, "JPEG_QUALITY", std::to_string(*quality));
                const bool huffman = !NextJpegTable(tables, kMarkerHuffman).empty();
                d.SetItem(kImageStructure, "JPEGTABLESMODE", huffman ? "3" : "1");
            }
        }
    }
    const bool predicts = main.compression == kCompressionLzw || main.compression == kCompressionAdobeDeflate ||
                          main.compression == kCompressionZstd;
    if (predicts && main.predictor > 1) d.SetItem(kImageStructure, "PREDICTOR", std::to_string(main.predictor));
}

// The default domain's TIFF tags ------------------------------------------------------------------

void ReadTiffTags(const TiffReader& r, const IfdInfo& main, MetadataDomains& d) {
    struct StringTagName {
        const char* name;
        uint16_t tag;
    };
    static constexpr std::array<StringTagName, 7> kStrings = {{{"TIFFTAG_DOCUMENTNAME", tags::kDocumentName},
                                                                {"TIFFTAG_IMAGEDESCRIPTION", tag::kImageDescription},
                                                                {"TIFFTAG_SOFTWARE", tag::kSoftware},
                                                                {"TIFFTAG_DATETIME", tag::kDateTime},
                                                                {"TIFFTAG_ARTIST", tags::kArtist},
                                                                {"TIFFTAG_HOSTCOMPUTER", tags::kHostComputer},
                                                                {"TIFFTAG_COPYRIGHT", tags::kCopyright}}};
    for (const auto& t : kStrings)
        if (const auto s = StringTag(r, main, t.tag)) d.SetItem("", t.name, *s);
    // libtiff keeps the resolutions as floats; GDAL prints them with %.8g
    for (const auto& [name, tag] : {std::pair{"TIFFTAG_XRESOLUTION", tags::kXResolution},
                                    std::pair{"TIFFTAG_YRESOLUTION", tags::kYResolution}})
        if (const auto v = DoublesTag(r, main, tag))
            d.SetItem("", name, FormatG(static_cast<double>(static_cast<float>(v->front())), 8));
    for (const auto& [name, tag] : {std::pair{"TIFFTAG_MINSAMPLEVALUE", tags::kMinSampleValue},
                                    std::pair{"TIFFTAG_MAXSAMPLEVALUE", tags::kMaxSampleValue}})
        if (const auto v = UIntsTag(r, main, tag)) d.SetItem("", name, std::to_string(static_cast<uint16_t>(v->front())));
    if (const auto b = BytesTag(r, main, tags::kGeoMetadata)) d.SetItem("", "GEO_METADATA", CString(*b));
    if (const auto s = StringTag(r, main, tags::kTiffRsid)) d.SetItem("", "TIFF_RSID", *s);
    if (const auto v = UIntsTag(r, main, tags::kResolutionUnit)) {
        const auto unit = static_cast<uint16_t>(v->front());
        const std::string n = std::to_string(unit);
        d.SetItem("", "TIFFTAG_RESOLUTIONUNIT",
                  unit == 1   ? n + " (unitless)"
                  : unit == 2 ? n + " (pixels/inch)"
                  : unit == 3 ? n + " (pixels/cm)"
                              : n);
    }
    if (const auto b = BytesTag(r, main, tags::kXmlPacket)) d.SetItems("xml:XMP", {CString(*b)});
}

// COLOR_PROFILE: the ICC profile, or the colorimetry of a TIFF without one -----------------------

std::string TransferFunctionText(std::span<const uint16_t> table) {
    std::string s;
    for (size_t i = 0; i < table.size(); ++i) {
        if (i > 0) s += ", ";
        s += std::to_string(table[i]);
    }
    return s;
}

void ReadColorProfile(const TiffReader& r, const IfdInfo& main, MetadataDomains& d) {
    static constexpr std::string_view kDomain = "COLOR_PROFILE";
    if (const auto icc = BytesTag(r, main, tags::kIccProfile)) {
        d.SetItem(kDomain, "SOURCE_ICC_PROFILE", Base64(*icc));
        return;
    }
    const auto chr = DoublesTag(r, main, tags::kPrimaryChromaticities);
    const auto wp = DoublesTag(r, main, tags::kWhitePoint);
    if (!chr || chr->size() < 6 || !wp || wp->size() < 2) return;
    // libtiff's transfer function: the file's (one table, or three), else its default (gamma 2.2)
    // for 1 << bits entries; three only when the image has more than one colour sample
    if (main.bitsPerSample > 24) return;
    const size_t n = size_t{1} << main.bitsPerSample;
    const int colourSamples = main.samplesPerPixel - static_cast<int>(main.extraSamples.size());
    if (colourSamples <= 1) return;  // libtiff gives one table, GDAL needs three
    std::array<std::vector<uint16_t>, 3> tf;
    if (const auto stored = UIntsTag(r, main, tags::kTransferFunction)) {
        if (stored->size() != n && stored->size() != 3 * n) return;  // libtiff refuses the tag
        for (size_t c = 0; c < 3; ++c) {
            const size_t base = stored->size() == 3 * n ? c * n : 0;
            for (size_t i = 0; i < n; ++i) tf[c].push_back(static_cast<uint16_t>((*stored)[base + i]));
        }
    } else {
        tf[0].assign(n, 0);
        for (size_t i = 1; i < n; ++i)
            tf[0][i] = static_cast<uint16_t>(std::floor(65535.0 * std::pow(static_cast<double>(i) / (static_cast<double>(n) - 1.0), 2.2) + 0.5));
        tf[1] = tf[0];
        tf[2] = tf[0];
    }
    const auto pair = [](double a, double b) {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%.9f, %.9f, 1.0", static_cast<double>(static_cast<float>(a)),
                      static_cast<double>(static_cast<float>(b)));
        return std::string(buf);
    };
    d.SetItem(kDomain, "SOURCE_PRIMARIES_RED", pair((*chr)[0], (*chr)[1]));
    d.SetItem(kDomain, "SOURCE_PRIMARIES_GREEN", pair((*chr)[2], (*chr)[3]));
    d.SetItem(kDomain, "SOURCE_PRIMARIES_BLUE", pair((*chr)[4], (*chr)[5]));
    d.SetItem(kDomain, "SOURCE_WHITEPOINT", pair((*wp)[0], (*wp)[1]));
    d.SetItem(kDomain, "TIFFTAG_TRANSFERFUNCTION_RED", TransferFunctionText(tf[0]));
    d.SetItem(kDomain, "TIFFTAG_TRANSFERFUNCTION_GREEN", TransferFunctionText(tf[1]));
    d.SetItem(kDomain, "TIFFTAG_TRANSFERFUNCTION_BLUE", TransferFunctionText(tf[2]));
    if (const auto range = UIntsTag(r, main, tags::kTransferRange); range && range->size() >= 6) {
        const auto three = [&](size_t a, size_t b, size_t c) {
            return std::to_string((*range)[a]) + ", " + std::to_string((*range)[b]) + ", " + std::to_string((*range)[c]);
        };
        d.SetItem(kDomain, "TIFFTAG_TRANSFERRANGE_BLACK", three(0, 2, 4));
        d.SetItem(kDomain, "TIFFTAG_TRANSFERRANGE_WHITE", three(1, 3, 5));
    }
}

// RPC: the 92 coefficients of the RPCCoefficientTag (TIFFRPCTagToRPCMetadata) -------------------

void ReadRpc(const TiffReader& r, const IfdInfo& main, MetadataDomains& d) {
    const auto rpc = DoublesTag(r, main, tags::kRpcCoefficient);
    if (!rpc || rpc->size() != 92) return;
    static constexpr std::array<const char*, 12> kScalars = {
        "ERR_BIAS",   "ERR_RAND",   "LINE_OFF",    "SAMP_OFF",   "LAT_OFF",    "LONG_OFF",
        "HEIGHT_OFF", "LINE_SCALE", "SAMP_SCALE", "LAT_SCALE", "LONG_SCALE", "HEIGHT_SCALE"};
    static constexpr std::array<const char*, 4> kPolynomials = {"LINE_NUM_COEFF", "LINE_DEN_COEFF", "SAMP_NUM_COEFF",
                                                                "SAMP_DEN_COEFF"};
    std::vector<std::string> items;
    for (size_t i = 0; i < kScalars.size(); ++i) items.push_back(std::string(kScalars[i]) + "=" + FormatG((*rpc)[i], 15));
    for (size_t p = 0; p < kPolynomials.size(); ++p) {
        std::string v;
        for (size_t i = 0; i < 20; ++i) {
            if (i > 0) v += ' ';
            v += FormatG((*rpc)[12 + p * 20 + i], 15);
        }
        items.push_back(std::string(kPolynomials[p]) + "=" + v);
    }
    d.SetItems("RPC", std::move(items));
}

}  // namespace

// MetadataDomains ---------------------------------------------------------------------------------

MetadataDomains::Domain& MetadataDomains::Of(std::string_view domain) {
    for (Domain& d : domains_)
        if (EqualNoCase(d.name, domain)) return d;
    domains_.push_back(Domain{std::string(domain), {}});
    return domains_.back();
}

void MetadataDomains::SetItem(std::string_view domain, std::string_view name, std::string_view value) {
    Domain& d = Of(domain);
    std::string entry = std::string(name) + "=" + std::string(value);
    const auto found = FindName(d.items, name);
    if (found != d.items.end()) {
        *found = std::move(entry);
    } else if (d.sorted) {
        const auto at = std::upper_bound(d.items.begin(), d.items.end(), entry,
                                         [](const std::string& a, const std::string& b) { return CompareKeys(a, b) < 0; });
        d.items.insert(at, std::move(entry));
    } else {
        d.items.push_back(std::move(entry));
    }
}

void MetadataDomains::SetItems(std::string_view domain, std::vector<std::string> items) {
    Domain& d = Of(domain);
    d.items = std::move(items);
    // GDAL sorts the name=value domains it is given whole, for its lookups
    d.sorted = !IsDocumentDomain(domain) && !EqualNoCase(domain, "SUBDATASETS") && !EqualNoCase(domain, "IMD");
    if (d.sorted)
        std::stable_sort(d.items.begin(), d.items.end(),
                         [](const std::string& a, const std::string& b) { return CompareKeys(a, b) < 0; });
}

std::optional<std::string> MetadataDomains::Item(std::string_view domain, std::string_view name) const {
    for (const Domain& d : domains_) {
        if (!EqualNoCase(d.name, domain)) continue;
        for (const std::string& item : d.items)
            if (item.size() > name.size() && StartsWithNoCase(item, name) &&
                (item[name.size()] == '=' || item[name.size()] == ':'))
                return item.substr(name.size() + 1);
    }
    return std::nullopt;
}

const std::vector<std::string>* MetadataDomains::Items(std::string_view domain) const {
    for (const Domain& d : domains_)
        if (EqualNoCase(d.name, domain)) return d.items.empty() ? nullptr : &d.items;
    return nullptr;
}

// GDAL_METADATA ----------------------------------------------------------------------------------

std::vector<GdalMetadataItem> ParseGdalMetadata(std::string_view xml) {
    std::vector<GdalMetadataItem> items;
    const auto root = xml::Parse(xml);
    if (!root || !EqualNoCase(root->name, "GDALMetadata")) return items;
    for (const xml::Element& e : root->children) {
        if (!EqualNoCase(e.name, "Item")) continue;
        const std::string* name = e.Attribute("name");
        if (name == nullptr || !e.value) continue;
        GdalMetadataItem it;
        it.name = *name;
        const std::string* sample = e.Attribute("sample");
        it.sample = sample != nullptr ? std::atoi(sample->c_str()) : -1;
        if (const std::string* role = e.Attribute("role")) it.role = *role;
        if (const std::string* domain = e.Attribute("domain")) it.domain = *domain;
        // GDAL decodes the value a second time, for backward compatibility
        it.value = xml::GdalUnescape(*e.value);
        items.push_back(std::move(it));
    }
    return items;
}

std::vector<GdalBandMetadata> GdalMetadataBands(std::string_view xml, int bandCount) {
    GdalMetadata md;
    md.bands.resize(static_cast<size_t>(std::max(bandCount, 0)));
    ApplyGdalMetadata(xml, kCompressionNone, md);
    return std::move(md.bands);
}

// The whole -----------------------------------------------------------------------------------------

std::vector<std::string> GdalMetadata::DomainList() const {
    std::vector<std::string> names;
    for (const auto& d : dataset.All())
        if (!d.items.empty()) names.push_back(d.name);
    return names;
}

GdalMetadata ReadGdalMetadata(const TiffReader& reader, const GdalDatasetShape& shape, const GeoKeyDirectory& keys) {
    GdalMetadata md;
    md.bands.resize(static_cast<size_t>(std::max(shape.bandCount, 0)));
    const IfdInfo& main = reader.Ifd(reader.MainImage());
    MetadataDomains& d = md.dataset;

    // GTiffDataset::Open reads GDAL's ghost header first, then OpenOffset the directory, in this order
    if (GhostSaysCog(reader.Ghost())) d.SetItem(kImageStructure, "LAYOUT", "COG");
    if (const char* space = SourceColorSpace(main)) d.SetItem(kImageStructure, "SOURCE_COLOR_SPACE", space);
    ReadTiffTags(reader, main, d);
    ReadImageStructure(main, md);
    ApplyGdalMetadata(main.gdalMetadata, main.compression, md);
    ReadJpegAndPredictor(main, d);

    // What GetMetadataDomainList and GetMetadata load on first use, in their order
    if (!d.Item("", "AREA_OR_POINT"))
        if (const auto rasterType = keys.Short(geokey::kGTRasterType))
            d.SetItem("", "AREA_OR_POINT", *rasterType == 2 ? "Point" : "Area");
    ReadRpc(reader, main, d);
    ReadColorProfile(reader, main, d);
    if (!d.Item(kImageStructure, "LAYOUT") && HasCogLayout(reader, main, shape.overviewIfds))
        d.SetItem(kImageStructure, "LAYOUT", "COG");
    return md;
}

const char* GdalCompressionName(uint16_t compression) noexcept {
    switch (compression) {
        case 1: return "NONE";
        case 2: return "CCITTRLE";
        case 3: return "CCITTFAX3";
        case 4: return "CCITTFAX4";
        case 5: return "LZW";
        case 6: return "OJPEG";
        case 7: return "JPEG";
        case 8: return "DEFLATE";
        case 32766: return "NEXT";
        case 32771: return "CCITTRLEW";
        case 32773: return "PACKBITS";
        case 32809: return "THUNDERSCAN";
        case 32908: return "PIXARFILM";
        case 32909: return "PIXARLOG";
        case 32946: return "DEFLATE";
        case 32947: return "DCS";
        case 34661: return "JBIG";
        case 34676: return "SGILOG";
        case 34677: return "SGILOG24";
        case 34712: return "JP2000";
        case 34887: return "LERC";
        case 34925: return "LZMA";
        case 50000: return "ZSTD";
        case 50001: return "WEBP";
        case 50002: return "JXL";
        case 52546: return "JXL";
        default: return nullptr;
    }
}

std::optional<int> GuessJpegQuality(std::span<const uint8_t> jpegTables, bool ycbcr) {
    const std::vector<uint8_t> stored = QuantSegments(jpegTables);
    if (stored.empty()) return std::nullopt;
    for (int quality = 1; quality <= 100; ++quality)
        if (StandardJpegQuantTables(quality, ycbcr) == stored) return quality;
    return std::nullopt;
}

}  // namespace ddbt::cog::tiff
