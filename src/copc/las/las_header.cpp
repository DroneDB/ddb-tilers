// SPDX-License-Identifier: MPL-2.0
#include "copc/las/las_header.h"

#include <cmath>
#include <cstring>
#include <string>

#include "core/util/checked_math.h"
#include "core/util/endian.h"

namespace ddbt::copc::las {
namespace {

// Up to `n` characters, cut at the first NUL.
std::string FixedString(const uint8_t* p, size_t n) {
    size_t len = 0;
    while (len < n && p[len] != 0) ++len;
    return std::string(reinterpret_cast<const char*>(p), len);
}

// FixedString without trailing spaces (system identifier, generating software).
std::string TrimmedString(const uint8_t* p, size_t n) {
    std::string s = FixedString(p, n);
    while (!s.empty() && s.back() == ' ') s.pop_back();
    return s;
}

double F64(const uint8_t* p) { return LoadLE<double>(p); }

bool InterpretedRecord(const VlrRecord& r) {
    if (r.userId == "LASF_Projection")
        return r.recordId == 2112 || r.recordId == 4224 || r.recordId == 34735 || r.recordId == 34736 ||
               r.recordId == 34737;
    if (r.userId == "PDAL") return r.recordId == 4225;
    if (r.userId == "LASF_Spec") return r.recordId == 4;
    if (r.userId == "laszip encoded") return r.recordId == 22204;
    if (r.userId == "copc") return r.recordId == 1;
    return false;
}

// A text payload cut at its first NUL (PDAL's extractWkt).
std::string TextPayload(std::span<const uint8_t> p) {
    std::string w(reinterpret_cast<const char*>(p.data()), p.size());
    const size_t nul = w.find('\0');
    if (nul != std::string::npos) w.resize(nul);
    return w;
}

// Interprets a record whose payload is in memory. The first occurrence of each record wins.
void Interpret(LasMetadata& m, const VlrRecord& r) {
    if (!r.payloadKept) return;
    std::span<const uint8_t> p(r.payload);
    if (r.Is("PDAL", 4225) && !m.projJson) {
        m.projJson = TextPayload(p);
    } else if (r.userId == "LASF_Projection") {
        if (r.recordId == 2112 && !m.wkt) {
            m.wkt = TextPayload(p);
        } else if (r.recordId == 4224 && !m.wkt2) {
            m.wkt2 = TextPayload(p);
        } else if (r.recordId >= 34735 && r.recordId <= 34737) {
            if (!m.geoKeys) m.geoKeys.emplace();
            GeoKeyVlrs& g = *m.geoKeys;
            if (r.recordId == 34735 && g.directory.empty()) {
                g.directory.resize(p.size() / 2);
                for (size_t i = 0; i < g.directory.size(); ++i) g.directory[i] = LoadLE<uint16_t>(p.data() + 2 * i);
            } else if (r.recordId == 34736 && g.doubles.empty()) {
                g.doubles.resize(p.size() / 8);
                for (size_t i = 0; i < g.doubles.size(); ++i) g.doubles[i] = F64(p.data() + 8 * i);
            } else if (r.recordId == 34737 && g.ascii.empty()) {
                g.ascii.assign(reinterpret_cast<const char*>(p.data()), p.size());
            }
        }
    } else if (r.Is("LASF_Spec", 4) && m.extraBytes.empty()) {
        auto eb = ParseExtraBytes(p, m.header.ExtraBytesPerPoint());
        if (eb.ok())
            m.extraBytes = std::move(*eb);
        else
            m.warnings.push_back("extra bytes VLR ignored: " + eb.status().ToString());
    } else if (r.Is("laszip encoded", 22204) && !m.laz) {
        auto laz = ParseLazVlr(p);
        if (laz.ok())
            m.laz = std::move(*laz);
        else
            m.warnings.push_back("LAZ VLR ignored: " + laz.status().ToString());
    }
}

}  // namespace

Expected<LasHeader> ParseHeader(std::span<const uint8_t> b) {
    if (b.size() < kHeaderSize12) return Corrupt("LAS header: file shorter than 227 bytes");
    if (std::memcmp(b.data(), "LASF", 4) != 0) return Corrupt("LAS header: no LASF signature");
    LasHeader h;
    h.versionMajor = b[24];
    h.versionMinor = b[25];
    if (h.versionMajor != 1 || h.versionMinor > 4)
        return Unsupported("LAS header: version " + std::to_string(h.versionMajor) + "." +
                           std::to_string(h.versionMinor) + " (1.0 to 1.4 are read)");
    h.fileSourceId = LoadLE<uint16_t>(&b[4]);
    h.globalEncoding = LoadLE<uint16_t>(&b[6]);
    std::memcpy(h.projectId.data(), &b[8], 16);
    h.systemIdentifier = TrimmedString(&b[26], 32);
    h.generatingSoftware = TrimmedString(&b[58], 32);
    h.creationDayOfYear = LoadLE<uint16_t>(&b[90]);
    h.creationYear = LoadLE<uint16_t>(&b[92]);
    h.headerSize = LoadLE<uint16_t>(&b[94]);
    const size_t required = h.versionMinor >= 4 ? kHeaderSize14 : h.versionMinor == 3 ? kHeaderSize13 : kHeaderSize12;
    if (h.headerSize < required)
        return Corrupt("LAS header: header size " + std::to_string(h.headerSize) + " < " + std::to_string(required) +
                       " for version 1." + std::to_string(h.versionMinor));
    if (b.size() < required) return Corrupt("LAS header: truncated");
    h.pointDataOffset = LoadLE<uint32_t>(&b[96]);
    h.vlrCount = LoadLE<uint32_t>(&b[100]);
    const uint8_t formatByte = b[104];
    h.compressed = (formatByte & 0xC0) != 0;
    h.pointFormat = formatByte & 0x3F;
    if (h.pointFormat > kMaxPointFormat)
        return Unsupported("LAS header: point data record format " + std::to_string(h.pointFormat));
    h.pointRecordLength = LoadLE<uint16_t>(&b[105]);
    if (h.pointRecordLength < BaseRecordSize(h.pointFormat))
        return Corrupt("LAS header: record length " + std::to_string(h.pointRecordLength) + " < " +
                       std::to_string(BaseRecordSize(h.pointFormat)) + " for format " +
                       std::to_string(h.pointFormat));
    h.legacyPointCount = LoadLE<uint32_t>(&b[107]);
    for (size_t i = 0; i < 5; ++i) h.legacyPointsByReturn[i] = LoadLE<uint32_t>(&b[111 + 4 * i]);
    for (size_t i = 0; i < 3; ++i) {
        h.scale[i] = F64(&b[131 + 8 * i]);
        h.offset[i] = F64(&b[155 + 8 * i]);
        h.max[i] = F64(&b[179 + 16 * i]);
        h.min[i] = F64(&b[187 + 16 * i]);
        if (!(std::isfinite(h.scale[i]) && h.scale[i] != 0.0) || !std::isfinite(h.offset[i]))
            return Corrupt("LAS header: scale / offset of axis " + std::to_string(i) + " is zero or not finite");
    }
    if (h.pointDataOffset < h.headerSize)
        return Corrupt("LAS header: offset to point data " + std::to_string(h.pointDataOffset) + " < header size");
    if (h.versionMinor >= 3) h.waveformDataOffset = LoadLE<uint64_t>(&b[227]);
    h.pointCount = h.legacyPointCount;
    for (size_t i = 0; i < 5; ++i) h.pointsByReturn[i] = h.legacyPointsByReturn[i];
    if (h.versionMinor >= 4) {
        h.evlrOffset = LoadLE<uint64_t>(&b[235]);
        h.evlrCount = LoadLE<uint32_t>(&b[243]);
        const uint64_t count64 = LoadLE<uint64_t>(&b[247]);
        if (count64 != 0 || h.legacyPointCount == 0) h.pointCount = count64;
        for (size_t i = 0; i < 15; ++i) h.pointsByReturn[i] = LoadLE<uint64_t>(&b[255 + 8 * i]);
    }
    return h;
}

uint16_t ExtraBytesTypeSize(uint8_t t) noexcept {
    static constexpr uint16_t kSizes[11] = {0, 1, 1, 2, 2, 4, 4, 8, 8, 4, 8};
    return t <= 10 ? kSizes[t] : 0;
}

double ExtraBytesDescriptor::AnyAsDouble(const std::array<uint8_t, 24>& v, int i) const noexcept {
    if (dataType == 0 || i < 0 || i > 2) return 0;
    const uint8_t base = static_cast<uint8_t>((dataType - 1) % 10 + 1);
    const uint8_t* p = v.data() + 8 * i;
    if (base == 9 || base == 10) return LoadLE<double>(p);
    if (base % 2 == 1) return static_cast<double>(LoadLE<uint64_t>(p));
    return static_cast<double>(LoadLE<int64_t>(p));
}

Expected<std::vector<ExtraBytesDescriptor>> ParseExtraBytes(std::span<const uint8_t> p, uint16_t extraBytesPerPoint) {
    if (p.size() % kExtraBytesDescriptorSize != 0)
        return Corrupt("extra bytes VLR: size " + std::to_string(p.size()) + " is not a multiple of 192");
    std::vector<ExtraBytesDescriptor> out;
    uint32_t used = 0;
    for (size_t at = 0; at < p.size(); at += kExtraBytesDescriptorSize) {
        const uint8_t* d = p.data() + at;
        ExtraBytesDescriptor e;
        e.dataType = d[2];
        e.options = d[3];
        e.name = FixedString(d + 4, 32);
        std::memcpy(e.noData.data(), d + 40, 24);
        std::memcpy(e.minValue.data(), d + 64, 24);
        std::memcpy(e.maxValue.data(), d + 88, 24);
        for (size_t i = 0; i < 3; ++i) {
            e.scale[i] = F64(d + 112 + 8 * i);
            e.offset[i] = F64(d + 136 + 8 * i);
        }
        e.description = FixedString(d + 160, 32);
        if (e.dataType == 0) {
            e.size = e.options;  // undocumented bytes: the options field holds their count
            e.elements = 1;
        } else if (e.dataType <= 30) {
            e.elements = static_cast<uint8_t>((e.dataType - 1) / 10 + 1);
            e.size = static_cast<uint16_t>(ExtraBytesTypeSize(static_cast<uint8_t>((e.dataType - 1) % 10 + 1)) * e.elements);
        } else {
            return Corrupt("extra bytes VLR: data type " + std::to_string(e.dataType) + " of '" + e.name + "'");
        }
        e.byteOffset = static_cast<uint16_t>(used);
        used += e.size;
        if (used > extraBytesPerPoint)
            return Corrupt("extra bytes VLR: descriptors need " + std::to_string(used) + " bytes, records have " +
                           std::to_string(extraBytesPerPoint));
        out.push_back(std::move(e));
    }
    return out;
}

Expected<CopcInfo> ParseCopcInfo(std::span<const uint8_t> p) {
    if (p.size() != kCopcInfoSize) return Corrupt("COPC info VLR: " + std::to_string(p.size()) + " bytes, not 160");
    CopcInfo c;
    c.centerX = F64(&p[0]);
    c.centerY = F64(&p[8]);
    c.centerZ = F64(&p[16]);
    c.halfSize = F64(&p[24]);
    c.spacing = F64(&p[32]);
    c.rootHierOffset = LoadLE<uint64_t>(&p[40]);
    c.rootHierSize = LoadLE<uint64_t>(&p[48]);
    c.gpsTimeMin = F64(&p[56]);
    c.gpsTimeMax = F64(&p[64]);
    if (!std::isfinite(c.centerX) || !std::isfinite(c.centerY) || !std::isfinite(c.centerZ))
        return Corrupt("COPC info VLR: center is not finite");
    if (!(std::isfinite(c.halfSize) && c.halfSize > 0)) return Corrupt("COPC info VLR: halfsize is not positive");
    if (!(std::isfinite(c.spacing) && c.spacing > 0)) return Corrupt("COPC info VLR: spacing is not positive");
    return c;
}

Expected<LazVlr> ParseLazVlr(std::span<const uint8_t> p) {
    if (p.size() < 34) return Corrupt("LAZ VLR: shorter than 34 bytes");
    LazVlr v;
    v.compressor = LoadLE<uint16_t>(&p[0]);
    v.coder = LoadLE<uint16_t>(&p[2]);
    v.versionMajor = p[4];
    v.versionMinor = p[5];
    v.versionRevision = LoadLE<uint16_t>(&p[6]);
    v.options = LoadLE<uint32_t>(&p[8]);
    v.chunkSize = LoadLE<uint32_t>(&p[12]);
    v.specialEvlrCount = LoadLE<int64_t>(&p[16]);
    v.specialEvlrOffset = LoadLE<int64_t>(&p[24]);
    const uint16_t n = LoadLE<uint16_t>(&p[32]);
    if (p.size() < 34 + size_t{6} * n) return Corrupt("LAZ VLR: " + std::to_string(n) + " items do not fit");
    for (uint16_t i = 0; i < n; ++i) {
        const uint8_t* it = &p[34 + 6 * size_t{i}];
        v.items.push_back({LoadLE<uint16_t>(it), LoadLE<uint16_t>(it + 2), LoadLE<uint16_t>(it + 4)});
    }
    return v;
}

const VlrRecord* LasMetadata::FindVlr(std::string_view user, uint16_t record) const noexcept {
    for (const VlrRecord& r : vlrs)
        if (r.Is(user, record)) return &r;
    for (const VlrRecord& r : evlrs)
        if (r.Is(user, record)) return &r;
    return nullptr;
}

Expected<LasMetadata> ReadMetadata(const io::ByteSource& source, const ReadOptions& options) {
    const uint64_t fileSize = source.Size();
    LasMetadata m;

    std::vector<uint8_t> head(static_cast<size_t>(std::min<uint64_t>(fileSize, kHeaderSize14)));
    DDBT_TRY(source.Read(0, head));
    DDBT_ASSIGN_OR_RETURN(m.header, ParseHeader(head));
    const LasHeader& h = m.header;
    if (h.pointDataOffset > fileSize) return Corrupt("LAS: offset to point data is past the end of the file");

    // VLRs: one read of the area between the header and the point data.
    const uint64_t vlrArea = h.pointDataOffset - h.headerSize;
    if (h.vlrCount > 0) {
        if (vlrArea > (uint64_t{256} << 20)) return Unsupported("LAS: VLR area larger than 256 MiB");
        std::vector<uint8_t> area(static_cast<size_t>(vlrArea));
        DDBT_TRY(source.Read(h.headerSize, area));
        uint64_t at = 0;
        for (uint32_t i = 0; i < h.vlrCount; ++i) {
            if (!RangeInside(at, kVlrHeaderSize, area.size()))
                return Corrupt("LAS: VLR " + std::to_string(i) + " header extends past the offset to point data");
            const uint8_t* v = area.data() + at;
            VlrRecord r;
            r.userId = FixedString(v + 2, 16);
            r.recordId = LoadLE<uint16_t>(v + 18);
            r.dataSize = LoadLE<uint16_t>(v + 20);
            r.description = FixedString(v + 22, 32);
            r.headerOffset = h.headerSize + at;
            r.dataOffset = r.headerOffset + kVlrHeaderSize;
            if (!RangeInside(at + kVlrHeaderSize, r.dataSize, area.size()))
                return Corrupt("LAS: VLR " + std::to_string(i) + " (" + r.userId + " " + std::to_string(r.recordId) +
                               ") extends past the offset to point data");
            r.payloadKept = true;  // VLR payloads are at most 65535 bytes and already read
            r.payload.assign(v + kVlrHeaderSize, v + kVlrHeaderSize + r.dataSize);
            at += kVlrHeaderSize + r.dataSize;
            m.vlrs.push_back(std::move(r));
        }
    }

    // COPC: the info VLR must be the first VLR, right after a 375-byte header.
    for (size_t i = 1; i < m.vlrs.size(); ++i)
        if (m.vlrs[i].Is("copc", 1)) m.warnings.push_back("COPC info VLR is not the first VLR: not read as COPC");
    if (!m.vlrs.empty() && m.vlrs.front().Is("copc", 1)) {
        if (m.vlrs.front().headerOffset != kHeaderSize14) {
            m.warnings.push_back("COPC info VLR not at offset 375: not read as COPC");
        } else {
            DDBT_ASSIGN_OR_RETURN(m.copcInfo, ParseCopcInfo(m.vlrs.front().payload));
        }
    }

    // EVLRs: the headers one by one, payloads only when wanted (never the COPC hierarchy).
    if (h.VersionAtLeast(1, 4) && h.evlrCount > 0) {
        if (h.evlrOffset < h.pointDataOffset || h.evlrOffset > fileSize)
            return Corrupt("LAS: EVLR offset " + std::to_string(h.evlrOffset) + " outside the point data / file");
        uint64_t at = h.evlrOffset;
        for (uint32_t i = 0; i < h.evlrCount; ++i) {
            if (!RangeInside(at, kEvlrHeaderSize, fileSize))
                return Corrupt("LAS: EVLR " + std::to_string(i) + " header past the end of the file");
            uint8_t eh[kEvlrHeaderSize];
            DDBT_TRY(source.Read(at, std::span<uint8_t>(eh, kEvlrHeaderSize)));
            VlrRecord r;
            r.extended = true;
            r.userId = FixedString(eh + 2, 16);
            r.recordId = LoadLE<uint16_t>(eh + 18);
            r.dataSize = LoadLE<uint64_t>(eh + 20);
            r.description = FixedString(eh + 28, 32);
            r.headerOffset = at;
            r.dataOffset = at + kEvlrHeaderSize;
            if (!RangeInside(r.dataOffset, r.dataSize, fileSize))
                return Corrupt("LAS: EVLR " + std::to_string(i) + " (" + r.userId + " " + std::to_string(r.recordId) +
                               ") extends past the end of the file");
            const bool hierarchy = r.Is("copc", 1000);
            const bool interpreted = InterpretedRecord(r);
            if (!hierarchy && ((interpreted && r.dataSize <= options.maxInterpretedPayload) ||
                               r.dataSize <= options.maxKeptPayload)) {
                r.payload.resize(static_cast<size_t>(r.dataSize));
                DDBT_TRY(source.Read(r.dataOffset, r.payload));
                r.payloadKept = true;
            } else if (interpreted && !hierarchy) {
                m.warnings.push_back("EVLR " + r.userId + " " + std::to_string(r.recordId) + " too large, not read");
            }
            at = r.dataOffset + r.dataSize;
            if (hierarchy && !m.copcHierarchy) m.copcHierarchy = r;
            m.evlrs.push_back(std::move(r));
        }
    }

    for (const VlrRecord& r : m.vlrs) Interpret(m, r);
    for (const VlrRecord& r : m.evlrs) Interpret(m, r);
    return m;
}

}  // namespace ddbt::copc::las
