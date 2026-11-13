// SPDX-License-Identifier: MPL-2.0
//
// LAS 1.0-1.4 public header block, VLRs and EVLRs, and the records ddb-tilers interprets:
// WKT (LASF_Projection 2112), GeoTIFF keys (LASF_Projection 34735/34736/34737), extra-bytes
// descriptors (LASF_Spec 4), the LAZ VLR ("laszip encoded" 22204), the COPC info VLR ("copc" 1)
// and the COPC hierarchy EVLR ("copc" 1000).
//
//   auto meta = las::ReadMetadata(source);        // Expected<LasMetadata>
//   meta->header.pointFormat, meta->copcInfo->spacing, ...
//
// ReadMetadata reads the header, the VLR area (one read of [header size, offset to points)) and
// the EVLR headers (one 60-byte read each). Payloads are kept for the records above and for
// any VLR / EVLR up to ReadOptions::maxKeptPayload bytes; the COPC hierarchy EVLR payload is
// never read here (it is the hierarchy, read by HierarchyIndex page by page). Every size and
// offset is checked against the source size: a malformed file gives Corrupt / Unsupported,
// never an out-of-bounds read.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "copc/las/point_format.h"
#include "core/io/byte_source.h"
#include "core/util/status.h"

namespace ddbt::copc::las {

inline constexpr size_t kHeaderSize12 = 227;  // 1.0 - 1.2
inline constexpr size_t kHeaderSize13 = 235;
inline constexpr size_t kHeaderSize14 = 375;
inline constexpr size_t kVlrHeaderSize = 54;
inline constexpr size_t kEvlrHeaderSize = 60;
inline constexpr size_t kCopcInfoSize = 160;
inline constexpr size_t kExtraBytesDescriptorSize = 192;

/// Global encoding bits.
inline constexpr uint16_t kGlobalEncodingGpsStandard = 1u << 0;  // adjusted standard GPS time
inline constexpr uint16_t kGlobalEncodingWkt = 1u << 4;          // CRS is WKT (1.4)

struct LasHeader {
    uint8_t versionMajor = 1;
    uint8_t versionMinor = 0;
    uint16_t fileSourceId = 0;    // 1.0: reserved
    uint16_t globalEncoding = 0;  // 1.0: reserved
    std::array<uint8_t, 16> projectId{};
    std::string systemIdentifier;    // trailing NULs and spaces removed
    std::string generatingSoftware;  // trailing NULs and spaces removed
    uint16_t creationDayOfYear = 0;
    uint16_t creationYear = 0;
    uint16_t headerSize = 0;
    uint32_t pointDataOffset = 0;
    uint32_t vlrCount = 0;
    uint8_t pointFormat = 0;   // 0..10, compression bits removed
    bool compressed = false;   // bit 7 (or bit 6, old LASzip) of the format byte: LAZ
    uint16_t pointRecordLength = 0;
    uint32_t legacyPointCount = 0;
    std::array<uint32_t, 5> legacyPointsByReturn{};
    std::array<double, 3> scale{};
    std::array<double, 3> offset{};
    std::array<double, 3> min{};
    std::array<double, 3> max{};
    uint64_t waveformDataOffset = 0;  // 1.3+
    uint64_t evlrOffset = 0;          // 1.4
    uint32_t evlrCount = 0;           // 1.4
    uint64_t pointCount = 0;          // 1.4: the 64-bit count; earlier: the legacy count
    std::array<uint64_t, 15> pointsByReturn{};  // 1.4; earlier: the legacy counts

    bool VersionAtLeast(int major, int minor) const noexcept {
        return versionMajor > major || (versionMajor == major && versionMinor >= minor);
    }
    PointLayout Layout() const noexcept { return LayoutOf(pointFormat); }
    /// Extra bytes per record (record length - base size of the format).
    uint16_t ExtraBytesPerPoint() const noexcept {
        return static_cast<uint16_t>(pointRecordLength - BaseRecordSize(pointFormat));
    }
};

/// Parses the public header block from the first bytes of a file (at least the header size
/// of its version, kHeaderSize12 for 1.0-1.2). Checks the signature, version 1.0-1.4, header
/// size, format 0-10, record length >= base size, offset to points >= header size, non-zero
/// finite scales.
Expected<LasHeader> ParseHeader(std::span<const uint8_t> bytes);

/// A VLR or EVLR. `payload` holds the data when it was kept (see ReadOptions).
struct VlrRecord {
    std::string userId;       // up to 16 characters, NULs removed
    uint16_t recordId = 0;
    std::string description;  // up to 32 characters, NULs removed
    bool extended = false;    // EVLR
    uint64_t headerOffset = 0;
    uint64_t dataOffset = 0;  // file offset of the payload
    uint64_t dataSize = 0;
    bool payloadKept = false;
    std::vector<uint8_t> payload;

    bool Is(std::string_view user, uint16_t record) const noexcept { return userId == user && recordId == record; }
};

/// COPC info VLR ("copc" record 1, 160 bytes, the first VLR at offset 375).
struct CopcInfo {
    double centerX = 0, centerY = 0, centerZ = 0;
    double halfSize = 0;
    double spacing = 0;
    uint64_t rootHierOffset = 0;
    uint64_t rootHierSize = 0;
    double gpsTimeMin = 0, gpsTimeMax = 0;
};

/// LAZ VLR ("laszip encoded" record 22204).
struct LazVlr {
    struct Item {
        uint16_t type = 0, size = 0, version = 0;
    };
    uint16_t compressor = 0;  // 0 none, 1 pointwise, 2 pointwise chunked, 3 layered chunked
    uint16_t coder = 0;
    uint8_t versionMajor = 0, versionMinor = 0;
    uint16_t versionRevision = 0;
    uint32_t options = 0;
    uint32_t chunkSize = 0;  // 0xFFFFFFFF: variable-size chunks
    int64_t specialEvlrCount = 0;
    int64_t specialEvlrOffset = 0;
    std::vector<Item> items;

    bool VariableChunks() const noexcept { return chunkSize == 0xFFFFFFFFu; }
};

/// One extra-bytes descriptor (LASF_Spec record 4, 192 bytes each).
struct ExtraBytesDescriptor {
    uint8_t dataType = 0;  // 0: undocumented (`options` bytes), 1-10: u8 i8 u16 i16 u32 i32 u64 i64 f32 f64,
                           // 11-30: deprecated 2- and 3-element arrays of those
    uint8_t options = 0;   // bit 0 no_data, 1 min, 2 max, 3 scale, 4 offset are meaningful
    std::string name;
    std::string description;
    std::array<uint8_t, 24> noData{}, minValue{}, maxValue{};  // raw "anytype" values, 8 bytes per element
    std::array<double, 3> scale{1, 1, 1};
    std::array<double, 3> offset{0, 0, 0};
    uint16_t byteOffset = 0;  // offset inside the extra bytes of a record
    uint16_t size = 0;        // bytes per record
    uint8_t elements = 1;     // 1, 2 or 3

    bool HasNoData() const noexcept { return (options & 1u) != 0; }
    bool HasScale() const noexcept { return (options & 8u) != 0; }
    bool HasOffset() const noexcept { return (options & 16u) != 0; }
    /// Element `i` of a raw anytype value as a double (integer types as integers).
    double AnyAsDouble(const std::array<uint8_t, 24>& v, int i = 0) const noexcept;
};

/// Bytes per element of extra-bytes data type 1..10 (0 when unknown).
uint16_t ExtraBytesTypeSize(uint8_t baseType) noexcept;

/// Parses an extra-bytes VLR payload. Descriptors beyond `extraBytesPerPoint` bytes give
/// Corrupt (PDAL: "Extra byte specification exceeds point length").
Expected<std::vector<ExtraBytesDescriptor>> ParseExtraBytes(std::span<const uint8_t> payload,
                                                            uint16_t extraBytesPerPoint);
Expected<CopcInfo> ParseCopcInfo(std::span<const uint8_t> payload);
Expected<LazVlr> ParseLazVlr(std::span<const uint8_t> payload);

/// The GeoTIFF key VLRs as stored (LASF_Projection 34735 / 34736 / 34737).
struct GeoKeyVlrs {
    std::vector<uint16_t> directory;
    std::vector<double> doubles;
    std::string ascii;
};

struct ReadOptions {
    /// VLR / EVLR payloads up to this size are kept even when not interpreted.
    uint64_t maxKeptPayload = 64 * 1024;
    /// Limit on the size of an interpreted payload (WKT, GeoKeys, extra bytes, LAZ VLR).
    uint64_t maxInterpretedPayload = 16 * 1024 * 1024;
};

struct LasMetadata {
    LasHeader header;
    std::vector<VlrRecord> vlrs;   // in file order
    std::vector<VlrRecord> evlrs;  // in file order
    std::optional<std::string> wkt;       // LASF_Projection 2112 (VLR or EVLR), cut at the first NUL
    std::optional<std::string> wkt2;      // LASF_Projection 4224 (WKT2, written by PDAL)
    std::optional<std::string> projJson;  // PDAL 4225
    std::optional<GeoKeyVlrs> geoKeys;
    std::vector<ExtraBytesDescriptor> extraBytes;
    std::optional<LazVlr> laz;
    std::optional<CopcInfo> copcInfo;         // only when it is the first VLR at offset 375
    std::optional<VlrRecord> copcHierarchy;   // EVLR "copc" 1000 (payload not read)
    /// Problems that did not stop the read (unparsable optional records), one line each.
    std::vector<std::string> warnings;

    const VlrRecord* FindVlr(std::string_view user, uint16_t record) const noexcept;
    bool IsCopc() const noexcept { return copcInfo.has_value(); }
};

/// Reads the header, VLRs and EVLR headers of a LAS / LAZ / COPC file.
Expected<LasMetadata> ReadMetadata(const io::ByteSource& source, const ReadOptions& options = {});

}  // namespace ddbt::copc::las
