// SPDX-License-Identifier: MPL-2.0
//
// Point cloud input readers of the COPC builder: LAS / LAZ (every point format 0-10), PLY
// (ASCII, binary little / big endian), E57 and XYZ / PTS text.
//
// A reader splits its input into independent units ("tasks": a LAZ chunk, a range of LAS
// records, a newline-aligned byte range of a text file, a stride of PLY vertices, an E57 data
// packet range) and decodes any of them on any thread, writing LAS 1.4 point records of
// format 6, 7 or 8 (the COPC output format) in batches to a sink:
//
//   auto reader = OpenReader(path);                  // Expected<unique_ptr<PointReader>>
//   reader->Info();                                  // count, bounds, attributes, CRS, ...
//   reader->Read(task, spec, sink);                  // thread-safe, const
//
// Conversions (the same for every reader, as PDAL reads and writes them): formats 0-5 become 6+ (return
// number / count widened to 4 bits, classification 12 "overlap" becomes class 0 + the overlap
// flag as PDAL reads it, synthetic / key-point / withheld moved to the class flags, scan angle
// rank in degrees becomes round(deg / 0.006) in float as PDAL writes it); 8-bit colour of
// non-LAS inputs is scaled by 257; PTS intensity i becomes (i + 2048) * 16.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "copc/las/las_header.h"
#include "core/util/status.h"

namespace ddbt::copc::readers {

/// Output record layout: LAS 1.4 point data record format 6, 7 or 8, then `extraBytes` bytes.
struct OutputSpec {
    int format = 6;
    uint16_t extraBytes = 0;
    /// Quantisation of X / Y / Z (value = int * scale + offset).
    std::array<double, 3> scale{0.001, 0.001, 0.001};
    std::array<double, 3> offset{0, 0, 0};
    /// False in the planning pass (scale / offset unknown): readers then give coordinates as
    /// doubles (Batch::xyz).
    bool quantized = true;

    size_t RecordSize() const noexcept { return BaseSize() + extraBytes; }
    size_t BaseSize() const noexcept { return format == 8 ? 38 : format == 7 ? 36 : 30; }
};

/// Field offsets of formats 6-8.
namespace rec {
inline constexpr size_t kX = 0, kY = 4, kZ = 8, kIntensity = 12, kReturns = 14, kFlags = 15, kClass = 16,
                        kUserData = 17, kScanAngle = 18, kPointSource = 20, kGpsTime = 22, kRgb = 30, kNir = 36;
}

/// Records handed to a sink. When `xyz` is not null the X / Y / Z fields of the records are not
/// set and xyz[3 i .. 3 i + 2] holds point i's coordinates; otherwise the records hold X / Y / Z
/// quantised with the spec. The sink may modify the records (e.g. write X / Y / Z).
struct Batch {
    size_t count = 0;
    uint8_t* records = nullptr;
    const double* xyz = nullptr;
};

class BatchSink {
public:
    virtual ~BatchSink() = default;
    virtual Status Consume(Batch& batch) = 0;
};

/// What the builder needs to know about an input before reading it.
struct SourceInfo {
    std::string path;
    std::string kind;  // "las", "laz", "ply", "e57", "xyz", "pts"
    uint64_t pointCount = 0;
    bool countExact = true;
    /// Bounds from the header that can be trusted for planning (LAS / LAZ); otherwise the
    /// builder runs a bounds pass over the data.
    bool boundsKnown = false;
    std::array<double, 3> min{}, max{};

    bool hasRgb = false;
    bool hasNir = false;
    bool hasIntensity = false;
    bool hasGpsTime = false;
    /// Colour is 16-bit as stored (LAS); false: 8-bit values scaled by 257 by the reader.
    bool rgb16 = false;

    // LAS / LAZ only.
    bool isLas = false;
    int lasFormat = -1;
    std::array<double, 3> lasScale{1, 1, 1};
    std::array<double, 3> lasOffset{0, 0, 0};
    uint16_t lasExtraBytes = 0;
    /// The extra-bytes VLR payload as stored (descriptors), empty when absent.
    std::vector<uint8_t> extraBytesVlr;
    std::vector<las::ExtraBytesDescriptor> extraBytes;
    las::LasHeader lasHeader;

    /// CRS as WKT (WKT1 as PDAL writes it in a COPC), empty when unknown.
    std::string wkt;
    /// Dimension names as PDAL calls them, in PDAL's order.
    std::vector<std::string> dimensions;
};

class PointReader {
public:
    virtual ~PointReader() = default;
    virtual const SourceInfo& Info() const noexcept = 0;
    virtual size_t TaskCount() const noexcept = 0;
    /// Approximate number of points of a task (exact for LAS / LAZ).
    virtual uint64_t TaskPoints(size_t task) const noexcept = 0;
    /// Decodes task `task` into `spec` records, in one or more batches. Thread-safe.
    virtual Status Read(size_t task, const OutputSpec& spec, BatchSink& sink) const = 0;
    /// A cheap spatial sample of a task for planning: about `maxPoints` of its points. The
    /// default reads the whole task.
    virtual Status Sample(size_t task, size_t maxPoints, const OutputSpec& spec, BatchSink& sink) const {
        (void)maxPoints;
        return Read(task, spec, sink);
    }
};

struct ReaderOptions {
    /// Points per task of formats that can be cut anywhere (LAS, PLY binary).
    uint64_t pointsPerTask = 1u << 18;
    /// Bytes per task of text formats.
    uint64_t bytesPerTask = 8u << 20;
};

/// Opens a reader for `path` by its content (LAS signature, "ply", E57 signature) and its
/// extension (.xyz, .pts, .txt).
Expected<std::unique_ptr<PointReader>> OpenReader(const std::string& path, const ReaderOptions& options = {});

/// Writes one output record's attributes from double coordinates `xyz` (quantised with `spec`).
/// Fails with OutOfRange when a coordinate does not fit an int32 after quantisation.
Status QuantizeXyz(const OutputSpec& spec, const double* xyz, size_t count, uint8_t* records);

}  // namespace ddbt::copc::readers
