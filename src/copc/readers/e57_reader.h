// SPDX-License-Identifier: MPL-2.0
//
// E57 reader (ASTM E2807), without libE57 / Xerces:
//
//   - the file is read through its 1024-byte pages (1020 data bytes + a big-endian CRC-32C,
//     checked for every page read, SSE4.2 when available);
//   - the XML section is parsed by a minimal parser (elements, attributes, text, CDATA,
//     entities, comments) into a tree; data3D scans give their pose (quaternion + translation),
//     their point prototype (Float single / double, Integer, ScaledInteger fields), their
//     limits (colour, intensity) and the CompressedVector of their points;
//   - the data packets of each CompressedVector are indexed once (bytes of every bytestream in
//     every packet), so any record range is decoded on its own: the values of a field have a
//     fixed bit width, record r starts at bit r * width of the field's stream;
//   - points as PDAL's E57 reader gives them: spherical coordinates converted to cartesian,
//     the pose applied, colour and intensity multiplied by float(65535 / (max - min)) of their
//     limits (8-bit colour: x 257), points with an invalid state kept. Also kept (PDAL drops
//     them): returnIndex + 1 / returnCount as return number / count, timeStamp as GPS time.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "copc/readers/point_reader.h"
#include "core/io/byte_source.h"

namespace ddbt::copc::readers {

/// CRC-32C (Castagnoli), as E57 pages use it.
uint32_t Crc32c(const uint8_t* data, size_t n) noexcept;

/// A minimal XML element tree (for the E57 XML section).
struct XmlElement {
    std::string name;
    std::vector<std::pair<std::string, std::string>> attributes;
    std::string text;
    std::vector<XmlElement> children;
    const XmlElement* Child(std::string_view n) const noexcept;
    const std::string* Attribute(std::string_view n) const noexcept;
};
Expected<XmlElement> ParseXml(std::string_view xml);

class E57Reader final : public PointReader {
public:
    static Expected<std::unique_ptr<E57Reader>> Open(const std::string& path, const ReaderOptions& options = {});
    ~E57Reader() override;

    const SourceInfo& Info() const noexcept override { return info_; }
    size_t TaskCount() const noexcept override { return tasks_.size(); }
    uint64_t TaskPoints(size_t task) const noexcept override;
    Status Read(size_t task, const OutputSpec& spec, BatchSink& sink) const override;

    size_t ScanCount() const noexcept;

    struct Scan;

private:
    E57Reader();
    Status ReadLogical(uint64_t logical, uint8_t* out, size_t n) const;

    struct Task {
        uint32_t scan;
        uint64_t first, count;
    };
    std::shared_ptr<const io::ByteSource> source_;
    SourceInfo info_;
    uint64_t pageSize_ = 1024;
    std::vector<Scan> scans_;
    std::vector<Task> tasks_;
};

}  // namespace ddbt::copc::readers
