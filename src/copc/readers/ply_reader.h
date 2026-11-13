// SPDX-License-Identifier: MPL-2.0
//
// PLY point clouds (ASCII and binary, little / big endian): see ply_reader.cpp.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "copc/readers/point_reader.h"
#include "core/io/byte_source.h"

namespace ddbt::copc::readers {

class PlyReader final : public PointReader {
public:
    static Expected<std::unique_ptr<PlyReader>> Open(const std::string& path, const ReaderOptions& options = {});
    ~PlyReader() override;

    const SourceInfo& Info() const noexcept override { return info_; }
    size_t TaskCount() const noexcept override;
    uint64_t TaskPoints(size_t task) const noexcept override;
    Status Read(size_t task, const OutputSpec& spec, BatchSink& sink) const override;

    /// Colour multiplier (257 for 8-bit colour).
    double colourScale() const noexcept { return colourScale_; }

private:
    PlyReader();
    enum class Fmt { kAscii, kLittle, kBig };
    struct Prop;

    std::shared_ptr<const io::ByteSource> source_;
    SourceInfo info_;
    Fmt fmt_ = Fmt::kAscii;
    std::vector<Prop> props_;
    uint64_t count_ = 0;
    size_t stride_ = 0;
    uint64_t dataStart_ = 0;
    uint64_t pointsPerTask_ = 1u << 18;
    std::vector<uint64_t> blocks_;     // ASCII: task byte ranges relative to dataStart_
    std::vector<uint64_t> firstLine_;  // ASCII: line index of each block start
    double colourScale_ = 257;
};

}  // namespace ddbt::copc::readers
