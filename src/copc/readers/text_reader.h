// SPDX-License-Identifier: MPL-2.0
//
// XYZ and PTS text point clouds: see text_reader.cpp.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "copc/readers/fields.h"
#include "copc/readers/point_reader.h"
#include "core/io/byte_source.h"

namespace ddbt::copc::readers {

class TextReader final : public PointReader {
public:
    /// `pts`: Leica PTS rules; otherwise XYZ (DroneDB's column detection).
    static Expected<std::unique_ptr<TextReader>> Open(const std::string& path, bool pts, const ReaderOptions& options = {});
    ~TextReader() override;

    const SourceInfo& Info() const noexcept override { return info_; }
    size_t TaskCount() const noexcept override;
    uint64_t TaskPoints(size_t task) const noexcept override;
    Status Read(size_t task, const OutputSpec& spec, BatchSink& sink) const override;

    const std::vector<Field>& fields() const noexcept { return fields_; }
    char separator() const noexcept { return sep_; }
    double colourScale() const noexcept { return colourScale_; }

private:
    TextReader();
    std::shared_ptr<const io::ByteSource> source_;
    SourceInfo info_;
    bool pts_ = false;
    std::vector<Field> fields_;
    size_t minValues_ = 3;
    char sep_ = ' ';
    bool decimalComma_ = false;
    uint64_t dataStart_ = 0;
    std::vector<uint64_t> blocks_;
    std::vector<uint64_t> firstLine_;  // PTS: line index of each block start
    uint64_t maxLine_ = UINT64_MAX;    // PTS: lines after the announced count are not read
    double colourScale_ = 257;
};

}  // namespace ddbt::copc::readers
