// SPDX-License-Identifier: MPL-2.0
//
// LAS / LAZ reader (point formats 0-10, LAS 1.0-1.4, COPC files included). Tasks: one LAZ chunk
// each (from the chunk table, also when its offset is stored as -1 with the real offset in the
// last 8 bytes of the file, as LASzip writes it on unseekable streams), or a range of LAS
// records. The waveform packets of formats 4, 5, 9 and 10 are dropped (COPC has no waveforms).
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "copc/las/las_header.h"
#include "copc/readers/point_reader.h"
#include "core/io/byte_source.h"

namespace ddbt::copc::readers {

/// One LAZ chunk of a file.
struct LazChunk {
    uint64_t offset = 0;     // file offset of the chunk
    uint64_t bytes = 0;
    uint64_t firstPoint = 0;
    uint32_t count = 0;
};

/// Reads the chunk table of a LAZ file (compressor 2 or 3). Corrupt when it is missing or
/// inconsistent with the header and the file size.
Expected<std::vector<LazChunk>> ReadLazChunkTable(const io::ByteSource& source, const las::LasMetadata& meta);

class LasReader final : public PointReader {
public:
    static Expected<std::unique_ptr<LasReader>> Open(const std::string& path, const ReaderOptions& options = {});
    static Expected<std::unique_ptr<LasReader>> Open(std::shared_ptr<const io::ByteSource> source, std::string path,
                                                     const ReaderOptions& options = {});

    const SourceInfo& Info() const noexcept override { return info_; }
    size_t TaskCount() const noexcept override;
    uint64_t TaskPoints(size_t task) const noexcept override;
    Status Read(size_t task, const OutputSpec& spec, BatchSink& sink) const override;
    Status Sample(size_t task, size_t maxPoints, const OutputSpec& spec, BatchSink& sink) const override;

    const las::LasMetadata& metadata() const noexcept { return meta_; }
    const std::vector<LazChunk>& chunks() const noexcept { return chunks_; }

    /// Decodes `count` raw input records (the file's own format and record length) of task
    /// `task`, starting at its first point, into `out`. For tests and tools.
    Status ReadRawRecords(size_t task, uint32_t count, std::vector<uint8_t>& out) const;

private:
    LasReader() = default;
    Status ReadTask(size_t task, uint32_t limit, bool stride, const OutputSpec& spec, BatchSink& sink) const;

    std::shared_ptr<const io::ByteSource> source_;
    las::LasMetadata meta_;
    SourceInfo info_;
    std::vector<LazChunk> chunks_;
    uint64_t pointsPerTask_ = 0;
};

/// Converts `count` LAS records of `inFormat` (record length `inLength`) to output records.
/// When `xyzOut` is null X / Y / Z are copied (the caller checked the quantisation is the
/// same); otherwise xyzOut receives int * inScale + inOffset.
void ConvertLasRecords(int inFormat, size_t inLength, const uint8_t* in, size_t count, const OutputSpec& spec,
                       const std::array<double, 3>& inScale, const std::array<double, 3>& inOffset, uint8_t* out,
                       double* xyzOut);

}  // namespace ddbt::copc::readers
