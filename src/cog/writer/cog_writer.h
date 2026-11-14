// SPDX-License-Identifier: MPL-2.0
//
// CogWriter: writes a Cloud Optimized GeoTIFF with GDAL's COG layout from encoded blocks that
// arrive in any order, from any thread, in one pass.
//
// Layout (what GDAL's COG driver writes, gtiffdataset_write.cpp CreateCopy):
//   header | ghost header (GDAL_STRUCTURAL_METADATA: LAYOUT=IFDS_BEFORE_DATA, BLOCK_ORDER=
//   ROW_MAJOR, BLOCK_LEADER=SIZE_AS_UINT4, BLOCK_TRAILER=LAST_4_BYTES_REPEATED,
//   KNOWN_INCOMPATIBLE_EDITION=NO, MASK_INTERLEAVED_WITH_IMAGERY=YES with a mask)
//   | IFDs: full resolution, its mask, the overviews (largest first), their masks
//   | tile data: smallest overview first ... full resolution last; in each level the blocks in
//   row-major order, each image block followed by its mask block; every block is preceded by
//   its size (uint32 LE) and followed by a copy of its last 4 bytes.
//
// How it is written without a second pass: full-resolution blocks are written straight to the
// output file (a temporary next to it) as soon as they are contiguous in row-major order (a
// reorder buffer holds the early ones); overview blocks are kept in memory, then spilled to a
// temporary file beyond a budget. Finish() builds the prefix (header, IFDs with the final
// offsets, overview data) and puts it in front of the full-resolution data: on Linux with
// fallocate(FALLOC_FL_INSERT_RANGE) when the filesystem supports it (ext4, XFS: the prefix is
// rounded up to the filesystem block, the few padding bytes after the overview data are never
// referenced), otherwise by copying the data after the prefix into a new file. Classic TIFF
// unless an offset would not fit 32 bits (decided in Finish, from the actual sizes).
//
// Sparse blocks: Put with empty bytes writes nothing (offset and size 0), as GDAL's SPARSE_OK.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "cog/encode/block_encoder.h"
#include "cog/tiff/tiff_reader.h"
#include "core/util/status.h"

namespace ddbt::cog::writer {

/// A tag copied as is to the full-resolution IFD (GDAL's TIFFTAG_* metadata items).
struct RawTag {
    uint16_t tag = 0;
    uint16_t type = 0;  // TIFF field type
    uint64_t count = 0;
    std::vector<uint8_t> data;  // little-endian values
};

struct ImageSpec {
    uint32_t width = 0, height = 0;  // full resolution
    uint32_t blockSize = 512;        // square tiles
    uint16_t samples = 1;            // samples per pixel (all bands, alpha included)
    tiff::SampleType type = tiff::SampleType::kU8;  // of each component of a complex sample
    bool complex = false;            // samples of two components (SampleFormat 5 / 6)
    uint16_t photometric = 1;        // 1 min-is-black, 2 RGB, 3 palette, 6 YCbCr (JPEG)
    std::vector<uint16_t> extraSamples;  // ExtraSamples values (empty: no tag)
    std::vector<uint16_t> colorMap;      // ColorMap of a palette image (R, G, B; empty: no tag)
    encode::Codec codec = encode::Codec::kLzw;
    int predictor = 1;
    std::vector<uint8_t> jpegTables;  // JPEGTables tag of JPEG images
    bool hasMask = false;              // 1-bit DEFLATE masks for every level
    std::string noDataText;            // GDAL_NODATA (empty: none)
    std::string gdalMetadata;          // GDAL_METADATA XML (empty: none)
    std::vector<RawTag> extraTags;     // ImageDescription, Software, resolution... (full resolution)
    std::array<double, 6> geoTransform{};  // GDAL order; EPSG:3857 (GDAL's GeoKeys)
    bool georeferenced = true;
    std::vector<std::pair<uint32_t, uint32_t>> overviews;  // largest first
};

struct WriterOptions {
    /// Bytes of overview blocks kept in memory before spilling to "<output>.ovr.<pid>.tmp".
    uint64_t overviewMemory = uint64_t{512} << 20;
    /// Try fallocate(FALLOC_FL_INSERT_RANGE) to put the prefix in front (else copy).
    bool insertRange = true;
    /// Force BigTIFF (tests).
    bool forceBigTiff = false;
    /// fdatasync before the rename.
    bool durable = false;
};

struct WriterStats {
    uint64_t fileSize = 0;
    uint64_t prefixBytes = 0;
    uint64_t fullResBytes = 0;
    uint64_t overviewBytes = 0;
    uint64_t spilledBytes = 0;
    uint64_t sparseBlocks = 0;
    bool bigTiff = false;
    bool insertedRange = false;  // false: the copy fallback ran
};

class CogWriter {
public:
    static Expected<std::unique_ptr<CogWriter>> Create(const std::string& path, ImageSpec spec,
                                                       const WriterOptions& options = {});
    ~CogWriter();
    CogWriter(const CogWriter&) = delete;
    CogWriter& operator=(const CogWriter&) = delete;

    /// Number of levels (1 + overviews) and the size in blocks of a level.
    uint32_t Levels() const noexcept;
    uint32_t BlocksAcross(uint32_t level) const;
    uint32_t BlocksDown(uint32_t level) const;

    /// Hands over the encoded bytes of block (col, row) of `level` (0 = full resolution),
    /// image (mask = false) or mask. Empty bytes: sparse. Each block exactly once. Thread-safe.
    Status Put(uint32_t level, uint32_t col, uint32_t row, bool mask, encode::ByteBuffer&& bytes);

    /// Writes the prefix and publishes the file (rename onto the path). Every block must have
    /// been put.
    Status Finish();
    /// Deletes the temporaries. Idempotent; the destructor aborts an unfinished writer.
    void Abort() noexcept;

    const WriterStats& Stats() const noexcept;
    /// Bytes currently held by the reorder buffer and the in-memory overview blocks.
    uint64_t BufferedBytes() const noexcept;

    struct Impl;

private:
    explicit CogWriter(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

/// The GDAL_STRUCTURAL_METADATA ghost header GDAL writes (with the size line).
std::string GhostHeader(bool mask);

}  // namespace ddbt::cog::writer
