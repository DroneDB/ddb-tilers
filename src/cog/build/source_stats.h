// SPDX-License-Identifier: MPL-2.0
//
// SourceStatistics: the exact band statistics of the input of a COG build (the pixels of the file,
// not of the COG: before the warp, the resampling and the encoding), counted while the build reads
// them, so that a host never reads every pixel of the input again for them.
//
// - It wraps the source the input file opens to (under the georeferencing of a photo and its
//   resampling): every block of the full resolution is counted the first time a read of the build
//   meets it, whichever way it is read (ReadBlock: TIFF, JPEG 2000 and a photo spilled to a scratch
//   file; ResidentBlock: a photo in memory; SharedDataBlock), and never again (a flag per block):
//   the blocks a warp reads twice, or a second pass of the build reads again (a graded
//   transparency), count once. Finish() reads and counts the blocks no read of the build met (a
//   warp that needed none of their pixels).
// - The sums are exact (stats_kernel.h's ExactSums), one per block and band, merged in block order
//   at the end: the result does not depend on the order the blocks were read in nor on the thread
//   count. A block's sums are its own (set, not added to), so only the flag keeps two reads from
//   counting one block at once (and from counting it again for nothing). The rules are those of the tiler's exact pass (tiler/stats.h) over the file: a sample
//   has no value when it is not finite, equals the NoData value of the bands (compared in the
//   precision of the sample), or its pixel is 0 in the per-dataset mask or, without NoData nor mask,
//   0 in the alpha band of a 2- or 4-band 8 / 16-bit raster (GDAL's mask rule); a complex band is
//   measured by its magnitude, a Float16 band as Float32, a 64-bit integer band as Float64; the
//   values are raw (no scale or offset). Every band has statistics, the alpha band too.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "cog/build/derived_source.h"
#include "cog/build/stats.h"
#include "cog/tiler/stats_kernel.h"

namespace ddbt::cog::build {

class SourceStatistics final : public ForwardingSource {
public:
    explicit SourceStatistics(std::shared_ptr<const RasterSource> inner);

    Expected<uint32_t> ReadBlock(uint32_t col, uint32_t row, uint8_t* out) const override;
    const uint8_t* ResidentBlock(uint32_t col, uint32_t row) const override;
    SharedBlock SharedDataBlock(uint32_t col, uint32_t row) const override;

    /// Counts the blocks no read met, then the statistics of each band (nullopt: no pixel of the
    /// band has a value). Once the build is done (no read in flight).
    Expected<std::vector<std::optional<BandStats>>> Finish() const;

    /// Blocks counted by the reads of the build, and by Finish() (tests, reporting).
    uint64_t BlocksCountedByReads() const noexcept { return countedByReads_.load(std::memory_order_relaxed); }
    uint64_t BlocksCountedAfter() const noexcept { return countedAfter_; }

private:
    // The first read of block (col, row) takes it to count it: false when another read took it.
    bool Claim(uint32_t col, uint32_t row) const;
    // The sums of block (col, row) from its samples (`rows` rows of BlockWidth() pixels, as
    // ReadBlock writes them).
    Status Count(uint32_t col, uint32_t row, const uint8_t* samples, uint32_t rows) const;
    // Count() of a block a read took; on an error the block is left to Finish(), which reports it.
    void CountClaimed(uint32_t col, uint32_t row, const uint8_t* samples, uint32_t rows) const;
    size_t BlockIndex(uint32_t col, uint32_t row) const noexcept {
        return static_cast<size_t>(row) * across_ + col;
    }

    uint32_t across_ = 0;
    size_t blocks_ = 0;
    int bands_ = 0;                      // bands measured (complex: one per complex band)
    int alphaBand_ = -1;                 // 0-based, for kAlphaBand
    bool convert_ = false;               // complex, Float16 or 64-bit integer samples
    std::vector<stats::SampleFilter> filters_;
    std::vector<StatsMaskKind> masks_;
    std::unique_ptr<std::atomic<uint8_t>[]> claimed_;
    mutable std::vector<stats::ExactSums> sums_;  // [block * bands_ + band]
    mutable std::atomic<uint64_t> countedByReads_{0};
    mutable uint64_t countedAfter_ = 0;
};

}  // namespace ddbt::cog::build
