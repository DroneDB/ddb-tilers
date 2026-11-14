// SPDX-License-Identifier: MPL-2.0
//
// ImageSource: the base of the readers of whole-image formats (JPEG, PNG, GIF, WebP). GDAL's
// drivers for them decode the file from its start, row after row, whatever block is asked for;
// here the image is decoded once, row by row, on the first block read (or when Load() is called),
// and its blocks are strips of those rows. The rows are kept in memory when they take at most
// half of the memory budget, else in an anonymous scratch file in the system's temporary folder
// (read back with positional reads), so that an image of any size can be built.
//
// A reader describes the raster (ImageInfo: size, bands, sample type, colour interpretations,
// colour table, nodata, alpha, NBITS, mask) when it opens the file, and implements Decode(),
// which hands every row (and every row of the mask) to a RowSink in order. Photos carry no
// georeferencing of their own (GDAL reads none from them without sidecar files, which libddb
// does not use for photos): Georef() is empty.
#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "cog/build/pipeline.h"
#include "cog/build/source.h"
#include "core/io/scratch_file.h"

namespace ddbt::cog::build {

/// What a whole-image reader reports of its raster, as GDAL's driver reports it.
struct ImageInfo {
    uint32_t width = 0, height = 0;
    int bands = 0;
    tiff::SampleType type = tiff::SampleType::kU8;
    std::vector<std::string> interps;  ///< one per band ("Red", "Gray", "Palette", "Alpha", ...)
    std::optional<std::vector<std::array<uint8_t, 3>>> colorTable;
    std::optional<double> noData;
    std::string noDataText;
    int alphaBand = -1;
    std::optional<int> nbits;  ///< every band's NBITS, when it has one
    bool mask = false;         ///< a per-dataset mask (Decode() writes its rows too)
};

/// Where Decode() writes the decoded rows.
class RowSink {
public:
    virtual ~RowSink() = default;
    /// Row y (0-based, in order) of the image: width * bands samples, pixel-interleaved, host order.
    virtual Status Row(uint32_t y, const uint8_t* samples) = 0;
    /// Row y of the mask: width bytes, 0 = invalid, 255 = valid.
    virtual Status MaskRow(uint32_t y, const uint8_t* mask) = 0;
};

class ImageSource : public RasterSource {
public:
    const std::string& Path() const noexcept override { return path_; }
    const tiff::Georeferencing& Georef() const noexcept override { return georef_; }
    uint32_t Width() const noexcept override { return info_.width; }
    uint32_t Height() const noexcept override { return info_.height; }
    int Bands() const noexcept override { return info_.bands; }
    bool Complex() const noexcept override { return false; }
    tiff::SampleType Type() const noexcept override { return info_.type; }
    /// Strips of whole rows (at most 256, about 4 MiB).
    uint32_t BlockWidth() const noexcept override { return info_.width; }
    uint32_t BlockHeight() const noexcept override { return blockHeight_; }

    std::optional<double> NoData() const noexcept override { return info_.noData; }
    const std::string& NoDataText() const noexcept override { return info_.noDataText; }
    bool HasMask() const noexcept override { return info_.mask; }
    int AlphaBand() const noexcept override { return info_.alphaBand; }
    std::string ColorInterp(int b) const override;
    std::optional<std::vector<std::array<uint8_t, 3>>> ColorTable() const override { return info_.colorTable; }
    std::optional<int> Nbits(int b) const override;

    const std::string& GdalMetadata() const noexcept override { return empty_; }
    std::vector<writer::RawTag> MetadataTags() const override { return {}; }

    Expected<uint32_t> ReadBlock(uint32_t col, uint32_t row, uint8_t* out) const override;
    Status ReadMaskBlock(uint32_t col, uint32_t row, uint8_t* out) const override;
    /// The rows in memory (not when they went to the scratch file).
    const uint8_t* ResidentBlock(uint32_t col, uint32_t row) const override;
    const uint8_t* ResidentMaskBlock(uint32_t col, uint32_t row) const override;

    /// Decodes the image now (the first block read does it otherwise): the decoding error, if any.
    Status Load() const;
    /// The decoded rows are in a scratch file (they did not fit in half of the memory budget).
    bool Spilled() const noexcept { return spilled_; }
    void SetScratchBase(std::string base) override { scratchBase_ = std::move(base); }

protected:
    ImageSource(std::string path, uint64_t memoryBudget) : path_(std::move(path)), memoryBudget_(memoryBudget) {}
    /// Called by the reader's Open once the raster is described.
    Status SetInfo(ImageInfo info);
    const ImageInfo& Info() const noexcept { return info_; }
    size_t RowBytes() const noexcept { return static_cast<size_t>(info_.width) * PixelBytes(); }

    /// Decodes every row of the image (and of its mask) into `sink`, in order.
    virtual Status Decode(RowSink& sink) const = 0;

private:
    std::string path_;
    uint64_t memoryBudget_ = 0;
    std::string scratchBase_;
    tiff::Georeferencing georef_;
    ImageInfo info_;
    uint32_t blockHeight_ = 1;
    std::string empty_;

    mutable std::once_flag once_;
    mutable Status loadStatus_;
    mutable bool spilled_ = false;
    // The decoded rows (and mask rows): in memory, or in the scratch file.
    mutable LargeBytes pixels_, mask_;
    mutable io::ScratchFile file_;
    mutable uint64_t maskOffset_ = 0;
};

}  // namespace ddbt::cog::build
