// SPDX-License-Identifier: MPL-2.0
//
// Jp2Source: a JPEG 2000 file (JP2 boxes or a raw codestream) as GDAL's JP2OpenJPEG driver reads
// it (frmts/opjlike/jp2opjlikedataset.cpp, frmts/openjpeg/opjdatasetbase.h):
//
//   - the band type from the first component's precision and sign (8 bits or less: Byte, up to
//     16: Int16 / UInt16, more: Int32 / UInt32), the decoded integers clamped into it;
//   - every component at the size of the first, except 4:2:0 YCbCr (three or four 8-bit
//     components, the chroma halved, the colour space not sRGB), converted to RGB with GDAL's
//     arithmetic; other subsamplings are refused, as GDAL refuses them;
//   - the colour interpretation from the colr and cdef boxes, a 1-bit alpha component of an
//     8-bit image promoted to 0 / 255, a pclr box as the colour table of the first band;
//   - GDAL's blocks: the tiles of the codestream, or areas of at most 1024 x 1024 of a
//     single-tile image larger than 1024 (decoded with one codestream kept open, so that its
//     compressed data is read once), the whole image when it is small with tiny tiles, so that
//     the samples are those GDAL decodes;
//   - the georeferencing and metadata of jp2/jp2_file.h. No nodata and no mask (libddb disables
//     the .aux.xml sidecars that would hold them).
#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "cog/build/source.h"
#include "cog/jp2/codestream.h"
#include "cog/jp2/jp2_file.h"

namespace ddbt::cog::build {

class Jp2Source final : public RasterSource {
public:
    /// `memoryBudget` bounds the decoding buffers of the blocks decoded at the same time.
    static Expected<std::unique_ptr<Jp2Source>> Open(const std::string& path, uint64_t memoryBudget);

    const std::string& Path() const noexcept override { return path_; }
    const tiff::Georeferencing& Georef() const noexcept override { return georef_; }
    uint32_t Width() const noexcept override { return header_.Width(); }
    uint32_t Height() const noexcept override { return header_.Height(); }
    int Bands() const noexcept override { return static_cast<int>(header_.components.size()); }
    bool Complex() const noexcept override { return false; }
    tiff::SampleType Type() const noexcept override { return type_; }
    uint32_t BlockWidth() const noexcept override { return blockWidth_; }
    uint32_t BlockHeight() const noexcept override { return blockHeight_; }

    std::optional<double> NoData() const noexcept override { return std::nullopt; }
    const std::string& NoDataText() const noexcept override { return empty_; }
    bool HasMask() const noexcept override { return false; }
    int AlphaBand() const noexcept override;
    std::string ColorInterp(int b) const override;
    std::optional<std::vector<std::array<uint8_t, 3>>> ColorTable() const override;
    bool HasRpc() const noexcept override { return rpc_; }
    /// GDAL's NBITS of band b: its precision when not a multiple of 8 (8 for a promoted alpha).
    std::optional<int> Nbits(int b) const;

    const std::string& GdalMetadata() const noexcept override { return gdalMetadata_; }
    std::vector<writer::RawTag> MetadataTags() const override { return tags_; }

    Expected<uint32_t> ReadBlock(uint32_t col, uint32_t row, uint8_t* out) const override;
    Status ReadMaskBlock(uint32_t col, uint32_t row, uint8_t* out) const override;

private:
    // How a block is decoded: a tile of the codestream, or an area of the image with a
    // codestream opened for the block or the one kept open for the whole image.
    enum class Mode : uint8_t { kTile, kArea, kSharedArea };

    Jp2Source() = default;
    Status Decode(jp2::Codestream& cs, uint32_t col, uint32_t row) const;
    Status Convert(const jp2::Codestream& cs, uint32_t width, uint32_t rows, uint8_t* out) const;
    Expected<std::unique_ptr<jp2::Codestream>> OpenCodestream(int threads) const;
    void AcquireSlot() const;
    void ReleaseSlot() const;

    std::string path_;
    std::shared_ptr<const io::ByteSource> bytes_;
    jp2::Jp2File file_;
    jp2::CodestreamHeader header_;
    tiff::Georeferencing georef_;
    bool rpc_ = false;
    tiff::SampleType type_ = tiff::SampleType::kU8;
    uint32_t blockWidth_ = 0, blockHeight_ = 0;
    Mode mode_ = Mode::kTile;
    int threadsPerDecode_ = 1;
    bool is420_ = false;
    jp2::ColourSpace colourSpace_ = jp2::ColourSpace::kUnknown;
    jp2::ChannelRoles roles_;
    bool promoteAlpha_ = false;
    std::optional<std::vector<std::array<uint8_t, 3>>> colorTable_;
    std::string gdalMetadata_;
    std::vector<writer::RawTag> tags_;
    std::string empty_;

    // The codestream kept open for the areas of a single-tile image (one decode at a time).
    mutable std::mutex sharedMutex_;
    mutable std::unique_ptr<jp2::Codestream> shared_;
    // Decodes running at the same time, bounded by the memory budget.
    mutable std::mutex slotMutex_;
    mutable std::condition_variable slotFree_;
    mutable int freeSlots_ = 1;
};

}  // namespace ddbt::cog::build
