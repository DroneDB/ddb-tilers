// SPDX-License-Identifier: MPL-2.0
//
// SourceRaster: the input GeoTIFF of a COG build as the builder (and GDAL's warper) sees it:
// bands, sample type, transparency (internal mask, alpha band, nodata), georeferencing, and
// decoded full-resolution blocks (tiles or strips) as pixel-interleaved samples of every band.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "cog/tiff/geotiff.h"
#include "cog/tiff/tiff_reader.h"
#include "cog/writer/cog_writer.h"
#include "core/util/status.h"

namespace ddbt::cog::build {

class SourceRaster {
public:
    static Expected<std::unique_ptr<SourceRaster>> Open(const std::string& path);

    const std::string& Path() const noexcept { return path_; }
    const tiff::TiffReader& Reader() const noexcept { return *reader_; }
    const tiff::IfdInfo& Main() const noexcept { return reader_->Ifd(reader_->MainImage()); }
    const tiff::Georeferencing& Georef() const noexcept { return georef_; }

    uint32_t Width() const noexcept { return Main().width; }
    uint32_t Height() const noexcept { return Main().height; }
    int Bands() const noexcept { return Main().samplesPerPixel; }
    tiff::SampleType Type() const noexcept { return Main().sampleType; }
    size_t SampleSize() const noexcept { return tiff::SampleTypeSize(Type()); }
    size_t PixelBytes() const noexcept { return SampleSize() * static_cast<size_t>(Bands()); }
    uint32_t BlockWidth() const noexcept { return Main().blockWidth; }
    uint32_t BlockHeight() const noexcept { return Main().blockHeight; }
    uint32_t BlocksAcross() const noexcept { return Main().blocksAcross; }
    uint32_t BlocksDown() const noexcept { return Main().blocksDown; }

    /// GDAL_NODATA of the main image.
    std::optional<double> NoData() const noexcept { return Main().noData; }
    const std::string& NoDataText() const noexcept { return Main().noDataText; }
    /// The 1-bit internal mask IFD of the main image (GDAL's per-dataset mask).
    std::optional<uint32_t> MaskIfd() const noexcept { return maskIfd_; }
    /// 0-based index of the alpha band (ExtraSamples associated / unassociated alpha), -1 if none.
    int AlphaBand() const noexcept { return alphaBand_; }
    /// Colour interpretation of band b (0-based) as GDAL reports it: "Red", "Green", "Blue",
    /// "Gray", "Alpha", "Undefined", "Palette".
    std::string ColorInterp(int b) const;

    /// DroneDB's isOptimizedCogDataset: EPSG:3857, square 256 / 512 tiles, overviews.
    bool IsOptimizedCog() const;
    /// The source's JPEG quality (GDAL's JPEG_QUALITY estimate) when it is JPEG-compressed.
    std::optional<int> JpegQuality() const;
    bool IsJpeg() const noexcept { return Main().compression == 7; }

    /// The TIFF tags GDAL exposes as TIFFTAG_* metadata items and writes back to a copy
    /// (DocumentName, ImageDescription, Software, DateTime, Artist, HostComputer, Copyright,
    /// X / YResolution, ResolutionUnit, Min / MaxSampleValue), as little-endian raw tags.
    std::vector<writer::RawTag> MetadataTags() const;

    /// Decodes block (col, row) of the full resolution into `out`: BlockWidth() x rows of the
    /// block, every band interleaved (BAND-interleaved files are interleaved here), host order.
    /// Returns the number of rows of the block (strips: the last one may be shorter).
    Expected<uint32_t> ReadBlock(uint32_t col, uint32_t row, uint8_t* out) const;
    /// Bytes ReadBlock writes (a full block).
    size_t BlockBytes() const noexcept {
        return static_cast<size_t>(BlockWidth()) * BlockHeight() * PixelBytes();
    }
    /// Decodes block (col, row) of the internal mask as bytes 0 / 255 (rows of BlockWidth()).
    /// The mask's blocks are those of the image (GDAL writes them so); Unsupported otherwise.
    Status ReadMaskBlock(uint32_t col, uint32_t row, uint8_t* out) const;

private:
    std::string path_;
    std::shared_ptr<const tiff::TiffReader> reader_;
    tiff::Georeferencing georef_;
    std::optional<uint32_t> maskIfd_;
    int alphaBand_ = -1;
};

}  // namespace ddbt::cog::build
