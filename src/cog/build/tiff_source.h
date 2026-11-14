// SPDX-License-Identifier: MPL-2.0
//
// TiffSource: a GeoTIFF as GDAL's GTiff driver reads it (the main image of the file, its
// internal mask, ExtraSamples alpha, GDAL_NODATA, the GeoTIFF georeferencing and world file),
// decoded full-resolution blocks being the file's tiles or strips.
#pragma once

#include <memory>
#include <optional>
#include <string>

#include "cog/build/source.h"
#include "cog/tiff/geotiff.h"
#include "cog/tiff/tiff_reader.h"

namespace ddbt::cog::build {

class TiffSource final : public RasterSource {
public:
    static Expected<std::unique_ptr<TiffSource>> Open(const std::string& path);

    const std::string& Path() const noexcept override { return path_; }
    const tiff::Georeferencing& Georef() const noexcept override { return georef_; }

    uint32_t Width() const noexcept override { return Main().width; }
    uint32_t Height() const noexcept override { return Main().height; }
    int Bands() const noexcept override { return Main().samplesPerPixel * static_cast<int>(Main().Components()); }
    bool Complex() const noexcept override { return Main().complex; }
    tiff::SampleType Type() const noexcept override {
        return Main().sampleType == tiff::SampleType::kBit1 ? tiff::SampleType::kU8 : Main().sampleType;
    }
    uint32_t BlockWidth() const noexcept override { return Main().blockWidth; }
    uint32_t BlockHeight() const noexcept override { return Main().blockHeight; }

    /// GDAL_NODATA of the main image.
    std::optional<double> NoData() const noexcept override { return Main().noData; }
    const std::string& NoDataText() const noexcept override { return Main().noDataText; }
    /// The 1-bit internal mask IFD of the main image (GDAL's per-dataset mask).
    bool HasMask() const noexcept override { return maskIfd_.has_value(); }
    /// ExtraSamples associated / unassociated alpha.
    int AlphaBand() const noexcept override { return alphaBand_; }
    std::string ColorInterp(int b) const override;
    /// The colour table GDAL gives a 1-band raster of up to 16 bits: its ColorMap (scaled to 8
    /// bits as GDAL scales it), else black and white for a 1-bit image, an inverted grey ramp for
    /// min-is-white. None otherwise.
    std::optional<std::vector<std::array<uint8_t, 3>>> ColorTable() const override;
    /// BitsPerSample when it is not 8, 16, 32 or 64.
    std::optional<int> Nbits(int b) const override;

    bool IsOptimizedCog() const override;
    std::optional<int> JpegQuality() const override;
    bool IsJpeg() const noexcept override { return Main().compression == 7; }
    /// The 92 coefficients of the RPCCoefficientTag, which GDAL's TIFF driver reads.
    bool HasRpc() const noexcept override;

    /// The GDAL_METADATA tag of the main image, as stored.
    const std::string& GdalMetadata() const noexcept override { return Main().gdalMetadata; }
    std::vector<writer::RawTag> MetadataTags() const override;

    /// Blocks are the main image's tiles or strips (BAND-interleaved files are interleaved here).
    Expected<uint32_t> ReadBlock(uint32_t col, uint32_t row, uint8_t* out) const override;
    /// The mask's blocks are those of the image (GDAL writes them so); Unsupported otherwise.
    Status ReadMaskBlock(uint32_t col, uint32_t row, uint8_t* out) const override;

private:
    TiffSource() = default;
    const tiff::IfdInfo& Main() const noexcept { return reader_->Ifd(reader_->MainImage()); }

    std::string path_;
    std::shared_ptr<const tiff::TiffReader> reader_;
    tiff::Georeferencing georef_;
    std::optional<uint32_t> maskIfd_;
    int alphaBand_ = -1;
};

}  // namespace ddbt::cog::build
