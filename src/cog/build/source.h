// SPDX-License-Identifier: MPL-2.0
//
// RasterSource: the input raster of a COG build as the builder (and GDAL's warper) sees it,
// whatever its format: bands, sample type, transparency (a per-dataset mask, an alpha band,
// nodata), colour interpretation and table, georeferencing, the metadata GDAL copies to the
// output, and decoded full-resolution blocks as pixel-interleaved samples of every band.
//
// The builder and the warper depend on this interface only. Each format reads its file the way
// GDAL's driver for it does, so that the COG is the one GDAL would build:
//   - TiffSource (tiff_source.h): GeoTIFF, GDAL's GTiff driver;
//   - Jp2Source (jp2_source.h): JPEG 2000 (JP2 file or raw codestream), GDAL's JP2OpenJPEG driver.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "cog/tiff/geotiff.h"
#include "cog/tiff/tiff_reader.h"
#include "cog/writer/cog_writer.h"
#include "core/util/status.h"

namespace ddbt::cog::build {

class RasterSource {
public:
    virtual ~RasterSource() = default;
    RasterSource(const RasterSource&) = delete;
    RasterSource& operator=(const RasterSource&) = delete;

    virtual const std::string& Path() const noexcept = 0;
    /// Geotransform or ground control points, and the CRS, as GDAL reports them.
    virtual const tiff::Georeferencing& Georef() const noexcept = 0;

    virtual uint32_t Width() const noexcept = 0;
    virtual uint32_t Height() const noexcept = 0;
    /// The values of a pixel: its bands, each complex band counting its two components (real,
    /// imaginary), which the builder warps and resamples as separate values.
    virtual int Bands() const noexcept = 0;
    /// Complex samples (GDAL's CInt16 ... CFloat64): Bands() / 2 complex bands.
    virtual bool Complex() const noexcept = 0;
    /// The sample type of the bands as GDAL reads them (a 1-bit image reads as bytes 0 / 1).
    virtual tiff::SampleType Type() const noexcept = 0;
    size_t SampleSize() const noexcept { return tiff::SampleTypeSize(Type()); }
    size_t PixelBytes() const noexcept { return SampleSize() * static_cast<size_t>(Bands()); }

    /// The blocks ReadBlock decodes: GDAL's blocks of the raster.
    virtual uint32_t BlockWidth() const noexcept = 0;
    virtual uint32_t BlockHeight() const noexcept = 0;
    uint32_t BlocksAcross() const noexcept {
        return static_cast<uint32_t>((uint64_t{Width()} + BlockWidth() - 1) / BlockWidth());
    }
    uint32_t BlocksDown() const noexcept {
        return static_cast<uint32_t>((uint64_t{Height()} + BlockHeight() - 1) / BlockHeight());
    }
    /// Bytes ReadBlock writes (a full block).
    size_t BlockBytes() const noexcept { return static_cast<size_t>(BlockWidth()) * BlockHeight() * PixelBytes(); }

    /// The nodata value of the bands, and its text as stored.
    virtual std::optional<double> NoData() const noexcept = 0;
    virtual const std::string& NoDataText() const noexcept = 0;
    /// A per-dataset mask (GDAL's GMF_PER_DATASET mask band that is not an alpha band).
    virtual bool HasMask() const noexcept = 0;
    /// 0-based index of the alpha band, -1 if none.
    virtual int AlphaBand() const noexcept = 0;
    /// Colour interpretation of band b (0-based) as GDAL reports it: "Red", "Green", "Blue",
    /// "Gray", "Alpha", "Undefined", "Palette".
    virtual std::string ColorInterp(int b) const = 0;
    /// The colour table of the first band, when it has one (RGB, 8 bits per component).
    virtual std::optional<std::vector<std::array<uint8_t, 3>>> ColorTable() const = 0;
    /// GDAL's NBITS of band b (IMAGE_STRUCTURE metadata): its bits when they are not those of the
    /// sample type, none otherwise.
    virtual std::optional<int> Nbits(int b) const = 0;

    /// DroneDB's isOptimizedCogDataset: a COG in EPSG:3857 with square 256 / 512 tiles and
    /// overviews, re-encoded at its own grid instead of warped.
    virtual bool IsOptimizedCog() const { return false; }
    /// The source's JPEG quality (GDAL's JPEG_QUALITY estimate) when it is JPEG-compressed.
    virtual std::optional<int> JpegQuality() const { return std::nullopt; }
    virtual bool IsJpeg() const noexcept { return false; }
    /// RPC metadata (GDAL's RPC domain): gdalwarp then reads more source pixels around its
    /// windows and warps only within the suggested extent of the source.
    virtual bool HasRpc() const noexcept { return false; }

    /// The dataset and band metadata GDAL copies to the output, as a GTiff GDAL_METADATA
    /// document ("<GDALMetadata><Item ...>...</GDALMetadata>"), "" when none.
    virtual const std::string& GdalMetadata() const noexcept = 0;
    /// The TIFF tags GDAL writes back from TIFFTAG_* metadata items (DocumentName,
    /// ImageDescription, Software, DateTime, Artist, HostComputer, Copyright, X / YResolution,
    /// ResolutionUnit, Min / MaxSampleValue), as little-endian raw tags.
    virtual std::vector<writer::RawTag> MetadataTags() const = 0;

    /// Decodes block (col, row) of the full resolution into `out` (BlockBytes() bytes): rows of
    /// BlockWidth() pixels, every band interleaved, host order. Returns the number of rows of
    /// the block (the last ones may be shorter). Safe to call concurrently.
    virtual Expected<uint32_t> ReadBlock(uint32_t col, uint32_t row, uint8_t* out) const = 0;
    /// Decodes block (col, row) of the per-dataset mask as bytes 0 / 255 (rows of BlockWidth()).
    virtual Status ReadMaskBlock(uint32_t col, uint32_t row, uint8_t* out) const = 0;

protected:
    RasterSource() = default;
};

/// Opens `path` with the reader of its format (TIFF / BigTIFF, JPEG 2000), recognised by its
/// first bytes as GDAL recognises it. `memoryBudget` bounds the reader's decoding buffers.
Expected<std::unique_ptr<RasterSource>> OpenRasterSource(const std::string& path,
                                                         uint64_t memoryBudget = uint64_t{2} << 30);

/// TIFF tags as GDAL's GTiff driver writes TIFFTAG_* metadata items: ASCII (NUL-terminated),
/// SHORT, and RATIONAL from the double GDAL sets (kept as a float by libtiff, then written as
/// libtiff's continued-fraction rational of it).
writer::RawTag AsciiTag(uint16_t tag, const std::string& value);
writer::RawTag ShortTag(uint16_t tag, uint16_t value);
writer::RawTag RationalTag(uint16_t tag, double value);
/// The value GDAL's GTiff driver reports for a FLOAT tag it reads (X / YResolution): libtiff's
/// float, printed with %.8g and parsed back.
double GdalFloatTag(double value);

/// GDAL's GTiff writer for the default-domain metadata of a dataset (WriteMDMetadata): the
/// TIFFTAG_* items it knows become TIFF tags (`tags`), AREA_OR_POINT is left to the GeoKeys, the
/// other items become the GDAL_METADATA document (`gdalMetadata`, "" when there is none), each
/// value escaped twice as GDAL writes it.
void GtiffMetadata(const std::vector<std::pair<std::string, std::string>>& items, std::vector<writer::RawTag>& tags,
                   std::string& gdalMetadata);

}  // namespace ddbt::cog::build
