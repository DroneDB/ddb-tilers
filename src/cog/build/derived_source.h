// SPDX-License-Identifier: MPL-2.0
//
// Sources made from another source, for the geoprojection of a photo (libddb's gdal_translate of
// the photo with -a_srs and -gcp, optionally -outsize, then gdalwarp):
//
//   - WithGcps: the source georeferenced by ground control points in a CRS (gdal_translate -gcp
//     -a_srs), the GCPs' pixel and line in its own pixel coordinates;
//   - ScaledNearest: the source resampled to another size by nearest neighbour as GDAL reads a
//     raster at another size from its full resolution (GDALRasterBand::IRasterIO: source column
//     int(0.5 * dx + k * dx + 1e-10), dx = width / output width, accumulated; source row
//     int((k + 0.5) * dy + 1e-10)), every band and the mask.
//
// Both forward everything else to the source they are made from (ForwardingSource).
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cog/build/source.h"
#include "core/crs/crs.h"
#include "core/crs/gcp_transform.h"

namespace ddbt::cog::build {

/// A source that forwards every query to another one; the derived sources override what they
/// change.
class ForwardingSource : public RasterSource {
public:
    explicit ForwardingSource(std::unique_ptr<RasterSource> inner) : inner_(std::move(inner)) {}

    const std::string& Path() const noexcept override { return inner_->Path(); }
    const tiff::Georeferencing& Georef() const noexcept override { return inner_->Georef(); }
    uint32_t Width() const noexcept override { return inner_->Width(); }
    uint32_t Height() const noexcept override { return inner_->Height(); }
    int Bands() const noexcept override { return inner_->Bands(); }
    bool Complex() const noexcept override { return inner_->Complex(); }
    tiff::SampleType Type() const noexcept override { return inner_->Type(); }
    uint32_t BlockWidth() const noexcept override { return inner_->BlockWidth(); }
    uint32_t BlockHeight() const noexcept override { return inner_->BlockHeight(); }
    std::optional<double> NoData() const noexcept override { return inner_->NoData(); }
    const std::string& NoDataText() const noexcept override { return inner_->NoDataText(); }
    bool HasMask() const noexcept override { return inner_->HasMask(); }
    int AlphaBand() const noexcept override { return inner_->AlphaBand(); }
    std::string ColorInterp(int b) const override { return inner_->ColorInterp(b); }
    std::optional<std::vector<std::array<uint8_t, 3>>> ColorTable() const override { return inner_->ColorTable(); }
    std::optional<int> Nbits(int b) const override { return inner_->Nbits(b); }
    bool IsOptimizedCog() const override { return inner_->IsOptimizedCog(); }
    std::optional<int> JpegQuality() const override { return inner_->JpegQuality(); }
    bool IsJpeg() const noexcept override { return inner_->IsJpeg(); }
    bool HasRpc() const noexcept override { return inner_->HasRpc(); }
    const std::string& GdalMetadata() const noexcept override { return inner_->GdalMetadata(); }
    std::vector<writer::RawTag> MetadataTags() const override { return inner_->MetadataTags(); }
    Expected<uint32_t> ReadBlock(uint32_t col, uint32_t row, uint8_t* out) const override {
        return inner_->ReadBlock(col, row, out);
    }
    Status ReadMaskBlock(uint32_t col, uint32_t row, uint8_t* out) const override {
        return inner_->ReadMaskBlock(col, row, out);
    }

protected:
    const RasterSource& Inner() const noexcept { return *inner_; }

private:
    std::unique_ptr<RasterSource> inner_;
};

/// `source` georeferenced by `gcps` (pixel and line in its pixel coordinates; x, y in `gcpCrs`).
/// Refused when the source is georeferenced already (a CRS with a geotransform or GCPs); a
/// geotransform without a CRS (a world file) is replaced, as gdal_translate -gcp replaces it.
/// `crsDefinition` is the text the CRS was made from ("EPSG:4326").
Expected<std::unique_ptr<RasterSource>> WithGcps(std::unique_ptr<RasterSource> source, std::vector<crs::Gcp> gcps,
                                                 crs::Crs gcpCrs, std::string crsDefinition);

/// `source` resampled to width x height by GDAL's nearest neighbour (see above).
Expected<std::unique_ptr<RasterSource>> ScaledNearest(std::unique_ptr<RasterSource> source, uint32_t width,
                                                      uint32_t height);

}  // namespace ddbt::cog::build
