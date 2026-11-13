// SPDX-License-Identifier: MPL-2.0
//
// The reads of a tile, as GDAL's RasterIO serves the ones GDALTiler makes.
//
// A RasterView is the dataset DroneDB's GDALTiler reads (its inputDataset): the file itself when
// it is in EPSG:3857, else the virtual EPSG:3857 grid of the COG DroneDB would build (warp.h).
// PlanRead() resolves a read of window r into a buffer of bufW x bufH pixels to the pixel each
// buffer pixel takes, with GDAL's rules (gcore/rasterio.cpp):
//
//  - Compat: when the buffer is smaller than the window on either axis and the band has
//    overviews, GDALBandGetBestOverviewLevel2 picks the overview (desired factor of the least
//    reduced axis, oversampling threshold 1.2 for nearest, 0.1 epsilon) and the window is moved
//    to it with its integer rounding; then GDALRasterBand::IRasterIO's nearest sampling: source
//    row (int)clamp((j + 0.5) * h / bufH + y + 1e-10), source column from x + 0.5 * inc + 1e-10
//    stepped by inc (an accumulated double sum; an integer factor steps in integers and clamps
//    only the last column), clamped to the level.
//  - Default (DDBT_TILER_MODE unset or "compat" keeps compat; see CogTiler): the level comes
//    from the zoom, and each tile pixel takes the level pixel under its centre (no window
//    rounding: no seams between tiles).
//
// The plan is then gathered from the decoded blocks (block_cache.h): bands as their native
// samples, the "alpha" GDALTiler::readAlpha reads (the alpha band clamped to bytes, else the mask
// band of band 1: the internal mask, the nodata mask, all valid).
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "cog/tiler/block_cache.h"
#include "cog/tiler/cog_source.h"
#include "core/tile/compat.h"

namespace ddbt::cog::tiler {

/// The dataset GDALTiler reads: the file, or the warped virtual grid; optionally with a NoData
/// override (DroneDB withNodataOverride: a VRT whose bands all have that NoData value).
class RasterView {
public:
    explicit RasterView(const CogSource& source, std::optional<double> nodataOverride = std::nullopt);

    const CogSource& Source() const noexcept { return *source_; }
    bool Warped() const noexcept { return warped_; }
    int Width() const noexcept { return levels_[0].first; }
    int Height() const noexcept { return levels_[0].second; }
    const std::array<double, 6>& GeoTransform() const noexcept { return gt_; }
    const std::vector<std::pair<int, int>>& Levels() const noexcept { return levels_; }

    /// Bands of the view: the file's, plus the alpha band GDALAutoCreateWarpedVRT adds to a
    /// warped source without one.
    int BandCount() const noexcept { return bandCount_; }
    PixelType Type() const noexcept { return source_->Type(); }
    /// 1-based alpha band of the view (FindAlphaBand), 0 if none.
    int AlphaBand() const noexcept { return alphaBand_; }
    ColorInterp Interp(int band1) const;
    /// The NoData value of a band (the override when set; none for the warped alpha band).
    std::optional<double> Nodata(int band1) const;
    const std::optional<double>& NodataOverride() const noexcept { return override_; }
    /// The view has a per-dataset mask (detectBandNodata's datasetMask: no alpha band, and the
    /// mask of band 1 is per dataset, not a nodata mask).
    bool HasDatasetMask() const noexcept;
    /// dataBandCount(view).
    int DataBandCount() const noexcept;

private:
    const CogSource* source_;
    bool warped_ = false;
    std::array<double, 6> gt_{};
    std::vector<std::pair<int, int>> levels_;
    int bandCount_ = 0;
    int alphaBand_ = 0;
    std::optional<double> override_;
};

/// Where each buffer pixel of a read takes its sample.
struct SamplePlan {
    int bufW = 0, bufH = 0;
    int viewLevel = 0;
    /// Separable plan (file views): view-level column of each buffer column, row of each buffer
    /// row; -1 = no sample (outside the raster).
    std::vector<int32_t> cols, rows;
    /// Pointwise plan (warped views): the source level and the source pixel of every buffer
    /// pixel (row-major), -1 = none.
    bool pointwise = false;
    int srcLevel = 0;
    std::vector<int32_t> sx, sy;
    /// Every buffer pixel has a sample.
    bool AllInside() const noexcept;
};

/// GDALBandGetBestOverviewLevel(2): index into `levels` (0 = full resolution) of the level GDAL
/// reads for a downsampling factor `desired` (threshold: GDAL's oversampling threshold).
int BestLevel(const std::vector<std::pair<int, int>>& levels, double desired, double threshold = 1.2);

/// GDAL's RasterIO resolution of a read of window (x, y, w, h) into bufW x bufH (compat).
SamplePlan PlanRead(const RasterView& view, int x, int y, int w, int h, int bufW, int bufH);

/// Default mode: the tile pixels of a 3857 box (tile bounds, size px) on the view's level
/// chosen from the tile resolution.
SamplePlan PlanTile(const RasterView& view, double minX, double minY, double maxX, double maxY, int size);

/// Gathers the native samples of `bands` (1-based, file bands: the warped view's added alpha is
/// not a file band) into planar buffers: band i at out + i * planeBytes, bufW x bufH samples of
/// PixelTypeSize(type) bytes, row-major. Pixels without a sample are 0.
void GatherBands(const RasterView& view, const SamplePlan& plan, std::span<const int> bands, uint8_t* out,
                 size_t planeBytes);

/// GDALTiler::readAlpha: the alpha band (clamped to bytes), else the mask band of band 1 (the
/// internal mask, the nodata mask of band 1 with the view's nodata, else 255), 0 where there is
/// no sample. `band1` may give the native samples of band 1 already gathered (nodata mask).
void GatherAlpha(const RasterView& view, const SamplePlan& plan, uint8_t* out, const uint8_t* band1 = nullptr);

/// The dataset mask (internal mask) of the view's band 1 at the plan, 0 where there is no
/// sample. Only for views with a per-dataset mask.
void GatherDatasetMask(const RasterView& view, const SamplePlan& plan, uint8_t* out);

/// Summary of the internal mask blocks the plan touches (kAllZero / kAllOne when every touched
/// block is uniform that way, so no pixel needs reading). Mixed when the view has no mask.
MaskSummary PlanMaskSummary(const RasterView& view, const SamplePlan& plan);

/// GDAL's nodata mask test of a native sample (GDALNoDataMaskBand: integer types compare with
/// the nodata cast to the type, floats with ARE_REAL_EQUAL and NaN): true = nodata.
bool IsNodataSample(PixelType type, const uint8_t* sample, double nodata) noexcept;

/// GDALCopyWords of one native sample to double / float / byte.
double SampleToDouble(PixelType type, const uint8_t* sample) noexcept;
uint8_t SampleToByte(PixelType type, const uint8_t* sample) noexcept;

}  // namespace ddbt::cog::tiler
