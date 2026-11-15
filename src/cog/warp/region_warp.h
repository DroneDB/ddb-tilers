// SPDX-License-Identifier: MPL-2.0
//
// The warp of a raster into memory on a grid the host gives, as gdalwarp -of MEM -t_srs <crs>
// -te <extent> [-te_srs <crs>] -ts <width> <height> -r bilinear|near [-b ...] [-dstalpha]
// [-dstnodata <value>] makes it (the renders of a region of a map): the warp producer of the COG
// builds with gdalwarp's own rules (WarpSettings::grid), from the overview gdalwarp picks (-ovr AUTO:
// the source window of a 10 x 10 grid of the output, GDALBandGetBestOverviewLevel with threshold 1).
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "cog/tiff/tiff_reader.h"
#include "cog/warp/warp_producer.h"
#include "core/util/status.h"

namespace ddbt::cog::warp {

struct RegionRequest {
    std::string targetCrs;                 // -t_srs (any text PROJ reads)
    std::array<double, 4> extent{};        // -te: min x, min y, max x, max y
    std::string extentCrs;                 // -te_srs ("": the extent is in targetCrs)
    uint32_t width = 0, height = 0;        // -ts
    Resampling resampling = Resampling::kBilinear;
    std::vector<int> bands;                // -b, 1-based (empty: every band but a last alpha band)
    bool dstAlpha = false;                 // -dstalpha
    std::optional<double> dstNodata;       // -dstnodata
};

struct WarpedRaster {
    uint32_t width = 0, height = 0;
    int bands = 0;
    tiff::SampleType type = tiff::SampleType::kU8;
    std::array<double, 6> geoTransform{};
    std::string wkt;                       // the target CRS, as GDAL reports it
    std::vector<std::string> colorInterp;  // one per band (gdalwarp's MEM output: Undefined, Alpha)
    std::optional<double> nodata;          // of every band but the alpha band (gdalwarp sets none on it)
    int alphaBand = -1;                    // 0-based, -1 when none
    int sourceLevel = 0;                   // 0: the raster, i: its overview i - 1
    /// The colour table of the first band of the source, which gdalwarp gives the first band of its
    /// output: RGBA entries as GDAL reads a TIFF ColorMap (the entry of the nodata value transparent)
    std::vector<std::array<uint8_t, 4>> colorTable;
    std::vector<uint8_t> pixels;           // band after band, rows of width samples
};

Expected<WarpedRaster> WarpRegion(const std::string& path, const RegionRequest& request);

}  // namespace ddbt::cog::warp
