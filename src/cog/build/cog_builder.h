// SPDX-License-Identifier: MPL-2.0
//
// The COG builder: DroneDB 2.4.1's buildCog output contract (cog.cpp) without GDAL, in one pass.
//
//   - A source that is already an optimized COG (EPSG:3857, square 256 / 512 tiles, overviews:
//     DroneDB's isOptimizedCogDataset) is encoded again at its own grid (GDALTranslate -of COG
//     BLOCKSIZE=<source> OVERVIEWS=IGNORE_EXISTING): same pixels, overviews regenerated; JPEG at
//     the source's quality for a 3-band 8-bit JPEG source, LZW + predictor otherwise.
//   - Any other GeoTIFF is warped to EPSG:3857 on the GoogleMapsCompatible grid of the COG driver
//     (gdalwarp -of COG -r bilinear, ZOOM_LEVEL_STRATEGY=UPPER): JPEG q75 YCbCr for 8-bit RGB(A)
//     without nodata (the alpha becomes a 1-bit mask), LZW + predictor otherwise; transparency as
//     cog.cpp: a finite nodata is kept as the destination nodata, a NaN / infinite / huge nodata
//     or a stand-alone mask becomes an alpha band, a source alpha is kept (a mask for JPEG).
//   - A photo georeferenced by GCPs (BuildOptions::photo) is warped as libddb geoprojects it
//     (gdalwarp -dstalpha from the photo with the GCPs): to the EPSG:3857 COG above in one warp
//     (bilinear; JPEG for 8-bit RGB, with the pixels outside the photo, its alpha, mask or nodata
//     in the mask; LZW with an alpha band otherwise), or into the GCPs' CRS on gdalwarp's grid
//     with its default nearest neighbour resampling (the same compression and transparency).
//   - Overviews: GDAL's cubic resampling, mask-aware, each level from the previous one.
//   - The statistics sidecar "<output>.stats.json" (exact, version 2), from the pixels written.
//
// Everything is streamed: full-resolution blocks are produced row by row of blocks, compressed
// in parallel on the default pool, written in order; overview rows are computed as soon as the
// rows they read exist.
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "cog/build/source.h"
#include "cog/encode/block_encoder.h"
#include "cog/warp/warp_producer.h"
#include "core/crs/gcp_transform.h"
#include "cog/writer/cog_writer.h"
#include "core/util/status.h"

namespace ddbt::cog::build {

/// Progress: fraction 0..1 of the whole build and DroneDB's phase name ("warping", "overviews",
/// "compressing", "statistics"). The fraction is the share of full-resolution rows done, scaled to
/// 0..0.95 (0.95..1: the prefix and the sidecar); the name follows DroneDB's cogPhaseOfFraction
/// boundaries (with a warp: warping < 0.1875 <= overviews < 0.25 <= compressing < 0.5 <=
/// statistics; own grid: overviews < 0.1 <= compressing). The phases overlap in the single pass,
/// so the name follows the fraction, not the work done at that instant.
using ProgressFn = std::function<void(double fraction, const char* phase)>;

/// The georeferencing of a photo, as libddb geoprojects it (gdal_translate -a_srs <srs> -gcp ...
/// [-outsize], then gdalwarp -dstalpha), for a source that has none of its own.
struct PhotoGeoreference {
    /// Ground control points: pixel and line in the photo (after outScale), x and y in `srs`.
    std::vector<crs::Gcp> gcps;
    /// The CRS of the GCPs (any text PROJ reads: "EPSG:4326", WKT, PROJJSON).
    std::string srs = "EPSG:4326";
    /// The output: the EPSG:3857 COG of the photo (false: DroneDB's COG rules, bilinear, one warp
    /// of the photo), or the photo in the GCPs' CRS on gdalwarp's grid with gdalwarp's default
    /// (nearest neighbour) resampling and warp memory (true: libddb's geoprojected GeoTIFF, here
    /// with a COG layout).
    bool sourceCrs = false;
    /// gdal_translate -outsize: the photo resampled first to int(outScale * width) x
    /// int(outScale * height) by GDAL's nearest neighbour; 1: as it is.
    double outScale = 1.0;
};

struct BuildOptions {
    /// Codec of non-JPEG outputs: LZW (DroneDB's), DEFLATE or ZSTD (+ predictor).
    encode::Codec losslessCodec = encode::Codec::kLzw;
    /// DEFLATE / ZSTD level (0: GDAL's defaults 6 / 9).
    int level = 0;
    /// JPEG quality of warped JPEG outputs (0: DDB_COG_JPEG_QUALITY, else 75).
    int jpegQuality = 0;
    /// Never use JPEG (lossless outputs only).
    bool noJpeg = false;
    /// Memory budget for the pipeline (strips, buffers, caches), bytes.
    uint64_t memoryBudget = uint64_t{2} << 30;
    /// Write "<output>.stats.json".
    bool statistics = true;
    /// Snap the warped grid's origin to an even tile (a 512 px retina tile = one block).
    /// Off: GDAL's exact grid.
    bool evenTileOrigin = false;
    /// Writer options (insert range, spill budget).
    writer::WriterOptions writer;
    /// Georeferences a photo by ground control points and geoprojects it.
    std::optional<PhotoGeoreference> photo;
    /// Where a source spills its decoded image (RasterSource::SetScratchBase); BuildCog: next to
    /// the output ("<output>.image"), the folder it writes its other scratch files to.
    std::string scratchBase;
    ProgressFn progress;
};

struct BuildReport {
    bool ownGrid = false;
    uint32_t width = 0, height = 0, levels = 0;
    int bands = 0;
    std::string codec;
    int jpegQuality = 0;
    std::string transparency;  // "none", "mask", "alpha", "nodata"
    writer::WriterStats file;
    double secondsTotal = 0;
    uint64_t peakStripBytes = 0;
    uint64_t sourceBlocksDecoded = 0;
    uint64_t peakSourceCacheBytes = 0;
};

/// A raster whose 8-bit pixels a host computes, written as GDAL's GTiff driver writes a raster
/// created band by band and given its overviews by GDALBuildOverviews AVERAGE (libddb's exports):
/// its own grid and CRS, tiled, the overviews each from the level above.
struct HostRaster {
    uint32_t width = 0, height = 0;
    /// The type of the samples (any integer or floating-point type; the overviews are 8-bit only)
    tiff::SampleType type = tiff::SampleType::kU8;
    /// GDAL's colour interpretation of each band ("Red", "Green", "Blue", "Gray", "Alpha",
    /// "Undefined"...): the photometric and ExtraSamples GTiff writes for them; an alpha band masks
    /// the others in the overviews by GDAL's rule (the last of 2 or 4 bands).
    std::vector<std::string> colorInterp;
    /// The description of each band ("" for none; empty: none), in GDAL_METADATA as GTiff writes it
    std::vector<std::string> descriptions;
    /// Metadata a host copies from its source, written as GTiff writes it (WriteMetadata):
    /// - `metadata`: the default-domain items of the dataset, in order (GtiffMetadata: the TIFFTAG_*
    ///   items GTiff knows become their TIFF tags, AREA_OR_POINT is left to the GeoKeys);
    /// - per band (each empty: none): its default-domain items in order, its scale and offset (an
    ///   OFFSET and a SCALE item unless they are 0 and 1), its unit (UNITTYPE), in GDAL_METADATA
    ///   before its description;
    /// - `xmp`: the XMP packet (TIFFTAG_XMLPACKET), "" for none.
    std::vector<std::pair<std::string, std::string>> metadata;
    std::vector<std::vector<std::pair<std::string, std::string>>> bandMetadata;
    std::vector<double> scales, offsets;
    std::vector<std::string> units;
    std::string xmp;
    uint32_t blockSize = 256;  // square tiles, a multiple of 16
    encode::Codec codec = encode::Codec::kDeflate;
    int predictor = 2;  // 1 none, 2 horizontal, 3 floating point (floating-point samples only)
    int level = 0;  // DEFLATE / ZSTD level (0: GDAL's default)
    /// The nodata value of the bands, any number (GDAL_NODATA, written as GTiff writes it): one that
    /// Byte holds masks its pixels in the overviews; any other masks nothing, and the overviews take
    /// GDAL's cast of it to 8 bits (pyramid/average.h)
    std::optional<double> nodata;
    /// An internal 1-bit mask (GDAL's per-dataset mask band): the host fills it as one more sample
    /// after the bands of each pixel (0: transparent); its overviews are GDAL's AVERAGE of the mask
    /// with itself as its mask (255 wherever a pixel they cover is not 0), the bands' with it (a nodata
    /// value is kept too, and replaced where an average computes it).
    bool mask = false;
    /// The colour table of band 1 (RGBA entries; the ColorMap keeps red, green and blue): a palette
    /// image of 1 or 2 bands of 8 or 16-bit integers. Its overviews average the colours of the
    /// pixels as GDAL averages a palette band (pyramid::PaletteAverage, over the table GTiff reads
    /// back: 256 entries, only the nodata entry transparent), for one band without a mask; a table
    /// GTiff reads back as the identity averages the indices as any 8-bit band.
    std::vector<std::array<uint8_t, 4>> colorTable;
    /// GDALBuildOverviews factors (2, 4, 8...): level i is (width + f - 1) / f x (height + f - 1) / f.
    std::vector<uint32_t> overviewFactors;
    std::array<double, 6> geoTransform{};
    bool georeferenced = false;
    writer::GeoKeys geoKeys;
    /// Writes the window (x, y, w, h) of every band (samples of `type`, native byte order; and the
    /// mask byte, with `mask`), pixel-interleaved, rows `stride` bytes apart into `out`. Called one at a time, in no particular order; throws to
    /// stop the write.
    std::function<void(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint8_t* out, size_t stride)> fill;
};

/// Writes `output` (atomically, nothing left behind on failure) from `raster`. options: progress,
/// memoryBudget and writer; no statistics sidecar is written.
Status WriteHostRaster(const HostRaster& raster, const std::string& output, const BuildOptions& options);

/// The source a build reads: `input` as OpenRasterSource opens it, georeferenced as options.photo
/// says.
Expected<std::unique_ptr<RasterSource>> OpenBuildSource(const std::string& input, const BuildOptions& options);

/// The warp a build makes of a source that is not an optimized COG.
warp::WarpSettings WarpSettingsOf(const BuildOptions& options);

/// Builds `output` (a COG) from `input` (a raster OpenRasterSource reads; a photo with
/// options.photo). The output appears atomically (temporary + rename); on failure nothing is left
/// behind.
Status BuildCog(const std::string& input, const std::string& output, const BuildOptions& options,
                BuildReport* report = nullptr);

}  // namespace ddbt::cog::build
