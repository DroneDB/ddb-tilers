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
//   - Overviews: GDAL's cubic resampling, mask-aware, each level from the previous one.
//   - The statistics sidecar "<output>.stats.json" (exact, version 2), from the pixels written.
//
// Everything is streamed: full-resolution blocks are produced row by row of blocks, compressed
// in parallel on the default pool, written in order; overview rows are computed as soon as the
// rows they read exist.
#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "cog/encode/block_encoder.h"
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

/// Builds `output` (a COG) from `input` (a GeoTIFF). The output appears atomically (temporary +
/// rename); on failure nothing is left behind.
Status BuildCog(const std::string& input, const std::string& output, const BuildOptions& options,
                BuildReport* report = nullptr);

}  // namespace ddbt::cog::build
