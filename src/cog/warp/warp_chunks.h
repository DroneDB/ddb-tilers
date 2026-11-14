// SPDX-License-Identifier: MPL-2.0
//
// The chunks GDAL's warper splits a destination window into (GDALWarpOperation::CollectChunkList,
// alg/gdalwarpoperation.cpp, as gdalwarp runs it into its temporary tiled GTiff): a window is
// halved, along its longer side and on the 256 px blocks of that file when both halves span a
// block or more (OPTIMIZE_SIZE=AUTO), while the memory it needs (the source window it reads and
// itself, GetWorkingMemoryForWindow) exceeds the warp memory, or while it is over 100 px and its
// source window, clamped to the raster, holds less than half of the unclamped one (the source
// "fill ratio"); a window without source pixels is skipped (SKIP_NOSOURCE=YES). The source window
// of a destination window is ComputeSourceWindow's: its edges sampled at 21 points each (every
// pixel when a corner does not transform; a 23 x 23 grid, then the source grid sent forward, when
// an edge point does not or when a pole is in the output), the bilinear radius and SOURCE_EXTRA
// around them.
//
// The chunks matter to the pixels: GDAL's approximate transformer works on the destination
// pixels of one row of one chunk (RowApproximator), and a destination pixel whose source position
// is outside the source window of its chunk is not written.
#pragma once

#include <cstddef>
#include <functional>
#include <vector>

namespace ddbt::cog::warp {

struct WarpChunk {
    int dx = 0, dy = 0, dw = 0, dh = 0;  // destination window
    int sx = 0, sy = 0, sw = 0, sh = 0;  // its source window
};

struct WarpChunkOptions {
    /// gdalwarp -wm in bytes (libddb's default: 512 MB).
    double memoryLimit = 512.0 * 1024 * 1024;
    /// GetWorkingMemoryForWindow's cost of a source and of a destination pixel, in bits.
    int srcPixelBits = 8, dstPixelBits = 8;
    /// The blocks of the destination dataset (gdalwarp's temporary GTiff: TILED=YES).
    int dstBlockWidth = 256, dstBlockHeight = 256;
    /// SOURCE_EXTRA (gdalwarp sets 5 for a source with RPC metadata); negative: unset.
    int sourceExtra = -1;
    /// The resampling kernel's radius (GWKGetFilterRadius: 1 for bilinear).
    int filterRadius = 1;
    /// One of GDAL's "special points" (the poles, as GDALTransformLonLatToDestGenImgProjTransformer
    /// places them) is in the output: every window is then sampled on a grid as well.
    bool specialPointInOutput = false;
};

/// GDAL's exact general image transform between pixel positions: destination -> source when
/// dstToSrc, else source -> destination; n points in place, ok[i] = 0 where a point fails.
using PixelTransformer = std::function<void(bool dstToSrc, double* xs, double* ys, size_t n, char* ok)>;

/// The chunks of the destination window (x, y, w, h) of a warp from a srcW x srcH source, in
/// GDAL's order (top to bottom, then left to right).
std::vector<WarpChunk> CollectWarpChunks(int x, int y, int w, int h, int srcW, int srcH, const PixelTransformer& t,
                                         const WarpChunkOptions& options);

}  // namespace ddbt::cog::warp
