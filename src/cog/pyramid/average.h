// SPDX-License-Identifier: MPL-2.0
//
// Overview resampling by GDAL's AVERAGE of 8-bit samples (gcore/overview.cpp,
// GDALResampleChunk_AverageOrRMS_T with UInt8 work data), each level from the previous one as
// GDALRegenerateOverviewsMultiBand computes it:
//
//   - Destination pixel i of an axis of srcSize -> dstSize pixels covers the source
//     [i * ratio, (i + 1) * ratio) (ratio = srcSize / dstSize): its first and last source pixels
//     weigh the part of them it covers (1 when it covers one pixel), the others 1.
//   - Without a mask: the weighted sum of the samples (per row: first, middle, last pixel; the
//     rows weighted the same way) over the product of the weights of the axes, + 0.5, truncated.
//   - With a mask (an alpha band or a dataset mask, a band's nodata): only the pixels whose mask
//     is not 0 count, the weights of the counted pixels summed as they are counted; a destination
//     pixel that counts none gets the nodata value (0 without one).
//   - A computed value equal to the nodata value is replaced (+ 1, 254 for 255).
//   - GDAL's fast path, without a mask: where the columns are a half of the source (2 whole pixels
//     each) and a destination row covers 2 source rows, the four samples average to
//     (sum + 2) / 4, the weights of the rows left aside, and the nodata value is not replaced.
//
// The nodata value is GDAL's cast of it to 8 bits (static_cast of a double: a value outside 0..255
// wraps as an int32 truncated to its low byte), which a band whose nodata Byte cannot hold also
// has (its mask is all valid: MaskMode::kNone, the replacement still applies).
//
// A palette band (GCI_PaletteIndex) whose colour table is not the identity is averaged in colour
// instead (PaletteAverage): GTiff then regenerates its overviews band by band with
// GDALRegenerateOverviewsEx, which reads the level above in chunks of full-width rows (radius 0,
// no rows around them) and clips the source rows of each destination row to its chunk.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "cog/pyramid/overview.h"

namespace ddbt::cog::pyramid {

/// The source pixels of destination indices [dst0, dst1) of an axis: start and count, and the
/// weights GDAL gives them (the first, the last, and the sum of all of them).
struct AverageTaps {
    int dst0 = 0;
    bool half = false;  // the source is twice the destination: every destination pixel covers 2 whole pixels
    std::vector<int> start, count;
    std::vector<double> first, last, total;
    int SrcBegin() const;  // first source index any destination reads
    int SrcEnd() const;    // one past the last
};
/// chunkRows: 0 for the whole axis in one chunk (GDALRegenerateOverviewsMultiBand's windows), else
/// the rows of GDALRegenerateOverviewsEx's chunks (RegenerateChunkRows): chunk j is the source rows
/// [j * chunkRows, (j + 1) * chunkRows), it computes the destination rows
/// [int(0.5 + j * chunkRows / ratio), the same of chunk j + 1) (the last one: to dstSize) and their
/// source windows are clipped to it.
AverageTaps AverageAxis(int srcSize, int dstSize, int dst0, int dst1, int chunkRows = 0);

/// The factor GDALRegenerateOverviewsEx gives an overview of a band: int(size / overview size + 0.5)
/// of the larger of the axes, at least 1.
int OverviewFactor(int srcWidth, int srcHeight, int dstWidth, int dstHeight);

/// The rows of the chunks GDALRegenerateOverviewsEx reads a band of 8-bit samples in to compute
/// its overviews by AVERAGE (nFullResYChunk): the block height of the band, at least twice the
/// largest factor of the overviews it computes at once (OverviewFactor), doubled while a chunk holds
/// less than half of GDAL_OVR_CHUNK_MAX_SIZE (10 MiB) of samples; a first chunk larger than that is
/// kept as it is.
int RegenerateChunkRowsOfFactor(int srcWidth, int factor, int blockHeight);
/// The same for one overview (dstWidth x dstHeight) of a palette band, whose regeneration has no
/// other way to go.
int RegenerateChunkRows(int srcWidth, int srcHeight, int dstWidth, int dstHeight, int blockHeight);
/// The same for one overview of a band without a colour table (a mask, an alpha band), or 0 when
/// GDALRegenerateOverviewsEx gives it to GDALRegenerateOverviewsMultiBand instead, which reads it by
/// blocks without clipping rows to chunks: when its first chunk is larger than 10 MiB and the band is
/// tiled narrower than it is wide.
int RegenerateChunkRowsOrBlocks(int srcWidth, int srcHeight, int dstWidth, int dstHeight, int blockWidth,
                                int blockHeight);

/// GDAL's cast of a nodata value to an 8-bit sample (static_cast<GByte> of the double, as x86 does it)
uint8_t CastNoData(double nodata);

/// One destination window of every band of 8-bit samples, with the window conventions of
/// Downsample (overview.h): spec.type must be kU8. spec.mask: kNone (every pixel valid),
/// kExternal (srcMask, srcW x srcH bytes: the pixels whose value is not 0; nullptr: all of them,
/// as GDAL's all-valid mask band of an alpha band), kNoData (per band: the samples that are not
/// the nodata value).
void Average(const DownsampleSpec& spec, const uint8_t* src, size_t srcStride, int srcX0, int srcY0, int srcW,
             int srcH, const uint8_t* srcMask, const AverageTaps& cols, const AverageTaps& rows, int dstW, int dstH,
             uint8_t* dst, size_t dstStride);

/// An RGBA entry of a colour table (GDALColorEntry: c1, c2, c3, c4).
using ColorEntry = std::array<uint8_t, 4>;

/// GDALColorTable::IsIdentity: every entry i is (i, i, i, 255).
bool IsIdentityTable(const std::vector<ColorEntry>& entries);

/// The colour table and nodata value GDAL's palette AVERAGE works with
/// (GDALResampleChunk_AverageOrRMS_T): the band's table, the entry of the nodata value made
/// transparent (a nodata >= 0 cast to 8 bits, when the table has that entry), or without a nodata
/// value its first transparent entry taken as the nodata value. noData is what a destination pixel
/// gets when none of the pixels it covers has an opaque colour (0 without a nodata value).
struct PaletteTable {
    std::vector<ColorEntry> entries;
    uint8_t noData = 0;
};
PaletteTable AveragePalette(std::vector<ColorEntry> entries, std::optional<double> nodata);

/// BestColorEntry (gcore/overview.cpp): the opaque entry nearest to (r, g, b) by squared RGB
/// distance, the first one on a tie (0 when every entry is transparent).
int BestColorEntry(const std::vector<ColorEntry>& entries, int r, int g, int b);

/// One destination window of a palette band (1 sample per pixel), with the window conventions of
/// Average: each destination pixel averages, unweighted, the colours of the opaque entries of the
/// whole pixels of its window (indices past the table left out), each channel rounded
/// ((total + count / 2) / count), and takes the entry BestColorEntry gives that colour; a pixel
/// without any opaque colour takes table.noData. No mask is read: GDAL's palette branch ignores it.
void PaletteAverage(const PaletteTable& table, const uint8_t* src, size_t srcStride, int srcX0, int srcY0,
                    const AverageTaps& cols, const AverageTaps& rows, int dstW, int dstH, uint8_t* dst,
                    size_t dstStride);

}  // namespace ddbt::cog::pyramid
