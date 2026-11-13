// SPDX-License-Identifier: MPL-2.0
//
// The COG tiler: a web map tile of a GeoTIFF / COG, with DroneDB 2.4.1's GDALTiler semantics
// (gdaltiler.cpp, tiler.cpp, vis_render.cpp) and without GDAL.
//
//   TileRequest r{path, z, x, y, 256, /*tms*/ true, encode::TileFormat::kPng};
//   encode::ByteBuffer png = RenderTile(r);          // throws TileError
//
// Requests run on the caller's thread against the shared CogSource of the file (cog_source.h),
// the decoded-block cache (block_cache.h) and an encoded-tile cache (256 MiB by default,
// DDBT_TILE_CACHE_MB; the key holds the source version, the tile, its size, format, mode and
// visualisation parameters). There is no global lock on the hot path.
//
// Semantics kept from DroneDB's GDAL tiler: the inverted tms flag; the bounds of
// getMinMaxCoordsForZ (a plain tile outside them is blank, a tile with visualisation parameters
// fails "Out of bounds"); geoQuery's windows; GDAL's overview choice and nearest sampling; the
// data bands (dataBandCount; at most 3 shown); Byte bands as they are, other types stretched over
// the exact statistics (one band: its minimum to maximum; several: mean +- 2.33 sigma clamped to
// them; truncating stretchToByte, NaN shown as 255); the alpha of the alpha band or the mask; the
// visualisation parameters (preset, bands, formula, colormap, rescale, NoData override, a single
// band painted with a colormap).
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "cog/tiler/vis.h"
#include "core/cache/cache_stats.h"
#include "core/encode/byte_buffer.h"
#include "core/encode/tile_encoder.h"

namespace ddbt::cog::tiler {

/// How tile pixels are sampled.
enum class SamplingMode : uint8_t {
    /// GDAL's RasterIO on DroneDB's geoQuery window: the baseline's pixels (and its seams).
    kCompat,
    /// The overview from the zoom and the pixel under each tile pixel's centre: no seams.
    kDefault,
};

struct TileRequest {
    std::string path;
    int z = 0, x = 0, y = 0;
    int tileSize = 256;
    /// DroneDB's flag: true means y is an XYZ row (it is flipped to the TMS row internally).
    bool tms = false;
    encode::TileFormat format = encode::TileFormat::kPng;
    /// Visualisation parameters (DDBMemoryTileEx2); null for a plain tile (DDBMemoryTileFmt).
    const VisParams* vis = nullptr;
    /// Bypass the encoded-tile cache (forceRecreate).
    bool noCache = false;
};

/// Largest tile size (pixels a side): a tile's buffers grow with its square.
inline constexpr int kMaxTileSize = 4096;
/// Deepest zoom: rows and columns of z <= 30 fit an int.
inline constexpr int kMaxZoom = 30;
/// Per-thread working buffers larger than this (twice the RGBA image of a 512 tile) are released
/// after the request that grew them.
inline constexpr size_t kTileScratchKeepBytes = size_t{2} * 512 * 512 * 4;

/// Renders and encodes a tile. Throws TileError: kInvalidArgs for a tile size that is not a power
/// of 2 up to kMaxTileSize or coordinates outside z 0..kMaxZoom and [0, 2^z); kException when the
/// render would need more working memory than DDBT_TILE_REQUEST_MB (1024 MiB by default).
encode::ByteBuffer RenderTile(const TileRequest& request);

/// The largest working buffer the calling thread keeps between renders (tests).
size_t TileScratchBytes();

/// The process-wide sampling mode (DDBT_TILER_MODE=compat|default; compat when unset).
SamplingMode GetSamplingMode();
void SetSamplingMode(SamplingMode mode);

cache::CacheStats TileCacheStats();
void SetTileCacheBudget(size_t bytes);
void ClearTileCache();

}  // namespace ddbt::cog::tiler
