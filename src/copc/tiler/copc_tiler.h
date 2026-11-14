// SPDX-License-Identifier: MPL-2.0
//
// The COPC tiler: a web map tile of a COPC point cloud, with DroneDB 2.4.1's PointCloudTiler
// semantics (pctiler.cpp, copc_node_cache.cpp, point_rasterizer.cpp, tiler.cpp) and without PDAL.
//
//   TileRequest r{path, z, x, y, 256, /*tms*/ true, encode::TileFormat::kPng};
//   encode::ByteBuffer png = RenderTile(r);          // throws cog::tiler::TileError
//
// Three modes (DDBT_COPC_TILER_MODE=compat|hybrid|modern, else DDBT_TILER_MODE, else hybrid; see
// GetMode()):
//  - compat reproduces the baseline: window = the two corners of the tile padded by the
//    truncated int(res * ceil(T / 40)) metres, transformed to the cloud CRS; depth limit from the
//    cloud's spacing (in its own units) against res(z - 2); coloured clouds read in DroneDB's node
//    order with its per-tile 16-bit decision, colourless clouds ramp-coloured (pestel_shades over
//    the header z range, as filters.colorinterp) in readers.copc's node order; every point
//    transformed exactly (PROJ); PointRasterizer's 11-pixel disc 4 px west of the point, its
//    centre-only z-test and order dependence (see rasterizer.h). Bit-exact with PointRasterizer;
//    against the baseline, equal up to PDAL's own non-determinism (colourless clouds: the node
//    order of readers.copc's threads).
//  - hybrid (the default) keeps the baseline's look and fixes its bugs: the baseline's depth
//    limit (spacing in the cloud's units against res(z - 2), as compat), its 11-pixel stamp and
//    its ramp colours (filters.colorinterp's range), its 8/16-bit colour rule decided once per
//    cloud; the stamp is drawn on the point (its 4 x 4 box centred on the pixel corner nearest
//    the point, where the baseline draws it 4 px west and 1 px north), with a per-pixel z-test
//    (max key: order independent, deterministic), and the window and fit of the modern mode
//    (no seams at z21+). Against the baseline it is the same picture moved by (4, 1) px, up to
//    which point wins a pixel.
//  - modern fixes everything: window = 32 points of the padded tile's outline (padding = disc
//    radius + 1 px, in doubles, so no seams at z21+), at the lowest and highest heights for a
//    geocentric CRS; depth limit from the spacing projected to EPSG:3857 (spacing x sqrt|det J| of
//    cloud -> EPSG:3857 at the tile centre); a polynomial fit of cloud -> pixel per tile (affine,
//    quadratic or cubic, max residual <= 0.01 px, else exact PROJ); colours with the per-cloud
//    16-bit decision, the ramp over the exact header z range; the 13-pixel symmetric disc on the
//    point's pixel with a per-pixel z-test (max key), independent of the point order:
//    deterministic whatever the threads.
//
// Every mode keeps the baseline's contract: the inverted tms flag, "Out of bounds [...]" for a
// tile outside the footprint's tile range (any zoom accepted), the error messages of
// PointCloudTiler's constructor. Data path: the shared Cloud of the file (cloud.h), the
// process-wide node cache (copc/cache/node_cache.h), per-thread scratch buffers, blank tiles
// as cached bytes (outside the footprint, no node selected, nothing drawn), and an encoded-tile
// cache (DDBT_COPC_TILE_CACHE_MB, default 64; 0 disables).
#pragma once

#include "core/tile/compat.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/cache/cache_stats.h"
#include "core/encode/byte_buffer.h"
#include "core/encode/tile_encoder.h"

namespace ddbt::copc::tiler {

class Cloud;

enum class Mode : uint8_t { kCompat, kHybrid, kModern };

/// "compat", "hybrid", "modern"; "default" is the default mode (hybrid). nullopt otherwise.
std::optional<Mode> ParseMode(std::string_view name);
const char* ModeName(Mode mode);

struct TileRequest {
    std::string path;
    int z = 0, x = 0, y = 0;
    int tileSize = 256;
    /// DroneDB's flag: true means y is an XYZ row (it is flipped to the TMS row internally).
    bool tms = false;
    encode::TileFormat format = encode::TileFormat::kPng;
    /// Bypass the encoded-tile cache (forceRecreate).
    bool noCache = false;
    /// Unset: the process mode (GetMode()).
    std::optional<Mode> mode;
};

/// What a render did (tests, tools, benchmarks).
struct RenderInfo {
    enum class Transform : uint8_t { kNone, kExact, kAffine, kQuadratic, kCubic };
    bool blank = false;          // a blank shortcut (outside the footprint, no node) or nothing drawn
    uint32_t nodes = 0;          // nodes selected
    uint64_t points = 0;         // points of the selected nodes
    uint64_t inWindow = 0;       // points in the read window
    uint64_t opaque = 0;         // opaque pixels
    int depthEnd = 0;
    Transform transform = Transform::kNone;
    double fitResidual = 0;      // max residual of the fit (pixels)
    bool rawFallback = false;    // compat: re-read the raw colours for the 16-bit decision
};

/// Renders and encodes a tile. Throws cog::tiler::TileError.
encode::ByteBuffer RenderTile(const TileRequest& request);

/// Renders a tile of `cloud` into RGBA (tileSize^2 * 4 bytes, resized), without encoding or the
/// tile cache. Throws cog::tiler::TileError (out of bounds, transform failures).
void RenderRgba(const Cloud& cloud, const TileRequest& request, std::vector<uint8_t>& rgba,
                RenderInfo* info = nullptr);

/// A thumbnail: the whole cloud fitted, centred, in a square of size x size pixels.
struct ThumbnailRequest {
    std::string path;
    int size = 512;
    /// The points it may read: the deepest octree levels whose points, with those of every
    /// shallower level, are at most this many (at least the root); 0 = 2 per pixel.
    uint64_t pointBudget = 0;
};

/// Renders a thumbnail into RGBA (size^2 * 4 bytes, straight alpha, transparent background),
/// drawn as the modern tiles are (the 13-pixel disc on each point, a per-pixel depth test,
/// coloured points in their colours, others in the ramp over the header z range): in EPSG:3857
/// (north up) when the cloud has a footprint, else in its own x, y (a cloud without a CRS, or
/// without a transformation to EPSG:3857). Throws cog::tiler::TileError: kInvalidArgs for a
/// bad size, kException for a cloud of zero extent or without points, like RenderTile else.
void RenderThumbnail(const ThumbnailRequest& request, std::vector<uint8_t>& rgba, RenderInfo* info = nullptr);

/// The footprint and zoom range of `path` for `tileSize` px tiles in the process-wide mode, as
/// PointCloudTiler's constructor computes them (the footprint the tiles are bounded by, the span
/// DroneDB uses). Throws cog::tiler::TileError like RenderTile.
tile::compat::TileExtent Extent(const std::string& path, int tileSize);

/// The process-wide mode: DDBT_COPC_TILER_MODE (compat|hybrid|modern), else DDBT_TILER_MODE (the
/// variable the COG tiler reads: compat|default, where default means this tiler's default; hybrid
/// and modern are accepted too), else hybrid. Unknown values fall through to the next source.
Mode GetMode();
void SetMode(Mode mode);

/// Zoom levels between a tile and the octree level it reads (DroneDB's
/// DDB_PCTILE_RESOLUTION_OFFSET): DDBT_PCTILE_RESOLUTION_OFFSET, default 2.
int ResolutionOffset();

::ddbt::cache::CacheStats TileCacheStats();
void SetTileCacheBudget(size_t bytes);
size_t TileCacheBudget();
void ClearTileCache();

}  // namespace ddbt::copc::tiler
