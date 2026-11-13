// SPDX-License-Identifier: MPL-2.0
// The point-cloud branch of ddbtMemoryTileFmt, with DroneDB 2.4.1's argument checks and
// messages in its order (ddb.cpp: requireText, requireBufferOutput, requireTileArgs;
// tilerhelper.cpp: parseImageFormat; the Tiler constructor: the tile size), as tile_api.cpp does
// for rasters.
#include "api/copc_tile.h"

#include <cctype>
#include <cstring>

#include "api/api_common.h"
#include "cog/tiler/tile_error.h"
#include "copc/tiler/copc_tiler.h"

namespace ddbt::api {

bool IsCopcPath(const char* path) noexcept {
    if (path == nullptr) return false;
    static constexpr char kSuffix[] = ".copc.laz";
    const size_t n = std::strlen(path), m = sizeof(kSuffix) - 1;
    if (n < m) return false;
    for (size_t i = 0; i < m; ++i)
        if (std::tolower(static_cast<unsigned char>(path[n - m + i])) != kSuffix[i]) return false;
    return true;
}

ddbtErr CopcMemoryTile(const char* inputPath, int tz, int tx, int ty, uint8_t** outBuffer, int* outBufferSize,
                       int tileSize, bool tms, bool forceRecreate, const char* inputPathHash,
                       const char* outputFormat) noexcept {
    (void)inputPathHash;
    return GuardedBuffer(outBuffer, outBufferSize, [&] {
        using cog::tiler::FailInvalid;
        if (NullOrBlank(inputPath)) FailInvalid("No input path provided");
        if (outBuffer == nullptr) FailInvalid("Output buffer pointer is null");
        if (outBufferSize == nullptr) FailInvalid("Output buffer size pointer is null");
        if (tileSize < 0) FailInvalid("Invalid tile size parameter");
        if (tz < 0 || tx < 0 || ty < 0) FailInvalid("Invalid tile coordinates");

        copc::tiler::TileRequest r;
        r.format = ParseFormat(outputFormat);
        // Tiler's constructor.
        if (tileSize <= 0 || (tileSize & (tileSize - 1)) != 0)
            FailInvalid("Tile size must be a power of 2 greater than 0");
        r.path = inputPath;
        RequireLocal(r.path);
        r.z = tz;
        r.x = tx;
        r.y = ty;
        r.tileSize = tileSize;
        r.tms = tms;
        r.noCache = forceRecreate;
        return copc::tiler::RenderTile(r);
    });
}

}  // namespace ddbt::api
