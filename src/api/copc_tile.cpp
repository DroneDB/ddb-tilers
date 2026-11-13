// SPDX-License-Identifier: MPL-2.0
// The point-cloud branch of ddbtMemoryTileFmt, with DroneDB 2.4.1's argument checks and
// messages in its order (ddb.cpp: requireText, requireBufferOutput, requireTileArgs;
// tilerhelper.cpp: parseImageFormat; the Tiler constructor: the tile size), as tile_api.cpp does
// for rasters.
#include "api/copc_tile.h"

#include <cctype>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <new>
#include <string>

#include "cog/tiler/tile_error.h"
#include "copc/tiler/copc_tiler.h"
#include "core/util/last_error.h"

namespace ddbt::api {

namespace {

using cog::tiler::TileError;
using cog::tiler::TileErrorCode;

ddbtErr Report(ddbtErr code, const std::string& message) {
    ddbt::SetLastError(message);
    return code;
}

ddbtErr CodeOf(TileErrorCode c) {
    switch (c) {
        case TileErrorCode::kInvalidArgs:
        case TileErrorCode::kRefused: return DDBT_ERR_INVALIDARGS;
        case TileErrorCode::kNotFound: return DDBT_ERR_NOTFOUND;
        case TileErrorCode::kNotImplemented: return DDBT_ERR_NOT_IMPLEMENTED;
        case TileErrorCode::kException: break;
    }
    return DDBT_ERR_EXCEPTION;
}

bool NullOrBlank(const char* s) {
    if (s == nullptr) return true;
    for (; *s; ++s)
        if (!std::isspace(static_cast<unsigned char>(*s))) return false;
    return true;
}

bool StartsWithCI(const std::string& s, const char* prefix) {
    const size_t n = std::strlen(prefix);
    if (s.size() < n) return false;
    for (size_t i = 0; i < n; ++i)
        if (std::tolower(static_cast<unsigned char>(s[i])) != prefix[i]) return false;
    return true;
}

encode::TileFormat ParseFormat(const char* outputFormat) {
    const std::string f = outputFormat != nullptr ? outputFormat : "";
    const auto parsed = encode::ParseTileFormat(f);
    if (!parsed) cog::tiler::FailInvalid("Unsupported image format: " + f);
    return *parsed;
}

}  // namespace

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
    try {
        ddbt::ClearLastError();
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
        if (StartsWithCI(r.path, "http://") || StartsWithCI(r.path, "https://") || StartsWithCI(r.path, "s3://") ||
            StartsWithCI(r.path, "/vsi"))
            throw TileError(TileErrorCode::kNotImplemented, "Remote inputs are not supported yet: " + r.path);
        r.z = tz;
        r.x = tx;
        r.y = ty;
        r.tileSize = tileSize;
        r.tms = tms;
        r.noCache = forceRecreate;
        encode::ByteBuffer bytes = copc::tiler::RenderTile(r);
        if (bytes.size() > static_cast<size_t>(INT32_MAX)) return Report(DDBT_ERR_EXCEPTION, "Tile too large");
        const int size = static_cast<int>(bytes.size());
        uint8_t* data = bytes.release();
        if (data == nullptr) {
            data = static_cast<uint8_t*>(std::malloc(1));
            if (data == nullptr) return Report(DDBT_ERR_EXCEPTION, "Out of memory");
        }
        *outBuffer = data;
        *outBufferSize = size;
        return DDBT_ERR_NONE;
    } catch (const TileError& e) {
        return Report(CodeOf(e.code()), e.what());
    } catch (const std::bad_alloc&) {
        return Report(DDBT_ERR_EXCEPTION, "Out of memory");
    } catch (const std::exception& e) {
        return Report(DDBT_ERR_EXCEPTION, e.what());
    } catch (...) {
        return Report(DDBT_ERR_EXCEPTION, "Unknown error occurred");
    }
}

}  // namespace ddbt::api
