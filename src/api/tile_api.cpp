// SPDX-License-Identifier: MPL-2.0
// The tile exports of the C API: ddbtMemoryTileFmt (DDBMemoryTileFmt), ddbtMemoryTileEx2
// (DDBMemoryTileEx2) and ddbtMemoryTileEx3 (DDBMemoryTileEx3, libddb 2.5), with DroneDB 2.4.1's argument checks and messages (ddb.cpp: requireText,
// requireBufferOutput, requireTileArgs, parseImageFormat, rasterOptions, rescaleOf) in its order.
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <new>
#include <optional>
#include <string>

#include "cog/tiler/cog_tiler.h"
#include "api/copc_tile.h"
#include "cog/tiler/tile_error.h"
#include "cog/tiler/vis.h"
#include "core/util/last_error.h"
#include "ddbt/ddbt.h"

namespace {

using ddbt::cog::tiler::TileError;
using ddbt::cog::tiler::TileErrorCode;

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

// Runs body(buffer) and hands the bytes to the caller only when it returned (guardedBuffer).
template <typename Body>
ddbtErr GuardedBuffer(uint8_t** outBuffer, int* outBufferSize, Body&& body) noexcept {
    try {
        ddbt::ClearLastError();
        ddbt::encode::ByteBuffer bytes = body();
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

bool NullOrBlank(const char* s) {
    if (s == nullptr) return true;
    for (; *s; ++s)
        if (!std::isspace(static_cast<unsigned char>(*s))) return false;
    return true;
}

std::string OptStr(const char* s) { return s != nullptr ? std::string(s) : std::string(); }

void RequireCommon(const char* inputPath, uint8_t** outBuffer, int* outBufferSize, int tz, int tx, int ty,
                   int tileSize) {
    using ddbt::cog::tiler::FailInvalid;
    if (NullOrBlank(inputPath)) FailInvalid("No input path provided");
    if (outBuffer == nullptr) FailInvalid("Output buffer pointer is null");
    if (outBufferSize == nullptr) FailInvalid("Output buffer size pointer is null");
    if (tileSize < 0) FailInvalid("Invalid tile size parameter");
    if (tz < 0 || tx < 0 || ty < 0) FailInvalid("Invalid tile coordinates");
    // Checked before any tile maths: 2^tz must fit an int, and the tile must be on the grid.
    using ddbt::cog::tiler::kMaxZoom;
    if (tz > kMaxZoom || tx >= (1 << tz) || ty >= (1 << tz)) FailInvalid("Invalid tile coordinates");
}

void RequireTileSize(int tileSize) {
    // GDALTiler's constructor (libddb: a GDALException, after the input exists).
    if (tileSize <= 0 || (tileSize & (tileSize - 1)) != 0)
        ddbt::cog::tiler::FailInvalid("Tile size must be a power of 2 greater than 0");
    // Not in libddb: the buffers of a tile grow with its square.
    if (tileSize > ddbt::cog::tiler::kMaxTileSize)
        ddbt::cog::tiler::FailInvalid("Tile size must be at most " + std::to_string(ddbt::cog::tiler::kMaxTileSize));
}

void RequireLocal(const std::string& path) {
    const auto starts = [&](const char* p) {
        const size_t n = std::strlen(p);
        if (path.size() < n) return false;
        for (size_t i = 0; i < n; ++i)
            if (std::tolower(static_cast<unsigned char>(path[i])) != p[i]) return false;
        return true;
    };
    if (starts("http://") || starts("https://") || starts("s3://") || starts("/vsi"))
        throw TileError(TileErrorCode::kNotImplemented, "Remote inputs are not supported yet: " + path);
}

ddbt::encode::TileFormat ParseFormat(const char* outputFormat) {
    // tile_encoder.cpp parseImageFormat (case-insensitive).
    std::string f = OptStr(outputFormat);
    std::string l = f;
    for (auto& c : l) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (l.empty() || l == "png" || l == "image/png") return ddbt::encode::TileFormat::kPng;
    if (l == "jpeg" || l == "jpg" || l == "image/jpeg" || l == "image/jpg") return ddbt::encode::TileFormat::kJpeg;
    if (l == "webp" || l == "image/webp") return ddbt::encode::TileFormat::kWebp;
    ddbt::cog::tiler::FailInvalid("Unsupported image format: " + f);
}

}  // namespace

extern "C" {

ddbtErr ddbtMemoryTileFmt(const char* inputPath, int tz, int tx, int ty, uint8_t** outBuffer, int* outBufferSize,
                          int tileSize, bool tms, bool forceRecreate, const char* inputPathHash,
                          const char* outputFormat) {
    // A COPC point cloud (".copc.laz", as DroneDB's isCopcPath) is tiled by the point-cloud tiler
    if (inputPath != nullptr && ddbt::api::IsCopcPath(inputPath))
        return ddbt::api::CopcMemoryTile(inputPath, tz, tx, ty, outBuffer, outBufferSize, tileSize, tms,
                                         forceRecreate, inputPathHash, outputFormat);
    (void)inputPathHash;
    return GuardedBuffer(outBuffer, outBufferSize, [&] {
        RequireCommon(inputPath, outBuffer, outBufferSize, tz, tx, ty, tileSize);
        ddbt::cog::tiler::TileRequest r;
        r.format = ParseFormat(outputFormat);
        RequireTileSize(tileSize);
        r.path = inputPath;
        RequireLocal(r.path);
        r.z = tz;
        r.x = tx;
        r.y = ty;
        r.tileSize = tileSize;
        r.tms = tms;
        r.noCache = forceRecreate;
        return ddbt::cog::tiler::RenderTile(r);
    });
}

}  // extern "C"

namespace {

// ddbtMemoryTileEx2 (PNG) and ddbtMemoryTileEx3: the visualisation tile in outputFormat (NULL or "": PNG).
ddbtErr MemoryTileWithVis(const char* inputPath, int tz, int tx, int ty, int tileSize, bool tms, bool forceRecreate,
                          const char* preset, const char* bands, const char* formula, const char* bandFilter,
                          const char* colormap, const char* rescale, const ddbtRasterOptions* options,
                          const char* outputFormat, uint8_t** outBuffer, int* outBufferSize) {
    return GuardedBuffer(outBuffer, outBufferSize, [&] {
        using namespace ddbt::cog::tiler;
        RequireCommon(inputPath, outBuffer, outBufferSize, tz, tx, ty, tileSize);

        // rasterOptions
        std::optional<double> nodata;
        std::optional<TemperatureUnit> rescaleUnit;
        if (options != nullptr) {
            const size_t minSize = offsetof(ddbtRasterOptions, formula);
            const size_t size = options->structSize;
            if (size < minSize)
                FailInvalid("options->structSize is " + std::to_string(size) + ", expected at least " +
                            std::to_string(minSize));
            const auto* bytes = reinterpret_cast<const unsigned char*>(options);
            for (size_t i = sizeof(ddbtRasterOptions); i < size; i++)
                if (bytes[i] != 0)
                    FailInvalid("options sets a field this libddb does not know (structSize " + std::to_string(size) +
                                ", known " + std::to_string(sizeof(ddbtRasterOptions)) + ")");
            ddbtRasterOptions o{};
            std::memcpy(&o, options, std::min(size, sizeof(ddbtRasterOptions)));
            if ((o.flags & ~(DDBT_RASTER_OPT_NODATA | DDBT_RASTER_OPT_BBOX)) != 0)
                FailInvalid("options->flags has an unknown bit");
            if ((o.flags & DDBT_RASTER_OPT_NODATA) != 0) {
                if (!std::isfinite(o.nodata)) FailInvalid("options->nodata must be a finite number");
                nodata = o.nodata;
            }
            if ((o.flags & DDBT_RASTER_OPT_BBOX) != 0)
                for (const double v : o.bbox)
                    if (!std::isfinite(v)) FailInvalid("options->bbox must hold finite numbers");
            if (size >= offsetof(ddbtRasterOptions, rescaleUnit) + sizeof(o.rescaleUnit) && o.rescaleUnit != nullptr &&
                o.rescaleUnit[0] != '\0') {
                rescaleUnit = ParseTemperatureUnit(o.rescaleUnit);
                if (!rescaleUnit)
                    FailInvalid(std::string("options->rescaleUnit is not a temperature unit (K, C or F): ") +
                                o.rescaleUnit);
            }
        }
        // rescaleOf: a thermal rescale in another unit is converted to the formula's unit.
        std::string rescaleText = OptStr(rescale);
        if (!rescaleText.empty() && rescaleUnit) {
            if (const auto target = ThermalFormulaUnit(OptStr(formula)))
                rescaleText = ConvertRescale(rescaleText, *rescaleUnit, *target);
        }
        VisParams vis;
        vis.preset = OptStr(preset);
        vis.bands = OptStr(bands);
        vis.formula = OptStr(formula);
        vis.bandFilter = OptStr(bandFilter);
        vis.colormap = OptStr(colormap);
        vis.rescale = rescaleText;
        vis.nodata = nodata;

        // libddb parses the format before it opens the tiler (TilerHelper::getTile), so after the options
        const ddbt::encode::TileFormat format = ParseFormat(outputFormat);
        RequireTileSize(tileSize);
        TileRequest r;
        r.path = inputPath;
        RequireLocal(r.path);
        r.z = tz;
        r.x = tx;
        r.y = ty;
        r.tileSize = tileSize;
        r.tms = tms;
        r.noCache = forceRecreate;
        r.format = format;
        r.vis = &vis;
        return RenderTile(r);
    });
}

}  // namespace

extern "C" {

ddbtErr ddbtMemoryTileEx2(const char* inputPath, int tz, int tx, int ty, int tileSize, bool tms, bool forceRecreate,
                          const char* inputPathHash, const char* preset, const char* bands, const char* formula,
                          const char* bandFilter, const char* colormap, const char* rescale,
                          const ddbtRasterOptions* options, uint8_t** outBuffer, int* outBufferSize) {
    (void)inputPathHash;
    return MemoryTileWithVis(inputPath, tz, tx, ty, tileSize, tms, forceRecreate, preset, bands, formula, bandFilter,
                             colormap, rescale, options, nullptr, outBuffer, outBufferSize);
}

ddbtErr ddbtMemoryTileEx3(const char* inputPath, int tz, int tx, int ty, int tileSize, bool tms, bool forceRecreate,
                          const char* inputPathHash, const char* preset, const char* bands, const char* formula,
                          const char* bandFilter, const char* colormap, const char* rescale,
                          const ddbtRasterOptions* options, const char* outputFormat, uint8_t** outBuffer,
                          int* outBufferSize) {
    (void)inputPathHash;
    return MemoryTileWithVis(inputPath, tz, tx, ty, tileSize, tms, forceRecreate, preset, bands, formula, bandFilter,
                             colormap, rescale, options, outputFormat, outBuffer, outBufferSize);
}

}  // extern "C"
