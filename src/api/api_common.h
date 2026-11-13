// SPDX-License-Identifier: MPL-2.0
#pragma once

// Helpers shared by the C API exports (src/api): the last-error report, argument checks, error
// code mapping and the hand-over of strings to the caller.

#include <climits>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <new>
#include <string>

#include "cog/tiler/tile_error.h"
#include "core/encode/tile_encoder.h"
#include "core/util/last_error.h"
#include "core/util/status.h"
#include "ddbt/ddbt.h"

namespace ddbt::api {

/// Records `message` as the calling thread's last error (ddbtLastError) and returns `code`.
ddbtErr Fail(ddbtErr code, const std::string& message);

/// NULL or "".
bool IsEmpty(const char* s) noexcept;
/// NULL, "" or only white space.
bool NullOrBlank(const char* s) noexcept;

/// The code of a tile error.
ddbtErr CodeOf(cog::tiler::TileErrorCode code) noexcept;
/// The code of a failed Status: invalid argument, not found, unsupported (not implemented),
/// cancelled; anything else is DDBT_ERR_EXCEPTION.
ddbtErr CodeOf(const Status& status) noexcept;

/// Whether the UTF-8 path exists (no exception).
bool PathExists(const std::string& utf8Path);

/// Copies `text` into a NUL-terminated buffer the caller releases with ddbtFree.
ddbtErr CopyOut(const std::string& text, char** out);

/// The tile format of an output format name (DroneDB's parseImageFormat: "png", "jpeg" / "jpg",
/// "webp" or their MIME types, case-insensitive; NULL or "" is PNG). Throws TileError
/// (kInvalidArgs, "Unsupported image format: X") otherwise.
encode::TileFormat ParseFormat(const char* outputFormat);

/// Throws TileError (kNotImplemented) for a remote path (http, https, s3, GDAL /vsi).
void RequireLocal(const std::string& path);

/// Clears the last error and runs body(), which returns a ddbtErr. Exceptions become codes and
/// the last error: TileError by its code, bad_alloc "Out of memory", anything else
/// DDBT_ERR_EXCEPTION.
template <typename Body>
ddbtErr GuardedCall(Body&& body) noexcept {
    try {
        ClearLastError();
        return body();
    } catch (const cog::tiler::TileError& e) {
        return Fail(CodeOf(e.code()), e.what());
    } catch (const std::bad_alloc&) {
        return Fail(DDBT_ERR_EXCEPTION, "Out of memory");
    } catch (const std::exception& e) {
        return Fail(DDBT_ERR_EXCEPTION, e.what());
    } catch (...) {
        return Fail(DDBT_ERR_EXCEPTION, "Unknown error occurred");
    }
}

/// GuardedCall of body(), which returns the encoded tile: its bytes go to the caller only when it
/// returned (a malloc'd buffer the caller releases with ddbtFree; one byte for an empty tile).
template <typename Body>
ddbtErr GuardedBuffer(uint8_t** outBuffer, int* outBufferSize, Body&& body) noexcept {
    return GuardedCall([&]() -> ddbtErr {
        encode::ByteBuffer bytes = body();
        if (bytes.size() > static_cast<size_t>(INT32_MAX)) return Fail(DDBT_ERR_EXCEPTION, "Tile too large");
        const int size = static_cast<int>(bytes.size());
        uint8_t* data = bytes.release();
        if (data == nullptr) {
            data = static_cast<uint8_t*>(std::malloc(1));
            if (data == nullptr) return Fail(DDBT_ERR_EXCEPTION, "Out of memory");
        }
        *outBuffer = data;
        *outBufferSize = size;
        return DDBT_ERR_NONE;
    });
}

}  // namespace ddbt::api
