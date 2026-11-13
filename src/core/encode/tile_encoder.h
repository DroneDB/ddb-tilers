// SPDX-License-Identifier: MPL-2.0
//
// Tile encoding with DroneDB's semantics (DroneDB src/library/tile_encoder.cpp):
// - PNG ZLEVEL=1, JPEG QUALITY=75, WebP QUALITY=80 LOSSLESS=FALSE METHOD=2;
// - an alpha channel that is 255 everywhere is dropped;
// - JPEG: pixels of alpha 0 become white, grey stays 1-band; WebP: always 3 colour bands;
// - a fully transparent tile gives cached "blank" bytes, encoded once per format, size, colour
//   bands and options.
#pragma once

#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "core/encode/byte_buffer.h"
#include "core/encode/jpeg_encoder.h"
#include "core/encode/png_encoder.h"
#include "core/encode/tile_image.h"
#include "core/encode/webp_encoder.h"

namespace ddbt::encode {

enum class TileFormat : uint8_t { kPng, kJpeg, kWebp };

/// "png", "jpeg" / "jpg", "webp" or their MIME types, case-insensitive; "" is PNG (as DroneDB).
std::optional<TileFormat> ParseTileFormat(std::string_view name) noexcept;

/// File extension without the dot: "png", "jpg", "webp".
const char* TileFormatExtension(TileFormat format) noexcept;

/// MIME type: "image/png", "image/jpeg", "image/webp".
const char* TileFormatMimeType(TileFormat format) noexcept;

struct EncodeOptions {
    TileFormat format = TileFormat::kPng;
    /// JPEG / WebP quality 1..100; 0 = the format default (JPEG 75, WebP 80); above 100 is 100.
    /// It replaces jpeg.quality and webp.quality.
    int quality = 0;
    PngOptions png;
    JpegOptions jpeg;
    WebpOptions webp;
};

/// Quality used for options (the format default when options.quality is 0); 0 for PNG.
int EffectiveQuality(const EncodeOptions& options) noexcept;

/// Encodes image into out (contents replaced, capacity kept). A fully transparent image gives
/// BlankTileBytes. The alpha summary is scanned when image.alphaKnown is false.
/// Throws std::invalid_argument, std::bad_alloc or std::runtime_error.
void EncodeTile(const TileImage& image, const EncodeOptions& options, ByteBuffer& out);

/// Same, into a new buffer (release() it for the C API: free with ddbtFree).
ByteBuffer EncodeTile(const TileImage& image, const EncodeOptions& options);

/// Encoded bytes of a fully transparent image of the given size and colour channels (1 grey,
/// 3 RGB), for options. Encoded on first use and kept for the life of the process: the
/// reference stays valid and is the same object for the same key. Thread safe.
const std::vector<uint8_t>& BlankTileBytes(uint32_t width, uint32_t height, uint32_t colorChannels,
                                           const EncodeOptions& options);

}  // namespace ddbt::encode
