// SPDX-License-Identifier: MPL-2.0
#pragma once

#include "core/encode/byte_buffer.h"
#include "core/encode/tile_image.h"

namespace ddbt::encode {

/// The defaults are DroneDB's (QUALITY=80, LOSSLESS=FALSE) but METHOD=1 (DefaultWebpMethod), on top of what GDAL's WEBP
/// driver leaves at libwebp's WebPConfigInit defaults: preset DEFAULT, exact 0, alpha_compression
/// 1 (lossless alpha), alpha_filtering 1 (fast), alpha_quality 100, single-threaded.
/// The method -1 stands for in this process: $DDBT_WEBP_METHOD (0..6) when set, else 1. Method 1
/// encodes tiles 12-41 % faster than DroneDB 2.4's 2, with a slightly higher PSNR (method 2 spends
/// its extra work on size) and at most 8 % more bytes (256 px tiles of the test corpus).
int DefaultWebpMethod() noexcept;

/// `method`, or DefaultWebpMethod() when it is -1.
inline int ResolveWebpMethod(int method) noexcept { return method < 0 ? DefaultWebpMethod() : method; }

struct WebpOptions {
    float quality = 80.0f;  ///< 0..100
    int method = -1;        ///< 0 (fastest) .. 6 (smallest); -1 = DefaultWebpMethod()
    bool lossless = false;
    bool exact = false;  ///< keep RGB under alpha 0 (larger)
};

/// Encodes image as a WebP (libwebp) into out (contents replaced).
///
/// As DroneDB: always 3 colour bands (grey is expanded, palette images are looked up), plus the
/// alpha unless every pixel is opaque. The RGB/RGBA pixels are imported straight from the view
/// (WebPPictureImportRGB/RGBX/RGBA); the WebPPicture is kept per thread.
///
/// Throws std::invalid_argument, std::bad_alloc or std::runtime_error (libwebp failure).
void EncodeWebp(const TileImage& image, const WebpOptions& options, ByteBuffer& out);

}  // namespace ddbt::encode
