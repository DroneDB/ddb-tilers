// SPDX-License-Identifier: MPL-2.0
#pragma once

#include "core/encode/byte_buffer.h"
#include "core/encode/tile_image.h"

namespace ddbt::encode {

enum class JpegSubsampling : uint8_t { k420, k422, k444 };

/// The defaults are GDAL's JPEG driver (CreateCopy): jpeg_set_defaults (4:2:0, accurate integer
/// DCT, baseline), jpeg_set_quality(QUALITY, force_baseline) and optimized Huffman tables.
struct JpegOptions {
    int quality = 75;  ///< 1..100 (DroneDB: 75)
    JpegSubsampling subsampling = JpegSubsampling::k420;
    bool optimizeHuffman = true;
    bool fastDct = false;
    bool progressive = false;
};

/// Encodes image as a JPEG (libjpeg-turbo, TurboJPEG 3 API) into out (contents replaced).
///
/// JPEG has no alpha: pixels of alpha 0 are written white and the alpha is dropped, as DroneDB
/// does. Grey (and grey + alpha) images stay 1-band (greyscale JPEG); palette images are
/// expanded to RGB. One TurboJPEG handle per thread is kept for the life of the thread.
///
/// Throws std::invalid_argument, std::bad_alloc or std::runtime_error (TurboJPEG failure).
void EncodeJpeg(const TileImage& image, const JpegOptions& options, ByteBuffer& out);

}  // namespace ddbt::encode
