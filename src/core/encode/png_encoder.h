// SPDX-License-Identifier: MPL-2.0
#pragma once

#include "core/encode/byte_buffer.h"
#include "core/encode/kernels.h"
#include "core/encode/tile_image.h"

namespace ddbt::encode {

using PngRowFilter = kernels::PngRowFilter;

/// How the image data of grey / RGB images is compressed. The pixels are the same either way.
enum class PngEngine : uint8_t {
    kDefault,  ///< the process default: $DDBT_PNG_ENGINE (fpnge or deflate), else kDeflate
    kDeflate,  ///< our filters (PngOptions::filter) + libdeflate at PngOptions::level
    kFpnge,    ///< fpnge (third_party/fpnge): SIMD filters + Huffman / RLE, at fpngeLevel
};

/// The engine kDefault stands for in this process.
PngEngine DefaultPngEngine() noexcept;

struct PngOptions {
    PngEngine engine = PngEngine::kDefault;
    /// fpnge level 1 (fastest) to 5 (smallest); fpnge's default is 4.
    int fpngeLevel = 4;
    /// libdeflate level, 0 (stored) to 12. DroneDB writes tiles with GDAL's ZLEVEL=1.
    int level = 1;
    /// Row filter of grey / RGB images (palette images are always unfiltered, as libpng does).
    PngRowFilter filter = PngRowFilter::kAdaptive;
    /// An alpha channel that is 255 everywhere is not written (grey / RGB instead), as DroneDB.
    bool dropOpaqueAlpha = true;
};

/// Encodes image as an 8-bit PNG into out (its contents are replaced; its capacity is kept).
///
/// Colour type: 0 grey, 4 grey + alpha, 2 RGB, 6 RGBA, or 3 for a palette image (PLTE, plus a
/// tRNS chunk when an entry is not opaque and the image is not known to be all opaque). One
/// IDAT, no ancillary chunk. Palette images always use libdeflate; others use the engine of
/// options. The alpha summary of image is computed when the caller did not give it.
///
/// Throws std::invalid_argument for an unusable image or option, std::bad_alloc, or
/// std::runtime_error when libdeflate fails.
void EncodePng(const TileImage& image, const PngOptions& options, ByteBuffer& out);

}  // namespace ddbt::encode
