// SPDX-License-Identifier: MPL-2.0
//
// JPEG-in-TIFF decoder (libjpeg-turbo, libjpeg API), configured as libtiff's JPEG codec is when
// GDAL reads a file:
//  - JPEGTables (an abbreviated "tables only" datastream) is loaded before each block, so the
//    abbreviated block datastreams find their Huffman and quantization tables;
//  - Photometric YCbCr is converted to RGB by libjpeg (GDAL sets JPEGCOLORMODE=RGB): libjpeg's
//    defaults apply (islow IDCT, fancy upsampling), exactly as in tif_jpeg.c;
//  - any other photometric decodes the components as stored (no colour conversion).
// Optional scaled (1/2, 1/4, 1/8) and region (jpeg_crop_scanline + jpeg_skip_scanlines)
// decodes are the entry points of the tiler's partial reads.
//
// Each thread keeps one libjpeg decompressor; warnings (corrupt but decodable data) are
// counted, not printed, and decoding goes on as libtiff does.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "core/util/status.h"

namespace ddbt::cog::codecs {

struct JpegDecodeOptions {
    /// Content of the JPEGTables tag (may be empty).
    std::span<const uint8_t> tables;
    /// Photometric YCbCr with 3 samples: let libjpeg convert to RGB.
    bool ycbcrToRgb = false;
    /// Output scale 1/scaleDenom (1, 2, 4 or 8): libjpeg's DCT scaling.
    uint32_t scaleDenom = 1;
};

/// Header of a block datastream (after the tables were loaded).
struct JpegHeader {
    uint32_t width = 0;   // full-resolution size of the datastream
    uint32_t height = 0;
    uint32_t components = 0;
    uint32_t precision = 0;
    uint32_t hSampling = 1;  // sampling factors of component 0
    uint32_t vSampling = 1;
    bool progressive = false;
};

/// Reads the block's header (loading `options.tables` first).
Expected<JpegHeader> ReadJpegHeader(std::span<const uint8_t> in, const JpegDecodeOptions& options);

/// Size of the output at scale 1/denom: ceil(size / denom), libjpeg's rule.
constexpr uint32_t JpegScaledSize(uint32_t size, uint32_t denom) noexcept { return (size + denom - 1) / denom; }

/// Decodes the window [x, x + width) x [y, y + height) of the (scaled) output into `out`
/// (rows of outStride bytes, width * components bytes used per row). The window must lie inside
/// the scaled image. `expectedComponents` must equal the datastream's component count; the
/// datastream must be 8-bit. A full decode is the window (0, 0, scaled width, scaled height).
/// Columns are cropped with jpeg_crop_scanline (to the iMCU boundary on the left, then copied
/// from the row buffer), rows above the window are skipped with jpeg_skip_scanlines.
Status DecodeJpeg(std::span<const uint8_t> in, const JpegDecodeOptions& options, uint32_t expectedComponents,
                  uint32_t x, uint32_t y, uint32_t width, uint32_t height, uint8_t* out, size_t outStride);

/// Number of libjpeg warnings seen by the calling thread (corrupt but decodable data).
uint64_t JpegWarningCount() noexcept;

}  // namespace ddbt::cog::codecs
