// SPDX-License-Identifier: MPL-2.0
//
// Encoding of one COG block (a tile of the image or of its 1-bit mask) as GDAL's GTiff driver
// writes it, without libtiff:
//
//   - lossless: predictor 2 (horizontal differencing in the sample width) or 3 (floating point:
//     byte planes, most significant first, then byte differencing), then LZW (own encoder),
//     DEFLATE (libdeflate, zlib stream, level 6 = GDAL's ZLEVEL default) or ZSTD (level 9 =
//     GDAL's ZSTD_LEVEL default), or none;
//   - JPEG: libjpeg configured as libtiff's JPEG codec with PHOTOMETRIC=YCBCR and
//     JPEGCOLORMODE=RGB (RGB in, YCbCr 2x2 subsampling, no JFIF marker, islow DCT, optimized
//     Huffman tables) and JPEGTABLESMODE=1: the quantization tables live in the JPEGTables tag
//     (JpegTables()), each tile is an abbreviated datastream without them;
//   - masks: rows of packed bits (MSB first, a set bit = valid), DEFLATE level 6.
//
// Partial edge tiles: the caller passes a full block; for JPEG the pixels right of and below the
// raster are replaced by copies of the last column / row first (GDAL's WriteEncodedTile does so
// to avoid edge artefacts), see SpreadJpegEdges.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "cog/tiff/tiff_reader.h"
#include "core/encode/byte_buffer.h"

namespace ddbt::cog::encode {

using ddbt::encode::ByteBuffer;

enum class Codec : uint8_t { kNone, kLzw, kDeflate, kZstd, kJpeg };

/// TIFF Compression tag value of a codec.
uint16_t CompressionTag(Codec c) noexcept;
/// "NONE", "LZW", "DEFLATE", "ZSTD", "JPEG".
const char* CodecName(Codec c) noexcept;

struct BlockFormat {
    uint32_t width = 512;    // block width (pixels)
    uint32_t height = 512;   // block height
    uint32_t samples = 1;    // samples per pixel stored in the block
    tiff::SampleType type = tiff::SampleType::kU8;
    Codec codec = Codec::kLzw;
    int predictor = 1;       // 1 none, 2 horizontal, 3 floating point (lossless codecs)
    int level = 0;           // DEFLATE / ZSTD level; 0 = GDAL's default (6 / 9)
    int jpegQuality = 75;
};

/// Encodes one full block (format.width x format.height pixels of format.samples samples, host
/// byte order, rows packed) and appends it to `out`. The pixels are not modified.
/// Throws std::runtime_error on a codec failure, std::invalid_argument on a bad format.
void EncodeBlock(const BlockFormat& format, const uint8_t* pixels, ByteBuffer& out);

/// Replicates the last valid column / row of an RGB(A) 8-bit block into the part of the block
/// outside the raster (validWidth x validHeight are the pixels inside), in place.
void SpreadJpegEdges(uint8_t* pixels, uint32_t blockWidth, uint32_t blockHeight, uint32_t samples,
                     uint32_t validWidth, uint32_t validHeight);

/// A 1-bit mask block from bytes (0 = invalid, anything else = valid), width x height, rows of
/// `width` bytes: packed MSB first, DEFLATE level 6, appended to `out`.
void EncodeMaskBlock(const uint8_t* mask, uint32_t width, uint32_t height, ByteBuffer& out);

/// The JPEGTables tag of JPEG YCbCr tiles of `quality`: a tables-only datastream (SOI, the
/// luminance and chrominance quantization tables in two DQT markers, EOI), as libtiff writes it.
std::vector<uint8_t> JpegTables(int quality);

/// The quality whose libjpeg quantization tables are those of a JPEGTables tag (GDAL's
/// GuessJPEGQuality for 8-bit data): the first of 1..100 that matches, -1 when none does or the
/// tag has no quantization table. `components`: 1 (one table) or 3 (luminance + chrominance).
int GuessJpegQuality(std::span<const uint8_t> jpegTables, int components);

/// Forward predictors, in place on a block (rows of width * samples samples).
void ApplyHorizontalPredictor(uint8_t* data, uint32_t width, uint32_t height, uint32_t samples, size_t sampleBytes);
void ApplyFloatingPointPredictor(const uint8_t* in, uint8_t* out, uint32_t width, uint32_t height, uint32_t samples,
                                 size_t sampleBytes);

}  // namespace ddbt::cog::encode
