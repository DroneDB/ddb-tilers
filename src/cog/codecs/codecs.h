// SPDX-License-Identifier: MPL-2.0
//
// Block codecs of TIFF / COG files, decode side. Every decoder writes into a caller-provided
// buffer of the exact decoded size (the tile buffer) and never reads or writes outside the
// spans it is given, whatever the input bytes are. Decoders are stateless or keep per-thread
// state, so any number of threads may call them at once.
//
// Byte order: the lossless decoders (none, LZW, PackBits, DEFLATE, ZSTD) return the bytes as
// stored in the file; the TIFF reader byte-swaps them and undoes predictors (predictor.h).
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "core/util/status.h"

namespace ddbt::cog::codecs {

/// TIFF Compression tag values (the ones we know by name).
enum class Compression : uint16_t {
    kNone = 1,
    kCcittRle = 2,
    kCcittFax3 = 3,
    kCcittFax4 = 4,
    kLzw = 5,
    kOldJpeg = 6,
    kJpeg = 7,
    kAdobeDeflate = 8,
    kJbig = 34661,
    kNext = 32766,
    kPackBits = 32773,
    kThunderScan = 32809,
    kDeflate = 32946,  // the obsolete "Pixar" code GDAL / libtiff still read as DEFLATE
    kLerc = 34887,
    kLzma = 34925,
    kZstd = 50000,
    kWebp = 50001,
    kJxl = 50002,
    kJxlDng = 52546,
};

/// "LZW", "DEFLATE", ... or "UNKNOWN".
std::string_view CompressionName(uint16_t compression) noexcept;

/// True for the compressions DecodeLossless / the JPEG, WebP and LERC decoders handle.
bool IsSupportedCompression(uint16_t compression) noexcept;

/// Uncompressed: copies min(in, out) bytes; Corrupt when the input is shorter than the output
/// (libtiff's "Read error" on a short strip / tile).
Status DecodeNone(std::span<const uint8_t> in, std::span<uint8_t> out);

/// TIFF LZW (MSB-first codes, 9 to 12 bits, early change). The old-style (pre TIFF 5, LSB-first)
/// variant is rejected with Unsupported. Output stops when `out` is full; a stream that ends
/// before (no EOI and no more bits) is Corrupt, as in libtiff.
Status DecodeLzw(std::span<const uint8_t> in, std::span<uint8_t> out);

/// PackBits (Macintosh RLE). Runs that would overflow `out` are clipped (libtiff discards the
/// excess with a warning); an input that ends before `out` is full is Corrupt.
Status DecodePackBits(std::span<const uint8_t> in, std::span<uint8_t> out);

/// DEFLATE (zlib stream, TIFF codes 8 and 32946) through libdeflate. The stream must produce at
/// least out.size() bytes; extra output is discarded (libtiff stops when its buffer is full).
Status DecodeDeflate(std::span<const uint8_t> in, std::span<uint8_t> out);

/// Zstandard (one frame or more); the output must be at least out.size() bytes (extra bytes
/// are an error from libzstd: dstSize too small, reported Corrupt).
Status DecodeZstd(std::span<const uint8_t> in, std::span<uint8_t> out);

/// LERC as GDAL writes it in TIFF (LercParameters tag 50674: version, additional compression
/// 0 none / 1 DEFLATE / 2 ZSTD), with libtiff's mask handling: an 8-bit pixel-interleaved
/// block whose last extra sample is unassociated alpha gets its alpha from the LERC mask, and
/// floating-point samples that the mask marks invalid become NaN.
struct LercBlockInfo {
    uint32_t width = 0;        // segment width (tile width, or image width for strips)
    uint32_t height = 0;       // segment height (tile height, or rows of this strip)
    uint32_t samples = 1;      // samples in the block (spp for PIXEL interleave, 1 for BAND)
    uint32_t bitsPerSample = 8;
    uint16_t sampleFormat = 1;  // TIFF SampleFormat (1 uint, 2 int, 3 IEEE float)
    bool lastSampleIsUnassociatedAlpha = false;
    uint32_t additionalCompression = 0;  // 0 none, 1 DEFLATE, 2 ZSTD
};
Status DecodeLerc(std::span<const uint8_t> in, const LercBlockInfo& info, std::span<uint8_t> out);

/// WebP (lossy or lossless) into `samples` (3 = RGB, 4 = RGBA) interleaved bytes per pixel,
/// rows of width * samples bytes. The bitstream must be exactly width x height.
Status DecodeWebp(std::span<const uint8_t> in, uint32_t width, uint32_t height, uint32_t samples,
                  std::span<uint8_t> out);

}  // namespace ddbt::cog::codecs
