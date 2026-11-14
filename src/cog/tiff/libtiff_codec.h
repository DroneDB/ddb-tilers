// SPDX-License-Identifier: MPL-2.0
#pragma once

// The blocks of the compressions ddbt has no codec of its own for, decoded by libtiff: CCITT RLE,
// Group 3 and Group 4 fax, old-style JPEG, LZMA, 12-bit JPEG, old-style LZW, NeXT, ThunderScan,
// PixarLog, SGILog. They are rare in geodata, and GDAL reads them through the same libtiff, so
// the samples are GDAL's.

#include <cstdint>
#include <memory>
#include <span>

#include "core/io/byte_source.h"
#include "core/util/status.h"

namespace ddbt::cog::tiff {

class LibtiffCodec {
public:
    /// Whether libtiff (as built) decodes `compression`.
    static bool Supports(uint16_t compression) noexcept;

    explicit LibtiffCodec(std::shared_ptr<const io::ByteSource> source);
    ~LibtiffCodec();
    LibtiffCodec(const LibtiffCodec&) = delete;
    LibtiffCodec& operator=(const LibtiffCodec&) = delete;

    /// Decodes `raw`, the stored bytes of block `index` of the directory at file offset
    /// `ifdOffset`, into `out` in libtiff's layout: samples packed as stored (rows padded to a
    /// byte), host byte order, predictor undone, bit order of FillOrder 2 restored. Thread-safe:
    /// the decodes of one file take turns on its libtiff handle.
    Status Decode(uint64_t ifdOffset, uint32_t index, std::span<const uint8_t> raw, std::span<uint8_t> out) const;

    /// Decodes block (x, y) (pixel origin of the tile or strip, blockWidth x blockHeight, `rows`
    /// of which are in the image) of the directory at `ifdOffset` with libtiff's RGBA interface
    /// (TIFFReadRGBATile / TIFFReadRGBAStrip), as GDAL's GTiffRGBABand does: `bands` 8-bit
    /// samples per pixel (3: R, G, B; 4: and alpha), top row first, into rows of `outStride`.
    Status DecodeRgba(uint64_t ifdOffset, bool tiled, uint32_t x, uint32_t y, uint32_t blockWidth, uint32_t blockHeight,
                      uint32_t rows, uint32_t bands, uint8_t* out, size_t outStride) const;

private:
    Status SelectDirectory(uint64_t ifdOffset) const;
    struct Handle;
    std::unique_ptr<Handle> handle_;
};

}  // namespace ddbt::cog::tiff
