// SPDX-License-Identifier: MPL-2.0
//
// A JPEG 2000 codestream decoded with OpenJPEG as GDAL's JP2OpenJPEG driver decodes it
// (frmts/openjpeg/opjdatasetbase.h): always as a raw codestream (the JP2 boxes are read
// separately, so OpenJPEG applies no palette, channel definition or colour conversion), in
// strict mode, either an area of the image (opj_set_decode_area + opj_decode) or one tile
// (opj_get_decoded_tile), at full resolution. Samples come out as 32-bit integers per component.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "core/io/byte_source.h"
#include "core/util/status.h"

namespace ddbt::cog::jp2 {

struct ComponentInfo {
    uint32_t dx = 1, dy = 1;  // subsampling
    uint32_t w = 0, h = 0;    // size in samples
    uint32_t prec = 0;        // bits
    bool sgnd = false;
};

/// The main header of a codestream (SIZ and the default coding style).
struct CodestreamHeader {
    uint32_t x0 = 0, y0 = 0, x1 = 0, y1 = 0;  // image area on the reference grid
    uint32_t tileWidth = 0, tileHeight = 0;
    uint32_t tileX0 = 0, tileY0 = 0, tilesAcross = 0, tilesDown = 0;
    int resolutions = 0;
    std::vector<ComponentInfo> components;
    uint32_t Width() const noexcept { return x1 - x0; }
    uint32_t Height() const noexcept { return y1 - y0; }
};

/// The samples of one component of the last decode: rows of `w` 32-bit integers.
struct DecodedComponent {
    const int32_t* data = nullptr;
    uint32_t w = 0, h = 0;
};

class Codestream {
public:
    /// Reads the main header of the codestream at [offset, offset + length) of `src`. `threads`:
    /// OpenJPEG's decoding threads for this codestream (1: the calling thread only).
    static Expected<std::unique_ptr<Codestream>> Open(std::shared_ptr<const io::ByteSource> src, uint64_t offset,
                                                      uint64_t length, int threads);
    ~Codestream();
    Codestream(const Codestream&) = delete;
    Codestream& operator=(const Codestream&) = delete;

    const CodestreamHeader& Header() const noexcept { return header_; }

    /// Decodes [x0, x1) x [y0, y1) of the reference grid. A codestream can decode several areas
    /// in turn (OpenJPEG keeps the compressed data of a single-tile image between them).
    Status DecodeArea(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1);
    /// Decodes tile `index` (row-major in the tile grid).
    Status DecodeTile(uint32_t index);

    /// The samples of component c of the last decode.
    DecodedComponent Component(size_t c) const noexcept;

    struct Impl;

private:
    explicit Codestream(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
    CodestreamHeader header_;
};

}  // namespace ddbt::cog::jp2
