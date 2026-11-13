// SPDX-License-Identifier: Apache-2.0
// ddbt fork (new file): layer-selective, reusable decoding of one LAZ chunk of point format
// 6, 7 or 8 (a COPC node), into LAS records or straight into arrays (structure of arrays).
//
//     lazperf::layered_decompressor d(7, 0, lazperf::layer::xy | lazperf::layer::z | lazperf::layer::rgb);
//     d.begin(chunk, chunkBytes);           // borrowed until the next begin()
//     d.decompress(sink, pointCount);       // x/y/z int32, r/g/b uint16
//     d.begin(otherChunk, otherBytes);      // same models, reset, no allocation
//
// Every layer outside the mask is skipped without reading its bytes. Decoding the XY, Z and
// RGB / NIR layers alone gives the same values as a full decode (layers.hpp).
// Errors (truncated chunk, unsupported format) are thrown as lazperf::error.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "lazperf_base.hpp"
#include "layers.hpp"

namespace lazperf
{

// Destination arrays of decompress(sink, n): element i of each non-null array receives the
// value of point i. Null arrays are not written (their layers may still be decoded).
struct soa_sink
{
    int32_t *x = nullptr;
    int32_t *y = nullptr;
    int32_t *z = nullptr;
    uint16_t *r = nullptr;
    uint16_t *g = nullptr;
    uint16_t *b = nullptr;
    uint16_t *nir = nullptr;
};

class LAZPERF_EXPORT layered_decompressor
{
public:
    // format: 6 to 10 (ddbt fork: 9 / 10 are 6 / 8 with a wave-packet layer, never decoded:
    // records get zeros there). ebCount: extra bytes per point. layers: layer::* bits to decode (xy
    // is always decoded; bits of layers the format lacks are ignored).
    layered_decompressor(int format, int ebCount, uint32_t layers = layer::all);
    ~layered_decompressor();
    layered_decompressor(const layered_decompressor&) = delete;
    layered_decompressor& operator=(const layered_decompressor&) = delete;

    int format() const;
    int ebCount() const;
    // The effective mask (xy added, bits of absent layers removed).
    uint32_t layers() const;
    // Bytes of one LAS record of this format (30, 36 or 38 + ebCount).
    size_t pointSize() const;

    // Starts a chunk: `size` bytes at `data`, borrowed until the next begin(). Resets the
    // models of the previous chunk without reallocating them.
    void begin(const char *data, size_t size);

    // Decodes the next point into a LAS record of pointSize() bytes. Fields of skipped layers
    // hold stale values. Returns out + pointSize().
    char *decompress(char *out);

    // Decodes the next `count` points into the arrays of `sink`, from index 0.
    void decompress(const soa_sink& sink, size_t count);

    // Points decoded since begin().
    uint64_t decoded() const;
    // The point count stored in the chunk (known once the first point is decoded, else 0).
    uint32_t chunkPointCount() const;
    // Bytes of the chunk consumed so far (the layers are consumed when the first point is
    // decoded, skipped ones included).
    size_t consumed() const;

private:
    struct Private;
    std::unique_ptr<Private> p_;
};

} // namespace lazperf
