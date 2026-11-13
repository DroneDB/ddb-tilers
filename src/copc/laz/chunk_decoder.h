// SPDX-License-Identifier: MPL-2.0
//
// LAZ chunk decoding and encoding on top of the laz-perf fork (third_party/lazperf), with
// Status errors and no laz-perf types in the interface.
//
//   ChunkDecoder (formats 6-8): one reusable, layer-selective decoder of one chunk at a time
//     into arrays (int32 x/y/z, uint16 r/g/b/nir) or LAS records. Not thread-safe; use
//     ThreadDecoder() for a per-thread instance keyed by (format, extra bytes, layers).
//   DecodeChunkReference: upstream laz-perf's chunk_decompressor (every format laz-perf
//     reads, full records), the reference the layered path is tested against.
//   CompressChunk: one chunk from LAS records (laz-perf's compressors), for tests and the
//     builder.
//
// Layers that can be skipped exactly: every layer of a point 6-10 chunk has its own arithmetic
// decoder over its own bytes, and only the XY layer (changed-values symbol, scanner channel,
// return numbers) feeds the contexts of the others, so decoding XY + Z + RGB (+ NIR) gives the
// same values as a full decode (lazperf/layers.hpp). kLayerXY is always decoded.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include "core/util/status.h"

namespace ddbt::copc::laz {

/// Layer bits (equal to lazperf::layer::*).
inline constexpr uint32_t kLayerXY = 1u << 0;
inline constexpr uint32_t kLayerZ = 1u << 1;
inline constexpr uint32_t kLayerClassification = 1u << 2;
inline constexpr uint32_t kLayerFlags = 1u << 3;
inline constexpr uint32_t kLayerIntensity = 1u << 4;
inline constexpr uint32_t kLayerScanAngle = 1u << 5;
inline constexpr uint32_t kLayerUserData = 1u << 6;
inline constexpr uint32_t kLayerPointSource = 1u << 7;
inline constexpr uint32_t kLayerGpsTime = 1u << 8;
inline constexpr uint32_t kLayerRgb = 1u << 9;
inline constexpr uint32_t kLayerNir = 1u << 10;
inline constexpr uint32_t kLayerExtraBytes = 1u << 11;
inline constexpr uint32_t kLayersAll = (1u << 12) - 1;
/// What a tile needs: positions and colour.
inline constexpr uint32_t kLayersXyzRgb = kLayerXY | kLayerZ | kLayerRgb;
inline constexpr uint32_t kLayersXyz = kLayerXY | kLayerZ;

/// Destination arrays: element i receives point i. Null arrays are not written.
struct SoaOut {
    int32_t* x = nullptr;
    int32_t* y = nullptr;
    int32_t* z = nullptr;
    uint16_t* r = nullptr;
    uint16_t* g = nullptr;
    uint16_t* b = nullptr;
    uint16_t* nir = nullptr;
};

class ChunkDecoder {
public:
    /// format 6, 7 or 8; extraBytes per record; layers: kLayer* bits (XY is added, bits of
    /// layers the format lacks are dropped). Unsupported for other formats.
    static Expected<std::unique_ptr<ChunkDecoder>> Create(int format, int extraBytes, uint32_t layers);
    ~ChunkDecoder();
    ChunkDecoder(const ChunkDecoder&) = delete;
    ChunkDecoder& operator=(const ChunkDecoder&) = delete;

    int format() const noexcept { return format_; }
    int extraBytes() const noexcept { return extraBytes_; }
    uint32_t layers() const noexcept;
    /// Bytes of one LAS record (30 / 36 / 38 + extra bytes).
    size_t recordSize() const noexcept;

    /// Decodes the `pointCount` points of `chunk` into `out`. Corrupt when the chunk is
    /// truncated or stores another point count (checked after the first point).
    Status DecodeSoa(std::span<const uint8_t> chunk, uint32_t pointCount, const SoaOut& out);
    /// Same into records (recordSize() bytes each; fields of skipped layers are left as they
    /// were in `out`).
    Status DecodeRecords(std::span<const uint8_t> chunk, uint32_t pointCount, std::span<uint8_t> out);

    /// Chunks decoded by this decoder (for tests of reuse).
    uint64_t chunksDecoded() const noexcept { return chunks_; }

private:
    ChunkDecoder(int format, int extraBytes);
    struct Impl;
    std::unique_ptr<Impl> impl_;
    int format_;
    int extraBytes_;
    uint64_t chunks_ = 0;
};

/// The decoder of this thread for (format, extraBytes, layers), created on first use and
/// reused afterwards (a few per thread, least recently used dropped beyond 8).
Expected<ChunkDecoder*> ThreadDecoder(int format, int extraBytes, uint32_t layers);

/// Upstream laz-perf decode of a chunk (formats 0-8; 4 and 5 by the fork) into records of
/// `recordSize` bytes. The chunk must be complete: laz-perf's callback reader has no bound.
Status DecodeChunkReference(int format, int extraBytes, std::span<const uint8_t> chunk, uint32_t pointCount,
                            std::span<uint8_t> out);

/// One LAZ chunk holding `pointCount` records (format 0-8, `extraBytes` each).
Expected<std::vector<uint8_t>> CompressChunk(int format, int extraBytes, std::span<const uint8_t> records,
                                             uint32_t pointCount);

/// One entry of a LAZ chunk table.
struct ChunkTableEntry {
    uint64_t count = 0;  // points (written for variable-size chunks only)
    uint64_t bytes = 0;  // compressed size
};

/// The compressed body of a LAZ chunk table (what follows its version and count words).
std::vector<uint8_t> CompressChunkTable(std::span<const ChunkTableEntry> chunks, bool variable);

}  // namespace ddbt::copc::laz
