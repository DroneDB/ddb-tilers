// SPDX-License-Identifier: MPL-2.0
//
// The decoded-block cache of the tiler: one process-wide ShardedLru (core/cache) keyed by
// (source id, IFD, block index), byte budget (BlockCacheBudgetFor, DDBT_BLOCK_CACHE_MB), single-flight
// loads (concurrent misses of one block decode it once). Masks (1-bit IFDs) are cached expanded
// to bytes 0 / 255 with a summary (all 0, all 255, mixed), so a tile can tell from the summaries
// alone that it is blank or opaque without decoding imagery.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "cog/tiff/tiff_reader.h"
#include "core/cache/cache_stats.h"

namespace ddbt::cog::tiler {

class CogSource;

enum class MaskSummary : uint8_t { kMixed, kAllZero, kAllOne };

struct CachedBlock {
    tiff::DecodedBlock block;  // for masks: one byte per pixel, 0 or 255
    MaskSummary summary = MaskSummary::kMixed;
    /// Sample bytes of one pixel in the buffer.
    size_t pixelBytes = 1;
};

using BlockPtr = std::shared_ptr<const CachedBlock>;

/// Block `index` of `ifd` of `source`, decoded (from the cache or now). Throws TileError when it
/// cannot be read or decoded.
BlockPtr GetBlock(const CogSource& source, uint32_t ifd, uint32_t index);

/// A decoded block of complex samples as the tiler reads it: the magnitude sqrt(re^2 + im^2) of
/// each sample, Float64 (the measure of GDAL's statistics of complex bands).
tiff::DecodedBlock ComplexMagnitudes(const tiff::DecodedBlock& decoded);

/// The default budget of the block cache for a host with `physicalBytes` of memory: 3 % of it,
/// between 64 MiB and 1 GiB (1 GiB when unknown), so a large server keeps the warm working set of
/// big rasters and a small instance does not give a third of its memory to decoded blocks.
/// DDBT_BLOCK_CACHE_MB (MiB, 0 = off) replaces it.
size_t BlockCacheBudgetFor(uint64_t physicalBytes) noexcept;

/// Whether the block is in the cache (tests).
bool IsBlockCached(const CogSource& source, uint32_t ifd, uint32_t index);

/// The block cache's statistics (hits, misses, loads, bytes...).
cache::CacheStats BlockCacheStats();
/// Changes the budget (bytes); 0 disables caching (blocks are decoded per use).
void SetBlockCacheBudget(size_t bytes);
size_t BlockCacheBudget();
/// Drops every cached block (tests, benchmarks).
void ClearBlockCache();

}  // namespace ddbt::cog::tiler
