// SPDX-License-Identifier: MPL-2.0
#include "cog/tiler/block_cache.h"

#include <cstdlib>
#include <algorithm>
#include <cstring>
#include <string>

#include "cog/tiler/cog_source.h"
#include "cog/tiler/tile_error.h"
#include "core/cache/sharded_lru.h"
#include "core/mem/system_memory.h"

namespace ddbt::cog::tiler {

namespace {

struct BlockKey {
    uint64_t source = 0;
    uint32_t ifd = 0;
    uint32_t index = 0;
    bool operator==(const BlockKey&) const = default;
};

struct BlockKeyHash {
    size_t operator()(const BlockKey& k) const noexcept {
        uint64_t x = k.source * 0x9E3779B97F4A7C15ull ^ (static_cast<uint64_t>(k.ifd) << 32 | k.index);
        x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
        x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
        return static_cast<size_t>(x ^ (x >> 31));
    }
};

struct BlockCost {
    size_t operator()(const CachedBlock& b) const noexcept { return sizeof(CachedBlock) + b.block.size(); }
};

using Cache = cache::ShardedLru<BlockKey, CachedBlock, BlockKeyHash, BlockCost>;

size_t DefaultBudget() {
    if (const char* e = std::getenv("DDBT_BLOCK_CACHE_MB")) {
        const long long mb = std::atoll(e);
        if (mb >= 0) return static_cast<size_t>(mb) << 20;
    }
    return BlockCacheBudgetFor(mem::PhysicalMemoryBytes());
}

Cache& TheCache() {
    static Cache cache(DefaultBudget(), 64);
    return cache;
}

CachedBlock Decode(const CogSource& source, uint32_t ifd, uint32_t index) {
    const tiff::TiffReader& r = source.Reader();
    auto decoded = r.ReadBlock(ifd, index);
    if (!decoded.ok())
        ThrowIfError(decoded.status(), "Cannot read block " + std::to_string(index) + " of " + source.Path());
    CachedBlock cb;
    const tiff::IfdInfo& info = r.Ifd(ifd);
    if (decoded->sampleType == tiff::SampleType::kBit1) {
        // A mask: bytes 0 / 255 (GDAL reads a 1-bit mask band as 0 / 255).
        const uint32_t w = decoded->width, h = decoded->height;
        tiff::DecodedBlock bytes;
        bytes.buffer = tiff::BlockBuffer::Allocate(static_cast<size_t>(w) * h, nullptr);
        bytes.width = w;
        bytes.height = h;
        bytes.samples = 1;
        bytes.bitsPerSample = 8;
        bytes.sampleType = tiff::SampleType::kU8;
        bytes.rowStride = w;
        bytes.sparse = decoded->sparse;
        if (decoded->sparse) {
            std::memset(bytes.data(), 0, bytes.size());
        } else {
            tiff::ExpandBits(decoded->data(), decoded->rowStride, w, h, 255, bytes.data(), w);
        }
        // Summary over the valid part (the raster may end inside the block), from the packed bits.
        const uint32_t col = info.BlockColumn(index), row = info.BlockRow(index);
        const uint32_t vw = std::min(w, info.width - col * info.blockWidth);
        const uint32_t vh = std::min(h, info.height > row * info.blockHeight ? info.height - row * info.blockHeight : 0u);
        bool any0 = false, any1 = false;
        if (decoded->sparse) {
            any0 = vw != 0 && vh != 0;
        } else {
            const tiff::BitsSummary s = tiff::SummarizeBits(decoded->data(), decoded->rowStride, vw, vh);
            any0 = s.any0;
            any1 = s.any1;
        }
        cb.summary = any0 && any1 ? MaskSummary::kMixed : (any1 ? MaskSummary::kAllOne : MaskSummary::kAllZero);
        cb.block = std::move(bytes);
        cb.pixelBytes = 1;
        return cb;
    }
    cb.block = std::move(*decoded);
    cb.pixelBytes = static_cast<size_t>(cb.block.samples) * tiff::SampleTypeSize(cb.block.sampleType);
    return cb;
}

}  // namespace

size_t BlockCacheBudgetFor(uint64_t physicalBytes) noexcept {
    constexpr uint64_t kMax = uint64_t{1} << 30, kMin = uint64_t{64} << 20;
    if (physicalBytes == 0) return static_cast<size_t>(kMax);
    return static_cast<size_t>(std::clamp(physicalBytes * 3 / 100, kMin, kMax));
}

BlockPtr GetBlock(const CogSource& source, uint32_t ifd, uint32_t index) {
    Cache& c = TheCache();
    if (c.BudgetBytes() == 0) return std::make_shared<const CachedBlock>(Decode(source, ifd, index));
    return c.GetOrLoad(BlockKey{source.Id(), ifd, index}, [&] { return Decode(source, ifd, index); });
}

bool IsBlockCached(const CogSource& source, uint32_t ifd, uint32_t index) {
    return TheCache().Contains(BlockKey{source.Id(), ifd, index});
}

cache::CacheStats BlockCacheStats() { return TheCache().Stats(); }
void SetBlockCacheBudget(size_t bytes) { TheCache().SetBudget(bytes); }
size_t BlockCacheBudget() { return TheCache().BudgetBytes(); }
void ClearBlockCache() { TheCache().Clear(); }

}  // namespace ddbt::cog::tiler
