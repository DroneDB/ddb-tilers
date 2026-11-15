// SPDX-License-Identifier: MPL-2.0
#include "cog/warp/region_source.h"

#include <list>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <vector>

#include "cog/build/derived_source.h"
#include "cog/tiler/block_cache.h"
#include "cog/tiler/cog_source.h"
#include "core/exec/thread_pool.h"

namespace ddbt::cog::warp {

namespace {

// A level of a version of a file
struct LevelKey {
    std::string path;
    tiler::FileIdentity identity;
    int level = 0;
    bool operator==(const LevelKey&) const = default;
};

// The shared levels: the last ones used (a parsed TIFF holds its IFDs, not its pixels)
class Levels {
public:
    std::shared_ptr<const build::TiffSource> Find(const LevelKey& key) {
        std::lock_guard lock(mutex_);
        for (auto it = entries_.begin(); it != entries_.end(); ++it) {
            if (it->first == key) {
                entries_.splice(entries_.begin(), entries_, it);
                return it->second;
            }
        }
        return nullptr;
    }
    void Put(const LevelKey& key, std::shared_ptr<const build::TiffSource> source) {
        std::lock_guard lock(mutex_);
        entries_.emplace_front(key, std::move(source));
        if (entries_.size() > kMax) entries_.pop_back();
    }
    void Clear() {
        std::lock_guard lock(mutex_);
        entries_.clear();
    }

private:
    static constexpr size_t kMax = 32;
    std::mutex mutex_;
    std::list<std::pair<LevelKey, std::shared_ptr<const build::TiffSource>>> entries_;
};

Levels& TheLevels() {
    static Levels levels;
    return levels;
}

// A source whose blocks come from the tiler's block cache (decoded once, shared with the tiles and
// the reads of the raster, under one budget), read in place by the warp producer, which holds each
// one while it copies it. The blocks the tiler keeps as the producer reads them: those of a
// pixel-interleaved (or one-sample) image of whole samples, and the masks (bytes 0 / 255 both);
// the producer decodes the others itself.
class CachedBlockSource final : public build::ForwardingSource {
public:
    CachedBlockSource(std::shared_ptr<const build::TiffSource> source, std::shared_ptr<const tiler::CogSource> cog)
        : ForwardingSource(source), source_(std::move(source)), cog_(std::move(cog)) {
        const tiff::IfdInfo& m = source_->LevelIfd();
        dataCached_ = (m.IsPixelInterleaved() || m.samplesPerPixel == 1) &&
                      m.sampleType != tiff::SampleType::kBit1 && !m.complex;
    }

    SharedBlock SharedDataBlock(uint32_t col, uint32_t row) const override {
        if (!dataCached_) return {};
        const tiff::IfdInfo& m = source_->LevelIfd();
        return Get(source_->LevelIfdIndex(), m.BlockIndex(col, row, 0), source_->PixelBytes());
    }
    SharedBlock SharedMaskBlock(uint32_t col, uint32_t row) const override {
        const std::optional<uint32_t> ifd = source_->MaskIfdIndex();
        if (!ifd) return {};
        // The mask's blocks are those of the image (TiffSource::ReadMaskBlock refuses the others)
        const tiff::IfdInfo& m = source_->LevelIfd();
        const tiff::IfdInfo& k = cog_->Reader().Ifd(*ifd);
        if (k.blockWidth != m.blockWidth || k.blockHeight != m.blockHeight) return {};
        return Get(*ifd, k.BlockIndex(col, row, 0), 1);
    }

private:
    SharedBlock Get(uint32_t ifd, uint32_t index, size_t pixelBytes) const {
        tiler::BlockPtr block;
        try {
            // A thread waiting for a block another one decodes helps with queued tasks, and the
            // producer keeps per-thread scratch through a tile: its wait runs no task but its own,
            // or another tile would start on that thread in the middle of this one
            const exec::IsolationScope isolated(this);
            block = tiler::GetBlock(*cog_, ifd, index);
        } catch (const std::exception&) {
            return {};  // the producer decodes it itself and reports why it cannot
        }
        // In place only when its rows are those ReadBlock writes
        const tiff::IfdInfo& m = source_->LevelIfd();
        if (block->pixelBytes != pixelBytes || block->block.rowStride != m.blockWidth * pixelBytes) return {};
        const uint8_t* data = block->block.data();
        return SharedBlock{std::move(block), data};
    }

    std::shared_ptr<const build::TiffSource> source_;
    std::shared_ptr<const tiler::CogSource> cog_;
    bool dataCached_ = false;
};

}  // namespace

Expected<std::shared_ptr<const build::TiffSource>> SharedTiffLevel(const std::string& path, int level) {
    const std::optional<tiler::FileIdentity> identity = tiler::IdentifyFile(path);
    if (identity) {
        const LevelKey key{path, *identity, level};
        if (auto found = TheLevels().Find(key)) return found;
        auto opened = level == 0 ? build::TiffSource::Open(path) : build::TiffSource::OpenLevel(path, level);
        if (!opened.ok()) return opened.status();
        std::shared_ptr<const build::TiffSource> shared = std::move(*opened);
        TheLevels().Put(key, shared);
        return shared;
    }
    // Not a regular file: opened for this warp only (the open says why it cannot)
    auto opened = level == 0 ? build::TiffSource::Open(path) : build::TiffSource::OpenLevel(path, level);
    if (!opened.ok()) return opened.status();
    return std::shared_ptr<const build::TiffSource>(std::move(*opened));
}

std::unique_ptr<build::RasterSource> WithCachedBlocks(std::shared_ptr<const build::TiffSource> source,
                                                      const std::string& path, int level) {
    (void)level;  // the tiler keys the blocks by the IFD of the level
    std::shared_ptr<const tiler::CogSource> cog;
    try {
        cog = tiler::CogSource::Get(path, tiler::SourceUse::kRead);
    } catch (const std::exception&) {
    }
    if (!cog) return std::make_unique<build::ForwardingSource>(std::move(source));
    return std::make_unique<CachedBlockSource>(std::move(source), std::move(cog));
}

void ClearRegionSources() { TheLevels().Clear(); }

}  // namespace ddbt::cog::warp
