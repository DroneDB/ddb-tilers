// SPDX-License-Identifier: MPL-2.0
#include "cog/build/source_stats.h"

#include <algorithm>
#include <cstring>
#include <exception>
#include <string>
#include <utility>

#include "cog/build/pipeline.h"
#include "cog/tiler/block_cache.h"
#include "core/exec/parallel_for.h"

namespace ddbt::cog::build {

namespace {

// The type the statistics kernels read the samples of a band in (block_cache.h AsStatisticsSamples,
// ComplexMagnitudes)
tiff::SampleType StatisticsType(const RasterSource& in) {
    if (in.Complex()) return tiff::SampleType::kF64;
    switch (in.Type()) {
        case tiff::SampleType::kF16: return tiff::SampleType::kF32;
        case tiff::SampleType::kI64:
        case tiff::SampleType::kU64: return tiff::SampleType::kF64;
        default: return in.Type();
    }
}

}  // namespace

SourceStatistics::SourceStatistics(std::shared_ptr<const RasterSource> inner) : ForwardingSource(std::move(inner)) {
    const RasterSource& in = Inner();
    across_ = in.BlocksAcross();
    blocks_ = static_cast<size_t>(across_) * in.BlocksDown();
    const tiff::SampleType type = StatisticsType(in);
    convert_ = in.Complex() || type != in.Type();
    bands_ = in.Complex() ? in.Bands() / 2 : in.Bands();
    const std::optional<double> nodata = in.NoData();
    // GDAL's mask of a band: the per-dataset mask, else (without NoData) the alpha band
    const bool alpha = !in.HasMask() && !nodata && !in.Complex() && AlphaMaskRule(in.Bands(), in.AlphaBand(), in.Type());
    alphaBand_ = alpha ? in.AlphaBand() : -1;
    for (int b = 0; b < bands_; ++b) {
        filters_.push_back(stats::SampleFilter{type, nodata.has_value(), nodata.value_or(0)});
        masks_.push_back(in.HasMask()                  ? StatsMaskKind::kMask
                         : alpha && b != alphaBand_ ? StatsMaskKind::kAlphaBand
                                                     : StatsMaskKind::kNone);
    }
    claimed_ = std::make_unique<std::atomic<uint8_t>[]>(blocks_);
    sums_.resize(blocks_ * static_cast<size_t>(bands_));
}

bool SourceStatistics::Claim(uint32_t col, uint32_t row) const {
    std::atomic<uint8_t>& c = claimed_[BlockIndex(col, row)];
    return c.load(std::memory_order_relaxed) == 0 && c.exchange(1, std::memory_order_acq_rel) == 0;
}

Status SourceStatistics::Count(uint32_t col, uint32_t row, const uint8_t* samples, uint32_t rows) const {
    try {
        const RasterSource& in = Inner();
        const uint32_t bw = in.BlockWidth(), bh = in.BlockHeight();
        const uint32_t width = std::min(bw, in.Width() - col * bw);
        rows = std::min(rows, in.Height() - row * bh);
        // The block's mask: in place when the source keeps or shares it, else decoded here
        const uint8_t* mask = nullptr;
        SharedBlock sharedMask;
        thread_local std::vector<uint8_t> maskBlock;
        if (in.HasMask()) {
            mask = in.ResidentMaskBlock(col, row);
            if (mask == nullptr) {
                sharedMask = in.SharedMaskBlock(col, row);
                mask = sharedMask.data;
            }
            if (mask == nullptr) {
                maskBlock.resize(static_cast<size_t>(bw) * bh);
                DDBT_TRY(in.ReadMaskBlock(col, row, maskBlock.data()));
                mask = maskBlock.data();
            }
        }
        size_t stride = static_cast<size_t>(bw) * in.PixelBytes();
        tiff::DecodedBlock converted;
        if (convert_) {
            // The samples as the tiler's exact pass reads them (magnitudes, Float16 as Float32, 64-bit
            // integers as Float64)
            tiff::DecodedBlock raw;
            raw.buffer = tiff::BlockBuffer::Allocate(stride * rows, nullptr);
            std::memcpy(raw.data(), samples, stride * rows);
            raw.width = bw;
            raw.height = rows;
            raw.samples = static_cast<uint32_t>(in.Bands());
            raw.bitsPerSample = static_cast<uint32_t>(in.SampleSize() * 8);
            raw.sampleType = in.Type();
            raw.rowStride = stride;
            converted = in.Complex() ? tiler::ComplexMagnitudes(raw) : tiler::AsStatisticsSamples(std::move(raw));
            samples = converted.data();
            stride = converted.rowStride;
        }
        stats::ExactSums* sums = sums_.data() + BlockIndex(col, row) * static_cast<size_t>(bands_);
        for (int b = 0; b < bands_; ++b) {
            sums[b] = stats::ExactSums{};
            AddBandRows(sums[b], filters_[static_cast<size_t>(b)], masks_[static_cast<size_t>(b)], bands_, b, alphaBand_,
                        samples, stride, width, rows, mask, bw);
        }
    } catch (const std::exception& e) {
        return Status(StatusCode::kInternal, e.what());
    }
    return Status::Ok();
}

void SourceStatistics::CountClaimed(uint32_t col, uint32_t row, const uint8_t* samples, uint32_t rows) const {
    if (Count(col, row, samples, rows).ok()) {
        countedByReads_.fetch_add(1, std::memory_order_relaxed);
    } else {
        // Finish() reads it again and reports the error
        claimed_[BlockIndex(col, row)].store(0, std::memory_order_release);
    }
}

Expected<uint32_t> SourceStatistics::ReadBlock(uint32_t col, uint32_t row, uint8_t* out) const {
    auto rows = Inner().ReadBlock(col, row, out);
    if (rows.ok() && Claim(col, row)) CountClaimed(col, row, out, *rows);
    return rows;
}

const uint8_t* SourceStatistics::ResidentBlock(uint32_t col, uint32_t row) const {
    const uint8_t* data = Inner().ResidentBlock(col, row);
    if (data != nullptr && Claim(col, row))
        CountClaimed(col, row, data, std::min(BlockHeight(), Height() - row * BlockHeight()));
    return data;
}

RasterSource::SharedBlock SourceStatistics::SharedDataBlock(uint32_t col, uint32_t row) const {
    SharedBlock shared = Inner().SharedDataBlock(col, row);
    if (shared.data != nullptr && Claim(col, row))
        CountClaimed(col, row, shared.data, std::min(BlockHeight(), Height() - row * BlockHeight()));
    return shared;
}

Expected<std::vector<std::optional<BandStats>>> SourceStatistics::Finish() const {
    std::vector<size_t> missing;
    for (size_t b = 0; b < blocks_; ++b)
        if (claimed_[b].load(std::memory_order_acquire) == 0) missing.push_back(b);
    std::vector<Status> errors(missing.size());
    exec::ParallelFor(exec::DefaultPool(), 0, missing.size(), 1, [&](size_t k) {
        const uint32_t col = static_cast<uint32_t>(missing[k] % across_);
        const uint32_t row = static_cast<uint32_t>(missing[k] / across_);
        std::vector<uint8_t> block(BlockBytes());
        auto rows = Inner().ReadBlock(col, row, block.data());
        errors[k] = rows.ok() ? Count(col, row, block.data(), *rows) : rows.status();
        if (errors[k].ok()) claimed_[missing[k]].store(1, std::memory_order_relaxed);
    });
    for (const Status& e : errors)
        if (!e.ok()) return Status(e.code(), "Cannot compute the statistics of the source: " + std::string(e.message()));
    countedAfter_ += missing.size();
    std::vector<stats::ExactSums> total(static_cast<size_t>(bands_));
    for (size_t b = 0; b < blocks_; ++b)
        for (size_t i = 0; i < total.size(); ++i) stats::Merge(total[i], sums_[b * total.size() + i]);
    const uint64_t pixels = static_cast<uint64_t>(Width()) * Height();
    std::vector<std::optional<BandStats>> out(total.size());
    for (size_t i = 0; i < total.size(); ++i) out[i] = BandStatsOf(stats::Finish(total[i]), pixels);
    return out;
}

}  // namespace ddbt::cog::build
