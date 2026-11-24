// SPDX-License-Identifier: MPL-2.0
//
// Exact band statistics of the COG being built, accumulated while it is written, with the
// semantics and arithmetic of DroneDB 2.4.1's RasterStatsProvider on the finished file
// (raster_stats.cpp; the same rules as the tiler's stats.cpp): every pixel of the full
// resolution, rows top to bottom each across the whole width, Welford's running mean and M2 in
// that order (same bits), skipping non-finite samples, the band's nodata value (Float32 bands
// compare the value rounded to float), and pixels whose mask is 0 when the band's mask is the
// internal mask or the alpha band (2-band / 4-band 8 / 16-bit rasters). The values are those
// a reader decodes from the file: for JPEG the decoded tiles, not the pixels before encoding.
// The kernels are the tiler's (cog/tiler/stats_kernel.h); DDBT_STATS_MODE: exact sums by
// default (the tiler's exact mode: the same values for integer bands), compat for Welford's bits.
//
// The sidecar "<cog>.stats.json" (version 2) is written as DroneDB writes it.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "cog/tiff/tiff_reader.h"
#include "cog/tiler/stats_kernel.h"
#include "core/util/status.h"

namespace ddbt::cog::build {

struct BandStats {
    double min = 0, max = 0, mean = 0, stdDev = 0;
    uint64_t valid = 0, total = 0;
};

/// What masks the samples of a band: nothing, a mask (0 = no value), or the alpha band of the pixel.
enum class StatsMaskKind : uint8_t { kNone, kMask, kAlphaBand };

/// Accumulates `rows` rows of `width` pixels of band `band` (0-based) into `acc`: pixels interleaved
/// (`bands` samples of f.type each), rows `stride` bytes apart; masked by `mask` (rows `maskStride`
/// bytes apart) for kMask, by the sample of band `alphaBand` for kAlphaBand. One AddRun per row.
template <typename Acc>
void AddBandRows(Acc& acc, const stats::SampleFilter& f, StatsMaskKind kind, int bands, int band, int alphaBand,
                 const uint8_t* pixels, size_t stride, uint32_t width, uint32_t rows, const uint8_t* mask,
                 size_t maskStride) {
    const size_t ss = tiff::SampleTypeSize(f.type);
    const size_t step = ss * static_cast<size_t>(bands);
    for (uint32_t y = 0; y < rows; ++y) {
        const uint8_t* row = pixels + static_cast<size_t>(y) * stride;
        const uint8_t* m =
            kind == StatsMaskKind::kMask && mask != nullptr ? mask + static_cast<size_t>(y) * maskStride : nullptr;
        const uint8_t* al = kind == StatsMaskKind::kAlphaBand ? row + static_cast<size_t>(alphaBand) * ss : nullptr;
        stats::AddRun(acc, f, row + static_cast<size_t>(band) * ss, step, width, m, al);
    }
}

/// The statistics of a band from its exact sums or Welford's accumulation (nullopt: no sample has a
/// value); `total` = width * height.
std::optional<BandStats> BandStatsOf(const std::optional<stats::StatsResult>& r, uint64_t total);

class StatsAccumulator {
public:
    using MaskKind = StatsMaskKind;

    /// `masks[b]`: what masks band b; alphaBand: index of the alpha band for kAlphaBand.
    StatsAccumulator(int bands, tiff::SampleType type, std::vector<MaskKind> masks, int alphaBand,
                     std::optional<double> nodata);

    /// Accumulates `rows` full-width rows of band b: pixels interleaved (`bands` samples per
    /// pixel), rows `stride` bytes apart; `mask` (rows of `width` bytes, 0 = invalid) for kMask.
    /// Calls for one band must come in row order; different bands may run concurrently.
    void AccumulateRows(int band, const uint8_t* pixels, size_t stride, uint32_t width, uint32_t rows,
                        const uint8_t* mask);

    /// Per band: nullopt when no pixel has a value. `total` = width * height.
    std::vector<std::optional<BandStats>> Result(uint64_t total) const;

private:
    struct Acc {
        stats::Welford welford;
        stats::ExactSums exact;
    };
    int bands_;
    tiff::SampleType type_;
    std::vector<MaskKind> masks_;
    int alphaBand_;
    std::optional<double> nodata_;
    stats::StatsMode mode_;
    std::vector<Acc> acc_;
};

/// Writes the statistics of `bands` to `path` (DroneDB's schema: bands."<b>" = {max, mean, min, std,
/// total, valid}, a band without a value left out, computedAt, exact: true, version: 2), atomically
/// (temporary + rename).
Status WriteStatsFile(const std::string& path, const std::vector<std::optional<BandStats>>& bands);

/// WriteStatsFile to "<rasterPath>.stats.json".
Status WriteStatsSidecar(const std::string& rasterPath, const std::vector<std::optional<BandStats>>& bands);

}  // namespace ddbt::cog::build
