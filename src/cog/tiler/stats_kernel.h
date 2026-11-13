// SPDX-License-Identifier: MPL-2.0
//
// The band statistics kernels shared by the tiler's exact pass (tiler/stats.cpp) and the COG
// builder (build/stats.cpp): which samples have a value, and two ways to accumulate them.
//
// - A sample has a value when it is finite, is not the NoData value (a Float32 band compares the
//   value rounded to float: GDAL's rule) and its pixel is not masked: mask byte 0, or alpha 0 (a
//   16-bit alpha is rescaled as GDAL's mask of an alpha band does: alpha / 257).
// - Welford: DroneDB 2.4.1's RasterStatsProvider arithmetic (running mean and M2, min / max with
//   std::min / std::max). Fed the samples in DroneDB's order (rows top to bottom, each across the
//   whole width) it gives DroneDB's bits; it is serial: one dependent division per sample.
// - ExactSums: count, min, max and the sums of the samples and of their squares, kept as
//   double-double values. For integer samples the sums are exact (integers below 2^106), so the
//   result does not depend on the order, the chunking or the thread count, and the mean is the
//   correctly rounded quotient. For floating-point samples the sums carry ~2^-104 relative error
//   and the result depends only on the (fixed) order of runs and merges. Runs of one chunk can be
//   accumulated on any thread and chunks merged in a fixed order (Merge).
//
// The two differ: Welford's mean carries its own rounding errors (~sqrt(n) ulps), so ExactSums is
// not bit-identical to DroneDB's statistics but closer to the exact values (DDBT_STATS_MODE:
// exact by default, compat for DroneDB's bits).
#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

#include "cog/tiff/tiff_reader.h"

namespace ddbt::cog::stats {

/// DDBT_STATS_MODE: "exact" (default: ExactSums over chunks accumulated in parallel, merged in a
/// fixed order) or "compat" (opt-in: Welford in DroneDB's order, DroneDB's bits; the pass decodes
/// in parallel and runs one serial accumulation per band). Any other value is "exact".
enum class StatsMode : uint8_t { kCompat, kExact };
StatsMode StatsModeFromEnv();

/// Which samples have a value, besides the mask / alpha of a run.
struct SampleFilter {
    tiff::SampleType type = tiff::SampleType::kU8;
    bool hasNodata = false;
    double nodata = 0;
};

struct StatsResult {
    double min = 0, max = 0, mean = 0, stdDev = 0;
    uint64_t valid = 0;
};

struct Welford {
    double mean = 0, m2 = 0;
    double lo = std::numeric_limits<double>::infinity(), hi = -std::numeric_limits<double>::infinity();
    uint64_t n = 0;
};

struct DoubleDouble {
    double hi = 0, lo = 0;
};

struct ExactSums {
    uint64_t n = 0;
    double lo = std::numeric_limits<double>::infinity(), hi = -std::numeric_limits<double>::infinity();
    DoubleDouble sum, sumSq;
};

/// Accumulates `count` samples `step` bytes apart from `p` (type f.type). `mask`: `count` bytes,
/// 0 = no value (or null); `alpha`: the alpha sample of each pixel, same type and step (or null).
void AddRun(Welford& a, const SampleFilter& f, const uint8_t* p, size_t step, uint32_t count, const uint8_t* mask,
            const uint8_t* alpha);
void AddRun(ExactSums& a, const SampleFilter& f, const uint8_t* p, size_t step, uint32_t count, const uint8_t* mask,
            const uint8_t* alpha);

/// into += from (merge chunks in a fixed order for floating-point samples).
void Merge(ExactSums& into, const ExactSums& from);

/// nullopt when no sample has a value. Welford: DroneDB's finish (std = sqrt(M2 / n)).
std::optional<StatsResult> Finish(const Welford& a);
std::optional<StatsResult> Finish(const ExactSums& a);

}  // namespace ddbt::cog::stats
