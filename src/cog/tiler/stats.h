// SPDX-License-Identifier: MPL-2.0
//
// Band statistics for the stretch of non-Byte tiles: DroneDB 2.4.1's RasterStatsProvider
// (raster_stats.cpp) on the file of a CogSource.
//
// - Exact statistics only: one pass over every pixel of the full-resolution image, the pixels
//   without a value left out (the band's NoData value, or the override, compared in the precision
//   of the sample: a Float32 band against the value rounded to float; non-finite samples; a zero
//   in the mask of the band when its mask is per dataset or alpha, not nodata).
// - The arithmetic (stats_kernel.h), by DDBT_STATS_MODE:
//   compat (opt-in): Welford's accumulation in DroneDB's order (rows top to bottom, each across
//   the whole width), so mean and standard deviation are DroneDB's bits. Chunks of block rows are
//   decoded in parallel (the next one while the current one is accumulated) and the bands
//   accumulate in parallel, but each band's accumulation is serial (one dependent division per
//   sample: ~20 cycles).
//   exact (default): every block is a chunk accumulated on any thread into exact sums, merged in block
//   order: the same result whatever the thread count; integer bands give the correctly rounded
//   mean, which differs from Welford's in the last bits (Welford's own rounding errors).
// - Order of sources per (file version, band, effective NoData): memory (an LRU of kMaxStatsFiles
//   files by canonical path and file identity, as DroneDB's MaxEntries, each with an LRU of
//   kMaxStatsNodataKeysPerFile NoData keys), then the
//   "<raster>.stats.json" sidecar when it is fresh (not older than the raster), parses, says
//   "exact": true and has the entry ("bands"."<b>", or "overrides"."<%.17g nodata>"."<b>"), then
//   the computation. Concurrent requests for a missing key wait for one computation.
// - Writing the sidecar: DroneDB writes one next to a raster it considers its own artefact (a
//   ".ddb/build/<key>/" build output, its cache's artefacts). The tiler never writes next to its
//   inputs unless DDBT_STATS_SIDECAR=write is set (and then only where DroneDB would): tile
//   serving stays read-only. Writes go through a temporary of the process (pid + random suffix)
//   renamed over the sidecar; concurrent writers are not locked, so one may drop another's new
//   entry, which is then computed again (and written) by the next request that misses it.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "cog/tiler/stats_kernel.h"
#include "cog/tiler/tile_error.h"

namespace ddbt::cog::tiler {

class CogSource;

struct BandStatistics {
    double min = 0, max = 0, mean = 0, stdDev = 0;
    uint64_t validCount = 0, pixelCount = 0;
};

/// No pixel of the band has a value (DroneDB NoValidPixelsException, a GDALException).
class NoValidPixelsError : public TileError {
public:
    explicit NoValidPixelsError(std::string m) : TileError(TileErrorCode::kException, std::move(m)) {}
    std::function<void()> MakeThrower() const override {
        return [m = std::string(what())] { throw NoValidPixelsError(m); };
    }
};

/// Exact statistics of band `band` (1-based) of the source's file, with the NoData override.
/// Throws NoValidPixelsError, or TileError when the raster cannot be read.
BandStatistics GetBandStatistics(const CogSource& source, int band, const std::optional<double>& nodata);

/// The exact pass alone, without memory or sidecar (tests): one statistic per band of `bands`
/// (1-based), nullopt where no pixel has a value.
std::vector<std::optional<BandStatistics>> ComputeBandStatistics(const CogSource& source, const std::vector<int>& bands,
                                                                 const std::optional<double>& nodata);
/// The same with an explicit arithmetic (the other one reads DDBT_STATS_MODE).
std::vector<std::optional<BandStatistics>> ComputeBandStatistics(const CogSource& source, const std::vector<int>& bands,
                                                                 const std::optional<double>& nodata,
                                                                 stats::StatsMode mode);

/// The sidecar part alone (tests): the entry of (band, nodata) of the sidecar next to `rasterPath`.
std::optional<BandStatistics> ReadStatsSidecar(const std::string& rasterPath, int band,
                                               const std::optional<double>& nodata);

/// DroneDB's nodataKey: "" (no override), "nan", or "%.17g".
std::string NodataKey(const std::optional<double>& nodata);

/// raster_artefacts.cpp isLibddbArtefact, for the paths the tiler sees: a ".ddb/build/<64 hex
/// key>/cog/cog.tif" build output.
bool IsLibddbBuildArtefact(const std::string& path);

/// Counters: statistics read from sidecars, computed.
struct StatsCounters {
    uint64_t fromSidecar = 0, computed = 0;
};
StatsCounters GetStatsCounters();
/// Forgets every cached statistic (tests).
void ClearStatsCache();

/// Files whose statistics are kept (DroneDB raster_stats.cpp MaxEntries).
inline constexpr size_t kMaxStatsFiles = 256;
/// NoData keys (the band's own NoData and the overrides) kept per file.
inline constexpr size_t kMaxStatsNodataKeysPerFile = 16;
/// Passes kept in memory and the files they belong to (tests).
size_t StatsCacheEntryCount();
size_t StatsCacheFileCount();

}  // namespace ddbt::cog::tiler
