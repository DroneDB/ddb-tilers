// SPDX-License-Identifier: MPL-2.0
//
// COG tiler microbenchmarks on corpus rasters (skipped when missing):
//
//   Tile/<raster>/<format>   whole tiles (plan, gather, render, encode) of a fixed set of tiles at
//                            z..z+2 over the raster, the encoded-tile cache off, blocks warm
//   Stage/Plan               PlanRead of those windows (GDAL's overview choice + sampling)
//   Stage/Gather             PlanRead + gather of the bands (warm blocks)
//   Stage/Alpha              PlanRead + mask / alpha
//   Stretch/U16 Lut|Direct   65536 UInt16 samples to bytes, LUT vs stretchToByte per sample
//   Warp/Map                 the warped grid's source mapping of a 256 x 256 tile (R3: BD72)
//   Stats/R5                 the exact statistics pass of R5 (461 MB Float32, 521 Mpx), once
//
// Items are tiles per second. Pin with taskset (CCD1 during baseline runs elsewhere).

#include <benchmark/benchmark.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

#include "cog/tiler/block_cache.h"
#include "cog/tiler/cog_source.h"
#include "cog/tiler/cog_tiler.h"
#include "cog/tiler/raster_io.h"
#include "cog/tiler/stats.h"
#include "cog/tiler/stretch.h"
#include "cog/tiler/warp.h"
#include "core/tile/compat.h"

namespace {

using namespace ddbt::cog::tiler;
namespace compat = ddbt::tile::compat;

struct Raster {
    const char* id;
    const char* path;
};

const Raster kRasters[] = {
    {"R1", "/mnt/data/repos/test_data/ortho/brighton-beach-cog.tif"},
    {"R5", "/mnt/data/datasets/orthos/test-tif/.ddb/build/"
           "159b8f07cdc8e83b8098466a497cfe4d4e9d4d79983259f25e705b5a30d55285/cog/cog.tif"},
    {"R6", "/mnt/data/datasets/orthos/err-mask/.ddb/build/"
           "29b077eede0e6ece29a9e52f136bcc30e9f359d588947b50841996a11b2ce0ce/cog/cog.tif"},
    {"R3", "/mnt/data/repos/test_data/ortho/vo.tif"},
};

const char* PathOf(const std::string& id) {
    for (const auto& r : kRasters)
        if (id == r.id) return std::filesystem::exists(r.path) ? r.path : nullptr;
    return nullptr;
}

struct Tile {
    int z, x, y;  // y: XYZ row
};

// Up to `n` tiles of the raster's top zoom (and the two below), centred on it, rows XYZ.
std::vector<Tile> TilesOf(const CogSource& src, int n) {
    const RasterView view(src);
    compat::RasterTiling t(256, view.GeoTransform().data(), view.Width(), view.Height());
    std::vector<Tile> out;
    for (int z = t.zooms.max - 2; z <= t.zooms.max && static_cast<int>(out.size()) < n; ++z) {
        const auto box = compat::MinMaxCoordsForZ(t.mercator, t.bounds.oMinX, t.bounds.oMinY, t.bounds.oMaxX,
                                                  t.bounds.oMaxY, z);
        const int cx = (box.min.x + box.max.x) / 2, cy = (box.min.y + box.max.y) / 2;
        for (int dy = -3; dy <= 3; ++dy)
            for (int dx = -3; dx <= 3; ++dx) {
                const int x = cx + dx, row = cy + dy;
                if (x < box.min.x || x > box.max.x || row < box.min.y || row > box.max.y) continue;
                out.push_back({z, x, compat::FlipRow(row, z)});
            }
    }
    if (static_cast<int>(out.size()) > n) out.resize(static_cast<size_t>(n));
    return out;
}

void BM_Tile(benchmark::State& state, const char* id, ddbt::encode::TileFormat format) {
    const char* path = PathOf(id);
    if (path == nullptr) {
        state.SkipWithError("corpus file missing");
        return;
    }
    SetTileCacheBudget(0);
    const auto src = CogSource::Get(path);
    const auto tiles = TilesOf(*src, 64);
    size_t bytes = 0, done = 0;
    for (const auto& t : tiles) {  // warm the blocks (and the statistics)
        TileRequest r{path, t.z, t.x, t.y, 256, true, format};
        benchmark::DoNotOptimize(RenderTile(r));
    }
    for (auto _ : state) {
        for (const auto& t : tiles) {
            TileRequest r{path, t.z, t.x, t.y, 256, true, format};
            const auto b = RenderTile(r);
            bytes += b.size();
            ++done;
        }
    }
    state.counters["tiles"] = benchmark::Counter(static_cast<double>(done), benchmark::Counter::kIsRate);
    state.counters["bytes_per_tile"] = static_cast<double>(bytes) / static_cast<double>(done ? done : 1);
    SetTileCacheBudget(size_t{256} << 20);
}

enum class Stage { kPlan, kGather, kAlpha };

void BM_Stage(benchmark::State& state, const char* id, Stage stage) {
    const char* path = PathOf(id);
    if (path == nullptr) {
        state.SkipWithError("corpus file missing");
        return;
    }
    const auto src = CogSource::Get(path);
    const RasterView view(*src);
    compat::RasterTiling t(256, view.GeoTransform().data(), view.Width(), view.Height());
    std::vector<compat::GQResult> windows;
    for (const auto& tile : TilesOf(*src, 64)) {
        compat::GQResult g;
        if (compat::TileWindow(t, tile.z, tile.x, compat::FlipRow(tile.y, tile.z), g)) windows.push_back(g);
    }
    const int nb = std::min(3, src->DataBandCount());
    std::vector<int> bands;
    for (int b = 1; b <= nb; ++b) bands.push_back(b);
    std::vector<uint8_t> buf(256 * 256 * 8 * 4);
    size_t done = 0;
    for (auto _ : state) {
        for (const auto& g : windows) {
            const SamplePlan p = PlanRead(view, g.r.x, g.r.y, g.r.xsize, g.r.ysize, g.w.xsize, g.w.ysize);
            const size_t n = static_cast<size_t>(p.bufW) * static_cast<size_t>(p.bufH);
            if (stage == Stage::kGather) GatherBands(view, p, bands, buf.data(), n * PixelTypeSize(src->Type()));
            if (stage == Stage::kAlpha) GatherAlpha(view, p, buf.data());
            benchmark::DoNotOptimize(p);
            ++done;
        }
    }
    state.counters["tiles"] = benchmark::Counter(static_cast<double>(done), benchmark::Counter::kIsRate);
}

void BM_StretchU16(benchmark::State& state, bool lut) {
    std::vector<uint16_t> v(65536);
    std::mt19937 rng(1);
    for (auto& x : v) x = static_cast<uint16_t>(rng());
    std::vector<uint8_t> out(v.size());
    for (auto _ : state) {
        if (lut) {
            StretchSamples(PixelType::kU16, reinterpret_cast<const uint8_t*>(v.data()), v.size(), 1234.5, 40000.25,
                           false, out.data(), 1);
        } else {
            for (size_t i = 0; i < v.size(); ++i) out[i] = StretchToByte(v[i], 1234.5, 40000.25);
        }
        benchmark::DoNotOptimize(out.data());
    }
    state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * 65536);
}

void BM_WarpMap(benchmark::State& state) {
    const char* path = PathOf("R3");
    if (path == nullptr) {
        state.SkipWithError("corpus file missing");
        return;
    }
    const auto src = CogSource::Get(path);
    const WarpModel& w = *src->Warp();
    std::vector<int32_t> cols(256), rows(256), sx, sy;
    size_t done = 0;
    int level = 0;
    for (auto _ : state) {
        const int off = static_cast<int>(done % 16) * 256;
        for (int i = 0; i < 256; ++i) cols[static_cast<size_t>(i)] = std::min(w.Width() - 1, off + i);
        for (int i = 0; i < 256; ++i) rows[static_cast<size_t>(i)] = std::min(w.Height() - 1, off + i);
        w.Map(*src, 0, cols, rows, level, sx, sy);
        ++done;
    }
    state.counters["tiles"] = benchmark::Counter(static_cast<double>(done), benchmark::Counter::kIsRate);
}

void BM_StatsR5(benchmark::State& state) {
    const char* path = PathOf("R5");
    if (path == nullptr) {
        state.SkipWithError("corpus file missing");
        return;
    }
    const auto src = CogSource::Get(path);
    for (auto _ : state) benchmark::DoNotOptimize(ComputeBandStatistics(*src, {1}, std::nullopt));
    state.counters["Mpx_per_s"] = benchmark::Counter(
        static_cast<double>(src->Width()) * src->Height() * static_cast<double>(state.iterations()) / 1e6,
        benchmark::Counter::kIsRate);
}

}  // namespace

BENCHMARK_CAPTURE(BM_Tile, R1_png, "R1", ddbt::encode::TileFormat::kPng)->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Tile, R6_png, "R6", ddbt::encode::TileFormat::kPng)->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Tile, R6_jpeg, "R6", ddbt::encode::TileFormat::kJpeg)->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Tile, R6_webp, "R6", ddbt::encode::TileFormat::kWebp)->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Tile, R5_png, "R5", ddbt::encode::TileFormat::kPng)->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Tile, R3_png_warp, "R3", ddbt::encode::TileFormat::kPng)->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Stage, Plan_R6, "R6", Stage::kPlan)->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Stage, Gather_R6, "R6", Stage::kGather)->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Stage, Alpha_R6, "R6", Stage::kAlpha)->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Stage, Gather_R5, "R5", Stage::kGather)->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Stage, Alpha_R5, "R5", Stage::kAlpha)->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_StretchU16, Lut, true);
BENCHMARK_CAPTURE(BM_StretchU16, Direct, false);
BENCHMARK(BM_WarpMap)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_StatsR5)->Unit(benchmark::kSecond)->Iterations(1);
