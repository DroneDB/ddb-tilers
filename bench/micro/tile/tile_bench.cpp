// SPDX-License-Identifier: MPL-2.0
//
// Tile maths per call: lon/lat -> tile, tile bounds, bounds -> tile range, and the baseline's
// geoQuery / tileWindow (compat). These are far from any hot path today; the numbers guard
// against regressions when the tilers call them per tile or per request.

#include <benchmark/benchmark.h>

#include <cstdint>
#include <random>
#include <vector>

#include "core/tile/compat.h"
#include "core/tile/web_mercator.h"

namespace {

using namespace ddbt::tile;

constexpr size_t kN = 1024;

void BM_LonLatToTile(benchmark::State& state) {
    std::mt19937_64 rng(1);
    std::uniform_real_distribution<double> lon(-180, 180), lat(-85, 85);
    std::vector<double> lo(kN), la(kN);
    for (size_t i = 0; i < kN; ++i) {
        lo[i] = lon(rng);
        la[i] = lat(rng);
    }
    for (auto _ : state) {
        uint64_t acc = 0;
        for (size_t i = 0; i < kN; ++i) acc += WebMercatorQuad::LonLatToTile(lo[i], la[i], 18).packed();
        benchmark::DoNotOptimize(acc);
    }
    state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(kN));
}

void BM_TileBounds(benchmark::State& state) {
    const WebMercatorQuad q(static_cast<int>(state.range(0)));
    for (auto _ : state) {
        double acc = 0;
        for (uint32_t i = 0; i < kN; ++i) acc += q.TileBounds(TileKey(18, 130000 + i, 90000 + i)).minX;
        benchmark::DoNotOptimize(acc);
    }
    state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(kN));
}

void BM_TilesForBounds(benchmark::State& state) {
    const WebMercatorQuad q(256);
    const Bounds b{478000.5, 6590000.25, 490000.75, 6600000.5};
    for (auto _ : state) {
        int64_t acc = 0;
        for (int z = 0; z < 23; ++z) acc += q.TilesForBounds(b, z, Scheme::XYZ).Count();
        benchmark::DoNotOptimize(acc);
    }
    state.SetItemsProcessed(state.iterations() * 23);
}

void BM_CompatTileWindow(benchmark::State& state) {
    // R1-like 3857 COG; every tile of z20 over it, as GDALTiler::tileWindow computes them.
    const double gt[6] = {-10551262.5321, 0.0298582141, 0.0, 3434589.4129, 0.0, -0.0298582141};
    const compat::RasterTiling t(256, gt, 1913, 1860);
    const auto r = compat::MinMaxCoordsForZ(t.mercator, t.bounds.oMinX, t.bounds.oMinY, t.bounds.oMaxX,
                                            t.bounds.oMaxY, 20);
    int64_t tiles = 0;
    for (auto _ : state) {
        int ok = 0;
        tiles = 0;
        for (int tx = r.min.x; tx <= r.max.x; ++tx)
            for (int ty = r.min.y; ty <= r.max.y; ++ty) {
                compat::GQResult g;
                ok += compat::TileWindow(t, 20, tx, ty, g);
                ++tiles;
            }
        benchmark::DoNotOptimize(ok);
    }
    state.SetItemsProcessed(state.iterations() * tiles);
}

BENCHMARK(BM_LonLatToTile);
BENCHMARK(BM_TileBounds)->Arg(256)->Arg(512);
BENCHMARK(BM_TilesForBounds);
BENCHMARK(BM_CompatTileWindow);

}  // namespace
