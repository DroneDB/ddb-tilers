// SPDX-License-Identifier: MPL-2.0
//
// Coordinate transform throughput: exact PROJ batches vs GridApprox vs the analytic EPSG:4326 ->
// EPSG:3857 form vs a polynomial fit, 1 to 32 threads sharing one prepared transform.
//
// Counters: items_per_second = points/s of all threads together (wall time), pts_per_thread =
// points/s of one thread. One iteration transforms a batch of 4096 points (a 64 x 64 sample of a
// 512 px tile, the order of a warp row block or a COPC node).

#include <benchmark/benchmark.h>

#include <cstdint>
#include <random>
#include <vector>

#include "core/crs/crs.h"
#include "core/crs/grid_approx.h"
#include "core/crs/polyfit.h"
#include "core/crs/transform.h"
#include "core/crs/web_mercator_analytic.h"

namespace {

using namespace ddbt::crs;

constexpr size_t kBatch = 4096;

struct Window {
    int epsg;
    double minX, minY, maxX, maxY;
};
// A z14 tile-sized (2.4 km) window of R3's CRS (BD72 / Belgian Lambert 72) and of SWEREF99.
constexpr Window kBd72{31370, 150000, 170000, 152446, 172446};
constexpr Window kSweref{3008, 150000, 6400000, 152446, 6402446};

std::vector<double> Uniform(double lo, double hi, size_t n, uint64_t seed) {
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> d(lo, hi);
    std::vector<double> v(n);
    for (auto& x : v) x = d(rng);
    return v;
}

void Count(benchmark::State& state) {
    const auto n = static_cast<double>(state.iterations()) * static_cast<double>(kBatch);
    state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(kBatch));
    state.counters["pts_per_thread"] = benchmark::Counter(n, benchmark::Counter::kIsRate | benchmark::Counter::kAvgThreads);
}

std::shared_ptr<const Transform> TransformOf(const Window& w, int dst = 3857, bool analytic = true) {
    TransformOptions o;
    o.analyticWebMercator = analytic;
    return TransformCache::Global().GetForExtent(Crs::FromEpsg(w.epsg), Crs::FromEpsg(dst), w.minX, w.minY, w.maxX,
                                                 w.maxY, o);
}

template <const Window& W>
void BM_ProjExact(benchmark::State& state) {
    static const auto t = TransformOf(W);
    const auto xs = Uniform(W.minX, W.maxX, kBatch, 1 + static_cast<uint64_t>(state.thread_index()));
    const auto ys = Uniform(W.minY, W.maxY, kBatch, 2 + static_cast<uint64_t>(state.thread_index()));
    std::vector<double> x(kBatch), y(kBatch);
    for (auto _ : state) {
        x = xs;
        y = ys;
        benchmark::DoNotOptimize(t->TransformPoints(x.data(), y.data(), nullptr, kBatch));
    }
    Count(state);
}

template <const Window& W>
void BM_GridApprox(benchmark::State& state) {
    // 0.125 px at z18 (0.597 m/px): 0.075 m; the per-point cost hardly depends on the bound.
    static const GridApprox g = [] {
        GridApproxOptions o;
        o.maxError = 0.075;
        return GridApprox::Build(TransformOf(W), W.minX, W.minY, W.maxX, W.maxY, o);
    }();
    const auto xs = Uniform(W.minX, W.maxX, kBatch, 1 + static_cast<uint64_t>(state.thread_index()));
    const auto ys = Uniform(W.minY, W.maxY, kBatch, 2 + static_cast<uint64_t>(state.thread_index()));
    std::vector<double> x(kBatch), y(kBatch);
    for (auto _ : state) {
        x = xs;
        y = ys;
        benchmark::DoNotOptimize(g.TransformPoints(x.data(), y.data(), kBatch));
    }
    Count(state);
    state.counters["leaves"] = benchmark::Counter(static_cast<double>(g.Stats().leafCells), benchmark::Counter::kAvgThreads);
}

template <const Window& W>
void BM_PolyFitCubic(benchmark::State& state) {
    static const PolyFit cubic = [] {
        // Force degree 3 (the COPC tiler's worst case)
        const auto t = TransformOf(W);
        std::vector<double> x = Uniform(W.minX, W.maxX, 256, 7), y = Uniform(W.minY, W.maxY, 256, 8);
        std::vector<double> u = x, v = y;
        t->TransformPoints(u.data(), v.data(), nullptr, u.size());
        return *FitPolynomial(x.data(), y.data(), u.data(), v.data(), x.size(), 3);
    }();
    const auto xs = Uniform(W.minX, W.maxX, kBatch, 1 + static_cast<uint64_t>(state.thread_index()));
    const auto ys = Uniform(W.minY, W.maxY, kBatch, 2 + static_cast<uint64_t>(state.thread_index()));
    std::vector<double> x(kBatch), y(kBatch);
    for (auto _ : state) {
        cubic.Eval(xs.data(), ys.data(), x.data(), y.data(), kBatch);
        benchmark::DoNotOptimize(x.data());
        benchmark::DoNotOptimize(y.data());
    }
    Count(state);
}

void BM_Analytic4326To3857(benchmark::State& state) {
    const auto lon = Uniform(4.3, 4.4, kBatch, 3 + static_cast<uint64_t>(state.thread_index()));
    const auto lat = Uniform(50.8, 50.9, kBatch, 4 + static_cast<uint64_t>(state.thread_index()));
    std::vector<double> x(kBatch), y(kBatch);
    for (auto _ : state) {
        benchmark::DoNotOptimize(analytic::LonLatToWebMercator(lon.data(), lat.data(), x.data(), y.data(), kBatch));
    }
    Count(state);
}

void BM_Analytic3857To4326Gdal(benchmark::State& state) {
    const auto mx = Uniform(478000, 490000, kBatch, 3 + static_cast<uint64_t>(state.thread_index()));
    const auto my = Uniform(6590000, 6600000, kBatch, 4 + static_cast<uint64_t>(state.thread_index()));
    std::vector<double> x(kBatch), y(kBatch);
    for (auto _ : state) {
        benchmark::DoNotOptimize(analytic::WebMercatorToLonLatGdal(mx.data(), my.data(), x.data(), y.data(), kBatch));
    }
    Count(state);
}

void BM_Proj4326To3857(benchmark::State& state) {
    static const auto t = [] {
        TransformOptions o;
        o.analyticWebMercator = false;
        return Transform::Create(Crs::FromEpsg(4326), Crs::FromEpsg(3857), o);
    }();
    const auto lon = Uniform(4.3, 4.4, kBatch, 3 + static_cast<uint64_t>(state.thread_index()));
    const auto lat = Uniform(50.8, 50.9, kBatch, 4 + static_cast<uint64_t>(state.thread_index()));
    std::vector<double> x(kBatch), y(kBatch);
    for (auto _ : state) {
        x = lon;
        y = lat;
        benchmark::DoNotOptimize(t->TransformPoints(x.data(), y.data(), nullptr, kBatch));
    }
    Count(state);
}

void BM_GridApproxBuild(benchmark::State& state) {
    // Build cost for one 512 px z18 tile window (306 m) of BD72, at 0.125 px.
    const auto t = TransformOf(kBd72);
    GridApproxOptions o;
    o.maxError = 0.0373 * 0.125;
    for (auto _ : state) {
        auto g = GridApprox::Build(t, 150000, 170000, 150306, 170306, o);
        size_t leaves = g.Stats().leafCells;
        benchmark::DoNotOptimize(leaves);
    }
}

void BM_TransformCacheHit(benchmark::State& state) {
    const Crs a = Crs::FromEpsg(31370), b = Crs::FromEpsg(3857);
    TransformCache::Global().Get(a, b);
    for (auto _ : state) benchmark::DoNotOptimize(TransformCache::Global().Get(a, b));
}

#define DDBT_THREADS ->UseRealTime()->Threads(1)->Threads(2)->Threads(4)->Threads(8)->Threads(16)->Threads(32)

BENCHMARK_TEMPLATE(BM_ProjExact, kBd72) DDBT_THREADS;
BENCHMARK_TEMPLATE(BM_ProjExact, kSweref) DDBT_THREADS;
BENCHMARK_TEMPLATE(BM_GridApprox, kBd72) DDBT_THREADS;
BENCHMARK_TEMPLATE(BM_GridApprox, kSweref) DDBT_THREADS;
BENCHMARK_TEMPLATE(BM_PolyFitCubic, kBd72) DDBT_THREADS;
BENCHMARK(BM_Analytic4326To3857) DDBT_THREADS;
BENCHMARK(BM_Analytic3857To4326Gdal)->UseRealTime()->Threads(1)->Threads(32);
BENCHMARK(BM_Proj4326To3857) DDBT_THREADS;
BENCHMARK(BM_GridApproxBuild);
BENCHMARK(BM_TransformCacheHit)->Threads(1)->Threads(8);

}  // namespace
