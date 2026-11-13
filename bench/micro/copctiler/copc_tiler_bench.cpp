// SPDX-License-Identifier: MPL-2.0
// COPC tile latency per zoom: RenderTile (PNG, 256 px, encoded-tile cache off) on the tiles of
// a 7 x 7 grid around the centre of the cloud's footprint (fewer at low zooms), z12..z22.
//
//   BM_CopcTile/<cloud>/<zoom>/<mode>/<threads>/<cold>
//     cloud: 2 = P2 autzen (PDRF 7, ft, 10.7 M), 3 = P3 LHD (PDRF 6, ramp, 29.9 M),
//            1 = P1 brighton (prepared COPC), 5 = E5 (PDRF 8)
//     mode: 0 = compat, 1 = hybrid (the default), 2 = modern; threads: client threads sharing the
//     tile list
//     cold: 1 = node cache emptied and the file's pages evicted (madvise + posix_fadvise) before
//           each pass over the tile list; 0 = warm (one pass beforehand)
//
// One iteration = one pass over the list; counters: p50_ms / p99_ms / mean_ms of the tiles (of
// every pass), tiles_per_s (wall), tiles, decodes_per_pass (node cache loads), nodes_per_tile,
// points_per_tile (points in the read window). Corpus files missing:
// skipped. DDBT_COPC_BENCH_REQUESTS=DIR also writes each tile list as a ddbt-bench request file
// (DIR/<cloud>-z<zoom>.jsonl), so the baseline runs the same tiles.
#include <benchmark/benchmark.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "cog/tiler/tile_error.h"
#include "copc/cache/node_cache.h"
#include "copc/tiler/cloud.h"
#include "copc/tiler/copc_tiler.h"
#include "core/io/byte_source.h"
#include "core/io/file.h"
#include "core/tile/compat.h"

namespace {

using namespace ddbt::copc::tiler;
namespace compat = ddbt::tile::compat;

const char* PathOf(int64_t id) {
    switch (id) {
        case 1: return "/mnt/data/ddb-tilers-work/prepared/P1-b9c0ec9b47b4/cloud.copc.laz";
        case 2: return "/mnt/data/datasets/point-clouds/test-crash/autzen-classified.copc.laz";
        case 3: return "/mnt/data/datasets/point-clouds/test-crash/LHD_FXX_0658_6859_PTS_O_LAMB93_IGN69.copc.laz";
        case 5: return "/mnt/data/ddb-tilers-work/prepared/E5-f3f745895662/cloud.copc.laz";
        default: return "";
    }
}

Mode ModeOf(int64_t arg) {
    switch (arg) {
        case 0: return Mode::kCompat;
        case 2: return Mode::kModern;
        default: return Mode::kHybrid;
    }
}

struct TileXyz {
    int z, x, y;  // XYZ row
};

std::vector<TileXyz> TileList(const Cloud& cloud, int z) {
    std::vector<TileXyz> out;
    const auto& fp = cloud.compatFootprint();
    if (!fp) return out;
    const compat::GlobalMercator m(256);
    const auto range = compat::MinMaxCoordsForZ(m, fp->minX, fp->minY, fp->maxX, fp->maxY, z);
    const auto c = m.metersToTile((fp->minX + fp->maxX) / 2, (fp->minY + fp->maxY) / 2, z);
    for (int dy = -3; dy <= 3; ++dy)
        for (int dx = -3; dx <= 3; ++dx) {
            const int x = c.x + dx, tms = c.y + dy;
            if (range.Contains(x, tms)) out.push_back({z, x, compat::FlipRow(tms, z)});
        }
    return out;
}

void Evict(const Cloud& cloud) {
    ddbt::copc::cache::NodeCache::Global().Clear();
    if (const auto* m = dynamic_cast<const ddbt::io::MappedSource*>(&cloud.file().source()))
        m->map().Advise(ddbt::io::MapAdvice::kDontNeed);
    if (auto f = ddbt::io::File::Open(cloud.path()); f.ok()) (void)f->DropCache();
}

void WriteRequests(const char* dir, int64_t cloudId, int z, const std::string& path, const std::vector<TileXyz>& t) {
    std::filesystem::create_directories(dir);
    std::ofstream f(std::filesystem::path(dir) / ("P" + std::to_string(cloudId) + "-z" + std::to_string(z) + ".jsonl"));
    for (const auto& x : t)
        f << "{\"path\":\"" << path << "\",\"z\":" << x.z << ",\"x\":" << x.x << ",\"y\":" << x.y
          << ",\"size\":256,\"format\":\"png\",\"tms\":true,\"kind\":\"grid\"}\n";
}

void BM_CopcTile(benchmark::State& state) {
    const int64_t cloudId = state.range(0);
    const int z = static_cast<int>(state.range(1));
    const Mode mode = ModeOf(state.range(2));
    const int threads = static_cast<int>(state.range(3));
    const bool cold = state.range(4) != 0;
    const std::string path = PathOf(cloudId);
    if (path.empty() || !std::filesystem::exists(path)) {
        state.SkipWithError("corpus file missing");
        return;
    }
    SetTileCacheBudget(0);
    std::shared_ptr<const Cloud> cloud;
    try {
        cloud = Cloud::Get(path);
    } catch (const std::exception& e) {
        state.SkipWithError(e.what());
        return;
    }
    const auto tiles = TileList(*cloud, z);
    if (tiles.empty()) {
        state.SkipWithError("no tile in range");
        return;
    }
    if (const char* dir = std::getenv("DDBT_COPC_BENCH_REQUESTS")) WriteRequests(dir, cloudId, z, path, tiles);

    auto request = [&](const TileXyz& t) {
        TileRequest r;
        r.path = path;
        r.z = t.z, r.x = t.x, r.y = t.y, r.tileSize = 256, r.tms = true, r.mode = mode, r.noCache = true;
        return r;
    };
    if (!cold)
        for (const auto& t : tiles) RenderTile(request(t));

    std::vector<double> lat;
    std::mutex latMutex;
    double wall = 0;
    const uint64_t loads0 = ddbt::copc::cache::NodeCache::Global().Stats().loads;
    uint64_t passes = 0;
    for (auto _ : state) {
        if (cold) {
            state.PauseTiming();
            Evict(*cloud);
            state.ResumeTiming();
        }
        std::atomic<size_t> next{0};
        auto work = [&] {
            std::vector<double> mine;
            for (size_t i; (i = next.fetch_add(1)) < tiles.size();) {
                const auto t0 = std::chrono::steady_clock::now();
                auto bytes = RenderTile(request(tiles[i]));
                benchmark::DoNotOptimize(bytes.data());
                mine.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
            }
            std::lock_guard lock(latMutex);
            lat.insert(lat.end(), mine.begin(), mine.end());
        };
        const auto w0 = std::chrono::steady_clock::now();
        std::vector<std::thread> pool;
        for (int k = 1; k < threads; ++k) pool.emplace_back(work);
        work();
        for (auto& th : pool) th.join();
        wall += std::chrono::duration<double>(std::chrono::steady_clock::now() - w0).count();
        ++passes;
    }
    const uint64_t loads = ddbt::copc::cache::NodeCache::Global().Stats().loads - loads0;
    std::sort(lat.begin(), lat.end());
    auto pct = [&](double p) { return lat.empty() ? 0.0 : lat[static_cast<size_t>(p * static_cast<double>(lat.size() - 1))]; };
    double sum = 0;
    for (const double v : lat) sum += v;
    state.counters["p50_ms"] = pct(0.5);
    state.counters["p99_ms"] = pct(0.99);
    state.counters["mean_ms"] = lat.empty() ? 0 : sum / static_cast<double>(lat.size());
    state.counters["tiles_per_s"] = wall > 0 ? static_cast<double>(lat.size()) / wall : 0;
    state.counters["tiles"] = static_cast<double>(tiles.size());
    state.counters["decodes_per_pass"] = passes ? static_cast<double>(loads) / static_cast<double>(passes) : 0;

    // What a tile reads (from one render of each, warm).
    double nodes = 0, points = 0;
    for (const auto& t : tiles) {
        RenderInfo info;
        std::vector<uint8_t> rgba;
        RenderRgba(*cloud, request(t), rgba, &info);
        nodes += info.nodes;
        points += static_cast<double>(info.inWindow);
    }
    state.counters["nodes_per_tile"] = nodes / static_cast<double>(tiles.size());
    state.counters["points_per_tile"] = points / static_cast<double>(tiles.size());
    state.SetLabel(std::string(ModeName(mode)) + (cold ? " cold" : " warm"));
}

// RenderRgba only (no encoding), warm, one thread: the tiler's own cost per tile.
void BM_CopcRender(benchmark::State& state) {
    const int64_t cloudId = state.range(0);
    const int z = static_cast<int>(state.range(1));
    const Mode mode = ModeOf(state.range(2));
    const std::string path = PathOf(cloudId);
    if (path.empty() || !std::filesystem::exists(path)) {
        state.SkipWithError("corpus file missing");
        return;
    }
    const auto cloud = Cloud::Get(path);
    const auto tiles = TileList(*cloud, z);
    if (tiles.empty()) {
        state.SkipWithError("no tile in range");
        return;
    }
    std::vector<uint8_t> rgba;
    auto render = [&](const TileXyz& t) {
        TileRequest r;
        r.path = path;
        r.z = t.z, r.x = t.x, r.y = t.y, r.tileSize = 256, r.tms = true, r.mode = mode;
        RenderRgba(*cloud, r, rgba);
    };
    for (const auto& t : tiles) render(t);
    const auto before = ddbt::copc::cache::NodeCache::Global().Stats();
    size_t n = 0;
    for (auto _ : state) {
        for (const auto& t : tiles) render(t);
        n += tiles.size();
    }
    const auto after = ddbt::copc::cache::NodeCache::Global().Stats();
    state.counters["decodes"] = static_cast<double>(after.loads - before.loads);
    state.counters["evictions"] = static_cast<double>(after.evictions - before.evictions);
    state.counters["rejected"] = static_cast<double>(after.rejected - before.rejected);
    state.counters["cached_mb"] = static_cast<double>(after.bytes) / 1048576.0;
    state.counters["us_per_tile"] =
        benchmark::Counter(static_cast<double>(n), benchmark::Counter::kIsRate | benchmark::Counter::kInvert);
    state.SetLabel(std::string(ModeName(mode)) + " render only");
}
BENCHMARK(BM_CopcRender)
    ->ArgsProduct({{2, 3}, {14, 16, 18, 20, 22}, {1, 2}})
    ->Unit(benchmark::kMillisecond)
    ->UseRealTime()
    ->MinTime(0.2);

void Args(benchmark::Benchmark* b) {
    for (const int64_t cloud : {2, 3})
        for (int64_t z = 12; z <= 22; ++z)
            for (const int64_t mode : {1, 2, 0})
                for (const int64_t threads : {1, 8})
                    for (const int64_t cold : {0, 1}) b->Args({cloud, z, mode, threads, cold});
    for (const int64_t cloud : {1, 5})
        for (int64_t z = 14; z <= 22; z += 2)
            for (const int64_t mode : {1, 2})
                for (const int64_t cold : {0, 1}) b->Args({cloud, z, mode, 1, cold});
}

BENCHMARK(BM_CopcTile)->Apply(Args)->Unit(benchmark::kMillisecond)->UseRealTime()->MinTime(0.2);

}  // namespace
