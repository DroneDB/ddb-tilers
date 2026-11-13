// SPDX-License-Identifier: MPL-2.0
// COPC reader costs on P3 (LHD LAMB93, 29.9 M points, 1468 nodes) and P2 (autzen, 278 nodes):
//  - open: header, VLRs / EVLRs and the hierarchy (mapped file, page cache warm);
//  - select() per tile window at zoom 12..22: windows of the ground size of a 256-px web
//    mercator tile at the cloud's latitude (P3: 48.85 N; size = 40075016.686 * cos(lat) / 2^z),
//    on a 7x7 grid of centres over the cloud, with depthEnd from the tile's resolution (size /
//    256) as readers.copc computes it; against DroneDB's linear scan (every node, pow() per node);
//  - node decode into the compact form, and its memory (bytes per point).
#include <benchmark/benchmark.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "copc/copc/copc_file.h"
#include "copc/copc/node_decoder.h"

namespace {

using namespace ddbt::copc;

constexpr const char* kP3 = "/mnt/data/datasets/point-clouds/test-crash/LHD_FXX_0658_6859_PTS_O_LAMB93_IGN69.copc.laz";
constexpr const char* kP2 = "/mnt/data/datasets/point-clouds/test-crash/autzen-classified.copc.laz";

std::shared_ptr<const CopcFile> Shared(const char* path) {
    static std::mutex m;
    static std::vector<std::pair<std::string, std::shared_ptr<const CopcFile>>> files;
    std::lock_guard<std::mutex> lock(m);
    for (auto& [p, f] : files)
        if (p == path) return f;
    auto f = CopcFile::Open(path);
    files.emplace_back(path, f.ok() ? *f : nullptr);
    return files.back().second;
}

void BM_Copc_Open(benchmark::State& state) {
    const char* path = state.range(0) == 3 ? kP3 : kP2;
    if (!Shared(path)) {
        state.SkipWithError("corpus file missing");
        return;
    }
    for (auto _ : state) {
        auto f = CopcFile::Open(path);
        benchmark::DoNotOptimize(f);
    }
    state.SetLabel(state.range(0) == 3 ? "P3 header + VLRs + hierarchy (1468 nodes)" : "P2 (278 nodes)");
}
BENCHMARK(BM_Copc_Open)->Arg(3)->Arg(2)->Unit(benchmark::kMicrosecond);

void BM_Copc_HierarchyLoad(benchmark::State& state) {
    auto f = Shared(kP3);
    if (!f) {
        state.SkipWithError("corpus file missing");
        return;
    }
    const las::CopcInfo& i = f->info();
    const Box3 c = f->RootCube();
    for (auto _ : state) {
        auto h = HierarchyIndex::Load(f->sharedSource(), i.rootHierOffset, i.rootHierSize,
                                      {c.minX, c.minY, c.minZ, c.maxX, c.maxY, c.maxZ}, true);
        benchmark::DoNotOptimize(h);
    }
    state.SetLabel("P3 root page, 1468 entries");
}
BENCHMARK(BM_Copc_HierarchyLoad)->Unit(benchmark::kMicrosecond);

std::vector<SelectQuery> TileQueries(const CopcFile& f, int zoom, double latDeg) {
    const double size = 40075016.686 * std::cos(latDeg * 3.14159265358979323846 / 180) / std::pow(2.0, zoom);
    const las::LasHeader& h = f.header();
    std::vector<SelectQuery> out;
    for (int i = 0; i < 7; ++i)
        for (int j = 0; j < 7; ++j) {
            const double cx = h.min[0] + (h.max[0] - h.min[0]) * (i + 0.5) / 7;
            const double cy = h.min[1] + (h.max[1] - h.min[1]) * (j + 0.5) / 7;
            SelectQuery q;
            q.box = Box3::Xy(cx - size / 2, cy - size / 2, cx + size / 2, cy + size / 2);
            q.depthEnd = DepthEndForResolution(f.spacing(), size / 256);
            out.push_back(q);
        }
    return out;
}

void BM_Copc_Select(benchmark::State& state) {
    auto f = Shared(kP3);
    if (!f) {
        state.SkipWithError("corpus file missing");
        return;
    }
    const int zoom = static_cast<int>(state.range(0));
    const auto queries = TileQueries(*f, zoom, 48.85);
    uint64_t nodes = 0, visited = 0;
    for (auto _ : state) {
        nodes = visited = 0;
        for (const SelectQuery& q : queries) {
            auto s = f->hierarchy().Select(q);
            nodes += s->nodes.size();
            visited += s->visited;
            benchmark::DoNotOptimize(s);
        }
    }
    state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(queries.size()));
    state.counters["nodes_per_tile"] = static_cast<double>(nodes) / static_cast<double>(queries.size());
    state.counters["visited_per_tile"] = static_cast<double>(visited) / static_cast<double>(queries.size());
    state.counters["depthEnd"] = queries.front().depthEnd;
    state.SetLabel("P3, z" + std::to_string(zoom) + ", 49 windows");
}
BENCHMARK(BM_Copc_Select)->DenseRange(12, 22)->Unit(benchmark::kMicrosecond);

// DroneDB's CopcNodeReader::read: every node, depth test, pow() per node for the bounds.
void BM_Copc_SelectLinearScan(benchmark::State& state) {
    auto f = Shared(kP3);
    if (!f) {
        state.SkipWithError("corpus file missing");
        return;
    }
    const int zoom = static_cast<int>(state.range(0));
    const auto queries = TileQueries(*f, zoom, 48.85);
    auto all = f->hierarchy().All(NodeOrder::kFileOrder);
    const las::CopcInfo& info = f->info();
    for (auto _ : state) {
        size_t n = 0;
        for (const SelectQuery& q : queries) {
            std::vector<const HierarchyNode*> sel;
            for (const HierarchyNode* node : all->nodes) {
                if (node->key.d >= q.depthEnd) continue;
                const double rootMinX = info.centerX - info.halfSize, rootMaxX = info.centerX + info.halfSize;
                const double rootMinY = info.centerY - info.halfSize, rootMaxY = info.centerY + info.halfSize;
                const int width = static_cast<int>(std::pow(2, node->key.d));
                const double cw = (rootMaxX - rootMinX) / width;
                const double minX = node->key.x == 0 ? rootMinX : rootMinX + cw * node->key.x;
                const double maxX = node->key.x == width - 1 ? rootMaxX : rootMinX + cw * (node->key.x + 1);
                const double minY = node->key.y == 0 ? rootMinY : rootMinY + cw * node->key.y;
                const double maxY = node->key.y == width - 1 ? rootMaxY : rootMinY + cw * (node->key.y + 1);
                if (minX <= q.box.maxX && maxX >= q.box.minX && minY <= q.box.maxY && maxY >= q.box.minY)
                    sel.push_back(node);
            }
            n += sel.size();
            benchmark::DoNotOptimize(sel.data());
        }
        benchmark::DoNotOptimize(n);
    }
    state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(queries.size()));
    state.SetLabel("P3, z" + std::to_string(zoom) + ", 49 windows, linear scan as DroneDB");
}
BENCHMARK(BM_Copc_SelectLinearScan)->Arg(12)->Arg(16)->Arg(20)->Arg(22)->Unit(benchmark::kMicrosecond);

// Compact decode of every node (XYZ+RGB layers), and the memory it keeps.
void BM_Copc_DecodeCompact(benchmark::State& state) {
    auto f = Shared(state.range(0) == 3 ? kP3 : kP2);
    if (!f) {
        state.SkipWithError("corpus file missing");
        return;
    }
    auto all = f->hierarchy().All(NodeOrder::kFileOrder);
    NodeDecoder dec(f);
    uint64_t bytes = 0, points = 0;
    for (auto _ : state) {
        bytes = points = 0;
        for (const HierarchyNode* n : all->nodes) {
            auto c = dec.DecodeCompact(*n);
            bytes += c->Bytes();
            points += c->size();
            benchmark::DoNotOptimize(c);
        }
    }
    state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(points));
    state.counters["bytes_per_point"] = static_cast<double>(bytes) / static_cast<double>(points);
    state.SetLabel(state.range(0) == 3 ? "P3 every node (PDRF 6: XYZ)" : "P2 every node (PDRF 7: XYZ + RGBA8)");
}
BENCHMARK(BM_Copc_DecodeCompact)->Arg(3)->Arg(2)->Unit(benchmark::kMillisecond)->UseRealTime()->Iterations(3);

}  // namespace
