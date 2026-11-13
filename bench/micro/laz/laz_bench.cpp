// SPDX-License-Identifier: MPL-2.0
// LAZ chunk decode throughput (points per second) on the nodes of corpus COPC files, per point
// format: upstream laz-perf as PDAL / DroneDB run it (a new chunk_decompressor per chunk, full
// LAS records) against the fork's layered decoder (reused, all layers into records; XYZ+RGB(+NIR)
// into arrays; XYZ only into arrays). Each thread decodes the same nodes (~3 M points, file
// order), so items/s with N threads is the aggregate; divide by N for per-thread throughput.
// Decoder set-up cost: a new upstream decompressor vs reusing a layered one.
//
// Files: PDRF 6 = P3 (LHD LAMB93), PDRF 7 = P2 (autzen-classified), PDRF 8 = a COPC of E5 with
// 26 extra bytes (/mnt/data/ddb-tilers-work/fixtures/copc/E5-pdrf8-eb.copc.laz, written by
// `pdal translate point-cloud-complex.laz E5-pdrf8-eb.copc.laz --writers.copc.extra_dims=all`).
#include <benchmark/benchmark.h>
#include <lazperf/readers.hpp>
#if defined(__GLIBC__)
#include <malloc.h>
#endif

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "copc/copc/copc_file.h"
#include "copc/copc/node_decoder.h"
#include "copc/laz/chunk_decoder.h"

namespace {

using namespace ddbt::copc;

struct Corpus {
    std::shared_ptr<const CopcFile> file;
    std::vector<const HierarchyNode*> nodes;  // ~3 M points in file order
    uint64_t points = 0;
    size_t maxPoints = 0;
};

const char* PathOf(int format) {
    switch (format) {
        case 6: return "/mnt/data/datasets/point-clouds/test-crash/LHD_FXX_0658_6859_PTS_O_LAMB93_IGN69.copc.laz";
        case 7: return "/mnt/data/datasets/point-clouds/test-crash/autzen-classified.copc.laz";
        default: return "/mnt/data/ddb-tilers-work/fixtures/copc/E5-pdrf8-eb.copc.laz";
    }
}

const Corpus* Load(int format) {
    static Corpus corpora[3];
    static std::once_flag once[3];
    const size_t k = static_cast<size_t>(format - 6);
    std::call_once(once[k], [k, format] {
        auto f = CopcFile::Open(PathOf(format));
        if (!f.ok()) return;
        corpora[k].file = *f;
        auto all = (*f)->hierarchy().All(NodeOrder::kFileOrder);
        if (!all.ok()) return;
        // Skip the shallow nodes (the biggest), then take nodes until ~3 M points.
        for (const HierarchyNode* n : all->nodes) {
            if (n->key.d < 2) continue;
            corpora[k].nodes.push_back(n);
            corpora[k].points += static_cast<uint64_t>(n->pointCount);
            corpora[k].maxPoints = std::max(corpora[k].maxPoints, static_cast<size_t>(n->pointCount));
            if (corpora[k].points >= 3'000'000) break;
        }
    });
    return corpora[k].file ? &corpora[k] : nullptr;
}

enum Mode { kUpstream = 0, kLayeredRecords = 1, kXyzRgb = 2, kXyz = 3 };

void BM_Laz_Decode(benchmark::State& state) {
    const int format = static_cast<int>(state.range(0));
    const Mode mode = static_cast<Mode>(state.range(1));
    const Corpus* c = Load(format);
    if (c == nullptr) {
        state.SkipWithError("corpus file missing");
        return;
    }
    const CopcFile& f = *c->file;
    const int eb = f.extraBytes();
    const size_t rs = f.header().pointRecordLength;
    std::vector<uint8_t> records(c->maxPoints * rs);
    RawPoints raw;
    raw.x.resize(c->maxPoints);
    raw.y.resize(c->maxPoints);
    raw.z.resize(c->maxPoints);
    raw.r.resize(c->maxPoints);
    raw.g.resize(c->maxPoints);
    raw.b.resize(c->maxPoints);
    raw.nir.resize(c->maxPoints);
    const uint32_t layers = mode == kXyz ? laz::kLayersXyz
                            : mode == kXyzRgb ? (laz::kLayersXyzRgb | laz::kLayerNir)
                                              : laz::kLayersAll;
    auto dec = laz::ChunkDecoder::Create(format, eb, layers);
    if (!dec.ok()) {
        state.SkipWithError("decoder");
        return;
    }
    std::vector<uint8_t> scratch;
    for (auto _ : state) {
        for (const HierarchyNode* n : c->nodes) {
            auto chunk = NodeChunk(f, *n, scratch);
            const uint32_t count = static_cast<uint32_t>(n->pointCount);
            if (mode == kUpstream) {
                lazperf::reader::chunk_decompressor d(format, eb, reinterpret_cast<const char*>(chunk->data()));
                char* p = reinterpret_cast<char*>(records.data());
                for (uint32_t i = 0; i < count; ++i, p += rs) d.decompress(p);
            } else if (mode == kLayeredRecords) {
                if (!(*dec)->DecodeRecords(*chunk, count, records).ok()) state.SkipWithError("decode");
            } else {
                laz::SoaOut o;
                o.x = raw.x.data();
                o.y = raw.y.data();
                o.z = raw.z.data();
                if (mode == kXyzRgb) {
                    o.r = raw.r.data();
                    o.g = raw.g.data();
                    o.b = raw.b.data();
                    o.nir = raw.nir.data();
                }
                if (!(*dec)->DecodeSoa(*chunk, count, o).ok()) state.SkipWithError("decode");
            }
            benchmark::DoNotOptimize(records.data());
            benchmark::DoNotOptimize(raw.x.data());
        }
    }
    state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(c->points));
    static const char* names[] = {"upstream_full_records", "layered_full_records", "layered_xyz_rgb_soa",
                                  "layered_xyz_soa"};
    state.SetLabel(std::string("PDRF ") + std::to_string(format) + " " + names[mode]);
}

void DecodeArgs(benchmark::Benchmark* b) {
    for (int format : {6, 7, 8})
        for (int mode : {0, 1, 2, 3}) b->Args({format, mode});
}
BENCHMARK(BM_Laz_Decode)->Apply(DecodeArgs)->Unit(benchmark::kMillisecond)->UseRealTime();
BENCHMARK(BM_Laz_Decode)
    ->Args({7, 0})
    ->Args({7, 2})
    ->Args({6, 0})
    ->Args({6, 3})
    ->Threads(8)
    ->Threads(16)
    ->Unit(benchmark::kMillisecond)
    ->UseRealTime();

// Decoder set-up per chunk: upstream builds every model of a new decompressor (and the first
// point); the layered decoder resets the models it used. Memory: heap bytes held by one decoder.
void BM_Laz_DecoderSetup(benchmark::State& state) {
    const bool layered = state.range(0) != 0;
    const Corpus* c = Load(7);
    if (c == nullptr) {
        state.SkipWithError("corpus file missing");
        return;
    }
    const CopcFile& f = *c->file;
    std::vector<uint8_t> scratch;
    auto chunk = NodeChunk(f, *c->nodes.front(), scratch);
    std::vector<uint8_t> rec(64);
#if defined(__GLIBC__)
    struct mallinfo2 before = mallinfo2();
#endif
    auto dec = laz::ChunkDecoder::Create(7, 0, laz::kLayersXyzRgb);
    std::vector<int32_t> x(1), y(1), z(1);
    std::vector<uint16_t> r(1), g(1), b(1);
    laz::SoaOut o{x.data(), y.data(), z.data(), r.data(), g.data(), b.data(), nullptr};
    (void)(*dec)->DecodeSoa(chunk->subspan(0), 1, o);  // reports a count mismatch: set-up only
#if defined(__GLIBC__)
    struct mallinfo2 after = mallinfo2();
#endif
    for (auto _ : state) {
        if (layered) {
            (void)(*dec)->DecodeSoa(*chunk, 1, o);
        } else {
            lazperf::reader::chunk_decompressor d(7, 0, reinterpret_cast<const char*>(chunk->data()));
            d.decompress(reinterpret_cast<char*>(rec.data()));
        }
        benchmark::ClobberMemory();
    }
#if defined(__GLIBC__)
    if (layered) {
        state.counters["decoder_heap_bytes"] = static_cast<double>(after.uordblks - before.uordblks);
    } else {
        struct mallinfo2 b0 = mallinfo2();
        auto* d = new lazperf::reader::chunk_decompressor(7, 0, reinterpret_cast<const char*>(chunk->data()));
        d->decompress(reinterpret_cast<char*>(rec.data()));
        struct mallinfo2 b1 = mallinfo2();
        delete d;
        state.counters["decoder_heap_bytes"] = static_cast<double>(b1.uordblks - b0.uordblks);
    }
#endif
    state.SetLabel(layered ? "layered reset + first point" : "upstream new decompressor + first point");
}
BENCHMARK(BM_Laz_DecoderSetup)->Arg(0)->Arg(1)->Unit(benchmark::kMicrosecond);

}  // namespace
