// SPDX-License-Identifier: MPL-2.0
//
// TiffReader microbenchmarks on R9 (ortho.tif, 7.3 GB BigTIFF COG, 89559 x 56721 RGBA,
// 512 x 512 DEFLATE tiles, 8 overviews + masks: 18 IFDs) and R10 (1-row strips).
//
//   Open/mmap, Open/pread      header + every IFD (eager tags; block indexes not loaded)
//   IndexLoad/pread            Open + every IFD's TileOffsets / TileByteCounts loaded
//                              (the mmap source reads entries straight from the mapping)
//   ReadBlock/R9               random full-resolution tiles: locate + read + DEFLATE + copy
//                              (page cache warm after the first pass)
//
// Paths come from the TIFF fixtures index (see codecs_bench.cpp).

#include <benchmark/benchmark.h>

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <random>
#include <string>

#include "cog/tiff/tiff_reader.h"
#include "core/io/byte_source.h"
#include "core/mem/aligned.h"

namespace {

using namespace ddbt;
using namespace ddbt::cog::tiff;

std::string CorpusPath(const std::string& id) {
    const char* e = std::getenv("DDBT_TIFF_FIXTURES");
    const std::string dir = e != nullptr ? e : "/mnt/data/ddb-tilers-work/fixtures/tiff";
    std::ifstream f(dir + "/index.tsv");
    std::string line;
    while (std::getline(f, line)) {
        const size_t a = line.find('\t');
        const size_t b = line.find('\t', a + 1);
        if (a != std::string::npos && b != std::string::npos && line.substr(0, a) == id) return line.substr(a + 1, b - a - 1);
    }
    return {};
}

Expected<std::shared_ptr<const TiffReader>> OpenWith(const std::string& path, io::SourceKind kind) {
    auto src = io::OpenByteSource(path, kind, io::AccessHint::kRandom);
    if (!src.ok()) return std::move(src).TakeStatus();
    return TiffReader::Open(std::shared_ptr<const io::ByteSource>(std::move(*src)));
}

void BM_Open(benchmark::State& state, const char* id, io::SourceKind kind, bool loadIndex) {
    const std::string path = CorpusPath(id);
    if (path.empty()) {
        state.SkipWithError("corpus file missing (fixtures index)");
        return;
    }
    size_t ifds = 0, blocks = 0;
    for (auto _ : state) {
        auto r = OpenWith(path, kind);
        if (!r.ok()) {
            state.SkipWithError(r.status().ToString().c_str());
            return;
        }
        ifds = (*r)->IfdCount();
        if (loadIndex) {
            blocks = 0;
            for (uint32_t i = 0; i < ifds; ++i) {
                if (!(*r)->LoadBlockIndex(i).ok()) {
                    state.SkipWithError("LoadBlockIndex failed");
                    return;
                }
                blocks += (*r)->Ifd(i).blockCount;
            }
        }
        benchmark::DoNotOptimize(r);
    }
    state.counters["ifds"] = static_cast<double>(ifds);
    state.counters["blocks"] = static_cast<double>(blocks);
}

void BM_ReadBlock(benchmark::State& state, const char* id) {
    const std::string path = CorpusPath(id);
    auto r = path.empty() ? Expected<std::shared_ptr<const TiffReader>>(NotFound("missing")) : TiffReader::Open(path);
    if (!r.ok()) {
        state.SkipWithError("corpus file missing (fixtures index)");
        return;
    }
    const TiffReader& rd = **r;
    const uint32_t ifd = rd.MainImage();
    const IfdInfo& info = rd.Ifd(ifd);
    // A fixed set of 64 tiles (the page cache holds them after the first pass).
    std::mt19937 rng(42 + static_cast<uint32_t>(state.thread_index()));
    std::vector<uint32_t> picks;
    while (picks.size() < 64) {
        const uint32_t idx = static_cast<uint32_t>(rng() % info.blockCount);
        auto sparse = rd.IsSparse(ifd, idx);
        if (sparse.ok() && !*sparse) picks.push_back(idx);
    }
    const uint64_t size = info.DecodedBlockBytes(0);
    mem::AlignedBytes out = mem::MakeAlignedBytes(size);
    for (auto _ : state) {
        for (uint32_t idx : picks) {
            if (!rd.ReadBlockInto(ifd, idx, std::span<uint8_t>(out.get(), info.DecodedBlockBytes(idx))).ok()) {
                state.SkipWithError("ReadBlockInto failed");
                return;
            }
        }
    }
    state.SetBytesProcessed(static_cast<int64_t>(static_cast<uint64_t>(state.iterations()) * picks.size() * size));
    state.counters["blocks"] = benchmark::Counter(static_cast<double>(static_cast<uint64_t>(state.iterations()) * picks.size()), benchmark::Counter::kIsRate);
}

BENCHMARK_CAPTURE(BM_Open, R9_mmap, "R9", io::SourceKind::kMmap, false)->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_Open, R9_pread, "R9", io::SourceKind::kPread, false)->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_Open, R9_pread_index, "R9", io::SourceKind::kPread, true)->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_Open, R9_mmap_index, "R9", io::SourceKind::kMmap, true)->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_Open, R10_pread_index, "R10", io::SourceKind::kPread, true)->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_ReadBlock, R9, "R9")->Threads(1)->Threads(8)->UseRealTime()->Unit(benchmark::kMillisecond);

}  // namespace
