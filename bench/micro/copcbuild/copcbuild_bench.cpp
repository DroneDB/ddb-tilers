// SPDX-License-Identifier: MPL-2.0
//
// COPC builder kernels: Morton keys (BMI2 pdep vs the portable spread), the LOD winner kernel,
// the partition radix sort, a whole partition build (sort + LOD, no encoding), node encoding
// (laz-perf layered compressor), text number parsing and E57 page CRC-32C.
#include <benchmark/benchmark.h>

#include <algorithm>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "copc/build/lod.h"
#include "copc/build/morton.h"
#include "copc/build/node_encoder.h"
#include "copc/readers/e57_reader.h"
#include "copc/readers/text_util.h"
#include "core/util/endian.h"

namespace ddbt::copc::build {
namespace {

std::vector<uint32_t> Coords(size_t n, uint32_t seed) {
    std::mt19937 rng(seed);
    std::vector<uint32_t> c(3 * n);
    for (auto& v : c) v = rng() & (kKeyCells - 1);
    return c;
}

void BM_MortonEncode(benchmark::State& st) {
    const auto c = Coords(1 << 16, 1);
    for (auto _ : st) {
        uint64_t acc = 0;
        for (size_t i = 0; i < c.size(); i += 3) acc ^= MortonEncode(c[i], c[i + 1], c[i + 2]);
        benchmark::DoNotOptimize(acc);
    }
    st.SetItemsProcessed(static_cast<int64_t>(st.iterations()) * (1 << 16));
}
BENCHMARK(BM_MortonEncode);

void BM_MortonEncodePortable(benchmark::State& st) {
    const auto c = Coords(1 << 16, 1);
    for (auto _ : st) {
        uint64_t acc = 0;
        for (size_t i = 0; i < c.size(); i += 3) acc ^= MortonEncodePortable(c[i], c[i + 1], c[i + 2]);
        benchmark::DoNotOptimize(acc);
    }
    st.SetItemsProcessed(static_cast<int64_t>(st.iterations()) * (1 << 16));
}
BENCHMARK(BM_MortonEncodePortable);

/// Sorted keys of a surface-like cloud (z from x, y) inside one node.
std::vector<uint64_t> SurfaceKeys(size_t n, uint32_t seed) {
    std::mt19937 rng(seed);
    std::vector<uint64_t> k(n);
    for (auto& v : k) {
        const uint32_t x = rng() & (kKeyCells - 1), y = rng() & (kKeyCells - 1);
        const uint32_t z = static_cast<uint32_t>((uint64_t{x} + y) / 4 + (rng() & 1023)) & (kKeyCells - 1);
        v = MortonEncode(x, y, z);
    }
    std::sort(k.begin(), k.end());
    return k;
}

void BM_SelectWinners(benchmark::State& st) {
    const size_t n = static_cast<size_t>(st.range(0));
    const auto keys = SurfaceKeys(n, 2);
    std::vector<uint8_t> win(n);
    std::vector<uint32_t> scratch;
    for (auto _ : st) {
        std::fill(win.begin(), win.end(), uint8_t{0});
        SelectWinners(keys.data(), n, 0, 148, win.data(), scratch);
        benchmark::DoNotOptimize(win.data());
    }
    st.SetItemsProcessed(static_cast<int64_t>(st.iterations()) * static_cast<int64_t>(n));
}
BENCHMARK(BM_SelectWinners)->Arg(1 << 16)->Arg(1 << 20);

void BM_SortEntries(benchmark::State& st) {
    const size_t n = static_cast<size_t>(st.range(0));
    std::mt19937_64 rng(3);
    std::vector<uint8_t> recs(n * 36);
    for (auto& b : recs) b = static_cast<uint8_t>(rng());
    std::vector<SortEntry> base(n), e, scratch;
    for (size_t i = 0; i < n; ++i) base[i] = {rng() >> 1, recs.data() + i * 36};
    for (auto _ : st) {
        st.PauseTiming();
        e = base;
        st.ResumeTiming();
        SortEntries(e, scratch, 36, 6);
        benchmark::DoNotOptimize(e.data());
    }
    st.SetItemsProcessed(static_cast<int64_t>(st.iterations()) * static_cast<int64_t>(n));
}
BENCHMARK(BM_SortEntries)->Arg(1 << 20)->Arg(4 << 20)->Unit(benchmark::kMillisecond);

void BM_BuildPartition(benchmark::State& st) {
    const size_t n = static_cast<size_t>(st.range(0));
    const auto keys = SurfaceKeys(n, 4);
    std::vector<uint8_t> recs(n * 36);
    std::mt19937_64 rng(5);
    for (auto& b : recs) b = static_cast<uint8_t>(rng());
    LodParams p;
    p.leafLevel = LeafLevel(n, p.leafMax);
    uint64_t emitted = 0;
    EmitFn emit = [&](int, uint64_t, const uint8_t*, uint32_t c) -> Status {
        emitted += c;
        return Status();
    };
    for (auto _ : st) {
        st.PauseTiming();
        std::vector<SortEntry> e(n);
        for (size_t i = 0; i < n; ++i) e[i] = {keys[(i * 7919) % n], recs.data() + i * 36};
        st.ResumeTiming();
        NodePoints root;
        if (!BuildPartition(0, 0, -1, e, 36, p, emit, root).ok()) st.SkipWithError("build failed");
        benchmark::DoNotOptimize(root.keys.data());
    }
    st.SetItemsProcessed(static_cast<int64_t>(st.iterations()) * static_cast<int64_t>(n));
}
BENCHMARK(BM_BuildPartition)->Arg(1 << 20)->Arg(4 << 20)->Unit(benchmark::kMillisecond);

void BM_EncodeNode(benchmark::State& st) {
    const int format = static_cast<int>(st.range(0));
    const size_t n = static_cast<size_t>(st.range(1)), rs = format == 8 ? 38 : format == 7 ? 36 : 30;
    std::mt19937_64 rng(6);
    std::vector<uint8_t> recs(n * rs, 0);
    const auto keys = SurfaceKeys(n, 7);
    for (size_t i = 0; i < n; ++i) {
        uint32_t x, y, z;
        MortonDecode(keys[i], x, y, z);
        uint8_t* r = recs.data() + i * rs;
        StoreLE<int32_t>(r, static_cast<int32_t>(x >> 4));
        StoreLE<int32_t>(r + 4, static_cast<int32_t>(y >> 4));
        StoreLE<int32_t>(r + 8, static_cast<int32_t>(z >> 4));
        StoreLE<uint16_t>(r + 12, static_cast<uint16_t>(rng() & 0x3FF));
        r[14] = 0x11;
        if (format >= 7)
            for (int c = 0; c < 3; ++c) StoreLE<uint16_t>(r + 30 + 2 * c, static_cast<uint16_t>((rng() & 0xFF) * 257));
    }
    std::vector<uint8_t> out;
    for (auto _ : st) {
        if (!EncodeNode(format, 0, recs.data(), n, out).ok()) st.SkipWithError("encode failed");
        benchmark::DoNotOptimize(out.data());
    }
    st.SetItemsProcessed(static_cast<int64_t>(st.iterations()) * static_cast<int64_t>(n));
    st.counters["bytes/pt"] = static_cast<double>(out.size()) / static_cast<double>(n);
}
BENCHMARK(BM_EncodeNode)->Args({6, 50000})->Args({7, 50000})->Args({8, 50000})->Args({7, 100})->Args({7, 1000})
    ->Unit(benchmark::kMicrosecond);

void BM_TextParseLine(benchmark::State& st) {
    std::string text;
    std::mt19937 rng(8);
    for (int i = 0; i < 100000; ++i)
        text += std::to_string(274800 + rng() % 300) + "." + std::to_string(rng() % 1000) + " " +
                std::to_string(4603000 + rng() % 300) + "." + std::to_string(rng() % 1000) + " " +
                std::to_string(rng() % 40) + ".5 " + std::to_string(rng() % 256) + " " + std::to_string(rng() % 256) +
                " " + std::to_string(rng() % 256) + "\n";
    for (auto _ : st) {
        const char* p = text.data();
        const char* end = p + text.size();
        double acc = 0, v;
        while (p < end) {
            const char* eol = static_cast<const char*>(std::memchr(p, '\n', static_cast<size_t>(end - p)));
            while (readers::text::NextDouble(p, eol, ' ', v)) acc += v;
            p = eol + 1;
        }
        benchmark::DoNotOptimize(acc);
    }
    st.SetBytesProcessed(static_cast<int64_t>(st.iterations()) * static_cast<int64_t>(text.size()));
    st.SetItemsProcessed(static_cast<int64_t>(st.iterations()) * 100000);
}
BENCHMARK(BM_TextParseLine);

void BM_Crc32cPage(benchmark::State& st) {
    std::vector<uint8_t> page(1020);
    for (size_t i = 0; i < page.size(); ++i) page[i] = static_cast<uint8_t>(i * 31);
    for (auto _ : st) benchmark::DoNotOptimize(readers::Crc32c(page.data(), page.size()));
    st.SetBytesProcessed(static_cast<int64_t>(st.iterations()) * 1020);
}
BENCHMARK(BM_Crc32cPage);

}  // namespace
}  // namespace ddbt::copc::build
