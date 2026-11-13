// SPDX-License-Identifier: MPL-2.0
//
// Tile encoder microbenchmarks on real tiles.
//
// Fixtures: the RGBA tiles written by bench/micro/encode/make_fixtures.py (DroneDB 2.4.1 renders
// of a COG and a COPC), read from $DDBT_ENCODE_FIXTURES (default
// /mnt/data/ddb-tilers-work/fixtures/encode). Without them a small synthetic set is used.
//
// One iteration encodes every tile of a group (256 or 512 pixels): the "tiles" counter is
// tiles/s, "bytes_per_tile" the mean output size. Reference: libspng + zlib level 1 with all
// filters (the libpng / GDAL PNG driver path at ZLEVEL=1).

#include <benchmark/benchmark.h>
#include <spng.h>
#include <zlib.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "core/encode/kernels.h"
#include "core/encode/tile_encoder.h"

namespace {

using namespace ddbt::encode;

struct Tile {
    std::string name;
    uint32_t size = 0;
    std::vector<uint8_t> rgba;
    std::vector<uint8_t> packed;  // RGB when opaque, else RGBA (what DroneDB hands to GDAL)
    bool opaque = false;
};

std::string FixtureDir() {
    const char* env = std::getenv("DDBT_ENCODE_FIXTURES");
    return env != nullptr ? env : "/mnt/data/ddb-tilers-work/fixtures/encode";
}

std::vector<uint8_t> ReadFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

std::vector<Tile> LoadTiles() {
    std::vector<Tile> tiles;
    const std::string dir = FixtureDir();
    std::ifstream manifest(dir + "/manifest.tsv");
    std::string line;
    if (manifest && std::getline(manifest, line)) {
        while (std::getline(manifest, line)) {
            std::istringstream fields(line);
            Tile t;
            uint32_t height = 0;
            fields >> t.name >> t.size >> height;
            std::ifstream f(dir + "/" + t.name + ".rgba", std::ios::binary);
            t.rgba.resize(static_cast<size_t>(t.size) * t.size * 4);
            if (!f.read(reinterpret_cast<char*>(t.rgba.data()), static_cast<std::streamsize>(t.rgba.size()))) continue;
            tiles.push_back(std::move(t));
        }
    }
    if (tiles.empty()) {
        std::fprintf(stderr, "encode_bench: no fixtures in %s, using synthetic tiles\n", dir.c_str());
        std::mt19937 rng(1);
        for (const uint32_t size : {256u, 512u}) {
            for (int i = 0; i < 6; ++i) {
                Tile t;
                t.name = "synthetic";
                t.size = size;
                t.rgba.resize(static_cast<size_t>(size) * size * 4);
                for (uint32_t y = 0; y < size; ++y) {
                    for (uint32_t x = 0; x < size; ++x) {
                        uint8_t* p = t.rgba.data() + (static_cast<size_t>(y) * size + x) * 4;
                        p[0] = static_cast<uint8_t>(x + rng() % 16);
                        p[1] = static_cast<uint8_t>(y + rng() % 16);
                        p[2] = static_cast<uint8_t>((x * y) >> 8);
                        p[3] = (i % 2 == 0 || x > size / 4) ? 255 : 0;
                    }
                }
                tiles.push_back(std::move(t));
            }
        }
    }
    for (auto& t : tiles) {
        TileImage img = TileImage::Packed(t.rgba.data(), t.size, t.size, 4);
        ScanAlpha(img);
        t.opaque = img.allOpaque;
        if (t.opaque) {
            t.packed.resize(static_cast<size_t>(t.size) * t.size * 3);
            kernels::DropAlpha4(t.rgba.data(), static_cast<size_t>(t.size) * t.size, t.packed.data());
        } else {
            t.packed = t.rgba;
        }
    }
    return tiles;
}

const std::vector<Tile>& Tiles() {
    static const std::vector<Tile> tiles = LoadTiles();
    return tiles;
}

std::vector<const Tile*> Group(uint32_t size) {
    std::vector<const Tile*> g;
    for (const auto& t : Tiles()) {
        if (t.size == size) g.push_back(&t);
    }
    return g;
}

void Report(benchmark::State& state, size_t tiles, size_t bytes, size_t rawBytes) {
    const double n = static_cast<double>(tiles);
    state.counters["tiles"] = benchmark::Counter(n, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["bytes_per_tile"] = benchmark::Counter(static_cast<double>(bytes) / n);
    state.counters["ratio"] = benchmark::Counter(static_cast<double>(bytes) / static_cast<double>(rawBytes));
}

size_t RawBytes(const std::vector<const Tile*>& g) {
    size_t n = 0;
    for (const Tile* t : g) n += t->packed.size();
    return n;
}

// ---------------------------------------------------------------------------------------- ours

void BmEncode(benchmark::State& state, uint32_t size, EncodeOptions options) {
    const auto g = Group(size);
    if (g.empty()) {
        state.SkipWithMessage("no tiles");
        return;
    }
    ByteBuffer out;
    size_t bytes = 0;
    for (auto _ : state) {
        bytes = 0;
        for (const Tile* t : g) {
            EncodeTile(TileImage::Packed(t->rgba.data(), t->size, t->size, 4), options, out);
            bytes += out.size();
            benchmark::DoNotOptimize(out.data());
        }
    }
    Report(state, g.size(), bytes, RawBytes(g));
}

// ----------------------------------------------------------------------------------- reference

// libspng + zlib, all filters, level 1: the libpng path of GDAL's PNG driver at ZLEVEL=1.
void BmSpng(benchmark::State& state, uint32_t size, int level) {
    const auto g = Group(size);
    if (g.empty()) {
        state.SkipWithMessage("no tiles");
        return;
    }
    size_t bytes = 0;
    for (auto _ : state) {
        bytes = 0;
        for (const Tile* t : g) {
            spng_ctx* ctx = spng_ctx_new(SPNG_CTX_ENCODER);
            spng_set_option(ctx, SPNG_ENCODE_TO_BUFFER, 1);
            spng_set_option(ctx, SPNG_IMG_COMPRESSION_LEVEL, level);
            spng_set_option(ctx, SPNG_FILTER_CHOICE, SPNG_FILTER_CHOICE_ALL);
            spng_ihdr ihdr{};
            ihdr.width = t->size;
            ihdr.height = t->size;
            ihdr.bit_depth = 8;
            ihdr.color_type = t->opaque ? SPNG_COLOR_TYPE_TRUECOLOR : SPNG_COLOR_TYPE_TRUECOLOR_ALPHA;
            spng_set_ihdr(ctx, &ihdr);
            if (spng_encode_image(ctx, t->packed.data(), t->packed.size(), SPNG_FMT_PNG, SPNG_ENCODE_FINALIZE) != 0) {
                state.SkipWithError("spng_encode_image failed");
                spng_ctx_free(ctx);
                return;
            }
            size_t len = 0;
            int err = 0;
            void* png = spng_get_png_buffer(ctx, &len, &err);
            bytes += len;
            std::free(png);
            spng_ctx_free(ctx);
        }
    }
    Report(state, g.size(), bytes, RawBytes(g));
}

// Our filtering (adaptive) with zlib's deflate at `level` instead of libdeflate: isolates what
// libdeflate brings.
void BmOursFilterZlib(benchmark::State& state, uint32_t size, int level) {
    const auto g = Group(size);
    if (g.empty()) {
        state.SkipWithMessage("no tiles");
        return;
    }
    std::vector<uint8_t> filtered, scratch, z;
    size_t bytes = 0;
    for (auto _ : state) {
        bytes = 0;
        for (const Tile* t : g) {
            const uint32_t ch = t->opaque ? 3 : 4;
            const size_t n = static_cast<size_t>(t->size) * ch;
            filtered.resize((n + 1) * t->size);
            scratch.resize(3 * n);
            kernels::PngFilterJob job;
            job.src = t->rgba.data();
            job.srcStride = static_cast<size_t>(t->size) * 4;
            job.width = job.height = t->size;
            job.srcChannels = 4;
            job.dstChannels = ch;
            job.filter = kernels::PngRowFilter::kAdaptive;
            job.out = filtered.data();
            job.scratch = scratch.data();
            kernels::PngFilterImage(job);
            uLongf zlen = compressBound(static_cast<uLong>(filtered.size()));
            z.resize(zlen);
            compress2(z.data(), &zlen, filtered.data(), static_cast<uLong>(filtered.size()), level);
            bytes += zlen + 57;  // + signature, IHDR, IDAT header and CRC, IEND
        }
    }
    Report(state, g.size(), bytes, RawBytes(g));
}

// Kernels alone
void BmScanAlpha(benchmark::State& state, uint32_t size) {
    const auto g = Group(size);
    if (g.empty()) {
        state.SkipWithMessage("no tiles");
        return;
    }
    for (auto _ : state) {
        for (const Tile* t : g) {
            TileImage img = TileImage::Packed(t->rgba.data(), t->size, t->size, 4);
            ScanAlpha(img);
            benchmark::DoNotOptimize(img.allOpaque);
        }
    }
    state.counters["tiles"] = benchmark::Counter(static_cast<double>(g.size()), benchmark::Counter::kIsIterationInvariantRate);
}

void BmFilterOnly(benchmark::State& state, uint32_t size, kernels::PngRowFilter filter) {
    const auto g = Group(size);
    if (g.empty()) {
        state.SkipWithMessage("no tiles");
        return;
    }
    std::vector<uint8_t> filtered, scratch;
    for (auto _ : state) {
        for (const Tile* t : g) {
            const uint32_t ch = t->opaque ? 3 : 4;
            const size_t n = static_cast<size_t>(t->size) * ch;
            filtered.resize((n + 1) * t->size);
            scratch.resize(3 * n);
            kernels::PngFilterJob job;
            job.src = t->rgba.data();
            job.srcStride = static_cast<size_t>(t->size) * 4;
            job.width = job.height = t->size;
            job.srcChannels = 4;
            job.dstChannels = ch;
            job.filter = filter;
            job.out = filtered.data();
            job.scratch = scratch.data();
            kernels::PngFilterImage(job);
            benchmark::DoNotOptimize(filtered.data());
        }
    }
    state.counters["tiles"] = benchmark::Counter(static_cast<double>(g.size()), benchmark::Counter::kIsIterationInvariantRate);
}

// Not a timing: our bytes against the bytes of DroneDB 2.4.1's GDAL for the same tiles
// (<name>.gdal.<format>, written by gdal_reference.py), and our PNGs decoded by libspng against
// the source pixels. Counters: tiles identical / lossless per format.
void CheckAgainstGdal(benchmark::State& state) {
    const std::string dir = FixtureDir();
    size_t tiles = 0, png = 0, jpeg = 0, webp[3] = {0, 0, 0}, missing = 0;
    for (auto _ : state) {
        for (const auto& t : Tiles()) {
            if (t.name == "synthetic") continue;
            ++tiles;
            const TileImage img = TileImage::Packed(t.rgba.data(), t.size, t.size, 4);
            EncodeOptions o;
            ByteBuffer out;
            EncodeTile(img, o, out);
            spng_ctx* ctx = spng_ctx_new(0);
            spng_set_png_buffer(ctx, out.data(), out.size());
            std::vector<uint8_t> decoded(t.rgba.size());
            if (spng_decode_image(ctx, decoded.data(), decoded.size(), SPNG_FMT_RGBA8, SPNG_DECODE_TRNS) == 0 &&
                decoded == t.rgba)
                ++png;
            spng_ctx_free(ctx);
            auto same = [&](const std::string& suffix) {
                const auto ref = ReadFile(dir + "/" + t.name + ".gdal." + suffix);
                if (ref.empty()) ++missing;
                return !ref.empty() && ref.size() == out.size() && std::memcmp(ref.data(), out.data(), ref.size()) == 0;
            };
            o.format = TileFormat::kJpeg;
            EncodeTile(img, o, out);
            if (same("jpeg")) ++jpeg;
            for (size_t m = 0; m < 3; ++m) {
                o.format = TileFormat::kWebp;
                o.webp.method = static_cast<int>(m);
                EncodeTile(img, o, out);
                if (same("webp-m" + std::to_string(m))) ++webp[m];
            }
        }
    }
    state.counters["tiles"] = static_cast<double>(tiles);
    state.counters["png_lossless"] = static_cast<double>(png);
    state.counters["jpeg_same_as_gdal"] = static_cast<double>(jpeg);
    state.counters["webp_m0_same"] = static_cast<double>(webp[0]);
    state.counters["webp_m1_same"] = static_cast<double>(webp[1]);
    state.counters["webp_m2_same"] = static_cast<double>(webp[2]);
    state.counters["gdal_files_missing"] = static_cast<double>(missing);
}

const char* FilterName(PngRowFilter f) {
    switch (f) {
        case PngRowFilter::kNone: return "none";
        case PngRowFilter::kSub: return "sub";
        case PngRowFilter::kUp: return "up";
        case PngRowFilter::kAverage: return "avg";
        case PngRowFilter::kPaeth: return "paeth";
        case PngRowFilter::kAdaptive: return "adaptive";
    }
    return "?";
}

int RegisterAll() {
    benchmark::RegisterBenchmark("Check/same_as_gdal", CheckAgainstGdal)->Iterations(1);
    const PngRowFilter filters[] = {PngRowFilter::kNone,  PngRowFilter::kSub,      PngRowFilter::kUp,
                                    PngRowFilter::kAverage, PngRowFilter::kPaeth, PngRowFilter::kAdaptive};
    for (const uint32_t size : {256u, 512u}) {
        const std::string s = std::to_string(size);
        benchmark::RegisterBenchmark(("Png/spng_zlib/L1/" + s).c_str(), BmSpng, size, 1);
        benchmark::RegisterBenchmark(("Png/ours_filter_zlib/L1/" + s).c_str(), BmOursFilterZlib, size, 1);
        for (const auto f : filters) {
            for (const int level : {1, 2, 3, 4, 6}) {
                if (level != 1 && f != PngRowFilter::kAdaptive && f != PngRowFilter::kPaeth && f != PngRowFilter::kUp)
                    continue;
                EncodeOptions o;
                o.png.filter = f;
                o.png.level = level;
                benchmark::RegisterBenchmark(("Png/ours/" + std::string(FilterName(f)) + "/L" + std::to_string(level) +
                                              "/" + s)
                                                 .c_str(),
                                             BmEncode, size, o);
            }
        }
        {
            EncodeOptions o;
            o.format = TileFormat::kJpeg;
            benchmark::RegisterBenchmark(("Jpeg/ours/q75_optimized/" + s).c_str(), BmEncode, size, o);
            o.jpeg.optimizeHuffman = false;
            benchmark::RegisterBenchmark(("Jpeg/ours/q75_plain/" + s).c_str(), BmEncode, size, o);
            o.jpeg.optimizeHuffman = true;
            o.jpeg.fastDct = true;
            benchmark::RegisterBenchmark(("Jpeg/ours/q75_optimized_fastdct/" + s).c_str(), BmEncode, size, o);
        }
        for (const int method : {0, 1, 2}) {
            EncodeOptions o;
            o.format = TileFormat::kWebp;
            o.webp.method = method;
            benchmark::RegisterBenchmark(("Webp/ours/q80_m" + std::to_string(method) + "/" + s).c_str(), BmEncode, size,
                                         o);
        }
        benchmark::RegisterBenchmark(("Kernel/scan_alpha/" + s).c_str(), BmScanAlpha, size);
        for (const auto f : {PngRowFilter::kUp, PngRowFilter::kPaeth, PngRowFilter::kAdaptive}) {
            benchmark::RegisterBenchmark(("Kernel/png_filter/" + std::string(FilterName(f)) + "/" + s).c_str(),
                                         BmFilterOnly, size, f);
        }
    }
    return 0;
}

const int kRegistered = RegisterAll();

}  // namespace
