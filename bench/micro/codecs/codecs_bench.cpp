// SPDX-License-Identifier: MPL-2.0
//
// Block codec microbenchmarks on real tiles of the corpus (paths from the TIFF fixtures index
// written by tests/unit/tiff/data/gdal_reference.py, $DDBT_TIFF_FIXTURES, default
// /mnt/data/ddb-tilers-work/fixtures/tiff/index.tsv). Compressed tiles are read once into
// memory; an iteration decodes all tiles of a set. bytes_per_second is decoded output.
//
//   Lzw/R7       RGBA 8-bit, predictor 2, 512x512: an 8192 x 8192 window of R7 written by GDAL
//                as LZW (R8, the only LZW RGBA corpus file, has no tile data)  codec / + predictor
//   Lzw/R4       5 x UInt16, predictor 2, 512x512 (multispectral)
//   Lzw/R5       Float32, no predictor, 512x512 (DEM COG)
//   Deflate/R7   RGBA 8-bit, 512x512 (Lamb-Hill COG)
//   Zstd/R7      the same window as ZSTD level 9 (GDAL's default) + predictor 2 (no corpus file
//                uses ZSTD)
//   Jpeg/R6      YCbCr 4:2:0 512x512 with JPEGTables: full, 1/2, 1/4, 1/8, 256x256 crop
//   Libtiff/*    the same blocks through libtiff's TIFFReadEncodedTile (codec + predictor),
//                against our ReadBlockInto on the mapped file
//   Predictor*   the Highway kernels against the scalar reference
//
// Threads: /threads:1 and /threads:8 (UseRealTime: bytes_per_second is the total of all
// threads; divide by 8 for the per-thread rate).

#include <benchmark/benchmark.h>
#include <tiffio.h>
#include <zstd.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "cog/codecs/codecs.h"
#include "cog/codecs/jpeg.h"
#include "cog/codecs/predictor.h"
#include "cog/tiff/tiff_reader.h"
#include "core/mem/aligned.h"

namespace {

using namespace ddbt;
using namespace ddbt::cog;

std::map<std::string, std::string> CorpusPaths() {
    const char* e = std::getenv("DDBT_TIFF_FIXTURES");
    const std::string dir = e != nullptr ? e : "/mnt/data/ddb-tilers-work/fixtures/tiff";
    std::map<std::string, std::string> out;
    std::ifstream f(dir + "/index.tsv");
    std::string line;
    while (std::getline(f, line)) {
        const size_t a = line.find('\t');
        const size_t b = line.find('\t', a + 1);
        if (a != std::string::npos && b != std::string::npos) out[line.substr(0, a)] = line.substr(a + 1, b - a - 1);
    }
    return out;
}

struct TileSet {
    std::string path;
    std::shared_ptr<const tiff::TiffReader> reader;
    uint32_t ifd = 0;
    std::vector<uint32_t> index;
    std::vector<std::vector<uint8_t>> raw;
    uint64_t decodedBytes = 0;  // per tile
    bool ok = false;
};

// Up to `count` non-sparse tiles from the middle of the main image.
TileSet Load(const char* id, size_t count) {
    TileSet s;
    const auto paths = CorpusPaths();
    auto it = paths.find(id);
    if (it == paths.end()) return s;
    s.path = it->second;
    auto r = tiff::TiffReader::Open(s.path);
    if (!r.ok()) return s;
    s.reader = *r;
    s.ifd = s.reader->MainImage();
    const tiff::IfdInfo& info = s.reader->Ifd(s.ifd);
    s.decodedBytes = info.DecodedBlockBytes(0);
    const uint32_t n = info.blocksAcross * info.blocksDown;
    for (uint32_t k = 0; k < n && s.index.size() < count; ++k) {
        const uint32_t idx = (n / 2 + k * 7919u) % n;
        if (!info.tiled || info.BlockRows(idx) != info.blockHeight) continue;
        if (info.BlockColumn(idx) + 1 == info.blocksAcross || info.BlockRow(idx) + 1 == info.blocksDown) continue;
        auto raw = s.reader->ReadBlockRaw(s.ifd, idx);
        if (!raw.ok() || raw->IsSparse()) continue;
        s.index.push_back(idx);
        s.raw.emplace_back(raw->bytes.begin(), raw->bytes.end());
    }
    s.ok = !s.raw.empty();
    return s;
}

const TileSet& Set(const char* id) {
    static std::mutex m;
    static std::map<std::string, TileSet> sets;
    std::lock_guard<std::mutex> lock(m);
    auto it = sets.find(id);
    if (it == sets.end()) it = sets.emplace(id, Load(id, 32)).first;
    return it->second;
}

template <typename Fn>
void RunTiles(benchmark::State& state, const TileSet& s, Fn&& fn) {
    if (!s.ok) {
        state.SkipWithError("corpus file missing (fixtures index)");
        return;
    }
    mem::AlignedBytes out = mem::MakeAlignedBytes(s.decodedBytes);
    for (auto _ : state) {
        for (size_t k = 0; k < s.raw.size(); ++k) {
            if (!fn(k, std::span<uint8_t>(out.get(), s.decodedBytes))) {
                state.SkipWithError("decode failed");
                return;
            }
        }
        benchmark::ClobberMemory();
    }
    state.SetBytesProcessed(static_cast<int64_t>(static_cast<uint64_t>(state.iterations()) * s.raw.size() * s.decodedBytes));
    state.counters["tiles"] = benchmark::Counter(static_cast<double>(static_cast<uint64_t>(state.iterations()) * s.raw.size()), benchmark::Counter::kIsRate);
    uint64_t compressed = 0;
    for (const auto& r : s.raw) compressed += r.size();
    state.counters["ratio"] = static_cast<double>(s.decodedBytes * s.raw.size()) / static_cast<double>(compressed);
}

void BM_LzwCodec(benchmark::State& state, const char* id) {
    const TileSet& s = Set(id);
    RunTiles(state, s, [&](size_t k, std::span<uint8_t> out) { return codecs::DecodeLzw(s.raw[k], out).ok(); });
}

void BM_Block(benchmark::State& state, const char* id) {
    const TileSet& s = Set(id);
    RunTiles(state, s, [&](size_t k, std::span<uint8_t> out) {
        return s.reader->DecodeBlock(s.ifd, s.index[k], s.raw[k], out).ok();
    });
}

void BM_DeflateCodec(benchmark::State& state) {
    const TileSet& s = Set("R7");
    RunTiles(state, s, [&](size_t k, std::span<uint8_t> out) { return codecs::DecodeDeflate(s.raw[k], out).ok(); });
}

void BM_ZstdCodec(benchmark::State& state) {
    const TileSet& s = Set("bench_r7_zstd_pred2");
    RunTiles(state, s, [&](size_t k, std::span<uint8_t> out) { return codecs::DecodeZstd(s.raw[k], out).ok(); });
}

// JPEG: denom 1/2/4/8 scaled full tiles; crop = 256x256 window at (128, 128) of a full-size
// decode (the bytes counted are the window's).
void BM_Jpeg(benchmark::State& state, uint32_t denom, bool crop) {
    const TileSet& s = Set("R6");
    if (!s.ok) {
        state.SkipWithError("corpus file missing (fixtures index)");
        return;
    }
    const tiff::IfdInfo& info = s.reader->Ifd(s.ifd);
    codecs::JpegDecodeOptions o;
    o.tables = info.jpegTables;
    o.ycbcrToRgb = info.photometric == tiff::photometric::kYCbCr;
    o.scaleDenom = denom;
    const uint32_t w = crop ? 256 : codecs::JpegScaledSize(info.blockWidth, denom);
    const uint32_t h = crop ? 256 : codecs::JpegScaledSize(info.blockHeight, denom);
    const uint32_t x = crop ? 128 : 0, y = crop ? 128 : 0;
    std::vector<uint8_t> out(size_t(w) * h * 3);
    for (auto _ : state) {
        for (const auto& raw : s.raw) {
            if (!codecs::DecodeJpeg(raw, o, 3, x, y, w, h, out.data(), size_t(w) * 3).ok()) {
                state.SkipWithError("decode failed");
                return;
            }
        }
    }
    state.SetBytesProcessed(static_cast<int64_t>(static_cast<uint64_t>(state.iterations()) * s.raw.size() * out.size()));
    state.counters["tiles"] = benchmark::Counter(static_cast<double>(static_cast<uint64_t>(state.iterations()) * s.raw.size()), benchmark::Counter::kIsRate);
}

// libtiff TIFFReadEncodedTile vs our ReadBlockInto, both on the file (page cache warm).
void BM_Libtiff(benchmark::State& state, const char* id) {
    const TileSet& s = Set(id);
    if (!s.ok) {
        state.SkipWithError("corpus file missing (fixtures index)");
        return;
    }
    TIFFSetWarningHandler(nullptr);
    TIFF* t = TIFFOpen(s.path.c_str(), "r");
    if (t == nullptr) {
        state.SkipWithError("TIFFOpen failed");
        return;
    }
    // GDAL reads YCbCr JPEG as RGB (libtiff's default is the raw subsampled YCbCr).
    uint16_t photometric = 0;
    if (TIFFGetField(t, TIFFTAG_PHOTOMETRIC, &photometric) && photometric == PHOTOMETRIC_YCBCR) {
        TIFFSetField(t, TIFFTAG_JPEGCOLORMODE, JPEGCOLORMODE_RGB);
    }
    std::vector<uint8_t> out(s.decodedBytes);
    for (auto _ : state) {
        for (uint32_t idx : s.index) {
            if (TIFFReadEncodedTile(t, idx, out.data(), static_cast<tmsize_t>(out.size())) < 0) {
                state.SkipWithError("TIFFReadEncodedTile failed");
                break;
            }
        }
    }
    TIFFClose(t);
    state.SetBytesProcessed(static_cast<int64_t>(static_cast<uint64_t>(state.iterations()) * s.index.size() * s.decodedBytes));
}

void BM_OursReadBlock(benchmark::State& state, const char* id) {
    const TileSet& s = Set(id);
    if (!s.ok) {
        state.SkipWithError("corpus file missing (fixtures index)");
        return;
    }
    mem::AlignedBytes out = mem::MakeAlignedBytes(s.decodedBytes);
    for (auto _ : state) {
        for (uint32_t idx : s.index) {
            if (!s.reader->ReadBlockInto(s.ifd, idx, std::span<uint8_t>(out.get(), s.decodedBytes)).ok()) {
                state.SkipWithError("ReadBlockInto failed");
                return;
            }
        }
    }
    state.SetBytesProcessed(static_cast<int64_t>(static_cast<uint64_t>(state.iterations()) * s.index.size() * s.decodedBytes));
}

void BM_Predictor(benchmark::State& state, bool simd, bool fp, size_t spp, size_t bps) {
    const size_t width = 512, rows = 512, stride = width * spp * bps;
    mem::AlignedBytes buf = mem::MakeAlignedBytes(stride * rows);
    for (size_t i = 0; i < stride * rows; ++i) buf.get()[i] = static_cast<uint8_t>(i * 2654435761u >> 13);
    for (auto _ : state) {
        if (fp) {
            if (simd) codecs::UndoFloatingPointPredictor(buf.get(), rows, stride, width, spp, bps);
            else codecs::reference::UndoFloatingPointPredictor(buf.get(), rows, stride, width, spp, bps);
        } else {
            if (simd) codecs::UndoHorizontalPredictor(buf.get(), rows, stride, width, spp, bps);
            else codecs::reference::UndoHorizontalPredictor(buf.get(), rows, stride, width, spp, bps);
        }
        benchmark::ClobberMemory();
    }
    state.SetBytesProcessed(static_cast<int64_t>(static_cast<uint64_t>(state.iterations()) * stride * rows));
}

#define DDBT_CODEC_THREADS ->Threads(1)->Threads(8)->UseRealTime()->Unit(benchmark::kMillisecond)

BENCHMARK_CAPTURE(BM_LzwCodec, R7_rgba8_pred2, "bench_r7_lzw_pred2") DDBT_CODEC_THREADS;
BENCHMARK_CAPTURE(BM_LzwCodec, R4_u16x5, "R4") DDBT_CODEC_THREADS;
BENCHMARK_CAPTURE(BM_LzwCodec, R5_f32, "R5") DDBT_CODEC_THREADS;
BENCHMARK_CAPTURE(BM_Block, Lzw_pred2_R7_rgba8, "bench_r7_lzw_pred2") DDBT_CODEC_THREADS;
BENCHMARK_CAPTURE(BM_Block, Zstd_pred2_R7_rgba8, "bench_r7_zstd_pred2") DDBT_CODEC_THREADS;
BENCHMARK_CAPTURE(BM_Block, Lzw_pred2_R4_u16x5, "R4") DDBT_CODEC_THREADS;
BENCHMARK(BM_DeflateCodec) DDBT_CODEC_THREADS;
BENCHMARK(BM_ZstdCodec) DDBT_CODEC_THREADS;
BENCHMARK_CAPTURE(BM_Jpeg, R6_full, 1u, false) DDBT_CODEC_THREADS;
BENCHMARK_CAPTURE(BM_Jpeg, R6_scale_1_2, 2u, false) DDBT_CODEC_THREADS;
BENCHMARK_CAPTURE(BM_Jpeg, R6_scale_1_4, 4u, false) DDBT_CODEC_THREADS;
BENCHMARK_CAPTURE(BM_Jpeg, R6_scale_1_8, 8u, false) DDBT_CODEC_THREADS;
BENCHMARK_CAPTURE(BM_Jpeg, R6_crop_256, 1u, true) DDBT_CODEC_THREADS;
BENCHMARK_CAPTURE(BM_Libtiff, R7_lzw_pred2_rgba8, "bench_r7_lzw_pred2")->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_OursReadBlock, R7_lzw_pred2_rgba8, "bench_r7_lzw_pred2")->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Libtiff, R7_zstd_pred2_rgba8, "bench_r7_zstd_pred2")->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_OursReadBlock, R7_zstd_pred2_rgba8, "bench_r7_zstd_pred2")->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Libtiff, R4_lzw_pred2_u16x5, "R4")->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_OursReadBlock, R4_lzw_pred2_u16x5, "R4")->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Libtiff, R5_lzw_f32, "R5")->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_OursReadBlock, R5_lzw_f32, "R5")->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Libtiff, R7_deflate, "R7")->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_OursReadBlock, R7_deflate, "R7")->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Libtiff, R6_jpeg, "R6")->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_OursReadBlock, R6_jpeg, "R6")->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Predictor, H2_u8x4_simd, true, false, 4, 1);
BENCHMARK_CAPTURE(BM_Predictor, H2_u8x4_scalar, false, false, 4, 1);
BENCHMARK_CAPTURE(BM_Predictor, H2_u16x5_simd, true, false, 5, 2);
BENCHMARK_CAPTURE(BM_Predictor, H2_u16x5_scalar, false, false, 5, 2);
BENCHMARK_CAPTURE(BM_Predictor, H2_u8x1_simd, true, false, 1, 1);
BENCHMARK_CAPTURE(BM_Predictor, H2_u8x1_scalar, false, false, 1, 1);
BENCHMARK_CAPTURE(BM_Predictor, FP3_f32_simd, true, true, 1, 4);
BENCHMARK_CAPTURE(BM_Predictor, FP3_f32_scalar, false, true, 1, 4);

}  // namespace
