// SPDX-License-Identifier: MPL-2.0
//
// COG builder microbenchmarks: block encoding per codec (LZW + predictor, DEFLATE, ZSTD, JPEG
// YCbCr q75, 1-bit masks), the LZW encoder alone, one overview tile (GDAL's cubic convolution,
// 1024 x 1024 -> 512 x 512, with and without a mask), and the exact statistics (Welford) per
// sample, and the warp kernel on one 512 x 512 output tile (GDAL's bilinear, an RGBA source
// slightly rotated and scaled, scalar reference vs SIMD). Synthetic tiles: a smooth gradient with
// seeded noise (an orthophoto-like entropy).

#include <benchmark/benchmark.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "cog/build/stats.h"
#include "cog/codecs/encoders.h"
#include "cog/encode/block_encoder.h"
#include "cog/pyramid/overview.h"
#include "cog/warp/bilinear.h"

namespace {

using namespace ddbt::cog;

std::vector<uint8_t> Image(uint32_t w, uint32_t h, uint32_t spp, uint32_t seed = 1) {
    std::mt19937 rng(seed);
    std::vector<uint8_t> px(static_cast<size_t>(w) * h * spp);
    for (uint32_t y = 0; y < h; ++y)
        for (uint32_t x = 0; x < w; ++x)
            for (uint32_t c = 0; c < spp; ++c) {
                const double v = 128 + 60 * std::sin(x * 0.02 + c) * std::cos(y * 0.015) + static_cast<double>(rng() % 24);
                px[(static_cast<size_t>(y) * w + x) * spp + c] = static_cast<uint8_t>(std::clamp(v, 0.0, 255.0));
            }
    return px;
}

void BM_CogBuild_EncodeBlock(benchmark::State& state) {
    const auto codec = static_cast<encode::Codec>(state.range(0));
    const uint32_t B = 512, spp = 3;
    const std::vector<uint8_t> px = Image(B, B, spp);
    encode::BlockFormat f{B, B, spp, tiff::SampleType::kU8, codec, codec == encode::Codec::kJpeg ? 1 : 2, 0, 75};
    encode::ByteBuffer out;
    size_t bytes = 0;
    for (auto _ : state) {
        out.clear();
        encode::EncodeBlock(f, px.data(), out);
        bytes = out.size();
        benchmark::DoNotOptimize(out.data());
    }
    state.SetBytesProcessed(static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(px.size()));
    state.counters["ratio"] = static_cast<double>(px.size()) / static_cast<double>(bytes);
    state.SetLabel(encode::CodecName(codec));
}
BENCHMARK(BM_CogBuild_EncodeBlock)
    ->Arg(static_cast<int>(encode::Codec::kLzw))
    ->Arg(static_cast<int>(encode::Codec::kDeflate))
    ->Arg(static_cast<int>(encode::Codec::kZstd))
    ->Arg(static_cast<int>(encode::Codec::kJpeg));

void BM_CogBuild_LzwEncodeOnly(benchmark::State& state) {
    const std::vector<uint8_t> px = Image(512, 512, 3);
    encode::ByteBuffer out;
    for (auto _ : state) {
        out.clear();
        codecs::EncodeLzw({px.data(), px.size()}, out);
        benchmark::DoNotOptimize(out.data());
    }
    state.SetBytesProcessed(static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(px.size()));
}
BENCHMARK(BM_CogBuild_LzwEncodeOnly);

void BM_CogBuild_MaskBlock(benchmark::State& state) {
    std::vector<uint8_t> m(512 * 512, 255);
    for (size_t i = 0; i < m.size(); ++i)
        if ((i % 512) < static_cast<size_t>(state.range(0))) m[i] = 0;
    encode::ByteBuffer out;
    for (auto _ : state) {
        out.clear();
        encode::EncodeMaskBlock(m.data(), 512, 512, out);
        benchmark::DoNotOptimize(out.data());
    }
    state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
}
BENCHMARK(BM_CogBuild_MaskBlock)->Arg(0)->Arg(200);

// Arg 0: with a mask (plus the mask's own level); arg 1: the scalar reference instead of the
// SIMD passes.
void BM_CogBuild_OverviewTile(benchmark::State& state) {
    const bool masked = state.range(0) != 0;
    const bool scalar = state.range(1) != 0;
    const int sw = 1031, sh = 1031;  // the taps of a 512 x 512 destination tile at a factor of 2
    const std::vector<uint8_t> px = Image(sw, sh, 3);
    std::vector<uint8_t> mask(static_cast<size_t>(sw) * sh, 255);
    for (int y = 0; y < sh; ++y)
        for (int x = 0; x < sw / 5; ++x) mask[static_cast<size_t>(y) * static_cast<size_t>(sw) + static_cast<size_t>(x)] = 0;
    const auto cols = pyramid::CubicTaps(2 * 2048, 2048, 512, 1024), rows = pyramid::CubicTaps(2 * 2048, 2048, 512, 1024);
    std::vector<uint8_t> dst(512 * 512 * 3), dstMask(512 * 512);
    pyramid::DownsampleSpec spec;
    spec.bands = 3;
    spec.mask = masked ? pyramid::MaskMode::kExternal : pyramid::MaskMode::kNone;
    const auto downsample = scalar ? &pyramid::reference::Downsample : &pyramid::Downsample;
    const auto downsampleMask = scalar ? &pyramid::reference::DownsampleMask : &pyramid::DownsampleMask;
    for (auto _ : state) {
        downsample(spec, px.data(), static_cast<size_t>(sw) * 3, cols.SrcBegin(), rows.SrcBegin(), sw, sh,
                   masked ? mask.data() : nullptr, cols, rows, 512, 512, dst.data(), 512 * 3);
        if (masked)
            downsampleMask(mask.data(), cols.SrcBegin(), rows.SrcBegin(), sw, sh, cols, rows, 512, 512, dstMask.data(),
                           512);
        benchmark::DoNotOptimize(dst.data());
    }
    state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * 512 * 512);
    state.SetLabel(std::string(masked ? "RGB + mask" : "RGB") + (scalar ? ", scalar reference" : ""));
}
BENCHMARK(BM_CogBuild_OverviewTile)->Args({0, 0})->Args({1, 0})->Args({0, 1})->Args({1, 1});

void BM_CogBuild_StatsWelford(benchmark::State& state) {
    const uint32_t W = 4096, H = 64;
    const std::vector<uint8_t> px = Image(W, H, 3);
    for (auto _ : state) {
        build::StatsAccumulator acc(3, tiff::SampleType::kU8, std::vector<build::StatsAccumulator::MaskKind>(3), -1,
                                    std::nullopt);
        acc.AccumulateRows(0, px.data(), static_cast<size_t>(W) * 3, W, H, nullptr);
        benchmark::DoNotOptimize(acc.Result(1));
    }
    state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * W * H);
}
BENCHMARK(BM_CogBuild_StatsWelford);

// The warp kernel on one 512 x 512 RGBA -> RGB + mask output tile (an orthophoto to Web
// Mercator: 2 degrees of rotation, a scale of 0.93). Arg 0: path (0 scalar, 1 SIMD); arg 1: the
// percentage of the source that is transparent (alpha 0, in a band on the left).
void BM_CogBuild_WarpTile(benchmark::State& state) {
    const auto path = state.range(0) == 0 ? warp::KernelPath::kScalar : warp::KernelPath::kSimd;
    const int transparent = static_cast<int>(state.range(1));
    const int W = 560, H = 560;
    warp::Window win;
    win.w = W;
    win.h = H;
    win.pb = 4;
    win.px = Image(W, H, 4);
    win.px.resize(win.px.size() + warp::kWindowPadding);
    win.density.resize(static_cast<size_t>(W) * H);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            uint8_t* const a = win.px.data() + (static_cast<size_t>(y) * W + static_cast<size_t>(x)) * 4 + 3;
            *a = x < W * transparent / 100 ? 0 : 255;
            win.density[static_cast<size_t>(y) * W + static_cast<size_t>(x)] = std::min(1.0f, static_cast<float>(*a) * (1.0f / 255.0f));
        }
    warp::IndexValidity(win);
    warp::KernelParams p;
    p.type = tiff::SampleType::kU8;
    p.sourceWidth = W;
    p.sourceHeight = H;
    p.bandOffsets = {0, 1, 2};
    p.outPixelBytes = 3;
    p.noMasks = false;
    p.dstDensityInit = 0;
    p.alphaOut = -1;
    const double c = std::cos(0.035) * 0.93, s = std::sin(0.035) * 0.93;
    std::vector<double> fx(512 * 512), fy(512 * 512);
    for (int y = 0; y < 512; ++y)
        for (int x = 0; x < 512; ++x) {
            fx[static_cast<size_t>(y) * 512 + static_cast<size_t>(x)] = 30.25 + (x + 0.5) * c - (y + 0.5) * s;
            fy[static_cast<size_t>(y) * 512 + static_cast<size_t>(x)] = 21.75 + (x + 0.5) * s + (y + 0.5) * c;
        }
    std::vector<uint8_t> out(512 * 512 * 3), mask(512 * 512);
    for (auto _ : state) {
        for (size_t y = 0; y < 512; ++y)
            warp::WarpRow(p, win, fx.data() + y * 512, fy.data() + y * 512, 512, out.data() + y * 512 * 3,
                          mask.data() + y * 512, path);
        benchmark::DoNotOptimize(out.data());
    }
    state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * 512 * 512);
}
BENCHMARK(BM_CogBuild_WarpTile)->Args({0, 0})->Args({1, 0})->Args({0, 40})->Args({1, 40});

}  // namespace
