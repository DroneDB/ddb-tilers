// SPDX-License-Identifier: MPL-2.0
//
// The COG tiler's Highway kernels against their scalar references on every target the CPU
// supports (sizes around the vector widths, unaligned buffers, NaN, infinities, signed zeros,
// subnormals, values a few ulps from the NoData value, ranges the vector stretch must leave to
// the scalar one), and the gathers that use them (GatherBands, GatherAlpha) against the planned
// pixels of synthetic rasters of every layout they take.
#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "cog/tiler/cog_source.h"
#include "cog/tiler/kernels.h"
#include "cog/tiler/raster_io.h"
#include "cog/tiler/stretch.h"
#include "core/simd/simd.h"
#include "synth_raster.h"

namespace fs = std::filesystem;
using namespace ddbt::cog::tiler;
using namespace ddbt::cog::tiler::testing;
namespace k = ddbt::cog::tiler::kernels;

namespace {

struct ForcedTargetGuard {
    ~ForcedTargetGuard() { ddbt::simd::ForceTargetForTest(0); }
};

// Runs fn once per Highway target the CPU supports, that target forced.
template <typename Fn>
void OnEveryTarget(Fn&& fn) {
    ForcedTargetGuard guard;
    for (const int64_t bit : ddbt::simd::SupportedTargetBits()) {
        ddbt::simd::ForceTargetForTest(bit);
        SCOPED_TRACE(ddbt::simd::TargetNameOf(bit));
        fn();
    }
}

// Sizes around every vector width (up to 64 bytes / 8 doubles per vector, 4 vectors per step).
const size_t kSizes[] = {0, 1, 2, 3, 7, 8, 9, 15, 16, 17, 31, 32, 33, 63, 64, 65, 127, 255, 256, 257, 1000};

template <typename T>
std::vector<uint8_t> AsBytes(const std::vector<T>& v, size_t shift = 0) {
    // `shift` bytes in front: the kernels must not need aligned buffers.
    std::vector<uint8_t> b(shift + v.size() * sizeof(T));
    if (!v.empty()) std::memcpy(b.data() + shift, v.data(), v.size() * sizeof(T));
    return b;
}

// Floats that stress the comparisons: the NoData value and its neighbours by 1 to 3 ulps,
// specials, the Byte clamp edges and random values of every magnitude.
std::vector<float> TestFloats(size_t n, float nodata, uint32_t seed) {
    std::mt19937 rng(seed);
    std::vector<float> special = {0.0f, -0.0f, 1.0f, -1.0f, 0.5f, 0.49999997f, 254.5f, 254.49998f, 255.0f, 255.5f,
                                  256.0f, -32767.0f, 1e30f, -1e30f, std::numeric_limits<float>::max(),
                                  -std::numeric_limits<float>::max(), std::numeric_limits<float>::min(),
                                  std::numeric_limits<float>::denorm_min(),
                                  -std::numeric_limits<float>::denorm_min(),
                                  std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
                                  std::numeric_limits<float>::quiet_NaN(), -std::numeric_limits<float>::quiet_NaN()};
    float up = nodata, down = nodata;
    for (int i = 0; i < 3; ++i) {
        up = std::nextafter(up, std::numeric_limits<float>::infinity());
        down = std::nextafter(down, -std::numeric_limits<float>::infinity());
        special.push_back(up);
        special.push_back(down);
    }
    special.push_back(nodata);
    std::vector<float> v(n);
    for (size_t i = 0; i < n; ++i) {
        switch (rng() % 4) {
            case 0: v[i] = special[rng() % special.size()]; break;
            case 1: v[i] = nodata; break;
            case 2: v[i] = std::uniform_real_distribution<float>(-300.0f, 300.0f)(rng); break;
            default: {
                uint32_t bits = static_cast<uint32_t>(rng());
                std::memcpy(&v[i], &bits, 4);
            }
        }
    }
    return v;
}

}  // namespace

// ------------------------------------------------------------------------------------- kernels

TEST(CogTilerKernels, NodataMaskMatchesAreRealEqualOnEveryTarget) {
    const float nodatas[] = {-32767.0f, 0.0f, -0.0f, 1.0f, -9999.0f, 1e-30f, 3.4e38f,
                             std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
                             std::numeric_limits<float>::denorm_min()};
    OnEveryTarget([&] {
        for (const float nd : nodatas)
            for (const size_t n : kSizes)
                for (const size_t shift : {size_t{0}, size_t{1}, size_t{3}}) {
                    const std::vector<float> v = TestFloats(n, nd, static_cast<uint32_t>(n * 7 + shift));
                    const std::vector<uint8_t> b = AsBytes(v, shift);
                    std::vector<uint8_t> out(n, 7);
                    k::NodataMaskF32(b.data() + shift, n, nd, out.data());
                    for (size_t i = 0; i < n; ++i)
                        ASSERT_EQ(out[i], k::AreRealEqual(v[i], nd) ? 0 : 255) << "value " << v[i] << " nodata " << nd;
                }
    });
}

TEST(CogTilerKernels, FloatToByteMatchesGdalCopyWordsOnEveryTarget) {
    // GDALCopyWords Float32 -> Byte: NaN and v <= 0 give 0, v >= 255 gives 255, else v + 0.5
    // truncated (in double).
    const auto ref = [](float v) -> uint8_t {
        if (std::isnan(v) || v <= 0) return 0;
        if (v >= 255) return 255;
        return static_cast<uint8_t>(static_cast<double>(v) + 0.5);
    };
    OnEveryTarget([&] {
        for (const size_t n : kSizes)
            for (const size_t shift : {size_t{0}, size_t{2}}) {
                const std::vector<float> v = TestFloats(n, 254.5f, static_cast<uint32_t>(n + 99 * shift));
                const std::vector<uint8_t> b = AsBytes(v, shift);
                std::vector<uint8_t> out(n, 7);
                k::FloatToByte(b.data() + shift, n, out.data());
                for (size_t i = 0; i < n; ++i) ASSERT_EQ(out[i], ref(v[i])) << "value " << v[i];
            }
    });
}

TEST(CogTilerKernels, VectorStretchMatchesStretchToByteOnEveryTarget) {
    const double inf = std::numeric_limits<double>::infinity();
    const std::pair<double, double> ranges[] = {{0, 255}, {-32767, 1200.5}, {283.15, 313.15}, {-1, 1},
                                                {1e-300, 2e-300}, {0, 1e-310}, {-1e300, 1e300}, {-1e306, 1e306},
                                                {-inf, 5}, {3, inf}, {5, 5}, {6, 5}};
    OnEveryTarget([&] {
        for (const auto& [lo, hi] : ranges) {
            for (const size_t n : kSizes) {
                const std::vector<float> f = TestFloats(n, static_cast<float>(lo), static_cast<uint32_t>(n + 5));
                std::vector<double> d(n);
                std::mt19937 rng(static_cast<uint32_t>(n));
                for (size_t i = 0; i < n; ++i)
                    d[i] = (i % 3 == 0) ? static_cast<double>(f[i])
                                        : lo + (hi - lo) * std::uniform_real_distribution<double>(-0.2, 1.2)(rng);
                const std::vector<uint8_t> fb = AsBytes(f, 1), db = AsBytes(d, 1);
                std::vector<uint8_t> out(n);
                // The vector kernels only when the range allows them; StretchSamples always.
                if (k::StretchVectorizable(lo, hi)) {
                    k::StretchF32(fb.data() + 1, n, lo, hi, out.data());
                    for (size_t i = 0; i < n; ++i)
                        ASSERT_EQ(out[i], StretchToByte(static_cast<double>(f[i]), lo, hi)) << f[i] << " in " << lo << ".." << hi;
                    k::StretchF64(db.data() + 1, n, lo, hi, out.data());
                    for (size_t i = 0; i < n; ++i)
                        ASSERT_EQ(out[i], StretchToByte(d[i], lo, hi)) << d[i] << " in " << lo << ".." << hi;
                }
                // Outside those ranges StretchToByte itself converts an infinite or NaN quotient (no
                // defined byte to compare with); the tiler's ranges are finite with hi > lo.
                if (!k::StretchVectorizable(lo, hi)) continue;
                StretchSamples(PixelType::kF32, fb.data() + 1, n, lo, hi, false, out.data(), 1);
                for (size_t i = 0; i < n; ++i) ASSERT_EQ(out[i], StretchToByte(static_cast<double>(f[i]), lo, hi));
                for (const bool viaFloat : {false, true}) {
                    StretchSamples(PixelType::kF64, db.data() + 1, n, lo, hi, viaFloat, out.data(), 1);
                    for (size_t i = 0; i < n; ++i) {
                        const double v = viaFloat ? static_cast<double>(static_cast<float>(d[i])) : d[i];
                        ASSERT_EQ(out[i], StretchToByte(v, lo, hi)) << d[i] << " viaFloat " << viaFloat;
                    }
                }
            }
        }
    });
    EXPECT_TRUE(k::StretchVectorizable(0, 1));
    EXPECT_FALSE(k::StretchVectorizable(1, 1));
    EXPECT_FALSE(k::StretchVectorizable(2, 1));
    EXPECT_FALSE(k::StretchVectorizable(-inf, 1));
    EXPECT_FALSE(k::StretchVectorizable(-1e306, 1e306));  // 255 * (hi - lo) overflows
    EXPECT_FALSE(k::StretchVectorizable(0, std::numeric_limits<double>::quiet_NaN()));
}

TEST(CogTilerKernels, StretchSamplesOfEveryTypeMatchesTheSampleByStretch) {
    // Every native type through StretchSamples (LUTs, vector kernels, typed loops) against
    // StretchToByte of SampleToDouble, with and without the Float32 read of readSelectedBands.
    std::mt19937 rng(3);
    const PixelType types[] = {PixelType::kU8, PixelType::kI8, PixelType::kU16, PixelType::kI16,
                               PixelType::kU32, PixelType::kI32, PixelType::kF32, PixelType::kF64};
    OnEveryTarget([&] {
        for (const PixelType t : types) {
            const size_t ss = PixelTypeSize(t);
            for (const size_t n : {size_t{1}, size_t{17}, size_t{256}, size_t{1001}}) {
                std::vector<uint8_t> src(n * ss);
                for (auto& b : src) b = static_cast<uint8_t>(rng());
                if (t == PixelType::kF32 || t == PixelType::kF64)  // finite values in a useful range
                    for (size_t i = 0; i < n; ++i) {
                        const double v = std::uniform_real_distribution<double>(-500, 70000)(rng);
                        if (t == PixelType::kF32) {
                            const float f = static_cast<float>(v);
                            std::memcpy(src.data() + i * 4, &f, 4);
                        } else {
                            std::memcpy(src.data() + i * 8, &v, 8);
                        }
                    }
                for (const auto& [lo, hi] : {std::pair{0.0, 255.0}, std::pair{-100.5, 60000.25}, std::pair{1e9, 3e9}}) {
                    for (const bool viaFloat : {false, true}) {
                        std::vector<uint8_t> out(n);
                        StretchSamples(t, src.data(), n, lo, hi, viaFloat, out.data(), 1);
                        for (size_t i = 0; i < n; ++i) {
                            double v = SampleToDouble(t, src.data() + i * ss);
                            if (viaFloat) v = static_cast<double>(static_cast<float>(v));
                            ASSERT_EQ(out[i], StretchToByte(v, lo, hi)) << PixelTypeName(t) << " " << v;
                        }
                    }
                }
            }
        }
    });
}

TEST(CogTilerKernels, InterleaveAndExtractMatchScalarOnEveryTarget) {
    std::mt19937 rng(17);
    OnEveryTarget([&] {
        for (const size_t n : kSizes) {
            // Interleave 1 to 4 planes.
            for (size_t ch = 1; ch <= 4; ++ch) {
                std::vector<std::vector<uint8_t>> planes(ch, std::vector<uint8_t>(n + 1));
                const uint8_t* ptrs[4];
                for (size_t c = 0; c < ch; ++c) {
                    for (auto& b : planes[c]) b = static_cast<uint8_t>(rng());
                    ptrs[c] = planes[c].data() + 1;
                }
                std::vector<uint8_t> out(n * ch), ref(n * ch);
                k::InterleavePlanes(ptrs, ch, n, out.data());
                k::InterleavePlanesScalar(ptrs, ch, n, ref.data());
                ASSERT_EQ(out, ref) << "channels " << ch << " n " << n;
            }
            // Extract samples of 1, 2, 4 or 8 bytes from 1 to 5 samples per pixel, any choice
            // (repeated and reordered indices too).
            for (const size_t sb : {size_t{1}, size_t{2}, size_t{4}, size_t{8}}) {
                for (size_t spp = 1; spp <= 5; ++spp) {
                    std::vector<uint8_t> px(1 + n * spp * sb);
                    for (auto& b : px) b = static_cast<uint8_t>(rng());
                    std::vector<std::vector<int>> choices = {{0}, {static_cast<int>(spp - 1)}};
                    std::vector<int> all, reversed;
                    for (size_t s = 0; s < spp; ++s) {
                        all.push_back(static_cast<int>(s));
                        reversed.insert(reversed.begin(), static_cast<int>(s));
                    }
                    choices.push_back(all);
                    choices.push_back(reversed);
                    choices.push_back({0, 0, static_cast<int>(spp / 2)});
                    for (const auto& idx : choices) {
                        std::vector<std::vector<uint8_t>> got(idx.size(), std::vector<uint8_t>(n * sb + 1, 0xAB));
                        std::vector<std::vector<uint8_t>> want(idx.size(), std::vector<uint8_t>(n * sb + 1, 0xAB));
                        std::vector<uint8_t*> g, w;
                        for (size_t b = 0; b < idx.size(); ++b) {
                            g.push_back(got[b].data() + 1);  // unaligned planes
                            w.push_back(want[b].data() + 1);
                        }
                        k::ExtractSamples(px.data() + 1, sb, spp, n, idx.data(), idx.size(), g.data());
                        k::ExtractSamplesScalar(px.data() + 1, sb, spp, n, idx.data(), idx.size(), w.data());
                        ASSERT_EQ(got, want) << "sample bytes " << sb << " spp " << spp << " n " << n;
                    }
                }
            }
        }
    });
}

TEST(CogTilerKernels, PickSamplesMatchesScalarOnEveryTarget) {
    // Non-decreasing columns with steps of 0 to 4 (upsampling, downsampling by less and more
    // than 2), runs ending at the last word of the row (the vector path must not read past it).
    std::mt19937 rng(23);
    OnEveryTarget([&] {
        for (const size_t n : kSizes) {
            for (const int maxStep : {1, 2, 3, 4}) {
                std::vector<uint32_t> x(n);
                uint32_t c = static_cast<uint32_t>(rng() % 5);
                for (size_t i = 0; i < n; ++i) {
                    x[i] = c;
                    c += static_cast<uint32_t>(rng() % static_cast<uint32_t>(maxStep + 1));
                }
                for (const size_t slack : {size_t{0}, size_t{1}, size_t{40}}) {
                    const size_t rowWords = (n > 0 ? x[n - 1] + 1 : 1) + slack;
                    std::vector<uint8_t> row(1 + rowWords * 4);
                    for (auto& b : row) b = static_cast<uint8_t>(rng());
                    std::vector<uint8_t> got(n * 4 + 1, 0), want(n * 4 + 1, 0);
                    k::PickSamples32(row.data() + 1, rowWords, x.data(), n, got.data() + 1);
                    k::PickSamples32Scalar(row.data() + 1, x.data(), n, want.data() + 1);
                    ASSERT_EQ(got, want) << "n " << n << " step " << maxStep << " slack " << slack;
                }
            }
        }
    });
}

TEST(CogTilerKernels, AllBytesEqualFindsEveryDifferingByteOnEveryTarget) {
    OnEveryTarget([&] {
        for (const size_t n : kSizes) {
            for (const uint8_t v : {uint8_t{0}, uint8_t{255}}) {
                std::vector<uint8_t> b(n + 1, v);
                EXPECT_TRUE(k::AllBytesEqual(b.data() + 1, n, v));
                for (size_t i = 0; i < n; ++i) {
                    b[i + 1] = static_cast<uint8_t>(v ^ 1);
                    ASSERT_FALSE(k::AllBytesEqual(b.data() + 1, n, v)) << "n " << n << " at " << i;
                    b[i + 1] = v;
                }
            }
        }
    });
}

// ------------------------------------------------------------------------------------- gathers

namespace {

class GatherDir {
public:
    GatherDir() {
        const char* base = std::getenv("DDBT_TEST_TMP");
        const fs::path root = base ? fs::path(base) : fs::temp_directory_path();
        path_ = root / ("ddbt-cogkernels-" + std::to_string(std::random_device{}()) + "-" + std::to_string(counter_++));
        fs::create_directories(path_);
    }
    ~GatherDir() {
        std::error_code ec;
        fs::remove_all(path_, ec);
    }
    const fs::path& path() const { return path_; }

private:
    static inline std::atomic<int> counter_{0};
    fs::path path_;
};

// Sample of band b (0-based) at (x, y) of the synthetic rasters below: unique per pixel and
// band, exact in Float32; Byte rasters keep the low 8 bits.
float SynthValue(int b, uint32_t x, uint32_t y) { return static_cast<float>(x + y * 1024) + 0.25f * static_cast<float>(b); }
float ByteValue(int b, uint32_t x, uint32_t y) { return static_cast<float>((x * 7 + y * 13 + static_cast<uint32_t>(b) * 61) & 255u); }

// Column / row lists of a plan over a level of `size` pixels: offsets plus i * step (a read at
// the level's resolution, downsampled by less and more than 2, upsampled), and -1 at the ends
// (pixels outside the raster).
std::vector<int32_t> Axis(int count, int size, double first, double step, int outsideHead, int outsideTail) {
    std::vector<int32_t> v;
    for (int i = 0; i < count; ++i) {
        if (i < outsideHead || i >= count - outsideTail) {
            v.push_back(-1);
            continue;
        }
        const int c = static_cast<int>(first + (i - outsideHead) * step);
        v.push_back(std::min(c, size - 1));
    }
    return v;
}

std::vector<SamplePlan> Plans(int width, int height) {
    std::vector<SamplePlan> plans;
    struct Shape {
        int bufW, bufH;
        double x0, y0, sx, sy;
        int headX, tailX, headY, tailY;
    };
    const Shape shapes[] = {
        {256, 64, 0, 0, 1, 1, 0, 0, 0, 0},       // the level's resolution, from the first block
        {200, 33, 37, 5, 1, 1, 0, 0, 0, 0},      // across block columns, odd sizes
        {255, 40, 0, 3, 1.37, 1.9, 0, 0, 0, 0},  // downsampled by less than 2
        {97, 31, 2, 1, 2.6, 3.1, 0, 0, 0, 0},    // by more than 2
        {301, 45, 11, 7, 0.61, 0.5, 0, 0, 0, 0}, // upsampled (repeated columns and rows)
        {64, 64, 0, 0, 1, 1, 5, 9, 3, 2},        // pixels outside on every side
        {77, 19, width - 60.0, height - 15.0, 0.8, 0.8, 0, 0, 0, 0},  // up to the last column and row
        {1, 1, 13, 17, 1, 1, 0, 0, 0, 0},
    };
    for (const Shape& s : shapes) {
        SamplePlan p;
        p.bufW = s.bufW;
        p.bufH = s.bufH;
        p.cols = Axis(s.bufW, width, s.x0, s.sx, s.headX, s.tailX);
        p.rows = Axis(s.bufH, height, s.y0, s.sy, s.headY, s.tailY);
        plans.push_back(std::move(p));
    }
    return plans;
}

}  // namespace

TEST(CogTilerKernels, GatheredBandsAreThePlannedPixelsOnEveryTarget) {
    // Every layout the gathers take: pixel-interleaved Byte (1 to 4 samples, the Byte
    // deinterleave), Float32 of 1 sample (the lane permutation) and of 2 (the 32-bit
    // deinterleave), band-interleaved Byte and Float32; blocks of 64 or 48 x 48 (runs across block
    // columns, a partial last block), any band choice.
    struct Layout {
        int bands, bytes;
        bool planar;
        uint32_t tile;
    };
    const Layout layouts[] = {{1, 1, false, 64}, {3, 1, false, 64}, {4, 1, false, 48}, {2, 1, false, 64},
                              {1, 4, false, 64}, {2, 4, false, 48}, {3, 4, false, 64}, {3, 1, true, 64},
                              {2, 4, true, 48}};
    GatherDir dir;
    const int width = 300, height = 200;
    int fileNo = 0;
    for (const Layout& l : layouts) {
        SynthSpec spec;
        spec.width = width;
        spec.height = height;
        spec.tile = l.tile;
        spec.bands = l.bands;
        spec.bytes = l.bytes;
        spec.planar = l.planar;
        spec.value = l.bytes == 1 ? ByteValue : SynthValue;
        const fs::path path = dir.path() / ("layout" + std::to_string(fileNo++) + ".tif");
        WriteSynth(path, spec);
        const auto src = CogSource::Open(path.string());
        const RasterView view(*src);
        std::vector<std::vector<int>> choices = {{1}, {l.bands}};
        std::vector<int> all;
        for (int b = 1; b <= l.bands; ++b) all.push_back(b);
        choices.push_back(all);
        choices.push_back({l.bands, 1, 1});
        const size_t ss = static_cast<size_t>(l.bytes);
        OnEveryTarget([&] {
            for (const SamplePlan& p : Plans(width, height)) {
                const size_t n = static_cast<size_t>(p.bufW) * static_cast<size_t>(p.bufH);
                for (const auto& bands : choices) {
                    std::vector<uint8_t> out(n * ss * bands.size(), 0xCD);
                    GatherBands(view, p, bands, out.data(), n * ss);
                    for (size_t b = 0; b < bands.size(); ++b)
                        for (int j = 0; j < p.bufH; ++j)
                            for (int i = 0; i < p.bufW; ++i) {
                                const int32_t c = p.cols[static_cast<size_t>(i)], r = p.rows[static_cast<size_t>(j)];
                                const size_t at = static_cast<size_t>(j) * static_cast<size_t>(p.bufW) + static_cast<size_t>(i);
                                const uint8_t* got = out.data() + b * n * ss + at * ss;
                                float want = 0;
                                if (c >= 0 && r >= 0)
                                    want = spec.value(bands[b] - 1, static_cast<uint32_t>(c), static_cast<uint32_t>(r));
                                if (l.bytes == 1) {
                                    ASSERT_EQ(*got, static_cast<uint8_t>(want)) << "band " << bands[b] << " at " << i << "," << j;
                                } else {
                                    float v;
                                    std::memcpy(&v, got, 4);
                                    ASSERT_EQ(v, want) << "band " << bands[b] << " at " << i << "," << j;
                                }
                            }
                }
            }
        });
    }
}

TEST(CogTilerKernels, NodataAlphaClearsPixelsWithoutASampleOnEveryTarget) {
    // GatherAlpha of a Float32 band with a NoData value: 0 at the NoData value (and within 2 ulps
    // of it), 255 elsewhere (NaN included), 0 for every pixel of a row or column without a sample.
    GatherDir dir;
    SynthSpec spec;
    spec.width = 300;
    spec.height = 200;
    spec.tile = 64;
    spec.nodata = "-9999";
    const float near = std::nextafter(-9999.0f, 0.0f);
    spec.value = [near](int, uint32_t x, uint32_t y) {
        switch ((x * 3 + y * 5) % 7) {
            case 0: return -9999.0f;
            case 1: return near;
            case 2: return std::numeric_limits<float>::quiet_NaN();
            default: return static_cast<float>(x) - static_cast<float>(y);
        }
    };
    const fs::path path = dir.path() / "nodata.tif";
    WriteSynth(path, spec);
    const auto src = CogSource::Open(path.string());
    const RasterView view(*src);
    OnEveryTarget([&] {
        for (const SamplePlan& p : Plans(static_cast<int>(spec.width), static_cast<int>(spec.height))) {
            const size_t n = static_cast<size_t>(p.bufW) * static_cast<size_t>(p.bufH);
            std::vector<uint8_t> alpha(n, 0x5A);
            GatherAlpha(view, p, alpha.data());
            for (int j = 0; j < p.bufH; ++j)
                for (int i = 0; i < p.bufW; ++i) {
                    const int32_t c = p.cols[static_cast<size_t>(i)], r = p.rows[static_cast<size_t>(j)];
                    uint8_t want = 0;
                    if (c >= 0 && r >= 0)
                        want = k::AreRealEqual(spec.value(0, static_cast<uint32_t>(c), static_cast<uint32_t>(r)), -9999.0f)
                                   ? 0
                                   : 255;
                    ASSERT_EQ(alpha[static_cast<size_t>(j) * static_cast<size_t>(p.bufW) + static_cast<size_t>(i)], want)
                        << i << "," << j;
                }
        }
    });
}
