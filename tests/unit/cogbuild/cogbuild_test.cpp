// SPDX-License-Identifier: MPL-2.0
//
// COG builder: overview resampling against a direct evaluation of GDAL's formulas (cubic
// weights at a factor of 2 and at odd sizes, masks, nodata, the "half the taps" rule),
// overview sizes, the statistics accumulator against a two-pass reference, and end-to-end own
// grid builds of synthetic EPSG:3857 COGs (written by our CogWriter) read back by TiffReader:
// same full-resolution pixels, overviews, mask, nodata, sidecar.

#include <gtest/gtest.h>

#include <tiffio.h>

// Index arithmetic of the synthetic images mixes int and size_t freely.
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wdouble-promotion"

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "../test_env.h"
#include "cog/build/cog_builder.h"
#include "cog/build/pipeline.h"
#include "cog/build/tiff_source.h"
#include "cog/build/stats.h"
#include "cog/encode/block_encoder.h"
#include "cog/pyramid/overview.h"
#include "cog/tiff/geotiff.h"
#include "cog/tiff/tiff_reader.h"
#include "cog/tiler/cog_source.h"
#include "cog/tiler/json_lite.h"
#include "cog/tiler/stats.h"
#include "cog/writer/cog_writer.h"
#include "ddbt/ddbt.h"

namespace ddbt::cog {
namespace {

namespace fs = std::filesystem;
using encode::ByteBuffer;

class TempDir {
public:
    TempDir() {
        const char* base = std::getenv("DDBT_TEST_TMP");
        fs::path root = base ? fs::path(base) : fs::temp_directory_path();
        path_ = root / ("ddbt-cogbuild-" + std::to_string(std::random_device{}()) + "-" + std::to_string(counter_++));
        fs::create_directories(path_);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path_, ec);
    }
    std::string File(const std::string& name) const { return (path_ / name).string(); }

private:
    static inline std::atomic<int> counter_{0};
    fs::path path_;
};

TEST(Overview, CubicTapsAtFactorTwo) {
    const pyramid::Taps t = pyramid::CubicTaps(1024, 512, 0, 512);
    // Interior pixel: 8 taps 2i-3 .. 2i+4, kernel at -1.75 .. 1.75 in steps of 0.5.
    const size_t i = 100;
    EXPECT_EQ(t.start[i], 197);
    EXPECT_EQ(t.count[i], 8);
    const double* w = t.weights.data() + i * static_cast<size_t>(t.maxTaps);
    double sum = 0;
    for (int k = 0; k < 8; ++k) {
        EXPECT_DOUBLE_EQ(w[k], pyramid::CubicKernel(-1.75 + 0.5 * k));
        sum += w[k];
    }
    EXPECT_NEAR(sum, 2.0, 1e-12);  // the stretched kernel sums to the ratio
    // Edges: clipped taps.
    EXPECT_EQ(t.start[0], 0);
    EXPECT_EQ(t.count[0], 5);
    EXPECT_EQ(t.start[511] + t.count[511], 1024);
}

// Direct evaluation of GDAL's separable convolution (no mask) for one destination pixel.
double Reference(const std::vector<float>& src, int sw, int sh, int dw, int dh, int x, int y) {
    const pyramid::Taps cx = pyramid::CubicTaps(sw, dw, x, x + 1), cy = pyramid::CubicTaps(sh, dh, y, y + 1);
    double v = 0;
    for (int j = 0; j < cy.count[0]; ++j) {
        double h = 0;
        for (int i = 0; i < cx.count[0]; ++i)
            h += src[static_cast<size_t>(cy.start[0] + j) * static_cast<size_t>(sw) + static_cast<size_t>(cx.start[0] + i)] *
                 (cx.weights[static_cast<size_t>(i)] * cx.invSum[0]);
        v += h * (cy.weights[static_cast<size_t>(j)] * cy.invSum[0]);
    }
    return v;
}

TEST(Overview, DownsampleMatchesDirectEvaluationAndWindowSplits) {
    const int sw = 331, sh = 207;
    const auto dims = pyramid::OverviewSizes(sw, sh, 64, false);
    ASSERT_FALSE(dims.empty());
    const int dw = static_cast<int>(dims[0].first), dh = static_cast<int>(dims[0].second);
    EXPECT_EQ(dw, 165);
    EXPECT_EQ(dh, 103);
    std::mt19937 rng(1);
    std::vector<float> src(static_cast<size_t>(sw) * sh);
    for (auto& v : src) v = static_cast<float>(rng() % 1000) / 7.0f;
    pyramid::DownsampleSpec spec;
    spec.type = tiff::SampleType::kF32;
    spec.bands = 1;
    std::vector<float> whole(static_cast<size_t>(dw) * dh), split(whole.size());
    const auto cols = pyramid::CubicTaps(sw, dw, 0, dw), rows = pyramid::CubicTaps(sh, dh, 0, dh);
    pyramid::Downsample(spec, reinterpret_cast<const uint8_t*>(src.data()), static_cast<size_t>(sw) * 4, 0, 0, sw, sh,
                        nullptr, cols, rows, dw, dh, reinterpret_cast<uint8_t*>(whole.data()), static_cast<size_t>(dw) * 4);
    for (int y = 0; y < dh; y += 17) {
        for (int x = 0; x < dw; x += 13) {
            EXPECT_NEAR(whole[static_cast<size_t>(y) * dw + x], Reference(src, sw, sh, dw, dh, x, y), 1e-3) << x << "," << y;
        }
    }
    // The same result from windows (tiles of 40 x 30 with only their taps' source rows).
    for (int y0 = 0; y0 < dh; y0 += 30) {
        for (int x0 = 0; x0 < dw; x0 += 40) {
            const int x1 = std::min(dw, x0 + 40), y1 = std::min(dh, y0 + 30);
            const auto c = pyramid::CubicTaps(sw, dw, x0, x1), r = pyramid::CubicTaps(sh, dh, y0, y1);
            const int cb = c.SrcBegin(), rb = r.SrcBegin();
            const int ww = c.SrcEnd() - cb, wh = r.SrcEnd() - rb;
            std::vector<float> win(static_cast<size_t>(ww) * wh);
            for (int y = 0; y < wh; ++y)
                std::memcpy(&win[static_cast<size_t>(y) * ww], &src[static_cast<size_t>(rb + y) * sw + cb], static_cast<size_t>(ww) * 4);
            pyramid::Downsample(spec, reinterpret_cast<const uint8_t*>(win.data()), static_cast<size_t>(ww) * 4, cb, rb, ww,
                                wh, nullptr, c, r, x1 - x0, y1 - y0,
                                reinterpret_cast<uint8_t*>(&split[static_cast<size_t>(y0) * dw + x0]),
                                static_cast<size_t>(dw) * 4);
        }
    }
    EXPECT_EQ(0, std::memcmp(whole.data(), split.data(), whole.size() * 4));
}

TEST(Overview, MaskRulesAndNodata) {
    // A 16 x 16 Byte image, valid only in its left half.
    const int sw = 16, sh = 16, dw = 8, dh = 8;
    std::vector<uint8_t> src(static_cast<size_t>(sw) * sh, 100), mask(src.size(), 0);
    for (int y = 0; y < sh; ++y)
        for (int x = 0; x < sw / 2; ++x) mask[static_cast<size_t>(y) * sw + x] = 255;
    const auto cols = pyramid::CubicTaps(sw, dw, 0, dw), rows = pyramid::CubicTaps(sh, dh, 0, dh);
    pyramid::DownsampleSpec spec;
    spec.mask = pyramid::MaskMode::kExternal;
    std::vector<uint8_t> out(static_cast<size_t>(dw) * dh), outMask(out.size());
    pyramid::Downsample(spec, src.data(), sw, 0, 0, sw, sh, mask.data(), cols, rows, dw, dh, out.data(), dw);
    pyramid::DownsampleMask(mask.data(), 0, 0, sw, sh, cols, rows, dw, dh, outMask.data(), dw);
    for (int x = 0; x < dw; ++x) {
        // Valid pixels average valid taps only: exactly 100. A destination pixel whose taps hold
        // a run of valid taps of at least half their count is valid.
        const auto c = pyramid::CubicTaps(sw, dw, x, x + 1);
        int valid = 0;
        for (int k = 0; k < c.count[0]; ++k) valid += c.start[0] + k < sw / 2;
        const bool expectValid = valid >= c.count[0] / 2 && valid > 0;
        EXPECT_EQ(outMask[static_cast<size_t>(4) * dw + x] != 0, expectValid) << x;
        EXPECT_EQ(out[static_cast<size_t>(4) * dw + x], expectValid ? 100 : 0) << x;
    }
    // Nodata: a computed value equal to the nodata value is replaced (100 -> 101), invalid
    // pixels get the nodata value.
    std::vector<uint8_t> nd(src);
    for (int y = 0; y < sh; ++y)
        for (int x = sw / 2; x < sw; ++x) nd[static_cast<size_t>(y) * sw + x] = 7;
    pyramid::DownsampleSpec ns;
    ns.mask = pyramid::MaskMode::kNoData;
    ns.hasNoData = true;
    ns.noData = 7;
    pyramid::Downsample(ns, nd.data(), sw, 0, 0, sw, sh, nullptr, cols, rows, dw, dh, out.data(), dw);
    EXPECT_EQ(out[static_cast<size_t>(4) * dw + 0], 100);
    EXPECT_EQ(out[static_cast<size_t>(4) * dw + dw - 1], 7);
    ns.noData = 100;
    std::vector<uint8_t> all(src.size(), 100);
    for (int y = 0; y < 4; ++y)
        for (int x = 0; x < 4; ++x) all[static_cast<size_t>(y) * sw + x] = 50;
    pyramid::Downsample(ns, all.data(), sw, 0, 0, sw, sh, nullptr, cols, rows, dw, dh, out.data(), dw);
    EXPECT_EQ(out[0], 50);        // its valid taps (a run of 4 of 5) are all 50
    EXPECT_EQ(out[dw - 1], 100);  // no valid tap: nodata
}

TEST(Overview, SizesFollowTheCogDriver) {
    const auto own = pyramid::OverviewSizes(1913, 1860, 512, false);
    ASSERT_EQ(own.size(), 2u);
    EXPECT_EQ(own[0], std::make_pair(956u, 930u));
    EXPECT_EQ(own[1], std::make_pair(478u, 465u));
    const auto tm = pyramid::OverviewSizes(7424, 5376, 512, true);
    ASSERT_EQ(tm.size(), 4u);
    EXPECT_EQ(tm[3], std::make_pair(464u, 336u));
    const auto odd = pyramid::OverviewSizes(1025, 600, 512, true);
    EXPECT_EQ(odd[0], std::make_pair(513u, 300u));
    EXPECT_EQ(odd[1], std::make_pair(257u, 150u));
}

TEST(Stats, WelfordMatchesTwoPassReference) {
    std::mt19937 rng(4);
    const uint32_t W = 97, H = 31;
    std::vector<float> px(static_cast<size_t>(W) * H * 2);
    std::vector<uint8_t> mask(static_cast<size_t>(W) * H);
    for (size_t i = 0; i < px.size(); ++i) px[i] = static_cast<float>(rng() % 10000) / 3.0f;
    for (auto& m : mask) m = rng() % 5 == 0 ? 0 : 255;
    px[10] = std::nanf("");
    px[12] = -32767.0f;
    build::StatsAccumulator acc(2, tiff::SampleType::kF32,
                                {build::StatsAccumulator::MaskKind::kMask, build::StatsAccumulator::MaskKind::kNone}, -1,
                                -32767.0);
    acc.AccumulateRows(0, reinterpret_cast<const uint8_t*>(px.data()), W * 8, W, 10, mask.data());
    acc.AccumulateRows(0, reinterpret_cast<const uint8_t*>(px.data()) + 10 * W * 8, W * 8, W, H - 10, mask.data() + 10 * W);
    acc.AccumulateRows(1, reinterpret_cast<const uint8_t*>(px.data()), W * 8, W, H, nullptr);
    const auto r = acc.Result(static_cast<uint64_t>(W) * H);
    for (int b = 0; b < 2; ++b) {
        double sum = 0, mn = 1e300, mx = -1e300;
        std::vector<double> v;
        for (size_t i = 0; i < static_cast<size_t>(W) * H; ++i) {
            const float f = px[i * 2 + static_cast<size_t>(b)];
            if (!std::isfinite(f) || f == -32767.0f) continue;
            if (b == 0 && mask[i] == 0) continue;
            v.push_back(f);
        }
        for (double x : v) {
            sum += x;
            mn = std::min(mn, x);
            mx = std::max(mx, x);
        }
        const double mean = sum / static_cast<double>(v.size());
        double ss = 0;
        for (double x : v) ss += (x - mean) * (x - mean);
        ASSERT_TRUE(r[static_cast<size_t>(b)].has_value());
        EXPECT_EQ(r[static_cast<size_t>(b)]->valid, v.size());
        EXPECT_EQ(r[static_cast<size_t>(b)]->min, mn);
        EXPECT_EQ(r[static_cast<size_t>(b)]->max, mx);
        EXPECT_NEAR(r[static_cast<size_t>(b)]->mean, mean, 1e-9 * std::abs(mean));
        EXPECT_NEAR(r[static_cast<size_t>(b)]->stdDev, std::sqrt(ss / static_cast<double>(v.size())), 1e-6);
    }
}

TEST(Stats, DefaultIsExactSumsAndCompatIsOptIn) {
    // 10^6 bytes whose Welford mean is not the correctly rounded one (the tiler's exact pass and
    // the builder share the kernels): the default accumulator gives sum / n, DDBT_STATS_MODE=compat
    // gives Welford's bits.
    ASSERT_EQ(std::getenv("DDBT_STATS_MODE"), nullptr) << "run the tests without DDBT_STATS_MODE";
    std::mt19937_64 rng(11);
    const uint32_t W = 1000, H = 1000;
    std::vector<uint8_t> px(static_cast<size_t>(W) * H);
    int64_t sum = 0;
    for (auto& x : px) {
        x = static_cast<uint8_t>(rng() % 256);
        sum += x;
    }
    const double exact = static_cast<double>(sum) / static_cast<double>(px.size());
    double wm = 0;
    uint64_t n = 0;
    for (uint8_t x : px) {
        ++n;
        wm += (static_cast<double>(x) - wm) / static_cast<double>(n);
    }
    const auto run = [&] {
        build::StatsAccumulator acc(1, tiff::SampleType::kU8, {build::StatsAccumulator::MaskKind::kNone}, -1, std::nullopt);
        acc.AccumulateRows(0, px.data(), W, W, H, nullptr);
        return acc.Result(px.size())[0]->mean;
    };
    EXPECT_EQ(run(), exact);
    double compat = 0;
    {
        ddbt::test::ScopedEnv env("DDBT_STATS_MODE", "compat");
        compat = run();
    }
    EXPECT_EQ(compat, wm);
    EXPECT_EQ(run(), exact);
}

// A synthetic optimized COG in EPSG:3857 (one level of overviews so DroneDB's rule sees one).
struct SourceCog {
    uint32_t W = 700, H = 530, B = 256;
    tiff::SampleType type;
    int bands;
    bool mask;
    std::optional<double> nodata;
    std::vector<uint8_t> pixels;
    std::vector<uint8_t> maskBytes;
    std::vector<uint16_t> colorMap;  // a palette source (1 band, 8-bit)
    bool complex = false;            // `bands` values are bands / 2 complex samples

    SourceCog(tiff::SampleType t, int n, bool m, std::optional<double> nd) : type(t), bands(n), mask(m), nodata(nd) {
        const size_t ss = tiff::SampleTypeSize(t);
        pixels.resize(static_cast<size_t>(W) * H * static_cast<size_t>(n) * ss);
        maskBytes.assign(static_cast<size_t>(W) * H, 255);
        for (uint32_t y = 0; y < H; ++y)
            for (uint32_t x = 0; x < W; ++x) {
                const size_t i = static_cast<size_t>(y) * W + x;
                if (m && (x * x + y * y) < 90000) maskBytes[i] = 0;
                for (int b = 0; b < n; ++b) {
                    uint8_t* p = pixels.data() + (i * static_cast<size_t>(n) + static_cast<size_t>(b)) * ss;
                    if (t == tiff::SampleType::kF32) {
                        float v = static_cast<float>(std::sin(x * 0.05 + b) * 100 + y);
                        if (nd && x > 600 && y > 400) v = static_cast<float>(*nd);
                        std::memcpy(p, &v, 4);
                    } else {
                        *p = static_cast<uint8_t>((x * (b + 1) + y * 3) & 0xFF);
                    }
                }
            }
    }

    void Write(const std::string& path) const {
        writer::ImageSpec s;
        s.width = W;
        s.height = H;
        s.blockSize = B;
        s.samples = static_cast<uint16_t>(complex ? bands / 2 : bands);
        s.type = type;
        s.complex = complex;
        s.photometric = !colorMap.empty() ? 3 : (!complex && bands >= 3) ? 2 : 1;
        s.colorMap = colorMap;
        s.codec = encode::Codec::kDeflate;
        s.predictor = 1;
        s.hasMask = mask;
        if (nodata) {
            std::ostringstream o;
            o << *nodata;
            s.noDataText = o.str();
        }
        s.geoTransform = {-1000000.0, 0.25, 0.0, 5000000.0, 0.0, -0.25};
        s.overviews = {{W / 2, H / 2}};
        auto w = writer::CogWriter::Create(path, s, {});
        ASSERT_TRUE(w.ok());
        const size_t pb = tiff::SampleTypeSize(type) * static_cast<size_t>(bands);
        for (uint32_t l = 0; l < 2; ++l) {
            for (uint32_t r = 0; r < (*w)->BlocksDown(l); ++r)
                for (uint32_t c = 0; c < (*w)->BlocksAcross(l); ++c) {
                    std::vector<uint8_t> block(static_cast<size_t>(B) * B * pb, 0), mb(static_cast<size_t>(B) * B, 0);
                    for (uint32_t y = 0; y < B; ++y)
                        for (uint32_t x = 0; x < B; ++x) {
                            const uint32_t sx = std::min(W - 1, (c * B + x) << l), sy = std::min(H - 1, (r * B + y) << l);
                            const size_t i = static_cast<size_t>(sy) * W + sx;
                            std::memcpy(&block[(static_cast<size_t>(y) * B + x) * pb], &pixels[i * pb], pb);
                            mb[static_cast<size_t>(y) * B + x] = maskBytes[i];
                        }
                    encode::BlockFormat f{B, B, static_cast<uint32_t>(bands), type, encode::Codec::kDeflate, 1, 0, 75};
                    ByteBuffer img, msk;
                    encode::EncodeBlock(f, block.data(), img);
                    ASSERT_TRUE((*w)->Put(l, c, r, false, std::move(img)).ok());
                    if (mask) {
                        encode::EncodeMaskBlock(mb.data(), B, B, msk);
                        ASSERT_TRUE((*w)->Put(l, c, r, true, std::move(msk)).ok());
                    }
                }
        }
        ASSERT_TRUE((*w)->Finish().ok());
    }
};

void CheckOwnGrid(const SourceCog& src, const std::string& out, const char* codec) {
    auto r = tiff::TiffReader::Open(out);
    ASSERT_TRUE(r.ok()) << r.status().message();
    const tiff::TiffReader& rd = **r;
    const tiff::IfdInfo& m = rd.Ifd(rd.MainImage());
    EXPECT_EQ(m.width, src.W);
    EXPECT_EQ(m.height, src.H);
    EXPECT_EQ(m.blockWidth, src.B);
    EXPECT_STREQ(codec, encode::CodecName(static_cast<encode::Codec>(
                            m.compression == 5 ? 1 : m.compression == 8 ? 2 : m.compression == 50000 ? 3 : 0)));
    EXPECT_EQ(rd.Overviews().size(), 2u);  // 700x530 -> 350x265 -> 175x132 (block 256)
    EXPECT_EQ(rd.MaskOf(m.index).has_value(), src.mask);
    if (src.nodata) {
        EXPECT_EQ(m.noData, src.nodata);
    }
    EXPECT_NE(m.gdalMetadata.find("OVERVIEW_RESAMPLING"), std::string::npos);
    const size_t pb = tiff::SampleTypeSize(src.type) * static_cast<size_t>(src.bands);
    for (uint32_t row = 0; row < m.blocksDown; ++row)
        for (uint32_t col = 0; col < m.blocksAcross; ++col) {
            auto b = rd.ReadBlock(m.index, m.BlockIndex(col, row));
            ASSERT_TRUE(b.ok()) << b.status().message();
            for (uint32_t y = 0; y < src.B && row * src.B + y < src.H; ++y) {
                const uint32_t vw = std::min(src.B, src.W - col * src.B);
                const size_t i = static_cast<size_t>(row * src.B + y) * src.W + col * src.B;
                if (src.mask) {
                    // Masked-out pixels may be anything (sparse blocks read 0).
                    for (uint32_t x = 0; x < vw; ++x) {
                        if (src.maskBytes[i + x]) {
                            ASSERT_EQ(0, std::memcmp(b->data() + (static_cast<size_t>(y) * src.B + x) * pb,
                                                     &src.pixels[(i + x) * pb], pb));
                        }
                    }
                } else {
                    ASSERT_EQ(0, std::memcmp(b->data() + static_cast<size_t>(y) * src.B * pb, &src.pixels[i * pb], vw * pb))
                        << col << "," << row << "," << y;
                }
            }
        }
    std::ifstream in(out + ".stats.json");
    std::stringstream ss;
    ss << in.rdbuf();
    const auto j = tiler::json::Parse(ss.str());
    ASSERT_TRUE(j && j->IsObject());
    ASSERT_NE(j->Find("bands"), nullptr);
    EXPECT_EQ(j->Find("version")->u, 2u);
    EXPECT_TRUE(j->Find("exact")->b);
    EXPECT_EQ(j->Find("bands")->obj.size(), static_cast<size_t>(src.bands));
}

TEST(CogBuild, OwnGridByteRgbWithMaskIsLosslessLzw) {
    TempDir dir;
    SourceCog src(tiff::SampleType::kU8, 3, true, std::nullopt);
    const std::string in = dir.File("in.tif"), out = dir.File("out.tif");
    src.Write(in);
    build::BuildReport rep;
    const Status s = build::BuildCog(in, out, {}, &rep);
    ASSERT_TRUE(s.ok()) << s.message();
    EXPECT_TRUE(rep.ownGrid);
    EXPECT_EQ(rep.transparency, "mask");
    CheckOwnGrid(src, out, "LZW");
}

TEST(CogBuild, OwnGridFloatNodataWithZstdOption) {
    TempDir dir;
    SourceCog src(tiff::SampleType::kF32, 1, false, -9999.0);
    const std::string in = dir.File("in.tif"), out = dir.File("out.tif");
    src.Write(in);
    build::BuildOptions o;
    o.losslessCodec = encode::Codec::kZstd;
    build::BuildReport rep;
    const Status s = build::BuildCog(in, out, o, &rep);
    ASSERT_TRUE(s.ok()) << s.message();
    EXPECT_EQ(rep.transparency, "nodata");
    CheckOwnGrid(src, out, "ZSTD");
}

// A palette source keeps its colour table (GDAL reads 8-bit values times 257 back as 8-bit and
// writes them times 257 again), its overviews are by nearest neighbour (the COG driver's rule for a
// colour table) and say so
TEST(CogBuild, OwnGridPaletteKeepsItsColorMapAndNearestOverviews) {
    TempDir dir;
    SourceCog src(tiff::SampleType::kU8, 1, false, std::nullopt);
    src.colorMap.resize(3 * 256);
    for (uint32_t i = 0; i < 256; ++i) {
        src.colorMap[i] = static_cast<uint16_t>(i * 257);
        src.colorMap[256 + i] = static_cast<uint16_t>((255 - i) * 257);
        src.colorMap[512 + i] = static_cast<uint16_t>(((i * 7) & 255) * 257);
    }
    const std::string in = dir.File("in.tif"), out = dir.File("out.tif");
    src.Write(in);
    ASSERT_TRUE(build::BuildCog(in, out, {}).ok());
    auto r = tiff::TiffReader::Open(out);
    ASSERT_TRUE(r.ok());
    const tiff::TiffReader& rd = **r;
    const tiff::IfdInfo& m = rd.Ifd(rd.MainImage());
    EXPECT_EQ(m.photometric, tiff::photometric::kPalette);
    EXPECT_EQ(m.colorMap, src.colorMap);
    EXPECT_NE(m.gdalMetadata.find("OVERVIEW_RESAMPLING\" domain=\"IMAGE_STRUCTURE\">NEAREST"), std::string::npos)
        << m.gdalMetadata;
    // Each overview pixel is the pixel of the level above at int(0.5 + d * size / overview size)
    auto level = [&](uint32_t ifd) {
        const tiff::IfdInfo& l = rd.Ifd(ifd);
        std::vector<uint8_t> px(static_cast<size_t>(l.width) * l.height);
        for (uint32_t b = 0; b < l.blockCount; ++b) {
            auto blk = rd.ReadBlock(ifd, b);
            EXPECT_TRUE(blk.ok());
            const uint32_t x0 = l.BlockColumn(b) * l.blockWidth, y0 = l.BlockRow(b) * l.blockHeight;
            for (uint32_t y = 0; y < l.blockHeight && y0 + y < l.height; ++y)
                for (uint32_t x = 0; x < l.blockWidth && x0 + x < l.width; ++x)
                    px[static_cast<size_t>(y0 + y) * l.width + x0 + x] = blk->data()[static_cast<size_t>(y) * blk->rowStride + x];
        }
        return std::pair{px, std::pair{l.width, l.height}};
    };
    auto [prev, ps] = level(m.index);
    for (const uint32_t ovr : rd.Overviews()) {
        auto [cur, cs] = level(ovr);
        EXPECT_EQ(rd.Ifd(ovr).colorMap, src.colorMap);
        for (uint32_t y = 0; y < cs.second; ++y)
            for (uint32_t x = 0; x < cs.first; ++x) {
                const auto sx = static_cast<uint32_t>(0.5 + x * (static_cast<double>(ps.first) / cs.first));
                const auto sy = static_cast<uint32_t>(0.5 + y * (static_cast<double>(ps.second) / cs.second));
                ASSERT_EQ(cur[static_cast<size_t>(y) * cs.first + x], prev[static_cast<size_t>(sy) * ps.first + sx])
                    << x << "," << y;
            }
        prev = std::move(cur);
        ps = cs;
    }
}

// A complex source (CInt16) keeps its samples (SampleFormat 5, 32 bits, predictor 2 on the whole
// sample), its overviews are by nearest neighbour, its statistics are those of the magnitudes
TEST(CogBuild, OwnGridComplexKeepsItsSamplesAndMeasuresTheirMagnitude) {
    TempDir dir;
    SourceCog src(tiff::SampleType::kI16, 2, false, std::nullopt);
    src.complex = true;
    auto* v = reinterpret_cast<int16_t*>(src.pixels.data());
    double sum = 0, maxMag = 0;
    for (size_t i = 0; i < src.pixels.size() / 4; ++i) {
        v[2 * i] = static_cast<int16_t>((i * 37) % 2000 - 1000);
        v[2 * i + 1] = static_cast<int16_t>((i * 11) % 600 - 300);
        const double m = std::sqrt(double(v[2 * i]) * v[2 * i] + double(v[2 * i + 1]) * v[2 * i + 1]);
        sum += m;
        maxMag = std::max(maxMag, m);
    }
    const std::string in = dir.File("in.tif"), out = dir.File("out.tif");
    src.Write(in);
    ASSERT_TRUE(build::BuildCog(in, out, {}).ok());
    auto r = tiff::TiffReader::Open(out);
    ASSERT_TRUE(r.ok());
    const tiff::IfdInfo& m = (*r)->Ifd((*r)->MainImage());
    EXPECT_TRUE(m.complex);
    EXPECT_EQ(m.sampleFormat, 5);
    EXPECT_EQ(m.bitsPerSample, 32);
    EXPECT_EQ(m.samplesPerPixel, 1);
    EXPECT_EQ(m.predictor, 2);
    EXPECT_NE(m.gdalMetadata.find(">NEAREST<"), std::string::npos);
    for (uint32_t row = 0; row < m.blocksDown; ++row)
        for (uint32_t col = 0; col < m.blocksAcross; ++col) {
            auto b = (*r)->ReadBlock(m.index, m.BlockIndex(col, row));
            ASSERT_TRUE(b.ok());
            for (uint32_t y = 0; y < src.B && row * src.B + y < src.H; ++y) {
                const uint32_t vw = std::min(src.B, src.W - col * src.B);
                const size_t i = static_cast<size_t>(row * src.B + y) * src.W + col * src.B;
                ASSERT_EQ(0, std::memcmp(b->data() + static_cast<size_t>(y) * src.B * 4, &src.pixels[i * 4], vw * 4));
            }
        }
    std::ifstream sin(out + ".stats.json");
    std::stringstream ss;
    ss << sin.rdbuf();
    const auto j = tiler::json::Parse(ss.str());
    ASSERT_TRUE(j && j->IsObject());
    const auto* band = j->Find("bands")->Find("1");
    ASSERT_NE(band, nullptr);
    EXPECT_NEAR(band->Find("mean")->num, sum / (src.pixels.size() / 4), 1e-9);
    EXPECT_NEAR(band->Find("max")->num, maxMag, 1e-9);
}

// A complex raster is tiled by its magnitude: its tiles are those of the Float64 raster of the
// magnitudes of its samples
TEST(CogBuild, AComplexRasterIsTiledByItsMagnitude) {
    TempDir dir;
    SourceCog cpx(tiff::SampleType::kI16, 2, false, std::nullopt);
    cpx.complex = true;
    SourceCog mag(tiff::SampleType::kF64, 1, false, std::nullopt);
    const auto* v = reinterpret_cast<int16_t*>(cpx.pixels.data());
    auto* m = reinterpret_cast<double*>(mag.pixels.data());
    for (size_t i = 0; i < cpx.pixels.size() / 4; ++i) {
        reinterpret_cast<int16_t*>(cpx.pixels.data())[2 * i] = static_cast<int16_t>((i * 37) % 2000 - 1000);
        reinterpret_cast<int16_t*>(cpx.pixels.data())[2 * i + 1] = static_cast<int16_t>((i * 11) % 600 - 300);
        m[i] = std::sqrt(double(v[2 * i]) * v[2 * i] + double(v[2 * i + 1]) * v[2 * i + 1]);
    }
    const std::string cin = dir.File("c.tif"), cout = dir.File("c.cog.tif"), min = dir.File("m.tif"),
                      mout = dir.File("m.cog.tif");
    cpx.Write(cin);
    mag.Write(min);
    ASSERT_TRUE(build::BuildCog(cin, cout, {}).ok());
    ASSERT_TRUE(build::BuildCog(min, mout, {}).ok());
    char* info = nullptr;
    ASSERT_EQ(ddbtTileInfo(cout.c_str(), 256, &info), DDBT_ERR_NONE) << ddbtLastError();
    const auto j = tiler::json::Parse(info);
    ddbtFree(info);
    const int z = static_cast<int>(j->Find("maxZoom")->num);
    const double R = 20037508.342789244, n = std::ldexp(1.0, z);
    const int x0 = static_cast<int>((j->Find("minX")->num + R) / (2 * R) * n);
    const int y0 = static_cast<int>((R - j->Find("maxY")->num) / (2 * R) * n);
    int tiles = 0;
    for (int dy = 0; dy < 2; ++dy)
        for (int dx = 0; dx < 2; ++dx) {
            uint8_t *a = nullptr, *b = nullptr;
            int as = 0, bs = 0;
            ASSERT_EQ(ddbtMemoryTileFmt(cout.c_str(), z, x0 + dx, y0 + dy, &a, &as, 256, true, true, nullptr, "png"),
                      DDBT_ERR_NONE)
                << ddbtLastError();
            ASSERT_EQ(ddbtMemoryTileFmt(mout.c_str(), z, x0 + dx, y0 + dy, &b, &bs, 256, true, true, nullptr, "png"),
                      DDBT_ERR_NONE);
            EXPECT_EQ(std::vector<uint8_t>(a, a + as), std::vector<uint8_t>(b, b + bs));
            ddbtFree(a);
            ddbtFree(b);
            ++tiles;
        }
    EXPECT_EQ(tiles, 4);
}

// The statistics the tiler computes for a complex raster without a sidecar are those of its
// magnitudes, in both arithmetics, with and without a mask
TEST(CogBuild, TheStatisticsOfAComplexRasterAreThoseOfItsMagnitudes) {
    TempDir dir;
    for (const bool mask : {false, true}) {
        SourceCog cpx(tiff::SampleType::kF32, 4, mask, std::nullopt);
        cpx.complex = true;
        SourceCog mag(tiff::SampleType::kF64, 2, mask, std::nullopt);
        auto* v = reinterpret_cast<float*>(cpx.pixels.data());
        auto* m = reinterpret_cast<double*>(mag.pixels.data());
        for (size_t i = 0; i < mag.pixels.size() / sizeof(double); ++i) {
            v[2 * i] = static_cast<float>(static_cast<int>((i * 37) % 2000) - 1000) * 0.25f;
            v[2 * i + 1] = static_cast<float>(static_cast<int>((i * 11) % 600) - 300) * 0.5f;
            m[i] = std::sqrt(double(v[2 * i]) * v[2 * i] + double(v[2 * i + 1]) * v[2 * i + 1]);
        }
        const std::string tag = mask ? "m" : "n";
        const std::string cin = dir.File(tag + "c.tif"), cout = dir.File(tag + "c.cog.tif"), min = dir.File(tag + "m.tif"),
                          mout = dir.File(tag + "m.cog.tif");
        cpx.Write(cin);
        mag.Write(min);
        ASSERT_TRUE(build::BuildCog(cin, cout, {}).ok());
        ASSERT_TRUE(build::BuildCog(min, mout, {}).ok());
        const auto c = tiler::CogSource::Open(cout), g = tiler::CogSource::Open(mout);
        ASSERT_EQ(c->BandCount(), 2);
        for (const auto mode : {stats::StatsMode::kCompat, stats::StatsMode::kExact}) {
            const auto a = tiler::ComputeBandStatistics(*c, {1, 2}, std::nullopt, mode);
            const auto b = tiler::ComputeBandStatistics(*g, {1, 2}, std::nullopt, mode);
            for (size_t k = 0; k < 2; ++k) {
                ASSERT_TRUE(a[k] && b[k]);
                EXPECT_EQ(a[k]->min, b[k]->min);
                EXPECT_EQ(a[k]->max, b[k]->max);
                EXPECT_EQ(a[k]->mean, b[k]->mean);
                EXPECT_EQ(a[k]->stdDev, b[k]->stdDev);
                EXPECT_EQ(a[k]->validCount, b[k]->validCount);
            }
        }
    }
}

// GDAL_METADATA of an output: OVERVIEW_RESAMPLING only with overviews; the colour
// interpretations the photometric gives (YCbCr JPEG, palette) are not stored
TEST(CogBuild, OutputMetadataFollowsGdal) {
    const std::string src =
        "<GDALMetadata>\n  <Item name=\"COLORINTERP\" sample=\"0\" role=\"colorinterp\">Undefined</Item>\n"
        "  <Item name=\"AREA\">x</Item>\n  <Item name=\"STATISTICS_MEAN\" sample=\"0\">1</Item>\n</GDALMetadata>\n";
    build::Plan p;
    EXPECT_EQ(build::OutputMetadata("", p, true), "");
    EXPECT_EQ(build::OutputMetadata(src, p, true),
              "<GDALMetadata>\n  <Item name=\"COLORINTERP\" sample=\"0\" role=\"colorinterp\">Undefined</Item>\n"
              "  <Item name=\"AREA\">x</Item>\n</GDALMetadata>\n");
    p.photometric = tiff::photometric::kYCbCr;
    p.overviews = {{256, 256}};
    EXPECT_EQ(build::OutputMetadata(src, p, false),
              "<GDALMetadata>\n  <Item name=\"AREA\">x</Item>\n  <Item name=\"STATISTICS_MEAN\" sample=\"0\">1</Item>\n"
              "  <Item name=\"OVERVIEW_RESAMPLING\" domain=\"IMAGE_STRUCTURE\">CUBIC</Item>\n</GDALMetadata>\n");
    build::SetPalette(p, {{1, 2, 3}});
    EXPECT_EQ(p.colorMap.size(), 3u * 256u);
    EXPECT_EQ(p.colorMap[0], 257u);
    EXPECT_EQ(p.colorMap[256], 514u);
    EXPECT_EQ(p.colorMap[512], 771u);
    EXPECT_EQ(p.colorMap[1], 0u);
    EXPECT_NE(build::OutputMetadata(src, p, true).find(">NEAREST<"), std::string::npos);
}

// The colour tables GDAL gives sources without a palette of their own: black and white for a
// 1-bit image (white first when min-is-white), an inverted grey ramp for 8-bit min-is-white; and a
// ColorMap of 8-bit values read as they are. A 1-bit image reads as bytes 0 / 1.
TEST(CogBuild, SourcesGetGdalsColourTables) {
    TempDir dir;
    auto write = [&](const std::string& name, uint16_t bits, uint16_t photometric, uint16_t compression,
                     const std::vector<uint16_t>& colorMap) {
        const std::string path = dir.File(name);
        TIFF* t = TIFFOpen(path.c_str(), "w");
        EXPECT_NE(t, nullptr);
        TIFFSetField(t, TIFFTAG_IMAGEWIDTH, 40);
        TIFFSetField(t, TIFFTAG_IMAGELENGTH, 8);
        TIFFSetField(t, TIFFTAG_SAMPLESPERPIXEL, 1);
        TIFFSetField(t, TIFFTAG_BITSPERSAMPLE, bits);
        TIFFSetField(t, TIFFTAG_PHOTOMETRIC, photometric);
        TIFFSetField(t, TIFFTAG_COMPRESSION, compression);
        TIFFSetField(t, TIFFTAG_ROWSPERSTRIP, 8);
        if (!colorMap.empty())
            TIFFSetField(t, TIFFTAG_COLORMAP, colorMap.data(), colorMap.data() + 256, colorMap.data() + 512);
        std::vector<uint8_t> rows(bits == 1 ? 5 * 8 : 40 * 8);
        for (size_t i = 0; i < rows.size(); ++i) rows[i] = bits == 1 ? 0xA5 : static_cast<uint8_t>(i);
        TIFFWriteEncodedStrip(t, 0, rows.data(), static_cast<tmsize_t>(rows.size()));
        TIFFClose(t);
        auto src = build::TiffSource::Open(path);
        EXPECT_TRUE(src.ok()) << src.status().message();
        return std::move(src).value();
    };
    using Table = std::vector<std::array<uint8_t, 3>>;
    const Table bw{{0, 0, 0}, {255, 255, 255}}, wb{{255, 255, 255}, {0, 0, 0}};

    auto g4 = write("g4.tif", 1, PHOTOMETRIC_MINISBLACK, COMPRESSION_CCITTFAX4, {});
    EXPECT_EQ(g4->Type(), tiff::SampleType::kU8);
    EXPECT_EQ(g4->ColorTable(), bw);
    EXPECT_EQ(g4->ColorInterp(0), "Palette");
    std::vector<uint8_t> px(g4->BlockBytes());
    ASSERT_TRUE(g4->ReadBlock(0, 0, px.data()).ok());
    EXPECT_EQ((std::vector<uint8_t>(px.begin(), px.begin() + 8)), (std::vector<uint8_t>{1, 0, 1, 0, 0, 1, 0, 1}));

    EXPECT_EQ(write("white.tif", 1, PHOTOMETRIC_MINISWHITE, COMPRESSION_NONE, {})->ColorTable(), wb);
    const auto ramp = write("white8.tif", 8, PHOTOMETRIC_MINISWHITE, COMPRESSION_NONE, {})->ColorTable();
    ASSERT_TRUE(ramp);
    ASSERT_EQ(ramp->size(), 256u);
    EXPECT_EQ((*ramp)[0], (std::array<uint8_t, 3>{255, 255, 255}));
    EXPECT_EQ((*ramp)[255], (std::array<uint8_t, 3>{0, 0, 0}));
    EXPECT_FALSE(write("gray8.tif", 8, PHOTOMETRIC_MINISBLACK, COMPRESSION_NONE, {})->ColorTable());

    std::vector<uint16_t> small(3 * 256, 0);  // 8-bit values: read as they are
    small[1] = 200;
    const auto t8 = write("pal8.tif", 8, PHOTOMETRIC_PALETTE, COMPRESSION_LZW, small)->ColorTable();
    ASSERT_TRUE(t8);
    EXPECT_EQ((*t8)[1], (std::array<uint8_t, 3>{200, 0, 0}));
    std::vector<uint16_t> by256(3 * 256, 0);  // multiples of 256: divided by 256
    by256[1] = 200 * 256;
    EXPECT_EQ((*write("pal256.tif", 8, PHOTOMETRIC_PALETTE, COMPRESSION_LZW, by256)->ColorTable())[1],
              (std::array<uint8_t, 3>{200, 0, 0}));
    std::vector<uint16_t> by257(3 * 256, 0);  // the others: divided by 257
    by257[1] = 65535;
    by257[2] = 1000;
    const auto t257 = write("pal257.tif", 8, PHOTOMETRIC_PALETTE, COMPRESSION_LZW, by257)->ColorTable();
    EXPECT_EQ((*t257)[1], (std::array<uint8_t, 3>{255, 0, 0}));
    EXPECT_EQ((*t257)[2], (std::array<uint8_t, 3>{3, 0, 0}));
}

TEST(CogBuild, MissingInputFails) {
    TempDir dir;
    EXPECT_FALSE(build::BuildCog(dir.File("nope.tif"), dir.File("out.tif"), {}).ok());
    EXPECT_FALSE(fs::exists(dir.File("out.tif")));
}

}  // namespace
}  // namespace ddbt::cog
