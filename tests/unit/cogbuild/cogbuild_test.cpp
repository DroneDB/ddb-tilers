// SPDX-License-Identifier: MPL-2.0
//
// COG builder: overview resampling against a direct evaluation of GDAL's formulas (cubic
// weights at a factor of 2 and at odd sizes, masks, nodata, the "half the taps" rule),
// overview sizes, the statistics accumulator against a two-pass reference, and end-to-end own
// grid builds of synthetic EPSG:3857 COGs (written by our CogWriter) read back by TiffReader:
// same full-resolution pixels, overviews, mask, nodata, sidecar.

#include <gtest/gtest.h>

// Index arithmetic of the synthetic images mixes int and size_t freely.
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wdouble-promotion"

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
#include "cog/build/stats.h"
#include "cog/encode/block_encoder.h"
#include "cog/pyramid/overview.h"
#include "cog/tiff/geotiff.h"
#include "cog/tiff/tiff_reader.h"
#include "cog/tiler/json_lite.h"
#include "cog/writer/cog_writer.h"

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
        s.samples = static_cast<uint16_t>(bands);
        s.type = type;
        s.photometric = bands >= 3 ? 2 : 1;
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

TEST(CogBuild, MissingInputFails) {
    TempDir dir;
    EXPECT_FALSE(build::BuildCog(dir.File("nope.tif"), dir.File("out.tif"), {}).ok());
    EXPECT_FALSE(fs::exists(dir.File("out.tif")));
}

}  // namespace
}  // namespace ddbt::cog
