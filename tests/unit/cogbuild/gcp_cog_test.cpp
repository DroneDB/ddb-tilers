// SPDX-License-Identifier: MPL-2.0
//
// The warped COG of GeoTIFFs georeferenced only by ground control points (GCPs, a CRS from the
// GeoKeys, no geotransform) against GDAL 3.13.3's gdalwarp with libddb's COG arguments
// (data/gen_gcp_cog_reference.py, data/gdal_cog_warp.cpp): the grid, the chunks of GDAL's warper,
// the pixels of every level, and GDAL's refusals; then the pieces: SourceGeoreference (pixel <->
// CRS by a geotransform or by GCPs), RowApproximator (GDAL's approximate transformer over a row,
// evaluated in parts) and CollectWarpChunks (GDAL's chunk list).

#include <gtest/gtest.h>

#include <libdeflate.h>

#include <array>
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

#include "../tiff/tiff_test_util.h"
#include "cog/build/cog_builder.h"
#include "cog/build/source.h"
#include "cog/tiff/geotiff.h"
#include "cog/tiff/tiff_reader.h"
#include "cog/warp/row_approx.h"
#include "cog/warp/source_georef.h"
#include "cog/warp/warp_chunks.h"
#include "cog/warp/warp_producer.h"
#include "core/crs/gcp_transform.h"

namespace ddbt::cog::warp {
namespace {

namespace fs = std::filesystem;

struct GcpCase {
    const char* name;
    const char* source;  // relative to tests/fixtures, or absolute (a file of this machine)
    bool ok;
    uint32_t width, height;
    std::array<double, 6> gt;
    const char* message;
    std::vector<std::array<int, 8>> chunks;  // sx, sy, sw, sh, dx, dy, dw, dh
    std::vector<uint32_t> crcs;              // per level and band, when GDAL's COG is not kept
};

const std::vector<GcpCase>& Cases() {
    static const std::vector<GcpCase> cases = {
#include "data/gcp_cog_reference.inc"
    };
    return cases;
}

std::string SourcePath(const GcpCase& c) {
    return fs::path(c.source).is_absolute() ? std::string(c.source) : std::string(DDBT_TEST_FIXTURES) + "/" + c.source;
}

class TempDir {
public:
    TempDir() {
        const char* base = std::getenv("DDBT_TEST_TMP");
        const fs::path root = base ? fs::path(base) : fs::temp_directory_path();
        path_ = root / ("ddbt-gcpcog-" + std::to_string(std::random_device{}()) + "-" + std::to_string(counter_++));
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

// Every band of every level (full resolution first) as GDAL reads it: the samples of the band,
// packed, a complex band as its two components.
std::vector<std::vector<uint8_t>> Levels(const std::string& path, tiff::SampleType& type) {
    auto r = tiff::TiffReader::Open(path);
    EXPECT_TRUE(r.ok()) << r.status().message();
    if (!r.ok()) return {};
    const tiff::TiffReader& rd = **r;
    std::vector<uint32_t> ifds{rd.MainImage()};
    for (uint32_t o : rd.Overviews()) ifds.push_back(o);
    std::vector<std::vector<uint8_t>> out;
    for (uint32_t ifd : ifds) {
        const tiff::IfdInfo& info = rd.Ifd(ifd);
        type = info.sampleType;
        std::vector<uint32_t> bands(info.samplesPerPixel);
        for (uint32_t b = 0; b < info.samplesPerPixel; ++b) bands[b] = b;
        auto w = tiff::testing::ReadWindow(rd, ifd, bands, 0, 0, info.width, info.height, 1);
        EXPECT_TRUE(w.ok()) << w.status().message();
        if (!w.ok()) return {};
        for (auto& b : *w) out.push_back(std::move(b));
    }
    return out;
}

template <typename T>
double Load(const uint8_t* p) {
    T v;
    std::memcpy(&v, p, sizeof v);
    return static_cast<double>(v);
}

double Value(tiff::SampleType t, const uint8_t* p) {
    switch (t) {
        case tiff::SampleType::kU8: return Load<uint8_t>(p);
        case tiff::SampleType::kI8: return Load<int8_t>(p);
        case tiff::SampleType::kU16: return Load<uint16_t>(p);
        case tiff::SampleType::kI16: return Load<int16_t>(p);
        case tiff::SampleType::kU32: return Load<uint32_t>(p);
        case tiff::SampleType::kI32: return Load<int32_t>(p);
        case tiff::SampleType::kF32: return Load<float>(p);
        case tiff::SampleType::kF64: return Load<double>(p);
        default: return std::numeric_limits<double>::quiet_NaN();
    }
}

class GcpCog : public testing::TestWithParam<GcpCase> {};

TEST_P(GcpCog, IsGdalsCog) {
    const GcpCase& c = GetParam();
    const std::string input = SourcePath(c);
    if (!fs::exists(input)) GTEST_SKIP() << input << " is not on this machine";
    TempDir dir;
    const std::string output = dir.File("out.tif");
    build::BuildOptions options;
    options.statistics = false;
    const Status s = build::BuildCog(input, output, options);
    if (std::string(c.name) == "no_crs") {
        // GDAL warps the GCPs as if they were in EPSG:3857; libddb refuses the raster before.
        ASSERT_FALSE(s.ok());
        EXPECT_NE(s.message().find("input file has no coordinate reference system"), std::string::npos) << s.message();
        return;
    }
    if (!c.ok) {
        ASSERT_FALSE(s.ok());
        EXPECT_NE(s.message().find(c.message), std::string::npos) << s.message();
        return;
    }
    ASSERT_TRUE(s.ok()) << s.message();

    // GDAL's grid, to the bit.
    auto src = build::OpenRasterSource(input);
    ASSERT_TRUE(src.ok());
    auto plan = WarpPlan(**src, {});
    ASSERT_TRUE(plan.ok()) << plan.status().message();
    EXPECT_EQ(plan->width, c.width);
    EXPECT_EQ(plan->height, c.height);
    for (int i = 0; i < 6; ++i) EXPECT_EQ(plan->geoTransform[static_cast<size_t>(i)], c.gt[static_cast<size_t>(i)]) << i;
    auto r = tiff::TiffReader::Open(output);
    ASSERT_TRUE(r.ok());
    auto georef = tiff::ReadGeoreferencing(**r);
    ASSERT_TRUE(georef.ok() && georef->geoTransform);
    for (int i = 0; i < 6; ++i)
        EXPECT_EQ((*georef->geoTransform)[static_cast<size_t>(i)], c.gt[static_cast<size_t>(i)]) << i;

    // GDAL's chunks (those its kernel ran: chunks without source pixels are skipped).
    auto chunks = GdalWarpChunks(**src, *plan);
    ASSERT_TRUE(chunks.ok()) << chunks.status().message();
    std::vector<std::array<int, 8>> ours;
    for (const WarpChunk& k : *chunks) ours.push_back({k.sx, k.sy, k.sw, k.sh, k.dx, k.dy, k.dw, k.dh});
    EXPECT_EQ(ours, c.chunks);

    // GDAL's pixels, every level.
    tiff::SampleType type = tiff::SampleType::kUnknown;
    const auto levels = Levels(output, type);
    if (!c.crcs.empty()) {
        ASSERT_EQ(levels.size(), c.crcs.size());
        for (size_t i = 0; i < levels.size(); ++i)
            EXPECT_EQ(libdeflate_crc32(0, levels[i].data(), levels[i].size()), c.crcs[i]) << "level / band " << i;
        return;
    }
    tiff::SampleType refType = tiff::SampleType::kUnknown;
    const auto expected = Levels(std::string(DDBT_TEST_FIXTURES) + "/gcp/gdal/" + c.name + ".tif", refType);
    ASSERT_EQ(type, refType);
    ASSERT_EQ(levels.size(), expected.size());
    const size_t ss = tiff::SampleTypeSize(type);
    const bool isFloat = type == tiff::SampleType::kF32 || type == tiff::SampleType::kF64;
    for (size_t i = 0; i < levels.size(); ++i) {
        ASSERT_EQ(levels[i].size(), expected[i].size()) << i;
        // Measured on Linux x86-64: equal, but for one ulp of Float32 where GDAL's bilinear kernel
        // rounds differently (1.5e-5 at 300). The bounds leave room for another libm.
        size_t differ = 0;
        double maxDiff = 0;
        for (size_t k = 0; k + ss <= levels[i].size(); k += ss) {
            const double a = Value(type, levels[i].data() + k), b = Value(type, expected[i].data() + k);
            const double d = std::fabs(a - b);
            if (d != 0) ++differ;
            maxDiff = std::max(maxDiff, isFloat ? d / std::max(1.0, std::fabs(b)) : d);
        }
        EXPECT_LE(maxDiff, isFloat ? 1e-6 : 1.0) << "level / band " << i;
        EXPECT_LE(static_cast<double>(differ), (isFloat ? 0.01 : 0.001) * static_cast<double>(levels[i].size() / ss))
            << "level / band " << i;
    }
}

INSTANTIATE_TEST_SUITE_P(Gdal, GcpCog, testing::ValuesIn(Cases()),
                         [](const testing::TestParamInfo<GcpCase>& i) { return std::string(i.param.name); });

TEST(SourceGeoreference, AGeoTransformMapsWithGdalsArithmetic) {
    const std::array<double, 6> gt{500000.25, 0.3, 0.07, 5000000.5, 0.05, -0.29};
    auto g = SourceGeoreference::FromGeoTransform(gt);
    ASSERT_TRUE(g.ok());
    ASSERT_TRUE(g->GeoTransform().has_value());
    std::array<double, 6> inv{};
    ASSERT_TRUE(InvertGeoTransform(gt, inv));
    EXPECT_EQ(g->PixelAffine(), inv);
    double xs[3] = {0.5, 1234.25, HUGE_VAL}, ys[3] = {0.5, 987.75, 3};
    EXPECT_EQ(g->PixelToCrs(xs, ys, 3), 1u);
    EXPECT_EQ(xs[1], gt[0] + 1234.25 * gt[1] + 987.75 * gt[2]);
    EXPECT_EQ(ys[1], gt[3] + 1234.25 * gt[4] + 987.75 * gt[5]);
    EXPECT_EQ(xs[2], HUGE_VAL);
    EXPECT_EQ(ys[2], HUGE_VAL);
    double x = xs[1], y = ys[1];
    ASSERT_TRUE(g->CrsToPixel(x, y));
    EXPECT_EQ(x, inv[0] + xs[1] * inv[1] + ys[1] * inv[2]);
    EXPECT_NEAR(x, 1234.25, 1e-6);
    EXPECT_NEAR(y, 987.75, 1e-6);
    auto singular = SourceGeoreference::FromGeoTransform({0, 1, 2, 0, 2, 4});
    ASSERT_FALSE(singular.ok());
    EXPECT_EQ(singular.status().message(), "Cannot invert geotransform");
}

TEST(SourceGeoreference, GcpsMapThroughTheirTransformAndAGeoTransformComesFirst) {
    tiff::Georeferencing georef;
    georef.crs = crs::Crs::FromEpsg(32633);
    const double gcps[6][4] = {{0, 0, 500000, 5000000},   {640, 0, 500320, 5000110}, {0, 480, 500060, 4999770},
                               {640, 480, 500390, 4999880}, {320, 160, 500200, 4999970}, {200, 400, 500160, 4999820}};
    std::vector<crs::Gcp> list;
    for (const auto& p : gcps) {
        georef.gcps.push_back({p[0], p[1], p[2], p[3], 0});
        list.push_back({p[0], p[1], p[2], p[3], 0});
    }
    auto g = SourceGeoreference::Of(georef);
    ASSERT_TRUE(g.ok()) << g.status().message();
    EXPECT_FALSE(g->GeoTransform().has_value());
    EXPECT_EQ(g->PixelAffine(), (std::array<double, 6>{0, 1, 0, 0, 0, 1}));
    auto t = crs::GcpTransform::Create(list, *georef.crs);
    ASSERT_TRUE(t.ok());
    double xs[2] = {12.5, 600.25}, ys[2] = {33.75, 470.5};
    double ex[2] = {12.5, 600.25}, ey[2] = {33.75, 470.5};
    g->PixelToCrs(xs, ys, 2);
    t->TransformPoints(ex, ey, 2);
    EXPECT_EQ(xs[0], ex[0]);
    EXPECT_EQ(ys[1], ey[1]);
    g->CrsToPixel(xs, ys, 2);
    t->TransformPoints(ex, ey, 2, crs::Direction::Inverse);
    EXPECT_EQ(xs[1], ex[1]);
    EXPECT_EQ(ys[0], ey[0]);
    // A geotransform is used before GCPs (GDAL's order), and neither is a refusal.
    georef.geoTransform = std::array<double, 6>{500000, 0.5, 0, 5000000, 0, -0.5};
    auto withGt = SourceGeoreference::Of(georef);
    ASSERT_TRUE(withGt.ok());
    EXPECT_TRUE(withGt->GeoTransform().has_value());
    georef.geoTransform.reset();
    georef.gcps.clear();
    auto none = SourceGeoreference::Of(georef);
    ASSERT_FALSE(none.ok());
    EXPECT_EQ(none.status().message(), "There is no affine transformation and no GCPs");
    // Too few GCPs: GDAL's refusal.
    georef.gcps.push_back({0, 0, 500000, 5000000, 0});
    auto one = SourceGeoreference::Of(georef);
    ASSERT_FALSE(one.ok());
    EXPECT_EQ(one.status().message(), "Failed to compute GCP transform: Not enough points available");
}

// An exact transform that bends: the approximation subdivides unevenly along the row.
void Bent(double* xs, double* ys, size_t n, char* ok) {
    for (size_t i = 0; i < n; ++i) {
        const double x = xs[i], y = ys[i];
        xs[i] = 3 + 0.4 * x + 2e-4 * x * x + 0.01 * y;
        ys[i] = 7 + 0.3 * y + 2.5 * std::sin(x / 90.0);
        ok[i] = 1;
    }
}

TEST(RowApproximator, APartOfARowIsThatPartOfTheWholeRow) {
    size_t calls = 0;
    const RowApproximator a(
        [&](double* xs, double* ys, size_t n, char* ok) {
            calls += n;
            Bent(xs, ys, n, ok);
        },
        0.125);
    for (const size_t start : {size_t{0}, size_t{37}}) {
        for (const size_t n : {size_t{5}, size_t{6}, size_t{64}, size_t{257}, size_t{1500}}) {
            std::vector<double> wx(n), wy(n);
            std::vector<char> wok(n);
            calls = 0;
            a.Points(start, n, 12.5, start, start + n, wx.data(), wy.data(), wok.data());
            const size_t wholeCalls = calls;
            EXPECT_LE(wholeCalls, n);
            double maxError = 0;
            for (size_t i = 0; i < n; ++i) {
                ASSERT_TRUE(wok[i]);
                double ex = static_cast<double>(start + i) + 0.5, ey = 12.5;
                char ok = 0;
                Bent(&ex, &ey, 1, &ok);
                maxError = std::max(maxError, std::fabs(ex - wx[i]) + std::fabs(ey - wy[i]));
            }
            EXPECT_LT(maxError, 0.5) << n;
            for (const auto& part : {std::pair<size_t, size_t>{0, 1}, {3, 17}, {n / 2, n}, {n - 1, n}}) {
                if (part.first >= part.second || part.second > n) continue;
                const size_t m = part.second - part.first;
                std::vector<double> px(m), py(m);
                std::vector<char> pok(m);
                a.Points(start, n, 12.5, start + part.first, start + part.second, px.data(), py.data(), pok.data());
                for (size_t i = 0; i < m; ++i) {
                    EXPECT_EQ(px[i], wx[part.first + i]) << n << " " << part.first + i;
                    EXPECT_EQ(py[i], wy[part.first + i]) << n << " " << part.first + i;
                }
            }
        }
    }
}

TEST(RowApproximator, ALinearTransformIsInterpolatedAndFailuresAreExact) {
    size_t calls = 0;
    const RowApproximator linear(
        [&](double* xs, double* ys, size_t n, char* ok) {
            calls += n;
            for (size_t i = 0; i < n; ++i) {
                xs[i] = 2 * xs[i] + 1;
                ys[i] = 3 * ys[i];
                ok[i] = 1;
            }
        },
        0.125);
    std::vector<double> x(1000), y(1000);
    std::vector<char> ok(1000);
    linear.Points(0, 1000, 4.5, 0, 1000, x.data(), y.data(), ok.data());
    EXPECT_EQ(calls, 3u);  // the ends and the middle
    EXPECT_EQ(x[500], 2 * 500.5 + 1);
    EXPECT_EQ(y[999], 13.5);
    // An end that fails: every point is transformed exactly, the failed ones flagged.
    const RowApproximator failing(
        [](double* xs, double* ys, size_t n, char* flags) {
            for (size_t i = 0; i < n; ++i) {
                flags[i] = xs[i] < 900;
                if (!flags[i]) xs[i] = ys[i] = HUGE_VAL;
            }
        },
        0.125);
    failing.Points(0, 1000, 4.5, 0, 1000, x.data(), y.data(), ok.data());
    EXPECT_TRUE(ok[899]);
    EXPECT_EQ(x[899], 899.5);
    EXPECT_FALSE(ok[900]);
    EXPECT_EQ(x[900], HUGE_VAL);
}

TEST(WarpChunks, MemoryAndFillRatioSplitAsGdal) {
    // Destination pixel (x, y) <-> source (x / 2 - 100, y / 2 - 50): a 512 x 256 source in the
    // middle of a 2048 x 1024 output.
    const PixelTransformer t = [](bool dstToSrc, double* xs, double* ys, size_t n, char* ok) {
        for (size_t i = 0; i < n; ++i) {
            if (dstToSrc) {
                xs[i] = xs[i] / 2 - 100;
                ys[i] = ys[i] / 2 - 50;
            } else {
                xs[i] = (xs[i] + 100) * 2;
                ys[i] = (ys[i] + 50) * 2;
            }
            ok[i] = 1;
        }
    };
    WarpChunkOptions o;
    // The source fills its windows enough (no fill-ratio split) and everything fits: one chunk.
    auto one = CollectWarpChunks(200, 100, 1024, 512, 512, 256, t, o);
    ASSERT_EQ(one.size(), 1u);
    EXPECT_EQ(one[0].sx, 0);
    EXPECT_EQ(one[0].sw, 512);
    // The whole output: the source holds a quarter of the source window -> halves (along x, the
    // longer side, on a 256 px block) until each window is filled at least by half.
    auto split = CollectWarpChunks(0, 0, 2048, 1024, 512, 256, t, o);
    ASSERT_GT(split.size(), 1u);
    // Little memory: halved further, along the longer side, on blocks.
    o.memoryLimit = 64 * 1024;
    auto small = CollectWarpChunks(0, 0, 2048, 1024, 512, 256, t, o);
    EXPECT_GT(small.size(), split.size());
    for (const auto* list : {&split, &small}) {
        uint64_t area = 0;
        for (size_t i = 0; i < list->size(); ++i) {
            const WarpChunk& c = (*list)[i];
            area += static_cast<uint64_t>(c.dw) * static_cast<uint64_t>(c.dh);
            EXPECT_GT(c.sw, 0);
            EXPECT_GT(c.sh, 0);
            if (i > 0) {
                const WarpChunk& p = (*list)[i - 1];
                EXPECT_TRUE(p.dy < c.dy || (p.dy == c.dy && p.dx < c.dx));  // GDAL's order
            }
        }
        EXPECT_LE(area, uint64_t{2048} * 1024);  // chunks without source pixels are dropped
    }
    for (const WarpChunk& c : small) {
        const double memory = (8.0 * c.sw * c.sh + 8.0 * c.dw * c.dh) / 8.0;
        EXPECT_TRUE(memory <= o.memoryLimit || (c.dw <= 2 && c.dh <= 2)) << c.dx << "," << c.dy;
    }
}

}  // namespace
}  // namespace ddbt::cog::warp
