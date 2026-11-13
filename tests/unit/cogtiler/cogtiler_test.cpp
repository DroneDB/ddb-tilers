// SPDX-License-Identifier: MPL-2.0
//
// COG tiler: GDAL RasterIO parity of the sampling plans (reference made with GDAL by
// data/gen_sampling.py), the stretch and its LUTs, the visualisation tables (colormap hashes of
// DroneDB's own golden test), statistics and their sidecar, caches, versioned sources,
// concurrency, the mask shortcut and the C API's errors. Corpus files are used when present
// (skipped otherwise).
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <typeinfo>
#include <vector>

#include "../encode/decode_helpers.h"
#include "cog/tiler/block_cache.h"
#include "cog/tiler/cog_source.h"
#include "cog/tiler/cog_tiler.h"
#include "cog/tiler/raster_io.h"
#include "cog/tiler/stats.h"
#include "cog/tiler/stretch.h"
#include "cog/tiler/tile_error.h"
#include "cog/tiler/vis.h"
#include "cog/tiler/warp.h"
#include "core/exec/thread_pool.h"
#include "core/tile/compat.h"
#include "ddbt/ddbt.h"

namespace fs = std::filesystem;
using namespace ddbt::cog::tiler;

namespace {

std::string DataDir() { return (fs::path(__FILE__).parent_path() / "data").string(); }
std::string SamplingTif() { return DataDir() + "/sampling.tif"; }

constexpr const char* kR1 = "/mnt/data/repos/test_data/ortho/brighton-beach-cog.tif";

uint64_t Fnv(const std::vector<int32_t>& v) {
    uint64_t h = 1469598103934665603ull;
    for (const int32_t x : v)
        for (int k = 0; k < 4; ++k) {
            h ^= (static_cast<uint32_t>(x) >> (8 * k)) & 0xFF;
            h *= 1099511628211ull;
        }
    return h;
}

struct SamplingCase {
    int x, y, w, h, bw, bh, level;
    uint64_t cols, rows;
    int col0, colN, row0, rowN;
};

const SamplingCase kSampling[] = {
#include "data/sampling.inc"
};

// A scratch directory of our own (removed by the fixture).
class TempDir {
public:
    TempDir() {
        const char* base = std::getenv("DDBT_TEST_TMP");
        fs::path root = base ? fs::path(base) : fs::temp_directory_path();
        path_ = root / ("ddbt-cogtiler-" + std::to_string(std::random_device{}()) + "-" + std::to_string(counter_++));
        fs::create_directories(path_);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path_, ec);
    }
    const fs::path& path() const { return path_; }

private:
    static inline std::atomic<int> counter_{0};
    fs::path path_;
};

ddbt::encode::ByteBuffer Render(const std::string& path, int z, int x, int y, ddbt::encode::TileFormat f =
                                                                                   ddbt::encode::TileFormat::kPng,
                                bool noCache = false) {
    TileRequest r;
    r.path = path;
    r.z = z;
    r.x = x;
    r.y = y;
    r.tileSize = 256;
    r.tms = true;
    r.format = f;
    r.noCache = noCache;
    return RenderTile(r);
}

std::vector<uint8_t> Bytes(const ddbt::encode::ByteBuffer& b) { return {b.data(), b.data() + b.size()}; }

// XYZ tile of a 3857 point.
std::pair<int, int> TileOf(double mx, double my, int z) {
    const double o = 20037508.342789244;
    const double span = 2 * o / std::pow(2.0, z);
    return {static_cast<int>(std::floor((mx + o) / span)), static_cast<int>(std::floor((o - my) / span))};
}

}  // namespace

// --------------------------------------------------------------------------------------- sampling

TEST(CogTilerSampling, PlanReadMatchesGdalRasterIo) {
    const auto src = CogSource::Open(SamplingTif());
    ASSERT_EQ(src->Levels().size(), 4u);
    const RasterView view(*src);
    for (const auto& c : kSampling) {
        const SamplePlan p = PlanRead(view, c.x, c.y, c.w, c.h, c.bw, c.bh);
        const std::string where = "window " + std::to_string(c.x) + "," + std::to_string(c.y) + " " +
                                  std::to_string(c.w) + "x" + std::to_string(c.h) + " -> " + std::to_string(c.bw) +
                                  "x" + std::to_string(c.bh);
        ASSERT_EQ(p.viewLevel, c.level) << where;
        ASSERT_EQ(p.cols.front(), c.col0) << where;
        ASSERT_EQ(p.cols.back(), c.colN) << where;
        ASSERT_EQ(p.rows.front(), c.row0) << where;
        ASSERT_EQ(p.rows.back(), c.rowN) << where;
        EXPECT_EQ(Fnv(p.cols), c.cols) << where;
        EXPECT_EQ(Fnv(p.rows), c.rows) << where;
    }
}

TEST(CogTilerSampling, GatheredSamplesAreThePlannedPixels) {
    // band 1 holds x | level << 12, band 2 y | level << 12: the gather must return exactly the
    // planned pixel of the planned level.
    const auto src = CogSource::Open(SamplingTif());
    const RasterView view(*src);
    for (const auto& c : {kSampling[0], kSampling[5], kSampling[14], kSampling[40]}) {
        const SamplePlan p = PlanRead(view, c.x, c.y, c.w, c.h, c.bw, c.bh);
        const size_t n = static_cast<size_t>(c.bw) * static_cast<size_t>(c.bh);
        std::vector<uint8_t> out(n * 2 * 2);
        const int bands[] = {1, 2};
        GatherBands(view, p, bands, out.data(), n * 2);
        const auto* b1 = reinterpret_cast<const uint16_t*>(out.data());
        const auto* b2 = reinterpret_cast<const uint16_t*>(out.data() + n * 2);
        for (int j = 0; j < c.bh; ++j)
            for (int i = 0; i < c.bw; ++i) {
                const size_t k = static_cast<size_t>(j) * static_cast<size_t>(c.bw) + static_cast<size_t>(i);
                ASSERT_EQ(b1[k], static_cast<uint16_t>(p.cols[static_cast<size_t>(i)] | (p.viewLevel << 12)));
                ASSERT_EQ(b2[k], static_cast<uint16_t>(p.rows[static_cast<size_t>(j)] | (p.viewLevel << 12)));
            }
    }
}

TEST(CogTilerSampling, BestLevelFollowsGdalThresholds) {
    const std::vector<std::pair<int, int>> levels = {{1000, 700}, {500, 350}, {250, 175}, {125, 88}};
    EXPECT_EQ(BestLevel(levels, 1.0), 0);
    EXPECT_EQ(BestLevel(levels, 1.5), 0);   // 2 >= 1.5 * 1.2 + 0.1
    EXPECT_EQ(BestLevel(levels, 1.6), 1);   // 2 < 2.02
    EXPECT_EQ(BestLevel(levels, 2.0), 1);
    EXPECT_EQ(BestLevel(levels, 3.3), 2);   // 4 < 3.3 * 1.2 + 0.1
    EXPECT_EQ(BestLevel(levels, 3.4), 2);
    EXPECT_EQ(BestLevel(levels, 100.0), 3);
}

TEST(CogTilerSampling, DefaultModePlansTheTileGridWithoutWindowRounding) {
    const auto src = CogSource::Open(SamplingTif());
    const RasterView view(*src);
    const auto& gt = view.GeoTransform();
    // A box exactly over pixels [100, 356) x [50, 306) at full resolution.
    const double minX = gt[0] + 100 * gt[1], maxX = gt[0] + 356 * gt[1];
    const double maxY = gt[3] + 50 * gt[5], minY = gt[3] + 306 * gt[5];
    const SamplePlan p = PlanTile(view, minX, minY, maxX, maxY, 256);
    EXPECT_EQ(p.viewLevel, 0);
    for (int i = 0; i < 256; ++i) {
        EXPECT_EQ(p.cols[static_cast<size_t>(i)], 100 + i);
        EXPECT_EQ(p.rows[static_cast<size_t>(i)], 50 + i);
    }
    // Twice as large: overview 1, still aligned.
    const SamplePlan q = PlanTile(view, minX, gt[3] + 562 * gt[5], gt[0] + 612 * gt[1], maxY, 256);
    EXPECT_EQ(q.viewLevel, 1);
    EXPECT_EQ(q.cols.front(), 50);
    EXPECT_EQ(q.cols.back(), 305);
}

// ---------------------------------------------------------------------------------------- stretch

TEST(CogTilerStretch, LutsEqualStretchToByteForEveryValue) {
    std::mt19937_64 rng(42);
    std::uniform_real_distribution<double> d(-70000, 70000);
    for (int round = 0; round < 40; ++round) {
        double lo = d(rng), hi = d(rng);
        if (round % 5 == 0) hi = lo + 1;
        if (lo > hi) std::swap(lo, hi);
        if (lo == hi) hi += 0.1;
        {
            std::vector<uint16_t> v(65536);
            for (int i = 0; i < 65536; ++i) v[static_cast<size_t>(i)] = static_cast<uint16_t>(i);
            std::vector<uint8_t> out(65536);
            StretchSamples(PixelType::kU16, reinterpret_cast<const uint8_t*>(v.data()), v.size(), lo, hi, false,
                           out.data(), 1);
            for (int i = 0; i < 65536; ++i) ASSERT_EQ(out[static_cast<size_t>(i)], StretchToByte(i, lo, hi)) << i;
        }
        {
            std::vector<int16_t> v(65536);
            for (int i = 0; i < 65536; ++i) v[static_cast<size_t>(i)] = static_cast<int16_t>(i - 32768);
            std::vector<uint8_t> out(65536);
            StretchSamples(PixelType::kI16, reinterpret_cast<const uint8_t*>(v.data()), v.size(), lo, hi, true,
                           out.data(), 1);
            for (int i = 0; i < 65536; ++i)
                ASSERT_EQ(out[static_cast<size_t>(i)], StretchToByte(i - 32768, lo, hi)) << i;
        }
        {
            std::vector<uint8_t> v(256), out(256);
            for (int i = 0; i < 256; ++i) v[static_cast<size_t>(i)] = static_cast<uint8_t>(i);
            StretchSamples(PixelType::kU8, v.data(), 256, lo / 256, hi / 256 + 1, false, out.data(), 1);
            for (int i = 0; i < 256; ++i)
                ASSERT_EQ(out[static_cast<size_t>(i)], StretchToByte(i, lo / 256, hi / 256 + 1)) << i;
        }
    }
}

TEST(CogTilerStretch, FloatsAndEdges) {
    EXPECT_EQ(StretchToByte(std::numeric_limits<double>::quiet_NaN(), 0, 10), 255);  // NaN shows as 255 (kept)
    EXPECT_EQ(StretchToByte(-5, 0, 10), 0);
    EXPECT_EQ(StretchToByte(15, 0, 10), 255);
    EXPECT_EQ(StretchToByte(5, 0, 10), 127);  // 127.5 truncated
    // viaFloat: 32-bit integers round to float before the stretch (readSelectedBands).
    const uint32_t big = 16777217u;  // 2^24 + 1: rounds to 2^24 as float
    uint8_t a = 0, b = 0;
    StretchSamples(PixelType::kU32, reinterpret_cast<const uint8_t*>(&big), 1, 16777216.0, 16777218.0, false, &a, 1);
    StretchSamples(PixelType::kU32, reinterpret_cast<const uint8_t*>(&big), 1, 16777216.0, 16777218.0, true, &b, 1);
    EXPECT_EQ(a, 127);
    EXPECT_EQ(b, 0);
}

TEST(CogTilerStretch, RangeRules) {
    BandStatistics s{0, 100, 50, 10, 1, 1};
    auto r = StretchRange(s);
    EXPECT_DOUBLE_EQ(r.first, 50 - 23.3);
    EXPECT_DOUBLE_EQ(r.second, 50 + 23.3);
    s = {0, 100, 50, 100, 1, 1};  // clamped to min / max
    r = StretchRange(s);
    EXPECT_EQ(r.first, 0);
    EXPECT_EQ(r.second, 100);
    s = {7, 7, 7, 0, 1, 1};  // empty: min..max then +1
    r = StretchRange(s);
    EXPECT_EQ(r.first, 7);
    EXPECT_EQ(r.second, 8);
}

// ------------------------------------------------------------------------------------------- vis

namespace {
uint64_t Fnv(const Colormap& c) {
    uint64_t h = 1469598103934665603ULL;
    for (const auto& e : c.entries)
        for (unsigned v : {unsigned{e.r}, unsigned{e.g}, unsigned{e.b}, unsigned{e.a}}) {
            h ^= v;
            h *= 1099511628211ULL;
        }
    return h;
}
}  // namespace

TEST(CogTilerVis, ColormapsMatchDroneDbGoldenHashes) {
    // The hashes of DroneDB's tests/unit/colormaps_test.cpp (MPL-2.0).
    struct E {
        const char* id;
        bool listed;
        uint64_t hash;
    };
    const E expected[] = {{"rdylgn", true, 0x244587b3f7c0ed99ULL},   {"discrete_ndvi", true, 0xb1abae782926740dULL},
                          {"spectral", true, 0xa6cc836132ef2cf7ULL}, {"viridis", true, 0x7b53b96533e317c0ULL},
                          {"plasma", true, 0x6ffc940a87bb54d0ULL},   {"inferno", true, 0x816cb71c5b98735bULL},
                          {"magma", true, 0xeb4b1220d345818fULL},    {"grayscale", true, 0x2ff9380f25bdbb83ULL},
                          {"ironbow", true, 0x224e4c36622c5b36ULL},  {"rainbow", true, 0xc425ac86013e4dfdULL},
                          {"bugn", true, 0x49edb8a85e01382bULL},     {"whitehot", true, 0x2ff9380f25bdbb83ULL},
                          {"blackhot", true, 0xaba423fdee0f7f83ULL}, {"arctic", true, 0x18829f5e1d4af975ULL},
                          {"lava", true, 0x5452424fad1d3d7cULL},     {"terrain", true, 0x1ee792461ddd19ddULL},
                          {"greys", true, 0xaba423fdee0f7f83ULL},    {"ryg", false, 0x2972fb019bf67c9cULL}};
    const auto& maps = AllColormaps();
    ASSERT_EQ(maps.size(), std::size(expected));
    for (size_t i = 0; i < maps.size(); ++i) {
        EXPECT_EQ(maps[i].id, expected[i].id);
        EXPECT_EQ(maps[i].listed, expected[i].listed);
        EXPECT_EQ(Fnv(maps[i]), expected[i].hash) << maps[i].id;
    }
    EXPECT_EQ(FindColormap("ryg"), nullptr);
    EXPECT_NE(FindColormap("ryg", true), nullptr);
}

TEST(CogTilerVis, FormulasAndNodata) {
    const Formula* ndvi = FindFormula("NDVI");
    ASSERT_NE(ndvi, nullptr);
    BandFilter f = ParseFilter("BGRNRe", 5);
    EXPECT_EQ(f.B, 0);
    EXPECT_EQ(f.R, 2);
    EXPECT_EQ(f.N, 3);
    EXPECT_EQ(f.Re, 4);
    std::vector<float> b(3), g(3), r = {0.1f, kNodataSentinel, 0.0f}, n = {0.5f, 0.4f, 0.0f}, re(3);
    std::vector<float*> ptrs = {b.data(), g.data(), r.data(), n.data(), re.data()};
    std::vector<float> out(3);
    ApplyFormula(*ndvi, f, ptrs, out.data(), 3, kNodataSentinel, TemperatureUnit::kCelsius);
    EXPECT_FLOAT_EQ(out[0], (0.5f - 0.1f) / (0.5f + 0.1f));
    EXPECT_EQ(out[1], kNodataSentinel);  // a nodata band
    EXPECT_EQ(out[2], kNodataSentinel);  // 0 / 0
    // Thermal formulas read band 0 and convert units.
    const Formula* kelvin = FindFormula("KELVIN");
    std::vector<float> t = {20.0f, kNodataSentinel};
    std::vector<float*> tp = {t.data()};
    std::vector<float> tk(2);
    ApplyFormula(*kelvin, BandFilter{}, tp, tk.data(), 2, kNodataSentinel, TemperatureUnit::kCelsius);
    EXPECT_FLOAT_EQ(tk[0], 293.15f);
    EXPECT_EQ(tk[1], kNodataSentinel);
    EXPECT_EQ(RenderRefusal(*ndvi, ParseFilter("RGB", 3), {"Byte", false, 3}), "Requires bands: RN");
    EXPECT_EQ(RenderRefusal(*FindFormula("SAVI"), f, {"UInt16", false, 5}),
              "SAVI needs reflectance values (0 to 1, a floating-point raster): the raster is UInt16, whose values "
              "are not reflectance");
    EXPECT_EQ(RenderRefusal(*kelvin, BandFilter{}, {"Float32", true, 1}), "");
    EXPECT_EQ(BandsRead(*FindFormula("NDRE"), f), (std::vector<int>{3, 4}));
}

TEST(CogTilerVis, ColormapApplication) {
    const Colormap* c = FindColormap("grayscale");
    const float v[] = {0.0f, 0.5f, 1.0f, 2.0f, kNodataSentinel, std::nanf("")};
    uint8_t rgba[24];
    ApplyColormap(v, rgba, 6, *c, 0.0f, 1.0f, kNodataSentinel);
    EXPECT_EQ(rgba[0], 0);
    EXPECT_EQ(rgba[4], 127);  // 0.5 * 255 truncated
    EXPECT_EQ(rgba[8], 255);
    EXPECT_EQ(rgba[12], 255);  // clamped
    EXPECT_EQ(rgba[19], 0);    // nodata transparent
    EXPECT_EQ(rgba[23], 0);    // NaN transparent
}

TEST(CogTilerVis, RescaleAndUnits) {
    EXPECT_EQ(ParseRescaleRange("1,2"), (std::pair<float, float>(1.0f, 2.0f)));
    try {
        ParseRescaleRange("1");
        FAIL();
    } catch (const TileError& e) {
        EXPECT_EQ(e.code(), TileErrorCode::kInvalidArgs);
        EXPECT_STREQ(e.what(), "Invalid rescale format: 1. Expected format: min,max");
    }
    EXPECT_EQ(ConvertRescale("0,100", TemperatureUnit::kCelsius, TemperatureUnit::kKelvin), "273.15,373.15");
    EXPECT_EQ(ConvertRescale("273.15,373.15", TemperatureUnit::kKelvin, TemperatureUnit::kCelsius), "0,100");
    EXPECT_EQ(ParseTemperatureUnit("°K"), TemperatureUnit::kKelvin);
    EXPECT_EQ(ParseTemperatureUnit("degF"), TemperatureUnit::kFahrenheit);
    EXPECT_FALSE(ParseTemperatureUnit("metre").has_value());
    EXPECT_EQ(BandSymbolOf(" Red Edge "), 'e');
    EXPECT_EQ(BandSymbolOf("near_infrared"), 'N');
    EXPECT_EQ(BandSymbolOf("Coastal"), 0);
    EXPECT_EQ(StyledBand(1, "viridis", {}), 1);
    EXPECT_EQ(StyledBand(3, "viridis", {}), 0);
    EXPECT_EQ(StyledBand(3, "viridis", {2, 2, 2}), 2);
    EXPECT_EQ(StyledBand(3, "viridis", {1, 2}), 0);
    EXPECT_EQ(StyledBand(1, "", {}), 0);
}

// Single-flight waiters (block cache, source registry) get their own copy of a TileError: same
// class, code and message, a distinct object.
TEST(CogTilerErrors, WaiterCopiesKeepClassCodeAndMessage) {
    const auto check = [](std::exception_ptr original, auto expectType) {
        const auto thrower = ddbt::cache::WaiterThrower(original);
        const void* first = nullptr;
        for (int i = 0; i < 2; ++i) {
            try {
                thrower();
                ADD_FAILURE() << "no throw";
            } catch (const TileError& e) {
                expectType(e);
                if (first == nullptr) {
                    first = &e;
                    try {
                        std::rethrow_exception(original);
                    } catch (const TileError& o) {
                        EXPECT_NE(&o, &e);
                        EXPECT_EQ(o.code(), e.code());
                        EXPECT_STREQ(o.what(), e.what());
                    }
                }
            }
        }
    };
    check(std::make_exception_ptr(TileError(TileErrorCode::kNotFound, "x.tif does not exist")),
          [](const TileError& e) {
              EXPECT_EQ(typeid(e), typeid(TileError));
              EXPECT_EQ(e.code(), TileErrorCode::kNotFound);
              EXPECT_STREQ(e.what(), "x.tif does not exist");
          });
    check(std::make_exception_ptr(NoValidPixelsError("Cannot compute statistics: no pixel of the band has a value")),
          [](const TileError& e) {
              EXPECT_EQ(typeid(e), typeid(NoValidPixelsError));
              EXPECT_EQ(e.code(), TileErrorCode::kException);
          });
}

// ----------------------------------------------------------------------------------------- stats

TEST(CogTilerStats, CompatPassInDroneDbOrder) {
    const auto src = CogSource::Open(SamplingTif());
    const auto s = ComputeBandStatistics(*src, {1, 2}, std::nullopt, ddbt::cog::stats::StatsMode::kCompat);
    ASSERT_TRUE(s[0] && s[1]);
    // Welford over rows top to bottom, each across the width: band 1 = x, band 2 = y.
    double mean = 0, m2 = 0;
    uint64_t n = 0;
    for (int y = 0; y < 700; ++y)
        for (int x = 0; x < 1000; ++x) {
            const double v = x;
            n++;
            const double delta = v - mean;
            mean += delta / static_cast<double>(n);
            m2 += delta * (v - mean);
        }
    EXPECT_EQ(s[0]->min, 0);
    EXPECT_EQ(s[0]->max, 999);
    EXPECT_EQ(s[0]->mean, mean);
    EXPECT_EQ(s[0]->stdDev, std::sqrt(m2 / static_cast<double>(n)));
    EXPECT_EQ(s[0]->validCount, 700000u);
    EXPECT_EQ(s[1]->max, 699);
    // With 0 as NoData the first column / row are left out.
    const auto o = ComputeBandStatistics(*src, {1}, 0.0);
    EXPECT_EQ(o[0]->min, 1);
    EXPECT_EQ(o[0]->validCount, 699300u);
}

TEST(CogTilerStats, CompatPassEqualsLibddbOnR5) {
    // R5 (Float32 DEM, nodata -32767) copied into a libddb-style build folder of the work tree, where
    // libddb 2.4.1 wrote its exact statistics sidecar on its first tile (parity.py). Ours must be
    // the same bits.
    const std::string r5 =
        "/mnt/data/ddb-tilers-work/prepared/cogtiler/R5/.ddb/build/"
        "159b8f07cdc8e83b8098466a497cfe4d4e9d4d79983259f25e705b5a30d55285/cog/cog.tif";
    const auto ref = fs::exists(r5) ? ReadStatsSidecar(r5, 1, std::nullopt) : std::nullopt;
    if (!ref) GTEST_SKIP() << "R5 work copy or its libddb sidecar missing";
    const auto src = CogSource::Open(r5);
    const auto s = ComputeBandStatistics(*src, {1}, std::nullopt, ddbt::cog::stats::StatsMode::kCompat);
    ASSERT_TRUE(s[0]);
    EXPECT_EQ(s[0]->min, ref->min);
    EXPECT_EQ(s[0]->max, ref->max);
    EXPECT_EQ(s[0]->mean, ref->mean);
    EXPECT_EQ(s[0]->stdDev, ref->stdDev);
    EXPECT_EQ(s[0]->validCount, ref->validCount);
    EXPECT_EQ(s[0]->pixelCount, ref->pixelCount);
}

// libddb 2.4.1's exact sidecars of its own builds (the baseline snapshot): Byte + mask (R1), RGBA warped
// to JPEG + mask (R2), Byte without mask (R3), 53 x Float32 (E1), Float32 + alpha (E2). The
// compat pass gives their bits; the exact pass the same min / max / count and a mean / std within
// Welford's rounding drift.
namespace {
const char* kBaselineBuilds = "/mnt/data/ddb-tilers-work/prepared/baseline-builds/";

struct SidecarCase {
    std::string path;
    std::vector<int> bands;
    std::vector<BandStatistics> ref;
};

std::optional<SidecarCase> BaselineCase(const std::string& id) {
    SidecarCase c;
    c.path = std::string(kBaselineBuilds) + id + "/cog.tif";
    if (!fs::exists(c.path)) return std::nullopt;
    const auto src = CogSource::Open(c.path);
    for (int b = 1; b <= src->BandCount(); ++b) {
        const auto s = ReadStatsSidecar(c.path, b, std::nullopt);
        if (!s) return std::nullopt;
        c.bands.push_back(b);
        c.ref.push_back(*s);
    }
    return c;
}

// Runs fn with a default pool of `threads` workers (0: the default count), then restores it.
template <typename F>
auto WithDefaultPool(size_t threads, F fn) {
    ddbt::exec::ShutdownDefaultPool();
    ddbt::exec::SetDefaultPoolThreads(threads);
    auto r = fn();
    ddbt::exec::ShutdownDefaultPool();
    ddbt::exec::SetDefaultPoolThreads(0);
    return r;
}
}  // namespace

TEST(CogTilerStats, CompatPassEqualsLibddbSidecars) {
    int cases = 0;
    for (const char* id : {"R1", "R2", "R3", "E1", "E2"}) {
        const auto c = BaselineCase(id);
        if (!c) continue;
        ++cases;
        const auto src = CogSource::Open(c->path);
        const auto s = ComputeBandStatistics(*src, c->bands, std::nullopt, ddbt::cog::stats::StatsMode::kCompat);
        for (size_t i = 0; i < c->bands.size(); ++i) {
            ASSERT_TRUE(s[i]) << id << " band " << c->bands[i];
            EXPECT_EQ(s[i]->min, c->ref[i].min) << id << " band " << c->bands[i];
            EXPECT_EQ(s[i]->max, c->ref[i].max) << id << " band " << c->bands[i];
            EXPECT_EQ(s[i]->mean, c->ref[i].mean) << id << " band " << c->bands[i];
            EXPECT_EQ(s[i]->stdDev, c->ref[i].stdDev) << id << " band " << c->bands[i];
            EXPECT_EQ(s[i]->validCount, c->ref[i].validCount) << id << " band " << c->bands[i];
            EXPECT_EQ(s[i]->pixelCount, c->ref[i].pixelCount) << id << " band " << c->bands[i];
        }
    }
    if (cases == 0) GTEST_SKIP() << "baseline builds missing";
}

TEST(CogTilerStats, ExactPassIsCloseToLibddbAndIndependentOfThreads) {
    using ddbt::cog::stats::StatsMode;
    const std::string r5 =
        "/mnt/data/ddb-tilers-work/prepared/cogtiler/R5/.ddb/build/"
        "159b8f07cdc8e83b8098466a497cfe4d4e9d4d79983259f25e705b5a30d55285/cog/cog.tif";
    std::vector<SidecarCase> cases;
    for (const char* id : {"R1", "R3", "E2"})
        if (auto c = BaselineCase(id)) cases.push_back(*c);
    if (fs::exists(r5)) {
        if (auto ref = ReadStatsSidecar(r5, 1, std::nullopt)) cases.push_back({r5, {1}, {*ref}});
    }
    if (cases.empty()) GTEST_SKIP() << "baseline builds missing";
    double worstMean = 0, worstStd = 0;
    for (const auto& c : cases) {
        const auto src = CogSource::Open(c.path);
        const auto run = [&] { return ComputeBandStatistics(*src, c.bands, std::nullopt, StatsMode::kExact); };
        const auto one = WithDefaultPool(1, run);
        const auto three = WithDefaultPool(3, run);
        const auto all = run();
        for (size_t i = 0; i < c.bands.size(); ++i) {
            ASSERT_TRUE(one[i] && three[i] && all[i]) << c.path;
            for (const auto* o : {&three[i], &all[i]}) {
                EXPECT_EQ((*o)->mean, one[i]->mean) << c.path;
                EXPECT_EQ((*o)->stdDev, one[i]->stdDev) << c.path;
                EXPECT_EQ((*o)->min, one[i]->min) << c.path;
                EXPECT_EQ((*o)->validCount, one[i]->validCount) << c.path;
            }
            const BandStatistics& ref = c.ref[i];
            EXPECT_EQ(all[i]->min, ref.min) << c.path;
            EXPECT_EQ(all[i]->max, ref.max) << c.path;
            EXPECT_EQ(all[i]->validCount, ref.validCount) << c.path;
            const double dm = std::abs(all[i]->mean - ref.mean) / std::abs(ref.mean);
            const double ds = std::abs(all[i]->stdDev - ref.stdDev) / ref.stdDev;
            EXPECT_LT(dm, 1e-11) << c.path << " band " << c.bands[i];
            EXPECT_LT(ds, 1e-11) << c.path << " band " << c.bands[i];
            worstMean = std::max(worstMean, dm);
            worstStd = std::max(worstStd, ds);
        }
    }
    char text[64];
    std::snprintf(text, sizeof(text), "%.3g / %.3g", worstMean, worstStd);
    RecordProperty("worst_mean_std_relative_difference", text);
}

TEST(CogTilerStats, DefaultPassIsTheExactPass) {
    // No DDBT_STATS_MODE: the two-argument overload runs the exact pass (same bits as an explicit
    // kExact), and the exact mean of the sampling raster is the correctly rounded one.
    ASSERT_EQ(std::getenv("DDBT_STATS_MODE"), nullptr) << "run the tests without DDBT_STATS_MODE";
    const auto src = CogSource::Open(SamplingTif());
    const auto d = ComputeBandStatistics(*src, {1, 2}, std::nullopt);
    const auto e = ComputeBandStatistics(*src, {1, 2}, std::nullopt, ddbt::cog::stats::StatsMode::kExact);
    for (size_t i = 0; i < 2; ++i) {
        ASSERT_TRUE(d[i] && e[i]);
        EXPECT_EQ(d[i]->mean, e[i]->mean);
        EXPECT_EQ(d[i]->stdDev, e[i]->stdDev);
        EXPECT_EQ(d[i]->min, e[i]->min);
        EXPECT_EQ(d[i]->max, e[i]->max);
        EXPECT_EQ(d[i]->validCount, e[i]->validCount);
    }
    EXPECT_EQ(d[0]->mean, 499.5);  // (999 * 1000 / 2) / 1000, exactly
    EXPECT_EQ(d[1]->mean, 349.5);
}

TEST(CogTilerStats, CompatPassIndependentOfThreads) {
    const auto c = BaselineCase("R1");
    if (!c) GTEST_SKIP() << "R1 baseline build missing";
    const auto src = CogSource::Open(c->path);
    const auto run = [&] {
        return ComputeBandStatistics(*src, c->bands, std::nullopt, ddbt::cog::stats::StatsMode::kCompat);
    };
    const auto one = WithDefaultPool(1, run);
    const auto all = run();
    for (size_t i = 0; i < c->bands.size(); ++i) {
        EXPECT_EQ(one[i]->mean, c->ref[i].mean);
        EXPECT_EQ(one[i]->stdDev, c->ref[i].stdDev);
        EXPECT_EQ(all[i]->mean, c->ref[i].mean);
    }
}

TEST(CogTilerStats, SidecarRules) {
    TempDir dir;
    const fs::path raster = dir.path() / "r.tif";
    fs::copy_file(SamplingTif(), raster);
    const fs::path side = dir.path() / "r.tif.stats.json";
    const auto write = [&](const std::string& text) {
        std::ofstream(side) << text;
        fs::last_write_time(side, fs::last_write_time(raster) + std::chrono::seconds(5));
    };
    write(R"({"bands": {"1": {"min": 1.5, "max": 2.5, "mean": 2, "std": 0.25, "valid": 7, "total": 9}},
              "overrides": {"0": {"1": {"min": 3, "max": 4, "mean": 3.5, "std": 0.5}}}, "exact": true, "version": 2})");
    auto s = ReadStatsSidecar(raster.string(), 1, std::nullopt);
    ASSERT_TRUE(s);
    EXPECT_EQ(s->min, 1.5);
    EXPECT_EQ(s->validCount, 7u);
    EXPECT_FALSE(ReadStatsSidecar(raster.string(), 2, std::nullopt));
    ASSERT_TRUE(ReadStatsSidecar(raster.string(), 1, 0.0));
    EXPECT_EQ(ReadStatsSidecar(raster.string(), 1, 0.0)->max, 4);
    EXPECT_FALSE(ReadStatsSidecar(raster.string(), 1, 1.0));
    // Not exact (a build of before version 2): ignored.
    write(R"({"bands": {"1": {"min": 1, "max": 2, "mean": 1.5, "std": 0.5}}})");
    EXPECT_FALSE(ReadStatsSidecar(raster.string(), 1, std::nullopt));
    // Older than the raster: ignored.
    write(R"({"bands": {"1": {"min": 1, "max": 2, "mean": 1.5, "std": 0.5}}, "exact": true})");
    fs::last_write_time(side, fs::last_write_time(raster) - std::chrono::seconds(5));
    EXPECT_FALSE(ReadStatsSidecar(raster.string(), 1, std::nullopt));
    EXPECT_EQ(NodataKey(std::nullopt), "");
    EXPECT_EQ(NodataKey(-32767.0), "-32767");
    EXPECT_EQ(NodataKey(0.1), "0.10000000000000001");
    EXPECT_TRUE(IsLibddbBuildArtefact(
        "/d/.ddb/build/159b8f07cdc8e83b8098466a497cfe4d4e9d4d79983259f25e705b5a30d55285/cog/cog.tif"));
    EXPECT_FALSE(IsLibddbBuildArtefact("/d/.ddb/build/159b8f07/cog/cog.tif"));
    EXPECT_FALSE(IsLibddbBuildArtefact("/d/ortho.tif"));
}

// ---------------------------------------------------------------------------------------- tiles

namespace {
// The z / x / XYZ row of a tile well inside sampling.tif (0.25 m pixels near -10240777, 5916486).
constexpr int kZ = 19;
std::pair<int, int> SamplingTile() { return TileOf(-10240777.0 + 125, 5916486.0 - 87.5, kZ); }
}  // namespace

TEST(CogTilerTiles, GreyStretchOfAUInt16Band) {
    // 2 bands: one data band (dataBandCount), UInt16: stretched over its exact min..max (0..999).
    ClearTileCache();
    const auto [x, y] = SamplingTile();
    const auto png = Render(SamplingTif(), kZ, x, y);
    const auto d = ddbt::encode::testing::DecodePng(png.data(), png.size());
    ASSERT_TRUE(d.ok);
    EXPECT_EQ(d.colorType, 0);  // grey, opaque: no alpha
    // Re-derive the expected pixels from the plan.
    const auto src = CogSource::Open(SamplingTif());
    const RasterView view(*src);
    ddbt::tile::compat::RasterTiling t(256, view.GeoTransform().data(), view.Width(), view.Height());
    ddbt::tile::compat::GQResult g;
    ASSERT_TRUE(ddbt::tile::compat::TileWindow(t, kZ, x, ddbt::tile::compat::InternalRow(y, kZ, true), g));
    ASSERT_EQ(g.w.xsize, 256);
    const SamplePlan p = PlanRead(view, g.r.x, g.r.y, g.r.xsize, g.r.ysize, g.w.xsize, g.w.ysize);
    for (int j = 0; j < 256; j += 17)
        for (int i = 0; i < 256; i += 13)
            ASSERT_EQ(d.rgba[(static_cast<size_t>(j) * 256 + static_cast<size_t>(i)) * 4],
                      StretchToByte(p.cols[static_cast<size_t>(i)] | (p.viewLevel << 12), 0, 999))
                << i << "," << j;
}

TEST(CogTilerTiles, ConcurrentRendersGiveIdenticalBytes) {
    std::vector<std::pair<std::string, std::array<int, 3>>> tiles;
    const auto [x, y] = SamplingTile();
    for (int z = 15; z <= 21; ++z)
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy) {
                const int s = z >= kZ ? 1 << (z - kZ) : 1;
                const int tx = z >= kZ ? x * s : x >> (kZ - z), ty = z >= kZ ? y * s : y >> (kZ - z);
                tiles.push_back({SamplingTif(), {z, tx + dx, ty + dy}});
            }
    if (fs::exists(kR1))
        for (int z = 18; z <= 21; ++z)
            for (int i = 0; i < 6; ++i) {
                const auto [tx, ty] = TileOf(-10240777.0 + 20 * i, 5916486.0 - 20 * i, z);
                tiles.push_back({kR1, {z, tx, ty}});
            }
    for (const auto f : {ddbt::encode::TileFormat::kPng, ddbt::encode::TileFormat::kWebp}) {
        std::vector<std::vector<uint8_t>> serial;
        for (const auto& t : tiles) serial.push_back(Bytes(Render(t.first, t.second[0], t.second[1], t.second[2], f, true)));
        ClearTileCache();
        ClearBlockCache();
        std::vector<std::thread> threads;
        std::atomic<int> bad{0};
        for (int k = 0; k < 32; ++k)
            threads.emplace_back([&, k] {
                for (size_t i = 0; i < tiles.size(); ++i) {
                    const size_t j = (i * 7 + static_cast<size_t>(k)) % tiles.size();
                    const auto& t = tiles[j];
                    if (Bytes(Render(t.first, t.second[0], t.second[1], t.second[2], f)) != serial[j]) bad++;
                }
            });
        for (auto& th : threads) th.join();
        EXPECT_EQ(bad.load(), 0);
    }
}

TEST(CogTilerTiles, CachesAndVersionedSources) {
    TempDir dir;
    const fs::path copy = dir.path() / "copy.tif";
    fs::copy_file(SamplingTif(), copy);
    ClearTileCache();
    ClearBlockCache();
    const auto [x, y] = SamplingTile();
    const auto before = BlockCacheStats();
    const auto a = Bytes(Render(copy.string(), kZ, x, y));
    const auto mid = BlockCacheStats();
    EXPECT_GT(mid.loads, before.loads);
    const auto tilesBefore = TileCacheStats();
    const auto b = Bytes(Render(copy.string(), kZ, x, y));
    EXPECT_EQ(a, b);
    EXPECT_EQ(TileCacheStats().hits, tilesBefore.hits + 1);
    EXPECT_EQ(BlockCacheStats().loads, mid.loads);  // served from the tile cache: no block touched
    // forceRecreate: rendered again from the cached blocks.
    const auto c = Bytes(Render(copy.string(), kZ, x, y, ddbt::encode::TileFormat::kPng, true));
    EXPECT_EQ(a, c);
    EXPECT_EQ(BlockCacheStats().loads, mid.loads);
    // A new version of the file is a new source.
    const auto s1 = CogSource::Get(copy.string());
    fs::last_write_time(copy, fs::last_write_time(copy) + std::chrono::seconds(10));
    const auto s2 = CogSource::Get(copy.string());
    EXPECT_NE(s1->Id(), s2->Id());
    EXPECT_EQ(CogSource::Get(copy.string())->Id(), s2->Id());
    const auto d = Bytes(Render(copy.string(), kZ, x, y));
    EXPECT_EQ(a, d);
    EXPECT_GT(BlockCacheStats().loads, mid.loads);
}

TEST(CogTilerTiles, MaskShortcutGivesBlankBytesWithoutImagery) {
    // R6, the 3.3 Gpx JPEG COG with a mask (only its mask blocks are read here).
    const char* kR6 =
        "/mnt/data/datasets/orthos/err-mask/.ddb/build/"
        "29b077eede0e6ece29a9e52f136bcc30e9f359d588947b50841996a11b2ce0ce/cog/cog.tif";
    if (!fs::exists(kR6)) GTEST_SKIP() << "corpus file missing";
    const auto src = CogSource::Get(kR6);
    ASSERT_EQ(src->Mask(), MaskKind::kPerDataset);
    const RasterView view(*src);
    ddbt::tile::compat::RasterTiling t(256, view.GeoTransform().data(), view.Width(), view.Height());
    // Find in-bounds tiles whose mask blocks are all zero.
    int found = 0;
    for (int z = 17; z <= 20 && found < 3; ++z) {
        const auto box = ddbt::tile::compat::MinMaxCoordsForZ(t.mercator, t.bounds.oMinX, t.bounds.oMinY,
                                                              t.bounds.oMaxX, t.bounds.oMaxY, z);
        for (int tx = box.min.x; tx <= box.max.x && found < 3; ++tx)
            for (int row = box.min.y; row <= box.max.y && found < 3; ++row) {
                ddbt::tile::compat::GQResult g;
                if (!ddbt::tile::compat::TileWindow(t, z, tx, row, g)) continue;
                const SamplePlan p = PlanRead(view, g.r.x, g.r.y, g.r.xsize, g.r.ysize, g.w.xsize, g.w.ysize);
                if (PlanMaskSummary(view, p) != MaskSummary::kAllZero) continue;
                ++found;
                ClearTileCache();
                ClearBlockCache();
                const int y = ddbt::tile::compat::FlipRow(row, z);
                const auto png = Bytes(Render(kR6, z, tx, y));
                const auto& blank = ddbt::encode::BlankTileBytes(256, 256, 3, {});
                EXPECT_EQ(png, blank);
                // The mask blocks under the window were decoded, the imagery blocks were not.
                const SourceLevel& lv = src->Levels()[static_cast<size_t>(p.viewLevel)];
                const auto& img = src->Reader().Ifd(lv.ifd);
                const auto& msk = src->Reader().Ifd(*lv.maskIfd);
                const uint32_t bc = static_cast<uint32_t>(p.cols.front()) / img.blockWidth;
                const uint32_t br = static_cast<uint32_t>(p.rows.front()) / img.blockHeight;
                EXPECT_TRUE(IsBlockCached(*src, *lv.maskIfd, msk.BlockIndex(bc, br)));
                EXPECT_FALSE(IsBlockCached(*src, lv.ifd, img.BlockIndex(bc, br)));
            }
    }
    EXPECT_GT(found, 0);
}

TEST(CogTilerTiles, OutOfBoundsPlainIsBlankAndVisFails) {
    const auto [x, y] = SamplingTile();
    const auto png = Bytes(Render(SamplingTif(), kZ, x + 50, y));
    EXPECT_EQ(png, ddbt::encode::BlankTileBytes(256, 256, 1, {}));
    VisParams vis;
    vis.formula = "VARI";
    TileRequest r;
    r.path = SamplingTif();
    r.z = kZ;
    r.x = x + 50;
    r.y = y;
    r.tms = true;
    r.vis = &vis;
    try {
        RenderTile(r);
        FAIL();
    } catch (const TileError& e) {
        EXPECT_STREQ(e.what(), "Out of bounds");
        EXPECT_EQ(e.code(), TileErrorCode::kException);
    }
    vis = VisParams{};
    vis.colormap = "viridis";  // no parameter: the plain (blank) tile
    EXPECT_EQ(Bytes(RenderTile(r)), ddbt::encode::BlankTileBytes(256, 256, 1, {}));
}

TEST(CogTilerTiles, StyledBandAndFormulaRefusal) {
    const auto [x, y] = SamplingTile();
    VisParams vis;
    vis.colormap = "viridis";
    TileRequest r;
    r.path = SamplingTif();
    r.z = kZ;
    r.x = x;
    r.y = y;
    r.tms = true;
    r.vis = &vis;
    const auto png = RenderTile(r);
    const auto d = ddbt::encode::testing::DecodePng(png.data(), png.size());
    ASSERT_TRUE(d.ok);
    const Colormap* cm = FindColormap("viridis");
    // The single data band painted over its min..max (0..999).
    const uint8_t* p0 = d.rgba.data();
    bool found = false;
    for (int i = 0; i < 256 && !found; ++i)
        for (const auto& e : cm->entries)
            if (p0[0] == e.r && p0[1] == e.g && p0[2] == e.b) found = true;
    EXPECT_TRUE(found);
    vis = VisParams{};
    vis.formula = "NDVI";
    try {
        RenderTile(r);
        FAIL();
    } catch (const TileError& e) {
        EXPECT_EQ(e.code(), TileErrorCode::kRefused);
        EXPECT_STREQ(e.what(), "Requires bands: RN");
    }
}

// ---------------------------------------------------------------------------------- warped grids

TEST(CogTilerWarp, VirtualGridMatchesTheCogDroneDbBuilt) {
    // Inputs and the COGs libddb 2.4.1 built for them (the artefacts of its DDB_HOME after
    // bench/micro/cogtiler/parity.py ran), when present.
    const std::string homes = "/mnt/data/ddb-tilers-work/home/cogtiler-parity";
    const std::pair<const char*, const char*> cases[] = {
        {"/mnt/data/repos/test_data/ortho/aukerman.tif", "R2"},
        {"/mnt/data/repos/test_data/ortho/vo.tif", "R3"},
        {"/mnt/data/datasets/orthos/Lamb-Hill-Farmland.Ortho.tif", "R7"},
        {"/mnt/data/datasets/orthos/moulin-hub-2cm/2cm/25.10.02_moulin_cote_2cm.tif", "R10"}};
    int checked = 0;
    for (const auto& [input, label] : cases) {
        const fs::path dir = fs::path(homes) / label / ".ddb" / "artefacts";
        std::string cog;
        std::error_code ec;
        for (const auto& e : fs::directory_iterator(dir, ec))
            if (e.path().extension() == ".tif") cog = e.path().string();
        if (!fs::exists(input) || cog.empty()) continue;
        const auto src = CogSource::Open(input);
        const auto ref = CogSource::Open(cog);
        ASSERT_NE(src->Warp(), nullptr);
        const WarpModel& w = *src->Warp();
        EXPECT_EQ(w.Width(), ref->Width()) << input;
        EXPECT_EQ(w.Height(), ref->Height()) << input;
        for (int k = 0; k < 6; ++k)
            EXPECT_NEAR(w.GeoTransform()[static_cast<size_t>(k)], ref->GeoTransform()[static_cast<size_t>(k)], 1e-9)
                << input << " gt " << k;
        ASSERT_EQ(w.Levels().size(), ref->Levels().size()) << input;
        for (size_t l = 0; l < w.Levels().size(); ++l) {
            EXPECT_EQ(w.Levels()[l].first, static_cast<int>(ref->Levels()[l].width)) << input << " level " << l;
            EXPECT_EQ(w.Levels()[l].second, static_cast<int>(ref->Levels()[l].height)) << input << " level " << l;
        }
        ++checked;
    }
    if (checked == 0) GTEST_SKIP() << "no baseline-built COG present";
}

// ------------------------------------------------------------------------------------------ API

TEST(CogTilerApi, ArgumentsAndErrors) {
    uint8_t* buf = nullptr;
    int size = 0;
    EXPECT_EQ(ddbtMemoryTileFmt("  ", 1, 0, 0, &buf, &size, 256, false, false, nullptr, "png"), DDBT_ERR_INVALIDARGS);
    EXPECT_STREQ(ddbtLastError(), "No input path provided");
    EXPECT_EQ(ddbtMemoryTileFmt("x.tif", 1, 0, 0, &buf, nullptr, 256, false, false, nullptr, "png"),
              DDBT_ERR_INVALIDARGS);
    EXPECT_STREQ(ddbtLastError(), "Output buffer size pointer is null");
    EXPECT_EQ(ddbtMemoryTileFmt("x.tif", 1, 0, 0, &buf, &size, -1, false, false, nullptr, "png"), DDBT_ERR_INVALIDARGS);
    EXPECT_STREQ(ddbtLastError(), "Invalid tile size parameter");
    EXPECT_EQ(ddbtMemoryTileFmt("x.tif", 1, -1, 0, &buf, &size, 256, false, false, nullptr, "png"),
              DDBT_ERR_INVALIDARGS);
    EXPECT_STREQ(ddbtLastError(), "Invalid tile coordinates");
    EXPECT_EQ(ddbtMemoryTileFmt("x.tif", 1, 0, 0, &buf, &size, 256, false, false, nullptr, "bmp"), DDBT_ERR_INVALIDARGS);
    EXPECT_STREQ(ddbtLastError(), "Unsupported image format: bmp");
    EXPECT_EQ(ddbtMemoryTileFmt("x.tif", 1, 0, 0, &buf, &size, 100, false, false, nullptr, "png"), DDBT_ERR_INVALIDARGS);
    EXPECT_STREQ(ddbtLastError(), "Tile size must be a power of 2 greater than 0");
    EXPECT_EQ(ddbtMemoryTileFmt("/nonexistent/x.tif", 1, 0, 0, &buf, &size, 256, false, false, nullptr, "IMAGE/JPG"),
              DDBT_ERR_NOTFOUND);
    EXPECT_STREQ(ddbtLastError(), "/nonexistent/x.tif does not exist");
    EXPECT_EQ(ddbtMemoryTileFmt("https://example.com/x.tif", 1, 0, 0, &buf, &size, 256, false, false, nullptr, "png"),
              DDBT_ERR_NOT_IMPLEMENTED);
    TempDir dir;
    const fs::path png = dir.path() / "photo.png";
    std::ofstream(png) << "\x89PNG\r\n\x1a\n not a tiff";
    EXPECT_EQ(ddbtMemoryTileFmt(png.string().c_str(), 1, 0, 0, &buf, &size, 256, false, false, nullptr, "png"),
              DDBT_ERR_NOT_IMPLEMENTED);
    EXPECT_EQ(buf, nullptr);
    EXPECT_EQ(size, 0);

    // A tile.
    const auto [x, y] = SamplingTile();
    ASSERT_EQ(ddbtMemoryTileFmt(SamplingTif().c_str(), kZ, x, y, &buf, &size, 256, true, false, nullptr, "webp"),
              DDBT_ERR_NONE);
    ASSERT_NE(buf, nullptr);
    EXPECT_GT(size, 0);
    EXPECT_EQ(std::memcmp(buf + 8, "WEBP", 4), 0);
    EXPECT_STREQ(ddbtLastError(), "");
    ddbtFree(buf);

    // Ex2 options.
    buf = nullptr;
    ddbtRasterOptions o{};
    o.structSize = sizeof(o);
    o.flags = 0x80;
    EXPECT_EQ(ddbtMemoryTileEx2(SamplingTif().c_str(), kZ, x, y, 256, true, false, nullptr, nullptr, nullptr, nullptr,
                                nullptr, nullptr, nullptr, &o, &buf, &size),
              DDBT_ERR_INVALIDARGS);
    EXPECT_STREQ(ddbtLastError(), "options->flags has an unknown bit");
    o.flags = DDBT_RASTER_OPT_NODATA;
    o.nodata = std::numeric_limits<double>::infinity();
    EXPECT_EQ(ddbtMemoryTileEx2(SamplingTif().c_str(), kZ, x, y, 256, true, false, nullptr, nullptr, nullptr, nullptr,
                                nullptr, nullptr, nullptr, &o, &buf, &size),
              DDBT_ERR_INVALIDARGS);
    EXPECT_STREQ(ddbtLastError(), "options->nodata must be a finite number");
    o.nodata = 0;
    o.structSize = 4;
    EXPECT_EQ(ddbtMemoryTileEx2(SamplingTif().c_str(), kZ, x, y, 256, true, false, nullptr, nullptr, nullptr, nullptr,
                                nullptr, nullptr, nullptr, &o, &buf, &size),
              DDBT_ERR_INVALIDARGS);
    o.structSize = sizeof(o);
    EXPECT_EQ(ddbtMemoryTileEx2(SamplingTif().c_str(), kZ, x, y, 256, true, false, nullptr, nullptr, nullptr, "NOPE",
                                nullptr, nullptr, nullptr, &o, &buf, &size),
              DDBT_ERR_INVALIDARGS);
    EXPECT_STREQ(ddbtLastError(), "Unknown formula: NOPE");
    EXPECT_EQ(ddbtMemoryTileEx2(SamplingTif().c_str(), kZ, x, y, 256, true, false, nullptr, nullptr, nullptr, nullptr,
                                nullptr, "nope", nullptr, &o, &buf, &size),
              DDBT_ERR_INVALIDARGS);
    EXPECT_STREQ(ddbtLastError(), "Unknown colormap: nope");
    ASSERT_EQ(ddbtMemoryTileEx2(SamplingTif().c_str(), kZ, x, y, 256, true, false, nullptr, nullptr, "1", nullptr,
                                nullptr, "magma", "100,800", &o, &buf, &size),
              DDBT_ERR_NONE);
    EXPECT_EQ(std::memcmp(buf + 1, "PNG", 3), 0);
    const std::vector<uint8_t> ex2Png(buf, buf + size);
    ddbtFree(buf);

    // Ex3: the same tile in the requested format, encoded once (PNG, NULL and "" are Ex2's bytes); an unknown
    // format is refused after the options, as libddb 2.5's DDBMemoryTileEx3.
    auto ex3 = [&](const char* format, ddbtErr expected) {
        uint8_t* b = nullptr;
        int n = 0;
        EXPECT_EQ(ddbtMemoryTileEx3(SamplingTif().c_str(), kZ, x, y, 256, true, false, nullptr, nullptr, "1", nullptr,
                                    nullptr, "magma", "100,800", &o, format, &b, &n),
                  expected)
            << (format ? format : "NULL") << ": " << ddbtLastError();
        std::vector<uint8_t> v;
        if (b != nullptr) v.assign(b, b + n);
        ddbtFree(b);
        return v;
    };
    for (const char* f : {static_cast<const char*>(nullptr), "", "png", "image/png"})
        EXPECT_EQ(ex3(f, DDBT_ERR_NONE), ex2Png) << (f ? f : "NULL");
    const std::vector<uint8_t> webp = ex3("webp", DDBT_ERR_NONE);
    ASSERT_GT(webp.size(), 12u);
    EXPECT_EQ(std::memcmp(webp.data() + 8, "WEBP", 4), 0);
    EXPECT_EQ(ex3("image/webp", DDBT_ERR_NONE), webp);
    const std::vector<uint8_t> jpeg = ex3("jpg", DDBT_ERR_NONE);
    ASSERT_GT(jpeg.size(), 2u);
    EXPECT_EQ(jpeg[0], 0xFF);
    EXPECT_EQ(jpeg[1], 0xD8);
    EXPECT_TRUE(ex3("bmp", DDBT_ERR_INVALIDARGS).empty());
    EXPECT_STREQ(ddbtLastError(), "Unsupported image format: bmp");
    o.flags = 0x80;  // the options are checked first
    EXPECT_TRUE(ex3("bmp", DDBT_ERR_INVALIDARGS).empty());
    EXPECT_STREQ(ddbtLastError(), "options->flags has an unknown bit");
}

// ------------------------------------------------------------------------------------- robustness

TEST(CogTilerTiles, TinyCachesEvictAndStayDeterministic) {
    // Budgets of a few blocks force evictions and re-decodes while 32 threads overlap on the same
    // tiles; the bytes must equal the serial render with the default budgets.
    std::vector<std::array<int, 3>> tiles;
    const auto [x, y] = SamplingTile();
    for (int z = 16; z <= 21; ++z)
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy) {
                const int s = z >= kZ ? 1 << (z - kZ) : 1;
                tiles.push_back({z, (z >= kZ ? x * s : x >> (kZ - z)) + dx, (z >= kZ ? y * s : y >> (kZ - z)) + dy});
            }
    const size_t savedBlocks = BlockCacheBudget();
    std::vector<std::vector<uint8_t>> serial;
    for (const auto& t : tiles) serial.push_back(Bytes(Render(SamplingTif(), t[0], t[1], t[2], ddbt::encode::TileFormat::kPng, true)));
    for (const size_t budget : {size_t{1} << 16, size_t{1} << 20}) {
        ClearTileCache();
        ClearBlockCache();
        SetBlockCacheBudget(budget);
        SetTileCacheBudget(budget);
        std::vector<std::thread> threads;
        std::atomic<int> bad{0};
        for (int k = 0; k < 32; ++k)
            threads.emplace_back([&, k] {
                for (size_t i = 0; i < tiles.size(); ++i) {
                    const size_t j = (i * 5 + static_cast<size_t>(k) * 3) % tiles.size();
                    const auto& t = tiles[j];
                    if (Bytes(Render(SamplingTif(), t[0], t[1], t[2])) != serial[j]) bad++;
                }
            });
        for (auto& th : threads) th.join();
        EXPECT_EQ(bad.load(), 0) << "budget " << budget;
    }
    SetBlockCacheBudget(savedBlocks);
    SetTileCacheBudget(size_t{256} << 20);
    ClearTileCache();
    ClearBlockCache();
}

TEST(CogTilerApi, CorruptedCopiesFailCleanly) {
    // Truncated, bit-flipped and zeroed copies of a real COG: every call returns a tile or a
    // clean error (message set, no buffer), and never crashes (run it under ASan / UBSan).
    std::ifstream in(SamplingTif(), std::ios::binary);
    const std::vector<char> original((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    ASSERT_GT(original.size(), size_t{1000});
    TempDir dir;
    std::mt19937 rng(20240611);
    const auto [tx, ty] = SamplingTile();
    int copies = 0, errors = 0, tiles = 0;
    const auto probe = [&](const std::vector<char>& bytes) {
        const std::string path = (dir.path() / ("c" + std::to_string(copies++) + ".tif")).string();
        std::ofstream(path, std::ios::binary).write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
        for (const int variant : {0, 1}) {
            uint8_t* buf = nullptr;
            int size = -1;
            const ddbtErr e = variant == 0
                                  ? ddbtMemoryTileFmt(path.c_str(), kZ, tx, ty, &buf, &size, 256, true, false, nullptr, "png")
                                  : ddbtMemoryTileEx2(path.c_str(), kZ - 2, tx >> 2, ty >> 2, 256, true, false, nullptr, nullptr, "1",
                                                      nullptr, nullptr, "viridis", nullptr, nullptr, &buf, &size);
            if (e == DDBT_ERR_NONE) {
                ASSERT_NE(buf, nullptr);
                EXPECT_GT(size, 0);
                ++tiles;
            } else {
                EXPECT_EQ(buf, nullptr);
                EXPECT_EQ(size, -1);
                ASSERT_NE(ddbtLastError(), nullptr);
                EXPECT_NE(std::string(ddbtLastError()), "") << "code " << e;
                ++errors;
            }
            ddbtFree(buf);
        }
    };
    for (const size_t keep : {size_t{0}, size_t{8}, size_t{100}, original.size() / 4, original.size() / 2,
                              original.size() - 1})
        probe(std::vector<char>(original.begin(), original.begin() + static_cast<std::ptrdiff_t>(keep)));
    for (int i = 0; i < 60; ++i) {
        std::vector<char> m = original;
        const int flips = 1 + static_cast<int>(rng() % 4);
        for (int f = 0; f < flips; ++f) {
            // Half of the flips land in the header / IFDs, the rest anywhere.
            const size_t pos = (i % 2 == 0) ? rng() % std::min<size_t>(m.size(), 2048) : rng() % m.size();
            m[pos] = static_cast<char>(m[pos] ^ (1 << (rng() % 8)));
        }
        probe(m);
    }
    for (int i = 0; i < 6; ++i) {
        std::vector<char> m = original;
        const size_t start = rng() % m.size(), len = std::min<size_t>(m.size() - start, 64u << (i % 4));
        std::fill(m.begin() + static_cast<std::ptrdiff_t>(start), m.begin() + static_cast<std::ptrdiff_t>(start + len), '\0');
        probe(m);
    }
    EXPECT_GT(errors, 0);  // the corruption is detected, not silently rendered
    EXPECT_GT(tiles, 0);   // and harmless flips still tile
}
