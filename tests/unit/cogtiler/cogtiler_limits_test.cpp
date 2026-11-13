// SPDX-License-Identifier: MPL-2.0
//
// Regression tests of the COG tiler's lifetimes, bounds and identities: sensor detections held
// by callers, statistics keyed by file identity, band-count and request limits, the formula
// reading only the bands it needs, tile size and coordinate checks, the source registry's file
// identity, UTF-8 paths, sidecar writes and the encoded-tile key.
#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "../encode/decode_helpers.h"
#include "cog/tiler/block_cache.h"
#include "cog/tiler/cog_source.h"
#include "cog/tiler/cog_tiler.h"
#include "cog/tiler/stats.h"
#include "cog/tiler/tile_error.h"
#include "cog/tiler/vis.h"
#include "core/tile/compat.h"
#include "ddbt/ddbt.h"
#include "../test_env.h"
#include "synth_raster.h"

namespace fs = std::filesystem;
using namespace ddbt::cog::tiler;
using namespace ddbt::cog::tiler::testing;

namespace {

std::string SamplingTif() { return (fs::path(__FILE__).parent_path() / "data" / "sampling.tif").string(); }

class TempDir {
public:
    TempDir() {
        const char* base = std::getenv("DDBT_TEST_TMP");
        const fs::path root = base ? fs::path(base) : fs::temp_directory_path();
        path_ = root / ("ddbt-coglimits-" + std::to_string(std::random_device{}()) + "-" + std::to_string(counter_++));
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

using ddbt::test::ScopedEnv;

// XYZ tile of a 3857 point.
std::pair<int, int> TileOf(double mx, double my, int z) {
    const double o = 20037508.342789244;
    const double span = 2 * o / std::pow(2.0, z);
    return {static_cast<int>(std::floor((mx + o) / span)), static_cast<int>(std::floor((o - my) / span))};
}

// A z20 tile inside the synthetic rasters (38 m tiles, 64 m rasters).
constexpr int kSynthZ = 20;
std::pair<int, int> SynthTile() { return TileOf(kOriginX + 32, kOriginY - 32, kSynthZ); }

// A tile inside sampling.tif (0.25 m pixels near the same origin).
constexpr int kSamplingZ = 19;
std::pair<int, int> SamplingTile() { return TileOf(-10240777.0 + 125, 5916486.0 - 87.5, kSamplingZ); }

TileRequest Request(const std::string& path, int z, std::pair<int, int> xy, const VisParams* vis = nullptr) {
    TileRequest r;
    r.path = path;
    r.z = z;
    r.x = xy.first;
    r.y = xy.second;
    r.tileSize = 256;
    r.tms = true;
    r.vis = vis;
    return r;
}

}  // namespace

// ------------------------------------------------------------------------------- sensor detection

TEST(CogTilerLimits, SensorDetectionStaysValidWhileHeld) {
    // The detection of a source is held by its caller while many other sources are detected (and
    // released): it must stay alive and unchanged.
    const auto src = CogSource::Open(SamplingTif());
    const std::shared_ptr<const SensorDetection> held = DetectSensor(*src);
    ASSERT_NE(held, nullptr);
    const bool detected = held->detected;
    const std::string sensor = held->sensorId;
    const size_t bands = held->bands.size();
    for (int i = 0; i < 1100; ++i) {
        const auto other = CogSource::Open(SamplingTif());
        EXPECT_NE(DetectSensor(*other), nullptr);
    }
    EXPECT_EQ(held->detected, detected);
    EXPECT_EQ(held->sensorId, sensor);
    EXPECT_EQ(held->bands.size(), bands);
    // Memoised per source: the same object on each call.
    EXPECT_EQ(DetectSensor(*src).get(), held.get());
}

TEST(CogTilerLimits, SensorDetectionUnderConcurrentSources) {
    // Readers resolve bands of one source while other threads detect fresh sources.
    const auto src = CogSource::Open(SamplingTif());
    std::atomic<bool> stop{false};
    std::atomic<int> bad{0};
    std::vector<std::thread> readers;
    for (int k = 0; k < 4; ++k)
        readers.emplace_back([&] {
            VisParams vis;
            while (!stop.load()) {
                if (!ResolveVisBands(*src, vis, src->DataBandCount()).empty()) bad++;
                if (AutoDetectFilter(*src).id != "") bad++;
            }
        });
    std::vector<std::thread> writers;
    for (int k = 0; k < 4; ++k)
        writers.emplace_back([&] {
            for (int i = 0; i < 400; ++i) DetectSensor(*CogSource::Open(SamplingTif()));
        });
    for (auto& t : writers) t.join();
    stop = true;
    for (auto& t : readers) t.join();
    EXPECT_EQ(bad.load(), 0);
}

// ------------------------------------------------------------------------------------- statistics

TEST(CogTilerLimits, StatisticsAreKeyedByFileIdentity) {
    TempDir dir;
    const fs::path copy = dir.path() / "s.tif";
    fs::copy_file(SamplingTif(), copy);
    ClearStatsCache();
    const auto a = CogSource::Open(copy.string());
    const auto sa = GetBandStatistics(*a, 1, std::nullopt);
    const uint64_t computed = GetStatsCounters().computed;
    // Another source object of the same file version: the same statistics, not computed again.
    const auto b = CogSource::Open(copy.string());
    ASSERT_NE(a->Id(), b->Id());
    const auto sb = GetBandStatistics(*b, 1, std::nullopt);
    EXPECT_EQ(GetStatsCounters().computed, computed);
    EXPECT_EQ(sa.mean, sb.mean);
    // A new version of the file: computed again.
    fs::last_write_time(copy, fs::last_write_time(copy) + std::chrono::seconds(10));
    const auto c = CogSource::Open(copy.string());
    GetBandStatistics(*c, 1, std::nullopt);
    EXPECT_GT(GetStatsCounters().computed, computed);
}

TEST(CogTilerLimits, StatisticsCacheIsBounded) {
    ClearStatsCache();
    const auto src = CogSource::Open(SamplingTif());
    // Distinct NoData overrides of one file: at most kMaxStatsOverridesPerFile kept.
    for (int k = 0; k < 40; ++k) GetBandStatistics(*src, 1, 1000.0 + k);
    EXPECT_LE(StatsCacheEntryCount(), kMaxStatsNodataKeysPerFile);
    // The first override was evicted: computed again.
    const uint64_t computed = GetStatsCounters().computed;
    GetBandStatistics(*src, 1, 1000.0);
    EXPECT_GT(GetStatsCounters().computed, computed);
    // Many files: at most kMaxStatsFiles kept.
    TempDir dir;
    for (size_t k = 0; k < kMaxStatsFiles + 8; ++k) {
        const fs::path p = dir.path() / ("f" + std::to_string(k) + ".tif");
        WriteSynth(p, Spec(16, 1));
        GetBandStatistics(*CogSource::Open(p.string()), 1, std::nullopt);
    }
    EXPECT_LE(StatsCacheEntryCount(), kMaxStatsFiles * kMaxStatsNodataKeysPerFile);
    EXPECT_LE(StatsCacheFileCount(), kMaxStatsFiles);
    ClearStatsCache();
}

// ---------------------------------------------------------------------------- bands and memory

TEST(CogTilerLimits, BandCountIsCappedAtOpen) {
    TempDir dir;
    const fs::path p = dir.path() / "many.tif";
    WriteSynth(p, Spec(16, 1025, 1));
    try {
        CogSource::Open(p.string());
        FAIL() << "a 1025-band raster opened";
    } catch (const TileError& e) {
        EXPECT_EQ(e.code(), TileErrorCode::kException);
        EXPECT_NE(std::string(e.what()).find("1025 bands"), std::string::npos) << e.what();
        EXPECT_NE(std::string(e.what()).find("DDBT_TILER_MAX_BANDS"), std::string::npos) << e.what();
    }
    // Configurable.
    const fs::path five = dir.path() / "five.tif";
    WriteSynth(five, Spec(16, 5, 1));
    EXPECT_NO_THROW(CogSource::Open(five.string()));
    {
        ScopedEnv env("DDBT_TILER_MAX_BANDS", "4");
        EXPECT_THROW(CogSource::Open(five.string()), TileError);
    }
    {
        ScopedEnv env("DDBT_TILER_MAX_BANDS", "1100");
        EXPECT_NO_THROW(CogSource::Open(p.string()));
    }
}

TEST(CogTilerLimits, FormulaReadsOnlyTheBandsItNeeds) {
    // 8 band-interleaved Float32 bands without NoData: VARI over bands 1-3 decodes 3 planes.
    TempDir dir;
    const fs::path p = dir.path() / "planar.tif";
    SynthSpec spec = Spec(256, 8);
    spec.planar = true;
    spec.value = [](int b, uint32_t x, uint32_t) {
        return 0.1f + 0.05f * static_cast<float>(b) + 0.001f * static_cast<float>(x);
    };
    WriteSynth(p, spec);
    VisParams vis;
    vis.formula = "VARI";
    vis.bandFilter = "RGB";
    ClearTileCache();
    ClearBlockCache();
    const uint64_t before = BlockCacheStats().loads;
    auto r = Request(p.string(), kSynthZ, SynthTile(), &vis);
    r.noCache = true;
    const auto png = RenderTile(r);
    EXPECT_GT(png.size(), 0u);
    EXPECT_EQ(BlockCacheStats().loads - before, 3u);
}

TEST(CogTilerLimits, FormulaStillMasksNodataOfBandsItDoesNotUse) {
    // NodataMask::AnyBand: band 5 (not in the formula) has the NoData value on the left quarter
    // of the raster; those pixels are transparent although bands 1-3 have values there.
    TempDir dir;
    const fs::path p = dir.path() / "nd.tif";
    SynthSpec spec = Spec(256, 5);
    spec.nodata = "-1";
    spec.value = [](int b, uint32_t x, uint32_t) {
        if (b == 4) return x < 64 ? -1.0f : 1.0f;
        return 0.2f + 0.1f * static_cast<float>(b);
    };
    WriteSynth(p, spec);
    VisParams vis;
    vis.formula = "VARI";
    vis.bandFilter = "RGB";
    vis.colormap = "viridis";
    vis.rescale = "-1,1";
    auto r = Request(p.string(), kSynthZ, SynthTile(), &vis);
    r.noCache = true;
    const auto png = RenderTile(r);
    const auto d = ddbt::encode::testing::DecodePng(png.data(), png.size());
    ASSERT_TRUE(d.ok);
    // Tile pixels over raster columns 0..63 and 64..255 of the first raster row in the tile.
    ddbt::tile::compat::RasterTiling t(256, std::array<double, 6>{kOriginX, kPixel, 0, kOriginY, 0, -kPixel}.data(), 256,
                                       256);
    const auto [x, y] = SynthTile();
    ddbt::tile::compat::GQResult g;
    ASSERT_TRUE(ddbt::tile::compat::TileWindow(t, kSynthZ, x, ddbt::tile::compat::InternalRow(y, kSynthZ, true), g));
    int masked = 0, shown = 0;
    for (int j = g.w.y; j < g.w.y + g.w.ysize; ++j)
        for (int i = g.w.x; i < g.w.x + g.w.xsize; ++i) {
            // The raster column of tile pixel i (nearest, no overview: 256 px tile over ~153 px).
            const double col = g.r.x + (i - g.w.x + 0.5) * g.r.xsize / g.w.xsize;
            const uint8_t a = d.rgba[(static_cast<size_t>(j) * 256 + static_cast<size_t>(i)) * 4 + 3];
            if (col < 63.5) {
                EXPECT_EQ(a, 0) << i << "," << j;
                masked++;
            } else if (col > 64.5) {
                EXPECT_EQ(a, 255) << i << "," << j;
                shown++;
            }
        }
    EXPECT_GT(masked, 0);
    EXPECT_GT(shown, 0);
}

TEST(CogTilerLimits, RequestMemoryCap) {
    ScopedEnv env("DDBT_TILE_REQUEST_MB", "1");
    VisParams vis;
    vis.formula = "VARI";
    vis.bandFilter = "RGB";
    TempDir dir;
    const fs::path p = dir.path() / "f.tif";
    WriteSynth(p, Spec(256, 3));
    auto r = Request(p.string(), kSynthZ, SynthTile(), &vis);
    r.noCache = true;
    r.tileSize = 1024;  // ~30 MB of working buffers
    const auto [x, y] = TileOf(kOriginX + 32, kOriginY - 32, kSynthZ - 2);
    r.z = kSynthZ - 2;
    r.x = x;
    r.y = y;
    try {
        RenderTile(r);
        FAIL() << "a request over the memory cap rendered";
    } catch (const TileError& e) {
        EXPECT_NE(std::string(e.what()).find("DDBT_TILE_REQUEST_MB"), std::string::npos) << e.what();
    }
}

// ------------------------------------------------------------------------------- tile size, coords

TEST(CogTilerLimits, TileSizeIsCappedAndScratchShrinks) {
    uint8_t* buf = nullptr;
    int size = 0;
    const auto [x, y] = SamplingTile();
    EXPECT_EQ(ddbtMemoryTileFmt(SamplingTif().c_str(), kSamplingZ, x, y, &buf, &size, 8192, true, true, nullptr, "png"),
              DDBT_ERR_INVALIDARGS);
    EXPECT_STREQ(ddbtLastError(), "Tile size must be at most 4096");
    EXPECT_EQ(buf, nullptr);
    // The largest size renders, and the thread's working buffers are released afterwards.
    ASSERT_EQ(ddbtMemoryTileFmt(SamplingTif().c_str(), kSamplingZ, x, y, &buf, &size, 4096, true, true, nullptr, "png"),
              DDBT_ERR_NONE);
    ddbtFree(buf);
    auto r = Request(SamplingTif(), kSamplingZ, SamplingTile());
    r.tileSize = 4096;
    r.noCache = true;
    RenderTile(r);
    EXPECT_LE(TileScratchBytes(), size_t{2} * 512 * 512 * 4);
    r.tileSize = 512;
    RenderTile(r);
    EXPECT_GT(TileScratchBytes(), 0u);
    EXPECT_LE(TileScratchBytes(), size_t{2} * 512 * 512 * 4);
}

TEST(CogTilerLimits, TileCoordinatesAreCheckedBeforeTheMaths) {
    uint8_t* buf = nullptr;
    int size = 0;
    const std::string path = SamplingTif();
    const char* p = path.c_str();
    EXPECT_EQ(ddbtMemoryTileFmt(p, 31, 0, 0, &buf, &size, 256, true, false, nullptr, "png"), DDBT_ERR_INVALIDARGS);
    EXPECT_STREQ(ddbtLastError(), "Invalid tile coordinates");
    EXPECT_EQ(ddbtMemoryTileFmt(p, 1, 2, 0, &buf, &size, 256, true, false, nullptr, "png"), DDBT_ERR_INVALIDARGS);
    EXPECT_STREQ(ddbtLastError(), "Invalid tile coordinates");
    EXPECT_EQ(ddbtMemoryTileFmt(p, 1, 0, 2, &buf, &size, 256, false, false, nullptr, "png"), DDBT_ERR_INVALIDARGS);
    EXPECT_EQ(ddbtMemoryTileEx2(p, 40, 0, 0, 256, true, false, nullptr, nullptr, "1", nullptr, nullptr, nullptr, nullptr,
                                nullptr, &buf, &size),
              DDBT_ERR_INVALIDARGS);
    EXPECT_STREQ(ddbtLastError(), "Invalid tile coordinates");
    EXPECT_EQ(buf, nullptr);
    // The edges of the grid are fine.
    ASSERT_EQ(ddbtMemoryTileFmt(p, 30, (1 << 30) - 1, (1 << 30) - 1, &buf, &size, 256, true, false, nullptr, "png"),
              DDBT_ERR_NONE);
    ddbtFree(buf);
    buf = nullptr;
    ASSERT_EQ(ddbtMemoryTileFmt(p, 0, 0, 0, &buf, &size, 256, true, false, nullptr, "png"), DDBT_ERR_NONE);
    ddbtFree(buf);
}

TEST(CogTilerLimits, GeoQueryRejectsWindowsBeyondInt) {
    namespace compat = ddbt::tile::compat;
    const double gt[6] = {0, 1e-6, 0, 0, 0, -1e-6};
    // A window of 4e13 pixels: not representable.
    EXPECT_FALSE(compat::GeoQuery(gt, 1000, 1000, 0, 0, 4e7, -4e7, 256).has_value());
    EXPECT_FALSE(compat::GeoQuery(gt, 1000, 1000, -4e7, 4e7, 0, 0, 256).has_value());
    const double nanGt[6] = {std::nan(""), 1, 0, 0, 0, -1};
    EXPECT_FALSE(compat::GeoQuery(nanGt, 10, 10, 0, 0, 1, -1, 256).has_value());
    // A window far left of the raster whose placement would not fit: no window.
    const double unit[6] = {0, 1, 0, 0, 0, -1};
    EXPECT_FALSE(compat::GeoQuery(unit, 10, 10, -2.1e9, 0, -2.1e9 + 1, -1, 4096).has_value());
    // In range: unchanged (the baseline's truncations: x -4.999 -> -4, size 10.5 -> 10).
    const auto q = compat::GeoQuery(unit, 10, 10, -5, 0, 5, -10, 256);
    ASSERT_TRUE(q.has_value());
    EXPECT_EQ(q->r.x, 0);
    EXPECT_EQ(q->r.xsize, 6);
    EXPECT_EQ(q->w.x, 102);
    EXPECT_EQ(q->w.xsize, 154);
    // Zoom maths past int: saturated, not undefined.
    const compat::GlobalMercator m(256);
    EXPECT_EQ(compat::FlipRow(0, 40), std::numeric_limits<int>::max());
    const auto box = compat::MinMaxCoordsForZ(m, 0, 0, 1, 1, 40);
    EXPECT_EQ(box.max.x, std::numeric_limits<int>::max());
}

TEST(CogTilerLimits, RasterWiderThanIntIsRefused) {
    // 2^31 x 1, one strip (never read): the sizes the tiler computes with are ints.
    using ddbt::cog::tiff::testing::TiffBuilder;
    TempDir dir;
    TiffBuilder t(false, false);
    const uint64_t data = t.AddData(std::vector<uint8_t>(16, 0));
    t.AddIfd({t.Long(256, {0x80000000u}), t.Long(257, {1}), t.Short(258, {8}), t.Short(259, {1}), t.Short(262, {1}),
              t.Short(277, {1}), t.Long(273, {static_cast<uint32_t>(data)}), t.Long(278, {1}),
              t.Long(279, {0x80000000u}), t.Double(33550, {kPixel, kPixel, 0}),
              t.Double(33922, {0, 0, 0, kOriginX, kOriginY, 0}),
              t.Short(34735, {1, 1, 0, 3, 1024, 0, 1, 1, 1025, 0, 1, 1, 3072, 0, 1, 3857})});
    const fs::path p = dir.path() / "wide.tif";
    std::ofstream(p, std::ios::binary).write(reinterpret_cast<const char*>(t.bytes().data()),
                                              static_cast<std::streamsize>(t.bytes().size()));
    try {
        CogSource::Open(p.string());
        FAIL() << "a raster wider than INT_MAX opened";
    } catch (const TileError& e) {
        EXPECT_EQ(e.code(), TileErrorCode::kException);
        EXPECT_NE(std::string(e.what()).find("up to 2147483647 pixels a side"), std::string::npos) << e.what();
    }
}

// --------------------------------------------------------------------------------------- registry

TEST(CogTilerLimits, ReplacedFileWithTheSameSizeAndMtimeIsANewSource) {
    TempDir dir;
    const fs::path a = dir.path() / "a.tif";
    const fs::path b = dir.path() / "b.tif";
    fs::copy_file(SamplingTif(), a);
    const auto s1 = CogSource::Get(a.string());
    // Another file of the same size with the same mtime, renamed over the first.
    fs::copy_file(SamplingTif(), b);
    fs::last_write_time(b, fs::last_write_time(a));
    fs::rename(b, a);
    const auto s2 = CogSource::Get(a.string());
    EXPECT_NE(s1->Id(), s2->Id());
    EXPECT_EQ(CogSource::Get(a.string())->Id(), s2->Id());
}

TEST(CogTilerLimits, SpellingsOfOnePathShareASource) {
    TempDir dir;
    fs::create_directories(dir.path() / "sub");
    const fs::path a = dir.path() / "a.tif";
    fs::copy_file(SamplingTif(), a);
    const auto s1 = CogSource::Get(a.string());
    const auto s2 = CogSource::Get((dir.path() / "." / "a.tif").string());
    const auto s3 = CogSource::Get((dir.path() / "sub" / ".." / "a.tif").string());
    EXPECT_EQ(s1->Id(), s2->Id());
    EXPECT_EQ(s1->Id(), s3->Id());
    EXPECT_EQ(s1->CanonicalPath(), fs::weakly_canonical(a).string());
}

TEST(CogTilerLimits, Utf8Paths) {
    // The magic check and the reads take UTF-8 paths (on Windows: not the ANSI code page).
    TempDir dir;
    const std::u8string base = dir.path().u8string();
    const std::string root(base.begin(), base.end());
    const std::string name = "citt\xC3\xA0 \xE2\x9C\x93.tif";  // "città ✓.tif"
    const fs::path p = dir.path() / fs::path(std::u8string(name.begin(), name.end()));
    fs::copy_file(SamplingTif(), p);
    const std::string utf8 = root + "/" + name;
    EXPECT_EQ(CogSource::Get(utf8)->BandCount(), 2);
    const std::string png = root + "/\xC3\xA9t\xC3\xA9.png";
    std::ofstream(fs::path(std::u8string(png.begin(), png.end())), std::ios::binary) << "\x89PNG\r\n\x1a\n not a tiff";
    try {
        CogSource::Get(png);
        FAIL();
    } catch (const TileError& e) {
        EXPECT_EQ(e.code(), TileErrorCode::kNotImplemented);
    }
}

// ---------------------------------------------------------------------------------------- sidecar

TEST(CogTilerLimits, ConcurrentSidecarWritesLeaveValidJsonAndNoTemporaries) {
    TempDir dir;
    const fs::path build = dir.path() / ".ddb" / "build" /
                           "159b8f07cdc8e83b8098466a497cfe4d4e9d4d79983259f25e705b5a30d55285" / "cog";
    fs::create_directories(build);
    const fs::path raster = build / "cog.tif";
    fs::copy_file(SamplingTif(), raster);
    ScopedEnv env("DDBT_STATS_SIDECAR", "write");
    ClearStatsCache();
    std::vector<std::thread> threads;
    for (int k = 0; k < 8; ++k)
        threads.emplace_back([&, k] { GetBandStatistics(*CogSource::Open(raster.string()), 1, 100.0 + k); });
    for (auto& t : threads) t.join();
    // Every write left a parsable sidecar (the last one wins; lost entries are computed again).
    int found = 0;
    for (int k = 0; k < 8; ++k)
        if (ReadStatsSidecar(raster.string(), 1, 100.0 + k)) found++;
    EXPECT_GE(found, 1);
    for (const auto& e : fs::directory_iterator(build))
        EXPECT_NE(e.path().extension(), ".tmp") << "temporary left: " << e.path();
    ClearStatsCache();
}

// ----------------------------------------------------------------------------------- tile cache key

TEST(CogTilerLimits, StyleKeyFieldsAreUnambiguous) {
    // bands "1|f:" + formula "" and bands "1" + formula "|f:" joined the same way: the second
    // (an unknown formula) must not be served the first one's cached tile.
    ClearTileCache();
    VisParams a;
    a.bands = "1|f:";
    const auto ok = RenderTile(Request(SamplingTif(), kSamplingZ, SamplingTile(), &a));
    EXPECT_GT(ok.size(), 0u);
    VisParams b;
    b.bands = "1";
    b.formula = "|f:";
    try {
        RenderTile(Request(SamplingTif(), kSamplingZ, SamplingTile(), &b));
        FAIL() << "served another style's tile";
    } catch (const TileError& e) {
        EXPECT_EQ(e.code(), TileErrorCode::kRefused);
    }
}

// The default block-cache budget: 3 % of the physical memory between 64 MiB and 1 GiB, 1 GiB
// when the memory is unknown.
TEST(BlockCacheBudget, ScalesWithPhysicalMemory) {
    constexpr uint64_t MiB = uint64_t{1} << 20, GiB = uint64_t{1} << 30;
    EXPECT_EQ(BlockCacheBudgetFor(0), GiB);
    EXPECT_EQ(BlockCacheBudgetFor(1 * GiB), 64 * MiB);  // 30.7 MiB -> the floor
    EXPECT_EQ(BlockCacheBudgetFor(4 * GiB), 4 * GiB * 3 / 100);
    EXPECT_EQ(BlockCacheBudgetFor(16 * GiB), 16 * GiB * 3 / 100);  // ~492 MiB
    EXPECT_EQ(BlockCacheBudgetFor(32 * GiB), 32 * GiB * 3 / 100);  // 0.96 GiB, just under the cap
    EXPECT_EQ(BlockCacheBudgetFor(64 * GiB), GiB);
    EXPECT_EQ(BlockCacheBudgetFor(123 * GiB), GiB);
}
