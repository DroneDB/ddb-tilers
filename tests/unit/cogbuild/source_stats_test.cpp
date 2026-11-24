// SPDX-License-Identifier: MPL-2.0
//
// The statistics of the input of a COG build (BuildOptions::sourceStatistics, "sourceStatistics"
// of ddbtBuildCog), counted as the build pass reads the input: for every kind of input (a GeoTIFF
// warped, a COG encoded again on its own grid, JPEG 2000, photos georeferenced by GCPs, kept in
// memory or spilled to a scratch file) they are those the tiler's exact pass computes over the
// input file (ComputeBandStatistics on its CogSource; for a photo or a JPEG 2000 file, which the
// tiler does not open, a direct count of its decoded pixels, or the pass over its GeoTIFF copy), the
// same at any thread count, counted once when the build reads the input twice (a graded alpha), and
// they leave the output and its sidecar as they are.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "../cogtiler/synth_raster.h"
#include "../test_env.h"
#include "cog/build/cog_builder.h"
#include "cog/build/pipeline.h"
#include "cog/build/source.h"
#include "cog/build/source_stats.h"
#include "cog/build/tiff_source.h"
#include "cog/tiler/cog_source.h"
#include "cog/tiler/json_lite.h"
#include "cog/tiler/stats.h"
#include "core/exec/thread_pool.h"
#include "core/util/half.h"
#include "ddbt/ddbt.h"

namespace ddbt::cog::build {
namespace {

namespace fs = std::filesystem;
namespace syn = tiler::testing;
namespace json = tiler::json;

struct TempDir : ddbt::test::TempDir {
    TempDir() : ddbt::test::TempDir("ddbt-source-stats") {}
};

std::string Bytes(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

// A statistics file without the time it was computed at
std::string WithoutTime(const std::string& path) {
    return std::regex_replace(Bytes(path), std::regex("\"computedAt\"\\s*:\\s*\"[^\"]*\""), "");
}

// The bands of a statistics file (1-based), and whether it says it is exact, version 2
struct StatsFile {
    bool valid = false;
    std::map<int, BandStats> bands;
};

StatsFile ReadStatsFile(const std::string& path) {
    StatsFile f;
    const auto j = json::Parse(Bytes(path));
    if (!j || !j->IsObject()) return f;
    const json::Value* version = j->Find("version");
    const json::Value* exact = j->Find("exact");
    const json::Value* bands = j->Find("bands");
    const json::Value* at = j->Find("computedAt");
    f.valid = version != nullptr && version->num == 2 && exact != nullptr && exact->b && bands != nullptr &&
              bands->IsObject() && at != nullptr && at->kind == json::Value::Kind::kString;
    if (!f.valid) return f;
    for (const auto& [key, v] : bands->obj) {
        BandStats s;
        s.min = v.Find("min")->num;
        s.max = v.Find("max")->num;
        s.mean = v.Find("mean")->num;
        s.stdDev = v.Find("std")->num;
        s.valid = v.Find("valid")->u;
        s.total = v.Find("total")->u;
        f.bands[std::stoi(key)] = s;
    }
    return f;
}

// What the tiler's exact pass gives for every band of the file
std::vector<std::optional<BandStats>> TilerReference(const std::string& path) {
    const auto src = tiler::CogSource::Open(path);
    std::vector<int> bands;
    for (int b = 1; b <= src->BandCount(); ++b) bands.push_back(b);
    const auto r = tiler::ComputeBandStatistics(*src, bands, std::nullopt, stats::StatsMode::kExact);
    std::vector<std::optional<BandStats>> out(r.size());
    for (size_t i = 0; i < r.size(); ++i)
        if (r[i]) out[i] = BandStats{r[i]->min, r[i]->max, r[i]->mean, r[i]->stdDev, r[i]->validCount, r[i]->pixelCount};
    return out;
}

// The statistics of a file equal the reference: bit for bit, or (`approximate`) min, max, counts and
// mean (4 ulps) exactly and the standard deviation within 1e-12 of its value
void ExpectStats(const std::string& path, const std::vector<std::optional<BandStats>>& ref, const std::string& label,
                 bool approximate = false) {
    const StatsFile f = ReadStatsFile(path);
    ASSERT_TRUE(f.valid) << label << ": " << path;
    size_t withValue = 0;
    for (size_t i = 0; i < ref.size(); ++i) {
        const int band = static_cast<int>(i) + 1;
        const auto it = f.bands.find(band);
        if (!ref[i]) {
            EXPECT_EQ(it, f.bands.end()) << label << " band " << band << " has no pixel with a value";
            continue;
        }
        ++withValue;
        ASSERT_NE(it, f.bands.end()) << label << " band " << band;
        const BandStats& s = it->second;
        const BandStats& r = *ref[i];
        EXPECT_EQ(s.min, r.min) << label << " band " << band;
        EXPECT_EQ(s.max, r.max) << label << " band " << band;
        EXPECT_EQ(s.valid, r.valid) << label << " band " << band;
        EXPECT_EQ(s.total, r.total) << label << " band " << band;
        if (approximate) {
            EXPECT_DOUBLE_EQ(s.mean, r.mean) << label << " band " << band;
            EXPECT_NEAR(s.stdDev, r.stdDev, 1e-12 * std::max(1.0, r.stdDev)) << label << " band " << band;
        } else {
            EXPECT_EQ(s.mean, r.mean) << label << " band " << band;
            EXPECT_EQ(s.stdDev, r.stdDev) << label << " band " << band;
        }
    }
    EXPECT_EQ(f.bands.size(), withValue) << label;
}

// Builds `in` with its source statistics written to `stats`
BuildReport BuildCounted(const std::string& in, const std::string& out, const std::string& stats, BuildOptions o = {}) {
    o.sourceStatistics = stats;
    BuildReport rep;
    const Status s = BuildCog(in, out, o, &rep);
    EXPECT_TRUE(s.ok()) << in << ": " << s.message();
    return rep;
}

// Runs fn with a default pool of `threads` workers (0: the default count), then restores it.
template <typename F>
void WithDefaultPool(size_t threads, F fn) {
    exec::ShutdownDefaultPool();
    exec::SetDefaultPoolThreads(threads);
    fn();
    exec::ShutdownDefaultPool();
    exec::SetDefaultPoolThreads(0);
}

// --- the synthetic GeoTIFFs (EPSG:3857, off the tile grid: warped) ------------------------------------

syn::SynthSpec Rgb(uint32_t w, uint32_t h) {
    syn::SynthSpec s = syn::Spec(w, 3, 1);
    s.height = h;
    s.tile = 64;
    s.photometric = 2;
    s.value = [](int b, uint32_t x, uint32_t y) {
        return static_cast<float>((x * 7 + y * 13 + 50 * static_cast<uint32_t>(b)) % 256);
    };
    return s;
}

struct Case {
    std::string name;
    syn::SynthSpec spec;
};

uint64_t HalfBits(float v) { return util::FloatToHalf(v); }

std::vector<Case> SynthCases() {
    std::vector<Case> cases;
    cases.push_back({"rgb", Rgb(300, 200)});
    {
        // Five bands, band interleaved
        syn::SynthSpec s = Rgb(250, 190);
        s.bands = 5;
        s.photometric = 1;
        s.planar = true;
        cases.push_back({"planar5", s});
    }
    {
        // An alpha: the pixels of alpha 0 are out of the colour bands' statistics, not the alpha band's
        syn::SynthSpec s = Rgb(300, 200);
        s.bands = 4;
        s.extraSamples = {2};
        s.value = [](int b, uint32_t x, uint32_t y) {
            if (b == 3) return x + y < 150 ? 0.0f : x > 250 ? 128.0f : 255.0f;
            return static_cast<float>((x * 3 + y * 5 + 40 * static_cast<uint32_t>(b)) % 256);
        };
        cases.push_back({"rgba", s});
    }
    {
        // A 16-bit alpha masks a pixel where alpha / 257 is 0 (GDAL's rescaled alpha): 0 and 200
        syn::SynthSpec s = syn::Spec(200, 4, 2);
        s.height = 150;
        s.tile = 64;
        s.sampleFormat = 1;
        s.photometric = 2;
        s.extraSamples = {2};
        s.raw = [](int b, uint32_t x, uint32_t y) -> uint64_t {
            if (b == 3) return x < 30 ? 0 : x < 60 ? 200 : x < 90 ? 257 : 65535;
            return (x * 311 + y * 97 + 1000 * static_cast<uint32_t>(b)) % 65536;
        };
        cases.push_back({"rgba16", s});
    }
    {
        // NoData on a byte band
        syn::SynthSpec s = syn::Spec(230, 1, 1);
        s.height = 170;
        s.tile = 64;
        s.nodata = "7";
        s.value = [](int, uint32_t x, uint32_t y) { return static_cast<float>((x + 3 * y) % 23); };
        cases.push_back({"nodata", s});
    }
    {
        // Float32: NoData (compared as a float), NaN and infinities have no value
        syn::SynthSpec s = syn::Spec(260, 1, 4);
        s.height = 180;
        s.tile = 64;
        s.nodata = "-9999.1";
        s.value = [](int, uint32_t x, uint32_t y) {
            if ((x + y) % 31 == 0) return std::numeric_limits<float>::quiet_NaN();
            if ((x * y) % 47 == 1) return std::numeric_limits<float>::infinity();
            if ((x * y) % 53 == 2) return -std::numeric_limits<float>::infinity();
            if (x < 20) return -9999.1f;
            return 100.0f + std::sin(static_cast<float>(x) * 0.37f) * 55.5f + static_cast<float>(y) * 0.013f;
        };
        cases.push_back({"float_nodata_nan", s});
    }
    {
        // Two Float32 bands without NoData
        syn::SynthSpec s = syn::Spec(210, 2, 4);
        s.height = 140;
        s.tile = 64;
        s.value = [](int b, uint32_t x, uint32_t y) {
            return static_cast<float>(b + 1) * 1e3f + static_cast<float>(x) / 7.0f - static_cast<float>(y) / 3.0f;
        };
        cases.push_back({"float2", s});
    }
    {
        // A per-dataset mask (1 bit)
        syn::SynthSpec s = Rgb(240, 200);
        s.mask = [](uint32_t x, uint32_t y, int) { return (x / 10 + y / 10) % 3 != 0; };
        cases.push_back({"mask", s});
    }
    {
        // Complex integers: measured by their magnitudes
        syn::SynthSpec s = syn::Spec(150, 1, 4);
        s.height = 130;
        s.tile = 64;
        s.sampleFormat = 5;
        s.raw = [](int, uint32_t x, uint32_t y) -> uint64_t {
            const auto re = static_cast<uint16_t>(static_cast<int16_t>(static_cast<int>(x) - 70));
            const auto im = static_cast<uint16_t>(static_cast<int16_t>(static_cast<int>(y * 3) - 100));
            return re | (uint64_t{im} << 16);
        };
        cases.push_back({"cint16", s});
    }
    {
        // Float16: read as Float32
        syn::SynthSpec s = syn::Spec(170, 1, 2);
        s.height = 120;
        s.tile = 64;
        s.sampleFormat = 3;
        s.raw = [](int, uint32_t x, uint32_t y) -> uint64_t {
            return HalfBits(static_cast<float>(x) * 0.25f - static_cast<float>(y) * 0.5f);
        };
        cases.push_back({"float16", s});
    }
    return cases;
}

// Every kind of GeoTIFF that is warped: the statistics of its pixels as the tiler's exact pass reads
// the file, bit for bit (the same blocks, the same exact sums merged in the same block order)
TEST(SourceStatistics, OfAWarpedGeoTiffAreTheTilersExactPassOverTheFile) {
    TempDir dir;
    for (const Case& c : SynthCases()) {
        const std::string in = dir.File(c.name + ".tif"), out = dir.File(c.name + "-cog.tif");
        const std::string stats = dir.File(c.name + ".source.json");
        syn::WriteSynth(in, c.spec);
        const BuildReport rep = BuildCounted(in, out, stats);
        EXPECT_FALSE(rep.ownGrid) << c.name;
        EXPECT_EQ(rep.sourceBlocksCountedAfter, 0u) << c.name << ": the warp read every block";
        ExpectStats(stats, TilerReference(in), c.name);
    }
}

// The output and its sidecar are the same with and without the source statistics; without the option
// nothing else is written
TEST(SourceStatistics, LeaveTheOutputAndItsSidecarAsTheyAre) {
    TempDir dir;
    for (const Case& c : SynthCases()) {
        if (c.name != "rgba" && c.name != "float_nodata_nan" && c.name != "mask") continue;
        const std::string in = dir.File(c.name + ".tif");
        syn::WriteSynth(in, c.spec);
        TempDir plainDir;
        const std::string plain = plainDir.File("plain.tif"), counted = dir.File(c.name + "-counted.tif");
        ASSERT_TRUE(BuildCog(in, plain, {}).ok()) << c.name;
        EXPECT_EQ(plainDir.Entries(), 2u) << c.name << ": the COG and its sidecar only";
        BuildCounted(in, counted, dir.File(c.name + ".source.json"));
        EXPECT_TRUE(Bytes(plain) == Bytes(counted)) << c.name << ": the COGs differ";
        EXPECT_EQ(WithoutTime(plain + ".stats.json"), WithoutTime(counted + ".stats.json")) << c.name;
    }
}

// A COG already on the tile grid is encoded again on its own grid: its blocks are read once each
TEST(SourceStatistics, OfACogEncodedOnItsOwnGridAreTheTilersExactPassOverIt) {
    TempDir dir;
    struct OwnCase {
        std::string name;
        syn::SynthSpec spec;
        bool noJpeg;
    };
    syn::SynthSpec rgb = Rgb(1200, 800);  // a JPEG COG with an internal mask
    rgb.tile = 256;
    syn::SynthSpec dem = syn::Spec(1100, 1, 4);  // Float32 with NoData
    dem.height = 700;
    dem.tile = 256;
    dem.nodata = "-32768";
    dem.value = [](int, uint32_t x, uint32_t y) {
        return x < 40 ? -32768.0f : 300.0f + std::cos(static_cast<float>(x) * 0.01f) * static_cast<float>(y) * 0.1f;
    };
    for (const OwnCase& c : {OwnCase{"rgb", rgb, false}, OwnCase{"rgb_lossless", rgb, true}, OwnCase{"dem", dem, false}}) {
        const std::string in = dir.File(c.name + ".tif"), cog = dir.File(c.name + "-cog.tif");
        const std::string again = dir.File(c.name + "-again.tif"), stats = dir.File(c.name + ".source.json");
        syn::WriteSynth(in, c.spec);
        BuildOptions o;
        o.noJpeg = c.noJpeg;
        ASSERT_TRUE(BuildCog(in, cog, o).ok()) << c.name;
        const BuildReport rep = BuildCounted(cog, again, stats, o);
        EXPECT_TRUE(rep.ownGrid) << c.name;
        EXPECT_EQ(rep.sourceBlocksCountedAfter, 0u) << c.name;
        ExpectStats(stats, TilerReference(cog), c.name);
    }
}

// A graded alpha stops the first pass and the build reads the source again: every pixel still counts once
TEST(SourceStatistics, CountEveryPixelOnceWhenAGradedAlphaMakesTheBuildReadTheSourceTwice) {
    TempDir dir;
    syn::SynthSpec s = syn::Spec(1200, 4, 1);
    s.height = 800;
    s.tile = 256;
    s.photometric = 2;
    s.extraSamples = {2};
    s.value = [](int b, uint32_t x, uint32_t y) {
        if (b == 3) return x < 200 ? 0.0f : x >= 1000 ? 255.0f : static_cast<float>((x - 200) * 255 / 800);
        return static_cast<float>(60 + (x / 7 + y / 5 + 40 * static_cast<uint32_t>(b)) % 120);
    };
    const std::string in = dir.File("ramp.tif"), out = dir.File("ramp-cog.tif"), stats = dir.File("ramp.source.json");
    syn::WriteSynth(in, s);
    const BuildReport rep = BuildCounted(in, out, stats);
    EXPECT_EQ(rep.passes, 2);
    EXPECT_EQ(rep.maskBits, 8);
    // 5 x 4 blocks of 256 x 256, each counted once over the two passes, all by their reads
    EXPECT_EQ(rep.sourceBlocksCounted, 20u);
    ExpectStats(stats, TilerReference(in), "ramp");
    EXPECT_EQ(rep.sourceBlocksCountedAfter, 0u);
    // Every band of 1200 x 800 pixels, the colour bands without the 204 columns of alpha 0 (the ramp
    // starts at 0 and reaches 1 at column 204)
    const StatsFile f = ReadStatsFile(stats);
    ASSERT_EQ(f.bands.size(), 4u);
    EXPECT_EQ(f.bands.at(1).valid, 996u * 800u);
    EXPECT_EQ(f.bands.at(4).valid, 1200u * 800u);
}

// The same bits at 1, 2 and every thread: the sums are merged in block order
TEST(SourceStatistics, AreTheSameAtAnyThreadCount) {
    TempDir dir;
    for (const Case& c : SynthCases()) {
        if (c.name != "float_nodata_nan" && c.name != "float2" && c.name != "rgba") continue;
        const std::string in = dir.File(c.name + ".tif");
        syn::WriteSynth(in, c.spec);
        std::vector<std::string> results;
        for (const size_t threads : {size_t{1}, size_t{2}, size_t{0}}) {
            const std::string tag = c.name + "-" + std::to_string(threads);
            const std::string stats = dir.File(tag + ".source.json");
            WithDefaultPool(threads, [&] { BuildCounted(in, dir.File(tag + ".tif"), stats); });
            results.push_back(WithoutTime(stats));
        }
        EXPECT_EQ(results[0], results[1]) << c.name;
        EXPECT_EQ(results[0], results[2]) << c.name;
        ExpectStats(dir.File(c.name + "-1.source.json"), TilerReference(in), c.name);
    }
}

// --- JPEG 2000 ----------------------------------------------------------------------------------------

// A JPEG 2000 file's statistics are those of its GeoTIFF copy (the same pixels)
TEST(SourceStatistics, OfAJpeg2000FileAreThoseOfItsGeoTiffCopy) {
    TempDir dir;
    const std::string fixtures = std::string(DDBT_TEST_FIXTURES) + "/jp2/";
    for (const char* name : {"gen_rgba", "gen_uint16_rgba", "gen_int32", "gen_palette", "513x513", "tile_size_16"}) {
        const std::string stats = dir.File(std::string(name) + ".source.json");
        const BuildReport rep = BuildCounted(fixtures + name + ".jp2", dir.File(std::string(name) + ".tif"), stats);
        EXPECT_EQ(rep.sourceBlocksCountedAfter, 0u) << name;
        ExpectStats(stats, TilerReference(fixtures + "gtiff/" + name + ".tif"), name);
    }
}

// --- photos -------------------------------------------------------------------------------------------

// The statistics of a raster counted pixel by pixel from its decoded blocks, by the rules of the
// tiler's pass: integer samples only (the photos' 8 and 16 bits), whose sums of a small photo are
// exact in 64-bit integers; the mean is their quotient, the variance a second pass over the
// deviations from it
std::vector<std::optional<BandStats>> PixelReference(const std::string& path) {
    auto opened = OpenRasterSource(path);
    EXPECT_TRUE(opened.ok()) << path;
    if (!opened.ok()) return {};
    const RasterSource& s = **opened;
    EXPECT_TRUE(s.Type() == tiff::SampleType::kU8 || s.Type() == tiff::SampleType::kU16) << path;
    const int n = s.Bands();
    const bool wide = s.Type() == tiff::SampleType::kU16;
    const bool alphaRule = !s.HasMask() && !s.NoData() && AlphaMaskRule(n, s.AlphaBand(), s.Type());
    // Calls fn(band, value) for every sample with a value
    const auto visit = [&](const auto& fn) {
        std::vector<uint8_t> block(s.BlockBytes()), mask(static_cast<size_t>(s.BlockWidth()) * s.BlockHeight());
        for (uint32_t by = 0; by < s.BlocksDown(); ++by)
            for (uint32_t bx = 0; bx < s.BlocksAcross(); ++bx) {
                EXPECT_TRUE(s.ReadBlock(bx, by, block.data()).ok());
                if (s.HasMask()) {
                    EXPECT_TRUE(s.ReadMaskBlock(bx, by, mask.data()).ok());
                }
                for (uint32_t y = 0; y < s.BlockHeight() && by * s.BlockHeight() + y < s.Height(); ++y)
                    for (uint32_t x = 0; x < s.BlockWidth() && bx * s.BlockWidth() + x < s.Width(); ++x) {
                        const size_t px = static_cast<size_t>(y) * s.BlockWidth() + x;
                        if (s.HasMask() && mask[px] == 0) continue;
                        const auto sample = [&](int b) -> uint64_t {
                            const uint8_t* p = block.data() + px * s.PixelBytes() + static_cast<size_t>(b) * s.SampleSize();
                            if (!wide) return *p;
                            uint16_t v;
                            std::memcpy(&v, p, 2);
                            return v;
                        };
                        const bool transparent =
                            alphaRule && (wide ? sample(s.AlphaBand()) / 257 : sample(s.AlphaBand())) == 0;
                        for (int b = 0; b < n; ++b) {
                            if (transparent && b != s.AlphaBand()) continue;
                            const uint64_t v = sample(b);
                            if (s.NoData() && static_cast<double>(v) == *s.NoData()) continue;
                            fn(static_cast<size_t>(b), v);
                        }
                    }
            }
    };
    struct Acc {
        uint64_t sum = 0, count = 0;
        double lo = 1e300, hi = -1e300, mean = 0, m2 = 0;
    };
    std::vector<Acc> acc(static_cast<size_t>(n));
    visit([&](size_t b, uint64_t v) {
        Acc& a = acc[b];
        a.sum += v;
        ++a.count;
        a.lo = std::min(a.lo, static_cast<double>(v));
        a.hi = std::max(a.hi, static_cast<double>(v));
    });
    for (Acc& a : acc)
        if (a.count != 0) a.mean = static_cast<double>(a.sum) / static_cast<double>(a.count);
    visit([&](size_t b, uint64_t v) {
        const double d = static_cast<double>(v) - acc[b].mean;
        acc[b].m2 += d * d;
    });
    std::vector<std::optional<BandStats>> out(acc.size());
    for (size_t b = 0; b < acc.size(); ++b) {
        const Acc& a = acc[b];
        if (a.count == 0) continue;
        out[b] = BandStats{a.lo, a.hi, a.mean, std::sqrt(a.m2 / static_cast<double>(a.count)), a.count,
                           static_cast<uint64_t>(s.Width()) * s.Height()};
    }
    return out;
}

// GCPs at the corners of a photo of w x h pixels (geo_photo.jpg's, stretched to its size)
std::vector<crs::Gcp> CornerGcps(double w, double h) {
    return {{0, 0, 12.4960204442829, 41.9028236117254, 0},
            {0, h, 12.4962577650602, 41.9025521698054, 0},
            {w, h, 12.4967594868457, 41.9027677644874, 0},
            {w, 0, 12.4965623038112, 41.9030564539819, 0}};
}

// A photo georeferenced by GCPs: the statistics of the file's pixels (before outScale), whether the
// photo is in memory or spilled to a scratch file, into EPSG:3857 or into the GCPs' CRS, its alpha, mask
// or NoData (tRNS) applied
TEST(SourceStatistics, OfAPhotoAreThoseOfTheFilesPixels) {
    TempDir dir;
    const std::string fixtures = std::string(DDBT_TEST_FIXTURES) + "/photo/";
    struct PhotoCase {
        const char* name;
        bool sourceCrs;
        double outScale;
        uint64_t memoryBudget;
    };
    const PhotoCase cases[] = {
        {"geo_photo.jpg", false, 1.0, 0},       {"geo_photo.jpg", true, 1.0, 0},
        {"geo_photo.jpg", false, 1.0, 16 << 10}, {"geo_photo_trns.png", true, 0.5, 0},
        {"gdal_masked.jpg", false, 1.0, 0},     {"png_rgba.png", false, 1.0, 0},
        {"gdal_rgba16.png", false, 0.5, 0},     {"png_gray16.png", true, 1.0, 0},
        {"png_gray_alpha.png", false, 1.0, 0},  {"webp_alpha.webp", false, 1.0, 0},
    };
    int k = 0;
    for (const PhotoCase& c : cases) {
        const std::string path = fixtures + c.name;
        const std::string label = std::string(c.name) + " #" + std::to_string(k);
        auto opened = OpenRasterSource(path);
        ASSERT_TRUE(opened.ok()) << label;
        BuildOptions o;
        PhotoGeoreference g;
        g.gcps = CornerGcps(static_cast<int>(c.outScale * (*opened)->Width()), static_cast<int>(c.outScale * (*opened)->Height()));
        g.sourceCrs = c.sourceCrs;
        g.outScale = c.outScale;
        o.photo = g;
        if (c.memoryBudget != 0) o.memoryBudget = c.memoryBudget;
        const std::string stats = dir.File("photo" + std::to_string(k) + ".source.json");
        const BuildReport rep = BuildCounted(path, dir.File("photo" + std::to_string(k) + ".tif"), stats, o);
        EXPECT_EQ(rep.sourceBlocksCountedAfter, 0u) << label;
        // The exact sums of integers give the correctly rounded mean, as the reference's integer sum
        // does; the reference's standard deviation is summed in double (not exact), hence the tolerance
        ExpectStats(stats, PixelReference(path), label, true);
        ++k;
    }
}

// --- the counter alone and the C API ------------------------------------------------------------------

// A block read several times counts once; the blocks no read met are read by Finish()
TEST(SourceStatistics, CountABlockOnceAndFinishTheBlocksNoReadMet) {
    TempDir dir;
    const std::string in = dir.File("in.tif");
    syn::SynthSpec spec;
    for (const Case& c : SynthCases())
        if (c.name == "float_nodata_nan") spec = c.spec;
    syn::WriteSynth(in, spec);
    auto opened = TiffSource::Open(in);
    ASSERT_TRUE(opened.ok());
    const SourceStatistics counter(std::shared_ptr<const RasterSource>(std::move(*opened)));
    ASSERT_GT(counter.BlocksAcross() * counter.BlocksDown(), 4u);
    std::vector<uint8_t> block(counter.BlockBytes());
    for (int i = 0; i < 3; ++i) ASSERT_TRUE(counter.ReadBlock(0, 0, block.data()).ok());
    ASSERT_TRUE(counter.ReadBlock(1, 1, block.data()).ok());
    EXPECT_EQ(counter.BlocksCountedByReads(), 2u);
    const auto r = counter.Finish();
    ASSERT_TRUE(r.ok()) << r.status().message();
    EXPECT_EQ(counter.BlocksCountedAfter(), counter.BlocksAcross() * counter.BlocksDown() - 2);
    const std::string stats = dir.File("in.source.json");
    ASSERT_TRUE(WriteStatsFile(stats, *r).ok());
    ExpectStats(stats, TilerReference(in), "counter");
}

TEST(SourceStatistics, AreAnOptionOfDdbtBuildCog) {
    TempDir dir;
    const std::string in = dir.File("in.tif"), out = dir.File("out.tif"), stats = dir.File("source.json");
    syn::WriteSynth(in, Rgb(300, 200));
    // A Windows path has backslashes, which a JSON string escapes
    std::string escaped;
    for (const char c : stats) {
        if (c == '\\' || c == '"') escaped += '\\';
        escaped += c;
    }
    const std::string options = "{\"sourceStatistics\": \"" + escaped + "\", \"noJpeg\": true}";
    ASSERT_EQ(ddbtBuildCog(in.c_str(), out.c_str(), options.c_str()), DDBT_ERR_NONE) << ddbtLastError();
    ExpectStats(stats, TilerReference(in), "api");
    for (const char* bad : {R"({"sourceStatistics": 3})", R"({"sourceStatistics": ""})", R"({"sourceStatistics": true})"}) {
        EXPECT_EQ(ddbtBuildCog(in.c_str(), dir.File("bad.tif").c_str(), bad), DDBT_ERR_INVALIDARGS) << bad;
        EXPECT_STREQ(ddbtLastError(), "Invalid optionsJson: sourceStatistics must be the path of a file");
    }
}

}  // namespace
}  // namespace ddbt::cog::build
