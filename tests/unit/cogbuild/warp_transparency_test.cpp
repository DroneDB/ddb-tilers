// SPDX-License-Identifier: MPL-2.0
//
// The transparency of a warped COG: what a reader takes as the mask of the output (GDAL's
// GetMaskBand of band 1: the internal mask before the nodata of the band) for the pixels outside
// the source and the pixels the source marks as empty, and the statistics that leave them out.

#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "../cogtiler/synth_raster.h"
#include "../test_env.h"
#include "../tiff/tiff_test_util.h"
#include "cog/build/cog_builder.h"
#include "cog/tiff/tiff_reader.h"
#include "cog/tiler/json_lite.h"

namespace ddbt::cog::build {
namespace {

namespace syn = tiler::testing;

struct TempDir : ddbt::test::TempDir {
    TempDir() : ddbt::test::TempDir("ddbt-warp-transparency") {}
};

// The full-resolution output: its bands, its mask (0 / 255; empty without one), its nodata.
struct Output {
    uint32_t width = 0, height = 0;
    std::vector<std::vector<uint8_t>> bands;
    std::vector<uint8_t> mask;
    std::optional<double> nodata;
};

Output ReadOutput(const std::string& path) {
    Output o;
    auto r = tiff::TiffReader::Open(path);
    EXPECT_TRUE(r.ok()) << r.status().message();
    if (!r.ok()) return o;
    const tiff::TiffReader& rd = **r;
    const tiff::IfdInfo& m = rd.Ifd(rd.MainImage());
    o.width = m.width;
    o.height = m.height;
    o.nodata = m.noData;
    std::vector<uint32_t> bands(m.samplesPerPixel);
    for (uint32_t b = 0; b < m.samplesPerPixel; ++b) bands[b] = b;
    auto w = tiff::testing::ReadWindow(rd, m.index, bands, 0, 0, m.width, m.height, 1);
    EXPECT_TRUE(w.ok()) << w.status().message();
    if (w.ok()) o.bands = std::move(*w);
    if (const auto mask = rd.MaskOf(m.index)) {
        auto mw = tiff::testing::ReadWindow(rd, *mask, {0}, 0, 0, m.width, m.height, 255);
        EXPECT_TRUE(mw.ok());
        if (mw.ok()) o.mask = std::move((*mw)[0]);
    }
    return o;
}

// The minimum of band `band` (1-based) in the statistics sidecar of `path`
double StatsMin(const std::string& path, int band) {
    std::ifstream in(path + ".stats.json");
    std::stringstream ss;
    ss << in.rdbuf();
    const auto j = tiler::json::Parse(ss.str());
    EXPECT_TRUE(j && j->IsObject());
    if (!j || !j->Find("bands")) return -1;
    const tiler::json::Value* b = j->Find("bands")->Find(std::to_string(band));
    EXPECT_NE(b, nullptr);
    return b != nullptr && b->Find("min") != nullptr ? b->Find("min")->num : -1;
}

// An EPSG:3857 raster without nodata, mask or alpha, whose values are never 0 and not on the tile
// grid (so the COG pads it): `bands` bands of 8-bit samples.
syn::SynthSpec OpaqueSource(int bands) {
    syn::SynthSpec s = syn::Spec(300, bands, 1);
    s.height = 200;
    s.value = [](int b, uint32_t x, uint32_t y) { return static_cast<float>(50 + (x + 2 * y + 30 * b) % 150); };
    return s;
}

// A source without any transparency gets an internal mask of the pixels the warp writes (gdalwarp
// -dstalpha, as the COG driver's ADD_ALPHA): the padding of the tile grid is transparent, holds 0, and
// stays out of the statistics; the band count is the source's.
TEST(WarpTransparency, ASourceWithoutTransparencyGetsAMaskOfThePixelsOutsideIt) {
    TempDir dir;
    for (const bool jpeg : {false, true}) {
        const int bands = jpeg ? 3 : 1;
        const std::string in = dir.File("in" + std::to_string(bands) + ".tif");
        const std::string out = dir.File("out" + std::to_string(bands) + ".tif");
        syn::WriteSynth(in, OpaqueSource(bands));
        BuildReport rep;
        const Status s = BuildCog(in, out, {}, &rep);
        ASSERT_TRUE(s.ok()) << s.message();
        EXPECT_FALSE(rep.ownGrid);
        EXPECT_EQ(rep.transparency, "mask");
        EXPECT_EQ(rep.codec, jpeg ? "JPEG" : "LZW");
        EXPECT_EQ(rep.bands, bands);
        const Output o = ReadOutput(out);
        ASSERT_EQ(o.bands.size(), static_cast<size_t>(bands));
        ASSERT_FALSE(o.mask.empty()) << "no mask: the padding is opaque";
        size_t inside = 0, outside = 0;
        for (size_t i = 0; i < o.mask.size(); ++i) {
            if (o.mask[i] == 255) {
                ++inside;
                // A JPEG moves the values by a few units
                if (!jpeg) ASSERT_GE(o.bands[0][i], 50) << i;
            } else {
                ASSERT_EQ(o.mask[i], 0);
                ++outside;
                if (!jpeg) ASSERT_EQ(o.bands[0][i], 0) << i;
            }
        }
        // 300 x 200 source pixels of 0.25 m warped at the next zoom's finer resolution
        EXPECT_GT(inside, 300u * 200u);
        EXPECT_GT(outside, 0u);
        if (!jpeg) EXPECT_GE(StatsMin(out, 1), 50.0) << "the padding is in the statistics";
    }
}

// An RGB source with nodata 0 on its bands: a pixel is empty when every band is at 0 (GDAL's
// UNIFIED_SRC_NODATA, the nodata mask of all the bands), not when its red alone is. The output keeps
// the nodata and gets an internal mask of the pixels where a band is not at 0, which readers take
// before the nodata of band 1.
TEST(WarpTransparency, ThePixelsOfAnRgbWithNodataAreMaskedWhenEveryBandIsAtNodata) {
    TempDir dir;
    syn::SynthSpec spec = syn::Spec(300, 3, 1);
    spec.nodata = "0";
    // Left half: no red (a dark blue, 0 70 140); top right quarter: empty; the rest: grey
    spec.value = [](int b, uint32_t x, uint32_t y) {
        if (x < 150) return b == 0 ? 0.0f : b == 1 ? 70.0f : 140.0f;
        if (y < 150) return 0.0f;
        return 120.0f;
    };
    const std::string in = dir.File("in.tif"), out = dir.File("out.tif");
    syn::WriteSynth(in, spec);
    BuildReport rep;
    const Status s = BuildCog(in, out, {}, &rep);
    ASSERT_TRUE(s.ok()) << s.message();
    EXPECT_EQ(rep.transparency, "mask");
    const Output o = ReadOutput(out);
    ASSERT_EQ(o.bands.size(), 3u);
    EXPECT_EQ(o.nodata, 0.0);
    ASSERT_FALSE(o.mask.empty()) << "no mask: band 1's nodata hides the pixels without red";
    size_t noRed = 0, empty = 0;
    for (size_t i = 0; i < o.mask.size(); ++i) {
        const bool allZero = o.bands[0][i] == 0 && o.bands[1][i] == 0 && o.bands[2][i] == 0;
        if (allZero) {
            ++empty;
            ASSERT_EQ(o.mask[i], 0) << i;
        } else {
            ASSERT_EQ(o.mask[i], 255) << i;
            if (o.bands[0][i] == 0) ++noRed;
        }
    }
    EXPECT_GT(noRed, 150u * 300u);  // the left half, warped to a finer grid
    EXPECT_GT(empty, 150u * 150u);  // the empty quarter and the padding
}

// A raster of one pixel-interleaved RGB pixel (in a strip of one pixel) is read whole: its COG (lossless)
// holds the pixel's three values in the pixels it covers, the only ones its mask keeps; a JPEG COG of it holds
// what the JPEG codec makes of a lone pixel (grey, as GDAL's: the 2 x 2 chroma subsampling averages
// its colour with black, and the block around it rings).
TEST(WarpTransparency, AOnePixelRgbIsReadWhole) {
    TempDir dir;
    syn::SynthSpec spec = syn::Spec(1, 3, 1);
    spec.value = [](int b, uint32_t, uint32_t) { return b == 0 ? 170.0f : b == 1 ? 211.0f : 223.0f; };
    const std::string in = dir.File("in.tif"), out = dir.File("out.tif");
    syn::WriteSynth(in, spec);
    BuildOptions o;
    o.noJpeg = true;
    const Status s = BuildCog(in, out, o);
    ASSERT_TRUE(s.ok()) << s.message();
    const Output r = ReadOutput(out);
    ASSERT_EQ(r.bands.size(), 3u);
    ASSERT_FALSE(r.mask.empty());
    size_t kept = 0;
    for (size_t i = 0; i < r.mask.size(); ++i) {
        if (r.mask[i] == 0) {
            ASSERT_TRUE(r.bands[0][i] == 0 && r.bands[1][i] == 0 && r.bands[2][i] == 0) << i;
            continue;
        }
        ++kept;
        EXPECT_EQ(r.bands[0][i], 170);
        EXPECT_EQ(r.bands[1][i], 211);
        EXPECT_EQ(r.bands[2][i], 223);
    }
    EXPECT_GE(kept, 1u);  // the output pixels the source pixel covers (the grid of the next zoom is finer)
}

}  // namespace
}  // namespace ddbt::cog::build
