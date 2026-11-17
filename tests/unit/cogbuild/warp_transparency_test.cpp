// SPDX-License-Identifier: MPL-2.0
//
// The transparency of a warped COG: what a reader takes as the mask of the output (GDAL's
// GetMaskBand of band 1: the internal mask before the nodata of the band) for the pixels outside
// the source and the pixels the source marks as empty, and the statistics that leave them out.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
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
    s.value = [](int b, uint32_t x, uint32_t y) {
        return static_cast<float>(50 + (x + 2 * y + 30 * static_cast<uint32_t>(b)) % 150);
    };
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
                if (!jpeg) {
                    ASSERT_GE(o.bands[0][i], 50) << i;
                }
            } else {
                ASSERT_EQ(o.mask[i], 0);
                ++outside;
                if (!jpeg) {
                    ASSERT_EQ(o.bands[0][i], 0) << i;
                }
            }
        }
        // 300 x 200 source pixels of 0.25 m warped at the next zoom's finer resolution
        EXPECT_GT(inside, 300u * 200u);
        EXPECT_GT(outside, 0u);
        if (!jpeg) {
            EXPECT_GE(StatsMin(out, 1), 50.0) << "the padding is in the statistics";
        }
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

// The values of a palette band are indices into its colour table: the warp keeps them (nearest
// neighbour, as the COG driver reprojects a source with a colour table) instead of interpolating
// them into indices of other colours.
TEST(WarpTransparency, APaletteIsWarpedByNearestNeighbour) {
    TempDir dir;
    syn::SynthSpec spec = syn::Spec(64, 1, 1);
    spec.photometric = 3;  // palette
    spec.colorMap.assign(3 * 256, 0);
    for (size_t i = 0; i < 256; ++i) spec.colorMap[i] = static_cast<uint16_t>(i * 257);  // a grey ramp
    // A checkerboard of 3 x 3 pixel squares of indices 10 and 200
    spec.value = [](int, uint32_t x, uint32_t y) { return ((x / 3 + y / 3) % 2) != 0 ? 200.0f : 10.0f; };
    const std::string in = dir.File("in.tif"), out = dir.File("out.tif");
    syn::WriteSynth(in, spec);
    const Status s = BuildCog(in, out, {});
    ASSERT_TRUE(s.ok()) << s.message();
    const Output o = ReadOutput(out);
    ASSERT_EQ(o.bands.size(), 1u);
    size_t inside = 0;
    for (size_t i = 0; i < o.bands[0].size(); ++i) {
        if (!o.mask.empty() && o.mask[i] == 0) continue;
        ++inside;
        ASSERT_TRUE(o.bands[0][i] == 10 || o.bands[0][i] == 200) << "index " << int{o.bands[0][i]} << " at " << i;
    }
    EXPECT_GT(inside, 64u * 64u);
}

// An 8-bit RGBA source off the tile grid (warped to a JPEG COG, its alpha into the internal mask),
// large enough for overviews (so that its COG is encoded again on its own grid), with the alpha
// `alpha` (0 transparent, 255 opaque).
syn::SynthSpec RgbaSource(std::function<uint8_t(uint32_t x, uint32_t y)> alpha) {
    syn::SynthSpec s = syn::Spec(1200, 4, 1);
    s.height = 800;
    s.tile = 256;
    s.photometric = 2;
    s.extraSamples = {2};  // unassociated alpha
    s.value = [alpha = std::move(alpha)](int b, uint32_t x, uint32_t y) {
        return b == 3 ? static_cast<float>(alpha(x, y))
                      : static_cast<float>(60 + (x / 7 + y / 5 + 40 * static_cast<uint32_t>(b)) % 120);
    };
    return s;
}

// The bits of a sample of the internal mask of every level of `path` (0 for a level without one)
std::vector<uint16_t> MaskBitsOfLevels(const std::string& path) {
    std::vector<uint16_t> bits;
    auto r = tiff::TiffReader::Open(path);
    EXPECT_TRUE(r.ok());
    if (!r.ok()) return bits;
    const tiff::TiffReader& rd = **r;
    std::vector<uint32_t> levels = {rd.MainImage()};
    for (const uint32_t o : rd.Overviews()) levels.push_back(o);
    for (const uint32_t l : levels) {
        const auto m = rd.MaskOf(l);
        bits.push_back(m ? rd.Ifd(*m).bitsPerSample : 0);
    }
    return bits;
}

// The values of a mask between transparent and opaque, and how many distinct ones
struct Grades {
    size_t pixels = 0, values = 0;
};
Grades GradesOf(const std::vector<uint8_t>& mask) {
    Grades g;
    std::array<bool, 256> seen{};
    for (const uint8_t v : mask) {
        if (v == 0 || v == 255) continue;
        ++g.pixels;
        if (!seen[v]) ++g.values;
        seen[v] = true;
    }
    return g;
}

// A graded alpha (a feathered edge, a semi-transparent overlay: values between 0 and 255) is kept in
// an internal mask of 8 bits a pixel at every level, as the warp computes it, instead of the 1-bit
// mask that made each of its pixels transparent or opaque. The build pass sees the alpha as it
// reads it: a pass that meets a graded value stops and the build runs again with the 8-bit mask,
// also when the only graded values are in the last row of the source.
TEST(WarpTransparency, AGradedAlphaIsKeptInAnEightBitMask) {
    TempDir dir;
    const auto ramp = [](uint32_t x, uint32_t) -> uint8_t {
        return x < 200 ? 0 : x >= 1000 ? 255 : static_cast<uint8_t>((x - 200) * 255 / 800);
    };
    const auto lastRow = [](uint32_t x, uint32_t y) -> uint8_t {
        return y < 799 ? 255 : static_cast<uint8_t>(x * 255 / 1200);
    };
    for (const bool everywhere : {true, false}) {
        const std::string in = dir.File(everywhere ? "ramp.tif" : "last-row.tif");
        const std::string out = dir.File(everywhere ? "ramp-cog.tif" : "last-row-cog.tif");
        syn::WriteSynth(in, RgbaSource(everywhere ? std::function<uint8_t(uint32_t, uint32_t)>(ramp) : lastRow));
        BuildReport rep;
        const Status s = BuildCog(in, out, {}, &rep);
        ASSERT_TRUE(s.ok()) << s.message();
        EXPECT_EQ(rep.codec, "JPEG");
        EXPECT_EQ(rep.transparency, "mask");
        EXPECT_EQ(rep.maskBits, 8) << everywhere;
        EXPECT_EQ(rep.passes, 2) << everywhere;
        const std::vector<uint16_t> bits = MaskBitsOfLevels(out);
        ASSERT_GT(bits.size(), 1u) << "no overviews";
        for (const uint16_t b : bits) EXPECT_EQ(b, 8) << everywhere;
        const Output o = ReadOutput(out);
        ASSERT_FALSE(o.mask.empty());
        const Grades g = GradesOf(o.mask);
        // The ramp covers 800 source columns (some 1360 output columns) with every value; the last row
        // its two output rows
        EXPECT_GT(g.values, everywhere ? 250u : 200u) << everywhere;
        EXPECT_GT(g.pixels, everywhere ? 1000000u : 1500u) << everywhere;
        if (everywhere) {
            // Left to right along a row inside the source, the mask follows the ramp (up to the right
            // edge of the source, past which it is 0)
            const size_t row = o.height / 2;
            uint32_t end = o.width;
            while (end > 0 && o.mask[row * o.width + end - 1] != 255) --end;
            ASSERT_GT(end, o.width / 2);
            int previous = 0, drops = 0;
            for (uint32_t x = 0; x < end; ++x) {
                const int v = o.mask[row * o.width + x];
                drops += v + 1 < previous;
                previous = std::max(previous, v);
            }
            EXPECT_EQ(drops, 0);
        }
    }
}

// An alpha of transparent and opaque pixels only (an orthophoto's) keeps GDAL's 1-bit mask, built in
// one pass: the values the warp computes between them at the edge are not a graded alpha.
TEST(WarpTransparency, AnAlphaOfTransparentAndOpaquePixelsKeepsAOneBitMask) {
    TempDir dir;
    const std::string in = dir.File("in.tif"), out = dir.File("out.tif");
    syn::WriteSynth(in, RgbaSource([](uint32_t x, uint32_t y) -> uint8_t { return x + y < 300 ? 0 : 255; }));
    BuildReport rep;
    const Status s = BuildCog(in, out, {}, &rep);
    ASSERT_TRUE(s.ok()) << s.message();
    EXPECT_EQ(rep.transparency, "mask");
    EXPECT_EQ(rep.maskBits, 1);
    EXPECT_EQ(rep.passes, 1);
    for (const uint16_t b : MaskBitsOfLevels(out)) EXPECT_EQ(b, 1);
    const Output o = ReadOutput(out);
    EXPECT_EQ(GradesOf(o.mask).pixels, 0u);
}

// The COG of a graded alpha, encoded again on its own grid (a COG already on the tile grid), keeps its
// 8-bit mask as it is: the source's mask was refused ("Mask blocks differ"), and a 1-bit one would
// have flattened it.
TEST(WarpTransparency, AnEightBitMaskIsKeptWhenItsCogIsEncodedAgain) {
    TempDir dir;
    const std::string in = dir.File("in.tif"), cog = dir.File("cog.tif"), again = dir.File("again.tif");
    syn::WriteSynth(in, RgbaSource([](uint32_t x, uint32_t) -> uint8_t { return static_cast<uint8_t>(x * 255 / 1200); }));
    ASSERT_TRUE(BuildCog(in, cog, {}).ok());
    BuildReport rep;
    const Status s = BuildCog(cog, again, {}, &rep);
    ASSERT_TRUE(s.ok()) << s.message();
    EXPECT_TRUE(rep.ownGrid);
    EXPECT_EQ(rep.transparency, "mask");
    EXPECT_EQ(rep.maskBits, 8);
    for (const uint16_t b : MaskBitsOfLevels(again)) EXPECT_EQ(b, 8);
    const Output a = ReadOutput(cog), b = ReadOutput(again);
    ASSERT_EQ(a.mask.size(), b.mask.size());
    EXPECT_TRUE(a.mask == b.mask);
    EXPECT_GT(GradesOf(b.mask).values, 250u);
}

}  // namespace
}  // namespace ddbt::cog::build
