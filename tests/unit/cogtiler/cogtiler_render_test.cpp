// SPDX-License-Identifier: MPL-2.0
//
// COG tiler: what a plain tile draws, checked pixel by pixel on synthetic rasters whose samples
// say where they are (the level, the column and the row of each pixel): tiles draw each tile
// pixel from the level pixel under its centre, where GDAL draws it.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "../encode/decode_helpers.h"
#include "../test_env.h"
#include "cog/tiler/block_cache.h"
#include "cog/tiler/cog_tiler.h"
#include "synth_raster.h"

namespace fs = std::filesystem;
using namespace ddbt::cog::tiler;
using namespace ddbt::cog::tiler::testing;

namespace {

constexpr double kWorld = 20037508.342789244;

struct TempDir : ddbt::test::TempDir {
    TempDir() : ddbt::test::TempDir("ddbt-cogrender") {}
};

/// A tile of the process's sampling mode, decoded to RGBA.
std::vector<uint8_t> RenderRgba(const std::string& path, int z, int x, int y) {
    TileRequest r;
    r.path = path;
    r.z = z;
    r.x = x;
    r.y = y;
    r.tms = true;  // y is an XYZ row
    r.noCache = true;
    const auto png = RenderTile(r);
    const auto d = ddbt::encode::testing::DecodePng(png.data(), png.size());
    EXPECT_TRUE(d.ok);
    return d.rgba;
}

double TileSpan(int z) { return 2 * kWorld / std::pow(2.0, z); }

/// XYZ tile of a 3857 point.
std::pair<int, int> TileOf(double mx, double my, int z) {
    const double span = TileSpan(z);
    return {static_cast<int>(std::floor((mx + kWorld) / span)), static_cast<int>(std::floor((kWorld - my) / span))};
}

// A 1000 x 700 raster of 0.25 m pixels off the tile grid, with 3 overviews (500 x 350, 250 x 175,
// 125 x 88), 3 Byte bands that encode each pixel: band 1 the low 8 bits of its column, band 2 those
// of its row, band 3 its level * 64 + (column >> 8) * 4 + (row >> 8).
constexpr uint32_t kW = 1000, kH = 700;
constexpr int kOverviews = 3;

SynthSpec WhereSpec() {
    SynthSpec s = Spec(kW, 3, 1);
    s.height = kH;
    s.tile = 256;
    s.overviews = kOverviews;
    s.levelValue = [](int band, uint32_t x, uint32_t y, int level) -> float {
        if (band == 0) return static_cast<float>(x & 255);
        if (band == 1) return static_cast<float>(y & 255);
        return static_cast<float>(level * 64 + static_cast<int>(x >> 8) * 4 + static_cast<int>(y >> 8));
    };
    s.value = [f = s.levelValue](int band, uint32_t x, uint32_t y) { return f(band, x, y, 0); };
    return s;
}

/// Width and height of a level of the where-raster.
std::pair<uint32_t, uint32_t> LevelSize(int level) {
    uint32_t w = kW, h = kH;
    for (int l = 0; l < level; ++l) {
        w = (w + 1) / 2;
        h = (h + 1) / 2;
    }
    return {w, h};
}

/// Pixels of tile (z, x, y) of the where-raster whose centre is not in the level pixel they show,
/// or that are transparent inside the raster or opaque outside it; `drawn` counts the opaque ones.
int MisplacedPixels(const std::vector<uint8_t>& rgba, int z, int tx, int ty, int& drawn) {
    const double span = TileSpan(z), res = span / 256;
    const double minX = -kWorld + tx * span, maxY = kWorld - ty * span;
    const double rasterMaxX = kOriginX + kW * kPixel, rasterMinY = kOriginY - kH * kPixel;
    int bad = 0;
    drawn = 0;
    for (int j = 0; j < 256; ++j) {
        for (int i = 0; i < 256; ++i) {
            const uint8_t* p = rgba.data() + (static_cast<size_t>(j) * 256 + static_cast<size_t>(i)) * 4;
            const double cx = minX + (i + 0.5) * res, cy = maxY - (j + 0.5) * res;
            const bool inside = cx >= kOriginX && cx < rasterMaxX && cy <= kOriginY && cy > rasterMinY;
            if (p[3] == 0) {
                if (inside) ++bad;
                continue;
            }
            ++drawn;
            if (!inside) {
                ++bad;
                continue;
            }
            const int level = p[2] / 64;
            const auto [lw, lh] = LevelSize(level);
            const double resX = kPixel * kW / lw, resY = kPixel * kH / lh;
            const int col = ((p[2] % 64) / 4) * 256 + p[0], row = (p[2] % 4) * 256 + p[1];
            const double left = kOriginX + col * resX, top = kOriginY - row * resY;
            if (!(cx >= left && cx < left + resX && cy <= top && cy > top - resY)) ++bad;
        }
    }
    return bad;
}

}  // namespace

TEST(CogTilerRender, TilePixelsShowTheLevelPixelUnderTheirCentre) {
    // A raster off the tile grid, from far below its smallest overview (z13: the tile is 20 times
    // the raster) to 7 zooms above its resolution (z22, 0.037 m tile pixels): DroneDB's geoQuery
    // window, rounded to whole pixels of the level read, moved tiles by up to a pixel of that level.
    TempDir dir;
    const fs::path tif = dir.path() / "where.tif";
    WriteSynth(tif, WhereSpec());
    const double cx = kOriginX + kW * kPixel * 0.37, cy = kOriginY - kH * kPixel * 0.61;
    for (int z = 13; z <= 22; ++z) {
        const auto [tx, ty] = TileOf(cx, cy, z);
        for (int dx = -1; dx <= 1; ++dx) {
            const auto rgba = RenderRgba(tif.string(), z, tx + dx, ty);
            int drawn = 0;
            EXPECT_EQ(MisplacedPixels(rgba, z, tx + dx, ty, drawn), 0) << "z" << z << " x" << tx + dx;
            if (dx == 0) {
                EXPECT_GT(drawn, 0) << "z" << z;
            }
        }
    }
}

TEST(CogTilerRender, TilesKeepDrawingFarAboveTheResolution) {
    // 0.25 m pixels (z19), drawn up to z30 (tile pixels of 0.15 mm): the tiles around a pixel
    // corner show the enlarged pixels on each side of it. geoQuery's window, rounded to whole
    // pixels, became empty where a tile covers less than half a pixel (z29 here, z20 for a COG at
    // z11), and those tiles were blank.
    TempDir dir;
    const fs::path tif = dir.path() / "where.tif";
    WriteSynth(tif, WhereSpec());
    const double cx = kOriginX + 400 * kPixel, cy = kOriginY - 300 * kPixel;
    for (int z = 20; z <= 30; ++z) {
        const auto [tx, ty] = TileOf(cx, cy, z);
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const auto rgba = RenderRgba(tif.string(), z, tx + dx, ty + dy);
                int drawn = 0;
                EXPECT_EQ(MisplacedPixels(rgba, z, tx + dx, ty + dy, drawn), 0) << "z" << z << " " << dx << "," << dy;
                EXPECT_EQ(drawn, 256 * 256) << "z" << z << " " << dx << "," << dy;
            }
        }
    }
}

namespace {

/// The colour of entry i of the test palette (16-bit TIFF values: v * 257).
std::array<uint8_t, 3> PaletteColour(int i) {
    return {static_cast<uint8_t>(i), static_cast<uint8_t>(255 - i), static_cast<uint8_t>((i * 7) % 256)};
}

}  // namespace

TEST(CogTilerRender, PaletteBandsAreDrawnInTheirColours) {
    // A palette band (index = column / 4, 0 to 249) with NoData 7, whose colour table entry is then
    // transparent: each tile pixel takes the colour of the index under its centre. DroneDB 2.4.1
    // drew the indices as grey levels.
    TempDir dir;
    const fs::path tif = dir.path() / "palette.tif";
    SynthSpec s = Spec(kW, 1, 1);
    s.height = kH;
    s.tile = 256;
    s.photometric = 3;
    s.nodata = "7";
    s.value = [](int, uint32_t x, uint32_t) { return static_cast<float>(x / 4); };
    s.colorMap.resize(3 * 256);
    for (int i = 0; i < 256; ++i) {
        const auto c = PaletteColour(i);
        for (size_t k = 0; k < 3; ++k) s.colorMap[k * 256 + static_cast<size_t>(i)] = static_cast<uint16_t>(c[k] * 257);
    }
    WriteSynth(tif, s);
    const int z = 20;  // 0.149 m tile pixels over 0.25 m pixels
    const auto [tx, ty] = TileOf(kOriginX + 7.5, kOriginY - 20, z);  // over column 30 (index 7)
    const auto rgba = RenderRgba(tif.string(), z, tx, ty);
    const double span = TileSpan(z), res = span / 256, minX = -kWorld + tx * span, maxY = kWorld - ty * span;
    int bad = 0, transparent = 0, coloured = 0;
    for (int j = 0; j < 256; ++j)
        for (int i = 0; i < 256; ++i) {
            const double col = (minX + (i + 0.5) * res - kOriginX) / kPixel;
            const double row = (kOriginY - (maxY - (j + 0.5) * res)) / kPixel;
            if (col < 0 || col >= kW || row < 0 || row >= kH) continue;
            const int index = static_cast<int>(col) / 4;
            const uint8_t* p = rgba.data() + (static_cast<size_t>(j) * 256 + static_cast<size_t>(i)) * 4;
            if (index == 7) {
                transparent++;
                bad += p[3] != 0;
                continue;
            }
            const auto c = PaletteColour(index);
            coloured++;
            bad += p[0] != c[0] || p[1] != c[1] || p[2] != c[2] || p[3] != 255;
        }
    EXPECT_EQ(bad, 0);
    EXPECT_GT(transparent, 0);
    EXPECT_GT(coloured, 10000);
}

namespace {

/// The alpha of each tile pixel of tile (z, x, y) over raster columns < `split` and >= `split`
/// (rows inside the raster): how many are transparent on each side, and how many there are.
struct SplitAlpha {
    int leftTransparent = 0, left = 0, rightTransparent = 0, right = 0;
};

SplitAlpha AlphaAroundColumn(const std::vector<uint8_t>& rgba, int z, int tx, int ty, double split) {
    const double span = TileSpan(z), res = span / 256, minX = -kWorld + tx * span, maxY = kWorld - ty * span;
    SplitAlpha a;
    for (int j = 0; j < 256; ++j)
        for (int i = 0; i < 256; ++i) {
            const double col = (minX + (i + 0.5) * res - kOriginX) / kPixel;
            const double row = (kOriginY - (maxY - (j + 0.5) * res)) / kPixel;
            if (col < 0 || col >= kW || row < 0 || row >= kH) continue;
            const bool transparent = rgba[(static_cast<size_t>(j) * 256 + static_cast<size_t>(i)) * 4 + 3] == 0;
            if (col < split) {
                a.left++;
                a.leftTransparent += transparent;
            } else {
                a.right++;
                a.rightTransparent += transparent;
            }
        }
    return a;
}

}  // namespace

TEST(CogTilerRender, NanIsTransparent) {
    // A Float32 band without NoData, NaN over columns 0 to 99: transparent there (DroneDB 2.4.1
    // stretched NaN to 255, opaque white), drawn elsewhere; the same over 3 bands.
    TempDir dir;
    for (const int bands : {1, 3}) {
        const fs::path tif = dir.path() / ("nan" + std::to_string(bands) + ".tif");
        SynthSpec s = Spec(kW, bands, 4);
        s.height = kH;
        s.tile = 256;
        s.value = [](int band, uint32_t x, uint32_t y) {
            return x < 100 && band == 0 ? std::nanf("") : static_cast<float>(x + y + static_cast<uint32_t>(band));
        };
        WriteSynth(tif, s);
        const int z = 19;
        const auto [tx, ty] = TileOf(kOriginX + 100 * kPixel, kOriginY - 300 * kPixel, z);
        const SplitAlpha a = AlphaAroundColumn(RenderRgba(tif.string(), z, tx, ty), z, tx, ty, 100);
        EXPECT_GT(a.left, 1000) << bands;
        EXPECT_GT(a.right, 1000) << bands;
        EXPECT_EQ(a.leftTransparent, a.left) << bands;
        EXPECT_EQ(a.rightTransparent, 0) << bands;
    }
}

TEST(CogTilerRender, TwoBandsAreRedAndGreenWithTheirAlpha) {
    // Two Byte bands (200, 100) with their transparency over columns 0 to 99 in a third, alpha band
    // (a COG of a 2-band raster with a mask), or in an internal mask: drawn (200, 100, 0) and opaque
    // over the other columns, transparent over those. DroneDB 2.4.1 drew band 1 as grey and took band
    // 2 (100) for the alpha.
    TempDir dir;
    for (const bool alphaBand : {true, false}) {
        const fs::path tif = dir.path() / (alphaBand ? "alpha.tif" : "mask.tif");
        SynthSpec s = Spec(kW, alphaBand ? 3 : 2, 1);
        s.height = kH;
        s.tile = 256;
        s.value = [](int band, uint32_t x, uint32_t) {
            return band == 0 ? 200.0f : band == 1 ? 100.0f : (x < 100 ? 0.0f : 255.0f);
        };
        if (alphaBand) s.extraSamples = {0, 2};
        else s.mask = [](uint32_t x, uint32_t, int) { return x >= 100; };
        WriteSynth(tif, s);
        const int z = 19;
        const auto [tx, ty] = TileOf(kOriginX + 100 * kPixel, kOriginY - 300 * kPixel, z);
        const auto rgba = RenderRgba(tif.string(), z, tx, ty);
        const SplitAlpha a = AlphaAroundColumn(rgba, z, tx, ty, 100);
        EXPECT_GT(a.left, 1000) << alphaBand;
        EXPECT_EQ(a.leftTransparent, a.left) << alphaBand;
        EXPECT_EQ(a.rightTransparent, 0) << alphaBand;
        int wrong = 0, opaque = 0;
        for (size_t i = 0; i < rgba.size(); i += 4) {
            if (rgba[i + 3] == 0) continue;
            ++opaque;
            wrong += rgba[i] != 200 || rgba[i + 1] != 100 || rgba[i + 2] != 0 || rgba[i + 3] != 255;
        }
        EXPECT_GT(opaque, 1000) << alphaBand;
        EXPECT_EQ(wrong, 0) << alphaBand;
    }
}

namespace {

/// A graded mask over the columns of the where-raster (full-resolution columns): transparent left of
/// 256, opaque from 768, a ramp between (one block column of 256 all 0, one all 255, two graded)
uint8_t GradedMaskOfColumn(uint32_t column) {
    if (column < 256) return 0;
    if (column >= 768) return 255;
    return static_cast<uint8_t>((column - 256) * 255 / 512);
}

}  // namespace

TEST(CogTilerRender, AnEightBitMaskIsTheAlphaOfTheTile) {
    // An internal mask of 8 bits a pixel (a feathered or semi-transparent raster), which GDAL reads
    // as it is: the alpha of each tile pixel is the mask value of the level pixel it shows, the
    // blocks all 0 and all 255 included.
    TempDir dir;
    const fs::path tif = dir.path() / "graded.tif";
    SynthSpec s = WhereSpec();
    s.mask8 = [](uint32_t x, uint32_t, int level) { return GradedMaskOfColumn(x << level); };
    WriteSynth(tif, s);
    int graded = 0, drawn = 0, wrong = 0;
    for (const int z : {17, 19, 21}) {
        const double cy = kOriginY - kH * kPixel * 0.5;
        for (const uint32_t column : {100u, 300u, 512u, 760u, 900u}) {
            const auto [tx, ty] = TileOf(kOriginX + column * kPixel, cy, z);
            const auto rgba = RenderRgba(tif.string(), z, tx, ty);
            for (size_t i = 0; i < rgba.size(); i += 4) {
                const uint8_t a = rgba[i + 3];
                if (a == 0) continue;
                ++drawn;
                const int level = rgba[i + 2] / 64;
                const uint32_t col = static_cast<uint32_t>(((rgba[i + 2] % 64) / 4) * 256 + rgba[i]);
                wrong += a != GradedMaskOfColumn(col << level);
                graded += a != 255;
            }
        }
    }
    EXPECT_GT(drawn, 10000);
    EXPECT_GT(graded, 1000) << "the mask was drawn as 0 / 255";
    EXPECT_EQ(wrong, 0);
}
