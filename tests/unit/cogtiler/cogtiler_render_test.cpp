// SPDX-License-Identifier: MPL-2.0
//
// COG tiler: what a plain tile draws, checked pixel by pixel on synthetic rasters whose samples
// say where they are (the level, the column and the row of each pixel): tiles draw each tile
// pixel from the level pixel under its centre, where GDAL draws it.
#include <gtest/gtest.h>

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
