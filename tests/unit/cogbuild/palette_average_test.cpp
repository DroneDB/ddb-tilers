// SPDX-License-Identifier: MPL-2.0
//
// GDAL's AVERAGE of a palette band (pyramid/average.h PaletteAverage): bit for bit against GDAL's
// loops written out in tests/unit/gdal_palette_average.h, on random tables and indices, odd sizes,
// cascades and the chunks GDAL reads a level in; and its rules one by one (unweighted whole pixels,
// indices past the table, the nodata entry, BestColorEntry's ties).

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>
#include <vector>

#include "cog/pyramid/average.h"
#include "../gdal_palette_average.h"

namespace {

using namespace ddbt::cog::pyramid;
using ddbt::test::GdalColorEntry;

std::vector<ColorEntry> Entries(const std::vector<GdalColorEntry>& table) {
    std::vector<ColorEntry> out;
    for (const GdalColorEntry& e : table)
        out.push_back({static_cast<uint8_t>(e.c1), static_cast<uint8_t>(e.c2), static_cast<uint8_t>(e.c3),
                       static_cast<uint8_t>(e.c4)});
    return out;
}

// A level of the implementation from the level above, as the pipeline computes it: destination
// windows of `tile` x `tile` pixels, each from a copy of the source window it reads
std::vector<uint8_t> Level(const std::vector<uint8_t>& src, int w, int h, int dw, int dh, int blockHeight,
                           const PaletteTable& table, int tile = 37, bool chunked = true) {
    const int chunkRows = chunked ? RegenerateChunkRows(w, h, dw, dh, blockHeight) : 0;
    std::vector<uint8_t> dst(static_cast<size_t>(dw) * static_cast<size_t>(dh));
    std::vector<uint8_t> win;
    for (int ty = 0; ty < dh; ty += tile) {
        const int ty1 = std::min(dh, ty + tile);
        const AverageTaps rows = AverageAxis(h, dh, ty, ty1, chunkRows);
        for (int tx = 0; tx < dw; tx += tile) {
            const int tx1 = std::min(dw, tx + tile);
            const AverageTaps cols = AverageAxis(w, dw, tx, tx1);
            const int cb = cols.SrcBegin(), ce = cols.SrcEnd(), rb = rows.SrcBegin(), re = rows.SrcEnd();
            const size_t ww = static_cast<size_t>(ce - cb);
            win.assign(ww * static_cast<size_t>(re - rb), 0);
            for (int y = rb; y < re; ++y)
                for (int x = cb; x < ce; ++x)
                    win[static_cast<size_t>(y - rb) * ww + static_cast<size_t>(x - cb)] =
                        src[static_cast<size_t>(y) * static_cast<size_t>(w) + static_cast<size_t>(x)];
            PaletteAverage(table, win.data(), ww, cb, rb, cols, rows, tx1 - tx, ty1 - ty,
                           dst.data() + static_cast<size_t>(ty) * static_cast<size_t>(dw) + static_cast<size_t>(tx),
                           static_cast<size_t>(dw));
        }
    }
    return dst;
}

// Every level of `factors` (ceil(size / factor)), each from the one above, the overviews in GTiff's
// overview blocks (256 stays 256)
std::vector<std::vector<uint8_t>> Levels(const std::vector<uint8_t>& src, int width, int height, int blockSize,
                                         const std::vector<int>& factors, const PaletteTable& table) {
    std::vector<std::vector<uint8_t>> levels;
    levels.reserve(factors.size());
    const std::vector<uint8_t>* above = &src;
    int w = width, h = height;
    for (const int f : factors) {
        const int dw = (width + f - 1) / f, dh = (height + f - 1) / f;
        levels.push_back(Level(*above, w, h, dw, dh, blockSize, table));
        above = &levels.back();
        w = dw;
        h = dh;
    }
    return levels;
}

std::vector<std::array<uint8_t, 3>> RandomRgb(std::mt19937& rng, size_t n) {
    std::uniform_int_distribution<int> c(0, 255);
    std::vector<std::array<uint8_t, 3>> rgb(n);
    for (auto& e : rgb) e = {static_cast<uint8_t>(c(rng)), static_cast<uint8_t>(c(rng)), static_cast<uint8_t>(c(rng))};
    return rgb;
}

// Indices in runs (palette images are made of areas of one class) with random ones among them
std::vector<uint8_t> RandomIndices(std::mt19937& rng, int w, int h, int maxIndex) {
    std::uniform_int_distribution<int> idx(0, maxIndex), run(1, 6), coin(0, 3);
    std::vector<uint8_t> px(static_cast<size_t>(w) * static_cast<size_t>(h));
    size_t i = 0;
    while (i < px.size()) {
        const uint8_t v = static_cast<uint8_t>(idx(rng));
        for (int k = run(rng); k > 0 && i < px.size(); --k, ++i) px[i] = coin(rng) == 0 ? static_cast<uint8_t>(idx(rng)) : v;
    }
    return px;
}

// Random tables (1 to 256 colours, padded to 256 as GTiff reads them back) and indices of a
// 301 x 203 raster, without nodata and with every kind of nodata value, overviews 2, 4, 8 each
// from the one above: every sample of every level is GDAL's
TEST(PaletteAverage, isGdalsAverageOfAPaletteBandOnRandomTablesAndIndices) {
    std::mt19937 rng(20261116);
    const int w = 301, h = 203;
    const std::vector<int> factors{2, 4, 8};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    // none, an entry of the table, a padding entry, past Byte (cast to 44), in (-1, 0) (entry 0
    // transparent when read back, not in the average), NaN
    const std::vector<std::optional<double>> nodatas{std::nullopt, 3.0, 200.0, 300.0, -0.5, nan};
    for (const size_t colours : {size_t{1}, size_t{7}, size_t{40}, size_t{255}, size_t{256}}) {
        for (const std::optional<double>& nodata : nodatas) {
            const auto rgb = RandomRgb(rng, colours);
            const std::vector<uint8_t> src = RandomIndices(rng, w, h, 255);
            const auto table = ddbt::test::GtiffReadBackTable(rgb, nodata.has_value(), nodata.value_or(0));
            const auto expected =
                ddbt::test::GdalPaletteOverviews(src, w, h, 256, factors, table, nodata.has_value(), nodata.value_or(0));
            const auto got = Levels(src, w, h, 256, factors, AveragePalette(Entries(table), nodata));
            for (size_t l = 0; l < factors.size(); ++l)
                ASSERT_EQ(got[l], expected[l]) << colours << " colours, nodata " << nodata.value_or(-1) << ", level "
                                               << l + 1;
        }
    }
}

// A level 10240 to 20479 samples wide is read in chunks of 512 rows (twice the 256 of the block); a
// destination row whose window starts above its chunk loses those rows (701 -> 351: row 256 covers
// [511.3, 513.3), its chunk starts at 512), as GDAL computes it
TEST(PaletteAverage, clipsTheWindowOfARowToTheChunkGdalReadsItIn) {
    std::mt19937 rng(7);
    const int w = 10241, h = 701, dw = 5121, dh = 351;
    ASSERT_EQ(RegenerateChunkRows(w, h, dw, dh, 256), 512);
    const auto rgb = RandomRgb(rng, 24);
    const std::vector<uint8_t> src = RandomIndices(rng, w, h, 23);
    const auto table = ddbt::test::GtiffReadBackTable(rgb, false, 0);
    const PaletteTable palette = AveragePalette(Entries(table), std::nullopt);
    const auto expected = ddbt::test::GdalPaletteOverview(src, w, h, dw, dh, 256, table, false, 0);
    EXPECT_EQ(Level(src, w, h, dw, dh, 256, palette, 256), expected);
    EXPECT_NE(Level(src, w, h, dw, dh, 256, palette, 256, false), expected) << "the chunks change some rows";
}

TEST(PaletteAverage, chunksAreTheBlockHeightGrownAsGdalGrowsThem) {
    EXPECT_EQ(RegenerateChunkRows(301, 203, 151, 102, 256), 32768);  // doubled while under 5 MiB
    EXPECT_EQ(RegenerateChunkRows(50000, 1000, 25000, 500, 256), 256);  // over 10 MiB: kept
    EXPECT_EQ(RegenerateChunkRows(1000, 1000, 1, 1, 64), 8000);  // twice the factor, then doubled
}

// GDAL averages the whole pixels of the window alike: 3 -> 2 columns, destination pixel 0 covers
// [0, 1.5) and counts pixels 0 and 1 whole (a weighted average would give pixel 1 half its weight)
TEST(PaletteAverage, countsEveryPixelOfTheWindowAlike) {
    const PaletteTable t = AveragePalette({{0, 0, 0, 255}, {90, 90, 90, 255}, {45, 45, 45, 255}, {30, 30, 30, 255}},
                                          std::nullopt);
    const uint8_t src[3] = {0, 1, 1};
    const AverageTaps cols = AverageAxis(3, 2, 0, 2), rows = AverageAxis(1, 1, 0, 1);
    uint8_t out[2] = {9, 9};
    PaletteAverage(t, src, 3, 0, 0, cols, rows, 2, 1, out, 2);
    EXPECT_EQ(out[0], 2);  // (0 + 90) / 2 = 45, not (0 + 90 / 2) / 1.5 = 30
    EXPECT_EQ(out[1], 1);
}

// Indices past the table and transparent entries are left out; a pixel without any colour is the
// nodata value of the table
TEST(PaletteAverage, leavesOutIndicesPastTheTableAndTransparentEntries) {
    const PaletteTable t = AveragePalette({{0, 0, 0, 255}, {200, 0, 0, 0}, {100, 100, 100, 255}, {50, 50, 50, 255}}, 7.0);
    const AverageTaps axis = AverageAxis(2, 1, 0, 1);
    const auto average = [&](std::vector<uint8_t> px) {
        uint8_t out = 0;
        PaletteAverage(t, px.data(), 2, 0, 0, axis, axis, 1, 1, &out, 1);
        return out;
    };
    EXPECT_EQ(average({0, 9, 2, 200}), 3);  // (0 + 100) / 2: 50
    EXPECT_EQ(average({1, 1, 2, 2}), 2);    // entry 1 transparent
    EXPECT_EQ(average({1, 9, 7, 200}), 7);  // no colour: the nodata value
}

// Rounded half up per channel, then the nearest opaque entry, the first of equals
TEST(PaletteAverage, roundsEachChannelHalfUp) {
    const PaletteTable t = AveragePalette({{0, 0, 0, 255}, {1, 1, 1, 255}, {2, 2, 2, 255}, {3, 3, 3, 255}}, std::nullopt);
    const AverageTaps axis = AverageAxis(2, 1, 0, 1);
    uint8_t out = 0;
    const uint8_t px[4] = {0, 0, 1, 2};  // 3 / 4 = 0.75: 1
    PaletteAverage(t, px, 2, 0, 0, axis, axis, 1, 1, &out, 1);
    EXPECT_EQ(out, 1);
    const uint8_t half[4] = {0, 0, 1, 1};  // 2 / 4 = 0.5: 1
    PaletteAverage(t, half, 2, 0, 0, axis, axis, 1, 1, &out, 1);
    EXPECT_EQ(out, 1);
}

TEST(BestColorEntry, isTheFirstOfTheNearestOpaqueEntries) {
    const std::vector<ColorEntry> e{{10, 0, 0, 255}, {0, 10, 0, 255}, {5, 5, 0, 0}, {0, 0, 200, 255}};
    EXPECT_EQ(BestColorEntry(e, 5, 5, 0), 0);  // 0 and 1 tie; 2 is transparent
    EXPECT_EQ(BestColorEntry(e, 4, 6, 0), 1);
    EXPECT_EQ(BestColorEntry(e, 0, 0, 255), 3);
    EXPECT_EQ(BestColorEntry({{1, 1, 1, 0}, {2, 2, 2, 0}}, 1, 1, 1), 0) << "no opaque entry: 0";
}

// GDAL's preparation of the table: the entry of a nodata >= 0 (cast to 8 bits) transparent, or
// without nodata the first transparent entry taken as the nodata value
TEST(AveragePalette, makesTheNodataEntryTransparentAsGdal) {
    std::vector<ColorEntry> table(256, ColorEntry{1, 2, 3, 255});
    const auto transparent = [](const PaletteTable& t) {
        std::vector<size_t> out;
        for (size_t i = 0; i < t.entries.size(); ++i)
            if (t.entries[i][3] == 0) out.push_back(i);
        return out;
    };
    PaletteTable t = AveragePalette(table, 3.0);
    EXPECT_EQ(t.noData, 3);
    EXPECT_EQ(transparent(t), std::vector<size_t>{3});
    t = AveragePalette(table, 300.0);  // static_cast<GByte>(300.0): 44
    EXPECT_EQ(t.noData, 44);
    EXPECT_EQ(transparent(t), std::vector<size_t>{44});
    t = AveragePalette(std::vector<ColorEntry>(10, ColorEntry{1, 2, 3, 255}), 30.0);  // past the table
    EXPECT_EQ(t.noData, 30);
    EXPECT_TRUE(transparent(t).empty());
    t = AveragePalette(table, -0.5);  // negative: the table as it is
    EXPECT_EQ(t.noData, 0);
    EXPECT_TRUE(transparent(t).empty());
    t = AveragePalette(table, std::numeric_limits<double>::quiet_NaN());
    EXPECT_EQ(t.noData, 0);
    EXPECT_TRUE(transparent(t).empty());
    table[9][3] = 0;
    table[12][3] = 0;
    t = AveragePalette(table, std::nullopt);
    EXPECT_EQ(t.noData, 9);
    t = AveragePalette(table, 5.0);  // a nodata value: the transparent entries stay, 5 joins them
    EXPECT_EQ(t.noData, 5);
    EXPECT_EQ(transparent(t), (std::vector<size_t>{5, 9, 12}));
}

TEST(AveragePalette, theIdentityIsEveryEntryItsOwnGreyOpaque) {
    std::vector<ColorEntry> table;
    for (int i = 0; i < 256; ++i) table.push_back({static_cast<uint8_t>(i), static_cast<uint8_t>(i), static_cast<uint8_t>(i), 255});
    EXPECT_TRUE(IsIdentityTable(table));
    table[7][3] = 0;  // the entry of a nodata value
    EXPECT_FALSE(IsIdentityTable(table));
    table[7] = {7, 7, 8, 255};
    EXPECT_FALSE(IsIdentityTable(table));
}

}  // namespace
