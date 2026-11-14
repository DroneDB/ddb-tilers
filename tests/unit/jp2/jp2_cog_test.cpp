// SPDX-License-Identifier: MPL-2.0
//
// The COG built from a JPEG 2000 file is the COG built from the GeoTIFF GDAL translates it to
// (tests/fixtures/jp2/gtiff, written by gen.py): the same pixels, grid, colour table, transparency,
// metadata and statistics, byte for byte. Covers a colour table, RGBA (a JPEG output with a mask),
// 16-bit RGBA (an alpha band), GCPs, a rotated geotransform, a GMLJP2 grid, 4:2:0 YCbCr, the GDAL
// metadata and resolution boxes, 24-bit samples and tiled / tiny-tiled codestreams.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <regex>
#include <string>

#include <algorithm>
#include <cstring>
#include <set>

#include "../test_env.h"
#include "../tiff/tiff_test_util.h"
#include "cog/build/cog_builder.h"
#include "cog/build/source.h"
#include "cog/tiff/tiff_reader.h"

namespace ddbt::cog::build {
namespace {

std::string Bytes(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

// The statistics sidecar without the time it was computed at.
std::string Statistics(const std::string& cog) {
    const std::string s = Bytes(cog + ".stats.json");
    return std::regex_replace(s, std::regex("\"computedAt\"\\s*:\\s*\"[^\"]*\""), "");
}

class Jp2Cog : public testing::TestWithParam<const char*> {};

TEST_P(Jp2Cog, IsTheCogOfItsGeoTiffCopy) {
    const std::string name = GetParam();
    const std::string dir = std::string(DDBT_TEST_FIXTURES) + "/jp2/";
    test::TempDir tmp("ddbt-jp2cog");
    const std::string fromJp2 = tmp.File("jp2.tif"), fromTiff = tmp.File("tiff.tif");
    BuildReport jr, tr;
    const Status js = BuildCog(dir + name + ".jp2", fromJp2, BuildOptions{}, &jr);
    ASSERT_TRUE(js.ok()) << js.message();
    const Status ts = BuildCog(dir + "gtiff/" + name + ".tif", fromTiff, BuildOptions{}, &tr);
    ASSERT_TRUE(ts.ok()) << ts.message();
    EXPECT_EQ(jr.width, tr.width);
    EXPECT_EQ(jr.height, tr.height);
    EXPECT_EQ(jr.codec, tr.codec);
    EXPECT_EQ(jr.transparency, tr.transparency);
    EXPECT_TRUE(Bytes(fromJp2) == Bytes(fromTiff)) << "the COGs differ";
    EXPECT_EQ(Statistics(fromJp2), Statistics(fromTiff));
}

INSTANTIATE_TEST_SUITE_P(Samples, Jp2Cog,
                         testing::Values("gen_palette", "gen_rgba", "gen_uint16_rgba", "gen_gcps", "gen_rotated",
                                         "gen_gml_only", "gen_ycbcr420", "gen_metadata", "gen_int32", "small_200ppcm",
                                         "tile_size_16", "513x513"),
                         [](const testing::TestParamInfo<const char*>& i) { return std::string(i.param); });

// gdalwarp scales a 16-bit alpha band with SRC_ALPHA_MAX = DST_ALPHA_MAX = 65535 (SetAlphaMax): an
// opaque pixel stays 65535 in the output, as in GDAL's COG of gen_uint16_rgba (where 255 was
// written before).
TEST(Jp2Cog, ASixteenBitAlphaIsScaledAsGdalScalesIt) {
    test::TempDir tmp("ddbt-jp2cog");
    const std::string out = tmp.File("cog.tif");
    BuildOptions o;
    o.statistics = false;
    ASSERT_TRUE(BuildCog(std::string(DDBT_TEST_FIXTURES) + "/jp2/gen_uint16_rgba.jp2", out, o).ok());
    auto r = tiff::TiffReader::Open(out);
    ASSERT_TRUE(r.ok());
    const tiff::IfdInfo& m = (*r)->Ifd((*r)->MainImage());
    ASSERT_EQ(m.samplesPerPixel, 4);
    auto alpha = tiff::testing::ReadWindow(**r, m.index, {3}, 0, 0, m.width, m.height, 1);
    ASSERT_TRUE(alpha.ok());
    std::vector<uint16_t> a((*alpha)[0].size() / 2);
    std::memcpy(a.data(), (*alpha)[0].data(), a.size() * 2);
    const std::set<uint16_t> values(a.begin(), a.end());
    EXPECT_EQ(*values.rbegin(), 65535);
    EXPECT_EQ(values.count(255), 0u);
    EXPECT_GT(std::count(a.begin(), a.end(), uint16_t{65535}), 5000);
}

// A 16-bit alpha of 32768 is a density of 32768 / SRC_ALPHA_MAX (65535): 32768 again in the output
// (density * 65535.1), as in GDAL's COG of the file (an opaque 65535 when the density was
// value / 255).
TEST(Jp2Cog, ASixteenBitAlphaIsADensityOfItsMaximum) {
    test::TempDir tmp("ddbt-jp2cog");
    const std::string out = tmp.File("cog.tif");
    BuildOptions o;
    o.statistics = false;
    ASSERT_TRUE(BuildCog(std::string(DDBT_TEST_FIXTURES) + "/jp2/gtiff/uint16_half_alpha.tif", out, o).ok());
    auto r = tiff::TiffReader::Open(out);
    ASSERT_TRUE(r.ok());
    const tiff::IfdInfo& m = (*r)->Ifd((*r)->MainImage());
    auto alpha = tiff::testing::ReadWindow(**r, m.index, {3}, 0, 0, m.width, m.height, 1);
    ASSERT_TRUE(alpha.ok());
    std::vector<uint16_t> a((*alpha)[0].size() / 2);
    std::memcpy(a.data(), (*alpha)[0].data(), a.size() * 2);
    EXPECT_EQ(*std::max_element(a.begin(), a.end()), 32768);
    EXPECT_GT(std::count(a.begin(), a.end(), uint16_t{32768}), 1000);
}

// The TIFF tags and GDAL_METADATA items of the metadata GDAL copies, as its GTiff driver writes
// them: X / YResolution through libtiff's float and continued-fraction rational, ResolutionUnit
// from the number at the start of the text, values escaped twice, AREA_OR_POINT left out.
TEST(GtiffMetadata, TagsAndItemsAreGdals) {
    std::vector<writer::RawTag> tags;
    std::string doc;
    GtiffMetadata({{"TIFFTAG_XRESOLUTION", "200.012"},
                   {"TIFFTAG_RESOLUTIONUNIT", "3 (pixels/cm)"},
                   {"TIFFTAG_SOFTWARE", "gen"},
                   {"TIFFTAG_UNKNOWN", "x"},
                   {"AREA_OR_POINT", "Point"},
                   {"NOTE", "a & <b> \"c\""}},
                  tags, doc);
    ASSERT_EQ(tags.size(), 3u);
    EXPECT_EQ(tags[0].tag, 282);
    uint32_t nd[2];
    std::memcpy(nd, tags[0].data.data(), 8);  // little-endian hosts
    EXPECT_EQ(nd[0], 6553993u);  // GDAL's COG of small_200ppcm.jp2 holds 6553993 / 32768
    EXPECT_EQ(nd[1], 32768u);
    EXPECT_EQ(tags[1].tag, 296);
    EXPECT_EQ(tags[1].data, (std::vector<uint8_t>{3, 0}));
    EXPECT_EQ(tags[2].tag, 305);
    EXPECT_EQ(doc, "<GDALMetadata>\n  <Item name=\"NOTE\">a &amp;amp; &amp;lt;b&amp;gt; &amp;quot;c&amp;quot;</Item>\n"
                   "</GDALMetadata>\n");
    // A resolution GDAL reads from a TIFF: libtiff's float printed with 8 digits.
    EXPECT_EQ(GdalFloatTag(200.012), 200.01199);
    std::memcpy(nd, RationalTag(282, 72).data.data(), 8);
    EXPECT_EQ(nd[0], 72u);
    EXPECT_EQ(nd[1], 1u);
}

}  // namespace
}  // namespace ddbt::cog::build
