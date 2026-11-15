// SPDX-License-Identifier: MPL-2.0
//
// The rules of GDAL's GTiff driver (GDAL 3.13) the metadata report follows where a TIFF GDAL writes
// does not show them: how GDAL decodes the values of GDAL_METADATA, which items it keeps, the band roles,
// the domains it keeps sorted, the quality it gives JPEG tables. DroneDB checks the whole report against
// GDAL on the files GDAL writes (TilersMetadataParity).
#include <gtest/gtest.h>

#include <set>
#include <string>
#include <vector>

#include "cog/tiff/color_interp.h"
#include "cog/tiff/gdal_metadata.h"
#include "cog/tiff/geotiff.h"
#include "cog/tiff/tiff_tags.h"
#include "core/io/byte_source.h"
#include "tiff_test_util.h"
#include "core/util/xml_lite.h"

namespace ddbt::cog::tiff {
namespace {

TEST(GdalMetadata, valuesAreDecodedAsCplUnescapeStringDecodesThem) {
    EXPECT_EQ(xml::GdalUnescape("a &lt; b &GT; c &Amp; &quot;d&apos;"), "a < b > c & \"d'");
    EXPECT_EQ(xml::GdalUnescape("&#65;&#x42;&#X43;&#xe9;"), "ABC\xC3\xA9");
    // GDAL stops at an '&' it cannot decode, and at a reference without ';'
    EXPECT_EQ(xml::GdalUnescape("R&D lab"), "R");
    EXPECT_EQ(xml::GdalUnescape("x&#65 y"), "x");
    // A reference to 0 adds nothing; the text ends at its first NUL
    EXPECT_EQ(xml::GdalUnescape("a&#0;b"), "ab");
    EXPECT_EQ(xml::GdalUnescape(std::string_view("a\0b", 3)), "a");
}

TEST(GdalMetadata, theValueOfAnElementIsItsOnlyTextToken) {
    const auto doc = xml::Parse("<r><a>  lead and trail  </a><b>  </b><c>x<!--n-->y</c><d><![CDATA[&lt;]]></d>"
                                "<e>t<f/></e><g/><h>&amp;lt;</h></r>");
    ASSERT_TRUE(doc.has_value());
    const auto value = [&](const char* name) { return doc->Child(name)->value; };
    EXPECT_EQ(value("a"), "lead and trail  ");  // GDAL's tokens start after white space
    EXPECT_FALSE(value("b").has_value());       // white space only: no token
    EXPECT_FALSE(value("c").has_value());       // three nodes
    EXPECT_EQ(value("d"), "&lt;");              // CDATA as it is
    EXPECT_FALSE(value("e").has_value());       // text and an element
    EXPECT_FALSE(value("g").has_value());
    EXPECT_EQ(value("h"), "&lt;");
}

TEST(GdalMetadata, itemsWithANameAndAValueAreReadAndDecodedTwice) {
    const auto items = ParseGdalMetadata(
        "<GDALMetadata><Item name=\"A\">a &amp;amp;lt; b</Item><Item name=\"B\" sample=\"2\" role=\"x\" "
        "domain=\"D\">v</Item><Item>no name</Item><Item name=\"E\"></Item><Other name=\"O\">o</Other>"
        "<Item name=\"R\" sample=\"0\" role=\"rat\"><GDALRasterAttributeTable/></Item></GDALMetadata>");
    ASSERT_EQ(items.size(), 2u);
    EXPECT_EQ(items[0].name, "A");
    EXPECT_EQ(items[0].value, "a &lt; b");
    EXPECT_EQ(items[0].sample, -1);
    EXPECT_EQ(items[1].sample, 2);
    EXPECT_EQ(items[1].role, "x");
    EXPECT_EQ(items[1].domain, "D");
    EXPECT_TRUE(ParseGdalMetadata("<GDALMetadata><Item name=\"A\">a</GDALMetadata>").empty());
    EXPECT_TRUE(ParseGdalMetadata("<Other><Item name=\"A\">a</Item></Other>").empty());
}

TEST(GdalMetadata, theRolesOfABandAreGdals) {
    const auto bands = GdalMetadataBands(
        "<GDALMetadata>"
        "<Item name=\"OFFSET\" sample=\"0\" role=\"offset\">149</Item>"
        "<Item name=\"COLOR\" sample=\"0\" role=\"colorinterp\">thermal</Item>"
        "<Item name=\"COLOR\" sample=\"1\" role=\"colorinterp\">rededge</Item>"
        "<Item name=\"COLOR\" sample=\"2\" role=\"ColorInterp\">Undefined</Item>"
        "<Item name=\"WAVELENGTH\" sample=\"1\">717</Item>"
        "<Item name=\"CENTRE\" sample=\"1\" domain=\"SPECTRAL\">717</Item>"
        "<Item name=\"X\" sample=\"3\">beyond the bands</Item>"
        "</GDALMetadata>",
        3);
    ASSERT_EQ(bands.size(), 3u);
    // An offset alone gives a scale of 1 that GDAL reports as set
    EXPECT_TRUE(bands[0].roles.hasOffsetScale);
    EXPECT_EQ(bands[0].roles.scale, 1.0);
    EXPECT_EQ(bands[0].roles.offset, 149.0);
    // A name GDAL does not know is Undefined and a band item
    EXPECT_EQ(bands[0].roles.colorInterp, ColorInterp::kUndefined);
    ASSERT_NE(bands[0].domains.Items(""), nullptr);
    EXPECT_EQ(*bands[0].domains.Items(""), std::vector<std::string>{"COLOR_INTERPRETATION=thermal"});
    EXPECT_EQ(bands[1].roles.colorInterp, ColorInterp::kRedEdge);
    EXPECT_EQ(*bands[1].domains.Items(""), std::vector<std::string>{"WAVELENGTH=717"});
    EXPECT_EQ(*bands[1].domains.Items("SPECTRAL"), std::vector<std::string>{"CENTRE=717"});
    EXPECT_EQ(bands[2].roles.colorInterp, ColorInterp::kUndefined);
    EXPECT_EQ(bands[2].domains.Items(""), nullptr);
}

TEST(GdalMetadata, colourInterpretationsAreNamedAsGdalNamesThem) {
    EXPECT_STREQ(ColorInterpName(ColorInterp::kNir), "NIR");
    EXPECT_STREQ(ColorInterpName(ColorInterp::kSarReserved2), "SAR_Reserved_2");
    EXPECT_EQ(ColorInterpByName("grey"), ColorInterp::kGray);
    EXPECT_EQ(ColorInterpByName("YCBCR_Y"), ColorInterp::kYCbCrY);
    EXPECT_EQ(ColorInterpByName("Other"), ColorInterp::kUndefined);
    for (int i = 0; i <= static_cast<int>(ColorInterp::kSarReserved2); ++i)
        EXPECT_EQ(static_cast<int>(ColorInterpByName(ColorInterpName(static_cast<ColorInterp>(i)))), i);
}

TEST(GdalMetadata, itemsReplaceTheirNameAndSortedDomainsStaySorted) {
    MetadataDomains d;
    d.SetItem("", "B", "1");
    d.SetItem("", "a", "2");
    d.SetItem("", "b", "3");  // replaces "B" where it is
    EXPECT_EQ(*d.Items(""), (std::vector<std::string>{"b=3", "a=2"}));
    d.SetItems("RPC", {"LINE_OFF=1", "ERR_BIAS=2"});
    d.SetItem("rpc", "HEIGHT_OFF", "3");  // domains are named without case
    EXPECT_EQ(*d.Items("RPC"), (std::vector<std::string>{"ERR_BIAS=2", "HEIGHT_OFF=3", "LINE_OFF=1"}));
    d.SetItems("xml:doc", {"<b/>"});
    EXPECT_EQ(d.Item("RPC", "line_off"), "1");
    EXPECT_FALSE(d.Item("RPC", "LINE").has_value());
    d.SetItems("EMPTY", {});
    EXPECT_EQ(d.Items("EMPTY"), nullptr);
}

TEST(GdalMetadata, everyJpegQualityIsGuessedFromItsOwnTables) {
    // GDAL takes the first quality whose tables are the file's: each quality has tables no other gives
    const auto wrapped = [](const std::vector<uint8_t>& segments) {
        std::vector<uint8_t> t = {0xFF, 0xD8};
        for (size_t at = 0; at < segments.size();) {
            const size_t len = size_t{segments[at]} * 256 + segments[at + 1];
            t.insert(t.end(), {0xFF, 0xDB});
            t.insert(t.end(), segments.begin() + static_cast<std::ptrdiff_t>(at),
                     segments.begin() + static_cast<std::ptrdiff_t>(at + len));
            at += len;
        }
        t.insert(t.end(), {0xFF, 0xD9});
        return t;
    };
    for (const bool ycbcr : {false, true})
        for (int q = 1; q <= 100; ++q)
            EXPECT_EQ(GuessJpegQuality(wrapped(StandardJpegQuantTables(q, ycbcr)), ycbcr), q) << q << " " << ycbcr;

    // libjpeg's luminance table at 75, in zigzag order, as JPEG files hold it
    std::vector<uint8_t> tables = {0xFF, 0xD8, 0xFF, 0xDB, 0x00, 0x43, 0x00};
    const uint8_t q75Zigzag[64] = {8,  6,  6,  7,  6,  5,  8,  7,  7,  7,  9,  9,  8,  10, 12, 20,
                                   13, 12, 11, 11, 12, 25, 18, 19, 15, 20, 29, 26, 31, 30, 29, 26,
                                   28, 28, 32, 36, 46, 39, 32, 34, 44, 35, 28, 28, 40, 55, 41, 44,
                                   48, 49, 52, 52, 52, 31, 39, 57, 61, 56, 50, 60, 46, 51, 52, 50};
    tables.insert(tables.end(), q75Zigzag, q75Zigzag + 64);
    tables.insert(tables.end(), {0xFF, 0xD9});
    EXPECT_EQ(GuessJpegQuality(tables, false), 75);
    EXPECT_FALSE(GuessJpegQuality(tables, true).has_value());  // YCbCr has a chrominance table too
    tables[20] ^= 1;
    EXPECT_FALSE(GuessJpegQuality(tables, false).has_value());
    EXPECT_FALSE(GuessJpegQuality(std::vector<uint8_t>{0xFF, 0xD8, 0xFF, 0xD9}, false).has_value());
}

// What GDAL writes cannot show: a resolution libtiff keeps as a float (a rational GDAL did not write), an
// unknown resolution unit, the NBITS of the bands of 12-bit samples
TEST(GdalMetadata, theTagsAreReadAsLibtiffHandsThemToGdal) {
    testing::TiffBuilder b(false, false);
    const uint64_t strip = b.AddData(std::vector<uint8_t>(12, 0));  // 4 x 2 samples of 12 bits
    std::vector<uint8_t> third(8);
    StoreEndian<uint32_t>(third.data(), 1, false);
    StoreEndian<uint32_t>(third.data() + 4, 3, false);
    b.AddIfd({b.Long(tag::kImageWidth, {4}), b.Long(tag::kImageLength, {2}), b.Short(tag::kBitsPerSample, {12}),
              b.Short(tag::kCompression, {1}), b.Short(tag::kPhotometric, {1}), b.Short(tag::kSamplesPerPixel, {1}),
              b.Long(tag::kRowsPerStrip, {2}), b.Long(tag::kStripOffsets, {static_cast<uint32_t>(strip)}),
              b.Long(tag::kStripByteCounts, {12}), b.Bytes(282, 5, third), b.Short(296, {5})});
    auto reader = TiffReader::Open(std::make_shared<io::MemorySource>(b.bytes(), "<test>"));
    ASSERT_TRUE(reader.ok());
    GdalDatasetShape shape;
    shape.bandCount = 1;
    const GdalMetadata md = ReadGdalMetadata(**reader, shape, GeoKeyDirectory{});
    EXPECT_EQ(md.dataset.Item("", "TIFFTAG_XRESOLUTION"), "0.33333334");  // the float, not 0.33333333
    EXPECT_EQ(md.dataset.Item("", "TIFFTAG_RESOLUTIONUNIT"), "5");
    EXPECT_FALSE(md.dataset.Item("", "AREA_OR_POINT").has_value());  // no GeoKeys
    ASSERT_EQ(md.bands.size(), 1u);
    EXPECT_EQ(md.bands[0].domains.Item("IMAGE_STRUCTURE", "NBITS"), "12");
    EXPECT_EQ(md.DomainList(), (std::vector<std::string>{"", "IMAGE_STRUCTURE"}));
}

TEST(GdalMetadata, compressionsAreNamedAsGdalNamesThem) {
    EXPECT_STREQ(GdalCompressionName(8), "DEFLATE");
    EXPECT_STREQ(GdalCompressionName(32946), "DEFLATE");
    EXPECT_STREQ(GdalCompressionName(52546), "JXL");
    EXPECT_EQ(GdalCompressionName(12345), nullptr);
}

}  // namespace
}  // namespace ddbt::cog::tiff
