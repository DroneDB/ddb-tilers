// SPDX-License-Identifier: MPL-2.0
//
// The photo readers beyond what GDAL reads (photo_samples_test.cpp): the files they refuse where
// GDAL would read more than they model, the reader each format opens with, and the decoded rows
// kept in a scratch file when they do not fit in the memory budget.

#include <gtest/gtest.h>

#include <cstddef>
#include <string>
#include <type_traits>
#include <vector>

#include "cog/build/gif_source.h"
#include "cog/build/image_source.h"
#include "cog/build/jpeg_source.h"
#include "cog/build/png_source.h"
#include "cog/build/source.h"
#include "cog/build/tiff_source.h"
#include "cog/build/webp_source.h"

namespace ddbt::cog::build {
namespace {

std::string Fixture(const std::string& name) { return std::string(DDBT_TEST_FIXTURES) + "/photo/" + name; }

// Every sample of every band, row after row.
std::vector<uint8_t> AllSamples(const RasterSource& s) {
    std::vector<uint8_t> out, block(s.BlockBytes());
    for (uint32_t by = 0; by < s.BlocksDown(); ++by) {
        auto rows = s.ReadBlock(0, by, block.data());
        EXPECT_TRUE(rows.ok()) << rows.status().message();
        if (!rows.ok()) return {};
        out.insert(out.end(), block.begin(), block.begin() + static_cast<std::ptrdiff_t>(*rows * s.BlockWidth() * s.PixelBytes()));
    }
    return out;
}

TEST(PhotoSource, EachFormatHasItsReader) {
    auto is = [](const std::string& name, auto* tag) {
        auto s = OpenRasterSource(Fixture(name));
        ASSERT_TRUE(s.ok()) << name << ": " << s.status().message();
        EXPECT_NE(dynamic_cast<std::remove_pointer_t<decltype(tag)>*>(s->get()), nullptr) << name;
        EXPECT_FALSE((*s)->Georef().crs.has_value()) << name;
        EXPECT_FALSE((*s)->Georef().geoTransform.has_value()) << name;
        EXPECT_TRUE((*s)->Georef().gcps.empty()) << name;
    };
    is("jpeg_baseline_420.jpg", static_cast<JpegSource*>(nullptr));
    is("png_rgb.png", static_cast<PngSource*>(nullptr));
    is("gif_plain.gif", static_cast<GifSource*>(nullptr));
    is("webp_lossy.webp", static_cast<WebpSource*>(nullptr));
    is("tiff_photo_jpeg.tif", static_cast<TiffSource*>(nullptr));
    is("dng_fc220_preview.dng", static_cast<TiffSource*>(nullptr));
}

TEST(PhotoSource, AGifOfSeveralImagesIsRefused) {
    // GDAL reads each image of the size of the first as a band of its own.
    auto s = OpenRasterSource(Fixture("refused/gif_animated.gif"));
    ASSERT_FALSE(s.ok());
    EXPECT_EQ(s.status().code(), StatusCode::kUnsupported);
}

TEST(PhotoSource, AnRgbTransparencyOfDifferentValuesIsRefused) {
    // GDAL gives each band its own nodata value (40, 50, 60); a source has one.
    auto s = OpenRasterSource(Fixture("refused/png_rgb_trns_bands.png"));
    ASSERT_FALSE(s.ok());
    EXPECT_EQ(s.status().code(), StatusCode::kUnsupported);
}

TEST(PhotoSource, RowsLargerThanTheBudgetGoToAScratchFile) {
    for (const char* name : {"geo_photo.jpg", "gdal_masked.jpg", "png_rgba.png", "webp_alpha.webp", "gif_plain.gif"}) {
        SCOPED_TRACE(name);
        auto inMemory = OpenRasterSource(Fixture(name));
        auto spilled = OpenRasterSource(Fixture(name), 1024);  // 1 KiB: nothing fits
        ASSERT_TRUE(inMemory.ok() && spilled.ok());
        const auto* a = dynamic_cast<const ImageSource*>(inMemory->get());
        const auto* b = dynamic_cast<const ImageSource*>(spilled->get());
        ASSERT_TRUE(a != nullptr && b != nullptr);
        ASSERT_TRUE(a->Load().ok() && b->Load().ok());
        EXPECT_FALSE(a->Spilled());
        EXPECT_TRUE(b->Spilled());
        EXPECT_EQ(AllSamples(*a), AllSamples(*b));
        if (a->HasMask()) {
            std::vector<uint8_t> ma(static_cast<size_t>(a->Width()) * a->BlockHeight()), mb(ma.size());
            for (uint32_t by = 0; by < a->BlocksDown(); ++by) {
                ASSERT_TRUE(a->ReadMaskBlock(0, by, ma.data()).ok());
                ASSERT_TRUE(b->ReadMaskBlock(0, by, mb.data()).ok());
                EXPECT_EQ(ma, mb);
            }
        }
    }
}

TEST(PhotoSource, ATruncatedJpegFailsToRead) {
    // "Premature end of JPEG file" is an error in GDAL; the source opens, the read fails.
    auto s = OpenRasterSource(Fixture("jpeg_truncated.jpg"));
    ASSERT_TRUE(s.ok());
    std::vector<uint8_t> block((*s)->BlockBytes());
    auto r = (*s)->ReadBlock(0, 0, block.data());
    ASSERT_FALSE(r.ok());
    EXPECT_NE(std::string(r.status().message()).find("Premature end"), std::string::npos) << r.status().message();
}

}  // namespace
}  // namespace ddbt::cog::build
