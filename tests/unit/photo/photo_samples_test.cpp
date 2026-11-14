// SPDX-License-Identifier: MPL-2.0
//
// Photos (JPEG, PNG, GIF, WebP, TIFF and DNG files without georeferencing) as GDAL's drivers
// read them (tests/fixtures/photo, expected.tsv written by tests/fixtures/tools/gdal_expected.py):
// the size, band type, every band's samples, colour interpretations, NBITS, nodata, the
// per-dataset mask and the colour table, and the files GDAL refuses (tests/unit/gdal_parity.h).
// The same check runs on any folder with DDBT_PHOTO_PARITY_DIR (the files) and
// DDBT_PHOTO_PARITY_TSV (gdal_expected.py's output for it), e.g. on GDAL's own samples or on
// drone photos.

#include <gtest/gtest.h>

#include <cstdlib>
#include <string>

#include "../gdal_parity.h"

namespace ddbt::cog::build {
namespace {

TEST(PhotoGdalSamples, EveryFileIsWhatGdalReads) {
    const std::string dir = std::string(DDBT_TEST_FIXTURES) + "/photo";
    EXPECT_GE(test::CheckGdalParity(dir, dir + "/expected.tsv"), 30u);
}

TEST(PhotoGdalSamples, FilesFromTheEnvironment) {
    const char* dir = std::getenv("DDBT_PHOTO_PARITY_DIR");
    const char* tsv = std::getenv("DDBT_PHOTO_PARITY_TSV");
    if (dir == nullptr || tsv == nullptr) GTEST_SKIP() << "DDBT_PHOTO_PARITY_DIR / DDBT_PHOTO_PARITY_TSV not set";
    EXPECT_GT(test::CheckGdalParity(dir, tsv), 0u);
}

}  // namespace
}  // namespace ddbt::cog::build
