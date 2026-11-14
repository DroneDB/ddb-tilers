// SPDX-License-Identifier: MPL-2.0
//
// JPEG 2000 files as GDAL's JP2OpenJPEG driver reads them (tests/fixtures/jp2, expected.tsv
// written by tests/fixtures/tools/gdal_expected.py): the size, band type, every band's samples,
// colour interpretations and NBITS, the colour table, the geotransform or GCPs, the CRS and RPC
// presence, and the files GDAL refuses (tests/unit/gdal_parity.h). The samples of a lossy file
// (the 9/7 wavelet) come from floating-point arithmetic that OpenJPEG builds round differently
// (MSVC's build decodes 1 level apart in a few pixels): their CRC is exact on Linux, where the
// expectations are made, and a grid of their samples is within 1 everywhere. The same check runs
// on any folder with DDBT_JP2_PARITY_DIR (the files) and DDBT_JP2_PARITY_TSV (gdal_expected.py's
// output for it), e.g. on all of GDAL's own JPEG 2000 samples.

#include <gtest/gtest.h>

#include <cstdlib>
#include <string>

#include "cog/build/jp2_source.h"
#include "../gdal_parity.h"

namespace ddbt::cog::build {
namespace {

void IsJp2Source(const RasterSource& s) { EXPECT_NE(dynamic_cast<const Jp2Source*>(&s), nullptr); }

TEST(Jp2GdalSamples, EveryFileIsWhatGdalReads) {
    const std::string dir = std::string(DDBT_TEST_FIXTURES) + "/jp2";
    EXPECT_GE(test::CheckGdalParity(dir, dir + "/expected.tsv", IsJp2Source), 20u);
}

TEST(Jp2GdalSamples, FilesFromTheEnvironment) {
    const char* dir = std::getenv("DDBT_JP2_PARITY_DIR");
    const char* tsv = std::getenv("DDBT_JP2_PARITY_TSV");
    if (dir == nullptr || tsv == nullptr) GTEST_SKIP() << "DDBT_JP2_PARITY_DIR / DDBT_JP2_PARITY_TSV not set";
    EXPECT_GT(test::CheckGdalParity(dir, tsv, IsJp2Source), 0u);
}

}  // namespace
}  // namespace ddbt::cog::build
