// SPDX-License-Identifier: MPL-2.0
//
// GDAL's own small TIFF samples (tests/fixtures/gdal): every band TiffReader decodes is what GDAL
// reads (expected.tsv: CRC-32 of band.ReadRaster()), for the rarer codecs and sample layouts:
// complex samples, 24-bit floats, odd bit widths, LZMA, CCITT, old JPEG, ThunderScan, NeXT, SGILog,
// YCbCr without JPEG, JPEG-XL.

#include <gtest/gtest.h>

#include <libdeflate.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "cog/tiff/tiff_reader.h"
#include "tiff_test_util.h"

namespace ddbt::cog::tiff {
namespace {

struct Expect {
    std::string file;
    uint32_t band = 0, width = 0, height = 0;
    std::string type;
    uint32_t crc = 0;
};

std::vector<Expect> Expectations() {
    std::ifstream in(std::string(DDBT_TEST_FIXTURES) + "/gdal/expected.tsv");
    std::vector<Expect> out;
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream s(line);
        Expect e;
        std::string crc;
        s >> e.file >> e.band >> e.width >> e.height >> e.type >> crc;
        e.crc = static_cast<uint32_t>(std::stoul(crc, nullptr, 16));
        out.push_back(e);
    }
    return out;
}

TEST(TiffGdalSamples, EveryBandIsWhatGdalReads) {
    const auto expectations = Expectations();
    ASSERT_GT(expectations.size(), 60u);
    for (const Expect& e : expectations) {
        SCOPED_TRACE(e.file + " band " + std::to_string(e.band));
        auto r = TiffReader::Open(std::string(DDBT_TEST_FIXTURES) + "/gdal/" + e.file);
        ASSERT_TRUE(r.ok()) << r.status().message();
        const IfdInfo& m = (*r)->Ifd((*r)->MainImage());
        ASSERT_EQ(m.width, e.width);
        ASSERT_EQ(m.height, e.height);
        auto bands = testing::ReadWindow(**r, m.index, {e.band - 1}, 0, 0, m.width, m.height, 1);
        ASSERT_TRUE(bands.ok()) << bands.status().message();
        const std::vector<uint8_t>& data = (*bands)[0];
        EXPECT_EQ(libdeflate_crc32(0, data.data(), data.size()), e.crc);
    }
}

}  // namespace
}  // namespace ddbt::cog::tiff
