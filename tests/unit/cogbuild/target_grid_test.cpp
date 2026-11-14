// SPDX-License-Identifier: MPL-2.0
//
// The warped COG grid against the grids libddb 2.4.1 built for corpus rasters (gdalinfo of the
// baseline COGs: size, origin, pixel size). The corpus is read in place, never written; a raster
// that is not on this machine skips its case.

#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <string>

#include "cog/build/source.h"
#include "cog/warp/target_grid.h"

namespace ddbt::cog::warp {
namespace {

struct GridCase {
    const char* id;
    const char* path;
    uint32_t width, height;
    int zoom;
    double originX, originY, pixel;
};

// Values: gdalinfo of /mnt/data/ddb-tilers-work/prepared/baseline-builds/<id>/cog.tif.
const GridCase kCases[] = {
    {"R2", "/mnt/data/repos/test_data/ortho/aukerman.tif", 12544, 10240, 22, -9100860.117232082411647,
     5057513.304259900003672, 0.037322767717379},
    {"R3", "/mnt/data/repos/test_data/ortho/vo.tif", 7424, 5376, 22, 483425.985389597713947, 6567020.402721168473363,
     0.037322767717154},
    {"R10", "/mnt/data/datasets/orthos/moulin-hub-2cm/2cm/25.10.02_moulin_cote_2cm.tif", 40192, 35840, 23,
     -134428.846182286739349, 5764732.576525690034032, 0.018661383858746},
    {"R7", "/mnt/data/datasets/orthos/Lamb-Hill-Farmland.Ortho.tif", 49664, 36352, 22, -273329.258519254624844,
     7173490.894392827525735, 0.037322767717361},
    {"E1", "/mnt/data/repos/test_data/ortho/edge/abetow-ERD2018-EBIRD_SCIENCE-20191109-a5cf4cb2_hr_2018_abundance_median.tiff",
     8192, 5888, 5, -20037508.342789243906736, 20037508.342789243906736, 4891.969810251280251},
    {"E2", "/mnt/data/repos/test_data/ortho/edge/spam2005v3r2_harvested-area_wheat_total.tiff", 8192, 6144, 5,
     -20037508.342789243906736, 20037508.342789243906736, 4891.969810251280251},
};

class TargetGridCorpus : public testing::TestWithParam<GridCase> {};

TEST_P(TargetGridCorpus, EqualsLibddbsCogGrid) {
    const GridCase& c = GetParam();
    if (!std::filesystem::exists(c.path)) GTEST_SKIP() << c.id << " not on this machine";
    auto src = build::OpenRasterSource(c.path);
    ASSERT_TRUE(src.ok()) << src.status().message();
    const build::RasterSource& s = **src;
    ASSERT_TRUE(s.Georef().crs.has_value());
    auto geo = SourceGeoreference::FromGeoTransform(*s.Georef().geoTransform);
    ASSERT_TRUE(geo.ok());
    auto g = ComputeTargetGrid(*s.Georef().crs, *geo, s.Width(), s.Height(), false);
    ASSERT_TRUE(g.ok()) << g.status().message();
    EXPECT_EQ(g->width, c.width) << c.id;
    EXPECT_EQ(g->height, c.height) << c.id;
    EXPECT_EQ(g->zoom, c.zoom) << c.id;
    EXPECT_NEAR(g->geoTransform[0], c.originX, 1e-6) << c.id;
    EXPECT_NEAR(g->geoTransform[3], c.originY, 1e-6) << c.id;
    EXPECT_NEAR(g->geoTransform[1], c.pixel, 1e-9 * c.pixel) << c.id;
    EXPECT_NEAR(-g->geoTransform[5], c.pixel, 1e-6 * c.pixel) << c.id;
    // The even-tile-origin option moves the origin to an even tile, never shrinking the grid.
    auto e = ComputeTargetGrid(*s.Georef().crs, *geo, s.Width(), s.Height(), true);
    ASSERT_TRUE(e.ok());
    EXPECT_LE(e->geoTransform[0], g->geoTransform[0]);
    EXPECT_GE(e->geoTransform[3], g->geoTransform[3]);
    EXPECT_GE(e->width, g->width);
    EXPECT_GE(e->height, g->height);
}

INSTANTIATE_TEST_SUITE_P(Corpus, TargetGridCorpus, testing::ValuesIn(kCases),
                         [](const testing::TestParamInfo<GridCase>& i) { return std::string(i.param.id); });

}  // namespace
}  // namespace ddbt::cog::warp
