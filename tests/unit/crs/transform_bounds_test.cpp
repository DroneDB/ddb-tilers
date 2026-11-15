// SPDX-License-Identifier: MPL-2.0
// TransformBounds against GDAL's TransformBounds with the area of interest gdalwarp gives -te_srs
// (data/gen_bounds_reference.py): boxes across the antimeridian, poles of a geographic or a
// projected target in the box, the meridian opposite a central meridian, a Mercator pole, a
// projection in grads, a box no point of which transforms.
#include <gtest/gtest.h>

#include "core/crs/transform_bounds.h"

namespace {

struct BoundsCase {
    const char* name;
    const char* from;
    const char* to;
    double box[4];
    bool transforms;
    double bounds[4];
};

#include "data/bounds_reference.inc"

using ddbt::crs::Bounds;
using ddbt::crs::Crs;
using ddbt::crs::TransformBounds;

TEST(TransformBounds, MatchesGdalForEveryCase) {
    for (const BoundsCase& c : kBoundsCases) {
        SCOPED_TRACE(c.name);
        const auto b = TransformBounds(Crs::FromUserInput(c.from), Crs::FromUserInput(c.to),
                                       Bounds{c.box[0], c.box[1], c.box[2], c.box[3]});
        ASSERT_EQ(b.has_value(), c.transforms);
        if (!b) continue;
        const double got[4] = {b->minX, b->minY, b->maxX, b->maxY};
        for (int i = 0; i < 4; ++i) EXPECT_EQ(got[i], c.bounds[i]) << "bound " << i;
    }
}

TEST(TransformBounds, AnEquivalentCrsKeepsTheBox) {
    const Bounds box{500000, 4999000, 501000, 5000000};
    const auto b = TransformBounds(Crs::FromEpsg(32633), Crs::FromUserInput("EPSG:32633"), box);
    ASSERT_TRUE(b);
    EXPECT_EQ(b->minX, box.minX);
    EXPECT_EQ(b->maxY, box.maxY);
}

TEST(TransformBounds, LatitudesOutOfOrderAreRefused) {
    EXPECT_FALSE(TransformBounds(Crs::FromEpsg(4326), Crs::FromEpsg(3857), Bounds{10, 50, 11, 40}));
}

}  // namespace
