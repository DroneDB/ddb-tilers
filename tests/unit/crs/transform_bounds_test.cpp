// SPDX-License-Identifier: MPL-2.0
// TransformBounds against GDAL's TransformBounds with the area of interest gdalwarp gives -te_srs
// (data/gen_bounds_reference.py): boxes across the antimeridian, poles of a geographic or a
// projected target in the box, the meridian opposite a central meridian, a Mercator pole, a
// projection in grads, a box no point of which transforms.
#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>

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

// The doubles between a and b (of one sign)
int64_t UlpsApart(double a, double b) {
    int64_t x = 0, y = 0;
    std::memcpy(&x, &a, sizeof x);
    std::memcpy(&y, &b, sizeof y);
    return std::llabs(x - y);
}

// The reference is GDAL's on Linux: with glibc's sin, cos, exp... the bounds are its own to the bit; another C
// library rounds those functions differently in their last bits (as GDAL on it does), a few ulps at the end
#if defined(__GLIBC__)
constexpr int64_t kMaxUlps = 0;
#else
constexpr int64_t kMaxUlps = 4;
#endif

TEST(TransformBounds, MatchesGdalForEveryCase) {
    for (const BoundsCase& c : kBoundsCases) {
        SCOPED_TRACE(c.name);
        const auto b = TransformBounds(Crs::FromUserInput(c.from), Crs::FromUserInput(c.to),
                                       Bounds{c.box[0], c.box[1], c.box[2], c.box[3]});
        ASSERT_EQ(b.has_value(), c.transforms);
        if (!b) continue;
        const double got[4] = {b->minX, b->minY, b->maxX, b->maxY};
        for (int i = 0; i < 4; ++i)
            EXPECT_LE(UlpsApart(got[i], c.bounds[i]), kMaxUlps) << "bound " << i << ": " << got[i] << " for " << c.bounds[i];
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
