// SPDX-License-Identifier: MPL-2.0
//
// core/crs/gcp_transform: the GCP transforms against GDAL's general image transformer on rasters
// with GCPs and no geotransform (data/gen_gcp_reference.py): the method and order GDAL chooses,
// its failures, both directions, the antimeridian unwrap, and the 225 GCPs of a real SAR raster.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <span>
#include <string>
#include <vector>

#include "../test_float.h"
#include "core/crs/crs.h"
#include "core/crs/gcp_transform.h"

namespace {

struct GcpReferenceCase {
    const char* name;
    const char* crs;  // "" = none
    int method;       // GcpMethod: 0 auto, 1 homography, 2 polynomial
    int maxOrder;
    size_t gcpCount;
    const double* gcps;  // pixel, line, x, y
    bool ok;             // GDAL builds the transformer
    const char* kind;    // "homography" / "polynomial"
    int order;
    size_t pointCount;
    const double* points;  // pixel, line
    const double* fwd;     // GDAL: pixel / line -> georeferenced
    const int* fwdOk;
    const double* inv;     // GDAL: fwd -> pixel / line
    const int* invOk;
    // The same with GDAL's own linear solver (a GDAL without Armadillo); null when GDAL fails.
    const double* fwdBuiltin;
    const double* invBuiltin;
};

#include "data/gcp_reference.inc"

using ddbt::StatusCode;
using ddbt::crs::Crs;
using ddbt::crs::Direction;
using ddbt::crs::Gcp;
using ddbt::crs::GcpMethod;
using ddbt::crs::GcpTransform;
using ddbt::crs::GcpTransformOptions;
using ddbt::test::UlpDistance;

std::vector<Gcp> GcpsOf(const GcpReferenceCase& c) {
    std::vector<Gcp> gcps(c.gcpCount);
    for (size_t i = 0; i < c.gcpCount; ++i) {
        gcps[i].pixel = c.gcps[4 * i];
        gcps[i].line = c.gcps[4 * i + 1];
        gcps[i].x = c.gcps[4 * i + 2];
        gcps[i].y = c.gcps[4 * i + 3];
    }
    return gcps;
}

ddbt::Expected<GcpTransform> CreateFor(const GcpReferenceCase& c) {
    GcpTransformOptions options;
    options.method = static_cast<GcpMethod>(c.method);
    options.maxOrder = c.maxOrder;
    const Crs crs = c.crs[0] != '\0' ? Crs::FromUserInput(c.crs) : Crs();
    return GcpTransform::Create(GcpsOf(c), crs, options);
}

// Differences between our points and GDAL's (x, y pairs).
struct Diff {
    double maxAbs = 0, scale = 0;
    uint64_t maxUlp = 0;
    size_t identical = 0, count = 0;
    void Add(double ours, double gdal) {
        maxAbs = std::max(maxAbs, std::fabs(ours - gdal));
        scale = std::max(scale, std::fabs(gdal));
        maxUlp = std::max(maxUlp, UlpDistance(ours, gdal));
        identical += ours == gdal ? 1 : 0;
        ++count;
    }
};

Diff Compare(const GcpTransform& t, const GcpReferenceCase& c, const double* in, const double* expected,
             const int* expectedOk, Direction dir) {
    std::vector<double> x(c.pointCount), y(c.pointCount);
    std::vector<int> ok(c.pointCount, -1);
    for (size_t i = 0; i < c.pointCount; ++i) {
        x[i] = in[2 * i];
        y[i] = in[2 * i + 1];
    }
    const size_t failed = t.TransformPoints(x.data(), y.data(), c.pointCount, dir, ok.data());
    Diff d;
    size_t gdalFailed = 0;
    for (size_t i = 0; i < c.pointCount; ++i) {
        EXPECT_EQ(ok[i], expectedOk[i]) << "point " << i;
        if (!expectedOk[i]) {
            ++gdalFailed;
            continue;
        }
        d.Add(x[i], expected[2 * i]);
        d.Add(y[i], expected[2 * i + 1]);
    }
    EXPECT_EQ(failed, gdalFailed);
    return d;
}

// GDAL solves the homography's 9 x 9 system with Armadillo (LAPACK) when it is built with it (the
// GDAL of the reference host is), with its own LU decomposition otherwise; this port follows the
// latter, which it matches to the last bit. Against the former: within this fraction of the
// largest coordinate (measured: 2e-13 forward, 7e-12 inverse). The polynomials are the same bits.
constexpr double kArmadilloHomographyTolerance = 1e-11;

TEST(GcpTransform, MatchesGdal) {
    for (const GcpReferenceCase& c : kGcpReferenceCases) {
        SCOPED_TRACE(c.name);
        auto t = CreateFor(c);
        ASSERT_EQ(t.ok(), c.ok) << t.status().ToString();
        if (!c.ok) {
            EXPECT_NE(t.status().code(), StatusCode::kUnsupported);
            continue;
        }
        const bool homography = std::string(c.kind) == "homography";
        EXPECT_EQ(t->GetKind(), homography ? GcpTransform::Kind::Homography : GcpTransform::Kind::Polynomial);
        EXPECT_EQ(t->Order(), c.order);

        // The inverse is run on GDAL's forward results.
        const Diff fwd = Compare(*t, c, c.points, c.fwd, c.fwdOk, Direction::Forward);
        const Diff inv = Compare(*t, c, c.fwd, c.inv, c.invOk, Direction::Inverse);
        std::printf("  %-30s forward: %zu/%zu identical, max %.3g (%.3g of %.3g); inverse: %zu/%zu, max %.3g "
                    "(%.3g of %.3g)\n",
                    c.name, fwd.identical, fwd.count, fwd.maxAbs, fwd.maxAbs / fwd.scale, fwd.scale, inv.identical,
                    inv.count, inv.maxAbs, inv.maxAbs / inv.scale, inv.scale);
        if (homography) {
            EXPECT_LE(fwd.maxAbs, kArmadilloHomographyTolerance * fwd.scale);
            EXPECT_LE(inv.maxAbs, kArmadilloHomographyTolerance * inv.scale);
        } else {
            EXPECT_EQ(fwd.maxUlp, 0u);
            EXPECT_EQ(inv.maxUlp, 0u);
        }

        ASSERT_NE(c.fwdBuiltin, nullptr) << "regenerate the reference with --builtin-gdal";
        {
            const Diff fwdBuiltin = Compare(*t, c, c.points, c.fwdBuiltin, c.fwdOk, Direction::Forward);
            const Diff invBuiltin = Compare(*t, c, c.fwd, c.invBuiltin, c.invOk, Direction::Inverse);
            EXPECT_EQ(fwdBuiltin.maxUlp, 0u);
            EXPECT_EQ(invBuiltin.maxUlp, 0u);
        }
    }
}

const GcpReferenceCase& CaseNamed(const char* name) {
    for (const GcpReferenceCase& c : kGcpReferenceCases) {
        if (std::string(c.name) == name) return c;
    }
    ADD_FAILURE() << "no reference case " << name;
    return kGcpReferenceCases[0];
}

TEST(GcpTransform, LongitudesAreUnwrappedAcrossTheAntimeridian) {
    auto t = CreateFor(CaseNamed("antimeridian_unwrapped"));
    ASSERT_TRUE(t.ok()) << t.status().ToString();
    EXPECT_TRUE(t->LongitudesUnwrapped());
    // The right edge is east of 180 degrees: the longitude goes on past 180.
    double x = 1000, y = 400;
    ASSERT_TRUE(t->TransformPoint(x, y));
    EXPECT_GT(x, 181.0);
    EXPECT_LT(x, 182.1);

    for (const char* name : {"antimeridian_not_unwrapped", "antimeridian_projected_crs", "twentyfive_gcps_noisy_auto"}) {
        auto other = CreateFor(CaseNamed(name));
        ASSERT_TRUE(other.ok()) << name;
        EXPECT_FALSE(other->LongitudesUnwrapped()) << name;
    }
    // No CRS: no unwrap either.
    const GcpReferenceCase& c = CaseNamed("antimeridian_unwrapped");
    auto none = GcpTransform::Create(GcpsOf(c), Crs());
    ASSERT_TRUE(none.ok());
    EXPECT_FALSE(none->LongitudesUnwrapped());
}

TEST(GcpTransform, AHomographyWithExactGcpsMapsThemBothWays) {
    const GcpReferenceCase& c = CaseNamed("four_gcps_homography");
    auto t = CreateFor(c);
    ASSERT_TRUE(t.ok());
    for (const Gcp& g : GcpsOf(c)) {
        double x = g.pixel, y = g.line;
        ASSERT_TRUE(t->TransformPoint(x, y));
        EXPECT_NEAR(x, g.x, 1e-6);
        EXPECT_NEAR(y, g.y, 1e-6);
        ASSERT_TRUE(t->TransformPoint(x, y, Direction::Inverse));
        EXPECT_NEAR(x, g.pixel, 1e-8);
        EXPECT_NEAR(y, g.line, 1e-8);
    }
}

TEST(GcpTransform, PointsThatAnEarlierStepFailedStayFailed) {
    for (const char* name : {"four_gcps_homography", "twentyfive_gcps_noisy_auto"}) {
        auto t = CreateFor(CaseNamed(name));
        ASSERT_TRUE(t.ok()) << name;
        double xs[] = {HUGE_VAL, 10, 20};
        double ys[] = {5, HUGE_VAL, 30};
        int ok[3] = {-1, -1, -1};
        EXPECT_EQ(t->TransformPoints(xs, ys, 3, Direction::Inverse, ok), 2u) << name;
        EXPECT_EQ(ok[0], 0);
        EXPECT_EQ(ok[1], 0);
        EXPECT_EQ(ok[2], 1);
        EXPECT_EQ(xs[0], HUGE_VAL);
        EXPECT_EQ(ys[0], HUGE_VAL);
        EXPECT_EQ(xs[1], HUGE_VAL);
        EXPECT_EQ(ys[1], HUGE_VAL);
        EXPECT_TRUE(std::isfinite(xs[2]) && std::isfinite(ys[2]));
    }
}

TEST(GcpTransform, UnsupportedAndInvalidRequests) {
    const std::vector<Gcp> gcps = GcpsOf(CaseNamed("twentyfive_gcps_noisy_auto"));
    // A negative order is GDAL's thin plate spline.
    GcpTransformOptions tps;
    tps.maxOrder = -1;
    EXPECT_EQ(GcpTransform::Create(gcps, Crs(), tps).code(), StatusCode::kUnsupported);
    // With METHOD=GCP_POLYNOMIAL GDAL takes it as 0.
    tps.method = GcpMethod::Polynomial;
    auto poly = GcpTransform::Create(gcps, Crs(), tps);
    ASSERT_TRUE(poly.ok());
    EXPECT_EQ(poly->Order(), 2);
    // The homography of 4 or 5 GCPs ignores the order, as GDAL does.
    auto h = GcpTransform::Create(GcpsOf(CaseNamed("four_gcps_homography")), Crs(), {GcpMethod::Auto, -1});
    ASSERT_TRUE(h.ok());
    EXPECT_EQ(h->GetKind(), GcpTransform::Kind::Homography);
    // GDAL fits a geotransform for a homography of fewer than 4 GCPs: not ported.
    EXPECT_EQ(GcpTransform::Create(std::span(gcps).first(3), Crs(), {GcpMethod::Homography, 0}).code(),
              StatusCode::kUnsupported);
    EXPECT_EQ(GcpTransform::Create({}, Crs()).code(), StatusCode::kInvalidArgument);
    EXPECT_EQ(GcpTransform::Create(gcps, Crs(), {GcpMethod::Auto, 4}).code(), StatusCode::kInvalidArgument);
}

}  // namespace
