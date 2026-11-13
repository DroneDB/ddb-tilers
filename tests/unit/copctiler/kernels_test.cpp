// SPDX-License-Identifier: MPL-2.0
// The COPC tiler's Highway kernels against their scalar references on every target the CPU
// supports, and the ramp against PDAL's filters.colorinterp definition.
#include <gtest/gtest.h>

#include <bit>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "copc/tiler/kernels.h"
#include "core/simd/simd.h"

namespace {

namespace k = ddbt::copc::tiler::kernels;

struct ForcedTargetGuard {
    ~ForcedTargetGuard() { ddbt::simd::ForceTargetForTest(0); }
};

// The node: per axis either 16-bit offsets or 32-bit values.
struct Node {
    std::vector<uint16_t> off[3];
    std::vector<int32_t> val[3];
    k::NodeView view;
};

Node MakeNode(size_t n, unsigned wideMask, std::mt19937& rng) {
    Node node;
    std::uniform_int_distribution<int> o(0, 65535);
    std::uniform_int_distribution<int32_t> w(-2000000000, 2000000000);
    for (int a = 0; a < 3; ++a) {
        if (wideMask & (1u << a)) {
            node.val[a].resize(n);
            for (auto& v : node.val[a]) v = w(rng);
            node.view.val32[a] = node.val[a].data();
        } else {
            node.off[a].resize(n);
            for (auto& v : node.off[a]) v = static_cast<uint16_t>(o(rng));
            node.view.off16[a] = node.off[a].data();
            node.view.base[a] = -30000 + 1000 * a;
        }
    }
    node.view.count = static_cast<uint32_t>(n);
    return node;
}

TEST(CopcTilerKernels, DecodeCullMatchesScalarOnEveryTarget) {
    ForcedTargetGuard guard;
    std::mt19937 rng(11);
    k::Scaling s;
    s.scale[0] = 0.001;
    s.scale[1] = 0.01;
    s.scale[2] = 0.0254;
    s.offset[0] = 637000.123;
    s.offset[1] = 851000.77;
    s.offset[2] = -12.5;
    for (const unsigned wide : {0u, 1u, 2u, 5u, 7u}) {
        for (const size_t n : {size_t{0}, size_t{1}, size_t{3}, size_t{17}, size_t{1000}, size_t{4099}}) {
            const Node node = MakeNode(n, wide, rng);
            // A window catching part of the points (and one that catches all, one that catches none).
            std::vector<k::Window> windows;
            std::vector<double> xs(n), ys(n), zs(n);
            std::vector<uint32_t> idx(n);
            const size_t all = k::DecodeCullScalar(node.view, 0, n, s,
                                                   {-1e300, -1e300, 1e300, 1e300}, xs.data(), ys.data(), zs.data(),
                                                   idx.data());
            ASSERT_EQ(all, n);
            double cx = 0, cy = 0;
            for (size_t i = 0; i < n; ++i) {
                cx += xs[i] / static_cast<double>(n);
                cy += ys[i] / static_cast<double>(n);
            }
            windows.push_back({-1e300, -1e300, 1e300, 1e300});
            windows.push_back({1, 1, 0, 0});
            if (n > 0) {
                windows.push_back({cx - 10, cy - 100, cx + 20, cy + 50});
                // Edges exactly on point coordinates (closed intervals).
                windows.push_back({xs[0], ys[0], xs[n - 1] > xs[0] ? xs[n - 1] : xs[0] + 1, ys[0] + 300});
            }
            for (const auto& w : windows) {
                std::vector<double> ex(n + 8), ey(n + 8), ez(n + 8);
                std::vector<uint32_t> ei(n + 8);
                const size_t expect =
                    k::DecodeCullScalar(node.view, 0, n, s, w, ex.data(), ey.data(), ez.data(), ei.data());
                for (const int64_t bit : ddbt::simd::SupportedTargetBits()) {
                    ddbt::simd::ForceTargetForTest(bit);
                    SCOPED_TRACE(ddbt::simd::TargetNameOf(bit));
                    std::vector<double> gx(n + 8), gy(n + 8), gz(n + 8);
                    std::vector<uint32_t> gi(n + 8);
                    // Two halves, like the tiler's batches.
                    const size_t half = n / 2;
                    size_t got = k::DecodeCull(node.view, 0, half, s, w, gx.data(), gy.data(), gz.data(), gi.data());
                    got += k::DecodeCull(node.view, half, n, s, w, gx.data() + got, gy.data() + got, gz.data() + got,
                                         gi.data() + got);
                    ASSERT_EQ(got, expect) << "n " << n << " wide " << wide;
                    for (size_t i = 0; i < got; ++i) {
                        ASSERT_EQ(gi[i], ei[i]);
                        ASSERT_EQ(std::bit_cast<uint64_t>(gx[i]), std::bit_cast<uint64_t>(ex[i]));
                        ASSERT_EQ(std::bit_cast<uint64_t>(gy[i]), std::bit_cast<uint64_t>(ey[i]));
                        ASSERT_EQ(std::bit_cast<uint64_t>(gz[i]), std::bit_cast<uint64_t>(ez[i]));
                    }
                }
                ddbt::simd::ForceTargetForTest(0);
            }
        }
    }
}

TEST(CopcTilerKernels, ScaledCoordinatesRoundTwice) {
    // x * scale + offset with two roundings (PDAL's builds do not contract into an FMA).
    std::vector<int32_t> xs = {123456789, -987654, 7, 2147483647};
    k::NodeView v;
    v.val32[0] = v.val32[1] = v.val32[2] = xs.data();
    v.count = static_cast<uint32_t>(xs.size());
    k::Scaling s;
    s.scale[0] = s.scale[1] = s.scale[2] = 0.1;
    s.offset[0] = s.offset[1] = s.offset[2] = 0.3;
    std::vector<double> ox(4), oy(4), oz(4);
    std::vector<uint32_t> oi(4);
    ASSERT_EQ(k::DecodeCull(v, 0, 4, s, {-1e300, -1e300, 1e300, 1e300}, ox.data(), oy.data(), oz.data(), oi.data()),
              4u);
    for (size_t i = 0; i < 4; ++i) {
        volatile double p = static_cast<double>(xs[i]) * 0.1;
        const double expect = p + 0.3;
        EXPECT_EQ(std::bit_cast<uint64_t>(ox[i]), std::bit_cast<uint64_t>(expect)) << xs[i];
    }
}

TEST(CopcTilerKernels, RampIndexIsPdalColorinterp) {
    // ColorinterpFilter::processOne: outside [min, max] no colour; position floor(f * 256),
    // the maximum clamped to 255.
    EXPECT_EQ(k::RampIndexScalar(10.0, 10.0, 20.0), 0u);
    EXPECT_EQ(k::RampIndexScalar(20.0, 10.0, 20.0), 255u);
    EXPECT_EQ(k::RampIndexScalar(15.0, 10.0, 20.0), 128u);
    EXPECT_EQ(k::RampIndexScalar(9.999999, 10.0, 20.0), k::kRampOutside);
    EXPECT_EQ(k::RampIndexScalar(20.000001, 10.0, 20.0), k::kRampOutside);
    EXPECT_EQ(k::RampIndexScalar(10.0 + 10.0 / 256 * 3, 10.0, 20.0), 3u);
    const uint32_t* t = k::RampTable();
    EXPECT_EQ(t[0], 0xFF000000u | (95u << 16) | (88u << 8) | 0u);  // pestel_shades starts at (0, 88, 95)
    EXPECT_EQ(t[k::kRampOutside], 0xFF000000u);
}

TEST(CopcTilerKernels, RampMatchesScalarOnEveryTarget) {
    ForcedTargetGuard guard;
    std::mt19937 rng(5);
    for (const auto& range : {std::pair{-3.25, 117.5}, std::pair{0.0, 1.0}, std::pair{1e5, 1e5 + 0.001}}) {
        const double lo = range.first, hi = range.second;
        std::uniform_real_distribution<double> d(lo - (hi - lo) * 0.1, hi + (hi - lo) * 0.1);
        std::vector<double> zs(4103);
        for (auto& z : zs) z = d(rng);
        // Exact bounds and the edges of ramp cells.
        zs[0] = lo;
        zs[1] = hi;
        for (int i = 0; i < 256; ++i) zs[2 + static_cast<size_t>(i)] = lo + (hi - lo) * i / 256.0;
        std::vector<uint32_t> expect(zs.size()), got(zs.size());
        k::RampRgbaScalar(zs.data(), zs.size(), lo, hi, expect.data());
        for (const int64_t bit : ddbt::simd::SupportedTargetBits()) {
            ddbt::simd::ForceTargetForTest(bit);
            SCOPED_TRACE(ddbt::simd::TargetNameOf(bit));
            for (const size_t n : {size_t{1}, size_t{7}, zs.size()}) {
                std::fill(got.begin(), got.end(), 0u);
                k::RampRgba(zs.data(), n, lo, hi, got.data());
                for (size_t i = 0; i < n; ++i) ASSERT_EQ(got[i], expect[i]) << "z " << zs[i];
            }
        }
    }
}

}  // namespace
