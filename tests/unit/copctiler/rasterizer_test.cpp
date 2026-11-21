// SPDX-License-Identifier: MPL-2.0
// The compat rasterizer against DroneDB 2.4.1's PointRasterizer golden test
// (tests/integration/pointcloud_render_test.cpp, PointRasterizer.goldenImage: same random points,
// same reference loop), and the splat rasterizer of the hybrid and modern modes (both discs)
// against a brute-force definition; its stamp against PointRasterizer's.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

#include "copc/tiler/rasterizer.h"

namespace {

using ddbt::copc::tiler::CompatMapping;
using ddbt::copc::tiler::CompatRasterizer;
using ddbt::copc::tiler::SplatKey;
using ddbt::copc::tiler::SplatRasterizer;

struct Batch {
    std::vector<double> x, y, z;
    std::vector<uint16_t> red, green, blue;
    size_t size() const { return x.size(); }
};

// The point drawing the tiler used to do, kept as the reference (DroneDB's test).
void ReferenceDrawCircle(uint8_t* buffer, uint8_t* alpha, int px, int py, int radius, uint8_t r, uint8_t g, uint8_t b,
                         int tileSize, int wSize) {
    const int r2 = radius * radius;
    const int area = r2 << 2;
    const int rr = radius << 1;
    for (int i = 0; i < area; i++) {
        const int tx = (i % rr) - radius;
        const int ty = (i / rr) - radius;
        if (tx * tx + ty * ty <= r2) {
            const int dx = px + tx;
            const int dy = py + ty;
            if (dx >= 0 && dx < tileSize && dy >= 0 && dy < tileSize) {
                buffer[dy * tileSize + dx + wSize * 0] = r;
                buffer[dy * tileSize + dx + wSize * 1] = g;
                buffer[dy * tileSize + dx + wSize * 2] = b;
                alpha[dy * tileSize + dx] = 255;
            }
        }
    }
}

// DroneDB's randomPoints (libstdc++'s distributions, as the baseline's build).
Batch RandomPoints(size_t n, double minX, double minY, double size, bool sixteenBit, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> pos(-0.05, 1.05);
    std::uniform_real_distribution<double> z(0, 100);
    std::uniform_int_distribution<int> color(0, sixteenBit ? 65535 : 255);
    Batch b;
    for (size_t i = 0; i < n; i++) {
        b.x.push_back(minX + pos(rng) * size);
        b.y.push_back(minY + pos(rng) * size);
        b.z.push_back(z(rng));
        b.red.push_back(static_cast<uint16_t>(color(rng)));
        b.green.push_back(static_cast<uint16_t>(color(rng)));
        b.blue.push_back(static_cast<uint16_t>(color(rng)));
    }
    return b;
}

TEST(CopcTilerRasterizer, CompatMatchesThePointRasterizerGolden) {
    const int tileSize = 256;
    const double minX = 1000, minY = 2000, tileMeters = 150;
    const double resolution = tileMeters / tileSize;

    for (const bool sixteenBit : {false, true}) {
        const auto points = RandomPoints(20000, minX, minY, tileMeters, sixteenBit, 42);

        // Reference: the loop the tiler used to run
        const int wSize = tileSize * tileSize;
        const int pointRadius = 2;
        const double pointRadiusMeters = pointRadius * resolution;
        const int paddedTileSize = tileSize + pointRadius * 2;
        std::vector<uint8_t> buffer(static_cast<size_t>(wSize) * 3, 0), alpha(static_cast<size_t>(wSize), 0);
        std::vector<float> zBuffer(static_cast<size_t>(paddedTileSize) * paddedTileSize, -99999.0f);
        const double maxX = minX + tileMeters, maxY = minY + tileMeters;
        const double paddedTileScaleW = paddedTileSize / (maxX - minX + pointRadiusMeters * 2.0);
        const double paddedTileScaleH = paddedTileSize / (maxY - minY + pointRadiusMeters * 2.0);
        for (size_t i = 0; i < points.size(); i++) {
            const double x = points.x[i], y = points.y[i], z = points.z[i];
            const int px = static_cast<int>(std::round((x - minX - pointRadiusMeters) * paddedTileScaleW));
            const int py =
                paddedTileSize - 1 - static_cast<int>(std::round((y - minY + pointRadiusMeters) * paddedTileScaleH));
            if (px >= 0 && px < paddedTileSize && py >= 0 && py < paddedTileSize &&
                static_cast<double>(zBuffer[static_cast<size_t>(py * paddedTileSize + px)]) < z) {
                zBuffer[static_cast<size_t>(py * paddedTileSize + px)] = static_cast<float>(z);
                const auto c = [&](uint16_t v) { return sixteenBit ? uint8_t(v >> 8) : uint8_t(v); };
                ReferenceDrawCircle(buffer.data(), alpha.data(), px - pointRadius, py - pointRadius, pointRadius,
                                    c(points.red[i]), c(points.green[i]), c(points.blue[i]), tileSize, wSize);
            }
        }

        CompatMapping m;
        m.originX = minX;
        m.shiftX = -pointRadiusMeters;
        m.scaleX = paddedTileScaleW;
        m.originY = minY;
        m.shiftY = pointRadiusMeters;
        m.scaleY = paddedTileScaleH;
        m.width = m.height = paddedTileSize;

        // Two batches: the result must not depend on how points are chunked
        CompatRasterizer rasterizer;
        rasterizer.Reset(tileSize, m, pointRadius, pointRadius);
        const size_t split = points.size() / 3;
        rasterizer.Add(points.x.data(), points.y.data(), points.z.data(), points.red.data(), points.green.data(),
                       points.blue.data(), split);
        rasterizer.Add(points.x.data() + split, points.y.data() + split, points.z.data() + split,
                       points.red.data() + split, points.green.data() + split, points.blue.data() + split,
                       points.size() - split);
        std::vector<uint8_t> color, a;
        rasterizer.ToPlanes(color, a);
        EXPECT_EQ(color, buffer) << "16-bit: " << sixteenBit;
        EXPECT_EQ(a, alpha) << "16-bit: " << sixteenBit;
        EXPECT_EQ(rasterizer.sixteenBit(), sixteenBit);
        EXPECT_GT(rasterizer.drawn(), 1000u);

        // The interleaved form carries the same pixels.
        std::vector<uint8_t> rgba(static_cast<size_t>(wSize) * 4);
        const size_t opaque = rasterizer.ToRgba(rgba.data());
        size_t expectOpaque = 0;
        bool same = true;
        for (int i = 0; i < wSize; ++i) {
            const size_t u = static_cast<size_t>(i);
            expectOpaque += alpha[u] != 0;
            same = same && rgba[u * 4] == buffer[u] && rgba[u * 4 + 1] == buffer[u + static_cast<size_t>(wSize)] &&
                   rgba[u * 4 + 2] == buffer[u + 2 * static_cast<size_t>(wSize)] && rgba[u * 4 + 3] == alpha[u];
        }
        EXPECT_TRUE(same);
        EXPECT_EQ(opaque, expectOpaque);

        // Reset reuses the buffers and starts clean.
        rasterizer.Reset(tileSize, m, pointRadius, pointRadius);
        EXPECT_EQ(rasterizer.drawn(), 0u);
        EXPECT_FALSE(rasterizer.sixteenBit());
        rasterizer.ToPlanes(color, a);
        EXPECT_TRUE(std::all_of(a.begin(), a.end(), [](uint8_t v) { return v == 0; }));
    }
}

TEST(CopcTilerRasterizer, CompatSkipsNonFiniteAndHugeCoordinates) {
    CompatMapping m;
    m.scaleX = m.scaleY = 1;
    m.width = m.height = 20;
    CompatRasterizer r;
    r.Reset(16, m, 2, 2);
    const double xs[] = {HUGE_VAL, 5, 1e300, 5, NAN};
    const double ys[] = {5, HUGE_VAL, 5, -1e300, 5};
    const double zs[] = {1, 1, 1, 1, 1};
    const uint16_t c[] = {300, 1, 1, 1, 1};
    r.Add(xs, ys, zs, c, c, c, 5);
    EXPECT_EQ(r.drawn(), 0u);
    EXPECT_TRUE(r.sixteenBit());  // checked before the finiteness test, as in the baseline
}

using Disc = SplatRasterizer::Disc;

// The disc offsets written out: the 13-pixel disc dx^2 + dy^2 <= 4, and PointRasterizer's
// drawCircle loop for radius 2 (i over 4 x 4 cells: tx = i % 4 - 2, ty = i / 4 - 2, within r^2).
std::vector<std::array<int, 2>> DiscOffsets(Disc disc) {
    std::vector<std::array<int, 2>> out;
    if (disc == Disc::kStamp11) {
        for (int i = 0; i < 16; ++i) {
            const int tx = (i % 4) - 2, ty = (i / 4) - 2;
            if (tx * tx + ty * ty <= 4) out.push_back({tx, ty});
        }
    } else {
        for (int dy = -2; dy <= 2; ++dy)
            for (int dx = -2; dx <= 2; ++dx)
                if (dx * dx + dy * dy <= 4) out.push_back({dx, dy});
    }
    return out;
}

// Brute force: every tile pixel takes the largest key of the points whose disc covers it.
std::vector<uint32_t> BruteSplat(int T, Disc disc, const std::vector<int>& cx, const std::vector<int>& cy,
                                 const std::vector<uint64_t>& keys) {
    const auto offsets = DiscOffsets(disc);
    std::vector<uint64_t> best(static_cast<size_t>(T) * static_cast<size_t>(T), 0);
    for (size_t i = 0; i < keys.size(); ++i)
        for (const auto& [dx, dy] : offsets) {
            const int x = cx[i] - SplatRasterizer::kRadius + dx, y = cy[i] - SplatRasterizer::kRadius + dy;
            if (x < 0 || y < 0 || x >= T || y >= T) continue;
            uint64_t& b = best[static_cast<size_t>(y) * static_cast<size_t>(T) + static_cast<size_t>(x)];
            b = std::max(b, keys[i]);
        }
    std::vector<uint32_t> out(best.size());
    for (size_t i = 0; i < best.size(); ++i) out[i] = static_cast<uint32_t>(best[i]);
    return out;
}

TEST(CopcTilerRasterizer, DiscsHaveTheirShapes) {
    EXPECT_EQ(DiscOffsets(Disc::kDisc13).size(), 13u);
    EXPECT_EQ(DiscOffsets(Disc::kStamp11).size(), 11u);
    for (const Disc disc : {Disc::kDisc13, Disc::kStamp11}) {
        size_t n = 0;
        for (int dy = -3; dy <= 3; ++dy)
            for (int dx = -3; dx <= 3; ++dx) n += SplatRasterizer::Covers(disc, dx, dy);
        EXPECT_EQ(n, DiscOffsets(disc).size());
        for (const auto& [dx, dy] : DiscOffsets(disc)) EXPECT_TRUE(SplatRasterizer::Covers(disc, dx, dy));
    }
}

TEST(CopcTilerRasterizer, StampIsPointRasterizersDisc) {
    // One point: PointRasterizer draws its disc at (px - 2, py - 2) of the image; the splat
    // rasterizer's stamp anchored on the same image pixel covers the same pixels.
    constexpr int T = 16;
    for (const int at : {0, 1, 7, 14, 15}) {
        CompatRasterizer c;
        CompatMapping m;
        m.width = m.height = T + 4;
        c.Reset(T, m, 2, 2);
        // py = height - 1 - round(y): image row at = py - 2.
        const double x = at + 2, y = (T + 4 - 1) - (at + 2), z = 1;
        const uint16_t r = 10, g = 20, b = 30;
        c.Add(&x, &y, &z, &r, &g, &b, 1);
        std::vector<uint8_t> want(static_cast<size_t>(T) * T * 4);
        c.ToRgba(want.data());

        SplatRasterizer s;
        s.Reset(T, Disc::kStamp11);
        s.Add(at + SplatRasterizer::kRadius, at + SplatRasterizer::kRadius, SplatKey(1.0f, 0xFF1E140Au));
        std::vector<uint8_t> got(want.size());
        s.Resolve(got.data());
        EXPECT_EQ(got, want) << "anchor " << at;
    }
}

TEST(CopcTilerRasterizer, SplatEqualsTheMaximumKeyOverTheDiscAndIgnoresOrder) {
    std::mt19937 rng(7);
    for (const Disc disc : {Disc::kDisc13, Disc::kStamp11}) {
        for (const int T : {16, 256, 512}) {
            for (const size_t n : {size_t{0}, size_t{1}, size_t{50}, size_t{20000}}) {
                const int P = T + 2 * SplatRasterizer::kRadius;
                std::vector<int> cx(n), cy(n);
                std::vector<uint64_t> keys(n);
                std::uniform_int_distribution<int> pos(0, P - 1);
                std::uniform_real_distribution<float> z(-50.f, 50.f);
                for (size_t i = 0; i < n; ++i) {
                    cx[i] = pos(rng);
                    cy[i] = pos(rng);
                    // A few equal heights to exercise the colour tie-break.
                    const float zi = (i % 7 == 0) ? 3.0f : z(rng);
                    keys[i] = SplatKey(zi, static_cast<uint32_t>(rng()) | 0xFF000000u);
                }
                const auto expect = BruteSplat(T, disc, cx, cy, keys);

                SplatRasterizer s;
                std::vector<uint8_t> rgba(static_cast<size_t>(T) * static_cast<size_t>(T) * 4);
                for (int round = 0; round < 3; ++round) {
                    // Same points in another order each round (and the buffers reused; the disc
                    // switched between rounds).
                    std::vector<size_t> perm(n);
                    for (size_t i = 0; i < n; ++i) perm[i] = i;
                    std::shuffle(perm.begin(), perm.end(), rng);
                    if (round == 1) {
                        s.Reset(T, disc == Disc::kStamp11 ? Disc::kDisc13 : Disc::kStamp11);
                        s.Add(P / 2, P / 2, SplatKey(1.0f, 0xFF000001u));
                        s.Resolve(rgba.data());
                    }
                    s.Reset(T, disc);
                    for (const size_t i : perm) s.Add(cx[i], cy[i], keys[i]);
                    const size_t opaque = s.Resolve(rgba.data());
                    std::vector<uint32_t> got(static_cast<size_t>(T) * static_cast<size_t>(T));
                    std::memcpy(got.data(), rgba.data(), rgba.size());
                    ASSERT_EQ(got, expect) << "disc " << static_cast<int>(disc) << " T " << T << " n " << n
                                           << " round " << round;
                    EXPECT_EQ(opaque, static_cast<size_t>(std::count_if(expect.begin(), expect.end(),
                                                                          [](uint32_t v) { return v != 0; })));
                }
            }
        }
    }
}

// Brute force of ResolveAdaptive: the anchors are the buffer pixels holding a point (with their
// largest key); each draws the disc dx^2 + dy^2 <= r^2 of r = AdaptiveRadius(the squared distance to
// its kNeighbours-th nearest other anchor within AdaptiveSearch(maxRadius) along each axis).
std::vector<uint32_t> BruteAdaptive(int T, int pad, int maxRadius, const std::vector<int>& cx,
                                    const std::vector<int>& cy, const std::vector<uint64_t>& keys) {
    const int P = T + 2 * pad;
    std::vector<uint64_t> anchor(static_cast<size_t>(P) * static_cast<size_t>(P), 0);
    for (size_t i = 0; i < keys.size(); ++i) {
        uint64_t& a = anchor[static_cast<size_t>(cy[i]) * static_cast<size_t>(P) + static_cast<size_t>(cx[i])];
        a = std::max(a, keys[i]);
    }
    std::vector<std::array<int, 2>> at;
    for (int y = 0; y < P; ++y)
        for (int x = 0; x < P; ++x)
            if (anchor[static_cast<size_t>(y) * static_cast<size_t>(P) + static_cast<size_t>(x)] != 0) at.push_back({x, y});
    const int search = SplatRasterizer::AdaptiveSearch(maxRadius);
    std::vector<uint64_t> best(static_cast<size_t>(T) * static_cast<size_t>(T), 0);
    for (const auto& [x, y] : at) {
        std::vector<int> d2;
        for (const auto& [ox, oy] : at)
            if ((ox != x || oy != y) && std::abs(ox - x) <= search && std::abs(oy - y) <= search)
                d2.push_back((ox - x) * (ox - x) + (oy - y) * (oy - y));
        std::sort(d2.begin(), d2.end());
        const int k = SplatRasterizer::kNeighbours;
        const int r = SplatRasterizer::AdaptiveRadius(d2.size() >= static_cast<size_t>(k) ? d2[k - 1] : 0, maxRadius);
        const uint64_t key = anchor[static_cast<size_t>(y) * static_cast<size_t>(P) + static_cast<size_t>(x)];
        for (int dy = -r; dy <= r; ++dy)
            for (int dx = -r; dx <= r; ++dx) {
                const int px = x - pad + dx, py = y - pad + dy;
                if (dx * dx + dy * dy > r * r || px < 0 || py < 0 || px >= T || py >= T) continue;
                uint64_t& b = best[static_cast<size_t>(py) * static_cast<size_t>(T) + static_cast<size_t>(px)];
                b = std::max(b, key);
            }
    }
    std::vector<uint32_t> out(best.size());
    for (size_t i = 0; i < best.size(); ++i) out[i] = static_cast<uint32_t>(best[i]);
    return out;
}

std::vector<uint32_t> Pixels(const std::vector<uint8_t>& rgba) {
    std::vector<uint32_t> out(rgba.size() / 4);
    std::memcpy(out.data(), rgba.data(), rgba.size());
    return out;
}

TEST(CopcTilerRasterizer, AdaptiveRadiusFollowsTheSpacingOfTheNeighbours) {
    constexpr int r = SplatRasterizer::kRadius;
    EXPECT_EQ(SplatRasterizer::AdaptiveRadius(0, 32), r);     // fewer neighbours than counted: isolated
    EXPECT_EQ(SplatRasterizer::AdaptiveRadius(1, 32), r);
    EXPECT_EQ(SplatRasterizer::AdaptiveRadius(8, 32), r);     // 2.83 px: the 13-pixel discs meet
    EXPECT_EQ(SplatRasterizer::AdaptiveRadius(9, 32), 3);     // ceil(0.8 x 3)
    EXPECT_EQ(SplatRasterizer::AdaptiveRadius(100, 32), 8);   // 10 px
    EXPECT_EQ(SplatRasterizer::AdaptiveRadius(1600, 32), 32); // 40 px: 32
    EXPECT_EQ(SplatRasterizer::AdaptiveRadius(1681, 32), 32); // beyond: capped
    EXPECT_EQ(SplatRasterizer::AdaptiveRadius(100, r), r);    // no growth allowed
    EXPECT_EQ(SplatRasterizer::AdaptiveSearch(32), 40);
    EXPECT_EQ(SplatRasterizer::AdaptivePad(32), 72);
}

TEST(CopcTilerRasterizer, AWiderAnchorPadDrawsTheSameDiscs) {
    std::mt19937 rng(11);
    for (const Disc disc : {Disc::kDisc13, Disc::kStamp11}) {
        constexpr int T = 64, extra = 7;
        std::uniform_int_distribution<int> pos(0, T + 2 * SplatRasterizer::kRadius - 1);
        SplatRasterizer narrow, wide;
        narrow.Reset(T, disc);
        wide.Reset(T, disc, SplatRasterizer::kRadius + extra);
        EXPECT_EQ(wide.PaddedSize(), T + 2 * (SplatRasterizer::kRadius + extra));
        for (int i = 0; i < 500; ++i) {
            const int x = pos(rng), y = pos(rng);
            const uint64_t key = SplatKey(static_cast<float>(i % 13), static_cast<uint32_t>(rng()) | 0xFF000000u);
            narrow.Add(x, y, key);
            wide.Add(x + extra, y + extra, key);
        }
        // Anchors of the wide buffer that no disc brings onto the tile
        wide.Add(0, 0, SplatKey(99.0f, 0xFFFFFFFFu));
        wide.Add(wide.PaddedSize() - 1, 3, SplatKey(99.0f, 0xFFFFFFFFu));
        std::vector<uint8_t> a(static_cast<size_t>(T) * T * 4), b(a.size());
        narrow.Resolve(a.data());
        wide.Resolve(b.data());
        EXPECT_EQ(a, b) << "disc " << static_cast<int>(disc);
    }
}

TEST(CopcTilerRasterizer, AdaptiveDiscsAreTheMaximumKeyOverDiscsSizedByTheirNeighbours) {
    std::mt19937 rng(5);
    SplatRasterizer s;  // reused across sizes, pads and orders
    for (const int maxRadius : {2, 6, 12}) {
        const int pad = SplatRasterizer::AdaptivePad(maxRadius);
        for (const int T : {16, 64, 128}) {
            const int P = T + 2 * pad;
            // From empty to every pixel taken: isolated points, clusters, sparse and dense spacings.
            for (const size_t n : {size_t{0}, size_t{1}, size_t{5}, size_t{60}, size_t{600}, size_t{6000}, size_t{40000}}) {
                std::vector<int> cx(n), cy(n);
                std::vector<uint64_t> keys(n);
                std::uniform_int_distribution<int> pos(0, P - 1);
                std::uniform_real_distribution<float> z(-50.f, 50.f);
                for (size_t i = 0; i < n; ++i) {
                    cx[i] = pos(rng);
                    cy[i] = pos(rng);
                    keys[i] = SplatKey((i % 5 == 0) ? 1.0f : z(rng), static_cast<uint32_t>(rng()) | 0xFF000000u);
                }
                const auto expect = BruteAdaptive(T, pad, maxRadius, cx, cy, keys);
                std::vector<uint8_t> rgba(static_cast<size_t>(T) * static_cast<size_t>(T) * 4);
                for (int round = 0; round < 2; ++round) {
                    std::vector<size_t> perm(n);
                    for (size_t i = 0; i < n; ++i) perm[i] = i;
                    std::shuffle(perm.begin(), perm.end(), rng);
                    s.Reset(T, Disc::kDisc13, pad);
                    for (const size_t i : perm) s.Add(cx[i], cy[i], keys[i]);
                    const size_t opaque = s.ResolveAdaptive(rgba.data(), maxRadius);
                    ASSERT_EQ(Pixels(rgba), expect) << "maxRadius " << maxRadius << " T " << T << " n " << n << " round " << round;
                    EXPECT_EQ(opaque, static_cast<size_t>(std::count_if(expect.begin(), expect.end(),
                                                                          [](uint32_t v) { return v != 0; })));
                }
            }
        }
    }
}

TEST(CopcTilerRasterizer, AdaptiveDiscsJoinASquareGridOfAnyPitchUpToTheLargestDisc) {
    constexpr int T = 128, maxRadius = 32;
    const int pad = SplatRasterizer::AdaptivePad(maxRadius);
    const int P = T + 2 * pad;
    SplatRasterizer s, plain;
    std::vector<uint8_t> rgba(static_cast<size_t>(T) * T * 4), disc13(rgba.size());
    for (int pitch = 1; pitch <= 40; ++pitch) {
        const int shift = pitch / 3;  // grid not aligned on the tile
        s.Reset(T, Disc::kDisc13, pad);
        plain.Reset(T, Disc::kDisc13, pad);
        uint32_t colour = 0;
        for (int y = shift; y < P; y += pitch)
            for (int x = shift; x < P; x += pitch) {
                const uint64_t key = SplatKey(static_cast<float>((x * 7 + y * 3) % 11), 0xFF000000u | ++colour);
                s.Add(x, y, key);
                plain.Add(x, y, key);
            }
        const size_t opaque = s.ResolveAdaptive(rgba.data(), maxRadius);
        EXPECT_EQ(opaque, static_cast<size_t>(T) * T) << "pitch " << pitch << ": holes between the discs";
        plain.Resolve(disc13.data());
        // Points at most 2 sqrt(2) px apart: the 13-pixel disc, as Resolve draws it.
        if (pitch <= 2) {
            EXPECT_EQ(rgba, disc13) << "pitch " << pitch;
        } else {
            EXPECT_NE(rgba, disc13) << "pitch " << pitch;
        }
    }
}

TEST(CopcTilerRasterizer, OrderableZKeepsTheOrderOfFloats) {
    const float v[] = {-INFINITY, -1e30f, -2.5f, -1e-30f, -0.0f, 0.0f, 1e-30f, 1.0f, 2.5f, 1e30f, INFINITY};
    for (size_t i = 0; i + 1 < std::size(v); ++i)
        EXPECT_LT(ddbt::copc::tiler::OrderableZ(v[i]), ddbt::copc::tiler::OrderableZ(v[i + 1])) << v[i];
    EXPECT_NE(SplatKey(-INFINITY, 0xFF000000u), 0u);
}

}  // namespace
