// SPDX-License-Identifier: MPL-2.0
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <random>
#include <vector>

#include "core/encode/kernels.h"
#include "core/simd/simd.h"

namespace k = ddbt::encode::kernels;

namespace {

// Restores automatic target selection when a test forces one.
struct ForcedTargetGuard {
    ~ForcedTargetGuard() { ddbt::simd::ForceTargetForTest(0); }
};

// Bytes that stress every branch: random, extremes, and runs (Paeth ties).
std::vector<uint8_t> TestBytes(size_t n, uint32_t seed) {
    std::mt19937 rng(seed);
    std::vector<uint8_t> v(n);
    for (size_t i = 0; i < n; ++i) {
        switch (rng() % 6) {
            case 0: v[i] = 0; break;
            case 1: v[i] = 255; break;
            case 2: v[i] = i > 0 ? v[i - 1] : 7; break;
            case 3: v[i] = static_cast<uint8_t>(127 + rng() % 3); break;
            default: v[i] = static_cast<uint8_t>(rng()); break;
        }
    }
    return v;
}

uint8_t RefPaeth(int a, int b, int c) {
    const int p = a + b - c;
    const int pa = std::abs(p - a), pb = std::abs(p - b), pc = std::abs(p - c);
    if (pa <= pb && pa <= pc) return static_cast<uint8_t>(a);
    if (pb <= pc) return static_cast<uint8_t>(b);
    return static_cast<uint8_t>(c);
}

// The PNG specification, byte by byte.
std::vector<uint8_t> RefFilter(const uint8_t* cur, const uint8_t* prev, size_t n, size_t bpp, int filter) {
    std::vector<uint8_t> out(n);
    for (size_t i = 0; i < n; ++i) {
        const int x = cur[i];
        const int a = i >= bpp ? cur[i - bpp] : 0;
        const int b = prev[i];
        const int c = i >= bpp ? prev[i - bpp] : 0;
        int v = x;
        if (filter == 1) v = x - a;
        if (filter == 2) v = x - b;
        if (filter == 3) v = x - (a + b) / 2;
        if (filter == 4) v = x - RefPaeth(a, b, c);
        out[i] = static_cast<uint8_t>(v);
    }
    return out;
}

uint64_t RefCost(const std::vector<uint8_t>& row) {
    uint64_t s = 0;
    for (const uint8_t v : row) s += v < 128 ? v : 256u - v;
    return s;
}

void CheckFiltersOnCurrentTarget() {
    for (const size_t bpp : {size_t{1}, size_t{2}, size_t{3}, size_t{4}}) {
        for (const size_t n : {size_t{1}, size_t{2}, size_t{3}, size_t{4}, size_t{5}, size_t{31}, size_t{32}, size_t{33},
                               size_t{63}, size_t{64}, size_t{65}, size_t{127}, size_t{200}, size_t{768}, size_t{1027},
                               size_t{2048}}) {
            if (n < bpp) continue;
            const auto cur = TestBytes(n, static_cast<uint32_t>(n * 7 + bpp));
            const auto prev = TestBytes(n, static_cast<uint32_t>(n * 13 + bpp + 1));
            uint64_t costs[5];
            k::PngFilterCosts(cur.data(), prev.data(), n, bpp, costs);
            for (int f = 0; f < 5; ++f) {
                const auto ref = RefFilter(cur.data(), prev.data(), n, bpp, f);
                std::vector<uint8_t> out(n, 0xCD);
                k::PngFilterRow(cur.data(), prev.data(), n, bpp, static_cast<k::PngFilterType>(f), out.data());
                ASSERT_EQ(out, ref) << "filter " << f << " bpp " << bpp << " n " << n;
                ASSERT_EQ(costs[f], RefCost(ref)) << "cost of filter " << f << " bpp " << bpp << " n " << n;
            }
        }
    }
}

}  // namespace

TEST(EncodeKernels, PngFiltersMatchTheSpecificationOnEveryTarget) {
    ForcedTargetGuard guard;
    for (const int64_t bit : ddbt::simd::SupportedTargetBits()) {
        ddbt::simd::ForceTargetForTest(bit);
        SCOPED_TRACE(ddbt::simd::TargetNameOf(bit));
        CheckFiltersOnCurrentTarget();
    }
}

TEST(EncodeKernels, PaethMatchesForEveryByteTriple) {
    // The byte-domain Paeth (no widening) against the definition, for all 2^24 (a, b, c). With
    // bpp = 65536 the second half of a row sees independent a (first half of cur), b (second
    // half of prev) and c (first half of prev).
    const size_t bpp = 65536, n = 2 * bpp;
    std::vector<uint8_t> cur(n), prev(n), out(n);
    for (size_t j = 0; j < bpp; ++j) {
        cur[j] = static_cast<uint8_t>(j & 255u);         // a
        prev[j] = static_cast<uint8_t>(j >> 8);          // c
        cur[bpp + j] = static_cast<uint8_t>(j * 7u + 3u);  // x
    }
    ForcedTargetGuard guard;
    for (const int64_t bit : ddbt::simd::SupportedTargetBits()) {
        ddbt::simd::ForceTargetForTest(bit);
        for (int b = 0; b < 256; ++b) {
            for (size_t j = 0; j < bpp; ++j) prev[bpp + j] = static_cast<uint8_t>(b);
            k::PngFilterRow(cur.data(), prev.data(), n, bpp, k::kFilterPaeth, out.data());
            for (size_t j = 0; j < bpp; ++j) {
                const uint8_t expected = static_cast<uint8_t>(cur[bpp + j] - RefPaeth(cur[j], b, prev[j]));
                ASSERT_EQ(out[bpp + j], expected) << ddbt::simd::TargetNameOf(bit) << " a " << int{cur[j]} << " b "
                                                  << b << " c " << int{prev[j]};
            }
        }
    }
}

TEST(EncodeKernels, AlphaRangeMatchesScalar) {
    ForcedTargetGuard guard;
    for (const int64_t bit : ddbt::simd::SupportedTargetBits()) {
        ddbt::simd::ForceTargetForTest(bit);
        for (const uint32_t ch : {2u, 4u}) {
            for (const size_t px : {size_t{1}, size_t{7}, size_t{16}, size_t{17}, size_t{255}, size_t{256}, size_t{513}}) {
                for (int pattern = 0; pattern < 4; ++pattern) {
                    std::mt19937 rng(static_cast<uint32_t>(px * 31 + ch + static_cast<size_t>(pattern)));
                    std::vector<uint8_t> row(px * ch);
                    for (auto& v : row) v = static_cast<uint8_t>(rng());
                    for (size_t i = 0; i < px; ++i) {
                        uint8_t& a = row[i * ch + ch - 1];
                        if (pattern == 0) a = 255;
                        if (pattern == 1) a = 0;
                        if (pattern == 2) a = (i == px - 1) ? 3 : 255;  // the tail pixel decides
                    }
                    uint8_t mn = 255, mx = 0, rmn = 255, rmx = 0;
                    for (size_t i = 0; i < px; ++i) {
                        const uint8_t a = row[i * ch + ch - 1];
                        rmn = a < rmn ? a : rmn;
                        rmx = a > rmx ? a : rmx;
                    }
                    k::AlphaRange(row.data(), px, ch, mn, mx);
                    ASSERT_EQ(mn, rmn) << ddbt::simd::TargetNameOf(bit) << " ch " << ch << " px " << px;
                    ASSERT_EQ(mx, rmx) << ddbt::simd::TargetNameOf(bit) << " ch " << ch << " px " << px;
                }
            }
        }
    }
}

TEST(EncodeKernels, ConversionsMatchScalar) {
    ForcedTargetGuard guard;
    for (const int64_t bit : ddbt::simd::SupportedTargetBits()) {
        ddbt::simd::ForceTargetForTest(bit);
        SCOPED_TRACE(ddbt::simd::TargetNameOf(bit));
        for (const size_t px : {size_t{1}, size_t{15}, size_t{64}, size_t{100}, size_t{257}}) {
            auto rgba = TestBytes(px * 4, static_cast<uint32_t>(px));
            auto ga = TestBytes(px * 2, static_cast<uint32_t>(px + 1));
            std::vector<uint8_t> rgb(px * 3), g(px), rgbx(px * 4), g2(px), rgb2(px * 3), rgba2(px * 4);
            k::DropAlpha4(rgba.data(), px, rgb.data());
            k::DropAlpha2(ga.data(), px, g.data());
            k::WhiteUnderTransparent4(rgba.data(), px, rgbx.data());
            k::WhiteUnderTransparent2(ga.data(), px, g2.data());
            k::GreyToRgb(g.data(), px, rgb2.data());
            k::GreyAlphaToRgba(ga.data(), px, rgba2.data());
            for (size_t i = 0; i < px; ++i) {
                const bool clear4 = rgba[4 * i + 3] == 0;
                const bool clear2 = ga[2 * i + 1] == 0;
                for (size_t c = 0; c < 3; ++c) {
                    ASSERT_EQ(rgb[3 * i + c], rgba[4 * i + c]);
                    ASSERT_EQ(rgbx[4 * i + c], clear4 ? 255 : rgba[4 * i + c]);
                    ASSERT_EQ(rgb2[3 * i + c], g[i]);
                    ASSERT_EQ(rgba2[4 * i + c], ga[2 * i]);
                }
                ASSERT_EQ(g[i], ga[2 * i]);
                ASSERT_EQ(g2[i], clear2 ? 255 : ga[2 * i]);
                ASSERT_EQ(rgba2[4 * i + 3], ga[2 * i + 1]);
            }
        }
    }
}

TEST(EncodeKernels, FilterImageDropsAlphaAndPicksTheCheapestFilter) {
    const uint32_t w = 37, h = 9;
    const size_t stride = static_cast<size_t>(w) * 4 + 5;  // odd stride
    const auto src = TestBytes(stride * h, 99);
    const size_t n = static_cast<size_t>(w) * 3;
    std::vector<uint8_t> out(h * (n + 1)), scratch(3 * n);
    k::PngFilterJob job;
    job.src = src.data();
    job.srcStride = stride;
    job.width = w;
    job.height = h;
    job.srcChannels = 4;
    job.dstChannels = 3;
    job.filter = k::PngRowFilter::kAdaptive;
    job.out = out.data();
    job.scratch = scratch.data();
    k::PngFilterImage(job);

    std::vector<uint8_t> prev(n, 0), cur(n);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            for (uint32_t c = 0; c < 3; ++c) cur[x * 3 + c] = src[y * stride + x * 4 + c];
        }
        uint64_t best = UINT64_MAX;
        int bestFilter = 0;
        std::vector<uint8_t> bestRow;
        for (int f = 0; f < 5; ++f) {
            const auto r = RefFilter(cur.data(), prev.data(), n, 3, f);
            const uint64_t cost = RefCost(r);
            if (cost < best) {
                best = cost;
                bestFilter = f;
                bestRow = r;
            }
        }
        ASSERT_EQ(out[y * (n + 1)], bestFilter) << "row " << y;
        ASSERT_TRUE(std::equal(bestRow.begin(), bestRow.end(), out.begin() + y * (n + 1) + 1)) << "row " << y;
        prev = cur;
    }
}
