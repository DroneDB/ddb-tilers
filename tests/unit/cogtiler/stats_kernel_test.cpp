// SPDX-License-Identifier: MPL-2.0
// The statistics kernels shared by the tiler and the COG builder (cog/tiler/stats_kernel.h):
// Welford as DroneDB computes it, exact sums independent of chunking and merge order for
// integer samples, the correctly rounded mean.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "cog/tiler/stats_kernel.h"

namespace stats = ddbt::cog::stats;
using ddbt::cog::tiff::SampleType;

namespace {

template <typename T>
std::vector<uint8_t> Bytes(const std::vector<T>& v) {
    std::vector<uint8_t> b(v.size() * sizeof(T));
    std::memcpy(b.data(), v.data(), b.size());
    return b;
}

// Runs of `count` samples (one band, step = sizeof(T)) split at `cuts`, each a chunk; merged in
// `order` (indexes of the chunks).
template <typename T>
stats::ExactSums ExactChunks(const std::vector<T>& v, const std::vector<uint8_t>& mask, const stats::SampleFilter& f,
                             std::vector<uint32_t> cuts, bool reverse) {
    const auto bytes = Bytes(v);
    cuts.insert(cuts.begin(), 0);
    cuts.push_back(static_cast<uint32_t>(v.size()));
    std::vector<stats::ExactSums> chunks;
    for (size_t k = 0; k + 1 < cuts.size(); ++k) {
        stats::ExactSums c;
        // Each chunk itself in two runs.
        const uint32_t a = cuts[k], b = cuts[k + 1], mid = a + (b - a) / 3;
        stats::AddRun(c, f, bytes.data() + a * sizeof(T), sizeof(T), mid - a, mask.empty() ? nullptr : mask.data() + a,
                      nullptr);
        stats::AddRun(c, f, bytes.data() + mid * sizeof(T), sizeof(T), b - mid,
                      mask.empty() ? nullptr : mask.data() + mid, nullptr);
        chunks.push_back(c);
    }
    if (reverse) std::reverse(chunks.begin(), chunks.end());
    stats::ExactSums total;
    for (const auto& c : chunks) stats::Merge(total, c);
    return total;
}

template <typename T>
void CheckIntegerExact(SampleType type, T lo, T hi, uint64_t seed) {
    std::mt19937_64 rng(seed);
    const size_t n = 200003;
    std::vector<T> v(n);
    std::vector<uint8_t> mask(n);
    for (size_t i = 0; i < n; ++i) {
        const uint64_t range = static_cast<uint64_t>(static_cast<int64_t>(hi) - static_cast<int64_t>(lo)) + 1;
        v[i] = static_cast<T>(static_cast<int64_t>(lo) + static_cast<int64_t>(rng() % range));
        mask[i] = rng() % 7 == 0 ? 0 : 255;
    }
    const T nd = v[5];
    const stats::SampleFilter f{type, true, static_cast<double>(nd)};
    // Reference: exact integer sums (the sum below 2^53, so sum / n in doubles is the correctly
    // rounded mean).
    long double sumL = 0;
    int64_t sum = 0;
    uint64_t count = 0;
    double mn = 1e300, mx = -1e300;
    std::vector<double> kept;
    for (size_t i = 0; i < n; ++i) {
        if (mask[i] == 0 || v[i] == nd) continue;
        sum += static_cast<int64_t>(v[i]);
        sumL += static_cast<long double>(v[i]);
        ++count;
        mn = std::min(mn, static_cast<double>(v[i]));
        mx = std::max(mx, static_cast<double>(v[i]));
        kept.push_back(static_cast<double>(v[i]));
    }
    ASSERT_LT(std::abs(sumL), 9.0e15L);
    const double mean = static_cast<double>(sum) / static_cast<double>(count);
    double ss = 0;  // two-pass variance around the exact mean, for the std (not bit-compared)
    for (const double x : kept) ss += (x - mean) * (x - mean);
    const double sd = std::sqrt(ss / static_cast<double>(count));

    const auto a = stats::Finish(ExactChunks(v, mask, f, {}, false));
    const auto b = stats::Finish(ExactChunks(v, mask, f, {1, 999, 50000, 50001, 123457}, false));
    const auto c = stats::Finish(ExactChunks(v, mask, f, {1, 999, 50000, 50001, 123457}, true));
    ASSERT_TRUE(a && b && c);
    for (const auto* r : {&*a, &*b, &*c}) {
        EXPECT_EQ(r->valid, count);
        EXPECT_EQ(r->min, mn);
        EXPECT_EQ(r->max, mx);
        EXPECT_EQ(r->mean, mean) << "type " << static_cast<int>(type);
        EXPECT_NEAR(r->stdDev, sd, 1e-12 * sd);
    }
    // Chunking and merge order do not change a bit.
    EXPECT_EQ(a->mean, c->mean);
    EXPECT_EQ(a->stdDev, b->stdDev);
    EXPECT_EQ(a->stdDev, c->stdDev);
}

}  // namespace

TEST(StatsKernel, IntegerSumsAreExactAndOrderFree) {
    CheckIntegerExact<uint8_t>(SampleType::kU8, 0, 255, 1);
    CheckIntegerExact<int8_t>(SampleType::kI8, -128, 127, 2);
    CheckIntegerExact<uint16_t>(SampleType::kU16, 0, 65535, 3);
    CheckIntegerExact<int16_t>(SampleType::kI16, -32768, 32767, 4);
    CheckIntegerExact<uint32_t>(SampleType::kU32, 0, 4000000000u, 5);
    CheckIntegerExact<int32_t>(SampleType::kI32, -2000000000, 2000000000, 6);
}

TEST(StatsKernel, IntegerMeanIsCorrectlyRoundedWhereWelfordIsNot) {
    // 10^7 bytes: Welford's running mean drifts by its rounding errors, the exact sums give
    // sum / n correctly rounded.
    std::mt19937_64 rng(11);
    std::vector<uint8_t> v(10000000);
    int64_t sum = 0;
    for (auto& x : v) {
        x = static_cast<uint8_t>(rng() % 256);
        sum += x;
    }
    const stats::SampleFilter f{SampleType::kU8, false, 0};
    stats::ExactSums e;
    stats::Welford w;
    stats::AddRun(e, f, v.data(), 1, static_cast<uint32_t>(v.size()), nullptr, nullptr);
    stats::AddRun(w, f, v.data(), 1, static_cast<uint32_t>(v.size()), nullptr, nullptr);
    const double exact = static_cast<double>(sum) / static_cast<double>(v.size());
    EXPECT_EQ(stats::Finish(e)->mean, exact);
    EXPECT_NEAR(stats::Finish(w)->mean, exact, 1e-9);  // close, not the same bits in general
}

TEST(StatsKernel, FloatSumsOfRepresentableValuesAreExact) {
    // Multiples of 2^-10 below 2^20: every double-double sum is exact, so the result is the
    // correctly rounded mean and independent of chunking.
    std::mt19937_64 rng(7);
    const size_t n = 100001;
    std::vector<float> v(n);
    std::vector<uint8_t> mask;
    int64_t k = 0;
    size_t count = 0;
    for (size_t i = 0; i < n; ++i) {
        const int64_t q = static_cast<int64_t>(rng() % (uint64_t{1} << 30)) - (int64_t{1} << 29);
        v[i] = static_cast<float>(static_cast<double>(q) / 1024.0);
        if (i % 1000 == 3) v[i] = std::numeric_limits<float>::quiet_NaN();
        if (i % 1000 == 5) v[i] = -std::numeric_limits<float>::infinity();
        if (i % 1000 == 7) v[i] = -32767.0f;
        if (!std::isfinite(v[i]) || v[i] == -32767.0f) continue;
        k += static_cast<int64_t>(static_cast<double>(v[i]) * 1024.0);
        ++count;
    }
    const stats::SampleFilter f{SampleType::kF32, true, -32767.0};
    const auto a = stats::Finish(ExactChunks(v, mask, f, {}, false));
    const auto b = stats::Finish(ExactChunks(v, mask, f, {10, 20000, 77777}, true));
    ASSERT_TRUE(a && b);
    EXPECT_EQ(a->valid, count);
    EXPECT_EQ(a->mean, static_cast<double>(k) / (1024.0 * static_cast<double>(count)));
    EXPECT_EQ(a->mean, b->mean);
    EXPECT_EQ(a->stdDev, b->stdDev);
}

TEST(StatsKernel, WelfordIsDroneDbsLoop) {
    std::mt19937_64 rng(3);
    const uint32_t n = 50000;
    std::vector<float> v(n);
    std::vector<uint8_t> mask(n);
    for (uint32_t i = 0; i < n; ++i) {
        v[i] = static_cast<float>(rng() % 100000) / 7.0f;
        mask[i] = rng() % 9 == 0 ? 0 : 255;
    }
    v[17] = 0.1f;
    const auto bytes = Bytes(v);
    // NoData 0.1 compared as float (the double 0.1 is not the float 0.1).
    const stats::SampleFilter f{SampleType::kF32, true, 0.1};
    stats::Welford w;
    stats::AddRun(w, f, bytes.data(), 4, 1000, mask.data(), nullptr);
    stats::AddRun(w, f, bytes.data() + 4000, 4, n - 1000, mask.data() + 1000, nullptr);
    double mean = 0, m2 = 0, lo = std::numeric_limits<double>::infinity(), hi = -lo;
    uint64_t c = 0;
    for (uint32_t i = 0; i < n; ++i) {
        const double x = v[i];
        if (mask[i] == 0 || static_cast<float>(x) == 0.1f) continue;
        ++c;
        lo = std::min(lo, x);
        hi = std::max(hi, x);
        const double d = x - mean;
        mean += d / static_cast<double>(c);
        m2 += d * (x - mean);
    }
    const auto r = stats::Finish(w);
    ASSERT_TRUE(r);
    EXPECT_EQ(r->valid, c);
    EXPECT_EQ(r->mean, mean);
    EXPECT_EQ(r->stdDev, std::sqrt(m2 / static_cast<double>(c)));
    EXPECT_EQ(r->min, lo);
    EXPECT_EQ(r->max, hi);
}

TEST(StatsKernel, SixteenBitAlphaIsRescaled) {
    // GDAL's mask of a 16-bit alpha band is alpha / 257: 256 masks the pixel, 257 does not.
    const std::vector<uint16_t> px = {10, 256, 20, 257, 30, 0, 40, 65535};  // (value, alpha) pairs
    const auto bytes = Bytes(px);
    const stats::SampleFilter f{SampleType::kU16, false, 0};
    stats::Welford w;
    stats::ExactSums e;
    stats::AddRun(w, f, bytes.data(), 4, 4, nullptr, bytes.data() + 2);
    stats::AddRun(e, f, bytes.data(), 4, 4, nullptr, bytes.data() + 2);
    EXPECT_EQ(stats::Finish(w)->valid, 2u);
    EXPECT_EQ(stats::Finish(e)->valid, 2u);
    EXPECT_EQ(stats::Finish(e)->mean, 30.0);
    EXPECT_EQ(stats::Finish(w)->min, 20.0);
    EXPECT_FALSE(stats::Finish(stats::ExactSums{}));
    EXPECT_FALSE(stats::Finish(stats::Welford{}));
}

namespace {
// Sets (or unsets, value == nullptr) DDBT_STATS_MODE for the scope and restores it.
class ScopedStatsMode {
public:
    explicit ScopedStatsMode(const char* value) {
        if (const char* old = std::getenv("DDBT_STATS_MODE")) {
            had_ = true;
            old_ = old;
        }
        Set(value);
    }
    ~ScopedStatsMode() { Set(had_ ? old_.c_str() : nullptr); }

private:
    static void Set(const char* v) {
        if (v) {
            setenv("DDBT_STATS_MODE", v, 1);
        } else {
            unsetenv("DDBT_STATS_MODE");
        }
    }
    bool had_ = false;
    std::string old_;
};
}  // namespace

TEST(StatsKernel, ExactIsTheDefaultMode) {
    {
        ScopedStatsMode m(nullptr);
        EXPECT_EQ(stats::StatsModeFromEnv(), stats::StatsMode::kExact);
    }
    {
        ScopedStatsMode m("");
        EXPECT_EQ(stats::StatsModeFromEnv(), stats::StatsMode::kExact);
    }
    {
        ScopedStatsMode m("exact");
        EXPECT_EQ(stats::StatsModeFromEnv(), stats::StatsMode::kExact);
    }
    {
        ScopedStatsMode m("something-else");
        EXPECT_EQ(stats::StatsModeFromEnv(), stats::StatsMode::kExact);
    }
}

TEST(StatsKernel, CompatIsOptIn) {
    ScopedStatsMode m("compat");
    EXPECT_EQ(stats::StatsModeFromEnv(), stats::StatsMode::kCompat);
}
