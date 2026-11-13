// SPDX-License-Identifier: MPL-2.0
// Compiled without floating-point contraction and without LTO (sources.cmake): Welford's sums
// must round as DroneDB's (generic x86-64, no FMA) do, and the double-double sums rely on every
// addition being rounded on its own.
#include "cog/tiler/stats_kernel.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <type_traits>

namespace ddbt::cog::stats {

StatsMode StatsModeFromEnv() {
    const char* e = std::getenv("DDBT_STATS_MODE");
    return e != nullptr && std::strcmp(e, "compat") == 0 ? StatsMode::kCompat : StatsMode::kExact;
}

namespace {

template <typename T>
inline T LoadT(const uint8_t* p) {
    T t;
    std::memcpy(&t, p, sizeof(T));
    return t;
}

// The alpha sample as GDAL's mask of an alpha band reads it: 8-bit as is, 16-bit rescaled
// (GDALRescaledAlphaBand: alpha / 257). Alpha masks only 8 / 16-bit rasters.
template <typename T>
inline bool AlphaZero(const uint8_t* p) {
    const T a = LoadT<T>(p);
    if constexpr (std::is_same_v<T, uint16_t>) return (a / 257) == 0;
    return a == 0;
}

// --- double-double arithmetic (Knuth's TwoSum, FMA products) ------------------------------------

inline void TwoSum(double a, double b, double& s, double& e) {
    s = a + b;
    const double bb = s - a;
    e = (a - (s - bb)) + (b - bb);
}

inline void QuickTwoSum(double a, double b, double& s, double& e) {  // |a| >= |b|
    s = a + b;
    e = b - (s - a);
}

inline void AddD(DoubleDouble& x, double v) {
    double s, e;
    TwoSum(x.hi, v, s, e);
    e += x.lo;
    QuickTwoSum(s, e, x.hi, x.lo);
}

inline void AddDD(DoubleDouble& x, const DoubleDouble& y) {
    double s, e, t, f;
    TwoSum(x.hi, y.hi, s, e);
    TwoSum(x.lo, y.lo, t, f);
    e += t;
    QuickTwoSum(s, e, s, e);
    e += f;
    QuickTwoSum(s, e, x.hi, x.lo);
}

inline DoubleDouble TwoProd(double a, double b) {
    const double p = a * b;
    return {p, std::fma(a, b, -p)};
}

// An integer below 2^62 in magnitude, exactly.
inline void AddInt(DoubleDouble& x, int64_t v) {
    const double h = static_cast<double>(v);
    AddDD(x, {h, static_cast<double>(v - static_cast<int64_t>(h))});
}

// v * 2^shift, exactly (v below 2^62).
inline void AddScaled(DoubleDouble& x, uint64_t v, int shift) {
    const double h = static_cast<double>(v);
    const double l = static_cast<double>(static_cast<int64_t>(v - static_cast<uint64_t>(h)));
    AddDD(x, {std::ldexp(h, shift), std::ldexp(l, shift)});
}

// --- Welford --------------------------------------------------------------------------------------

template <typename T>
void WelfordRun(Welford& acc, const SampleFilter& f, const uint8_t* p, size_t step, uint32_t count,
                const uint8_t* mask, const uint8_t* alpha) {
    // Local copies: the samples are read through uint8_t pointers, which may alias anything, so
    // state behind references would be stored and reloaded at every sample.
    double mean = acc.mean, m2 = acc.m2, lo = acc.lo, hi = acc.hi;
    uint64_t n = acc.n;
    const bool hasNd = f.hasNodata;
    const double nd = f.nodata;
    const bool ndNan = hasNd && std::isnan(nd);
    const float ndF = static_cast<float>(nd);
    for (uint32_t x = 0; x < count; ++x, p += step) {
        if (mask != nullptr && mask[x] == 0) continue;
        if (alpha != nullptr && AlphaZero<T>(alpha + static_cast<size_t>(x) * step)) continue;
        const T t = LoadT<T>(p);
        const double v = static_cast<double>(t);
        if constexpr (std::is_floating_point_v<T>) {
            if (!std::isfinite(v)) continue;
        }
        if (hasNd) {
            if constexpr (std::is_same_v<T, float>) {
                if (ndNan ? std::isnan(t) : t == ndF) continue;
            } else {
                if (ndNan ? std::isnan(v) : v == nd) continue;
            }
        }
        n++;
        lo = std::min(lo, v);
        hi = std::max(hi, v);
        const double delta = v - mean;
        mean += delta / static_cast<double>(n);
        m2 += delta * (v - mean);
    }
    acc.mean = mean;
    acc.m2 = m2;
    acc.lo = lo;
    acc.hi = hi;
    acc.n = n;
}

// --- exact sums -----------------------------------------------------------------------------------

// The NoData value as a sample of integer type T: a sample equals it (as doubles) only when it is
// an integer in T's range.
template <typename T>
bool IntegerNodata(const SampleFilter& f, T& out) {
    if (!f.hasNodata || std::isnan(f.nodata)) return false;
    const double nd = f.nodata;
    if (nd < static_cast<double>(std::numeric_limits<T>::min()) || nd > static_cast<double>(std::numeric_limits<T>::max()))
        return false;
    const T t = static_cast<T>(nd);
    if (static_cast<double>(t) != nd) return false;
    out = t;
    return true;
}

// Integer samples: exact integer sums over sub-runs (no overflow), added to the double-double sums
// exactly. 8 / 16-bit: samples and squares directly; 32-bit: the square of |v| = h * 2^16 + l
// as h^2 * 2^32 + 2hl * 2^16 + l^2.
template <typename T>
void ExactIntRun(ExactSums& acc, const SampleFilter& f, const uint8_t* p, size_t step, uint32_t count,
                 const uint8_t* mask, const uint8_t* alpha) {
    T ndT{};
    const bool ndHit = IntegerNodata<T>(f, ndT);
    constexpr uint32_t kSubRun = uint32_t{1} << 24;
    for (uint32_t base = 0; base < count; base += kSubRun) {
        const uint32_t end = std::min(count, base + kSubRun);
        int64_t s = 0;
        uint64_t q0 = 0, q1 = 0, q2 = 0, cnt = 0;
        T mn = std::numeric_limits<T>::max(), mx = std::numeric_limits<T>::lowest();
        const uint8_t* q = p + static_cast<size_t>(base) * step;
        for (uint32_t x = base; x < end; ++x, q += step) {
            const T t = LoadT<T>(q);
            bool valid = !(ndHit && t == ndT);
            if (mask != nullptr) valid = valid && mask[x] != 0;
            if (alpha != nullptr) valid = valid && !AlphaZero<T>(alpha + static_cast<size_t>(x) * step);
            const int64_t v = valid ? static_cast<int64_t>(t) : 0;
            s += v;
            cnt += valid ? 1u : 0u;
            if constexpr (sizeof(T) <= 2) {
                q0 += static_cast<uint64_t>(v * v);
            } else {
                const uint64_t a = static_cast<uint64_t>(v < 0 ? -v : v);
                const uint64_t h = a >> 16, l = a & 0xffffu;
                q2 += h * h;
                q1 += 2 * h * l;
                q0 += l * l;
            }
            if (valid) {
                mn = std::min(mn, t);
                mx = std::max(mx, t);
            }
        }
        if (cnt == 0) continue;
        acc.n += cnt;
        acc.lo = std::min(acc.lo, static_cast<double>(mn));
        acc.hi = std::max(acc.hi, static_cast<double>(mx));
        AddInt(acc.sum, s);
        AddScaled(acc.sumSq, q0, 0);
        if constexpr (sizeof(T) > 2) {
            AddScaled(acc.sumSq, q1, 16);
            AddScaled(acc.sumSq, q2, 32);
        }
    }
}

// Floating-point samples: double-double sums, four lanes (sample index mod 4) merged in order at
// the end of the run.
template <typename T>
void ExactFloatRun(ExactSums& acc, const SampleFilter& f, const uint8_t* p, size_t step, uint32_t count,
                   const uint8_t* mask, const uint8_t* alpha) {
    const bool hasNd = f.hasNodata;
    const double nd = f.nodata;
    const bool ndNan = hasNd && std::isnan(nd);
    const float ndF = static_cast<float>(nd);
    DoubleDouble sum[4], sq[4];
    double lo = acc.lo, hi = acc.hi;
    uint64_t n = 0;
    for (uint32_t x = 0; x < count; ++x, p += step) {
        if (mask != nullptr && mask[x] == 0) continue;
        if (alpha != nullptr && AlphaZero<T>(alpha + static_cast<size_t>(x) * step)) continue;
        const T t = LoadT<T>(p);
        const double v = static_cast<double>(t);
        if (!std::isfinite(v)) continue;
        if (hasNd) {
            if constexpr (std::is_same_v<T, float>) {
                if (ndNan ? std::isnan(t) : t == ndF) continue;
            } else {
                if (ndNan ? std::isnan(v) : v == nd) continue;
            }
        }
        n++;
        lo = std::min(lo, v);
        hi = std::max(hi, v);
        const unsigned lane = x & 3u;
        AddD(sum[lane], v);
        if constexpr (std::is_same_v<T, float>) {
            AddD(sq[lane], v * v);  // exact: 24-bit significands
        } else {
            AddDD(sq[lane], TwoProd(v, v));
        }
    }
    if (n == 0) return;
    acc.n += n;
    acc.lo = lo;
    acc.hi = hi;
    for (int k = 0; k < 4; ++k) {
        AddDD(acc.sum, sum[k]);
        AddDD(acc.sumSq, sq[k]);
    }
}

}  // namespace

void AddRun(Welford& a, const SampleFilter& f, const uint8_t* p, size_t step, uint32_t count, const uint8_t* mask,
            const uint8_t* alpha) {
    switch (f.type) {
        case tiff::SampleType::kU8: WelfordRun<uint8_t>(a, f, p, step, count, mask, alpha); break;
        case tiff::SampleType::kI8: WelfordRun<int8_t>(a, f, p, step, count, mask, alpha); break;
        case tiff::SampleType::kU16: WelfordRun<uint16_t>(a, f, p, step, count, mask, alpha); break;
        case tiff::SampleType::kI16: WelfordRun<int16_t>(a, f, p, step, count, mask, alpha); break;
        case tiff::SampleType::kU32: WelfordRun<uint32_t>(a, f, p, step, count, mask, alpha); break;
        case tiff::SampleType::kI32: WelfordRun<int32_t>(a, f, p, step, count, mask, alpha); break;
        case tiff::SampleType::kF32: WelfordRun<float>(a, f, p, step, count, mask, alpha); break;
        case tiff::SampleType::kF64: WelfordRun<double>(a, f, p, step, count, mask, alpha); break;
        default: break;
    }
}

void AddRun(ExactSums& a, const SampleFilter& f, const uint8_t* p, size_t step, uint32_t count, const uint8_t* mask,
            const uint8_t* alpha) {
    switch (f.type) {
        case tiff::SampleType::kU8: ExactIntRun<uint8_t>(a, f, p, step, count, mask, alpha); break;
        case tiff::SampleType::kI8: ExactIntRun<int8_t>(a, f, p, step, count, mask, alpha); break;
        case tiff::SampleType::kU16: ExactIntRun<uint16_t>(a, f, p, step, count, mask, alpha); break;
        case tiff::SampleType::kI16: ExactIntRun<int16_t>(a, f, p, step, count, mask, alpha); break;
        case tiff::SampleType::kU32: ExactIntRun<uint32_t>(a, f, p, step, count, mask, alpha); break;
        case tiff::SampleType::kI32: ExactIntRun<int32_t>(a, f, p, step, count, mask, alpha); break;
        case tiff::SampleType::kF32: ExactFloatRun<float>(a, f, p, step, count, mask, alpha); break;
        case tiff::SampleType::kF64: ExactFloatRun<double>(a, f, p, step, count, mask, alpha); break;
        default: break;
    }
}

void Merge(ExactSums& into, const ExactSums& from) {
    if (from.n == 0) return;
    into.n += from.n;
    into.lo = std::min(into.lo, from.lo);
    into.hi = std::max(into.hi, from.hi);
    AddDD(into.sum, from.sum);
    AddDD(into.sumSq, from.sumSq);
}

std::optional<StatsResult> Finish(const Welford& a) {
    if (a.n == 0) return std::nullopt;
    StatsResult r;
    r.min = a.lo;
    r.max = a.hi;
    r.mean = a.mean;
    r.stdDev = std::sqrt(a.m2 / static_cast<double>(a.n));
    r.valid = a.n;
    return r;
}

std::optional<StatsResult> Finish(const ExactSums& a) {
    if (a.n == 0) return std::nullopt;
    const double n = static_cast<double>(a.n);
    // mean = sum / n in double-double: q1, then the remainder sum - q1 * n divided again.
    const double q1 = a.sum.hi / n;
    const DoubleDouble p = TwoProd(q1, n);
    DoubleDouble r = a.sum;
    AddDD(r, {-p.hi, -p.lo});
    const double q2 = r.hi / n;
    DoubleDouble mean;
    QuickTwoSum(q1, q2, mean.hi, mean.lo);
    // M2 = sumSq - sum * mean.
    DoubleDouble sm = TwoProd(a.sum.hi, mean.hi);
    sm.lo += a.sum.hi * mean.lo + a.sum.lo * mean.hi;
    QuickTwoSum(sm.hi, sm.lo, sm.hi, sm.lo);
    DoubleDouble m2 = a.sumSq;
    AddDD(m2, {-sm.hi, -sm.lo});
    const double m2d = std::max(0.0, m2.hi + m2.lo);
    StatsResult s;
    s.min = a.lo;
    s.max = a.hi;
    s.mean = mean.hi;
    s.stdDev = std::sqrt(m2d / n);
    s.valid = a.n;
    return s;
}

}  // namespace ddbt::cog::stats
