// SPDX-License-Identifier: MPL-2.0
//
// Highway kernels of the COG tiler. Compiled once per enabled target (foreach_target.h
// re-includes this file); HWY_DYNAMIC_DISPATCH picks the best target at runtime. The file is
// compiled with -ffp-contract=off -fno-lto (sources.cmake): the stretch and the nodata test must
// round each operation as the scalar references do.

#include "cog/tiler/kernels.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "cog/tiler/raster_io.h"
#include "cog/tiler/stretch.h"

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "cog/tiler/kernels.cpp"
#include "hwy/foreach_target.h"  // IWYU pragma: keep, must come before highway.h
#include "hwy/highway.h"

#ifndef DDBT_COG_TILER_KERNELS_ONCE
#define DDBT_COG_TILER_KERNELS_ONCE

namespace ddbt::cog::tiler {

/// vis_render.h stretchToByte: v clipped to [lo, hi], 255 * (c - lo) / (hi - lo) truncated. The
/// clip is std::max(lo, std::min(hi, v)): a NaN v gives hi (255).
uint8_t StretchToByte(double v, double lo, double hi) {
    const double c = std::max(lo, std::min(hi, v));
    return static_cast<uint8_t>(255.0 * (c - lo) / (hi - lo));
}

namespace kernels {

namespace {

inline float LoadF32(const uint8_t* p) noexcept {
    float v;
    std::memcpy(&v, p, sizeof v);
    return v;
}

inline double LoadF64(const uint8_t* p) noexcept {
    double v;
    std::memcpy(&v, p, sizeof v);
    return v;
}

}  // namespace

void NodataMaskF32Scalar(const uint8_t* samples, size_t n, float nodata, uint8_t* out) noexcept {
    for (size_t i = 0; i < n; ++i) out[i] = AreRealEqual(LoadF32(samples + i * 4), nodata) ? 0 : 255;
}

void FloatToByteScalar(const uint8_t* samples, size_t n, uint8_t* out) noexcept {
    for (size_t i = 0; i < n; ++i) out[i] = SampleToByte(PixelType::kF32, samples + i * 4);
}

bool StretchVectorizable(double lo, double hi) noexcept { return hi > lo && std::isfinite(255.0 * (hi - lo)); }

void InterleavePlanesScalar(const uint8_t* const* planes, size_t channels, size_t n, uint8_t* dst) noexcept {
    for (size_t i = 0; i < n; ++i)
        for (size_t c = 0; c < channels; ++c) dst[i * channels + c] = planes[c][i];
}

void ExtractSamplesScalar(const uint8_t* pixels, size_t sampleBytes, size_t samplesPerPixel, size_t n,
                          const int* idx, size_t count, uint8_t* const* planes) noexcept {
    const size_t pixelBytes = sampleBytes * samplesPerPixel;
    for (size_t b = 0; b < count; ++b) {
        const uint8_t* s = pixels + static_cast<size_t>(idx[b]) * sampleBytes;
        uint8_t* d = planes[b];
        for (size_t i = 0; i < n; ++i) std::memcpy(d + i * sampleBytes, s + i * pixelBytes, sampleBytes);
    }
}

void PickSamples32Scalar(const uint8_t* row, const uint32_t* xIn, size_t n, uint8_t* dst) noexcept {
    for (size_t i = 0; i < n; ++i) std::memcpy(dst + i * 4, row + static_cast<size_t>(xIn[i]) * 4, 4);
}

}  // namespace kernels
}  // namespace ddbt::cog::tiler

#endif  // DDBT_COG_TILER_KERNELS_ONCE

HWY_BEFORE_NAMESPACE();
namespace ddbt::cog::tiler::kernels {
namespace HWY_NAMESPACE {

namespace hn = hwy::HWY_NAMESPACE;

void NodataMaskF32Impl(const uint8_t* samples, size_t n, float nodata, uint8_t* out) {
    const hn::ScalableTag<float> df;
    const hn::RebindToSigned<decltype(df)> di;
    const hn::Rebind<uint8_t, decltype(df)> d8;
    const size_t N = hn::Lanes(df);
    const auto nd = hn::Set(df, nodata);
    const auto eps = hn::Set(df, std::numeric_limits<float>::epsilon());
    const auto two = hn::Set(df, 2.0f);
    const auto opaque = hn::Set(di, 255);
    const float* in = reinterpret_cast<const float*>(samples);
    size_t i = 0;
    for (; i + N <= n; i += N) {
        const auto v = hn::LoadU(df, in + i);
        // a == b || |a - b| < eps * |a + b| * 2, multiplied left to right as the scalar test.
        const auto near = hn::Lt(hn::Abs(hn::Sub(v, nd)), hn::Mul(hn::Mul(eps, hn::Abs(hn::Add(v, nd))), two));
        const auto isNodata = hn::Or(hn::Eq(v, nd), near);
        hn::StoreU(hn::DemoteTo(d8, hn::IfThenZeroElse(hn::RebindMask(di, isNodata), opaque)), d8, out + i);
    }
    if (i < n) NodataMaskF32Scalar(samples + i * 4, n - i, nodata, out + i);
}

void FloatToByteImpl(const uint8_t* samples, size_t n, uint8_t* out) {
    const hn::ScalableTag<double> dd;
    const hn::Rebind<float, decltype(dd)> df;
    const hn::Rebind<int32_t, decltype(dd)> di;
    const hn::Rebind<uint8_t, decltype(dd)> d8;
    const size_t N = hn::Lanes(dd);
    const auto zero = hn::Zero(dd);
    const auto top = hn::Set(dd, 255.0);
    const auto half = hn::Set(dd, 0.5);
    const float* in = reinterpret_cast<const float*>(samples);
    size_t i = 0;
    for (; i + N <= n; i += N) {
        const auto v = hn::PromoteTo(dd, hn::LoadU(df, in + i));
        // NaN and v <= 0 give 0 (Gt is false for NaN), v >= 255 gives 255; in between the sum
        // is exact (a float below 255 plus 0.5 fits a double) and truncates.
        const auto inside = hn::And(hn::Gt(v, zero), hn::Lt(v, top));
        auto r = hn::IfThenElse(hn::Ge(v, top), top, hn::IfThenElseZero(inside, hn::Add(v, half)));
        hn::StoreU(hn::DemoteTo(d8, hn::DemoteTo(di, r)), d8, out + i);
    }
    if (i < n) FloatToByteScalar(samples + i * 4, n - i, out + i);
}

// stretchToByte of N doubles: std::min(hi, v) is (v < hi ? v : hi) and std::max(lo, m) is
// (lo < m ? m : lo), so a NaN v clips to hi; then 255 * (c - lo) / (hi - lo), truncated (in
// [0, 256) under StretchVectorizable).
template <class DD, class V>
HWY_INLINE void StretchStore(DD dd, V v, V lo, V hi, V range, V scale, uint8_t* out) {
    const hn::Rebind<int32_t, DD> di;
    const hn::Rebind<uint8_t, DD> d8;
    const auto m = hn::IfThenElse(hn::Lt(v, hi), v, hi);
    const auto c = hn::IfThenElse(hn::Lt(lo, m), m, lo);
    const auto r = hn::Div(hn::Mul(scale, hn::Sub(c, lo)), range);
    hn::StoreU(hn::DemoteTo(d8, hn::DemoteTo(di, r)), d8, out);
    (void)dd;
}

void StretchF32Impl(const uint8_t* samples, size_t n, double lo, double hi, uint8_t* out) {
    const hn::ScalableTag<double> dd;
    const hn::Rebind<float, decltype(dd)> df;
    const size_t N = hn::Lanes(dd);
    const auto vlo = hn::Set(dd, lo), vhi = hn::Set(dd, hi), range = hn::Set(dd, hi - lo), scale = hn::Set(dd, 255.0);
    const float* in = reinterpret_cast<const float*>(samples);
    size_t i = 0;
    for (; i + N <= n; i += N)
        StretchStore(dd, hn::PromoteTo(dd, hn::LoadU(df, in + i)), vlo, vhi, range, scale, out + i);
    for (; i < n; ++i) out[i] = StretchToByte(static_cast<double>(LoadF32(samples + i * 4)), lo, hi);
}

void StretchF64Impl(const uint8_t* samples, size_t n, double lo, double hi, uint8_t* out) {
    const hn::ScalableTag<double> dd;
    const size_t N = hn::Lanes(dd);
    const auto vlo = hn::Set(dd, lo), vhi = hn::Set(dd, hi), range = hn::Set(dd, hi - lo), scale = hn::Set(dd, 255.0);
    const double* in = reinterpret_cast<const double*>(samples);
    size_t i = 0;
    for (; i + N <= n; i += N) StretchStore(dd, hn::LoadU(dd, in + i), vlo, vhi, range, scale, out + i);
    for (; i < n; ++i) out[i] = StretchToByte(LoadF64(samples + i * 8), lo, hi);
}

void InterleavePlanesImpl(const uint8_t* const* planes, size_t channels, size_t n, uint8_t* dst) {
    const hn::ScalableTag<uint8_t> d;
    const size_t N = hn::Lanes(d);
    size_t i = 0;
    switch (channels) {
        case 1: std::memcpy(dst, planes[0], n); return;
        case 2:
            for (; i + N <= n; i += N)
                hn::StoreInterleaved2(hn::LoadU(d, planes[0] + i), hn::LoadU(d, planes[1] + i), d, dst + i * 2);
            break;
        case 3:
            for (; i + N <= n; i += N)
                hn::StoreInterleaved3(hn::LoadU(d, planes[0] + i), hn::LoadU(d, planes[1] + i),
                                      hn::LoadU(d, planes[2] + i), d, dst + i * 3);
            break;
        case 4:
            for (; i + N <= n; i += N)
                hn::StoreInterleaved4(hn::LoadU(d, planes[0] + i), hn::LoadU(d, planes[1] + i),
                                      hn::LoadU(d, planes[2] + i), hn::LoadU(d, planes[3] + i), d, dst + i * 4);
            break;
        default: break;
    }
    if (i < n) {
        const uint8_t* rest[4];
        for (size_t c = 0; c < channels; ++c) rest[c] = planes[c] + i;
        InterleavePlanesScalar(rest, channels, n - i, dst + i * channels);
    }
}

// ExtractSamples for samples of type T (the bits only: floats move as unsigned words).
template <typename T>
HWY_INLINE void ExtractTyped(const uint8_t* pixels, size_t samplesPerPixel, size_t n, const int* idx, size_t count,
                             uint8_t* const* planes) {
    const hn::ScalableTag<T> d;
    const size_t N = hn::Lanes(d);
    size_t i = 0;
    if (samplesPerPixel >= 2 && samplesPerPixel <= 4) {
        hn::Vec<decltype(d)> v[4];
        for (; i + N <= n; i += N) {
            const T* p = reinterpret_cast<const T*>(pixels) + i * samplesPerPixel;
            if (samplesPerPixel == 2) hn::LoadInterleaved2(d, p, v[0], v[1]);
            else if (samplesPerPixel == 3) hn::LoadInterleaved3(d, p, v[0], v[1], v[2]);
            else hn::LoadInterleaved4(d, p, v[0], v[1], v[2], v[3]);
            for (size_t b = 0; b < count; ++b) hn::StoreU(v[idx[b]], d, reinterpret_cast<T*>(planes[b]) + i);
        }
    }
    for (size_t b = 0; b < count && i < n; ++b) {
        uint8_t* p = planes[b] + i * sizeof(T);
        ExtractSamplesScalar(pixels + i * samplesPerPixel * sizeof(T), sizeof(T), samplesPerPixel, n - i, idx + b, 1,
                             &p);
    }
}

void ExtractSamplesImpl(const uint8_t* pixels, size_t sampleBytes, size_t samplesPerPixel, size_t n, const int* idx,
                        size_t count, uint8_t* const* planes) {
    if (samplesPerPixel == 1) {
        for (size_t b = 0; b < count; ++b) std::memcpy(planes[b], pixels, n * sampleBytes);
        return;
    }
    switch (sampleBytes) {
        case 1: return ExtractTyped<uint8_t>(pixels, samplesPerPixel, n, idx, count, planes);
        case 2: return ExtractTyped<uint16_t>(pixels, samplesPerPixel, n, idx, count, planes);
        case 4: return ExtractTyped<uint32_t>(pixels, samplesPerPixel, n, idx, count, planes);
        default: return ExtractTyped<uint64_t>(pixels, samplesPerPixel, n, idx, count, planes);
    }
}

void PickSamples32Impl(const uint8_t* row, size_t rowWords, const uint32_t* xIn, size_t n, uint8_t* dst) {
    const hn::ScalableTag<uint32_t> du;
    const hn::RebindToSigned<decltype(du)> di;
    const size_t N = hn::Lanes(du);
    const uint32_t* words = reinterpret_cast<const uint32_t*>(row);
    uint32_t* out = reinterpret_cast<uint32_t*>(dst);
    size_t i = 0;
    for (; i + N <= n; i += N) {
        const uint32_t first = xIn[i];
        if (xIn[i + N - 1] - first >= 2 * N || first + 2 * N > rowWords) {
            PickSamples32Scalar(row, xIn + i, N, dst + i * 4);
            continue;
        }
        const auto offsets = hn::Sub(hn::LoadU(du, xIn + i), hn::Set(du, first));
        const auto lo = hn::LoadU(du, words + first);
        const auto hi = hn::LoadU(du, words + first + N);
        hn::StoreU(hn::TwoTablesLookupLanes(du, lo, hi, hn::IndicesFromVec(du, hn::BitCast(di, offsets))), du,
                   out + i);
    }
    if (i < n) PickSamples32Scalar(row, xIn + i, n - i, dst + i * 4);
}

bool AllBytesEqualImpl(const uint8_t* p, size_t n, uint8_t value) {
    const hn::ScalableTag<uint8_t> d;
    const size_t N = hn::Lanes(d);
    const auto v = hn::Set(d, value);
    size_t i = 0;
    for (; i + 4 * N <= n; i += 4 * N) {
        const auto a = hn::And(hn::Eq(hn::LoadU(d, p + i), v), hn::Eq(hn::LoadU(d, p + i + N), v));
        const auto b = hn::And(hn::Eq(hn::LoadU(d, p + i + 2 * N), v), hn::Eq(hn::LoadU(d, p + i + 3 * N), v));
        if (!hn::AllTrue(d, hn::And(a, b))) return false;
    }
    for (; i + N <= n; i += N)
        if (!hn::AllTrue(d, hn::Eq(hn::LoadU(d, p + i), v))) return false;
    for (; i < n; ++i)
        if (p[i] != value) return false;
    return true;
}

void SourcePixelsImpl(const SourcePixelsJob& job) {
    for (size_t k = 0; k < job.n; ++k) SourcePixel(job, k);
}

}  // namespace HWY_NAMESPACE
}  // namespace ddbt::cog::tiler::kernels
HWY_AFTER_NAMESPACE();

#if HWY_ONCE

namespace ddbt::cog::tiler::kernels {

HWY_EXPORT(NodataMaskF32Impl);
HWY_EXPORT(FloatToByteImpl);
HWY_EXPORT(StretchF32Impl);
HWY_EXPORT(StretchF64Impl);
HWY_EXPORT(InterleavePlanesImpl);
HWY_EXPORT(ExtractSamplesImpl);
HWY_EXPORT(PickSamples32Impl);
HWY_EXPORT(AllBytesEqualImpl);
HWY_EXPORT(SourcePixelsImpl);

void NodataMaskF32(const uint8_t* samples, size_t n, float nodata, uint8_t* out) {
    HWY_DYNAMIC_DISPATCH(NodataMaskF32Impl)(samples, n, nodata, out);
}

void FloatToByte(const uint8_t* samples, size_t n, uint8_t* out) {
    HWY_DYNAMIC_DISPATCH(FloatToByteImpl)(samples, n, out);
}

void StretchF32(const uint8_t* samples, size_t n, double lo, double hi, uint8_t* out) {
    HWY_DYNAMIC_DISPATCH(StretchF32Impl)(samples, n, lo, hi, out);
}

void StretchF64(const uint8_t* samples, size_t n, double lo, double hi, uint8_t* out) {
    HWY_DYNAMIC_DISPATCH(StretchF64Impl)(samples, n, lo, hi, out);
}

void InterleavePlanes(const uint8_t* const* planes, size_t channels, size_t n, uint8_t* dst) {
    HWY_DYNAMIC_DISPATCH(InterleavePlanesImpl)(planes, channels, n, dst);
}

void ExtractSamples(const uint8_t* pixels, size_t sampleBytes, size_t samplesPerPixel, size_t n, const int* idx,
                    size_t count, uint8_t* const* planes) {
    HWY_DYNAMIC_DISPATCH(ExtractSamplesImpl)(pixels, sampleBytes, samplesPerPixel, n, idx, count, planes);
}

void PickSamples32(const uint8_t* row, size_t rowWords, const uint32_t* xIn, size_t n, uint8_t* dst) {
    HWY_DYNAMIC_DISPATCH(PickSamples32Impl)(row, rowWords, xIn, n, dst);
}

bool AllBytesEqual(const uint8_t* p, size_t n, uint8_t value) {
    return HWY_DYNAMIC_DISPATCH(AllBytesEqualImpl)(p, n, value);
}

void SourcePixels(const SourcePixelsJob& job) { HWY_DYNAMIC_DISPATCH(SourcePixelsImpl)(job); }

}  // namespace ddbt::cog::tiler::kernels

#endif  // HWY_ONCE
