// SPDX-License-Identifier: MPL-2.0
//
// TIFF predictors and byte swapping. Compiled once per enabled Highway target
// (foreach_target.h re-includes this file); HWY_DYNAMIC_DISPATCH picks the best target.
//
// Horizontal differencing is a prefix sum along the row with stride spp (one independent
// chain per sample of a pixel). Within a vector it is computed in log2(N / spp) steps
// (v += v slid up by spp, 2 spp, 4 spp, ...); the carry from the previous vector is its last
// value of the same sample, i.e. lane N - spp + (i mod spp) broadcast to lane i (one table
// lookup), which holds for any N and spp because N - spp + (i mod spp) == i (mod spp) after a
// shift of N. When spp >= N the lanes of a vector do not depend on each other at all.

#include "cog/codecs/predictor.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "core/util/endian.h"

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "cog/codecs/predictor.cpp"
#include "hwy/foreach_target.h"  // IWYU pragma: keep, must come before highway.h
#include "hwy/highway.h"

#if !HWY_IS_LITTLE_ENDIAN
#error "The predictor kernels assume a little-endian host"
#endif

HWY_BEFORE_NAMESPACE();
namespace ddbt::cog::codecs {
namespace HWY_NAMESPACE {

namespace hn = hwy::HWY_NAMESPACE;

template <typename T>
HWY_INLINE void AccumulateRow(T* HWY_RESTRICT p, size_t n, size_t stride) {
    const hn::ScalableTag<T> d;
    const size_t N = hn::Lanes(d);
    if (stride == 0 || n <= stride) return;
    size_t i = 0;
    if (stride >= N) {
        for (i = stride; i + N <= n; i += N) {
            hn::StoreU(hn::Add(hn::LoadU(d, p + i), hn::LoadU(d, p + i - stride)), d, p + i);
        }
        for (; i < n; ++i) p[i] = static_cast<T>(p[i] + p[i - stride]);
        return;
    }
    using TI = hwy::MakeSigned<T>;
    HWY_ALIGN TI idx[hn::MaxLanes(d)];
    for (size_t k = 0; k < N; ++k) idx[k] = static_cast<TI>(N - stride + k % stride);
    const auto table = hn::SetTableIndices(d, idx);
    auto carry = hn::Zero(d);
    for (; i + N <= n; i += N) {
        auto v = hn::LoadU(d, p + i);
        for (size_t s = stride; s < N; s <<= 1) v = hn::Add(v, hn::SlideUpLanes(d, v, s));
        v = hn::Add(v, hn::TableLookupLanes(carry, table));
        hn::StoreU(v, d, p + i);
        carry = v;
    }
    for (; i < n; ++i) {
        if (i >= stride) p[i] = static_cast<T>(p[i] + p[i - stride]);
    }
}

void Accumulate8(uint8_t* p, size_t n, size_t stride) { AccumulateRow(p, n, stride); }
void Accumulate16(uint8_t* p, size_t n, size_t stride) { AccumulateRow(reinterpret_cast<uint16_t*>(p), n, stride); }
void Accumulate32(uint8_t* p, size_t n, size_t stride) { AccumulateRow(reinterpret_cast<uint32_t*>(p), n, stride); }
void Accumulate64(uint8_t* p, size_t n, size_t stride) { AccumulateRow(reinterpret_cast<uint64_t*>(p), n, stride); }

template <typename T>
HWY_INLINE void SwapAll(uint8_t* bytes, size_t count) {
    const hn::ScalableTag<T> d;
    const size_t N = hn::Lanes(d);
    T* p = reinterpret_cast<T*>(bytes);
    size_t i = 0;
    for (; i + N <= count; i += N) hn::StoreU(hn::ReverseLaneBytes(hn::LoadU(d, p + i)), d, p + i);
    for (; i < count; ++i) p[i] = ByteSwap(p[i]);
}

void Swap16(uint8_t* p, size_t count) { SwapAll<uint16_t>(p, count); }
void Swap32(uint8_t* p, size_t count) { SwapAll<uint32_t>(p, count); }
void Swap64(uint8_t* p, size_t count) { SwapAll<uint64_t>(p, count); }

// Byte planes (most significant first, `wc` bytes each) -> little-endian samples.
void Unshuffle2(const uint8_t* HWY_RESTRICT planes, size_t wc, uint8_t* HWY_RESTRICT out) {
    const hn::ScalableTag<uint8_t> d;
    const size_t N = hn::Lanes(d);
    const uint8_t* p0 = planes;
    const uint8_t* p1 = planes + wc;
    size_t i = 0;
    for (; i + N <= wc; i += N) hn::StoreInterleaved2(hn::LoadU(d, p1 + i), hn::LoadU(d, p0 + i), d, out + 2 * i);
    for (; i < wc; ++i) {
        out[2 * i] = p1[i];
        out[2 * i + 1] = p0[i];
    }
}

void Unshuffle4(const uint8_t* HWY_RESTRICT planes, size_t wc, uint8_t* HWY_RESTRICT out) {
    const hn::ScalableTag<uint8_t> d;
    const size_t N = hn::Lanes(d);
    const uint8_t* p0 = planes;
    const uint8_t* p1 = planes + wc;
    const uint8_t* p2 = planes + 2 * wc;
    const uint8_t* p3 = planes + 3 * wc;
    size_t i = 0;
    for (; i + N <= wc; i += N) {
        hn::StoreInterleaved4(hn::LoadU(d, p3 + i), hn::LoadU(d, p2 + i), hn::LoadU(d, p1 + i), hn::LoadU(d, p0 + i), d,
                              out + 4 * i);
    }
    for (; i < wc; ++i) {
        out[4 * i] = p3[i];
        out[4 * i + 1] = p2[i];
        out[4 * i + 2] = p1[i];
        out[4 * i + 3] = p0[i];
    }
}

void Unshuffle8(const uint8_t* HWY_RESTRICT planes, size_t wc, uint8_t* HWY_RESTRICT out) {
    // f64 with predictor 3 is rare: a plain loop (compiled per target, the compiler may
    // vectorize it).
    for (size_t i = 0; i < wc; ++i) {
        uint64_t v = 0;
        for (size_t b = 0; b < 8; ++b) v |= uint64_t{planes[b * wc + i]} << (8 * (7 - b));
        std::memcpy(out + 8 * i, &v, 8);
    }
}

}  // namespace HWY_NAMESPACE
}  // namespace ddbt::cog::codecs
HWY_AFTER_NAMESPACE();

#if HWY_ONCE

namespace ddbt::cog::codecs {

HWY_EXPORT(Accumulate8);
HWY_EXPORT(Accumulate16);
HWY_EXPORT(Accumulate32);
HWY_EXPORT(Accumulate64);
HWY_EXPORT(Swap16);
HWY_EXPORT(Swap32);
HWY_EXPORT(Swap64);
HWY_EXPORT(Unshuffle2);
HWY_EXPORT(Unshuffle4);
HWY_EXPORT(Unshuffle8);

namespace {
thread_local std::vector<uint8_t> tlsPlanes;
}

void UndoHorizontalPredictor(uint8_t* data, size_t rows, size_t rowStride, size_t width, size_t spp,
                             size_t bytesPerSample) {
    const size_t n = width * spp;
    for (size_t r = 0; r < rows; ++r) {
        uint8_t* row = data + r * rowStride;
        switch (bytesPerSample) {
            case 1: HWY_DYNAMIC_DISPATCH(Accumulate8)(row, n, spp); break;
            case 2: HWY_DYNAMIC_DISPATCH(Accumulate16)(row, n, spp); break;
            case 4: HWY_DYNAMIC_DISPATCH(Accumulate32)(row, n, spp); break;
            case 8: HWY_DYNAMIC_DISPATCH(Accumulate64)(row, n, spp); break;
            default: break;
        }
    }
}

void UndoFloatingPointPredictor(uint8_t* data, size_t rows, size_t rowStride, size_t width, size_t spp,
                                size_t bytesPerSample) {
    const size_t wc = width * spp;
    const size_t rowBytes = wc * bytesPerSample;
    std::vector<uint8_t>& planes = tlsPlanes;
    if (planes.size() < rowBytes) planes.resize(rowBytes);
    for (size_t r = 0; r < rows; ++r) {
        uint8_t* row = data + r * rowStride;
        HWY_DYNAMIC_DISPATCH(Accumulate8)(row, rowBytes, spp);
        std::memcpy(planes.data(), row, rowBytes);
        switch (bytesPerSample) {
            case 2: HWY_DYNAMIC_DISPATCH(Unshuffle2)(planes.data(), wc, row); break;
            case 4: HWY_DYNAMIC_DISPATCH(Unshuffle4)(planes.data(), wc, row); break;
            case 8: HWY_DYNAMIC_DISPATCH(Unshuffle8)(planes.data(), wc, row); break;
            default:
                // other widths (24-bit floats, 128-bit complex): the byte planes, most significant first
                for (size_t c = 0; c < wc; ++c)
                    for (size_t b = 0; b < bytesPerSample; ++b)
                        row[bytesPerSample * c + b] = planes[(bytesPerSample - b - 1) * wc + c];
                break;
        }
    }
}

void ByteSwapSamples(uint8_t* data, size_t count, size_t bytesPerSample) {
    switch (bytesPerSample) {
        case 2: HWY_DYNAMIC_DISPATCH(Swap16)(data, count); break;
        case 4: HWY_DYNAMIC_DISPATCH(Swap32)(data, count); break;
        case 8: HWY_DYNAMIC_DISPATCH(Swap64)(data, count); break;
        default: reference::ByteSwapSamples(data, count, bytesPerSample); break;  // 3 bytes (FP24)
    }
}

namespace reference {

namespace {
template <typename T>
void AccumulateScalar(uint8_t* row, size_t n, size_t stride) {
    for (size_t i = stride; i < n; ++i) {
        T a, b;
        std::memcpy(&a, row + i * sizeof(T), sizeof(T));
        std::memcpy(&b, row + (i - stride) * sizeof(T), sizeof(T));
        a = static_cast<T>(a + b);
        std::memcpy(row + i * sizeof(T), &a, sizeof(T));
    }
}
}  // namespace

void UndoHorizontalPredictor(uint8_t* data, size_t rows, size_t rowStride, size_t width, size_t spp,
                             size_t bytesPerSample) {
    const size_t n = width * spp;
    for (size_t r = 0; r < rows; ++r) {
        uint8_t* row = data + r * rowStride;
        switch (bytesPerSample) {
            case 1: AccumulateScalar<uint8_t>(row, n, spp); break;
            case 2: AccumulateScalar<uint16_t>(row, n, spp); break;
            case 4: AccumulateScalar<uint32_t>(row, n, spp); break;
            case 8: AccumulateScalar<uint64_t>(row, n, spp); break;
            default: break;
        }
    }
}

void UndoFloatingPointPredictor(uint8_t* data, size_t rows, size_t rowStride, size_t width, size_t spp,
                                size_t bytesPerSample) {
    // libtiff's fpAcc, little-endian host.
    const size_t wc = width * spp;
    const size_t count = wc * bytesPerSample;
    std::vector<uint8_t> tmp(count);
    for (size_t r = 0; r < rows; ++r) {
        uint8_t* cp = data + r * rowStride;
        for (size_t i = spp; i < count; ++i) cp[i] = static_cast<uint8_t>(cp[i] + cp[i - spp]);
        std::memcpy(tmp.data(), cp, count);
        for (size_t c = 0; c < wc; ++c) {
            for (size_t b = 0; b < bytesPerSample; ++b) cp[bytesPerSample * c + b] = tmp[(bytesPerSample - b - 1) * wc + c];
        }
    }
}

void ByteSwapSamples(uint8_t* data, size_t count, size_t bytesPerSample) {
    for (size_t i = 0; i < count; ++i) {
        uint8_t* p = data + i * bytesPerSample;
        for (size_t a = 0, b = bytesPerSample - 1; a < b; ++a, --b) {
            const uint8_t t = p[a];
            p[a] = p[b];
            p[b] = t;
        }
    }
}

}  // namespace reference

}  // namespace ddbt::cog::codecs

#endif  // HWY_ONCE
