// SPDX-License-Identifier: MPL-2.0
//
// Highway SIMD kernels of the tile encoders. Compiled once per enabled target (foreach_target.h
// re-includes this file); HWY_DYNAMIC_DISPATCH picks the best target at runtime.
//
// PNG filtering only reads the unfiltered rows (the previous row and the pixel to the left are
// raw bytes, never filtered output), so every filter vectorizes across the whole row; only
// the decoder has a serial dependency.

#include "core/encode/kernels.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "core/encode/kernels.cpp"
#include "hwy/foreach_target.h"  // IWYU pragma: keep, must come before highway.h
#include "hwy/highway.h"

#if !HWY_IS_LITTLE_ENDIAN
#error "The encode kernels read RGBA pixels as little-endian 32-bit words"
#endif

HWY_BEFORE_NAMESPACE();
namespace ddbt::encode::kernels {
namespace HWY_NAMESPACE {

namespace hn = hwy::HWY_NAMESPACE;
using D8 = hn::ScalableTag<uint8_t>;
using V8 = hn::Vec<D8>;
using D64 = hn::Repartition<uint64_t, D8>;
using V64 = hn::Vec<D64>;

// ------------------------------------------------------------------------------------------
// Scalar helpers (row heads and tails)

HWY_INLINE uint8_t PaethPredictor(int a, int b, int c) {
    const int p = a + b - c;
    const int pa = std::abs(p - a);
    const int pb = std::abs(p - b);
    const int pc = std::abs(p - c);
    if (pa <= pb && pa <= pc) return static_cast<uint8_t>(a);
    if (pb <= pc) return static_cast<uint8_t>(b);
    return static_cast<uint8_t>(c);
}

HWY_INLINE uint32_t ByteCost(uint8_t v) { return v < 128 ? v : 256u - v; }

HWY_INLINE uint8_t FilterByte(uint32_t filter, uint8_t x, uint8_t a, uint8_t b, uint8_t c) {
    switch (filter) {
        case kFilterSub: return static_cast<uint8_t>(x - a);
        case kFilterUp: return static_cast<uint8_t>(x - b);
        case kFilterAverage: return static_cast<uint8_t>(x - ((a + b) >> 1));
        case kFilterPaeth: return static_cast<uint8_t>(x - PaethPredictor(a, b, c));
        default: return x;
    }
}

// ------------------------------------------------------------------------------------------
// Vector filters

// floor((a + b) / 2) without widening: AverageRound rounds up, so subtract the odd bit.
HWY_INLINE V8 AverageFloor(D8 d, V8 a, V8 b) {
    return hn::Sub(hn::AverageRound(a, b), hn::And(hn::Xor(a, b), hn::Set(d, uint8_t{1})));
}

// Paeth predictor on bytes, without widening: pa = |b - c|, pb = |a - c| and
// pc = |(a - c) + (b - c)|, which is pa + pb when a - c and b - c have the same sign and
// |pa - pb| otherwise. The saturating add only clips values >= 255, which compare the same.
HWY_INLINE V8 Paeth(V8 a, V8 b, V8 c) {
    const V8 pa = hn::AbsDiff(b, c);
    const V8 pb = hn::AbsDiff(a, c);
    const auto sameSign = hn::Not(hn::Xor(hn::Ge(a, c), hn::Ge(b, c)));
    const V8 pc = hn::IfThenElse(sameSign, hn::SaturatedAdd(pa, pb), hn::AbsDiff(pa, pb));
    return hn::IfThenElse(hn::And(hn::Le(pa, pb), hn::Le(pa, pc)), a, hn::IfThenElse(hn::Le(pb, pc), b, c));
}

// min(v, 256 - v) of each byte, summed per 8 bytes.
HWY_INLINE V64 Cost(D8 d, V8 v) { return hn::SumsOf8(hn::Min(v, hn::Sub(hn::Zero(d), v))); }

// Bit i of Mask: filter type i is evaluated.
template <uint32_t Mask>
HWY_INLINE void FilterCostsT(const uint8_t* HWY_RESTRICT cur, const uint8_t* HWY_RESTRICT prev, size_t n,
                             size_t bpp, uint64_t* HWY_RESTRICT costs) {
    const D8 d;
    const D64 d64;
    const size_t N = hn::Lanes(d);
    uint64_t s[5] = {0, 0, 0, 0, 0};
    size_t i = 0;
    // Head: the first pixel has no left neighbour (a = c = 0).
    for (; i < bpp && i < n; ++i) {
        for (uint32_t f = 0; f < 5; ++f) {
            if (Mask & (1u << f)) s[f] += ByteCost(FilterByte(f, cur[i], 0, prev[i], 0));
        }
    }
    V64 acc0 = hn::Zero(d64), acc1 = hn::Zero(d64), acc2 = hn::Zero(d64), acc3 = hn::Zero(d64),
        acc4 = hn::Zero(d64);
    for (; i + N <= n; i += N) {
        const V8 x = hn::LoadU(d, cur + i);
        const V8 a = hn::LoadU(d, cur + i - bpp);
        const V8 b = hn::LoadU(d, prev + i);
        if constexpr ((Mask & 1u) != 0) acc0 = hn::Add(acc0, Cost(d, x));
        if constexpr ((Mask & 2u) != 0) acc1 = hn::Add(acc1, Cost(d, hn::Sub(x, a)));
        if constexpr ((Mask & 4u) != 0) acc2 = hn::Add(acc2, Cost(d, hn::Sub(x, b)));
        if constexpr ((Mask & 8u) != 0) acc3 = hn::Add(acc3, Cost(d, hn::Sub(x, AverageFloor(d, a, b))));
        if constexpr ((Mask & 16u) != 0) {
            const V8 c = hn::LoadU(d, prev + i - bpp);
            acc4 = hn::Add(acc4, Cost(d, hn::Sub(x, Paeth(a, b, c))));
        }
    }
    if constexpr ((Mask & 1u) != 0) s[0] += hn::ReduceSum(d64, acc0);
    if constexpr ((Mask & 2u) != 0) s[1] += hn::ReduceSum(d64, acc1);
    if constexpr ((Mask & 4u) != 0) s[2] += hn::ReduceSum(d64, acc2);
    if constexpr ((Mask & 8u) != 0) s[3] += hn::ReduceSum(d64, acc3);
    if constexpr ((Mask & 16u) != 0) s[4] += hn::ReduceSum(d64, acc4);
    for (; i < n; ++i) {
        for (uint32_t f = 0; f < 5; ++f) {
            if (Mask & (1u << f)) s[f] += ByteCost(FilterByte(f, cur[i], cur[i - bpp], prev[i], prev[i - bpp]));
        }
    }
    for (uint32_t f = 0; f < 5; ++f) costs[f] = (Mask & (1u << f)) ? s[f] : UINT64_MAX;
}

template <uint32_t F>
HWY_INLINE void FilterRowT(const uint8_t* HWY_RESTRICT cur, const uint8_t* HWY_RESTRICT prev, size_t n, size_t bpp,
                           uint8_t* HWY_RESTRICT out) {
    if constexpr (F == kFilterNone) {
        std::memcpy(out, cur, n);
        return;
    }
    const D8 d;
    const size_t N = hn::Lanes(d);
    size_t i = 0;
    for (; i < bpp && i < n; ++i) out[i] = FilterByte(F, cur[i], 0, prev[i], 0);
    for (; i + N <= n; i += N) {
        const V8 x = hn::LoadU(d, cur + i);
        V8 r;
        if constexpr (F == kFilterSub) {
            r = hn::Sub(x, hn::LoadU(d, cur + i - bpp));
        } else if constexpr (F == kFilterUp) {
            r = hn::Sub(x, hn::LoadU(d, prev + i));
        } else if constexpr (F == kFilterAverage) {
            r = hn::Sub(x, AverageFloor(d, hn::LoadU(d, cur + i - bpp), hn::LoadU(d, prev + i)));
        } else {
            r = hn::Sub(x, Paeth(hn::LoadU(d, cur + i - bpp), hn::LoadU(d, prev + i), hn::LoadU(d, prev + i - bpp)));
        }
        hn::StoreU(r, d, out + i);
    }
    for (; i < n; ++i) out[i] = FilterByte(F, cur[i], cur[i - bpp], prev[i], prev[i - bpp]);
}

HWY_INLINE void FilterRowAny(const uint8_t* cur, const uint8_t* prev, size_t n, size_t bpp, uint32_t filter,
                             uint8_t* out) {
    switch (filter) {
        case kFilterSub: FilterRowT<kFilterSub>(cur, prev, n, bpp, out); break;
        case kFilterUp: FilterRowT<kFilterUp>(cur, prev, n, bpp, out); break;
        case kFilterAverage: FilterRowT<kFilterAverage>(cur, prev, n, bpp, out); break;
        case kFilterPaeth: FilterRowT<kFilterPaeth>(cur, prev, n, bpp, out); break;
        default: FilterRowT<kFilterNone>(cur, prev, n, bpp, out); break;
    }
}

void PngFilterCostsImpl(const uint8_t* cur, const uint8_t* prev, size_t n, size_t bpp, uint64_t* costs) {
    FilterCostsT<31u>(cur, prev, n, bpp, costs);
}

void PngFilterRowImpl(const uint8_t* cur, const uint8_t* prev, size_t n, size_t bpp, uint32_t filter,
                      uint8_t* out) {
    FilterRowAny(cur, prev, n, bpp, filter, out);
}

// ------------------------------------------------------------------------------------------
// Pixel conversions

void DropAlpha4Impl(const uint8_t* HWY_RESTRICT rgba, size_t pixels, uint8_t* HWY_RESTRICT rgb) {
    const D8 d;
    const size_t N = hn::Lanes(d);
    size_t i = 0;
    for (; i + N <= pixels; i += N) {
        V8 r, g, b, a;
        hn::LoadInterleaved4(d, rgba + 4 * i, r, g, b, a);
        hn::StoreInterleaved3(r, g, b, d, rgb + 3 * i);
    }
    for (; i < pixels; ++i) {
        rgb[3 * i] = rgba[4 * i];
        rgb[3 * i + 1] = rgba[4 * i + 1];
        rgb[3 * i + 2] = rgba[4 * i + 2];
    }
}

void DropAlpha2Impl(const uint8_t* HWY_RESTRICT ga, size_t pixels, uint8_t* HWY_RESTRICT g) {
    const D8 d;
    const size_t N = hn::Lanes(d);
    size_t i = 0;
    for (; i + N <= pixels; i += N) {
        V8 grey, a;
        hn::LoadInterleaved2(d, ga + 2 * i, grey, a);
        hn::StoreU(grey, d, g + i);
    }
    for (; i < pixels; ++i) g[i] = ga[2 * i];
}

void WhiteUnderTransparent4Impl(const uint8_t* HWY_RESTRICT rgba, size_t pixels, uint8_t* HWY_RESTRICT rgbx) {
    const hn::ScalableTag<uint32_t> d32;
    const hn::Repartition<uint8_t, decltype(d32)> d8;
    const size_t N = hn::Lanes(d32);
    // Little-endian RGBA word: alpha is the top byte, so alpha == 0 <=> word < 2^24.
    const auto limit = hn::Set(d32, 0x01000000u);
    const auto white = hn::Set(d32, 0xFFFFFFFFu);
    size_t i = 0;
    for (; i + N <= pixels; i += N) {
        const auto px = hn::BitCast(d32, hn::LoadU(d8, rgba + 4 * i));
        hn::StoreU(hn::BitCast(d8, hn::IfThenElse(hn::Lt(px, limit), white, px)), d8, rgbx + 4 * i);
    }
    for (; i < pixels; ++i) {
        const bool clear = rgba[4 * i + 3] == 0;
        for (size_t c = 0; c < 4; ++c) rgbx[4 * i + c] = clear ? uint8_t{255} : rgba[4 * i + c];
    }
}

void WhiteUnderTransparent2Impl(const uint8_t* HWY_RESTRICT ga, size_t pixels, uint8_t* HWY_RESTRICT g) {
    const D8 d;
    const size_t N = hn::Lanes(d);
    const V8 white = hn::Set(d, uint8_t{255});
    size_t i = 0;
    for (; i + N <= pixels; i += N) {
        V8 grey, a;
        hn::LoadInterleaved2(d, ga + 2 * i, grey, a);
        hn::StoreU(hn::IfThenElse(hn::Eq(a, hn::Zero(d)), white, grey), d, g + i);
    }
    for (; i < pixels; ++i) g[i] = ga[2 * i + 1] == 0 ? uint8_t{255} : ga[2 * i];
}

void GreyToRgbImpl(const uint8_t* HWY_RESTRICT g, size_t pixels, uint8_t* HWY_RESTRICT rgb) {
    const D8 d;
    const size_t N = hn::Lanes(d);
    size_t i = 0;
    for (; i + N <= pixels; i += N) {
        const V8 v = hn::LoadU(d, g + i);
        hn::StoreInterleaved3(v, v, v, d, rgb + 3 * i);
    }
    for (; i < pixels; ++i) rgb[3 * i] = rgb[3 * i + 1] = rgb[3 * i + 2] = g[i];
}

void GreyAlphaToRgbaImpl(const uint8_t* HWY_RESTRICT ga, size_t pixels, uint8_t* HWY_RESTRICT rgba) {
    const D8 d;
    const size_t N = hn::Lanes(d);
    size_t i = 0;
    for (; i + N <= pixels; i += N) {
        V8 v, a;
        hn::LoadInterleaved2(d, ga + 2 * i, v, a);
        hn::StoreInterleaved4(v, v, v, a, d, rgba + 4 * i);
    }
    for (; i < pixels; ++i) {
        rgba[4 * i] = rgba[4 * i + 1] = rgba[4 * i + 2] = ga[2 * i];
        rgba[4 * i + 3] = ga[2 * i + 1];
    }
}

// ------------------------------------------------------------------------------------------
// Alpha range

void AlphaRangeImpl(const uint8_t* HWY_RESTRICT row, size_t pixels, uint32_t channels, uint8_t* minAlpha,
                    uint8_t* maxAlpha) {
    const D8 d;
    const size_t N = hn::Lanes(d);
    const size_t bytes = pixels * channels;
    // 0xFF on the alpha bytes (the last of each pixel; vectors hold whole pixels).
    const V8 alphaMask = channels == 4 ? hn::BitCast(d, hn::Set(hn::Repartition<uint32_t, D8>(), 0xFF000000u))
                                       : hn::BitCast(d, hn::Set(hn::Repartition<uint16_t, D8>(), uint16_t{0xFF00}));
    const V8 colorMask = hn::Not(alphaMask);
    V8 vmin = hn::Set(d, *minAlpha);
    V8 vmax = hn::Set(d, *maxAlpha);
    size_t i = 0;
    for (; i + 2 * N <= bytes; i += 2 * N) {
        const V8 v0 = hn::LoadU(d, row + i);
        const V8 v1 = hn::LoadU(d, row + i + N);
        vmin = hn::Min(vmin, hn::Min(hn::Or(v0, colorMask), hn::Or(v1, colorMask)));
        vmax = hn::Max(vmax, hn::Max(hn::And(v0, alphaMask), hn::And(v1, alphaMask)));
    }
    for (; i + N <= bytes; i += N) {
        const V8 v = hn::LoadU(d, row + i);
        vmin = hn::Min(vmin, hn::Or(v, colorMask));
        vmax = hn::Max(vmax, hn::And(v, alphaMask));
    }
    uint8_t mn = hn::ReduceMin(d, vmin);
    uint8_t mx = hn::ReduceMax(d, vmax);
    for (i += channels - 1; i < bytes; i += channels) {
        mn = row[i] < mn ? row[i] : mn;
        mx = row[i] > mx ? row[i] : mx;
    }
    *minAlpha = mn;
    *maxAlpha = mx;
}

// ------------------------------------------------------------------------------------------
// Whole PNG image

void PngFilterImageImpl(const PngFilterJob& job) {
    const size_t bpp = job.dstChannels;
    const size_t n = static_cast<size_t>(job.width) * bpp;
    const bool convert = job.dstChannels != job.srcChannels;
    uint8_t* zeros = job.scratch + 2 * n;
    std::memset(zeros, 0, n);
    const uint8_t* prev = zeros;
    for (uint32_t y = 0; y < job.height; ++y) {
        const uint8_t* src = job.src + job.srcStride * y;
        const uint8_t* cur = src;
        if (convert) {
            uint8_t* dst = job.scratch + (y & 1u) * n;
            if (job.srcChannels == 4) {
                DropAlpha4Impl(src, job.width, dst);
            } else {
                DropAlpha2Impl(src, job.width, dst);
            }
            cur = dst;
        }
        uint8_t* out = job.out + y * (n + 1);
        uint32_t filter = kFilterNone;
        uint64_t costs[5];
        switch (job.filter) {
            case PngRowFilter::kNone: filter = kFilterNone; break;
            case PngRowFilter::kSub: filter = kFilterSub; break;
            case PngRowFilter::kUp: filter = kFilterUp; break;
            case PngRowFilter::kAverage: filter = kFilterAverage; break;
            case PngRowFilter::kPaeth: filter = kFilterPaeth; break;
            case PngRowFilter::kAdaptive: FilterCostsT<31u>(cur, prev, n, bpp, costs); break;
        }
        if (job.filter == PngRowFilter::kAdaptive) {
            // Ties go to the lower filter type, as in libpng.
            uint64_t best = costs[0];
            for (uint32_t f = 1; f < 5; ++f) {
                if (costs[f] < best) {
                    best = costs[f];
                    filter = f;
                }
            }
        }
        out[0] = static_cast<uint8_t>(filter);
        FilterRowAny(cur, prev, n, bpp, filter, out + 1);
        prev = cur;
    }
}

}  // namespace HWY_NAMESPACE
}  // namespace ddbt::encode::kernels
HWY_AFTER_NAMESPACE();

#if HWY_ONCE

namespace ddbt::encode::kernels {

HWY_EXPORT(AlphaRangeImpl);
HWY_EXPORT(PngFilterCostsImpl);
HWY_EXPORT(PngFilterRowImpl);
HWY_EXPORT(PngFilterImageImpl);
HWY_EXPORT(DropAlpha4Impl);
HWY_EXPORT(DropAlpha2Impl);
HWY_EXPORT(WhiteUnderTransparent4Impl);
HWY_EXPORT(WhiteUnderTransparent2Impl);
HWY_EXPORT(GreyToRgbImpl);
HWY_EXPORT(GreyAlphaToRgbaImpl);

void AlphaRange(const uint8_t* row, size_t pixels, uint32_t channels, uint8_t& minAlpha, uint8_t& maxAlpha) {
    HWY_DYNAMIC_DISPATCH(AlphaRangeImpl)(row, pixels, channels, &minAlpha, &maxAlpha);
}

void PngFilterCosts(const uint8_t* cur, const uint8_t* prev, size_t n, size_t bpp, uint64_t costs[5]) {
    HWY_DYNAMIC_DISPATCH(PngFilterCostsImpl)(cur, prev, n, bpp, costs);
}

void PngFilterRow(const uint8_t* cur, const uint8_t* prev, size_t n, size_t bpp, PngFilterType filter,
                  uint8_t* out) {
    HWY_DYNAMIC_DISPATCH(PngFilterRowImpl)(cur, prev, n, bpp, filter, out);
}

void PngFilterImage(const PngFilterJob& job) { HWY_DYNAMIC_DISPATCH(PngFilterImageImpl)(job); }

void DropAlpha4(const uint8_t* rgba, size_t pixels, uint8_t* rgb) {
    HWY_DYNAMIC_DISPATCH(DropAlpha4Impl)(rgba, pixels, rgb);
}

void DropAlpha2(const uint8_t* ga, size_t pixels, uint8_t* g) { HWY_DYNAMIC_DISPATCH(DropAlpha2Impl)(ga, pixels, g); }

void WhiteUnderTransparent4(const uint8_t* rgba, size_t pixels, uint8_t* rgbx) {
    HWY_DYNAMIC_DISPATCH(WhiteUnderTransparent4Impl)(rgba, pixels, rgbx);
}

void WhiteUnderTransparent2(const uint8_t* ga, size_t pixels, uint8_t* g) {
    HWY_DYNAMIC_DISPATCH(WhiteUnderTransparent2Impl)(ga, pixels, g);
}

void GreyToRgb(const uint8_t* g, size_t pixels, uint8_t* rgb) { HWY_DYNAMIC_DISPATCH(GreyToRgbImpl)(g, pixels, rgb); }

void GreyAlphaToRgba(const uint8_t* ga, size_t pixels, uint8_t* rgba) {
    HWY_DYNAMIC_DISPATCH(GreyAlphaToRgbaImpl)(ga, pixels, rgba);
}

}  // namespace ddbt::encode::kernels

#endif  // HWY_ONCE
