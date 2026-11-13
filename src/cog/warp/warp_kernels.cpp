// SPDX-License-Identifier: MPL-2.0
//
// Highway SIMD kernels of the warp. Compiled once per enabled target (foreach_target.h re-includes
// this file); HWY_DYNAMIC_DISPATCH picks the best target at runtime. A lane is a destination pixel
// (its source position is a double); the source samples of its four taps are gathered or, for
// 4-byte pixels, looked up in source rows loaded whole.
//
// Bit-identical to the scalar code: the same IEEE operations (+, -, *, /, floor, trunc, int <->
// double conversions, all exact or correctly rounded) in the same order, and no contraction of a
// multiply and an add into an FMA (this file is compiled with -ffp-contract=off: GCC implements
// the x86 intrinsics with vector arithmetic it would otherwise contract).

#include "cog/warp/warp_kernels.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <type_traits>

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "cog/warp/warp_kernels.cpp"
#include "hwy/foreach_target.h"  // IWYU pragma: keep, must come before highway.h
#include "hwy/highway.h"

#if !HWY_IS_LITTLE_ENDIAN
#error "The warp kernels read samples as little-endian 32-bit words"
#endif

HWY_BEFORE_NAMESPACE();
namespace ddbt::cog::warp::kernels {
namespace HWY_NAMESPACE {

namespace hn = hwy::HWY_NAMESPACE;
// At most 32 lanes, so that a mask fits the 64-bit BitsFromMask.
using DF = hn::CappedTag<double, 32>;
using VF = hn::Vec<DF>;
using MF = hn::Mask<DF>;
using DI = hn::Rebind<int32_t, DF>;
using VI = hn::Vec<DI>;
using DU = hn::Rebind<uint32_t, DF>;
using VU = hn::Vec<DU>;
using DF32 = hn::Rebind<float, DF>;
using DI64 = hn::RebindToSigned<DF>;
constexpr size_t kMaxLanes = HWY_MAX_LANES_D(DF);

// GDAL's thresholds (gdalwarpkernel.cpp), as in bilinear.cpp.
constexpr double kSrcDensityThreshold = 0.000000001;
constexpr double kMinDivisor = 0.00001;

HWY_INLINE uint64_t LaneBits(DF d, MF m, size_t count) {
    const uint64_t bits = hn::BitsFromMask(d, m);
    return count >= 64 ? bits : bits & ((uint64_t{1} << count) - 1);
}

HWY_INLINE unsigned LowestLane(uint64_t bits) { return static_cast<unsigned>(hwy::Num0BitsBelowLS1Bit_Nonzero64(bits)); }

// ------------------------------------------------------------------------------------------
// Source pixel positions

void ToSourcePixelsImpl(const SourcePixelMapping& m, double* HWY_RESTRICT xs, double* HWY_RESTRICT ys, size_t n,
                        uint32_t* HWY_RESTRICT outside, size_t* outsideCount) {
    const DF d;
    const size_t N = hn::Lanes(d);
    const VF i0 = hn::Set(d, m.inverse[0]), i1 = hn::Set(d, m.inverse[1]), i2 = hn::Set(d, m.inverse[2]);
    const VF i3 = hn::Set(d, m.inverse[3]), i4 = hn::Set(d, m.inverse[4]), i5 = hn::Set(d, m.inverse[5]);
    const VF zero = hn::Zero(d), slack = hn::Set(d, 1e-10);
    const VF width = hn::Set(d, m.width), height = hn::Set(d, m.height);
    const VF nan = hn::Set(d, std::numeric_limits<double>::quiet_NaN());
    size_t k = 0;
    for (size_t i = 0; i < n; i += N) {
        const size_t c = HWY_MIN(N, n - i);
        const VF X = hn::LoadN(d, xs + i, c), Y = hn::LoadN(d, ys + i, c);
        const MF finite = hn::And(hn::IsFinite(X), hn::IsFinite(Y));
        const VF fx = hn::Add(hn::Add(i0, hn::Mul(X, i1)), hn::Mul(Y, i2));
        const VF fy = hn::Add(hn::Add(i3, hn::Mul(X, i4)), hn::Mul(Y, i5));
        const MF inside = hn::And(hn::And(hn::Ge(fx, zero), hn::Ge(fy, zero)),
                                  hn::And(hn::Le(hn::Add(fx, slack), width), hn::Le(hn::Add(fy, slack), height)));
        hn::StoreN(hn::IfThenElse(finite, fx, nan), d, xs + i, c);
        hn::StoreN(hn::IfThenElse(finite, fy, nan), d, ys + i, c);
        for (uint64_t bits = LaneBits(d, hn::AndNot(inside, finite), c); bits != 0; bits &= bits - 1)
            outside[k++] = static_cast<uint32_t>(i + LowestLane(bits));
    }
    *outsideCount = k;
}

void TapOriginBoundsImpl(const double* HWY_RESTRICT xs, const double* HWY_RESTRICT ys, size_t n, TapOrigins* out) {
    const DF d;
    const size_t N = hn::Lanes(d);
    const VF half = hn::Set(d, 0.5);
    const VF inf = hn::Inf(d), negInf = hn::Neg(inf);
    VF mnx = inf, mny = inf, mxx = negInf, mxy = negInf;
    MF any = hn::MaskFalse(d);
    for (size_t i = 0; i < n; i += N) {
        const size_t c = HWY_MIN(N, n - i);
        const VF X = hn::LoadN(d, xs + i, c), Y = hn::LoadN(d, ys + i, c);
        const MF live = hn::AndNot(hn::IsNaN(X), hn::FirstN(d, c));
        const VF ox = hn::Floor(hn::Sub(X, half)), oy = hn::Floor(hn::Sub(Y, half));
        mnx = hn::IfThenElse(live, hn::Min(mnx, ox), mnx);
        mny = hn::IfThenElse(live, hn::Min(mny, oy), mny);
        mxx = hn::IfThenElse(live, hn::Max(mxx, ox), mxx);
        mxy = hn::IfThenElse(live, hn::Max(mxy, oy), mxy);
        any = hn::Or(any, live);
    }
    out->any = !hn::AllFalse(d, any);
    out->minX = hn::ReduceMin(d, mnx);
    out->minY = hn::ReduceMin(d, mny);
    out->maxX = hn::ReduceMax(d, mxx);
    out->maxY = hn::ReduceMax(d, mxy);
}

// ------------------------------------------------------------------------------------------
// Bilinear resampling

// The sample of type T at byte `offset` of 32-bit words (offset + sizeof(T) <= 4), as double.
template <typename T>
HWY_INLINE VF WordSample(DF d, VU word, int offset) {
    const DI di;
    constexpr int kBits = 8 * static_cast<int>(sizeof(T));
    if constexpr (std::is_same_v<T, float>) {
        return hn::PromoteTo(d, hn::BitCast(DF32(), word));
    } else if constexpr (std::is_same_v<T, uint32_t>) {
        return hn::PromoteTo(d, word);
    } else if constexpr (std::is_same_v<T, int32_t>) {
        return hn::PromoteTo(d, hn::BitCast(di, word));
    } else if constexpr (std::is_unsigned_v<T>) {
        const VU v = hn::And(hn::ShiftRightSame(word, 8 * offset), hn::Set(DU(), (uint32_t{1} << kBits) - 1));
        return hn::PromoteTo(d, v);
    } else {
        const VI v = hn::ShiftLeftSame(hn::BitCast(di, word), 32 - 8 * offset - kBits);
        return hn::PromoteTo(d, hn::ShiftRightSame(v, 32 - kBits));
    }
}

// The samples of type T at byte offsets `offsets` from `base`, as double.
template <typename T>
HWY_INLINE VF GatherSample(DF d, const uint8_t* base, VI offsets) {
    if constexpr (sizeof(T) == 8) {
        return hn::GatherOffset(d, reinterpret_cast<const double*>(base), hn::PromoteTo(DI64(), offsets));
    } else {
        return WordSample<T>(d, hn::GatherOffset(DU(), reinterpret_cast<const uint32_t*>(base), offsets), 0);
    }
}

// RoundNoMask of bilinear.cpp: floats as they are, signed floor(v + 0.5), unsigned trunc(v + 0.5).
template <typename T>
HWY_INLINE VF RoundNoMask(DF d, VF v) {
    if constexpr (std::is_floating_point_v<T>) {
        return v;
    } else if constexpr (std::is_signed_v<T>) {
        return hn::Floor(hn::Add(v, hn::Set(d, 0.5)));
    } else {
        return hn::Trunc(hn::Add(v, hn::Set(d, 0.5)));
    }
}

// ClampRound of bilinear.cpp: the type's range, then RoundNoMask.
template <typename T>
HWY_INLINE VF ClampRound(DF d, VF v) {
    if constexpr (std::is_floating_point_v<T>) {
        return v;
    } else {
        const VF lo = hn::Set(d, static_cast<double>(std::numeric_limits<T>::lowest()));
        const VF hi = hn::Set(d, static_cast<double>(std::numeric_limits<T>::max()));
        return hn::IfThenElse(hn::Lt(v, lo), lo, hn::IfThenElse(hn::Gt(v, hi), hi, RoundNoMask<T>(d, v)));
    }
}


// Stores the lanes of v (whole numbers of the type's range) as uint8_t.
HWY_INLINE hn::Vec<hn::Rebind<uint8_t, DF>> ToBytes(DF /*d*/, VF v) {
    return hn::DemoteTo(hn::Rebind<uint8_t, DF>(), hn::DemoteInRangeTo(DI(), v));
}

template <typename T>
void BilinearRowT(const BilinearJob& j) {
    const DF d;
    const DI di;
    const DU du;
    const hn::Rebind<uint64_t, DF> du64;
    const hn::Rebind<uint8_t, DF> d8;
    const size_t N = hn::Lanes(d);
    const VF zero = hn::Zero(d), half = hn::Set(d, 0.5), one = hn::Set(d, 1.0), oneHalf = hn::Set(d, 1.5);
    const VF srcW = hn::Set(d, j.sourceWidth), srcH = hn::Set(d, j.sourceHeight);
    const VF srcWm1 = hn::Set(d, j.sourceWidth - 1), srcHm1 = hn::Set(d, j.sourceHeight - 1);
    const VF winX0 = hn::Set(d, j.winX0), winY0 = hn::Set(d, j.winY0);
    const VF winX1 = hn::Set(d, static_cast<double>(j.winX0) + j.winW);
    const VF winY1 = hn::Set(d, static_cast<double>(j.winY0) + j.winH);
    const VF winW = hn::Set(d, j.winW), pixelBytes = hn::Set(d, static_cast<double>(j.pixelBytes));
    const VF tapDelta[4] = {zero, one, winW, hn::Set(d, j.winW + 1.0)};
    const bool masked = !j.noMasks;
    constexpr size_t kSample = sizeof(T);
    // Pixels of at most 4 bytes: one 64-bit word holds a tap and its right neighbour.
    const bool pairTaps = j.pixelBytes <= 4;
    const int pairShift = 8 * static_cast<int>(j.pixelBytes);
    // Bands whose sample lies in the first 32-bit word of the pixel come from the words gathered
    // per tap; the others are gathered on their own.
    bool anyWordBand = pairTaps;
    for (size_t b = 0; b < j.bands; ++b) anyWordBand = anyWordBand || (kSample <= 4 && j.bandOffsets[b] + kSample <= 4);
    // Whole vectors of uint8_t samples written straight into an output of only those bands.
    const bool storeBytes = std::is_same_v<T, uint8_t> && j.outPixelBytes == j.bands && (j.bands == 1 || j.bands == 3);
    const auto flagsAt = [&](VI pixel) {
        return hn::And(hn::GatherOffset(du, reinterpret_cast<const uint32_t*>(j.flags), pixel), hn::Set(du, 0xFFu));
    };
    const auto isSet = [&](VU flags, VU bit) { return hn::Ne(hn::PromoteTo(d, hn::And(flags, bit)), zero); };
    const VU quadFull = hn::Set(du, WindowFlags::kQuadFull), centreValid = hn::Set(du, WindowFlags::kCentreValid);
    // Pixels of 4 bytes whose taps span at most two lanes' width of a row pair (the usual case:
    // destination pixels near the source resolution): the taps and flags come from rows loaded
    // whole and looked up in registers instead of gathers. The loads stay in the window's
    // padded buffers.
    const hn::ScalableTag<uint32_t> dt;
    const hn::Rebind<uint8_t, decltype(dt)> dt8;
    const size_t tableLanes = hn::Lanes(dt);
    const bool rowTables = j.pixelBytes == 4 && tableLanes == 2 * N;
    const size_t windowPixels = static_cast<size_t>(j.winW) * static_cast<size_t>(j.winH);
    const auto lookup = [&](hn::Vec<decltype(dt)> table, VU index) {
        return hn::LowerHalf(hn::TableLookupLanes(table, hn::IndicesFromVec(dt, hn::ZeroExtendVector(dt, index))));
    };
    // The words of the four taps and the flags of tap 0 of the candidate lanes from whole rows;
    // false when the lanes span too much.
    const auto loadRows = [&](VF ix, VF iy, MF cand, VU* words, VU* flags) {
        const double x0 = hn::MaskedReduceMin(d, cand, ix), y0 = hn::MaskedReduceMin(d, cand, iy);
        const VF dx = hn::Sub(ix, hn::Set(d, x0)), dy = hn::Sub(iy, hn::Set(d, y0));
        const MF fits = hn::And(hn::Le(dx, hn::Set(d, static_cast<double>(tableLanes - 2))), hn::Le(dy, one));
        if (!hn::AllTrue(d, hn::Or(hn::Not(cand), fits))) return false;
        const bool twoRows = !hn::AllFalse(d, hn::And(cand, hn::Eq(dy, one)));
        const size_t first = static_cast<size_t>(y0 - j.winY0) * static_cast<size_t>(j.winW) + static_cast<size_t>(x0 - j.winX0);
        const size_t lastRow = first + (twoRows ? 2 : 1) * static_cast<size_t>(j.winW);
        if ((lastRow + tableLanes) * 4 > windowPixels * 4 + 8) return false;
        if (masked && lastRow - static_cast<size_t>(j.winW) + tableLanes > windowPixels + 8) return false;
        const auto row = [&](size_t r) {
            return hn::LoadU(dt, reinterpret_cast<const uint32_t*>(j.px + (first + r * static_cast<size_t>(j.winW)) * 4));
        };
        const VU index = hn::BitCast(du, hn::DemoteInRangeTo(di, hn::IfThenElseZero(cand, dx)));
        const VU next = hn::Add(index, hn::Set(du, 1u));
        const auto r0 = row(0), r1 = row(1);
        if (!twoRows) {
            words[0] = lookup(r0, index);
            words[1] = lookup(r0, next);
            words[2] = lookup(r1, index);
            words[3] = lookup(r1, next);
            if (masked) *flags = lookup(hn::PromoteTo(dt, hn::LoadU(dt8, j.flags + first)), index);
            return true;
        }
        const auto r2 = row(2);
        const auto upper = hn::RebindMask(du, hn::Eq(hn::DemoteInRangeTo(di, hn::IfThenElseZero(cand, dy)), hn::Zero(di)));
        words[0] = hn::IfThenElse(upper, lookup(r0, index), lookup(r1, index));
        words[1] = hn::IfThenElse(upper, lookup(r0, next), lookup(r1, next));
        words[2] = hn::IfThenElse(upper, lookup(r1, index), lookup(r2, index));
        words[3] = hn::IfThenElse(upper, lookup(r1, next), lookup(r2, next));
        if (masked) {
            const auto f0 = hn::PromoteTo(dt, hn::LoadU(dt8, j.flags + first));
            const auto f1 = hn::PromoteTo(dt, hn::LoadU(dt8, j.flags + first + static_cast<size_t>(j.winW)));
            *flags = hn::IfThenElse(upper, lookup(f0, index), lookup(f1, index));
        }
        return true;
    };
    const auto bandValue = [&](const VI* offset, const VU* words, size_t b, const VF* m, VF div, MF divIsOne, VF rx,
                               VF ry, VF rx1, VF ry1, bool finiteOnly, MF* finite) {
        const size_t bo = j.bandOffsets[b];
        VF v[4];
        for (int t = 0; t < 4; ++t) {
            if (kSample <= 4 && bo + kSample <= 4) {
                v[t] = WordSample<T>(d, words[t], static_cast<int>(bo));
            } else {
                v[t] = GatherSample<T>(d, j.px + bo, offset[t]);
            }
            if (finiteOnly) *finite = hn::And(*finite, hn::IsFinite(v[t]));
        }
        if (masked) {
            // GWKBilinearResample4Sample: the sum in tap order, divided by the weights' sum.
            const VF acc = hn::Add(
                hn::Add(hn::Add(hn::Add(zero, hn::Mul(v[0], m[0])), hn::Mul(v[1], m[1])), hn::Mul(v[2], m[2])),
                hn::Mul(v[3], m[3]));
            // The weights often sum to exactly 1: no division then.
            return ClampRound<T>(d, hn::AllTrue(d, divIsOne) ? acc : hn::IfThenElse(divIsOne, acc, hn::Div(acc, div)));
        }
        // GWKBilinearResampleNoMasks4SampleT.
        const VF acc = hn::Add(hn::Mul(hn::Add(hn::Mul(v[0], rx), hn::Mul(v[1], rx1)), ry),
                               hn::Mul(hn::Add(hn::Mul(v[2], rx), hn::Mul(v[3], rx1)), ry1));
        return RoundNoMask<T>(d, acc);
    };

    HWY_ALIGN double values[kMaxLanes];
    for (size_t i = 0; i < j.n; i += N) {
        const size_t c = HWY_MIN(N, j.n - i);
        const VF fx = hn::LoadN(d, j.fx + i, c), fy = hn::LoadN(d, j.fy + i, c);
        const MF live = hn::AndNot(hn::IsNaN(fx), hn::FirstN(d, c));
        MF skipped = hn::Not(live);
        const VF ix = hn::Floor(hn::Sub(fx, half)), iy = hn::Floor(hn::Sub(fy, half));
        // Candidates: the four taps inside the source and, with masks, fully valid (then the
        // centre pixel, one of them, is valid, the renormalised sum has the weights' sum as its
        // divisor and the band density is exactly 1: no blend).
        MF cand = hn::And(live, hn::And(hn::And(hn::Ge(ix, zero), hn::Lt(hn::Add(ix, one), srcW)),
                                        hn::And(hn::Ge(iy, zero), hn::Lt(hn::Add(iy, one), srcH))));
        VI offset[4];
        VU words[4];
        bool haveWords = false;
        if (masked) {
            // An invalid centre pixel means nothing is written at all. The centre is one of the four
            // taps (trunc(x + 1e-10) is ix or ix + 1), whose flags the tap block's byte holds.
            VF cx = hn::Trunc(hn::Add(fx, hn::Set(d, 1.0e-10))), cy = hn::Trunc(hn::Add(fy, hn::Set(d, 1.0e-10)));
            cx = hn::IfThenElse(hn::Eq(cx, srcW), srcWm1, cx);
            cy = hn::IfThenElse(hn::Eq(cy, srcH), srcHm1, cy);
            const VF dx = hn::Sub(cx, ix), dy = hn::Sub(cy, iy);
            cand = hn::And(cand, hn::And(hn::And(hn::Ge(dx, zero), hn::Le(dx, one)), hn::And(hn::Ge(dy, zero), hn::Le(dy, one))));
            MF open = hn::AndNot(cand, live);
            if (!hn::AllFalse(d, cand)) {
                VU flags;
                haveWords = rowTables && loadRows(ix, iy, cand, words, &flags);
                if (!haveWords) {
                    const VF base = hn::Add(hn::Mul(hn::Sub(iy, winY0), winW), hn::Sub(ix, winX0));
                    flags = flagsAt(hn::DemoteInRangeTo(di, hn::IfThenElseZero(cand, base)));
                }
                const VU bit = hn::Shl(centreValid, hn::BitCast(du, hn::DemoteInRangeTo(di, hn::IfThenElseZero(cand, hn::Add(dx, hn::Add(dy, dy))))));
                skipped = hn::Or(skipped, hn::AndNot(isSet(flags, bit), cand));
                cand = hn::And(cand, isSet(flags, quadFull));
            }
            // The live pixels whose taps leave the source: the centre pixel's own flags.
            if (!hn::AllFalse(d, open)) {
                const MF inWindow = hn::And(open, hn::And(hn::And(hn::Ge(cx, winX0), hn::Lt(cx, winX1)),
                                                          hn::And(hn::Ge(cy, winY0), hn::Lt(cy, winY1))));
                const VI centre = hn::DemoteInRangeTo(
                    di, hn::IfThenElseZero(inWindow, hn::Add(hn::Mul(hn::Sub(cy, winY0), winW), hn::Sub(cx, winX0))));
                skipped = hn::Or(skipped, hn::AndNot(isSet(flagsAt(centre), centreValid), inWindow));
            }
        }
        if (!masked && rowTables && !hn::AllFalse(d, cand)) haveWords = loadRows(ix, iy, cand, words, nullptr);
        if (!haveWords && !hn::AllFalse(d, cand)) {
            const VF base = hn::Add(hn::Mul(hn::Sub(iy, winY0), winW), hn::Sub(ix, winX0));
            for (int t = 0; t < 4; ++t)
                offset[t] = hn::DemoteInRangeTo(di, hn::Mul(hn::IfThenElseZero(cand, hn::Add(base, tapDelta[t])), pixelBytes));
        }
        uint64_t fastBits = 0;
        VF rx = zero, ry = zero, rx1 = zero, ry1 = zero, m[4] = {zero, zero, zero, zero}, div = zero;
        MF divIsOne = hn::MaskFalse(d);
        if (!hn::AllFalse(d, cand)) {
            rx = hn::Sub(oneHalf, hn::Sub(fx, ix));
            ry = hn::Sub(oneHalf, hn::Sub(fy, iy));
            rx1 = hn::Sub(one, rx);
            ry1 = hn::Sub(one, ry);
            if (masked) {
                m[0] = hn::Mul(rx, ry);
                m[1] = hn::Mul(rx1, ry);
                m[2] = hn::Mul(rx, ry1);
                m[3] = hn::Mul(rx1, ry1);
                div = hn::Add(hn::Add(hn::Add(hn::Add(zero, m[0]), m[1]), m[2]), m[3]);
                divIsOne = hn::Eq(div, one);
                cand = hn::AndNot(hn::Lt(div, hn::Set(d, kMinDivisor)), cand);
            }
            if (haveWords) {
                // From the row tables.
            } else if (pairTaps) {
                const auto q0 = hn::GatherOffset(du64, reinterpret_cast<const uint64_t*>(j.px), hn::PromoteTo(DI64(), offset[0]));
                const auto q2 = hn::GatherOffset(du64, reinterpret_cast<const uint64_t*>(j.px), hn::PromoteTo(DI64(), offset[2]));
                words[0] = hn::TruncateTo(du, q0);
                words[1] = hn::TruncateTo(du, hn::ShiftRightSame(q0, pairShift));
                words[2] = hn::TruncateTo(du, q2);
                words[3] = hn::TruncateTo(du, hn::ShiftRightSame(q2, pairShift));
            } else if (anyWordBand) {
                for (int t = 0; t < 4; ++t)
                    words[t] = hn::GatherOffset(du, reinterpret_cast<const uint32_t*>(j.px), offset[t]);
            }
            if constexpr (std::is_floating_point_v<T>) {
                // Only finite samples: NaN payloads would depend on the operand order.
                for (size_t b = 0; b < j.bands && !hn::AllFalse(d, cand); ++b)
                    (void)bandValue(offset, words, b, m, div, divIsOne, rx, ry, rx1, ry1, true, &cand);
            }
            fastBits = LaneBits(d, cand, c);
        }
        const VF status = hn::IfThenElse(cand, one, hn::IfThenElseZero(hn::Not(skipped), hn::Set(d, 2.0)));
        hn::StoreN(ToBytes(d, status), d8, j.status + i, c);
        if (fastBits == 0) continue;
        uint8_t* const out = j.out + i * j.outPixelBytes;
        const bool whole = c == N && fastBits == LaneBits(d, hn::FirstN(d, N), N);
        if (storeBytes && whole) {
            if (j.bands == 1) {
                hn::StoreU(ToBytes(d, bandValue(offset, words, 0, m, div, divIsOne, rx, ry, rx1, ry1, false, nullptr)), d8, out);
            } else {
                hn::StoreInterleaved3(ToBytes(d, bandValue(offset, words, 0, m, div, divIsOne, rx, ry, rx1, ry1, false, nullptr)),
                                      ToBytes(d, bandValue(offset, words, 1, m, div, divIsOne, rx, ry, rx1, ry1, false, nullptr)),
                                      ToBytes(d, bandValue(offset, words, 2, m, div, divIsOne, rx, ry, rx1, ry1, false, nullptr)),
                                      d8, out);
            }
        } else {
            for (size_t b = 0; b < j.bands; ++b) {
                hn::Store(bandValue(offset, words, b, m, div, divIsOne, rx, ry, rx1, ry1, false, nullptr), d, values);
                for (uint64_t bits = fastBits; bits != 0; bits &= bits - 1) {
                    const unsigned l = LowestLane(bits);
                    const T s = static_cast<T>(values[l]);
                    std::memcpy(out + l * j.outPixelBytes + b * kSample, &s, kSample);
                }
            }
        }
        if (j.mask != nullptr) {
            if (whole) {
                hn::StoreU(hn::Set(d8, j.maskValue), d8, j.mask + i);
            } else {
                for (uint64_t bits = fastBits; bits != 0; bits &= bits - 1) j.mask[i + LowestLane(bits)] = j.maskValue;
            }
        }
    }
}

void BilinearRowImpl(const BilinearJob& j) {
    switch (j.type) {
        case tiff::SampleType::kU8: BilinearRowT<uint8_t>(j); break;
        case tiff::SampleType::kI8: BilinearRowT<int8_t>(j); break;
        case tiff::SampleType::kU16: BilinearRowT<uint16_t>(j); break;
        case tiff::SampleType::kI16: BilinearRowT<int16_t>(j); break;
        case tiff::SampleType::kU32: BilinearRowT<uint32_t>(j); break;
        case tiff::SampleType::kI32: BilinearRowT<int32_t>(j); break;
        case tiff::SampleType::kF32: BilinearRowT<float>(j); break;
        case tiff::SampleType::kF64: BilinearRowT<double>(j); break;
        default: std::memset(j.status, kScalar, j.n); break;
    }
}

}  // namespace HWY_NAMESPACE
}  // namespace ddbt::cog::warp::kernels
HWY_AFTER_NAMESPACE();

#if HWY_ONCE
namespace ddbt::cog::warp::kernels {

HWY_EXPORT(ToSourcePixelsImpl);
HWY_EXPORT(TapOriginBoundsImpl);
HWY_EXPORT(BilinearRowImpl);

size_t ToSourcePixels(const SourcePixelMapping& m, double* xs, double* ys, size_t n, uint32_t* outside, Path path) {
    if (path == Path::kSimd) {
        size_t count = 0;
        HWY_DYNAMIC_DISPATCH(ToSourcePixelsImpl)(m, xs, ys, n, outside, &count);
        return count;
    }
    const auto& inv = m.inverse;
    size_t k = 0;
    for (size_t i = 0; i < n; ++i) {
        const double X = xs[i], Y = ys[i];
        if (!std::isfinite(X) || X == HUGE_VAL || !std::isfinite(Y)) {
            xs[i] = ys[i] = std::numeric_limits<double>::quiet_NaN();
            continue;
        }
        const double fx = inv[0] + X * inv[1] + Y * inv[2];
        const double fy = inv[3] + X * inv[4] + Y * inv[5];
        xs[i] = fx;
        ys[i] = fy;
        if (!(fx >= 0 && fy >= 0 && fx + 1e-10 <= m.width && fy + 1e-10 <= m.height)) outside[k++] = static_cast<uint32_t>(i);
    }
    return k;
}

TapOrigins TapOriginBounds(const double* xs, const double* ys, size_t n, Path path) {
    TapOrigins o;
    if (path == Path::kSimd) {
        HWY_DYNAMIC_DISPATCH(TapOriginBoundsImpl)(xs, ys, n, &o);
        return o;
    }
    o.minX = o.minY = HUGE_VAL;
    o.maxX = o.maxY = -HUGE_VAL;
    for (size_t i = 0; i < n; ++i) {
        if (std::isnan(xs[i])) continue;
        const double ox = std::floor(xs[i] - 0.5), oy = std::floor(ys[i] - 0.5);
        o.minX = std::min(o.minX, ox);
        o.minY = std::min(o.minY, oy);
        o.maxX = std::max(o.maxX, ox);
        o.maxY = std::max(o.maxY, oy);
        o.any = true;
    }
    return o;
}

void BilinearRow(const BilinearJob& job) { HWY_DYNAMIC_DISPATCH(BilinearRowImpl)(job); }

}  // namespace ddbt::cog::warp::kernels
#endif  // HWY_ONCE
