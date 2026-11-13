// SPDX-License-Identifier: MPL-2.0
//
// Highway kernels of the overview convolution (see overview_kernels.h). Compiled once per
// enabled target (foreach_target.h re-includes this file); HWY_DYNAMIC_DISPATCH picks the best
// target at runtime. Compiled without floating-point contraction (sources.cmake): a product
// followed by a sum must stay two roundings, as in the scalar reference.

#include "cog/pyramid/overview_kernels.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "cog/pyramid/overview_kernels.cpp"
#include "hwy/foreach_target.h"  // IWYU pragma: keep, must come before highway.h
#include "hwy/highway.h"

HWY_BEFORE_NAMESPACE();
namespace ddbt::cog::pyramid::kernels {
namespace HWY_NAMESPACE {

namespace hn = hwy::HWY_NAMESPACE;
using D = hn::ScalableTag<double>;
using V = hn::Vec<D>;
using M = hn::Mask<D>;

// The vertical pass keeps four vectors of columns in flight: a tap's sums depend on the previous
// tap's, so fewer would leave the adder waiting on its own latency.
constexpr size_t kVectors = 4;

size_t ColumnGroupImpl() { return kVectors * hn::Lanes(D()); }
size_t RowPaddingImpl() { return 2 * hn::Lanes(D()); }

// The reference's masked accumulation: acc += sample * weight, skipped for a zero weight (an
// infinite sample under a zero weight must not turn the sum into NaN).
HWY_INLINE V AddWeighted(V acc, V sample, V weight) {
    return hn::IfThenElse(hn::Ne(weight, hn::Zero(D())), hn::Add(acc, hn::Mul(sample, weight)), acc);
}

// One step of the "half the taps" rule: the current run of non-zero mask values and the longest.
HWY_INLINE void Run(V mask, V& run, V& best) {
    const D d;
    run = hn::IfThenElseZero(hn::Ne(mask, hn::Zero(d)), hn::Add(run, hn::Set(d, 1.0)));
    best = hn::Max(best, run);
}

// Stores the first `lanes` lanes of v.
HWY_INLINE void StoreLanes(V v, double* p, size_t lanes) {
    const D d;
    if (lanes == hn::Lanes(d))
        hn::StoreU(v, d, p);
    else
        hn::StoreN(v, d, p, lanes);
}

// ------------------------------------------------------------------------------------------
// Horizontal pass: lanes are the destination columns of a group, each with its own taps.

// The taps of every group of Lanes() destination columns, laid out by tap then lane: tap j of
// lane l of group g at [(g * maxTaps + j) * N + l]. Lanes past their count (and past dstW) have
// weight 0 and read the zero after the row: a no-op in every sum (+0 added to a sum that started
// at +0 never changes it, NaN and infinities included).
struct ColumnGroups {
    size_t count = 0;  // groups of this pass (the buffers may hold more)
    size_t maxTaps = 0;
    std::vector<uint8_t> regular;  // lanes start two columns apart with the same count
    std::vector<uint8_t> dense;    // regular, and no weight is zero
    std::vector<int> taps;         // the most taps of a lane
    std::vector<int> first;        // source column of lane 0's first tap
    std::vector<int> lo, hi;       // validity blocks the group's taps cover
    std::vector<double> half;      // count / 2 per lane (the run rule)
    std::vector<double> sum255;    // per lane, the sum of w * 255 in tap order (all-valid weight sum)
    std::vector<double> raw, normalised, raw255;
    std::vector<int64_t> index;  // source column per tap and lane (gathers)

    // Every entry is written (the buffers only grow: a pass over fewer columns between two larger
    // ones does not make the next one clear them again).
    void Build(const AxisTaps& cols, int dstW, int srcW) {
        const size_t N = hn::Lanes(D());
        const size_t groups = (static_cast<size_t>(dstW) + N - 1) / N;
        count = groups;
        maxTaps = static_cast<size_t>(cols.maxTaps);
        const size_t perTap = groups * maxTaps * N;
        GrowBuffer(regular, groups);
        GrowBuffer(dense, groups);
        GrowBuffer(taps, groups);
        GrowBuffer(first, groups);
        GrowBuffer(lo, groups);
        GrowBuffer(hi, groups);
        GrowBuffer(half, groups * N);
        GrowBuffer(sum255, groups * N);
        GrowBuffer(raw, perTap);
        GrowBuffer(normalised, perTap);
        GrowBuffer(raw255, perTap);
        GrowBuffer(index, perTap);
        for (size_t g = 0; g < groups; ++g) {
            const size_t i0 = g * N;
            bool reg = i0 + N <= static_cast<size_t>(dstW);
            bool nonZero = true;
            int most = 0, low = std::numeric_limits<int>::max(), high = 0;
            for (size_t l = 0; l < N; ++l) {
                const size_t i = i0 + l;
                const bool real = i < static_cast<size_t>(dstW);
                const int s = real ? cols.start[i] : 0, n = real ? cols.count[i] : 0;
                reg = reg && n == cols.count[i0] && s == cols.start[i0] + 2 * static_cast<int>(l);
                most = std::max(most, n);
                if (n > 0) {
                    low = std::min(low, s);
                    high = std::max(high, s + n);
                }
                half[g * N + l] = static_cast<double>(n / 2);
                double sum = 0.0;
                for (size_t j = 0; j < maxTaps; ++j) {
                    const size_t at = (g * maxTaps + j) * N + l;
                    if (j < static_cast<size_t>(n)) {
                        const double w = cols.weights[i * maxTaps + j];
                        nonZero = nonZero && w != 0;
                        raw[at] = w;
                        normalised[at] = cols.normalised[i * maxTaps + j];
                        raw255[at] = w * 255.0;  // the reference's w * m for m = 255
                        sum += raw255[at];
                        index[at] = s + static_cast<int>(j);
                    } else {
                        raw[at] = normalised[at] = raw255[at] = 0.0;
                        index[at] = srcW;
                    }
                }
                sum255[g * N + l] = sum;
            }
            regular[g] = reg ? 1 : 0;
            dense[g] = reg && nonZero ? 1 : 0;
            taps[g] = most;
            first[g] = cols.start[i0];
            // No lane with a tap: an empty range (the all-valid path, every weight 0).
            const int block = static_cast<int>(kValidityBlock);
            lo[g] = most > 0 ? low / block : 0;
            hi[g] = most > 0 ? (high + block - 1) / block : 0;
        }
    }
};

// The tap values of one row for the lanes of a group: One(j) is tap j of every lane, Pair(j)
// taps j and j + 1. Regular groups: lane l reads row[first + 2l + j], the even and odd lanes of
// one deinterleaving load.
struct RegularTaps {
    const double* p;
    HWY_INLINE void Pair(size_t j, V& t0, V& t1) const { hn::LoadInterleaved2(D(), p + j, t0, t1); }
    HWY_INLINE V One(size_t j) const {
        V t0, t1;
        hn::LoadInterleaved2(D(), p + j, t0, t1);
        return t0;
    }
};

// Other groups gather; their lanes past their count read the zero at row[srcW].
struct GatheredTaps {
    const double* row;
    const int64_t* index;
    HWY_INLINE V One(size_t j) const {
        const D d;
        const hn::RebindToSigned<D> di;
        return hn::GatherIndex(d, row, hn::LoadU(di, index + j * hn::Lanes(d)));
    }
    HWY_INLINE void Pair(size_t j, V& t0, V& t1) const {
        t0 = One(j);
        t1 = One(j + 1);
    }
};

// Calls f with the tap sources of group g in rows rowA and rowB.
template <class F>
HWY_INLINE void WithTaps(const ColumnGroups& cg, size_t g, const double* rowA, const double* rowB, const F& f) {
    if (cg.regular[g]) {
        f(RegularTaps{rowA + cg.first[g]}, RegularTaps{rowB + cg.first[g]});
    } else {
        const int64_t* index = cg.index.data() + g * cg.maxTaps * hn::Lanes(D());
        f(GatheredTaps{rowA, index}, GatheredTaps{rowB, index});
    }
}

// step(j, a, b) for every tap j in order, with tap j of both rows.
template <class Taps, class Step>
HWY_INLINE void ForEachTap(size_t n, const Taps& a, const Taps& b, const Step& step) {
    size_t j = 0;
    for (; j + 1 < n; j += 2) {
        V a0, a1, b0, b1;
        a.Pair(j, a0, a1);
        b.Pair(j, b0, b1);
        step(j, a0, b0);
        step(j + 1, a1, b1);
    }
    if (j < n) step(j, a.One(j), b.One(j));
}

// Per-thread buffers of the horizontal pass: the masked weights of two rows.
struct TapBuffers {
    std::vector<double> wa, wb;
    void Reserve(size_t taps) {
        const size_t n = taps * hn::Lanes(D());
        GrowBuffer(wa, n);
        GrowBuffer(wb, n);
    }
};

// The per-thread column groups and buffers of the horizontal pass. Out of line, so that the
// passes hold plain references (a thread-local looked up inside the loops costs a call there).
struct HorizontalState {
    ColumnGroups groups;
    TapBuffers buffers;
};
HWY_NOINLINE HorizontalState& ThreadHorizontalState() {
    thread_local HorizontalState state;
    return state;
}

// Rows a and b of a pair. `two` is false when only one row is left (b == a, not written).
struct RowPair {
    size_t a, b;
    bool two;
};

// Group g of a row pair without a mask.
HWY_INLINE void HorizontalPlain(const HorizontalJob& job, const ColumnGroups& cg, size_t g, RowPair r, size_t lanes) {
    const D d;
    const size_t N = hn::Lanes(d);
    const size_t n = static_cast<size_t>(cg.taps[g]);
    const double* w = cg.normalised.data() + g * cg.maxTaps * N;
    const size_t nb = static_cast<size_t>(job.bands);
    for (size_t b = 0; b < nb; ++b) {
        V accA = hn::Zero(d), accB = accA;
        WithTaps(cg, g, job.samples + (r.a * nb + b) * job.rowStride, job.samples + (r.b * nb + b) * job.rowStride,
                 [&](const auto& ta, const auto& tb) HWY_ATTR {
                     ForEachTap(n, ta, tb, [&](size_t j, V va, V vb) HWY_ATTR {
                         const V wj = hn::LoadU(d, w + j * N);
                         accA = hn::Add(accA, hn::Mul(va, wj));
                         accB = hn::Add(accB, hn::Mul(vb, wj));
                     });
                 });
        double* out = job.horiz + b * job.outPlane + g * N;
        StoreLanes(accA, out + r.a * job.outRow, lanes);
        if (r.two) StoreLanes(accB, out + r.b * job.outRow, lanes);
    }
}

// What the validity summary says of a row's mask under a group's taps.
enum class RowMask : uint8_t {
    kMixed,       // read the mask
    kAllValid,    // every value is 255
    kAllInvalid,  // every value is 0
};

HWY_INLINE RowMask Summary(const HorizontalJob& job, const ColumnGroups& cg, size_t g, size_t r, size_t mp) {
    if (job.partial == nullptr) return RowMask::kMixed;
    const size_t blocks = ValidityBlocks(static_cast<size_t>(job.srcW)) + 1;
    const int* notFull = job.partial + (r * static_cast<size_t>(job.mask.planes) + mp) * 2 * blocks;
    const int* notEmpty = notFull + blocks;
    const size_t lo = static_cast<size_t>(cg.lo[g]), hi = static_cast<size_t>(cg.hi[g]);
    if (notFull[hi] == notFull[lo]) return RowMask::kAllValid;
    if (notEmpty[hi] == notEmpty[lo]) return RowMask::kAllInvalid;
    return RowMask::kMixed;
}

// The masked weights w * m of row r's taps under mask plane mp (into wm), and the lanes the run
// rule makes invalid. All valid: the precomputed w * 255 and no invalid lane, without reading
// the mask.
HWY_INLINE const double* MaskedWeights(const HorizontalJob& job, const ColumnGroups& cg, size_t g, size_t r, size_t mp,
                                       RowMask summary, double* wm, M& invalid) {
    const D d;
    const size_t N = hn::Lanes(d);
    const size_t at = g * cg.maxTaps * N;
    if (summary == RowMask::kAllValid) {
        invalid = hn::MaskFalse(d);
        return cg.raw255.data() + at;
    }
    const size_t n = static_cast<size_t>(cg.taps[g]);
    const double* raw = cg.raw.data() + at;
    const double* row = job.masks + (r * static_cast<size_t>(job.mask.planes) + mp) * job.rowStride;
    V run = hn::Zero(d), best = run;
    WithTaps(cg, g, row, row, [&](const auto& taps, const auto&) HWY_ATTR {
        ForEachTap(n, taps, taps, [&](size_t j, V m, V) HWY_ATTR {
            Run(m, run, best);
            hn::StoreU(hn::Mul(hn::LoadU(d, raw + j * N), m), d, wm + j * N);
        });
    });
    invalid = hn::Lt(best, hn::LoadU(d, cg.half.data() + g * N));
    return wm;
}

// The weighted sums of one band of a row pair. kDense: both rows are all valid with non-zero
// weights and no NaN sample can occur, so no sample is skipped (no zero-weight test) and the
// weight sums are the group's precomputed ones (the same additions in the same order).
template <bool kNan, bool kDense>
HWY_INLINE void MaskedSums(const ColumnGroups& cg, size_t g, const double* rowA, const double* rowB, const double* wmA,
                           const double* wmB, V& accA, V& accB, V& sumA, V& sumB) {
    const D d;
    const size_t N = hn::Lanes(d);
    const size_t n = static_cast<size_t>(cg.taps[g]);
    WithTaps(cg, g, rowA, rowB, [&](const auto& ta, const auto& tb) HWY_ATTR {
        ForEachTap(n, ta, tb, [&](size_t j, V vA, V vB) HWY_ATTR {
            V tA = hn::LoadU(d, wmA + j * N), tB = kDense ? tA : hn::LoadU(d, wmB + j * N);
            if constexpr (kNan) {
                tA = hn::IfThenZeroElse(hn::IsNaN(vA), tA);
                tB = hn::IfThenZeroElse(hn::IsNaN(vB), tB);
            }
            if constexpr (kDense) {
                accA = hn::Add(accA, hn::Mul(vA, tA));
                accB = hn::Add(accB, hn::Mul(vB, tA));
            } else {
                accA = AddWeighted(accA, vA, tA);
                accB = AddWeighted(accB, vB, tB);
                sumA = hn::Add(sumA, tA);
                sumB = hn::Add(sumB, tB);
            }
        });
    });
    if constexpr (kDense) sumA = sumB = hn::LoadU(d, cg.sum255.data() + g * N);
}

// Clears the group state flags a run of `lanes` columns, `valid` of them valid, contradicts.
HWY_INLINE void UpdateState(size_t valid, size_t lanes, uint8_t& state) {
    if (valid != lanes) state &= static_cast<uint8_t>(~kGroupAllValid);
    if (valid != 0) state &= static_cast<uint8_t>(~kGroupAllInvalid);
}

// Group g of a row pair under its mask(s).
template <bool kNan>
HWY_INLINE void HorizontalMasked(const HorizontalJob& job, const ColumnGroups& cg, TapBuffers& tb, size_t g, RowPair r,
                                 size_t lanes) {
    const D d;
    const size_t N = hn::Lanes(d);
    const size_t nb = static_cast<size_t>(job.bands);
    const bool perBand = job.mask.planes != 1;
    const V zero = hn::Zero(d), one = hn::Set(d, 1.0);
    // A band with no valid tap in a valid pixel: 0, or NaN when floating-point bands share a
    // mask (the vertical pass then gives that band no weight there).
    const bool nanWhenEmpty = kNan && !perBand;
    const V empty = nanWhenEmpty ? hn::Set(d, std::numeric_limits<double>::quiet_NaN()) : zero;
    for (size_t mp = 0; mp < static_cast<size_t>(job.mask.planes); ++mp) {
        const size_t b0 = perBand ? mp : 0, b1 = perBand ? mp + 1 : nb;
        const RowMask summaryA = Summary(job, cg, g, r.a, mp);
        const RowMask summaryB = r.two ? Summary(job, cg, g, r.b, mp) : summaryA;
        uint8_t* state = job.groupState != nullptr ? job.groupState + mp * job.statePlane + g / kVectors : nullptr;
        if (summaryA == RowMask::kAllInvalid && summaryB == RowMask::kAllInvalid) {
            // Every weight is 0: no valid tap (the run rule fails, or no weight), so every band is
            // 0 and the pixels are invalid, whatever the samples.
            for (const size_t row : {r.a, r.b}) {
                for (size_t b = b0; b < b1; ++b)
                    StoreLanes(zero, job.horiz + b * job.outPlane + g * N + row * job.outRow, lanes);
                StoreLanes(zero, job.valid + mp * job.outPlane + g * N + row * job.outRow, lanes);
                if (state != nullptr) state[row * job.stateRow] &= static_cast<uint8_t>(~kGroupAllValid);
            }
            continue;
        }
        M invA, invB = hn::MaskFalse(d);
        const double* wmA = MaskedWeights(job, cg, g, r.a, mp, summaryA, tb.wa.data(), invA);
        const double* wmB = r.two ? MaskedWeights(job, cg, g, r.b, mp, summaryB, tb.wb.data(), invB) : wmA;
        // All valid with non-zero weights and no NaN: no tap is skipped.
        const bool dense =
            !kNan && cg.dense[g] && summaryA == RowMask::kAllValid && summaryB == RowMask::kAllValid;
        M anyA = hn::MaskFalse(d), anyB = anyA;
        for (size_t b = b0; b < b1; ++b) {
            const double* rowA = job.samples + (r.a * nb + b) * job.rowStride;
            const double* rowB = job.samples + (r.b * nb + b) * job.rowStride;
            V accA = zero, accB = zero, sumA = zero, sumB = zero;
            if (dense)
                MaskedSums<kNan, true>(cg, g, rowA, rowB, wmA, wmB, accA, accB, sumA, sumB);
            else
                MaskedSums<kNan, false>(cg, g, rowA, rowB, wmA, wmB, accA, accB, sumA, sumB);
            const M posA = hn::Gt(sumA, zero), posB = hn::Gt(sumB, zero);
            anyA = hn::Or(anyA, posA);
            anyB = hn::Or(anyB, posB);
            double* out = job.horiz + b * job.outPlane + g * N;
            StoreLanes(hn::IfThenZeroElse(invA, hn::IfThenElse(posA, hn::Div(accA, sumA), empty)),
                       out + r.a * job.outRow, lanes);
            if (r.two)
                StoreLanes(hn::IfThenZeroElse(invB, hn::IfThenElse(posB, hn::Div(accB, sumB), empty)),
                           out + r.b * job.outRow, lanes);
        }
        double* valid = job.valid + mp * job.outPlane + g * N;
        const M okA = hn::AndNot(invA, anyA), okB = hn::AndNot(invB, anyB);
        StoreLanes(hn::IfThenElseZero(okA, one), valid + r.a * job.outRow, lanes);
        if (r.two) StoreLanes(hn::IfThenElseZero(okB, one), valid + r.b * job.outRow, lanes);
        if (state != nullptr) {
            const M real = hn::FirstN(d, lanes);
            UpdateState(hn::CountTrue(d, hn::And(okA, real)), lanes, state[r.a * job.stateRow]);
            if (r.two) UpdateState(hn::CountTrue(d, hn::And(okB, real)), lanes, state[r.b * job.stateRow]);
        }
        if (nanWhenEmpty) {
            // A valid run but no band with a valid tap (every sample NaN): the pixel is invalid
            // and its bands are 0, not NaN.
            const M noneA = hn::AndNot(invA, hn::Not(anyA)), noneB = hn::AndNot(invB, hn::Not(anyB));
            for (size_t b = b0; b < b1; ++b) {
                double* out = job.horiz + b * job.outPlane + g * N;
                if (!hn::AllFalse(d, noneA))
                    StoreLanes(hn::IfThenZeroElse(noneA, hn::LoadU(d, out + r.a * job.outRow)), out + r.a * job.outRow,
                               lanes);
                if (r.two && !hn::AllFalse(d, noneB))
                    StoreLanes(hn::IfThenZeroElse(noneB, hn::LoadU(d, out + r.b * job.outRow)), out + r.b * job.outRow,
                               lanes);
            }
        }
    }
}

// Rows [0, job.rows) of the planes, written to the job's outputs.
HWY_INLINE void HorizontalChunk(const HorizontalJob& job, const ColumnGroups& cg, TapBuffers& tb) {
    const size_t N = hn::Lanes(D());
    const size_t groups = cg.count;
    const size_t rows = static_cast<size_t>(job.rows);
    // Two rows at a time: they share the group's weights, and their sums are independent chains.
    for (size_t r = 0; r < rows; r += 2) {
        const RowPair pair{r, r + 1 < rows ? r + 1 : r, r + 1 < rows};
        for (size_t g = 0; g < groups; ++g) {
            const size_t lanes = std::min(N, static_cast<size_t>(job.dstW) - g * N);
            if (job.mask.planes == 0)
                HorizontalPlain(job, cg, g, pair, lanes);
            else if (job.mask.floatSamples)
                HorizontalMasked<true>(job, cg, tb, g, pair, lanes);
            else
                HorizontalMasked<false>(job, cg, tb, g, pair, lanes);
        }
    }
}

void HorizontalPassImpl(const HorizontalJob& job) {
    HorizontalState& state = ThreadHorizontalState();
    ColumnGroups& cg = state.groups;
    TapBuffers& tb = state.buffers;
    cg.Build(job.cols, job.dstW, job.srcW);
    tb.Reserve(cg.maxTaps);
    for (int y = 0; y < job.rows; y += job.chunkRows) {
        HorizontalJob chunk = job;
        chunk.rows = std::min(job.chunkRows, job.rows - y);
        job.fill(job.fillContext, y, chunk.rows);
        const size_t at = static_cast<size_t>(y);
        chunk.horiz += at * job.outRow;
        if (chunk.valid != nullptr) chunk.valid += at * job.outRow;
        if (chunk.groupState != nullptr) chunk.groupState += at * job.stateRow;
        HorizontalChunk(chunk, cg, tb);
    }
}

// ------------------------------------------------------------------------------------------
// Vertical pass: lanes are destination columns, one destination row at a time.

HWY_INLINE void VerticalPlain(const VerticalJob& job, size_t x) {
    const D d;
    const size_t N = hn::Lanes(d);
    for (size_t b = 0; b < static_cast<size_t>(job.bands); ++b) {
        const double* p = job.horiz + b * job.plane + static_cast<size_t>(job.start) * job.rowStride + x;
        V a0 = hn::Zero(d), a1 = a0, a2 = a0, a3 = a0;
        for (int j = 0; j < job.count; ++j, p += job.rowStride) {
            const V wj = hn::Set(d, job.normalised[j]);
            a0 = hn::Add(a0, hn::Mul(hn::LoadU(d, p), wj));
            a1 = hn::Add(a1, hn::Mul(hn::LoadU(d, p + N), wj));
            a2 = hn::Add(a2, hn::Mul(hn::LoadU(d, p + 2 * N), wj));
            a3 = hn::Add(a3, hn::Mul(hn::LoadU(d, p + 3 * N), wj));
        }
        double* out = job.value + b * static_cast<size_t>(job.width) + x;
        hn::StoreU(a0, d, out);
        hn::StoreU(a1, d, out + N);
        hn::StoreU(a2, d, out + 2 * N);
        hn::StoreU(a3, d, out + 3 * N);
    }
}

// Columns [x, x + ColumnGroup()) of the bands under mask plane mp. kFull: every row the taps
// read is valid in these columns (the run rule holds, each weight is w * 1 = w).
template <bool kNan, bool kFull>
HWY_INLINE void VerticalMasked(const VerticalJob& job, size_t x, size_t mp) {
    const D d;
    const size_t N = hn::Lanes(d);
    const size_t nb = static_cast<size_t>(job.bands);
    const bool perBand = job.mask.planes != 1;
    const V zero = hn::Zero(d), one = hn::Set(d, 1.0);
    const size_t first = static_cast<size_t>(job.start) * job.rowStride + x;
    const double* vp = job.valid + mp * job.plane + first;
    M inv0 = hn::MaskFalse(d), inv1 = inv0, inv2 = inv0, inv3 = inv0;
    if constexpr (!kFull) {
        V run0 = zero, run1 = zero, run2 = zero, run3 = zero, best0 = zero, best1 = zero, best2 = zero, best3 = zero;
        for (int j = 0; j < job.count; ++j) {
            const double* p = vp + static_cast<size_t>(j) * job.rowStride;
            Run(hn::LoadU(d, p), run0, best0);
            Run(hn::LoadU(d, p + N), run1, best1);
            Run(hn::LoadU(d, p + 2 * N), run2, best2);
            Run(hn::LoadU(d, p + 3 * N), run3, best3);
        }
        const V half = hn::Set(d, static_cast<double>(job.count / 2));
        inv0 = hn::Lt(best0, half);
        inv1 = hn::Lt(best1, half);
        inv2 = hn::Lt(best2, half);
        inv3 = hn::Lt(best3, half);
    }
    const size_t b0 = perBand ? mp : 0, b1 = perBand ? mp + 1 : nb;
    for (size_t b = b0; b < b1; ++b) {
        const double* hp = job.horiz + b * job.plane + first;
        V a0 = zero, a1 = zero, a2 = zero, a3 = zero;
        V s0 = zero, s1 = zero, s2 = zero, s3 = zero;
        for (int j = 0; j < job.count; ++j) {
            const size_t o = static_cast<size_t>(j) * job.rowStride;
            const V wj = hn::Set(d, job.weights[j]);
            V t0 = wj, t1 = wj, t2 = wj, t3 = wj;
            if constexpr (!kFull) {
                t0 = hn::Mul(wj, hn::LoadU(d, vp + o));
                t1 = hn::Mul(wj, hn::LoadU(d, vp + o + N));
                t2 = hn::Mul(wj, hn::LoadU(d, vp + o + 2 * N));
                t3 = hn::Mul(wj, hn::LoadU(d, vp + o + 3 * N));
            }
            V h0 = hn::LoadU(d, hp + o), h1 = hn::LoadU(d, hp + o + N), h2 = hn::LoadU(d, hp + o + 2 * N),
              h3 = hn::LoadU(d, hp + o + 3 * N);
            if constexpr (kNan) {
                // This band had no valid tap in that row: no value, no weight.
                const M n0 = hn::IsNaN(h0), n1 = hn::IsNaN(h1), n2 = hn::IsNaN(h2), n3 = hn::IsNaN(h3);
                h0 = hn::IfThenZeroElse(n0, h0);
                h1 = hn::IfThenZeroElse(n1, h1);
                h2 = hn::IfThenZeroElse(n2, h2);
                h3 = hn::IfThenZeroElse(n3, h3);
                t0 = hn::IfThenZeroElse(n0, t0);
                t1 = hn::IfThenZeroElse(n1, t1);
                t2 = hn::IfThenZeroElse(n2, t2);
                t3 = hn::IfThenZeroElse(n3, t3);
            }
            a0 = hn::Add(a0, hn::Mul(h0, t0));
            a1 = hn::Add(a1, hn::Mul(h1, t1));
            a2 = hn::Add(a2, hn::Mul(h2, t2));
            a3 = hn::Add(a3, hn::Mul(h3, t3));
            s0 = hn::Add(s0, t0);
            s1 = hn::Add(s1, t1);
            s2 = hn::Add(s2, t2);
            s3 = hn::Add(s3, t3);
        }
        double* out = job.value + b * static_cast<size_t>(job.width) + x;
        double* fill = job.fill + b * static_cast<size_t>(job.width) + x;
        hn::StoreU(hn::Div(a0, s0), d, out);
        hn::StoreU(hn::Div(a1, s1), d, out + N);
        hn::StoreU(hn::Div(a2, s2), d, out + 2 * N);
        hn::StoreU(hn::Div(a3, s3), d, out + 3 * N);
        hn::StoreU(hn::IfThenElseZero(hn::Or(inv0, hn::Not(hn::Gt(s0, zero))), one), d, fill);
        hn::StoreU(hn::IfThenElseZero(hn::Or(inv1, hn::Not(hn::Gt(s1, zero))), one), d, fill + N);
        hn::StoreU(hn::IfThenElseZero(hn::Or(inv2, hn::Not(hn::Gt(s2, zero))), one), d, fill + 2 * N);
        hn::StoreU(hn::IfThenElseZero(hn::Or(inv3, hn::Not(hn::Gt(s3, zero))), one), d, fill + 3 * N);
    }
}

// The state of column group k under mask plane mp over the rows the taps read.
HWY_INLINE uint8_t TapRowsState(const VerticalJob& job, size_t k, size_t mp) {
    if (job.groupState == nullptr) return 0;
    const uint8_t* state = job.groupState + mp * job.statePlane + static_cast<size_t>(job.start) * job.stateRow + k;
    uint8_t all = kGroupAllValid | kGroupAllInvalid;
    for (int j = 0; j < job.count && all != 0; ++j) all &= state[static_cast<size_t>(j) * job.stateRow];
    return all;
}

// Columns [x, x + ColumnGroup()) of the bands under mask plane mp when no row the taps read has a
// valid pixel there: every weight is 0 (and the run rule fails), so each takes the nodata value.
HWY_INLINE void VerticalAllInvalid(const VerticalJob& job, size_t x, size_t mp) {
    const D d;
    const size_t N = hn::Lanes(d);
    const bool perBand = job.mask.planes != 1;
    const size_t b0 = perBand ? mp : 0, b1 = perBand ? mp + 1 : static_cast<size_t>(job.bands);
    for (size_t b = b0; b < b1; ++b) {
        double* fill = job.fill + b * static_cast<size_t>(job.width) + x;
        for (size_t v = 0; v < kVectors; ++v) hn::StoreU(hn::Set(d, 1.0), d, fill + v * N);
    }
}

template <bool kNan>
HWY_INLINE void VerticalMaskedGroup(const VerticalJob& job, size_t x, size_t k) {
    for (size_t mp = 0; mp < static_cast<size_t>(job.mask.planes); ++mp) {
        const uint8_t state = TapRowsState(job, k, mp);
        if (state & kGroupAllValid)
            VerticalMasked<kNan, true>(job, x, mp);
        else if (state & kGroupAllInvalid)
            VerticalAllInvalid(job, x, mp);
        else
            VerticalMasked<kNan, false>(job, x, mp);
    }
}

void VerticalPassImpl(const VerticalJob& job) {
    const size_t group = ColumnGroupImpl();
    for (size_t x = 0; x < static_cast<size_t>(job.width); x += group) {
        if (job.mask.planes == 0)
            VerticalPlain(job, x);
        else if (job.mask.floatSamples)
            VerticalMaskedGroup<true>(job, x, x / group);
        else
            VerticalMaskedGroup<false>(job, x, x / group);
    }
}

// ------------------------------------------------------------------------------------------
// 8-bit output.

size_t WriteBytesImpl(const double* value, const double* fill, int bands, size_t width, size_t count, uint8_t* dst) {
    const D d;
    const hn::Rebind<float, D> df;
    const hn::Rebind<int32_t, D> di;
    const hn::Rebind<uint8_t, D> d8;
    const size_t N = hn::Lanes(d);
    if (bands != 1 && bands != 3 && bands != 4) return 0;
    const auto byte = [&](size_t b, size_t i) HWY_ATTR {
        V v = hn::LoadU(d, value + b * width + i);
        if (fill != nullptr) v = hn::IfThenZeroElse(hn::Ne(hn::LoadU(d, fill + b * width + i), hn::Zero(d)), v);
        auto f = hn::DemoteTo(df, v);
        f = hn::IfThenZeroElse(hn::IsNaN(f), f);
        f = hn::Min(hn::Max(f, hn::Zero(df)), hn::Set(df, 255.0f));
        return hn::DemoteTo(d8, hn::ConvertTo(di, hn::Add(f, hn::Set(df, 0.5f))));
    };
    size_t i = 0;
    for (; i + N <= count; i += N) {
        uint8_t* out = dst + i * static_cast<size_t>(bands);
        if (bands == 1)
            hn::StoreU(byte(0, i), d8, out);
        else if (bands == 3)
            hn::StoreInterleaved3(byte(0, i), byte(1, i), byte(2, i), d8, out);
        else
            hn::StoreInterleaved4(byte(0, i), byte(1, i), byte(2, i), byte(3, i), d8, out);
    }
    return i;
}

}  // namespace HWY_NAMESPACE
}  // namespace ddbt::cog::pyramid::kernels
HWY_AFTER_NAMESPACE();

#if HWY_ONCE

namespace ddbt::cog::pyramid::kernels {

HWY_EXPORT(ColumnGroupImpl);
HWY_EXPORT(RowPaddingImpl);
HWY_EXPORT(HorizontalPassImpl);
HWY_EXPORT(VerticalPassImpl);
HWY_EXPORT(WriteBytesImpl);

size_t ColumnGroup() { return HWY_DYNAMIC_DISPATCH(ColumnGroupImpl)(); }
size_t RowPadding() { return HWY_DYNAMIC_DISPATCH(RowPaddingImpl)(); }
void HorizontalPass(const HorizontalJob& job) { HWY_DYNAMIC_DISPATCH(HorizontalPassImpl)(job); }
void VerticalPass(const VerticalJob& job) { HWY_DYNAMIC_DISPATCH(VerticalPassImpl)(job); }
size_t WriteBytes(const double* value, const double* fill, int bands, size_t width, size_t count, uint8_t* dst) {
    return HWY_DYNAMIC_DISPATCH(WriteBytesImpl)(value, fill, bands, width, count, dst);
}

}  // namespace ddbt::cog::pyramid::kernels

#endif  // HWY_ONCE
