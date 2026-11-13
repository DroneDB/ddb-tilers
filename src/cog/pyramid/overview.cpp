// SPDX-License-Identifier: MPL-2.0
// Compiled without floating-point contraction (sources.cmake): sums round as GDAL's do.
#include "cog/pyramid/overview.h"

#include "cog/pyramid/overview_kernels.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace ddbt::cog::pyramid {

double CubicKernel(double x) {
    const double a = std::fabs(x);
    if (a <= 1.0) {
        const double x2 = x * x;
        return x2 * (1.5 * a - 2.5) + 1;
    }
    if (a <= 2.0) {
        const double x2 = x * x;
        return x2 * (-0.5 * a + 2.5) - 4 * a + 2;
    }
    return 0.0;
}

int Taps::SrcBegin() const {
    int b = std::numeric_limits<int>::max();
    for (size_t i = 0; i < start.size(); ++i)
        if (count[i] > 0) b = std::min(b, start[i]);
    return b == std::numeric_limits<int>::max() ? 0 : b;
}

int Taps::SrcEnd() const {
    int e = 0;
    for (size_t i = 0; i < start.size(); ++i) e = std::max(e, start[i] + count[i]);
    return e;
}

Taps CubicTaps(int srcSize, int dstSize, int dst0, int dst1) {
    constexpr int kRadius = 2;
    Taps t;
    t.dst0 = dst0;
    const double ratio = static_cast<double>(srcSize) / dstSize;
    const double scale = 1.0 / ratio;
    const double scaleWeight = scale >= 1.0 ? 1.0 : scale;
    const double scaledRadius = kRadius / scaleWeight;
    t.maxTaps = static_cast<int>(2 + 2 * scaledRadius + 0.5);
    const size_t n = static_cast<size_t>(dst1 - dst0);
    t.start.resize(n);
    t.count.resize(n);
    t.invSum.resize(n);
    t.weights.assign(n * static_cast<size_t>(t.maxTaps), 0.0);
    for (int i = dst0; i < dst1; ++i) {
        const size_t k = static_cast<size_t>(i - dst0);
        const double srcPixel = (i + 0.5) * ratio;
        int s0 = static_cast<int>(std::floor(srcPixel - scaledRadius + 0.5));
        if (s0 < 0) s0 = 0;
        int s1 = static_cast<int>(srcPixel + scaledRadius + 0.5);
        if (s1 > srcSize) s1 = srcSize;
        const int cnt = std::max(0, s1 - s0);
        t.start[k] = s0;
        t.count[k] = cnt;
        double* w = t.weights.data() + k * static_cast<size_t>(t.maxTaps);
        double sum = 0.0;
        // GDAL fills groups of 4 as dfX, dfX + s, ... then the rest one by one; the values are the
        // same (dfX advanced by repeated addition).
        double x = scaleWeight * (s0 - srcPixel + 0.5);
        int j = 0;
        for (; j < cnt - 3; j += 4) {
            w[j] = x;
            x += scaleWeight;
            w[j + 1] = x;
            x += scaleWeight;
            w[j + 2] = x;
            x += scaleWeight;
            w[j + 3] = x;
            x += scaleWeight;
            // GDALFilterFunc4Values: the four kernel values, summed in this order.
            w[j] = CubicKernel(w[j]);
            w[j + 1] = CubicKernel(w[j + 1]);
            w[j + 2] = CubicKernel(w[j + 2]);
            w[j + 3] = CubicKernel(w[j + 3]);
            sum += w[j] + w[j + 1] + w[j + 2] + w[j + 3];
        }
        for (; j < cnt; ++j, x += scaleWeight) {
            w[j] = CubicKernel(x);
            sum += w[j];
        }
        t.invSum[k] = sum != 0 ? 1.0 / sum : 0.0;
    }
    return t;
}

namespace {

template <typename T>
inline double Load(const uint8_t* p) {
    T v;
    std::memcpy(&v, p, sizeof(T));
    return static_cast<double>(v);
}

// GDALCopyWord from the work type to an integer: NaN -> 0, clamped, rounded half away from zero.
template <typename T, typename W>
inline T ToSample(W v) {
    if constexpr (std::is_floating_point_v<T>) {
        return static_cast<T>(v);
    } else {
        if (std::isnan(v)) return 0;
        constexpr W lo = static_cast<W>(std::numeric_limits<T>::lowest());
        constexpr W hi = static_cast<W>(std::numeric_limits<T>::max());
        if (v <= lo) return std::numeric_limits<T>::lowest();
        if (v >= hi) return std::numeric_limits<T>::max();
        return static_cast<T>(v < 0 ? v - W(0.5) : v + W(0.5));
    }
}

template <typename T>
double ReplacementValue(double nd) {
    if constexpr (std::is_floating_point_v<T>) {
        if (std::isnan(nd) || nd < static_cast<double>(std::numeric_limits<T>::lowest()) ||
            nd > static_cast<double>(std::numeric_limits<T>::max()))
            return 0;
        if (nd == static_cast<double>(std::numeric_limits<T>::max()))
            return static_cast<double>(std::nextafter(static_cast<T>(nd), T(0)));
        return static_cast<double>(std::nextafter(static_cast<T>(nd), std::numeric_limits<T>::max()));
    } else {
        if (std::isnan(nd) || nd < static_cast<double>(std::numeric_limits<T>::lowest()) ||
            nd > static_cast<double>(std::numeric_limits<T>::max()))
            return 0;
        if (nd == static_cast<double>(std::numeric_limits<T>::max())) return nd - 1;
        return nd + 1;
    }
}

// Whether sample v (as loaded) is the band's nodata value: NaN for a NaN nodata, the value
// compared in the sample type for Float32.
template <typename T>
inline bool IsNoData(double v, double nd, bool ndNan) {
    if (ndNan) return std::isnan(v);
    if constexpr (std::is_same_v<T, float>) return static_cast<float>(v) == static_cast<float>(nd);
    return v == nd;
}

// The last step of a destination sample: the work-type value, GDAL's nodata replacement, the
// sample type.
template <typename T, typename W>
class SampleWriter {
public:
    explicit SampleWriter(const DownsampleSpec& spec)
        : hasNd_(spec.hasNoData),
          nd_(spec.hasNoData ? spec.noData : 0.0),
          repl_(spec.hasNoData ? ReplacementValue<T>(nd_) : nd_),
          ndInt64Valid_(!std::is_floating_point_v<T> && hasNd_ && nd_ == std::trunc(nd_) && std::fabs(nd_) < 9.2e18) {}

    /// A computed value: a value equal to the nodata value is replaced.
    void Value(double v, uint8_t* out) const { Store(ReplaceIfNoData(static_cast<W>(v)), out); }
    /// A pixel without a value: the nodata value (0 without nodata).
    void NoData(uint8_t* out) const { Store(static_cast<W>(nd_), out); }

private:
    W ReplaceIfNoData(W v) const {
        if (!hasNd_) return v;
        W c = v;
        if constexpr (!std::is_floating_point_v<T>) {
            constexpr W dstMin = static_cast<W>(std::numeric_limits<T>::lowest());
            constexpr W dstMax = static_cast<W>(std::numeric_limits<T>::max());
            if (c < dstMin) c = dstMin;
            else if (c > dstMax) c = dstMax;
            if (ndInt64Valid_ && static_cast<int64_t>(std::round(static_cast<double>(c))) == static_cast<int64_t>(nd_))
                return static_cast<W>(repl_);
            return c;
        } else {
            if (nd_ == static_cast<double>(c)) return static_cast<W>(repl_);
            return c;
        }
    }
    static void Store(W v, uint8_t* out) {
        const T t = ToSample<T, W>(v);
        std::memcpy(out, &t, sizeof(T));
    }

    bool hasNd_;
    double nd_;
    double repl_;
    bool ndInt64Valid_;
};

// The source rows the vertical taps read, checked against the source window.
std::pair<int, int> TapRows(const Taps& cols, const Taps& rows, int srcX0, int srcY0, int srcW, int srcH) {
    const int r0 = rows.SrcBegin(), r1 = rows.SrcEnd();
    if (r0 < srcY0 || r1 > srcY0 + srcH || cols.SrcBegin() < srcX0 || cols.SrcEnd() > srcX0 + srcW)
        throw std::logic_error("overview: the source window does not cover the taps");
    return {r0, r1};
}

// Calls f(T{}, W{}) with the sample type of `type` and its work type.
template <typename F>
void WithSampleType(tiff::SampleType type, F&& f) {
    using tiff::SampleType;
    switch (type) {
        case SampleType::kU8: f(uint8_t{}, float{}); break;
        case SampleType::kI8: f(int8_t{}, float{}); break;
        case SampleType::kU16: f(uint16_t{}, float{}); break;
        case SampleType::kI16: f(int16_t{}, float{}); break;
        case SampleType::kU32: f(uint32_t{}, double{}); break;
        case SampleType::kI32: f(int32_t{}, double{}); break;
        case SampleType::kF32: f(float{}, float{}); break;
        case SampleType::kF64: f(double{}, double{}); break;
        default: throw std::invalid_argument("overview: unsupported sample type");
    }
}

// The scalar implementation: one destination pixel at a time, every band of it in one walk over
// its taps. Kept as the reference the SIMD passes are tested against.
template <typename T, typename W, int NB>
void ReferenceT(const DownsampleSpec& spec, const uint8_t* src, size_t srcStride, int srcX0, int srcY0, int srcW,
                int srcH, const uint8_t* srcMask, const Taps& cols, const Taps& rows, int dstW, int dstH, uint8_t* dst,
                size_t dstStride) {
    const int bands = spec.bands;
    // NB > 0: the band count known at compile time (the accumulators stay in registers).
    const size_t nb = NB > 0 ? static_cast<size_t>(NB) : static_cast<size_t>(bands);
    const size_t px = sizeof(T) * nb;
    const bool masked = spec.mask != MaskMode::kNone;
    const bool hasNd = spec.hasNoData;
    const double nd = hasNd ? spec.noData : 0.0;
    const bool ndNan = hasNd && std::isnan(nd);
    const SampleWriter<T, W> writer(spec);

    // Source rows the vertical taps read.
    const auto [r0, r1] = TapRows(cols, rows, srcX0, srcY0, srcW, srcH);
    const int nRows = r1 - r0;
    const size_t sW = static_cast<size_t>(dstW);
    const size_t plane = static_cast<size_t>(nRows) * sW;  // one band of the horizontal pass
    // Masks shared by every band (external) are evaluated once per tap run; nodata masks are
    // per band.
    const bool perBandMask = spec.mask == MaskMode::kNoData;
    const size_t maskPlanes = masked ? (perBandMask ? nb : 1) : 0;
    thread_local std::vector<double> tlsHoriz;
    thread_local std::vector<uint8_t> tlsHorizValid, tlsBandMask;
    tlsHoriz.resize(plane * nb);
    tlsHorizValid.resize(plane * std::max<size_t>(maskPlanes, 1));
    double* const horiz = tlsHoriz.data();
    uint8_t* const horizValid = tlsHorizValid.data();
    const size_t mstride = static_cast<size_t>(srcW);
    const uint8_t* mbase = nullptr;
    if (spec.mask == MaskMode::kExternal) {
        mbase = srcMask + static_cast<size_t>(r0 - srcY0) * mstride;
    } else if (perBandMask) {
        tlsBandMask.resize(nb * static_cast<size_t>(nRows) * mstride);
        uint8_t* bm = tlsBandMask.data();
        for (size_t b = 0; b < nb; ++b)
            for (int y = 0; y < nRows; ++y) {
                const uint8_t* row = src + static_cast<size_t>(r0 - srcY0 + y) * srcStride + sizeof(T) * b;
                uint8_t* m = bm + (b * static_cast<size_t>(nRows) + static_cast<size_t>(y)) * mstride;
                for (int x = 0; x < srcW; ++x)
                    m[x] = IsNoData<T>(Load<T>(row + static_cast<size_t>(x) * px), nd, ndNan) ? 0 : 255;
            }
        mbase = bm;
    }
    constexpr size_t kFixed = NB > 0 ? static_cast<size_t>(NB) : 1;
    std::vector<double> accHeap(NB > 0 ? 0 : nb), wsHeap(NB > 0 ? 0 : nb);
    double accFixed[kFixed], wsFixed[kFixed];
    double* const acc = NB > 0 ? accFixed : accHeap.data();
    double* const ws = NB > 0 ? wsFixed : wsHeap.data();

    // Horizontal pass: every band of a destination column in one walk over its taps.
    for (int y = 0; y < nRows; ++y) {
        const uint8_t* row = src + static_cast<size_t>(r0 - srcY0 + y) * srcStride;
        for (int i = 0; i < dstW; ++i) {
            const size_t k = static_cast<size_t>(i);
            const int s = cols.start[k] - srcX0;
            const int n = cols.count[k];
            const double* w = cols.weights.data() + k * static_cast<size_t>(cols.maxTaps);
            const uint8_t* p0 = row + static_cast<size_t>(s) * px;
            const size_t out = static_cast<size_t>(y) * sW + k;
            if (!masked) {
                const double inv = cols.invSum[k];
                for (size_t b = 0; b < nb; ++b) acc[b] = 0.0;
                for (int j = 0; j < n; ++j) {
                    const double wj = w[j] * inv;
                    const uint8_t* p = p0 + static_cast<size_t>(j) * px;
                    for (size_t b = 0; b < nb; ++b) acc[b] += Load<T>(p + sizeof(T) * b) * wj;
                }
                for (size_t b = 0; b < nb; ++b) horiz[b * plane + out] = acc[b];
                continue;
            }
            for (size_t mp = 0; mp < maskPlanes; ++mp) {
                const uint8_t* m = mbase + (mp * static_cast<size_t>(nRows) + static_cast<size_t>(y)) * mstride +
                                   static_cast<size_t>(s);
                const size_t b0 = perBandMask ? mp : 0, b1 = perBandMask ? mp + 1 : nb;
                uint8_t* valid = horizValid + mp * plane + out;
                for (size_t b = b0; b < b1; ++b) acc[b] = ws[b] = 0;
                int run = 0, best = 0;
                for (int j = 0; j < n; ++j) {
                    if (m[j]) {
                        ++run;
                    } else if (run) {
                        best = std::max(best, run);
                        run = 0;
                    }
                    const double wm = w[j] * m[j];
                    const uint8_t* p = p0 + static_cast<size_t>(j) * px;
                    for (size_t b = b0; b < b1; ++b) {
                        const double sv = Load<T>(p + sizeof(T) * b);
                        double wt = wm;
                        if constexpr (std::is_floating_point_v<T>) {
                            if (std::isnan(sv)) wt = 0;
                        }
                        if (wt != 0) acc[b] += sv * wt;
                        ws[b] += wt;
                    }
                }
                if (std::max(best, run) < n / 2) {
                    for (size_t b = b0; b < b1; ++b) horiz[b * plane + out] = 0;
                    *valid = 0;
                    continue;
                }
                // One validity per mask plane: with a shared mask the bands only differ by NaN
                // samples (floating point), which GDAL handles per band as well.
                bool anyValid = false;
                for (size_t b = b0; b < b1; ++b) {
                    if (ws[b] > 0) {
                        horiz[b * plane + out] = acc[b] / ws[b];
                        anyValid = true;
                    } else {
                        horiz[b * plane + out] = 0;
                    }
                }
                *valid = anyValid ? 1 : 0;
                if constexpr (std::is_floating_point_v<T>) {
                    // Per-band validity when NaN samples made one band empty but not another:
                    // fall back to the exact per-band rule for this pixel.
                    if (!perBandMask && anyValid) {
                        for (size_t b = b0; b < b1; ++b)
                            if (!(ws[b] > 0)) horiz[b * plane + out] = std::numeric_limits<double>::quiet_NaN();
                    }
                }
            }
        }
    }
    // Vertical pass.
    for (int y = 0; y < dstH; ++y) {
        const size_t k = static_cast<size_t>(y);
        const int s = rows.start[k] - r0;
        const int n = rows.count[k];
        const double* w = rows.weights.data() + k * static_cast<size_t>(rows.maxTaps);
        const double inv = rows.invSum[k];
        uint8_t* drow = dst + k * dstStride;
        for (int i = 0; i < dstW; ++i) {
            const size_t at0 = static_cast<size_t>(s) * sW + static_cast<size_t>(i);
            uint8_t* dpx = drow + static_cast<size_t>(i) * px;
            if (!masked) {
                for (size_t b = 0; b < nb; ++b) acc[b] = 0.0;
                for (int j = 0; j < n; ++j) {
                    const double wj = w[j] * inv;
                    const size_t at = at0 + static_cast<size_t>(j) * sW;
                    for (size_t b = 0; b < nb; ++b) acc[b] += horiz[b * plane + at] * wj;
                }
                for (size_t b = 0; b < nb; ++b) writer.Value(acc[b], dpx + sizeof(T) * b);
                continue;
            }
            for (size_t mp = 0; mp < maskPlanes; ++mp) {
                const uint8_t* valid = horizValid + mp * plane + at0;
                const size_t b0 = perBandMask ? mp : 0, b1 = perBandMask ? mp + 1 : nb;
                for (size_t b = b0; b < b1; ++b) acc[b] = ws[b] = 0;
                int run = 0, best = 0;
                for (int j = 0; j < n; ++j) {
                    const size_t at = at0 + static_cast<size_t>(j) * sW;
                    const uint8_t vj = valid[static_cast<size_t>(j) * sW];
                    if (vj) {
                        ++run;
                    } else if (run) {
                        best = std::max(best, run);
                        run = 0;
                    }
                    const double wt = w[j] * vj;
                    for (size_t b = b0; b < b1; ++b) {
                        double hv = horiz[b * plane + at];
                        double wb = wt;
                        if constexpr (std::is_floating_point_v<T>) {
                            if (std::isnan(hv)) {  // this band had no valid tap in that row
                                hv = 0;
                                wb = 0;
                            }
                        }
                        acc[b] += hv * wb;
                        ws[b] += wb;
                    }
                }
                if (std::max(best, run) < n / 2) {
                    for (size_t b = b0; b < b1; ++b) writer.NoData(dpx + sizeof(T) * b);
                    continue;
                }
                for (size_t b = b0; b < b1; ++b) {
                    if (ws[b] > 0)
                        writer.Value(acc[b] / ws[b], dpx + sizeof(T) * b);
                    else
                        writer.NoData(dpx + sizeof(T) * b);
                }
            }
        }
    }
}

template <typename T, typename W>
void ReferenceBands(const DownsampleSpec& spec, const uint8_t* src, size_t srcStride, int srcX0, int srcY0, int srcW,
                    int srcH, const uint8_t* srcMask, const Taps& cols, const Taps& rows, int dstW, int dstH,
                    uint8_t* dst, size_t dstStride) {
    switch (spec.bands) {
        case 1: ReferenceT<T, W, 1>(spec, src, srcStride, srcX0, srcY0, srcW, srcH, srcMask, cols, rows, dstW, dstH, dst, dstStride); break;
        case 2: ReferenceT<T, W, 2>(spec, src, srcStride, srcX0, srcY0, srcW, srcH, srcMask, cols, rows, dstW, dstH, dst, dstStride); break;
        case 3: ReferenceT<T, W, 3>(spec, src, srcStride, srcX0, srcY0, srcW, srcH, srcMask, cols, rows, dstW, dstH, dst, dstStride); break;
        case 4: ReferenceT<T, W, 4>(spec, src, srcStride, srcX0, srcY0, srcW, srcH, srcMask, cols, rows, dstW, dstH, dst, dstStride); break;
        default: ReferenceT<T, W, 0>(spec, src, srcStride, srcX0, srcY0, srcW, srcH, srcMask, cols, rows, dstW, dstH, dst, dstStride);
    }
}

// ------------------------------------------------------------------------------------------
// The SIMD implementation (overview_kernels.h): runs of source rows converted to double planes
// go through the horizontal pass, the vertical pass works row by row, and the samples are
// written as the reference writes them.

// Source rows converted at a time: their planes stay in the L2 cache.
constexpr int kRowsPerChunk = 16;

// The per-thread buffers of one Downsample call.
struct Scratch {
    std::vector<double> samples, masks;          // source rows as double planes (HorizontalJob)
    std::vector<int> partial;                    // per row and mask plane, the validity summary
    std::vector<double> horiz, valid;            // the horizontal result, rows padded
    std::vector<double> colWeights, rowWeights;  // normalised weights (raw * invSum)
    std::vector<int> colStart;                   // column taps relative to the source window
    std::vector<double> value, fill;             // one destination row of the vertical pass
    std::vector<uint8_t> groupState;             // per row, column groups all valid / all invalid
};

// raw * invSum per tap, the reference's normalised weights.
void NormalisedWeights(const Taps& t, std::vector<double>& out) {
    kernels::GrowBuffer(out, t.weights.size());
    const size_t m = static_cast<size_t>(t.maxTaps);
    for (size_t k = 0; k < t.invSum.size(); ++k)
        for (size_t j = 0; j < m; ++j) out[k * m + j] = t.weights[k * m + j] * t.invSum[k];
}

// The samples of one interleaved row as one double plane per band (NB > 0: the band count known
// at compile time, so that the compiler vectorizes the loop).
template <typename T, int NB>
void ConvertRow(const uint8_t* row, size_t w, size_t bands, double* out, size_t rowStride) {
    const size_t nb = NB > 0 ? static_cast<size_t>(NB) : bands;
    for (size_t x = 0; x < w; ++x)
        for (size_t b = 0; b < nb; ++b) out[b * rowStride + x] = Load<T>(row + (x * nb + b) * sizeof(T));
}

// The validity summary of a row (HorizontalJob::partial): blocks(x0, x1) tells whether columns
// [x0, x1) hold a mask value other than 255 (first) and one other than 0 (second).
template <typename Blocks>
void SummarizeBlocks(size_t w, int* partial, const Blocks& blocks) {
    const size_t block = kernels::kValidityBlock;
    const size_t n = kernels::ValidityBlocks(w);
    int* notFull = partial;
    int* notEmpty = partial + n + 1;
    notFull[0] = notEmpty[0] = 0;
    for (size_t k = 0; k < n; ++k) {
        const auto [someNotFull, someNotEmpty] = blocks(k * block, std::min(w, (k + 1) * block));
        notFull[k + 1] = notFull[k] + (someNotFull ? 1 : 0);
        notEmpty[k + 1] = notEmpty[k] + (someNotEmpty ? 1 : 0);
    }
}

// Source rows [y, y + rows) of the window (y relative to the window) into the planes the
// horizontal pass reads: the samples of every band, the masks (external: the given mask;
// nodata: 0 / 255 per band) and their validity summaries. The rows are padded with zeros
// (samples and masks) up to rowStride.
template <typename T>
void FillRows(const DownsampleSpec& spec, const uint8_t* src, size_t srcStride, const uint8_t* srcMask, int srcW, int y,
              int rows, size_t rowStride, Scratch& s) {
    const size_t nb = static_cast<size_t>(spec.bands);
    const size_t w = static_cast<size_t>(srcW);
    const bool external = spec.mask == MaskMode::kExternal;
    const bool perBand = spec.mask == MaskMode::kNoData;
    const size_t planes = external ? 1 : perBand ? nb : 0;
    const size_t summary = kernels::ValidityBlocks(w) + 1;
    const double nd = spec.hasNoData ? spec.noData : 0.0;
    const bool ndNan = spec.hasNoData && std::isnan(nd);
    for (size_t r = 0; r < static_cast<size_t>(rows); ++r) {
        const uint8_t* row = src + (static_cast<size_t>(y) + r) * srcStride;
        double* samples = s.samples.data() + r * nb * rowStride;
        switch (nb) {
            case 1: ConvertRow<T, 1>(row, w, nb, samples, rowStride); break;
            case 2: ConvertRow<T, 2>(row, w, nb, samples, rowStride); break;
            case 3: ConvertRow<T, 3>(row, w, nb, samples, rowStride); break;
            case 4: ConvertRow<T, 4>(row, w, nb, samples, rowStride); break;
            default: ConvertRow<T, 0>(row, w, nb, samples, rowStride);
        }
        for (size_t b = 0; b < nb; ++b) std::fill(samples + b * rowStride + w, samples + (b + 1) * rowStride, 0.0);
        for (size_t p = 0; p < planes; ++p) {
            double* m = s.masks.data() + (r * planes + p) * rowStride;
            int* partial = s.partial.data() + (r * planes + p) * 2 * summary;
            if (external) {
                const uint8_t* mrow = srcMask + (static_cast<size_t>(y) + r) * w;
                SummarizeBlocks(w, partial, [&](size_t x0, size_t x1) {
                    uint8_t all = 255, any = 0;
                    for (size_t x = x0; x < x1; ++x) {
                        m[x] = mrow[x];
                        all &= mrow[x];
                        any |= mrow[x];
                    }
                    return std::pair<bool, bool>{all != 255, any != 0};
                });
            } else {
                const double* v = samples + p * rowStride;
                SummarizeBlocks(w, partial, [&](size_t x0, size_t x1) {
                    int noData = 0, data = 0;
                    for (size_t x = x0; x < x1; ++x) {
                        const bool isNd = IsNoData<T>(v[x], nd, ndNan);
                        m[x] = isNd ? 0.0 : 255.0;
                        noData |= isNd ? 1 : 0;
                        data |= isNd ? 0 : 1;
                    }
                    return std::pair<bool, bool>{noData != 0, data != 0};
                });
            }
            std::fill(m + w, m + rowStride, 0.0);
        }
    }
}

template <typename T, typename W>
void DownsampleT(const DownsampleSpec& spec, const uint8_t* src, size_t srcStride, int srcX0, int srcY0, int srcW,
                 int srcH, const uint8_t* srcMask, const Taps& cols, const Taps& rows, int dstW, int dstH, uint8_t* dst,
                 size_t dstStride) {
    const size_t nb = static_cast<size_t>(spec.bands);
    const size_t px = sizeof(T) * nb;
    const bool masked = spec.mask != MaskMode::kNone;
    kernels::MaskLayout layout;
    layout.planes = !masked ? 0 : spec.mask == MaskMode::kNoData ? spec.bands : 1;
    layout.floatSamples = std::is_floating_point_v<T>;
    const size_t maskPlanes = static_cast<size_t>(layout.planes);

    const auto [r0, r1] = TapRows(cols, rows, srcX0, srcY0, srcW, srcH);
    const size_t nRows = static_cast<size_t>(r1 - r0);
    const size_t group = kernels::ColumnGroup();
    const size_t width = (static_cast<size_t>(dstW) + group - 1) / group * group;
    const size_t plane = nRows * width;
    const size_t w = static_cast<size_t>(srcW);
    const size_t rowStride = w + kernels::RowPadding();

    thread_local Scratch s;
    // The padding columns of the horizontal result are computed by the vertical pass and dropped:
    // zero, so that they stay finite.
    double* const horiz = kernels::GrowBuffer(s.horiz, plane * nb);
    double* const valid = kernels::GrowBuffer(s.valid, plane * maskPlanes);
    if (width > static_cast<size_t>(dstW)) {
        for (const auto& [v, n] : {std::pair{horiz, plane * nb}, std::pair{valid, plane * maskPlanes}})
            for (size_t at = static_cast<size_t>(dstW); at < n; at += width)
                std::fill_n(v + at, width - static_cast<size_t>(dstW), 0.0);
    }
    kernels::GrowBuffer(s.samples, static_cast<size_t>(kRowsPerChunk) * nb * rowStride);
    kernels::GrowBuffer(s.masks, static_cast<size_t>(kRowsPerChunk) * maskPlanes * rowStride);
    const size_t summary = 2 * (kernels::ValidityBlocks(w) + 1);
    kernels::GrowBuffer(s.partial, static_cast<size_t>(kRowsPerChunk) * maskPlanes * summary);
    const size_t groups = width / group;
    const size_t states = maskPlanes * nRows * groups;
    std::fill_n(kernels::GrowBuffer(s.groupState, states), states, kernels::kGroupAllValid | kernels::kGroupAllInvalid);
    NormalisedWeights(cols, s.colWeights);
    NormalisedWeights(rows, s.rowWeights);
    kernels::GrowBuffer(s.colStart, static_cast<size_t>(dstW));
    for (size_t k = 0; k < static_cast<size_t>(dstW); ++k) s.colStart[k] = cols.start[k] - srcX0;

    kernels::HorizontalJob h;
    h.bands = spec.bands;
    h.mask = layout;
    h.rows = static_cast<int>(nRows);
    h.chunkRows = kRowsPerChunk;
    struct Rows {
        const DownsampleSpec& spec;
        const uint8_t* src;
        size_t srcStride;
        const uint8_t* srcMask;
        int srcW, top;
        size_t rowStride;
        Scratch& s;
    } rowsSource{spec, src, srcStride, srcMask, srcW, r0 - srcY0, rowStride, s};
    h.fill = [](void* context, int y, int n) {
        const Rows& r = *static_cast<const Rows*>(context);
        FillRows<T>(r.spec, r.src, r.srcStride, r.srcMask, r.srcW, r.top + y, n, r.rowStride, r.s);
    };
    h.fillContext = &rowsSource;
    h.samples = s.samples.data();
    h.masks = s.masks.data();
    h.rowStride = rowStride;
    h.srcW = srcW;
    h.partial = masked ? s.partial.data() : nullptr;
    h.cols = {s.colStart.data(), cols.count.data(), cols.weights.data(), s.colWeights.data(), cols.maxTaps};
    h.dstW = dstW;
    h.horiz = horiz;
    h.valid = valid;
    h.outPlane = plane;
    h.outRow = width;
    h.groupState = masked ? s.groupState.data() : nullptr;
    h.statePlane = nRows * groups;
    h.stateRow = groups;
    kernels::HorizontalPass(h);

    kernels::GrowBuffer(s.value, nb * width);
    kernels::GrowBuffer(s.fill, nb * width);
    kernels::VerticalJob v;
    v.bands = spec.bands;
    v.mask = layout;
    v.horiz = horiz;
    v.valid = valid;
    v.plane = plane;
    v.rowStride = width;
    v.width = static_cast<int>(width);
    v.value = s.value.data();
    v.fill = s.fill.data();
    v.groupState = masked ? s.groupState.data() : nullptr;
    v.statePlane = nRows * groups;
    v.stateRow = groups;
    const SampleWriter<T, W> writer(spec);
    const size_t m = static_cast<size_t>(rows.maxTaps);
    for (size_t k = 0; k < static_cast<size_t>(dstH); ++k) {
        v.start = rows.start[k] - r0;
        v.count = rows.count[k];
        v.weights = rows.weights.data() + k * m;
        v.normalised = s.rowWeights.data() + k * m;
        kernels::VerticalPass(v);
        uint8_t* drow = dst + k * dstStride;
        size_t i = 0;
        if constexpr (std::is_same_v<T, uint8_t>) {
            if (!spec.hasNoData)
                i = kernels::WriteBytes(s.value.data(), masked ? s.fill.data() : nullptr, spec.bands, width,
                                        static_cast<size_t>(dstW), drow);
        }
        for (; i < static_cast<size_t>(dstW); ++i) {
            uint8_t* dpx = drow + i * px;
            for (size_t b = 0; b < nb; ++b) {
                const size_t at = b * width + i;
                if (masked && s.fill[at] != 0)
                    writer.NoData(dpx + sizeof(T) * b);
                else
                    writer.Value(s.value[at], dpx + sizeof(T) * b);
            }
        }
    }
}

DownsampleSpec MaskSpec() {
    DownsampleSpec spec;
    spec.type = tiff::SampleType::kU8;
    spec.bands = 1;
    spec.mask = MaskMode::kExternal;
    return spec;
}

}  // namespace

void Downsample(const DownsampleSpec& spec, const uint8_t* src, size_t srcStride, int srcX0, int srcY0, int srcW,
                int srcH, const uint8_t* srcMask, const Taps& cols, const Taps& rows, int dstW, int dstH, uint8_t* dst,
                size_t dstStride) {
    WithSampleType(spec.type, [&](auto t, auto w) {
        DownsampleT<decltype(t), decltype(w)>(spec, src, srcStride, srcX0, srcY0, srcW, srcH, srcMask, cols, rows,
                                              dstW, dstH, dst, dstStride);
    });
}

void DownsampleMask(const uint8_t* src, int srcX0, int srcY0, int srcW, int srcH, const Taps& cols, const Taps& rows,
                    int dstW, int dstH, uint8_t* dst, size_t dstStride) {
    DownsampleT<uint8_t, float>(MaskSpec(), src, static_cast<size_t>(srcW), srcX0, srcY0, srcW, srcH, src, cols, rows,
                                dstW, dstH, dst, dstStride);
}

namespace reference {

void Downsample(const DownsampleSpec& spec, const uint8_t* src, size_t srcStride, int srcX0, int srcY0, int srcW,
                int srcH, const uint8_t* srcMask, const Taps& cols, const Taps& rows, int dstW, int dstH, uint8_t* dst,
                size_t dstStride) {
    WithSampleType(spec.type, [&](auto t, auto w) {
        ReferenceBands<decltype(t), decltype(w)>(spec, src, srcStride, srcX0, srcY0, srcW, srcH, srcMask, cols, rows,
                                                 dstW, dstH, dst, dstStride);
    });
}

void DownsampleMask(const uint8_t* src, int srcX0, int srcY0, int srcW, int srcH, const Taps& cols, const Taps& rows,
                    int dstW, int dstH, uint8_t* dst, size_t dstStride) {
    ReferenceT<uint8_t, float, 1>(MaskSpec(), src, static_cast<size_t>(srcW), srcX0, srcY0, srcW, srcH, src, cols,
                                  rows, dstW, dstH, dst, dstStride);
}

}  // namespace reference

std::vector<std::pair<uint32_t, uint32_t>> OverviewSizes(uint32_t width, uint32_t height, uint32_t blockSize,
                                                         bool tileMatrix) {
    std::vector<std::pair<uint32_t, uint32_t>> out;
    uint32_t w = width, h = height;
    while (w > blockSize || h > blockSize) {
        if (tileMatrix) {
            w = static_cast<uint32_t>(w / 2.0 + 0.5);
            h = static_cast<uint32_t>(h / 2.0 + 0.5);
        } else {
            w /= 2;
            h /= 2;
        }
        w = std::max(1u, w);
        h = std::max(1u, h);
        out.emplace_back(w, h);
    }
    return out;
}

}  // namespace ddbt::cog::pyramid
