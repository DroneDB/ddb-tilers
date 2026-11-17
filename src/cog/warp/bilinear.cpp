// SPDX-License-Identifier: MPL-2.0
//
// The scalar bilinear warp kernel (the reference) and the row driver that sends the pixels with
// four fully valid taps to the SIMD kernel. See bilinear.h.
#include "cog/warp/bilinear.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include "cog/warp/warp_kernels.h"

namespace ddbt::cog::warp {

namespace {

constexpr double kSrcDensityThreshold = 0.000000001;
constexpr double kBandDensityThreshold = 0.0000000001;

template <typename T>
T RoundNoMask(double v) {
    if constexpr (kRealSample<T>) {
        return FromDouble<T>(v);
    } else if constexpr (std::is_signed_v<T>) {
        return static_cast<T>(std::floor(v + 0.5));
    } else {
        return static_cast<T>(v + 0.5);
    }
}

template <typename T>
T ClampRound(double v) {
    if constexpr (kRealSample<T>) {
        return FromDouble<T>(v);
    } else {
        if (v < static_cast<double>(std::numeric_limits<T>::lowest())) return std::numeric_limits<T>::lowest();
        if (v > static_cast<double>(std::numeric_limits<T>::max())) return std::numeric_limits<T>::max();
        if constexpr (std::is_signed_v<T>) return static_cast<T>(std::floor(v + 0.5));
        return static_cast<T>(v + 0.5);
    }
}

template <typename T>
T AvoidNodata(T v) {
    if constexpr (std::is_same_v<T, util::Half>) {
        // GDAL's nextafter of a GFloat16: down from its largest value, else up towards it
        return util::Half{util::HalfNextAfter(v.bits, v.bits == util::kHalfMaxBits ? uint16_t{0} : util::kHalfMaxBits)};
    } else if constexpr (std::is_integral_v<T>) {
        if (v == std::numeric_limits<T>::lowest()) return static_cast<T>(std::numeric_limits<T>::lowest() + 1);
        return static_cast<T>(v - 1);
    } else {
        if (v == std::numeric_limits<T>::max()) return std::nextafter(v, T(0));
        return std::nextafter(v, std::numeric_limits<T>::max());
    }
}

// One row of destination pixels of one output, sample type T.
template <typename T>
class RowKernel {
public:
    RowKernel(const KernelParams& p, const Window& win, const ChunkKernel* chunk = nullptr)
        : p_(p),
          win_(win),
          general_(chunk != nullptr && !chunk->fourSamples ? chunk : nullptr),
          W_(p.sourceWidth),
          H_(p.sourceHeight),
          nOut_(static_cast<int>(p.bandOffsets.size())),
          hasValid_(!win.valid.empty()),
          hasDensity_(!win.density.empty()),
          perBand_(p.perBandNodata.has_value()),
          nodata_(p.perBandNodata.value_or(0)),
          dstNd_(p.dstNodata.value_or(0)) {}

    // The scalar kernel: every case of GDAL's, one pixel.
    void Pixel(double fx, double fy, uint8_t* out, uint8_t* mask) const {
        constexpr size_t ss = sizeof(T);
        if (std::isnan(fx)) return;
        if (p_.noMasks && general_ != nullptr) {
            // GWKResampleNoMasks(_SSE2)_T: the weights once per pixel, then every band
            for (int bi = 0; bi < nOut_; ++bi) {
                const T value = GeneralNoMask(fx, fy, p_.bandOffsets[static_cast<size_t>(bi)], weights_, bi == 0);
                std::memcpy(out + static_cast<size_t>(bi) * ss, &value, ss);
            }
            Finish(1.0, out, mask);
            return;
        }
        if (p_.noMasks) {
            // GWKBilinearResampleNoMasks4SampleT.
            for (int bi = 0; bi < nOut_; ++bi) {
                const T value = FourSampleNoMask(fx, fy, p_.bandOffsets[static_cast<size_t>(bi)], 0, 0, W_, H_);
                std::memcpy(out + static_cast<size_t>(bi) * ss, &value, ss);
            }
            // A written pixel has a destination density of 1.
            Finish(1.0, out, mask);
            return;
        }
        // General case: the centre pixel first.
        int cx = static_cast<int>(fx + 1.0e-10), cy = static_cast<int>(fy + 1.0e-10);
        if (cx == W_) --cx;
        if (cy == H_) --cy;
        double centreDensity = 1.0;
        if (hasDensity_) {
            centreDensity = static_cast<double>(win_.density[win_.Index(cx, cy)]);
            if (centreDensity < kSrcDensityThreshold) return;
        }
        if (hasValid_ && !win_.valid[win_.Index(cx, cy)]) return;
        int ix = static_cast<int>(std::floor(fx - 0.5));
        int iy = static_cast<int>(std::floor(fy - 0.5));
        double rx = 1.5 - (fx - ix), ry = 1.5 - (fy - iy);
        if (ix == -1) {
            ix = 0;
            rx = 1;
        }
        if (iy == -1) {
            iy = 0;
            ry = 1;
        }
        // The four taps (GWKBilinearResample4Sample): weights and densities once per pixel; per
        // band only the values (and, with per-band nodata, the densities).
        const double mul[4] = {rx * ry, (1.0 - rx) * ry, rx * (1.0 - ry), (1.0 - rx) * (1.0 - ry)};
        const int tapX[4] = {ix, ix + 1, ix, ix + 1};
        const int tapY[4] = {iy, iy, iy + 1, iy + 1};
        bool inside[4];
        double tapD[4];
        const uint8_t* tapP[4];
        for (int t = 0; t < 4; ++t) {
            inside[t] = tapX[t] >= 0 && tapX[t] < W_ && tapY[t] >= 0 && tapY[t] < H_;
            tapP[t] = inside[t] ? win_.Pixel(tapX[t], tapY[t]) : nullptr;
            if (inside[t]) {
                const size_t wi = win_.Index(tapX[t], tapY[t]);
                double d = (hasValid_ && !win_.valid[wi]) ? 0.0 : 1.0;
                if (hasDensity_ && d > kSrcDensityThreshold) d = static_cast<double>(win_.density[wi]);
                tapD[t] = d;
            } else {
                tapD[t] = 0.0;
            }
        }
        bool found = false;
        for (int bi = 0; bi < nOut_; ++bi) {
            const size_t bo = p_.bandOffsets[static_cast<size_t>(bi)];
            double acc = 0, accD = 0, div = 0;
            if (general_ != nullptr) {
                // GWKResample: the stretched filter over the taps of density, weighted
                double value = 0, dens = 0;
                if (!GeneralMasked(fx, fy, bo, value, dens) || dens < kBandDensityThreshold) continue;
                found = true;
                Write(bi, value, dens, out);
                continue;
            }
            for (int t = 0; t < 4; ++t) {
                if (!inside[t]) continue;
                double d = tapD[t];
                if (perBand_ && d > kSrcDensityThreshold && IsNodata<T>(tapP[t] + bo, nodata_))
                    d = 0.0;
                if (!(d > kSrcDensityThreshold)) continue;
                div += mul[t];
                acc += LoadSample<T>(tapP[t] + bo) * mul[t];
                accD += d * mul[t];
            }
            double value, dens;
            if (div == 1.0) {
                value = acc;
                dens = accD;
            } else if (div < 0.00001) {
                value = 0;
                dens = 0;
            } else {
                value = acc / div;
                dens = accD / div;
            }
            if (dens < kBandDensityThreshold) continue;
            found = true;
            Write(bi, value, dens, out);
        }
        if (!found) return;
        Finish(centreDensity, out, mask);
    }

    // GWKSetPixelValue: the value blended into the destination by its density, clamped and rounded,
    // moved off the destination nodata; nothing under a density of 0.0001.
    void Write(int bi, double value, double dens, uint8_t* out) const {
        constexpr size_t ss = sizeof(T);
        if (dens < 0.9999) {
            if (dens < 0.0001) return;
            T cur;
            std::memcpy(&cur, out + static_cast<size_t>(bi) * ss, ss);
            const double dstReal = ToDouble(cur);
            const double influence = (1.0 - dens) * p_.dstDensityInit;
            value = (value * dens + dstReal * influence) / (dens + influence);
        }
        T t = ClampRound<T>(value);
        if (p_.avoidSingle && ToDouble(t) == dstNd_) t = AvoidNodata<T>(t);
        std::memcpy(out + static_cast<size_t>(bi) * ss, &t, ss);
    }

    // The weights GWKComputeWeights computes for a pixel (at its first band)
    struct Weights {
        std::vector<double> h, v;
        double inv = 0;
    };

    static double Bilinear(double x) {
        const double a = std::fabs(x);
        return a <= 1.0 ? 1 - a : 0.0;
    }

    // GWKComputeWeights for bilinear: groups of 4 (each from the one before plus the scale, summed
    // in order), then one at a time; the sums start at the smallest normal double
    static void ComputeWeights(int iMin, int iMax, double dX, double xs, int jMin, int jMax, double dY, double ys,
                               Weights& w) {
        w.h.assign(static_cast<size_t>(iMax - iMin + 4), 0.0);
        w.v.assign(static_cast<size_t>(jMax - jMin + 4), 0.0);
        const auto axis = [](int lo, int hi, double d, double scale, std::vector<double>& out) {
            double sum = std::numeric_limits<double>::min();
            int i = lo, c = 0;
            for (; i + 2 < hi; i += 4, c += 4) {
                const size_t k = static_cast<size_t>(c);
                out[k] = (i - d) * scale;
                out[k + 1] = out[k] + scale;
                out[k + 2] = out[k + 1] + scale;
                out[k + 3] = out[k + 2] + scale;
                for (size_t q = 0; q < 4; ++q) out[k + q] = Bilinear(out[k + q]);
                sum += out[k] + out[k + 1] + out[k + 2] + out[k + 3];
            }
            for (; i <= hi; ++i, ++c) {
                const double weight = Bilinear((i - d) * scale);
                out[static_cast<size_t>(c)] = weight;
                sum += weight;
            }
            return sum;
        };
        const double sh = axis(iMin, iMax, dX, xs, w.h);
        const double sv = axis(jMin, jMax, dY, ys, w.v);
        w.inv = 1. / (sh * sv);
    }

    // GWKClampValueT
    static T ClampValue(double v) {
        if constexpr (kRealSample<T>) {
            return FromDouble<T>(v);
        } else {
            if (v < static_cast<double>(std::numeric_limits<T>::min())) return std::numeric_limits<T>::min();
            if (v > static_cast<double>(std::numeric_limits<T>::max())) return std::numeric_limits<T>::max();
            if constexpr (std::is_signed_v<T>) return static_cast<T>(std::floor(v + 0.5));
            return static_cast<T>(v + 0.5);
        }
    }

    // A sample of the chunk's source window (x, y relative to it)
    double At(int x, int y, size_t bo) const {
        return LoadSample<T>(win_.Pixel(general_->srcX0 + x, general_->srcY0 + y) + bo);
    }

    // GWKResampleNoMasks_SSE2_T (the kernel of every type without masks: 4 lanes as two SSE2 pairs,
    // summed (lane 0 + lane 2) + (lane 1 + lane 3); 4 rows at a time), in the chunk's source window;
    // the 2 x 2 formula where GDAL falls back to it
    T GeneralNoMask(double fxAbs, double fyAbs, size_t bo, Weights& w, bool first) const {
        const ChunkKernel& k = *general_;
        const double fx = fxAbs - k.srcX0, fy = fyAbs - k.srcY0;
        const int iSrcX = static_cast<int>(std::floor(fx - 0.5));
        const int iSrcY = static_cast<int>(std::floor(fy - 0.5));
        if (iSrcX >= k.srcW || iSrcY >= k.srcH || k.xRadius > k.srcW || k.yRadius > k.srcH)
            return FourSampleNoMask(fx, fy, bo, k.srcX0, k.srcY0, k.srcW, k.srcH);
        const double dX = fx - 0.5 - iSrcX, dY = fy - 0.5 - iSrcY;
        const double xs = std::min(k.xScale, 1.0), ys = std::min(k.yScale, 1.0);
        int iMin = 1 - k.xRadius;
        if (iSrcX + iMin < 0) iMin = -iSrcX;
        int iMax = k.xRadius;
        if (iSrcX + iMax >= k.srcW - 1) iMax = k.srcW - 1 - iSrcX;
        int jMin = 1 - k.yRadius;
        if (iSrcY + jMin < 0) jMin = -iSrcY;
        int jMax = k.yRadius;
        if (iSrcY + jMax >= k.srcH - 1) jMax = k.srcH - 1 - iSrcY;
        if (first) ComputeWeights(iMin, iMax, dX, xs, jMin, jMax, dY, ys, w);
        const auto px = [&](int i, int j) { return At(iSrcX + i, iSrcY + j, bo); };
        double acc = 0.0;
        {
            int j = jMin, jC = 0;
            for (; j + 2 < jMax; j += 4, jC += 4) {
                double a[4][4] = {};
                int i = iMin, iC = 0;
                for (; i + 2 < iMax; i += 4, iC += 4)
                    for (int r = 0; r < 4; ++r)
                        for (int l = 0; l < 4; ++l)
                            a[r][l] += px(i + l, j + r) * w.h[static_cast<size_t>(iC + l)];
                if (i < iMax) {
                    for (int r = 0; r < 4; ++r) {
                        a[r][0] += px(i, j + r) * w.h[static_cast<size_t>(iC)];
                        a[r][1] += px(i + 1, j + r) * w.h[static_cast<size_t>(iC + 1)];
                    }
                    i += 2;
                    iC += 2;
                }
                double local[4];
                for (int r = 0; r < 4; ++r) local[r] = (a[r][0] + a[r][2]) + (a[r][1] + a[r][3]);
                if (i == iMax)
                    for (int r = 0; r < 4; ++r) local[r] += px(i, j + r) * w.h[static_cast<size_t>(iC)];
                for (int r = 0; r < 4; ++r) acc += w.v[static_cast<size_t>(jC + r)] * local[r];
            }
            for (; j <= jMax; ++j, ++jC) {
                double a[4] = {};
                int i = iMin, iC = 0;
                for (; i + 2 < iMax; i += 4, iC += 4)
                    for (int l = 0; l < 4; ++l) a[l] += px(i + l, j) * w.h[static_cast<size_t>(iC + l)];
                double local = (a[0] + a[2]) + (a[1] + a[3]);
                if (i < iMax) {
                    local += px(i, j) * w.h[static_cast<size_t>(iC)];
                    local += px(i + 1, j) * w.h[static_cast<size_t>(iC + 1)];
                    i += 2;
                    iC += 2;
                }
                if (i == iMax) local += px(i, j) * w.h[static_cast<size_t>(iC)];
                acc += w.v[static_cast<size_t>(jC)] * local;
            }
        }
        return ClampValue(acc * w.inv);
    }

    // GWKBilinearResampleNoMasks4SampleT in the source window of w x h pixels at (x0, y0), fx / fy
    // relative to it: the 4 taps, or those of them in the window weighted by their share
    T FourSampleNoMask(double fx, double fy, size_t bo, int x0, int y0, int w, int h) const {
        const int ix = static_cast<int>(std::floor(fx - 0.5));
        const int iy = static_cast<int>(std::floor(fy - 0.5));
        const double rx = 1.5 - (fx - ix), ry = 1.5 - (fy - iy);
        const auto at = [&](int x, int y) { return LoadSample<T>(win_.Pixel(x0 + x, y0 + y) + bo); };
        if (ix >= 0 && ix + 1 < w && iy >= 0 && iy + 1 < h) {
            const double acc = (at(ix, iy) * rx + at(ix + 1, iy) * (1.0 - rx)) * ry +
                               (at(ix, iy + 1) * rx + at(ix + 1, iy + 1) * (1.0 - rx)) * (1.0 - ry);
            return RoundNoMask<T>(acc);
        }
        double div = 0, acc = 0;
        const auto tap = [&](int x, int y, double m) {
            if (x >= 0 && x < w && y >= 0 && y < h) {
                div += m;
                acc += at(x, y) * m;
            }
        };
        tap(ix, iy, rx * ry);
        tap(ix + 1, iy, (1.0 - rx) * ry);
        tap(ix + 1, iy + 1, (1.0 - rx) * (1.0 - ry));
        tap(ix, iy + 1, rx * (1.0 - ry));
        if (div < 0.00001) return T(0);
        return RoundNoMask<T>(div == 1.0 ? acc : acc / div);
    }

    // GWKResample (bilinear) in the chunk's source window: the taps from -radius to radius, clipped
    // at its edges, a row skipped when none of its taps is valid (GWKGetPixelRow), a tap of density
    // under 1e-9 skipped, the X and Y weights of the stretched filter, the sums of the values, the
    // densities and the weights; no value under a total weight or density of 1e-6
    bool GeneralMasked(double fxAbs, double fyAbs, size_t bo, double& value, double& density) const {
        const ChunkKernel& k = *general_;
        const double fx = fxAbs - k.srcX0, fy = fyAbs - k.srcY0;
        const int iSrcX = static_cast<int>(std::floor(fx - 0.5));
        const int iSrcY = static_cast<int>(std::floor(fy - 0.5));
        const double dX = fx - 0.5 - iSrcX, dY = fy - 0.5 - iSrcY;
        const bool xBelow = k.xScale < 1.0, yBelow = k.yScale < 1.0;
        int j = -k.yRadius, jMax = k.yRadius;
        if (iSrcY + j < 0) j = -iSrcY;
        if (iSrcY + jMax >= k.srcH) jMax = k.srcH - iSrcY - 1;
        int iMin = -k.xRadius, iMax = k.xRadius;
        if (iSrcX + iMin < 0) iMin = -iSrcX;
        if (iSrcX + iMax >= k.srcW) iMax = k.srcW - iSrcX - 1;
        double accR = 0, accD = 0, accW = 0;
        const bool densities = hasValid_ || hasDensity_ || perBand_;
        // The X weights of the pixel (GDAL computes each once, on its first tap of density)
        std::vector<double>& xw = weights_.h;
        xw.resize(static_cast<size_t>(std::max(0, iMax - iMin + 1)));
        for (int i = iMin; i <= iMax; ++i)
            xw[static_cast<size_t>(i - iMin)] = xBelow ? Bilinear((i - dX) * k.xScale) : Bilinear(i - dX);
        for (; j <= jMax; ++j) {
            const int y = iSrcY + j;
            const double w1 = yBelow ? Bilinear((j - dY) * k.yScale) : Bilinear(j - dY);
            double lr = 0, ld = 0, lw = 0;
            for (int i = iMin; i <= iMax; ++i) {
                const int x = iSrcX + i;
                double d = 1.0;
                if (densities) {
                    const size_t wi = win_.Index(k.srcX0 + x, k.srcY0 + y);
                    if (hasValid_ && !win_.valid[wi]) d = 0.0;
                    if (perBand_ && d > 0.0 && IsNodata<T>(win_.Pixel(k.srcX0 + x, k.srcY0 + y) + bo, nodata_)) d = 0.0;
                    if (d > kSrcDensityThreshold && hasDensity_) d = static_cast<double>(win_.density[wi]);
                    if (d < kSrcDensityThreshold) continue;
                }
                const double w2 = xw[static_cast<size_t>(i - iMin)];
                lr += At(x, y, bo) * w2;
                if (densities) ld += d * w2;
                lw += w2;
            }
            accR += lr * w1;
            accD += ld * w1;
            accW += lw * w1;
        }
        if (accW < 0.000001 || (densities && accD < 0.000001)) {
            density = 0.0;
            return false;
        }
        if (accW < 0.99999 || accW > 1.00001) {
            value = accR / accW;
            density = densities ? accD / accW : 1.0;
        } else {
            value = accR;
            density = densities ? accD : 1.0;
        }
        return true;
    }

    // GDAL's nearest neighbour kernel, one pixel.
    void Nearest(double fx, double fy, uint8_t* out, uint8_t* mask) const {
        constexpr size_t ss = sizeof(T);
        if (std::isnan(fx)) return;
        int cx = static_cast<int>(fx + 1.0e-10), cy = static_cast<int>(fy + 1.0e-10);
        if (cx == W_) --cx;
        if (cy == H_) --cy;
        const uint8_t* px = win_.Pixel(cx, cy);
        if (p_.noMasks) {
            // GWKResampleNoMasksOrDstDensityOnlyThread: the samples, a destination density of 1.
            for (int bi = 0; bi < nOut_; ++bi)
                std::memcpy(out + static_cast<size_t>(bi) * ss, px + p_.bandOffsets[static_cast<size_t>(bi)], ss);
            Finish(1.0, out, mask);
            return;
        }
        // GWKNearestThread.
        const size_t wi = win_.Index(cx, cy);
        if (hasValid_ && !win_.valid[wi]) return;
        double density = 1.0;
        if (hasDensity_) {
            density = static_cast<double>(win_.density[wi]);
            if (density < kSrcDensityThreshold) return;
        }
        for (int bi = 0; bi < nOut_; ++bi) {
            const uint8_t* sample = px + p_.bandOffsets[static_cast<size_t>(bi)];
            // GWKGetPixelT: the band's own nodata, then the density.
            if (perBand_ && IsNodata<T>(sample, nodata_)) continue;
            if (density == 0.0) continue;
            T v;
            std::memcpy(&v, sample, ss);
            // GWKSetPixelValueRealT.
            if (density < 0.9999) {
                if (density < 0.0001) continue;
                T cur;
                std::memcpy(&cur, out + static_cast<size_t>(bi) * ss, ss);
                const double influence = (1.0 - density) * p_.dstDensityInit;
                v = ClampRound<T>((ToDouble(v) * density + ToDouble(cur) * influence) / (density + influence));
            }
            if (p_.avoidSingle && ToDouble(v) == dstNd_) v = AvoidNodata<T>(v);
            std::memcpy(out + static_cast<size_t>(bi) * ss, &v, ss);
        }
        Finish(density, out, mask);
    }

    // What follows the bands of a pixel the SIMD kernel wrote. Its four taps were fully valid,
    // so its centre pixel (one of them) has density 1, and every band was set: each band's
    // ClampRoundAndAvoidNoData, then the rest of Pixel().
    void FinishWritten(uint8_t* out, uint8_t* mask) const {
        constexpr size_t ss = sizeof(T);
        if (p_.avoidSingle) {
            for (int bi = 0; bi < nOut_; ++bi) {
                T t;
                std::memcpy(&t, out + static_cast<size_t>(bi) * ss, ss);
                if (ToDouble(t) == dstNd_) {
                    t = AvoidNodata<T>(t);
                    std::memcpy(out + static_cast<size_t>(bi) * ss, &t, ss);
                }
            }
        }
        Finish(1.0, out, mask);
    }

    // The output mask of a pixel whose centre has density 1 (what Finish writes for it).
    uint8_t FullDensityMask() const {
        const float dd = 1.0f - (1.0f - 1.0f) * (1.0f - 0.0f);
        return static_cast<uint8_t>(static_cast<int>(dd * 255.1f));
    }

private:
    // GWKAvoidNoDataMultiBand, then the destination alpha (GWKOverlayDensity over a destination
    // density of 0, GDALWarpDstAlphaMasker).
    void Finish(double centreDensity, uint8_t* out, uint8_t* mask) const {
        constexpr size_t ss = sizeof(T);
        if (p_.avoidMulti) {
            bool all = true;
            for (int bi = 0; bi < nOut_ && all; ++bi) {
                T t;
                std::memcpy(&t, out + static_cast<size_t>(bi) * ss, ss);
                all = ToDouble(t) == dstNd_;
            }
            if (all) {
                for (int bi = 0; bi < nOut_; ++bi) {
                    T t;
                    std::memcpy(&t, out + static_cast<size_t>(bi) * ss, ss);
                    t = AvoidNodata<T>(t);
                    std::memcpy(out + static_cast<size_t>(bi) * ss, &t, ss);
                }
            }
        }
        if (p_.alphaOut == -2) return;
        if (centreDensity < 0.0001) return;
        const float dd = 1.0f - (1.0f - static_cast<float>(centreDensity)) * (1.0f - 0.0f);
        if (p_.alphaOut == -1) {
            *mask = static_cast<uint8_t>(static_cast<int>(dd * 255.1f));
        } else if constexpr (!kRealSample<T>) {
            const T a = static_cast<T>(static_cast<int>(dd * (p_.dstAlphaMax + 0.1f)));
            std::memcpy(out + static_cast<size_t>(p_.alphaOut) * ss, &a, ss);
        } else {
            const T a = FromDouble<T>(static_cast<double>(dd * p_.dstAlphaMax));
            std::memcpy(out + static_cast<size_t>(p_.alphaOut) * ss, &a, ss);
        }
    }

    const KernelParams& p_;
    const Window& win_;
    const ChunkKernel* general_;
    mutable Weights weights_;  // the filter weights of the pixel being resampled (buffers reused)
    const int W_, H_, nOut_;
    const bool hasValid_, hasDensity_, perBand_;
    const double nodata_, dstNd_;
};

// The SIMD kernel reads the window through int32 byte offsets and whole 32-bit or 64-bit words,
// and, with masks, through the window's flags.
bool SimdApplies(const KernelParams& p, const Window& win) {
    if (p.perBandNodata) return false;
    const size_t bytes = win.Pixels() * win.pb;
    if (bytes > static_cast<size_t>(std::numeric_limits<int32_t>::max()) - 64) return false;
    if (win.px.size() < bytes + kWindowPadding) return false;
    return p.noMasks || win.flags.size() >= win.Pixels() + kWindowPadding;
}

template <typename T>
void WarpRowT(const KernelParams& p, const Window& win, const double* fx, const double* fy, size_t n, uint8_t* out,
              uint8_t* mask, KernelPath path, const ChunkKernel* chunk) {
    const RowKernel<T> k(p, win, chunk);
    const size_t opb = p.outPixelBytes;
    const bool alphaMask = p.alphaOut == -1;
    if (p.nearest) {
        for (size_t i = 0; i < n; ++i) k.Nearest(fx[i], fy[i], out + i * opb, alphaMask ? mask + i : nullptr);
        return;
    }
    const bool general = chunk != nullptr && !chunk->fourSamples;
    if (general || path == KernelPath::kScalar || !SimdApplies(p, win)) {
        for (size_t i = 0; i < n; ++i) k.Pixel(fx[i], fy[i], out + i * opb, alphaMask ? mask + i : nullptr);
        return;
    }
    constexpr size_t kChunk = 256;
    uint8_t status[kChunk];
    kernels::BilinearJob job;
    job.type = p.type;
    job.px = win.px.data();
    job.pixelBytes = win.pb;
    job.flags = win.flags.empty() ? nullptr : win.flags.data();
    job.winX0 = win.x0;
    job.winY0 = win.y0;
    job.winW = win.w;
    job.winH = win.h;
    job.sourceWidth = p.sourceWidth;
    job.sourceHeight = p.sourceHeight;
    job.noMasks = p.noMasks;
    job.bandOffsets = p.bandOffsets.data();
    job.bands = p.bandOffsets.size();
    job.outPixelBytes = opb;
    job.status = status;
    if (alphaMask) job.maskValue = k.FullDensityMask();
    // What the kernel leaves to do on the pixels it wrote: the nodata avoidance, an alpha band.
    const bool finishWritten = p.avoidSingle || p.avoidMulti || p.alphaOut >= 0;
    for (size_t c0 = 0; c0 < n; c0 += kChunk) {
        const size_t m = std::min(kChunk, n - c0);
        job.fx = fx + c0;
        job.fy = fy + c0;
        job.n = m;
        job.out = out + c0 * opb;
        job.mask = alphaMask ? mask + c0 : nullptr;
        kernels::BilinearRow(job);
        if (finishWritten) {
            for (size_t i = 0; i < m; ++i) {
                const size_t at = c0 + i;
                if (status[i] == kernels::kScalar) {
                    k.Pixel(fx[at], fy[at], out + at * opb, alphaMask ? mask + at : nullptr);
                } else if (status[i] == kernels::kWritten) {
                    k.FinishWritten(out + at * opb, alphaMask ? mask + at : nullptr);
                }
            }
            continue;
        }
        // Only the pixels left to the scalar kernel (usually none, or a few at the edges).
        const uint8_t* const end = status + m;
        for (const uint8_t* q = status; (q = static_cast<const uint8_t*>(std::memchr(q, kernels::kScalar,
                                                                                      static_cast<size_t>(end - q)))) != nullptr;
             ++q) {
            const size_t at = c0 + static_cast<size_t>(q - status);
            k.Pixel(fx[at], fy[at], out + at * opb, alphaMask ? mask + at : nullptr);
        }
    }
}

}  // namespace

void WarpRow(const KernelParams& p, const Window& win, const double* fx, const double* fy, size_t n, uint8_t* out,
             uint8_t* mask, KernelPath path, const ChunkKernel* chunk) {
    VisitSampleType(p.type, [&]<typename T>() { WarpRowT<T>(p, win, fx, fy, n, out, mask, path, chunk); });
}

void UnifiedValidity(tiff::SampleType type, const uint8_t* pixels, size_t pixelBytes, size_t count,
                     const std::vector<size_t>& bandOffsets, double nodata, uint8_t valid, uint8_t* out) {
    VisitSampleType(type, [&]<typename T>() {
        for (size_t i = 0; i < count; ++i) {
            const uint8_t* const pixel = pixels + i * pixelBytes;
            out[i] = std::any_of(bandOffsets.begin(), bandOffsets.end(),
                                 [&](size_t o) { return !IsNodata<T>(pixel + o, nodata); })
                         ? valid
                         : 0;
        }
    });
}

void IndexValidity(Window& win) {
    if (win.valid.empty() && win.density.empty()) {
        win.flags.clear();
        return;
    }
    using kernels::WindowFlags::kCentreValid;
    using kernels::WindowFlags::kQuadFull;
    const size_t np = win.Pixels(), w = static_cast<size_t>(win.w), h = static_cast<size_t>(win.h);
    win.flags.resize(np + kWindowPadding);
    if (np == 0) return;
    uint8_t* const f = win.flags.data();
    const uint8_t* const valid = win.valid.empty() ? nullptr : win.valid.data();
    const float* const density = win.density.empty() ? nullptr : win.density.data();
    // Each pixel on its own: a valid centre (the scalar kernel's test: density >= 1e-9 as double,
    // then the validity), and a fully valid tap (validity 1, density 1; kQuadFull for now).
    for (size_t i = 0; i < np; ++i) {
        const bool v = valid == nullptr || valid[i] != 0;
        const float d = density == nullptr ? 1.0f : density[i];
        const bool centre = v && !(static_cast<double>(d) < kSrcDensityThreshold);
        const bool full = v && d == 1.0f;
        f[i] = static_cast<uint8_t>((centre ? kCentreValid : 0) | (full ? kQuadFull : 0));
    }
    // Then the 2 x 2 block starting at each pixel, in place: a pixel reads itself and pixels after
    // it, which still hold their own flags. Taps outside the window are invalid. Branch-free rows
    // (the compiler vectorizes them), the last column and row apart.
    const auto block = [](uint8_t t0, uint8_t t1, uint8_t t2, uint8_t t3) {
        return static_cast<uint8_t>((t0 & kCentreValid) | (t1 & kCentreValid) << 1 | (t2 & kCentreValid) << 2 |
                                    (t3 & kCentreValid) << 3 | (t0 & t1 & t2 & t3 & kQuadFull));
    };
    for (size_t y = 0; y + 1 < h; ++y) {
        uint8_t* const row = f + y * w;
        const uint8_t* const below = row + w;
        for (size_t x = 0; x + 1 < w; ++x) row[x] = block(row[x], row[x + 1], below[x], below[x + 1]);
        row[w - 1] = block(row[w - 1], 0, below[w - 1], 0);
    }
    uint8_t* const last = f + (h - 1) * w;
    for (size_t x = 0; x + 1 < w; ++x) last[x] = block(last[x], last[x + 1], 0, 0);
    last[w - 1] = block(last[w - 1], 0, 0, 0);
}

}  // namespace ddbt::cog::warp
