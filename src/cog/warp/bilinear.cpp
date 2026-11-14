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
    if constexpr (std::is_floating_point_v<T>) {
        return static_cast<T>(v);
    } else if constexpr (std::is_signed_v<T>) {
        return static_cast<T>(std::floor(v + 0.5));
    } else {
        return static_cast<T>(v + 0.5);
    }
}

template <typename T>
T ClampRound(double v) {
    if constexpr (std::is_floating_point_v<T>) {
        return static_cast<T>(v);
    } else {
        if (v < static_cast<double>(std::numeric_limits<T>::lowest())) return std::numeric_limits<T>::lowest();
        if (v > static_cast<double>(std::numeric_limits<T>::max())) return std::numeric_limits<T>::max();
        if constexpr (std::is_signed_v<T>) return static_cast<T>(std::floor(v + 0.5));
        return static_cast<T>(v + 0.5);
    }
}

template <typename T>
T AvoidNodata(T v) {
    if constexpr (std::is_integral_v<T>) {
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
    RowKernel(const KernelParams& p, const Window& win)
        : p_(p),
          win_(win),
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
        if (p_.noMasks) {
            // GWKBilinearResampleNoMasks4SampleT.
            const int ix = static_cast<int>(std::floor(fx - 0.5));
            const int iy = static_cast<int>(std::floor(fy - 0.5));
            const double rx = 1.5 - (fx - ix), ry = 1.5 - (fy - iy);
            for (int bi = 0; bi < nOut_; ++bi) {
                const size_t bo = p_.bandOffsets[static_cast<size_t>(bi)];
                T value;
                if (ix >= 0 && ix + 1 < W_ && iy >= 0 && iy + 1 < H_) {
                    const double a = LoadSample<T>(win_.Pixel(ix, iy) + bo), b = LoadSample<T>(win_.Pixel(ix + 1, iy) + bo);
                    const double c = LoadSample<T>(win_.Pixel(ix, iy + 1) + bo);
                    const double d = LoadSample<T>(win_.Pixel(ix + 1, iy + 1) + bo);
                    const double acc = (a * rx + b * (1.0 - rx)) * ry + (c * rx + d * (1.0 - rx)) * (1.0 - ry);
                    value = RoundNoMask<T>(acc);
                } else {
                    double div = 0, acc = 0;
                    if (ix >= 0 && ix < W_ && iy >= 0 && iy < H_) {
                        const double m = rx * ry;
                        div += m;
                        acc += LoadSample<T>(win_.Pixel(ix, iy) + bo) * m;
                    }
                    if (ix + 1 >= 0 && ix + 1 < W_ && iy >= 0 && iy < H_) {
                        const double m = (1.0 - rx) * ry;
                        div += m;
                        acc += LoadSample<T>(win_.Pixel(ix + 1, iy) + bo) * m;
                    }
                    if (ix + 1 >= 0 && ix + 1 < W_ && iy + 1 >= 0 && iy + 1 < H_) {
                        const double m = (1.0 - rx) * (1.0 - ry);
                        div += m;
                        acc += LoadSample<T>(win_.Pixel(ix + 1, iy + 1) + bo) * m;
                    }
                    if (ix >= 0 && ix < W_ && iy + 1 >= 0 && iy + 1 < H_) {
                        const double m = rx * (1.0 - ry);
                        div += m;
                        acc += LoadSample<T>(win_.Pixel(ix, iy + 1) + bo) * m;
                    }
                    if (div < 0.00001) {
                        value = T(0);
                    } else {
                        value = RoundNoMask<T>(div == 1.0 ? acc : acc / div);
                    }
                }
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
            // GWKSetPixelValue.
            if (dens < 0.9999) {
                if (dens < 0.0001) continue;
                T cur;
                std::memcpy(&cur, out + static_cast<size_t>(bi) * ss, ss);
                const double dstReal = static_cast<double>(cur);
                const double influence = (1.0 - dens) * p_.dstDensityInit;
                value = (value * dens + dstReal * influence) / (dens + influence);
            }
            T t = ClampRound<T>(value);
            if (p_.avoidSingle && static_cast<double>(t) == dstNd_) t = AvoidNodata<T>(t);
            std::memcpy(out + static_cast<size_t>(bi) * ss, &t, ss);
        }
        if (!found) return;
        Finish(centreDensity, out, mask);
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
                v = ClampRound<T>((static_cast<double>(v) * density + static_cast<double>(cur) * influence) /
                                  (density + influence));
            }
            if (p_.avoidSingle && static_cast<double>(v) == dstNd_) v = AvoidNodata<T>(v);
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
                if (static_cast<double>(t) == dstNd_) {
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
                all = static_cast<double>(t) == dstNd_;
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
        } else if constexpr (!std::is_floating_point_v<T>) {
            const T a = static_cast<T>(static_cast<int>(dd * (p_.dstAlphaMax + 0.1f)));
            std::memcpy(out + static_cast<size_t>(p_.alphaOut) * ss, &a, ss);
        } else {
            const T a = static_cast<T>(dd * p_.dstAlphaMax);
            std::memcpy(out + static_cast<size_t>(p_.alphaOut) * ss, &a, ss);
        }
    }

    const KernelParams& p_;
    const Window& win_;
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
              uint8_t* mask, KernelPath path) {
    const RowKernel<T> k(p, win);
    const size_t opb = p.outPixelBytes;
    const bool alphaMask = p.alphaOut == -1;
    if (p.nearest) {
        for (size_t i = 0; i < n; ++i) k.Nearest(fx[i], fy[i], out + i * opb, alphaMask ? mask + i : nullptr);
        return;
    }
    if (path == KernelPath::kScalar || !SimdApplies(p, win)) {
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
             uint8_t* mask, KernelPath path) {
    VisitSampleType(p.type, [&]<typename T>() { WarpRowT<T>(p, win, fx, fy, n, out, mask, path); });
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
