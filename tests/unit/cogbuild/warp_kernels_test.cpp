// SPDX-License-Identifier: MPL-2.0
//
// The SIMD warp kernels against their scalar references, bit for bit, on every Highway target
// the CPU supports: GDAL's bilinear kernel (every sample type, every mask configuration, the
// source edges, odd row lengths), the source pixel mapping and the tap bounds.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "cog/warp/bilinear.h"
#include "cog/warp/warp_kernels.h"
#include "core/simd/simd.h"

namespace ddbt::cog::warp {
namespace {

using tiff::SampleType;

struct ForcedTargetGuard {
    ~ForcedTargetGuard() { simd::ForceTargetForTest(0); }
};

const SampleType kTypes[] = {SampleType::kU8,  SampleType::kI8,  SampleType::kU16, SampleType::kI16,
                             SampleType::kU32, SampleType::kI32, SampleType::kF32, SampleType::kF64};

uint32_t Pick(std::mt19937& rng, uint32_t n) { return static_cast<uint32_t>(rng() % n); }

bool IsFloat(SampleType t) { return t == SampleType::kF32 || t == SampleType::kF64; }

// Writes v (a value of the type's range) as a sample.
void PutSample(SampleType t, double v, uint8_t* p) {
    switch (t) {
        case SampleType::kU8: *p = static_cast<uint8_t>(v); break;
        case SampleType::kI8: { const int8_t s = static_cast<int8_t>(v); std::memcpy(p, &s, 1); break; }
        case SampleType::kU16: { const uint16_t s = static_cast<uint16_t>(v); std::memcpy(p, &s, 2); break; }
        case SampleType::kI16: { const int16_t s = static_cast<int16_t>(v); std::memcpy(p, &s, 2); break; }
        case SampleType::kU32: { const uint32_t s = static_cast<uint32_t>(v); std::memcpy(p, &s, 4); break; }
        case SampleType::kI32: { const int32_t s = static_cast<int32_t>(v); std::memcpy(p, &s, 4); break; }
        case SampleType::kF32: { const float s = static_cast<float>(v); std::memcpy(p, &s, 4); break; }
        case SampleType::kF64: std::memcpy(p, &v, 8); break;
        default: break;
    }
}

void TypeRange(SampleType t, double& lo, double& hi) {
    switch (t) {
        case SampleType::kU8: lo = 0; hi = 255; break;
        case SampleType::kI8: lo = -128; hi = 127; break;
        case SampleType::kU16: lo = 0; hi = 65535; break;
        case SampleType::kI16: lo = -32768; hi = 32767; break;
        case SampleType::kU32: lo = 0; hi = 4294967295.0; break;
        case SampleType::kI32: lo = -2147483648.0; hi = 2147483647.0; break;
        default: lo = -1e6; hi = 1e6; break;
    }
}

// A sample value: extremes, the nodata value, runs of one value, random; for floats also NaN,
// infinities and values near the type's limits.
double RandomSample(SampleType t, std::mt19937& rng, std::optional<double> nodata) {
    double lo, hi;
    TypeRange(t, lo, hi);
    const uint32_t k = Pick(rng, 20);
    if (k == 0) return lo;
    if (k == 1) return hi;
    if (k == 2 && nodata) return *nodata;
    if (k == 3) return std::floor((lo + hi) / 2);
    if (IsFloat(t)) {
        if (k == 4) return std::numeric_limits<double>::quiet_NaN();
        if (k == 5) return (Pick(rng, 2) == 1) ? HUGE_VAL : -HUGE_VAL;
        if (k == 6) return t == SampleType::kF32 ? 3.0e38 : 1.0e300;
        if (k == 7) return -0.0;
        return std::uniform_real_distribution<double>(lo, hi)(rng);
    }
    return std::floor(std::uniform_real_distribution<double>(lo, hi + 1)(rng));
}

struct MaskConfig {
    const char* name;
    bool density, valid, perBand, dstNodata;
    int alphaOut;  // -2 none, -1 mask, 0 alpha band
};

const MaskConfig kConfigs[] = {
    {"no masks", false, false, false, false, -2},
    {"alpha -> mask", true, false, false, false, -1},
    {"alpha -> alpha band", true, false, false, false, 0},
    {"internal mask -> mask", false, true, false, false, -1},
    {"mask and alpha -> alpha band", true, true, false, false, 0},
    {"unified nodata", false, true, false, true, -2},
    {"per-band nodata -> alpha band", false, true, true, false, 0},
};

// One configuration: the source, its window, the output layout.
struct RowSetup {
    BilinearParams params;
    Window win;
    std::optional<double> nodata;
};

RowSetup MakeSetup(SampleType type, const MaskConfig& mc, int srcBands, int outBands, int W, int H, int x0, int y0, int w,
                int h, std::mt19937& rng) {
    RowSetup s;
    const size_t ss = tiff::SampleTypeSize(type);
    if (mc.dstNodata || mc.perBand) {
        double lo, hi;
        TypeRange(type, lo, hi);
        s.nodata = mc.perBand && IsFloat(type) ? -HUGE_VAL : std::floor((lo + hi) / 3);
    }
    BilinearParams& p = s.params;
    p.type = type;
    p.sourceWidth = W;
    p.sourceHeight = H;
    for (int b = 0; b < outBands; ++b) p.bandOffsets.push_back(static_cast<size_t>((b * 2 + 1) % srcBands) * ss);
    p.noMasks = !mc.density && !mc.valid && !mc.perBand && !mc.dstNodata;
    if (mc.perBand) p.perBandNodata = s.nodata;
    if (mc.dstNodata) {
        p.dstNodata = s.nodata;
        p.avoidSingle = outBands == 1;
        p.avoidMulti = outBands > 1;
    }
    p.alphaOut = mc.alphaOut == 0 ? outBands : mc.alphaOut;
    p.dstDensityInit = (p.alphaOut != -2 || p.dstNodata) ? 0.0 : 1.0;
    p.outPixelBytes = static_cast<size_t>(outBands + (p.alphaOut >= 0 ? 1 : 0)) * ss;

    Window& win = s.win;
    win.x0 = x0;
    win.y0 = y0;
    win.w = w;
    win.h = h;
    win.pb = static_cast<size_t>(srcBands) * ss;
    const size_t np = win.Pixels();
    win.px.assign(np * win.pb + kWindowPadding, 0xA5);
    for (size_t i = 0; i < np * static_cast<size_t>(srcBands); ++i)
        PutSample(type, RandomSample(type, rng, s.nodata), win.px.data() + i * ss);
    if (mc.valid) {
        win.valid.assign(np + kWindowPadding, 0x5A);
        for (size_t i = 0; i < np; ++i) win.valid[i] = (Pick(rng, 6)) == 0 ? 0 : 1;
    }
    if (mc.density) {
        win.density.resize(np);
        const float partial[] = {0.0f, 1e-10f, 0.5f, 254.0f / 255.0f, 0.99995f};
        for (size_t i = 0; i < np; ++i) win.density[i] = (Pick(rng, 5)) == 0 ? partial[Pick(rng, 5)] : 1.0f;
    }
    IndexValidity(win);
    return s;
}

// Source positions inside [lo, hi) of the source (x and y), with the edge cases of the kernel:
// pixel corners and centres, the last 1e-10 of the source, NaN.
void RandomPositions(std::mt19937& rng, size_t n, double loX, double hiX, double loY, double hiY, double W, double H,
                     std::vector<double>& fx, std::vector<double>& fy) {
    fx.resize(n);
    fy.resize(n);
    const auto special = [&](double lo, double hi, double size) {
        const double k = std::floor(std::uniform_real_distribution<double>(lo, hi)(rng));
        switch (Pick(rng, 7)) {
            case 0: return k;
            case 1: return k + 0.5;
            case 2: return std::nextafter(k + 0.5, -HUGE_VAL);
            case 3: return std::nextafter(k + 0.5, HUGE_VAL);
            case 4: return std::max(lo, size - 1e-10);
            case 5: return std::max(lo, size - 0.5);
            default: return std::max(lo, 0.25);
        }
    };
    for (size_t i = 0; i < n; ++i) {
        const uint32_t k = Pick(rng, 10);
        if (k == 0) {
            fx[i] = fy[i] = std::numeric_limits<double>::quiet_NaN();
            continue;
        }
        fx[i] = k < 3 ? special(loX, hiX, W) : std::uniform_real_distribution<double>(loX, hiX)(rng);
        fy[i] = (k == 1 || k == 3) ? special(loY, hiY, H) : std::uniform_real_distribution<double>(loY, hiY)(rng);
        fx[i] = std::min(std::max(fx[i], loX), hiX);
        fy[i] = std::min(std::max(fy[i], loY), hiY);
    }
}

// Warps the row with the scalar kernel and with the SIMD one on every target: same bytes.
void ExpectSameRow(const RowSetup& s, const std::vector<double>& fx, const std::vector<double>& fy,
                   const std::string& what) {
    const size_t n = fx.size();
    const size_t opb = s.params.outPixelBytes;
    // The destination before the warp (the blend reads it): nodata or a pattern.
    std::vector<uint8_t> init(n * opb), initMask(n, 7);
    for (size_t i = 0; i < init.size(); ++i) init[i] = static_cast<uint8_t>(i * 37 + 11);
    if (s.params.dstNodata) {
        const size_t ss = tiff::SampleTypeSize(s.params.type);
        for (size_t i = 0; i < init.size() / ss; ++i) PutSample(s.params.type, *s.params.dstNodata, init.data() + i * ss);
    }
    std::vector<uint8_t> refOut = init, refMask = initMask;
    WarpRow(s.params, s.win, fx.data(), fy.data(), n, refOut.data(), refMask.data(), KernelPath::kScalar);
    ForcedTargetGuard guard;
    for (const int64_t bit : simd::SupportedTargetBits()) {
        simd::ForceTargetForTest(bit);
        std::vector<uint8_t> out = init, mask = initMask;
        WarpRow(s.params, s.win, fx.data(), fy.data(), n, out.data(), mask.data(), KernelPath::kSimd);
        for (size_t i = 0; i < n; ++i) {
            ASSERT_EQ(0, std::memcmp(out.data() + i * opb, refOut.data() + i * opb, opb))
                << what << " target " << simd::TargetNameOf(bit) << " pixel " << i << " of " << n << " at (" << fx[i]
                << ", " << fy[i] << ")";
            ASSERT_EQ(mask[i], refMask[i]) << what << " target " << simd::TargetNameOf(bit) << " mask of pixel " << i;
        }
    }
}

}  // namespace

TEST(WarpKernels, BilinearRowsMatchTheScalarKernelOnEveryTarget) {
    std::mt19937 rng(20261112);
    const size_t lengths[] = {1, 2, 3, 5, 7, 8, 9, 15, 16, 17, 31, 33, 64, 255, 256, 257, 300};
    for (const SampleType type : kTypes) {
        for (const MaskConfig& mc : kConfigs) {
            for (const int srcBands : {1, 3, 4}) {
                const int outBands = srcBands == 4 ? 3 : srcBands;
                for (const size_t n : lengths) {
                    const std::string what = std::string(mc.name) + " type " +
                                             std::to_string(static_cast<int>(type)) + " bands " +
                                             std::to_string(srcBands) + " n " + std::to_string(n);
                    // The whole source as the window: every edge case of the kernel.
                    {
                        const int W = 2 + static_cast<int>(Pick(rng, 12)), H = 2 + static_cast<int>(Pick(rng, 9));
                        const RowSetup s = MakeSetup(type, mc, srcBands, outBands, W, H, 0, 0, W, H, rng);
                        std::vector<double> fx, fy;
                        RandomPositions(rng, n, 0, W - 1e-10, 0, H - 1e-10, W, H, fx, fy);
                        ExpectSameRow(s, fx, fy, what + " whole source");
                    }
                    // A window inside a larger source, of the size the positions need.
                    {
                        const int W = 300, H = 200;
                        const double loX = 37.3, hiX = 37.3 + 1 + static_cast<double>(Pick(rng, 40));
                        const double loY = 11.8, hiY = 11.8 + 1 + static_cast<double>(Pick(rng, 20));
                        std::vector<double> fx, fy;
                        RandomPositions(rng, n, loX, hiX, loY, hiY, W, H, fx, fy);
                        const kernels::TapOrigins o = kernels::TapOriginBounds(fx.data(), fy.data(), n, kernels::Path::kScalar);
                        if (!o.any) continue;
                        const int x0 = static_cast<int>(o.minX), y0 = static_cast<int>(o.minY);
                        const int x1 = static_cast<int>(o.maxX) + 2, y1 = static_cast<int>(o.maxY) + 2;
                        const RowSetup s = MakeSetup(type, mc, srcBands, outBands, W, H, x0, y0, x1 - x0, y1 - y0, rng);
                        ExpectSameRow(s, fx, fy, what + " window");
                    }
                }
            }
        }
    }
}

TEST(WarpKernels, BilinearRowWritesEveryInteriorPixelWithFullTaps) {
    // Interior positions over a source of fully valid, opaque pixels: the SIMD kernel writes every
    // one of them (no silent fall-back to the scalar code), with the scalar kernel's bytes.
    std::mt19937 rng(7);
    const MaskConfig opaque[] = {{"no masks", false, false, false, false, -2},
                                 {"opaque alpha -> mask", true, false, false, false, -1},
                                 {"valid mask -> alpha band", false, true, false, false, 0}};
    ForcedTargetGuard guard;
    for (const SampleType type : kTypes) {
        for (const MaskConfig& mc : opaque) {
            const int W = 40, H = 30;
            RowSetup s = MakeSetup(type, mc, 4, 3, W, H, 0, 0, W, H, rng);
            std::fill(s.win.density.begin(), s.win.density.end(), 1.0f);
            for (size_t i = 0; i < s.win.Pixels(); ++i) {
                if (!s.win.valid.empty()) s.win.valid[i] = 1;
                for (int b = 0; b < 4; ++b) {
                    const size_t ss = tiff::SampleTypeSize(type);
                    PutSample(type, static_cast<double>(Pick(rng, 100)), s.win.px.data() + i * s.win.pb + static_cast<size_t>(b) * ss);
                }
            }
            IndexValidity(s.win);
            const size_t n = 203;
            std::vector<double> fx(n), fy(n);
            for (size_t i = 0; i < n; ++i) {
                fx[i] = std::uniform_real_distribution<double>(0.5, W - 0.5)(rng);
                fy[i] = std::uniform_real_distribution<double>(0.5, H - 0.5)(rng);
            }
            for (const int64_t bit : simd::SupportedTargetBits()) {
                simd::ForceTargetForTest(bit);
                std::vector<uint8_t> status(n, 99), out(n * s.params.outPixelBytes, 0);
                kernels::BilinearJob job;
                job.type = type;
                job.px = s.win.px.data();
                job.pixelBytes = s.win.pb;
                job.flags = s.win.flags.empty() ? nullptr : s.win.flags.data();
                job.winW = W;
                job.winH = H;
                job.sourceWidth = W;
                job.sourceHeight = H;
                job.noMasks = s.params.noMasks;
                job.bandOffsets = s.params.bandOffsets.data();
                job.bands = s.params.bandOffsets.size();
                job.out = out.data();
                job.outPixelBytes = s.params.outPixelBytes;
                job.fx = fx.data();
                job.fy = fy.data();
                job.n = n;
                job.status = status.data();
                kernels::BilinearRow(job);
                for (size_t i = 0; i < n; ++i)
                    ASSERT_EQ(status[i], kernels::kWritten) << mc.name << " " << simd::TargetNameOf(bit) << " pixel " << i;
            }
            std::vector<double> fx2 = fx, fy2 = fy;
            ExpectSameRow(s, fx2, fy2, std::string(mc.name) + " opaque type " + std::to_string(static_cast<int>(type)));
        }
    }
}

TEST(WarpKernels, SourcePixelsAndTapBoundsMatchTheScalarCodeOnEveryTarget) {
    std::mt19937 rng(42);
    kernels::SourcePixelMapping m;
    m.inverse = {-1234.5, 0.05, 0.0013, 987.25, -0.0007, -0.05};
    m.width = 4000;
    m.height = 3000;
    ForcedTargetGuard guard;
    for (size_t n = 0; n <= 70; ++n) {
        std::vector<double> X(n), Y(n);
        for (size_t i = 0; i < n; ++i) {
            // Around the source: inside, just outside, far outside, not finite.
            X[i] = 24690.0 + std::uniform_real_distribution<double>(-2000, 82000)(rng);
            Y[i] = 19745.0 + std::uniform_real_distribution<double>(-2000, 62000)(rng);
            switch (Pick(rng, 12)) {
                case 0: X[i] = std::numeric_limits<double>::quiet_NaN(); break;
                case 1: Y[i] = HUGE_VAL; break;
                case 2: X[i] = -HUGE_VAL; break;
                case 3: X[i] = (0 - m.inverse[0] - Y[i] * m.inverse[2]) / m.inverse[1]; break;  // fx near 0
                default: break;
            }
        }
        std::vector<double> rx = X, ry = Y;
        std::vector<uint32_t> refOutside(n + 1);
        const size_t refCount = kernels::ToSourcePixels(m, rx.data(), ry.data(), n, refOutside.data(), kernels::Path::kScalar);
        const kernels::TapOrigins ro = kernels::TapOriginBounds(rx.data(), ry.data(), n, kernels::Path::kScalar);
        for (const int64_t bit : simd::SupportedTargetBits()) {
            simd::ForceTargetForTest(bit);
            SCOPED_TRACE(simd::TargetNameOf(bit) + " n " + std::to_string(n));
            std::vector<double> sx = X, sy = Y;
            std::vector<uint32_t> outside(n + 1);
            const size_t count = kernels::ToSourcePixels(m, sx.data(), sy.data(), n, outside.data());
            ASSERT_EQ(count, refCount);
            for (size_t k = 0; k < count; ++k) ASSERT_EQ(outside[k], refOutside[k]);
            ASSERT_EQ(0, std::memcmp(sx.data(), rx.data(), n * sizeof(double)));
            ASSERT_EQ(0, std::memcmp(sy.data(), ry.data(), n * sizeof(double)));
            const kernels::TapOrigins o = kernels::TapOriginBounds(sx.data(), sy.data(), n);
            ASSERT_EQ(o.any, ro.any);
            if (o.any) {
                EXPECT_EQ(o.minX, ro.minX);
                EXPECT_EQ(o.minY, ro.minY);
                EXPECT_EQ(o.maxX, ro.maxX);
                EXPECT_EQ(o.maxY, ro.maxY);
            }
        }
    }
}

}  // namespace ddbt::cog::warp
