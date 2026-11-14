// SPDX-License-Identifier: MPL-2.0
//
// GDAL's bilinear warp kernel (alg/gdalwarpkernel.cpp, as gdalwarp -r bilinear runs it) for one
// row of destination pixels whose source positions are known:
//
//   - without any source mask: GWKBilinearResampleNoMasks4SampleT (the 2 x 2 formula inside the
//     source, valid-tap renormalisation at its edges, rounding by + 0.5);
//   - with masks (source alpha as density, nodata, an internal mask): the centre pixel must be
//     valid; GWKBilinearResample4Sample (taps with density > 1e-9, renormalised, the density
//     interpolated too), the destination blend of GWKSetPixelValue, ClampRoundAndAvoidNoData,
//     GWKAvoidNoDataMultiBand, and the destination alpha from the centre pixel's density
//     (GWKOverlayDensity, then GDALWarpDstAlphaMasker: (int)(density * (DST_ALPHA_MAX + 0.1)) for
//     integer alpha, density * DST_ALPHA_MAX for floating point).
//
// The pixels whose four taps are inside the source and fully valid (the interior of almost every
// raster) go through a SIMD kernel (warp_kernels.h); every other pixel through the scalar code
// of bilinear.cpp. Both give the same bits: same operations in the same order, no FMA
// contraction (the files are compiled with -ffp-contract=off).
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include "cog/tiff/tiff_reader.h"

namespace ddbt::cog::warp {

/// Bytes after the samples and after the validity bytes of a Window: the SIMD kernel reads a
/// whole 32-bit word at the offset of a sample.
inline constexpr size_t kWindowPadding = 8;

/// A window of decoded source pixels (all source bands, interleaved), with the unified validity
/// and the density of each pixel.
struct Window {
    int x0 = 0, y0 = 0, w = 0, h = 0;
    size_t pb = 0;
    std::vector<uint8_t> px;     // w * h pixels of pb bytes, then kWindowPadding bytes
    std::vector<uint8_t> mask;   // internal mask bytes (0 = invalid)
    std::vector<uint8_t> valid;  // unified validity, w * h then kWindowPadding bytes; empty: all valid
    std::vector<float> density;  // alpha density (alpha sources); empty: all 1
    std::vector<uint8_t> flags;  // kernels::WindowFlags per pixel, then kWindowPadding bytes (IndexValidity)
    size_t Pixels() const { return static_cast<size_t>(w) * static_cast<size_t>(h); }
    size_t Index(int x, int y) const {
        return static_cast<size_t>(y - y0) * static_cast<size_t>(w) + static_cast<size_t>(x - x0);
    }
    const uint8_t* Pixel(int x, int y) const { return px.data() + Index(x, y) * pb; }
};

/// Computes the flags of the window (kernels::WindowFlags) from its validity and density, after
/// they change; clears them when the window has neither. The SIMD kernel reads one flag byte per
/// destination pixel instead of the validity and density of five source pixels.
void IndexValidity(Window& win);

/// How the kernel resamples and writes one output (GDAL's warp options, resolved).
struct BilinearParams {
    tiff::SampleType type = tiff::SampleType::kU8;
    int sourceWidth = 0, sourceHeight = 0;
    /// Per output band: the byte offset of its source sample in a window pixel.
    std::vector<size_t> bandOffsets;
    size_t outPixelBytes = 0;
    bool noMasks = true;  ///< no alpha, mask or nodata: GWKBilinearResampleNoMasks4SampleT
    /// Source nodata tested on every band (no UNIFIED_SRC_NODATA), on top of the unified validity.
    std::optional<double> perBandNodata;
    std::optional<double> dstNodata;
    bool avoidSingle = false;  ///< ClampRoundAndAvoidNoData on each band
    bool avoidMulti = false;   ///< GWKAvoidNoDataMultiBand
    /// Destination density when a value is blended (GWKSetPixelValue): 0 with a destination alpha
    /// or nodata, else 1.
    double dstDensityInit = 1.0;
    /// Destination alpha: -2 none, -1 the output mask, >= 0 that band of the output pixel.
    int alphaOut = -2;
    /// GDAL's DST_ALPHA_MAX of an alpha band (alphaOut >= 0): 65535 for UInt16, 32767 for Int16,
    /// 255 otherwise (the output mask is always 255).
    float dstAlphaMax = 255.0f;
};

enum class KernelPath : uint8_t {
    kSimd,    ///< the SIMD kernel where it applies, the scalar code elsewhere (production)
    kScalar,  ///< the scalar code for every pixel (the reference)
};

/// Warps n destination pixels of a row. fx / fy: their source pixel positions (NaN: no source,
/// not written); out: the first pixel (outPixelBytes apart, bands sample-size apart); mask: the
/// first output mask byte (alphaOut == -1), else unused. The window covers every tap of the row.
void WarpRow(const BilinearParams& p, const Window& win, const double* fx, const double* fy, size_t n, uint8_t* out,
             uint8_t* mask, KernelPath path = KernelPath::kSimd);

/// Calls f.template operator()<T>() with T the C++ type of the samples of `type`, for the types the
/// warper reads; throws std::runtime_error for the others.
template <typename F>
decltype(auto) VisitSampleType(tiff::SampleType type, F&& f) {
    switch (type) {
        case tiff::SampleType::kU8: return f.template operator()<uint8_t>();
        case tiff::SampleType::kI8: return f.template operator()<int8_t>();
        case tiff::SampleType::kU16: return f.template operator()<uint16_t>();
        case tiff::SampleType::kI16: return f.template operator()<int16_t>();
        case tiff::SampleType::kU32: return f.template operator()<uint32_t>();
        case tiff::SampleType::kI32: return f.template operator()<int32_t>();
        case tiff::SampleType::kF32: return f.template operator()<float>();
        case tiff::SampleType::kF64: return f.template operator()<double>();
        default: throw std::runtime_error("warp: unsupported sample type");
    }
}

/// A sample of C++ type T as double.
template <typename T>
inline double LoadSample(const uint8_t* p) {
    T v;
    std::memcpy(&v, p, sizeof(T));
    return static_cast<double>(v);
}

/// GDAL's ARE_REAL_EQUAL (cpl_port.h) for float / double nodata comparisons.
template <typename T>
inline bool RealEqual(T a, T b) {
    return a == b || std::abs(a - b) < std::numeric_limits<T>::epsilon() * std::abs(a + b) * 2;
}

/// GDALWarpNoDataMasker: a sample of C++ type T is nodata (integers exactly, floats
/// ARE_REAL_EQUAL, a NaN nodata matches NaN).
template <typename T>
inline bool IsNodata(const uint8_t* p, double nodata) {
    if constexpr (std::is_floating_point_v<T>) {
        T v;
        std::memcpy(&v, p, sizeof(T));
        const T nd = static_cast<T>(nodata);
        return std::isnan(nd) ? std::isnan(v) : RealEqual(v, nd);
    } else {
        return LoadSample<T>(p) == nodata;
    }
}

}  // namespace ddbt::cog::warp
