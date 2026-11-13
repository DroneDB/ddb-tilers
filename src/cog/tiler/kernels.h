// SPDX-License-Identifier: MPL-2.0
//
// Highway SIMD kernels of the COG tiler's per-pixel loops (runtime dispatch, see kernels.cpp).
// Every kernel gives the bytes of its scalar reference (the *Scalar functions, which are also
// the tails of the vector loops), so tiles stay byte-identical on every target:
//
//  - NodataMaskF32: GDAL's nodata mask of a Float32 band (ARE_REAL_EQUAL, 2 ulp).
//  - FloatToByte: GDALCopyWords of Float32 samples to Byte (the alpha band of a Float32 file).
//  - StretchF32 / StretchF64: DroneDB's stretchToByte in double, the same operations in the same
//    order (clip with the comparisons of std::min / std::max, so NaN shows as 255; subtract,
//    multiply by 255, divide, truncate). The file is compiled without contraction.
//  - InterleavePlanes / ExtractSamples: planar <-> pixel-interleaved bytes.
//  - AllBytesEqual: the all-transparent / all-opaque checks of an alpha plane.
//
// Internal API: the tiler and the unit tests use it. Float buffers are passed as bytes (the
// tiler's sample buffers); they need no alignment.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace ddbt::cog::tiler::kernels {

/// GDAL's ARE_REAL_EQUAL (gdal_cpp_functions.h), ulp = 2.
inline bool AreRealEqual(float a, float b) noexcept {
    return a == b || std::abs(a - b) < std::numeric_limits<float>::epsilon() * std::abs(a + b) * 2;
}
inline bool AreRealEqual(double a, double b) noexcept {
    return a == b || std::abs(a - b) < static_cast<double>(std::numeric_limits<float>::epsilon()) * std::abs(a + b) * 2;
}

/// out[i] = 0 where the Float32 sample i is AreRealEqual to `nodata` (not NaN), else 255.
void NodataMaskF32(const uint8_t* samples, size_t n, float nodata, uint8_t* out);
void NodataMaskF32Scalar(const uint8_t* samples, size_t n, float nodata, uint8_t* out) noexcept;

/// GDALCopyWords of Float32 samples to Byte (SampleToByte, raster_io.h): NaN and v <= 0 give 0,
/// v >= 255 gives 255, else v + 0.5 (in double) truncated.
void FloatToByte(const uint8_t* samples, size_t n, uint8_t* out);
void FloatToByteScalar(const uint8_t* samples, size_t n, uint8_t* out) noexcept;

/// The vector stretch needs every result in [0, 256): true when hi > lo and 255 * (hi - lo) is
/// finite (then lo and hi are finite and so is every product). Other ranges take the scalar
/// path, whose conversion of an out-of-range double is the compiler's.
bool StretchVectorizable(double lo, double hi) noexcept;

/// StretchToByte (stretch.h, the scalar reference) of n Float32 (promoted to double) / Float64 samples. Requires
/// StretchVectorizable(lo, hi).
void StretchF32(const uint8_t* samples, size_t n, double lo, double hi, uint8_t* out);
void StretchF64(const uint8_t* samples, size_t n, double lo, double hi, uint8_t* out);

/// Interleaves `channels` (1 to 4) planes of n bytes into dst (channels bytes per pixel).
void InterleavePlanes(const uint8_t* const* planes, size_t channels, size_t n, uint8_t* dst);
void InterleavePlanesScalar(const uint8_t* const* planes, size_t channels, size_t n, uint8_t* dst) noexcept;

/// Copies samples idx[0..count) (0-based, each below samplesPerPixel) of n consecutive pixels of
/// samplesPerPixel samples of sampleBytes (1, 2, 4 or 8) bytes into planes[0..count).
void ExtractSamples(const uint8_t* pixels, size_t sampleBytes, size_t samplesPerPixel, size_t n, const int* idx,
                    size_t count, uint8_t* const* planes);
void ExtractSamplesScalar(const uint8_t* pixels, size_t sampleBytes, size_t samplesPerPixel, size_t n,
                          const int* idx, size_t count, uint8_t* const* planes) noexcept;

/// dst[i] = word xIn[i] of `row` (rowWords 32-bit samples, one per pixel; n samples, 4 bytes
/// each): the columns of a resampled read of a single-band 32-bit level. The columns must be
/// non-decreasing (a plan's run) and below rowWords; where 2 vectors span the columns of a
/// vector of outputs (downsampling by less than 2), they are picked by a lane permutation.
void PickSamples32(const uint8_t* row, size_t rowWords, const uint32_t* xIn, size_t n, uint8_t* dst);
void PickSamples32Scalar(const uint8_t* row, const uint32_t* xIn, size_t n, uint8_t* dst) noexcept;

/// Every one of the n bytes equals `value` (true for n = 0).
bool AllBytesEqual(const uint8_t* p, size_t n, uint8_t value);

}  // namespace ddbt::cog::tiler::kernels
