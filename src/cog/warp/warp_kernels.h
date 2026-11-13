// SPDX-License-Identifier: MPL-2.0
//
// Highway SIMD kernels of the warp (runtime dispatch, see warp_kernels.cpp). Internal API: the
// warp producer, bilinear.cpp and the unit tests use it.
//
// Every kernel gives the bits of the scalar code it replaces: the same IEEE operations in the
// same order, no FMA contraction (warp_kernels.cpp is compiled with -ffp-contract=off). The
// kernels with a `path` argument carry that scalar code as their reference.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "cog/tiff/tiff_reader.h"

namespace ddbt::cog::warp::kernels {

enum class Path : uint8_t { kSimd, kScalar };

/// The inverse geotransform of the source and its size in pixels.
struct SourcePixelMapping {
    std::array<double, 6> inverse{};
    double width = 0, height = 0;
};

/// Source CRS positions to source pixel positions, in place (fx = inv[0] + X * inv[1] + Y * inv[2],
/// fy likewise). Points that are not finite become NaN. Points outside the source (GDAL's
/// GWKCheckAndComputeSrcOffsets: x >= 0, y >= 0, x + 1e-10 <= width, y + 1e-10 <= height) keep
/// their pixel position and their index is written to `outside` (n entries at most): the caller
/// decides (an exact transform may move them in). Returns the number of outside points.
size_t ToSourcePixels(const SourcePixelMapping& m, double* xs, double* ys, size_t n, uint32_t* outside,
                      Path path = Path::kSimd);

/// Origins of the 2 x 2 bilinear footprints of n source positions: the min / max of
/// floor(x - 0.5) and floor(y - 0.5) over the points whose x is not NaN.
struct TapOrigins {
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    bool any = false;
};
TapOrigins TapOriginBounds(const double* xs, const double* ys, size_t n, Path path = Path::kSimd);

/// The validity of the 2 x 2 taps (x, y), (x + 1, y), (x, y + 1), (x + 1, y + 1) that start at a
/// window pixel, one byte per pixel: bit t is set when tap t is a valid centre pixel (validity,
/// density >= 1e-9, as the scalar kernel tests the centre), kQuadFull when the four taps are in
/// the window and fully valid (validity 1, density 1).
namespace WindowFlags {
inline constexpr uint8_t kCentreValid = 1;  ///< bit 0: the pixel itself
inline constexpr uint8_t kQuadFull = 16;
}  // namespace WindowFlags

/// What BilinearRow did with each pixel.
enum PixelStatus : uint8_t {
    kSkipped = 0,  ///< nothing to write (no source position, or an invalid centre pixel)
    kWritten = 1,  ///< every band written; the four taps were inside the source and fully valid
    kScalar = 2,   ///< left to the scalar kernel
};

/// One row of destination pixels for BilinearRow.
struct BilinearJob {
    tiff::SampleType type = tiff::SampleType::kU8;
    // The source window: pixels of pixelBytes bytes, interleaved, row-major, padded by 8 bytes
    // (32-bit and 64-bit words are read at sample offsets); with masks, its WindowFlags bytes
    // (padded likewise). Its offsets (w * h * pixelBytes) must fit an int32.
    const uint8_t* px = nullptr;
    size_t pixelBytes = 0;
    const uint8_t* flags = nullptr;
    int winX0 = 0, winY0 = 0, winW = 0, winH = 0;
    int sourceWidth = 0, sourceHeight = 0;
    bool noMasks = true;  ///< GWKBilinearResampleNoMasks4SampleT (no flags)
    // The output: band b of pixel i at out + i * outPixelBytes + b * sample size.
    const size_t* bandOffsets = nullptr;  ///< per output band, its sample's offset in a window pixel
    size_t bands = 0;
    uint8_t* out = nullptr;
    size_t outPixelBytes = 0;
    uint8_t* mask = nullptr;  ///< the output mask of the row, set to maskValue where written (optional)
    uint8_t maskValue = 255;
    // The row.
    const double* fx = nullptr;
    const double* fy = nullptr;
    size_t n = 0;
    uint8_t* status = nullptr;  ///< n PixelStatus
};

/// Resamples the pixels of the row whose four taps are inside the source and (with masks) of
/// validity 1 and density 1 (kQuadFull), writing their bands (GDAL's value: the 2 x 2 formula
/// without masks, else the renormalised sum and ClampRound) and their mask byte; classifies every
/// pixel. Without masks there are no skipped pixels; with masks a pixel whose centre is invalid is
/// skipped. The per-band nodata of sources without UNIFIED_SRC_NODATA is not handled here (the
/// caller keeps those rows scalar).
void BilinearRow(const BilinearJob& job);

}  // namespace ddbt::cog::warp::kernels
