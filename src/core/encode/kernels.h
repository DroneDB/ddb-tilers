// SPDX-License-Identifier: MPL-2.0
//
// Highway SIMD kernels of the tile encoders (runtime dispatch, see kernels.cpp). Internal API:
// the encoders and the unit tests use it.
#pragma once

#include <cstddef>
#include <cstdint>

namespace ddbt::encode::kernels {

/// Minimum and maximum alpha of `pixels` interleaved pixels of `channels` (2 or 4) bytes, the
/// alpha last. Starts from the given min / max (pass 255 / 0).
void AlphaRange(const uint8_t* row, size_t pixels, uint32_t channels, uint8_t& minAlpha, uint8_t& maxAlpha);

/// PNG filter types (the filter byte of a row).
enum PngFilterType : uint8_t { kFilterNone = 0, kFilterSub = 1, kFilterUp = 2, kFilterAverage = 3, kFilterPaeth = 4 };

/// Sum over the row of min(v, 256 - v) of the bytes each filter would write (libpng's
/// "minimum sum of absolute differences" heuristic), for the five filters, into costs[0..4].
/// cur / prev: n bytes; prev is the previous (unfiltered) row, or zeros for the first row.
void PngFilterCosts(const uint8_t* cur, const uint8_t* prev, size_t n, size_t bpp, uint64_t costs[5]);

/// Writes the n filtered bytes of cur (filter type `filter`) to out.
void PngFilterRow(const uint8_t* cur, const uint8_t* prev, size_t n, size_t bpp, PngFilterType filter,
                  uint8_t* out);

/// How the rows of a PNG image are filtered.
enum class PngRowFilter : uint8_t {
    kNone,
    kSub,
    kUp,
    kAverage,
    kPaeth,
    kAdaptive,  ///< per row, the filter of minimum sum of absolute values (libpng's choice)
};

/// Rows of a PNG image to filter. The source rows have srcChannels bytes per pixel; when
/// dstChannels < srcChannels the alpha byte is dropped (4 -> 3 or 2 -> 1) on the way.
struct PngFilterJob {
    const uint8_t* src = nullptr;
    size_t srcStride = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t srcChannels = 0;
    uint32_t dstChannels = 0;
    PngRowFilter filter = PngRowFilter::kAdaptive;
    /// height * (1 + width * dstChannels) bytes: filter byte then filtered row, per row.
    uint8_t* out = nullptr;
    /// 3 * width * dstChannels bytes of scratch: two converted rows and a row of zeros.
    uint8_t* scratch = nullptr;
};

/// Filters the rows of job into job.out (the IDAT payload before deflate).
void PngFilterImage(const PngFilterJob& job);

/// rgba -> rgb (alpha dropped), `pixels` pixels.
void DropAlpha4(const uint8_t* rgba, size_t pixels, uint8_t* rgb);
/// grey+alpha -> grey (alpha dropped).
void DropAlpha2(const uint8_t* ga, size_t pixels, uint8_t* g);
/// rgba -> rgbx where every pixel of alpha 0 becomes white (x: the alpha, or 255 when white).
void WhiteUnderTransparent4(const uint8_t* rgba, size_t pixels, uint8_t* rgbx);
/// grey+alpha -> grey where every pixel of alpha 0 becomes white.
void WhiteUnderTransparent2(const uint8_t* ga, size_t pixels, uint8_t* g);
/// grey -> rgb.
void GreyToRgb(const uint8_t* g, size_t pixels, uint8_t* rgb);
/// grey+alpha -> rgba.
void GreyAlphaToRgba(const uint8_t* ga, size_t pixels, uint8_t* rgba);

}  // namespace ddbt::encode::kernels
