// SPDX-License-Identifier: MPL-2.0
//
// The two passes of the overview convolution (overview.h) on double-precision planes, with
// Highway runtime dispatch (overview_kernels.cpp). Internal API: overview.cpp and the unit tests
// use it.
//
// Every lane performs the scalar reference's operations in the reference's order: a product
// then a sum per tap, tap after tap, the same weight products, the same divisions, no fused
// multiply-add (the file is compiled without contraction). The results are therefore the
// reference's bits on every target. What changes is only which pixels share an instruction:
// in both passes the lanes are consecutive destination columns of one row. In the horizontal
// pass each lane has its own taps: a group of columns whose taps start two source columns apart
// (a factor of two) reads them with deinterleaving loads, any other group with gathers.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ddbt::cog::pyramid::kernels {

/// Grows a per-thread buffer to at least n elements, never shrinking it: a call over a smaller
/// window between two larger ones would otherwise make the larger one clear it again.
template <typename T>
T* GrowBuffer(std::vector<T>& v, size_t n) {
    if (v.size() < n) v.resize(n);
    return v.data();
}

/// The vertical pass computes columns in groups of this many: the row strides of both passes'
/// outputs and of the vertical pass's inputs are multiples of it (padding columns are computed by
/// the vertical pass, never read back).
size_t ColumnGroup();

/// Zero samples the horizontal pass may read past the last source column of a row.
size_t RowPadding();

/// Columns per block of the all-valid summary (HorizontalJob::partial).
constexpr size_t kValidityBlock = 32;
inline size_t ValidityBlocks(size_t srcW) { return (srcW + kValidityBlock - 1) / kValidityBlock; }

/// The taps of one axis: destination index k reads count[k] source indices from start[k]
/// (relative to the planes), with the raw weights weights[k * maxTaps + j] and the normalised
/// ones normalised[k * maxTaps + j] (raw weight * invSum, the product the reference uses).
struct AxisTaps {
    const int* start = nullptr;
    const int* count = nullptr;
    const double* weights = nullptr;
    const double* normalised = nullptr;
    int maxTaps = 0;
};

/// How the samples are weighted.
struct MaskLayout {
    /// 0: no mask; 1: one mask shared by every band; `bands`: one mask per band (its nodata).
    int planes = 0;
    /// Floating-point samples: under a mask, a NaN sample weighs 0; with a shared mask, a band
    /// left without a valid tap in a valid pixel is marked NaN for the vertical pass.
    bool floatSamples = false;
};

/// The states of a group of columns in a row of the horizontal result (HorizontalJob::groupState).
constexpr uint8_t kGroupAllValid = 1;
constexpr uint8_t kGroupAllInvalid = 2;

/// Horizontal pass of the source rows of a window.
struct HorizontalJob {
    int bands = 1;
    MaskLayout mask;
    int rows = 0;
    /// The source rows arrive in chunks of chunkRows (what the planes below hold): fill(context, y,
    /// n) writes rows [y, y + n) to plane rows [0, n). The column taps are prepared once per pass.
    int chunkRows = 0;
    void (*fill)(void* context, int y, int n) = nullptr;
    void* fillContext = nullptr;
    /// Band b of plane row r at source column x: samples[(r * bands + b) * rowStride + x]; mask
    /// plane p (values 0..255) at masks[(r * mask.planes + p) * rowStride + x]. Each row holds
    /// srcW values then at least RowPadding() zeros.
    const double* samples = nullptr;
    const double* masks = nullptr;
    size_t rowStride = 0;
    int srcW = 0;
    /// The validity summary: per plane row r and mask plane p, two runs of
    /// ValidityBlocks(srcW) + 1 counts from partial + (r * mask.planes + p) * 2 *
    /// (ValidityBlocks(srcW) + 1): the number of blocks of kValidityBlock columns before block k
    /// holding a mask value other than 255, then holding one other than 0. Groups of columns whose
    /// taps only cover full blocks take the all-valid path (the same products, without reading
    /// the mask); only empty blocks, the all-invalid one (every result 0, nothing read). May be
    /// null (every column reads its mask).
    const int* partial = nullptr;
    AxisTaps cols;
    int dstW = 0;
    /// Band b of row r, destination column i: horiz[b * outPlane + r * outRow + i]; mask plane p's
    /// validity (1 or 0) at valid[p * outPlane + r * outRow + i] (masked passes only). Columns
    /// from dstW to outRow are left untouched.
    double* horiz = nullptr;
    double* valid = nullptr;
    size_t outPlane = 0;
    size_t outRow = 0;
    /// Masked passes: per mask plane p, row r and group k of ColumnGroup() destination columns,
    /// groupState[p * statePlane + r * stateRow + k] holds kGroupAllValid and kGroupAllInvalid
    /// (the caller sets both first); the pass clears the first when a column of the group is not
    /// valid, the second when one is. May be null.
    uint8_t* groupState = nullptr;
    size_t statePlane = 0;
    size_t stateRow = 0;
};
void HorizontalPass(const HorizontalJob& job);

/// Vertical pass of one destination row.
struct VerticalJob {
    int bands = 1;
    MaskLayout mask;
    /// The horizontal result and validity, laid out as HorizontalJob writes them; row 0 is the
    /// first source row the row taps can name.
    const double* horiz = nullptr;
    const double* valid = nullptr;
    size_t plane = 0;
    size_t rowStride = 0;
    /// Columns to compute: a multiple of ColumnGroup(), at most rowStride.
    int width = 0;
    /// This destination row's taps (start relative to horiz row 0).
    int start = 0;
    int count = 0;
    const double* weights = nullptr;
    const double* normalised = nullptr;
    /// Band b, column i: value[b * width + i] is the convolution result (masked: the weighted sum
    /// divided by the weight sum); fill[b * width + i] is 1 where the pixel takes the nodata value
    /// instead (masked passes only: no run of valid taps long enough, or no valid tap).
    double* value = nullptr;
    double* fill = nullptr;
    /// The horizontal pass's group states (HorizontalJob::groupState, row 0 as for horiz). A group
    /// whose taps' rows are all valid takes the all-valid path (weights w * 1, the run rule holds);
    /// one whose taps' rows are all invalid takes the nodata value. May be null.
    const uint8_t* groupState = nullptr;
    size_t statePlane = 0;
    size_t stateRow = 0;
};
void VerticalPass(const VerticalJob& job);

/// The last step of 8-bit samples without a nodata value, as the scalar writer does it: the
/// value rounded to float, NaN to 0, clamped to 0..255, plus 0.5 and truncated; 0 where fill is
/// not 0 (fill may be null). Band b of pixel i is value[b * width + i] (fill likewise); pixel i's
/// bytes go to dst[i * bands + b]. Writes the first pixels of [0, count), a multiple of the
/// vector length, and returns how many: the caller writes the rest.
size_t WriteBytes(const double* value, const double* fill, int bands, size_t width, size_t count, uint8_t* dst);

}  // namespace ddbt::cog::pyramid::kernels
