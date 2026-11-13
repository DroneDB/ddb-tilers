// SPDX-License-Identifier: MPL-2.0
//
// TIFF predictors (decode side) and sample byte swapping, Highway-vectorized with runtime
// dispatch (predictor.cpp). Data is processed in place; rows hold `width` pixels of `spp`
// samples (spp = 1 for a BAND-interleaved block) of `bytesPerSample` bytes, rows are
// `rowStride` bytes apart, and `data` is aligned to bytesPerSample.
#pragma once

#include <cstddef>
#include <cstdint>

namespace ddbt::cog::codecs {

/// Predictor 2 (horizontal differencing): x[i] += x[i - spp] along each row, in the integer
/// width of the samples (1, 2, 4 or 8 bytes) with wrap-around. The samples must already be in
/// host byte order (libtiff swaps before accumulating).
void UndoHorizontalPredictor(uint8_t* data, size_t rows, size_t rowStride, size_t width, size_t spp,
                             size_t bytesPerSample);

/// Predictor 3 (floating point, 2 / 4 / 8 bytes): per row, a byte-wise accumulation with stride
/// spp, then the byte planes (most significant first) are interleaved back into samples. The
/// result is in host byte order whatever the file's byte order (as libtiff's fpAcc).
void UndoFloatingPointPredictor(uint8_t* data, size_t rows, size_t rowStride, size_t width, size_t spp,
                                size_t bytesPerSample);

/// Reverses the bytes of `count` samples of 2, 4 or 8 bytes (1: no-op).
void ByteSwapSamples(uint8_t* data, size_t count, size_t bytesPerSample);

/// Scalar implementations (the specification, for tests and benchmarks).
namespace reference {
void UndoHorizontalPredictor(uint8_t* data, size_t rows, size_t rowStride, size_t width, size_t spp,
                             size_t bytesPerSample);
void UndoFloatingPointPredictor(uint8_t* data, size_t rows, size_t rowStride, size_t width, size_t spp,
                                size_t bytesPerSample);
void ByteSwapSamples(uint8_t* data, size_t count, size_t bytesPerSample);
}  // namespace reference

}  // namespace ddbt::cog::codecs
