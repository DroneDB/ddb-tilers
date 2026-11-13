// SPDX-License-Identifier: MPL-2.0
//
// Block codecs of TIFF / COG files, encode side (the COG builder's writers). Each encoder
// appends a complete compressed block to a caller buffer and keeps per-thread state, so any
// number of threads may encode at once.
//
// The streams are what libtiff writes for the same data, so libtiff / GDAL and our decoders
// read them: LZW with libtiff's code-width "early change" and table reset (no adaptive reset on
// a falling ratio: the stream stays valid, its size differs slightly from libtiff's).
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "core/encode/byte_buffer.h"

namespace ddbt::cog::codecs {

/// Upper bound of the LZW stream for `n` input bytes (12-bit codes per byte, Clear and EOI codes
/// and the table resets included).
size_t LzwEncodedBound(size_t n) noexcept;

/// TIFF LZW (MSB-first codes, 9 to 12 bits, early change, Clear at the start and when the table
/// is full, EOI at the end). Appends the stream to `out`.
void EncodeLzw(std::span<const uint8_t> in, encode::ByteBuffer& out);

}  // namespace ddbt::cog::codecs
