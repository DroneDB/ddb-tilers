// SPDX-License-Identifier: MPL-2.0
//
// 63-bit Morton keys of the COPC builder: 21 bits per axis, x in the lowest bit of each triple
// (so the 3 bits below a node's prefix are its child direction as COPC numbers them: x = bit 0,
// y = bit 1, z = bit 2). Two encoders that give the same keys: the classic magic-number spreading
// (any CPU) and BMI2 PDEP / PEXT (x86-64 CPUs with BMI2). MortonEncode / MortonDecode use PDEP when
// the whole build targets BMI2; the hot loops of the builder (KeyMapper::KeysOfRecords,
// SelectWinners) are compiled per CPU target and pick PDEP at run time (simd::HasFastPdep()).
#pragma once

#include <cstdint>

#include "core/simd/target_attr.h"

#if DDBT_X86_64
#include <immintrin.h>
#define DDBT_MORTON_PDEP 1
#else
#define DDBT_MORTON_PDEP 0
#endif

namespace ddbt::copc::build {

inline constexpr int kKeyBits = 21;  // bits per axis
inline constexpr int kMaxKeyDepth = kKeyBits;
inline constexpr uint32_t kKeyCells = 1u << kKeyBits;
inline constexpr uint64_t kMortonMaskX = 0x1249249249249249ull;  // bits 0, 3, 6, ... 60

namespace detail {
constexpr uint64_t Spread3(uint64_t v) noexcept {
    v &= 0x1FFFFFull;
    v = (v | (v << 32)) & 0x001F00000000FFFFull;
    v = (v | (v << 16)) & 0x001F0000FF0000FFull;
    v = (v | (v << 8)) & 0x100F00F00F00F00Full;
    v = (v | (v << 4)) & 0x10C30C30C30C30C3ull;
    v = (v | (v << 2)) & 0x1249249249249249ull;
    return v;
}
constexpr uint32_t Compact3(uint64_t v) noexcept {
    v &= 0x1249249249249249ull;
    v = (v ^ (v >> 2)) & 0x10C30C30C30C30C3ull;
    v = (v ^ (v >> 4)) & 0x100F00F00F00F00Full;
    v = (v ^ (v >> 8)) & 0x001F0000FF0000FFull;
    v = (v ^ (v >> 16)) & 0x001F00000000FFFFull;
    v = (v ^ (v >> 32)) & 0x1FFFFFull;
    return static_cast<uint32_t>(v);
}
}  // namespace detail

/// Portable encoder (also the reference of the BMI2 path in tests).
constexpr uint64_t MortonEncodePortable(uint32_t x, uint32_t y, uint32_t z) noexcept {
    return detail::Spread3(x) | (detail::Spread3(y) << 1) | (detail::Spread3(z) << 2);
}
constexpr void MortonDecodePortable(uint64_t k, uint32_t& x, uint32_t& y, uint32_t& z) noexcept {
    x = detail::Compact3(k);
    y = detail::Compact3(k >> 1);
    z = detail::Compact3(k >> 2);
}

#if DDBT_MORTON_PDEP
/// BMI2 encoder: call it only from code compiled for BMI2 that runs after a CPU check.
DDBT_TARGET("bmi2") inline uint64_t MortonEncodePdep(uint32_t x, uint32_t y, uint32_t z) noexcept {
    return _pdep_u64(x, kMortonMaskX) | _pdep_u64(y, kMortonMaskX << 1) | _pdep_u64(z, kMortonMaskX << 2);
}
DDBT_TARGET("bmi2") inline void MortonDecodePdep(uint64_t k, uint32_t& x, uint32_t& y, uint32_t& z) noexcept {
    x = static_cast<uint32_t>(_pext_u64(k, kMortonMaskX));
    y = static_cast<uint32_t>(_pext_u64(k, kMortonMaskX << 1));
    z = static_cast<uint32_t>(_pext_u64(k, kMortonMaskX << 2));
}
#endif

/// The encoder a caller compiled for the build's baseline can use: PDEP when the baseline has BMI2.
inline uint64_t MortonEncode(uint32_t x, uint32_t y, uint32_t z) noexcept {
#if DDBT_MORTON_PDEP && defined(__BMI2__)
    return MortonEncodePdep(x, y, z);
#else
    return MortonEncodePortable(x, y, z);
#endif
}

inline void MortonDecode(uint64_t k, uint32_t& x, uint32_t& y, uint32_t& z) noexcept {
#if DDBT_MORTON_PDEP && defined(__BMI2__)
    MortonDecodePdep(k, x, y, z);
#else
    MortonDecodePortable(k, x, y, z);
#endif
}

/// Prefix of `key` at depth d (0..21): the node's Morton code.
constexpr uint64_t PrefixAt(uint64_t key, int d) noexcept { return key >> (3 * (kKeyBits - d)); }

}  // namespace ddbt::copc::build
