// SPDX-License-Identifier: MPL-2.0
//
// Per-target loops of the COPC builder (kernels.h). Compiled once per enabled target
// (foreach_target.h re-includes this file); HWY_DYNAMIC_DISPATCH picks the best target at run time
// and the target's code picks PDEP or the portable encoder. The code is plain C++: the compiler
// generates it for each target's instructions. Compiled with -ffp-contract=off -fno-lto
// (sources.cmake), as the key arithmetic of KeyMapper must round as PDAL's.

#include "copc/build/kernels.h"

#include <cstddef>
#include <cstdint>

#include "copc/build/key_mapper.h"
#include "copc/build/morton.h"
#include "core/simd/simd.h"
#include "core/util/endian.h"

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "copc/build/kernels.cpp"
#include "hwy/foreach_target.h"  // IWYU pragma: keep, must come before highway.h
#include "hwy/highway.h"

// PDEP / PEXT in the targets whose instructions include BMI2 (x86 AVX2 and better); evaluated again
// for each target.
#undef DDBT_TARGET_HAS_PDEP
#if DDBT_MORTON_PDEP && HWY_ARCH_X86 && HWY_TARGET <= HWY_AVX2
#define DDBT_TARGET_HAS_PDEP 1
#else
#define DDBT_TARGET_HAS_PDEP 0
#endif

HWY_BEFORE_NAMESPACE();
namespace ddbt::copc::build::kernels {
namespace HWY_NAMESPACE {

template <bool kPdep>
inline uint64_t Encode(uint32_t x, uint32_t y, uint32_t z) noexcept {
#if DDBT_TARGET_HAS_PDEP
    if constexpr (kPdep) return MortonEncodePdep(x, y, z);
#endif
    return MortonEncodePortable(x, y, z);
}

template <bool kPdep>
inline void Decode(uint64_t k, uint32_t& x, uint32_t& y, uint32_t& z) noexcept {
#if DDBT_TARGET_HAS_PDEP
    if constexpr (kPdep) {
        MortonDecodePdep(k, x, y, z);
        return;
    }
#endif
    MortonDecodePortable(k, x, y, z);
}

template <bool kPdep>
void KeysLoop(const KeyMapper& m, const uint8_t* recs, size_t rs, size_t n, uint64_t* keys, bool& outside) {
    for (size_t i = 0; i < n; ++i) {
        const uint8_t* r = recs + i * rs;
        keys[i] = Encode<kPdep>(m.Cell(0, LoadLE<int32_t>(r), outside), m.Cell(1, LoadLE<int32_t>(r + 4), outside),
                                m.Cell(2, LoadLE<int32_t>(r + 8), outside));
    }
}

void KeysOfRecordsImpl(const KeyMapper& m, const uint8_t* recs, size_t rs, size_t n, uint64_t* keys, bool* outside,
                       bool pdep) {
#if DDBT_TARGET_HAS_PDEP
    if (pdep) return KeysLoop<true>(m, recs, rs, n, keys, *outside);
#endif
    (void)pdep;
    KeysLoop<false>(m, recs, rs, n, keys, *outside);
}

/// Squared distance, in units of 1 / (2 g) key cells, from the centre of key cell l to the
/// centre of grid cell c (g grid cells over 2^b key cells, per axis).
inline uint64_t GridDistance(uint32_t lx, uint32_t ly, uint32_t lz, uint32_t cx, uint32_t cy, uint32_t cz, int64_t g,
                             int b) noexcept {
    const int64_t dx = g * (2 * int64_t{lx} + 1) - ((2 * int64_t{cx} + 1) << b);
    const int64_t dy = g * (2 * int64_t{ly} + 1) - ((2 * int64_t{cy} + 1) << b);
    const int64_t dz = g * (2 * int64_t{lz} + 1) - ((2 * int64_t{cz} + 1) << b);
    return static_cast<uint64_t>(dx * dx + dy * dy + dz * dz);
}

template <bool kPdep>
void WinnersLoop(const uint64_t* keys, size_t n, int depth, int cells, uint8_t* win, uint32_t* grid) {
    const size_t g = static_cast<size_t>(cells);
    const int b = kKeyBits - depth;
    const uint64_t mask = (uint64_t{1} << (3 * b)) - 1;
    const int64_t gi = cells;
    auto cellOf = [&](uint64_t key, uint32_t& lx, uint32_t& ly, uint32_t& lz, uint32_t& cx, uint32_t& cy, uint32_t& cz) {
        Decode<kPdep>(key & mask, lx, ly, lz);
        cx = static_cast<uint32_t>((uint64_t{lx} * g) >> b);
        cy = static_cast<uint32_t>((uint64_t{ly} * g) >> b);
        cz = static_cast<uint32_t>((uint64_t{lz} * g) >> b);
        return static_cast<uint32_t>(cx + g * (cy + g * cz));
    };
    uint32_t lx, ly, lz, cx, cy, cz;
    for (size_t i = 0; i < n; ++i) {
        const uint32_t id = cellOf(keys[i], lx, ly, lz, cx, cy, cz);
        uint32_t& best = grid[id];
        if (best == kNoCell) {
            best = static_cast<uint32_t>(i);
            continue;
        }
        const uint64_t d = GridDistance(lx, ly, lz, cx, cy, cz, gi, b);
        uint32_t bx, by, bz, ux, uy, uz;
        cellOf(keys[best], bx, by, bz, ux, uy, uz);
        if (d < GridDistance(bx, by, bz, ux, uy, uz, gi, b)) best = static_cast<uint32_t>(i);
    }
    // Mark the winners and leave the grid empty again.
    for (size_t i = 0; i < n; ++i) {
        const uint32_t id = cellOf(keys[i], lx, ly, lz, cx, cy, cz);
        if (grid[id] == static_cast<uint32_t>(i)) {
            win[i] = 1;
            grid[id] = kNoCell;
        }
    }
}

void SelectWinnersImpl(const uint64_t* keys, size_t n, int depth, int cells, uint8_t* win, uint32_t* grid,
                       bool pdep) {
#if DDBT_TARGET_HAS_PDEP
    if (pdep) return WinnersLoop<true>(keys, n, depth, cells, win, grid);
#endif
    (void)pdep;
    WinnersLoop<false>(keys, n, depth, cells, win, grid);
}

}  // namespace HWY_NAMESPACE
}  // namespace ddbt::copc::build::kernels
HWY_AFTER_NAMESPACE();

#if HWY_ONCE

namespace ddbt::copc::build {

namespace kernels {

HWY_EXPORT(KeysOfRecordsImpl);
HWY_EXPORT(SelectWinnersImpl);

void KeysOfRecords(const KeyMapper& m, const uint8_t* recs, size_t rs, size_t n, uint64_t* keys, bool& outside) {
    HWY_DYNAMIC_DISPATCH(KeysOfRecordsImpl)(m, recs, rs, n, keys, &outside, simd::HasFastPdep());
}

void SelectWinners(const uint64_t* keys, size_t n, int depth, int cells, uint8_t* win, uint32_t* grid) {
    HWY_DYNAMIC_DISPATCH(SelectWinnersImpl)(keys, n, depth, cells, win, grid, simd::HasFastPdep());
}

}  // namespace kernels

}  // namespace ddbt::copc::build

#endif  // HWY_ONCE
