// SPDX-License-Identifier: MPL-2.0
//
// The per-point loops of the COPC builder that depend on the Morton encoding: compiled once per
// CPU target (Highway dynamic dispatch), with BMI2 PDEP / PEXT on the targets that have it when
// the running CPU runs them fast (simd::HasFastPdep()), the portable bit spreading otherwise. The
// keys and the winners do not depend on the target.
#pragma once

#include <cstddef>
#include <cstdint>

namespace ddbt::copc::build {
class KeyMapper;
}

namespace ddbt::copc::build::kernels {

/// The grid entry of SelectWinners that holds no candidate.
inline constexpr uint32_t kNoCell = ~uint32_t{0};

/// keys[i] = key of record i (`n` records of `rs` bytes from `recs`); `outside` is set when a
/// point lies outside the cube (left as it is otherwise).
void KeysOfRecords(const KeyMapper& m, const uint8_t* recs, size_t rs, size_t n, uint64_t* keys, bool& outside);

/// SelectWinners (lod.h) on `grid`: cells^3 entries, all kNoCell on entry and on return.
void SelectWinners(const uint64_t* keys, size_t n, int depth, int cells, uint8_t* win, uint32_t* grid);

}  // namespace ddbt::copc::build::kernels
