// SPDX-License-Identifier: MPL-2.0
//
// The octree cube of a build and the Morton key of a point: cell = floor((p - cubeMin) *
// 2^21 / (2 halfSize)) per axis (clamped to [0, 2^21 - 1]), p = X * scale + offset rounded as
// PDAL rounds it (no fused multiply-add). A node of depth d holds the points whose key prefix
// at d is its Morton code, i.e. (up to the rounding of p) the points inside PDAL's
// Key::bounds of the node.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include "copc/build/kernels.h"
#include "copc/build/morton.h"
#include "copc/copc/hierarchy.h"
#include "core/util/endian.h"

namespace ddbt::copc::build {

struct Cube {
    std::array<double, 3> center{};
    double halfSize = 0;
    double Min(size_t a) const noexcept { return center[a] - halfSize; }
    double Max(size_t a) const noexcept { return center[a] + halfSize; }
};

class KeyMapper {
public:
    KeyMapper() = default;
    KeyMapper(const Cube& cube, const std::array<double, 3>& scale, const std::array<double, 3>& offset) noexcept
        : scale_(scale), offset_(offset) {
        for (size_t a = 0; a < 3; ++a) {
            min_[a] = cube.Min(a);
            max_[a] = cube.Max(a);
        }
        k_ = static_cast<double>(kKeyCells) / (2 * cube.halfSize);
        // PDAL's cell width (Key::bounds: the root's x extent / 2^d, for every axis).
        cw_ = (max_[0] - min_[0]) / static_cast<double>(kKeyCells);
    }

    /// Cell (0 .. 2^21 - 1) of integer coordinate v on axis a; `outside` is set when the point
    /// lies outside the cube. The cell is the one whose bounds, computed as PDAL's Key::bounds
    /// computes them (min + width * i, unfused; the root's own bounds at the edges), contain the
    /// point: node bounds at any depth are cell bounds of depth 21 (the products are the same
    /// reals rounded once), so every point lies inside its node by that arithmetic.
    uint32_t Cell(size_t a, int32_t v, bool& outside) const noexcept {
        const double p = MulAddUnfused(static_cast<double>(v), scale_[a], offset_[a]);
        const double t = (p - min_[a]) * k_;
        uint32_t c;
        if (!(t >= 0)) {
            if (p < min_[a]) outside = true;
            c = 0;
        } else if (t >= static_cast<double>(kKeyCells)) {
            if (p > max_[a]) outside = true;
            c = kKeyCells - 1;
        } else {
            c = static_cast<uint32_t>(t);
        }
        while (c > 0 && p < Lower(a, c)) --c;
        while (c < kKeyCells - 1 && p > Lower(a, c + 1)) ++c;
        return c;
    }

    uint64_t Key(int32_t x, int32_t y, int32_t z, bool& outside) const noexcept {
        return MortonEncode(Cell(0, x, outside), Cell(1, y, outside), Cell(2, z, outside));
    }

    /// Key of an output record (X / Y / Z little-endian int32 at offsets 0, 4, 8).
    uint64_t KeyOfRecord(const uint8_t* r, bool& outside) const noexcept {
        return Key(LoadLE<int32_t>(r), LoadLE<int32_t>(r + 4), LoadLE<int32_t>(r + 8), outside);
    }

    /// keys[i] = KeyOfRecord of record i (`n` records `rs` bytes apart), by the loop compiled for
    /// the running CPU (kernels.h).
    void KeysOfRecords(const uint8_t* recs, size_t rs, size_t n, uint64_t* keys, bool& outside) const {
        kernels::KeysOfRecords(*this, recs, rs, n, keys, outside);
    }

private:
    /// Lower bound of cell c on axis a (PDAL's arithmetic).
    double Lower(size_t a, uint32_t c) const noexcept {
        return c == 0 ? min_[a] : MulAddUnfused(cw_, static_cast<double>(c), min_[a]);
    }

    std::array<double, 3> scale_{1, 1, 1}, offset_{}, min_{}, max_{};
    double k_ = 1;
    double cw_ = 1;
};

}  // namespace ddbt::copc::build
