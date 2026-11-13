// SPDX-License-Identifier: MPL-2.0
//
// Per-point kernels of the COPC tiler (Highway, runtime dispatch; scalar references for the
// tests):
//  - DecodeCull: a compact node's LAS integers -> doubles (int * scale + offset, two roundings, as
//    PDAL and DroneDB compute them), keeping the points whose x and y lie in a closed window
//    (readers.copc's / CopcNodeReader's point test), written compacted with their index;
//  - RampRgba: PDAL filters.colorinterp's ramp colour of z (pestel_shades), same operations in
//    the same order, so the same index as PDAL for every z.
// Both are bit-exact with the scalar references on every target (no contraction: the file is
// compiled with -ffp-contract=off).
#pragma once

#include <cstddef>
#include <cstdint>

namespace ddbt::copc::tiler::kernels {

/// The coordinate arrays of a compact node (copc::CompactNode): per axis either uint16 offsets
/// from base[a] or int32 values.
struct NodeView {
    const uint16_t* off16[3] = {nullptr, nullptr, nullptr};
    const int32_t* val32[3] = {nullptr, nullptr, nullptr};
    int32_t base[3] = {0, 0, 0};
    uint32_t count = 0;
};

/// x / y window in the cloud's coordinates, closed on both ends.
struct Window {
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
};

/// LAS scale and offset.
struct Scaling {
    double scale[3] = {1, 1, 1};
    double offset[3] = {0, 0, 0};
};

/// Points [begin, end) of `node` whose scaled x, y lie in `w`: their scaled x, y, z go to xs, ys,
/// zs and their index (in the node) to idx, compacted, in point order. Returns how many. The
/// outputs need room for end - begin values.
size_t DecodeCull(const NodeView& node, size_t begin, size_t end, const Scaling& s, const Window& w, double* xs,
                  double* ys, double* zs, uint32_t* idx);
size_t DecodeCullScalar(const NodeView& node, size_t begin, size_t end, const Scaling& s, const Window& w,
                        double* xs, double* ys, double* zs, uint32_t* idx);

/// Entries of the colour table of RampRgba: the 256 ramp colours, then black (outside the range).
inline constexpr uint32_t kRampOutside = 256;

/// The ramp index of z for filters.colorinterp with minimum `min` and maximum `max`
/// (ColorinterpFilter::processOne): kRampOutside when z < min or z > max, else
/// min(size_t(floor((z - min) / (max - min) * 256)), 255).
uint32_t RampIndexScalar(double z, double min, double max) noexcept;

/// RGBA (R | G << 8 | B << 16 | 255 << 24) of each z: the ramp colour of RampIndexScalar, black
/// for kRampOutside (PDAL leaves the colour of such points unset: 0).
void RampRgba(const double* zs, size_t n, double min, double max, uint32_t* rgba);
void RampRgbaScalar(const double* zs, size_t n, double min, double max, uint32_t* rgba);

/// The 257-entry table (ramp, then black), RGBA.
const uint32_t* RampTable() noexcept;

}  // namespace ddbt::copc::tiler::kernels
