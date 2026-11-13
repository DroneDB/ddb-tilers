// SPDX-License-Identifier: Apache-2.0
// ddbt fork (new file): the layers of a LAZ point 6-10 chunk, as bits of a layer mask.
//
// A layered chunk stores the first point raw, the point count, the byte size of every layer,
// then the layers one after the other. Each layer has its own arithmetic decoder and models,
// so a decoder may skip the bytes of the layers it does not need. Which ones can be skipped
// without changing the others: the XY layer
// carries the changed-values symbol (incl. the gps-time-changed bit and the scanner channel)
// and the return numbers that every other layer's contexts use, so XY is always decoded.
#pragma once

#include <cstdint>

namespace lazperf
{
namespace layer
{

constexpr uint32_t xy = 1u << 0;            // changed values, channel, returns, X, Y
constexpr uint32_t z = 1u << 1;
constexpr uint32_t classification = 1u << 2;
constexpr uint32_t flags = 1u << 3;          // classification flags, scan direction, edge
constexpr uint32_t intensity = 1u << 4;
constexpr uint32_t scan_angle = 1u << 5;
constexpr uint32_t user_data = 1u << 6;
constexpr uint32_t point_source = 1u << 7;
constexpr uint32_t gps_time = 1u << 8;
constexpr uint32_t rgb = 1u << 9;
constexpr uint32_t nir = 1u << 10;
constexpr uint32_t extra_bytes = 1u << 11;
constexpr uint32_t all = (1u << 12) - 1;

// The layers of the point14 item (field_point14).
constexpr uint32_t point14 = xy | z | classification | flags | intensity | scan_angle |
    user_data | point_source | gps_time;

} // namespace layer
} // namespace lazperf
