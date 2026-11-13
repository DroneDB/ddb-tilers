// SPDX-License-Identifier: MPL-2.0
//
// Field conversions shared by the readers: LAS formats 0-10 to the output formats 6-8, PDAL's
// scan-angle and overlap rules, colour and intensity scaling.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "copc/readers/point_reader.h"
#include "core/util/endian.h"

namespace ddbt::copc::readers {

/// Formats 0-5: scan angle rank (int8 degrees) as an output scan angle (0.006 degree steps),
/// computed in float as PDAL's LAS writer does.
inline int16_t ScanAngleFromDegrees(float degrees) noexcept {
    const float v = std::round(degrees / .006f);
    if (!(v >= -32768.f)) return -32768;
    if (v > 32767.f) return 32767;
    return static_cast<int16_t>(v);
}

/// The output record bytes 14-17 (returns, flags, class, user data) of a format 0-5 record's
/// bytes 14 (returns / flags) and 15 (classification + bits), as PDAL reads and writes them:
/// classification 12 (legacy overlap) becomes class 0 + the overlap flag.
inline void LegacyReturnsAndClass(uint8_t returnsByte, uint8_t classByte, uint8_t* out) noexcept {
    const uint8_t rn = returnsByte & 7u, nr = (returnsByte >> 3) & 7u;
    const uint8_t sdf = (returnsByte >> 6) & 1u, eof = (returnsByte >> 7) & 1u;
    uint8_t cls = classByte & 0x1Fu;
    const uint8_t synthetic = (classByte >> 5) & 1u, keyPoint = (classByte >> 6) & 1u, withheld = (classByte >> 7) & 1u;
    uint8_t overlap = 0;
    if (cls == 12) {
        cls = 0;
        overlap = 1;
    }
    out[0] = static_cast<uint8_t>(rn | (nr << 4));
    out[1] = static_cast<uint8_t>(synthetic | (keyPoint << 1) | (withheld << 2) | (overlap << 3) | (sdf << 6) | (eof << 7));
    out[2] = cls;
}

/// 8-bit colour value as 16-bit (x 257).
inline uint16_t Color8To16(uint32_t v) noexcept { return static_cast<uint16_t>((v > 255 ? 255 : v) * 257u); }

/// Clamps a double to uint16 with rounding (NaN gives 0).
inline uint16_t ClampU16(double v) noexcept {
    if (!(v > 0)) return 0;
    if (v >= 65535.0) return 65535;
    return static_cast<uint16_t>(std::lround(v));
}

inline void PutU16(uint8_t* p, uint16_t v) noexcept { StoreLE<uint16_t>(p, v); }
inline void PutI32(uint8_t* p, int32_t v) noexcept { StoreLE<int32_t>(p, v); }

}  // namespace ddbt::copc::readers
