// SPDX-License-Identifier: MPL-2.0
//
// LAS point data record formats 0-10 (LAS 1.4 R15, section 2.6-2.16): base record sizes and
// field offsets. Extra bytes follow the base record (record length - base size).
#pragma once

#include <cstdint>

namespace ddbt::copc::las {

inline constexpr int kMaxPointFormat = 10;

/// Byte offset of each field in a record; -1 when the format lacks it.
struct PointLayout {
    uint8_t format = 0;
    uint16_t baseSize = 0;
    bool extended = false;  // formats 6-10 (16-bit returns byte pair, 16-bit scan angle, flags byte)
    int16_t x = 0, y = 4, z = 8;
    int16_t intensity = 12;
    int16_t returns = 14;         // 0-5: return (3 bits) / count (3) / scan dir / edge; 6-10: return (4) / count (4)
    int16_t flags = -1;           // 6-10: class flags (4) / scanner channel (2) / scan dir / edge
    int16_t classification = 15;  // 0-5: class (5 bits) + synthetic / key-point / withheld
    int16_t scanAngle = 16;       // 0-5: int8 rank in degrees; 6-10: int16 in 0.006 degree steps
    int16_t userData = 17;
    int16_t pointSourceId = 18;
    int16_t gpsTime = -1;
    int16_t rgb = -1;
    int16_t nir = -1;
    int16_t wavePacket = -1;      // 29 bytes: descriptor index, offset, size, location, xt, yt, zt

    bool HasGpsTime() const noexcept { return gpsTime >= 0; }
    bool HasRgb() const noexcept { return rgb >= 0; }
    bool HasNir() const noexcept { return nir >= 0; }
    bool HasWavePacket() const noexcept { return wavePacket >= 0; }
};

/// Layout of format `f` (0..10). Precondition: 0 <= f <= kMaxPointFormat.
constexpr PointLayout LayoutOf(int f) noexcept {
    PointLayout l;
    l.format = static_cast<uint8_t>(f);
    if (f <= 5) {
        uint16_t size = 20;
        if (f == 1 || f == 3 || f == 4 || f == 5) {
            l.gpsTime = static_cast<int16_t>(size);
            size += 8;
        }
        if (f == 2 || f == 3 || f == 5) {
            l.rgb = static_cast<int16_t>(size);
            size += 6;
        }
        if (f == 4 || f == 5) {
            l.wavePacket = static_cast<int16_t>(size);
            size += 29;
        }
        l.baseSize = size;
        return l;
    }
    l.extended = true;
    l.flags = 15;
    l.classification = 16;
    l.userData = 17;
    l.scanAngle = 18;
    l.pointSourceId = 20;
    l.gpsTime = 22;
    uint16_t size = 30;
    if (f == 7 || f == 8 || f == 10) {
        l.rgb = static_cast<int16_t>(size);
        size += 6;
    }
    if (f == 8 || f == 10) {
        l.nir = static_cast<int16_t>(size);
        size += 2;
    }
    if (f == 9 || f == 10) {
        l.wavePacket = static_cast<int16_t>(size);
        size += 29;
    }
    l.baseSize = size;
    return l;
}

/// Base record size of format `f` (0..10): 20, 28, 26, 34, 57, 63, 30, 36, 38, 59, 67.
constexpr uint16_t BaseRecordSize(int f) noexcept { return LayoutOf(f).baseSize; }

static_assert(BaseRecordSize(0) == 20 && BaseRecordSize(1) == 28 && BaseRecordSize(2) == 26 &&
              BaseRecordSize(3) == 34 && BaseRecordSize(4) == 57 && BaseRecordSize(5) == 63 &&
              BaseRecordSize(6) == 30 && BaseRecordSize(7) == 36 && BaseRecordSize(8) == 38 &&
              BaseRecordSize(9) == 59 && BaseRecordSize(10) == 67);

}  // namespace ddbt::copc::las
