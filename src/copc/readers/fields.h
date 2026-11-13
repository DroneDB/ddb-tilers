// SPDX-License-Identifier: MPL-2.0
//
// Named attributes of non-LAS inputs (PLY properties, XYZ / PTS columns, E57 fields) written
// into output records (formats 6-8) as PDAL's LAS writer writes such dimensions: values
// rounded half away from zero and clamped to the field, return number / count 1 when absent.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include "copc/readers/convert.h"
#include "copc/readers/point_reader.h"
#include "copc/readers/text_util.h"

namespace ddbt::copc::readers {

enum class Field : uint8_t {
    kNone,
    kX,
    kY,
    kZ,
    kRed,
    kGreen,
    kBlue,
    kInfrared,
    kIntensity,
    kClassification,
    kReturnNumber,
    kNumberOfReturns,
    kUserData,
    kPointSourceId,
    kGpsTime,
    kScanAngleRank,
    kScanDirectionFlag,
    kEdgeOfFlightLine,
    kSynthetic,
    kKeyPoint,
    kWithheld,
    kOverlap,
    kScanChannel,
};

/// PDAL dimension names (and common aliases: CloudCompare's, PLY's diffuse_*, scalar_*) to
/// fields; kNone when not kept.
inline Field FieldOfName(const std::string& raw) {
    std::string n = text::Lower(raw);
    if (n.rfind("scalar_", 0) == 0) n = n.substr(7);
    if (n.rfind("diffuse_", 0) == 0) n = n.substr(8);
    n.erase(std::remove(n.begin(), n.end(), '_'), n.end());
    if (n == "x") return Field::kX;
    if (n == "y") return Field::kY;
    if (n == "z") return Field::kZ;
    if (n == "red" || n == "r") return Field::kRed;
    if (n == "green" || n == "g") return Field::kGreen;
    if (n == "blue" || n == "b") return Field::kBlue;
    if (n == "infrared" || n == "nir") return Field::kInfrared;
    if (n == "intensity" || n == "i") return Field::kIntensity;
    if (n == "classification" || n == "class") return Field::kClassification;
    if (n == "returnnumber") return Field::kReturnNumber;
    if (n == "numberofreturns") return Field::kNumberOfReturns;
    if (n == "userdata") return Field::kUserData;
    if (n == "pointsourceid") return Field::kPointSourceId;
    if (n == "gpstime" || n == "time") return Field::kGpsTime;
    if (n == "scananglerank" || n == "scanangle") return Field::kScanAngleRank;
    if (n == "scandirectionflag") return Field::kScanDirectionFlag;
    if (n == "edgeofflightline") return Field::kEdgeOfFlightLine;
    if (n == "synthetic") return Field::kSynthetic;
    if (n == "keypoint") return Field::kKeyPoint;
    if (n == "withheld") return Field::kWithheld;
    if (n == "overlap") return Field::kOverlap;
    if (n == "scanchannel") return Field::kScanChannel;
    return Field::kNone;
}

/// A fresh record of a non-LAS point: zeros, return 1 of 1.
inline void InitRecord(uint8_t* r, size_t size) noexcept {
    std::memset(r, 0, size);
    r[rec::kReturns] = 0x11;
}

inline uint8_t ClampU8(double v) noexcept {
    if (!(v > 0)) return 0;
    if (v >= 255.0) return 255;
    return static_cast<uint8_t>(std::lround(v));
}

/// Sets bits [shift, shift + width) of `b` to round(v) (clamped to the width).
inline void SetBits(uint8_t& b, int shift, unsigned mask, double v) noexcept {
    const unsigned x = std::min<unsigned>(ClampU8(v), mask);
    b = static_cast<uint8_t>((b & ~(mask << shift)) | (x << shift));
}

/// Writes `v` as field `f` of record `r` (format `format`); X / Y / Z go to xyz[0..2].
/// colourScale multiplies colour values (257 for 8-bit colour), intensityScale intensity.
inline void SetField(Field f, double v, uint8_t* r, double* xyz, int format, double colourScale,
                     double intensityScale) noexcept {
    switch (f) {
        case Field::kX: xyz[0] = v; break;
        case Field::kY: xyz[1] = v; break;
        case Field::kZ: xyz[2] = v; break;
        case Field::kRed:
            if (format >= 7) PutU16(r + rec::kRgb, ClampU16(v * colourScale));
            break;
        case Field::kGreen:
            if (format >= 7) PutU16(r + rec::kRgb + 2, ClampU16(v * colourScale));
            break;
        case Field::kBlue:
            if (format >= 7) PutU16(r + rec::kRgb + 4, ClampU16(v * colourScale));
            break;
        case Field::kInfrared:
            if (format == 8) PutU16(r + rec::kNir, ClampU16(v * colourScale));
            break;
        case Field::kIntensity: PutU16(r + rec::kIntensity, ClampU16(v * intensityScale)); break;
        case Field::kClassification: r[rec::kClass] = ClampU8(v); break;
        case Field::kReturnNumber:
            r[rec::kReturns] = static_cast<uint8_t>((r[rec::kReturns] & 0xF0) | std::min<uint8_t>(ClampU8(v), 15));
            break;
        case Field::kNumberOfReturns:
            r[rec::kReturns] = static_cast<uint8_t>((r[rec::kReturns] & 0x0F) | (std::min<uint8_t>(ClampU8(v), 15) << 4));
            break;
        case Field::kUserData: r[rec::kUserData] = ClampU8(v); break;
        case Field::kPointSourceId: PutU16(r + rec::kPointSource, ClampU16(v)); break;
        case Field::kGpsTime: StoreLE<double>(r + rec::kGpsTime, v); break;
        case Field::kScanAngleRank: StoreLE<int16_t>(r + rec::kScanAngle, ScanAngleFromDegrees(static_cast<float>(v))); break;
        case Field::kScanDirectionFlag: SetBits(r[rec::kFlags], 6, 1, v); break;
        case Field::kEdgeOfFlightLine: SetBits(r[rec::kFlags], 7, 1, v); break;
        case Field::kSynthetic: SetBits(r[rec::kFlags], 0, 1, v); break;
        case Field::kKeyPoint: SetBits(r[rec::kFlags], 1, 1, v); break;
        case Field::kWithheld: SetBits(r[rec::kFlags], 2, 1, v); break;
        case Field::kOverlap: SetBits(r[rec::kFlags], 3, 1, v); break;
        case Field::kScanChannel: SetBits(r[rec::kFlags], 4, 3, v); break;
        case Field::kNone: break;
    }
}

/// PDAL name of a field (for the dimension list).
inline const char* FieldName(Field f) noexcept {
    switch (f) {
        case Field::kX: return "X";
        case Field::kY: return "Y";
        case Field::kZ: return "Z";
        case Field::kRed: return "Red";
        case Field::kGreen: return "Green";
        case Field::kBlue: return "Blue";
        case Field::kInfrared: return "Infrared";
        case Field::kIntensity: return "Intensity";
        case Field::kClassification: return "Classification";
        case Field::kReturnNumber: return "ReturnNumber";
        case Field::kNumberOfReturns: return "NumberOfReturns";
        case Field::kUserData: return "UserData";
        case Field::kPointSourceId: return "PointSourceId";
        case Field::kGpsTime: return "GpsTime";
        case Field::kScanAngleRank: return "ScanAngleRank";
        case Field::kScanDirectionFlag: return "ScanDirectionFlag";
        case Field::kEdgeOfFlightLine: return "EdgeOfFlightLine";
        case Field::kSynthetic: return "Synthetic";
        case Field::kKeyPoint: return "KeyPoint";
        case Field::kWithheld: return "Withheld";
        case Field::kOverlap: return "Overlap";
        case Field::kScanChannel: return "ScanChannel";
        case Field::kNone: break;
    }
    return "";
}

/// Colour scale of a source whose colour values reach `maxValue` (8-bit: x 257; values in
/// [0, 1]: x 65535; otherwise 16-bit as stored).
inline double ColourScaleForMax(double maxValue) noexcept {
    if (maxValue <= 1.0) return 65535.0;
    if (maxValue <= 255.0) return 257.0;
    return 1.0;
}

}  // namespace ddbt::copc::readers
