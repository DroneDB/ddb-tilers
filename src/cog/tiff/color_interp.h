// SPDX-License-Identifier: MPL-2.0
//
// GDAL's colour interpretations (GDALColorInterp, GDAL 3.13): the values, their names and the
// lookup by name.
#pragma once

#include <cstdint>
#include <string_view>

namespace ddbt::cog::tiff {

/// GDALColorInterp, with GDAL's values.
enum class ColorInterp : uint8_t {
    kUndefined = 0,
    kGray = 1,
    kPalette = 2,
    kRed = 3,
    kGreen = 4,
    kBlue = 5,
    kAlpha = 6,
    kHue = 7,
    kSaturation = 8,
    kLightness = 9,
    kCyan = 10,
    kMagenta = 11,
    kYellow = 12,
    kBlack = 13,
    kYCbCrY = 14,
    kYCbCrCb = 15,
    kYCbCrCr = 16,
    kPan = 17,
    kCoastal = 18,
    kRedEdge = 19,
    kNir = 20,
    kSwir = 21,
    kMwir = 22,
    kLwir = 23,
    kTir = 24,
    kOtherIr = 25,
    kIrReserved1 = 26,
    kIrReserved2 = 27,
    kIrReserved3 = 28,
    kIrReserved4 = 29,
    kSarKa = 30,
    kSarK = 31,
    kSarKu = 32,
    kSarX = 33,
    kSarC = 34,
    kSarS = 35,
    kSarL = 36,
    kSarP = 37,
    kSarReserved1 = 38,
    kSarReserved2 = 39,
};

/// GDALGetColorInterpretationName: "Undefined", "Gray", "RedEdge", "SAR_Ka", ...
const char* ColorInterpName(ColorInterp ci) noexcept;

/// GDALGetColorInterpretationByName: any of the names (case-insensitive) or "grey"; kUndefined for
/// another name.
ColorInterp ColorInterpByName(std::string_view name) noexcept;

}  // namespace ddbt::cog::tiff
