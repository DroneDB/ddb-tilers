// SPDX-License-Identifier: MPL-2.0
#include "cog/tiff/color_interp.h"

#include <array>

namespace ddbt::cog::tiff {

namespace {

// GDALGetColorInterpretationName of each value, in GDAL's order
constexpr std::array<const char*, 40> kNames = {
    "Undefined", "Gray",          "Palette",       "Red",           "Green",       "Blue",    "Alpha",
    "Hue",       "Saturation",    "Lightness",     "Cyan",          "Magenta",     "Yellow",  "Black",
    "YCbCr_Y",   "YCbCr_Cb",      "YCbCr_Cr",      "Pan",           "Coastal",     "RedEdge", "NIR",
    "SWIR",      "MWIR",          "LWIR",          "TIR",           "OtherIR",     "IR_Reserved_1",
    "IR_Reserved_2", "IR_Reserved_3", "IR_Reserved_4", "SAR_Ka",    "SAR_K",       "SAR_Ku",  "SAR_X",
    "SAR_C",     "SAR_S",         "SAR_L",         "SAR_P",         "SAR_Reserved_1", "SAR_Reserved_2",
};

bool EqualNoCase(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        char x = a[i], y = b[i];
        if (x >= 'a' && x <= 'z') x = static_cast<char>(x - 'a' + 'A');
        if (y >= 'a' && y <= 'z') y = static_cast<char>(y - 'a' + 'A');
        if (x != y) return false;
    }
    return true;
}

}  // namespace

const char* ColorInterpName(ColorInterp ci) noexcept {
    const auto i = static_cast<size_t>(ci);
    return i < kNames.size() ? kNames[i] : kNames[0];
}

ColorInterp ColorInterpByName(std::string_view name) noexcept {
    for (size_t i = 0; i < kNames.size(); ++i)
        if (EqualNoCase(kNames[i], name)) return static_cast<ColorInterp>(i);
    if (EqualNoCase(name, "grey")) return ColorInterp::kGray;  // GDAL accepts the British spelling
    return ColorInterp::kUndefined;
}

}  // namespace ddbt::cog::tiff
