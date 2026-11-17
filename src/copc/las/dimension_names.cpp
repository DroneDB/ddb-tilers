// SPDX-License-Identifier: MPL-2.0
#include "copc/las/dimension_names.h"

#include <algorithm>
#include <cctype>

#include "copc/las/point_format.h"

namespace ddbt::copc::las {

std::vector<std::string> PdalDimensionNames(int format, const std::vector<std::string>& extra) {
    std::vector<std::string> d = {"X",           "Y",          "Z",        "Intensity",     "ReturnNumber",
                                  "NumberOfReturns", "ScanDirectionFlag", "EdgeOfFlightLine", "Classification",
                                  "Synthetic",   "KeyPoint",   "Withheld", "Overlap",       "ScanAngleRank",
                                  "UserData",    "PointSourceId"};
    const PointLayout layout = LayoutOf(format);
    if (layout.HasGpsTime()) d.push_back("GpsTime");
    if (layout.extended) d.push_back("ScanChannel");
    if (layout.HasRgb()) {
        d.push_back("Red");
        d.push_back("Green");
        d.push_back("Blue");
    }
    if (layout.HasNir()) d.push_back("Infrared");
    for (const std::string& name : extra) {
        const auto lower = [](char c) { return std::tolower(static_cast<unsigned char>(c)); };
        const auto same = [&](const std::string& known) {
            return known.size() == name.size() &&
                   std::equal(known.begin(), known.end(), name.begin(),
                              [&](char a, char b) { return lower(a) == lower(b); });
        };
        if (std::none_of(d.begin(), d.end(), same)) d.push_back(name);
    }
    return d;
}

std::vector<std::string> PdalExtraDimensionNames(int minor, const std::vector<ExtraBytesDescriptor>& extra) {
    std::vector<std::string> names;
    if (minor < 4) return names;
    const auto fix = [](std::string name) {
        for (size_t i = 0; i < name.size(); ++i) {
            const unsigned char c = static_cast<unsigned char>(name[i]);
            const bool valid = i == 0 ? std::isalpha(c) != 0 : (std::isalnum(c) != 0 || c == '_' || c == ' ');
            if (!valid) name[i] = '_';
        }
        return name;
    };
    for (const ExtraBytesDescriptor& e : extra) {
        if (e.dataType == 0) continue;
        if (e.elements <= 1) {
            names.push_back(fix(e.name));
        } else {
            for (size_t i = 0; i < e.elements; ++i) names.push_back(fix(ExtraBytesElementName(e.name, i)));
        }
    }
    return names;
}

}  // namespace ddbt::copc::las
