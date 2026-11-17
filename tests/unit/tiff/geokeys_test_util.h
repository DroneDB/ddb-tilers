// SPDX-License-Identifier: MPL-2.0
//
// A GeoKeyDirectory written by hand, for the tests of the CRS read from GeoKeys.
#pragma once

#include <cstdint>
#include <initializer_list>
#include <utility>

#include "cog/tiff/geotiff.h"

namespace ddbt::cog::tiff::testing {

/// GeoTIFF 1.1 keys: SHORT values inline, DOUBLE values in GeoDoubleParams, ASCII values in
/// GeoAsciiParams (as ReadGeoKeys gives them: without the '|' terminator).
inline GeoKeyDirectory Keys(std::initializer_list<std::pair<uint16_t, int>> shorts,
                            std::initializer_list<std::pair<uint16_t, double>> doubles,
                            std::initializer_list<std::pair<uint16_t, const char*>> texts) {
    GeoKeyDirectory d;
    d.version = 1;
    d.revision = 1;
    for (const auto& [id, v] : shorts) d.keys[id] = GeoKey{id, 0, {static_cast<uint16_t>(v)}, {}, {}};
    for (const auto& [id, v] : doubles) d.keys[id] = GeoKey{id, 34736, {}, {v}, {}};
    for (const auto& [id, v] : texts) d.keys[id] = GeoKey{id, 34737, {}, {}, v};
    return d;
}

}  // namespace ddbt::cog::tiff::testing
