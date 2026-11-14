// SPDX-License-Identifier: MPL-2.0
#include "cog/build/source.h"

#include <algorithm>
#include <cmath>

#include "cog/build/tiff_source.h"
#include "core/io/byte_source.h"

namespace ddbt::cog::build {

Expected<std::unique_ptr<RasterSource>> OpenRasterSource(const std::string& path) {
    auto t = TiffSource::Open(path);
    if (!t.ok()) return t.status();
    return std::unique_ptr<RasterSource>(std::move(*t));
}

writer::RawTag AsciiTag(uint16_t tag, const std::string& value) {
    writer::RawTag t{tag, 2, value.size() + 1, {}};
    t.data.assign(value.begin(), value.end());
    t.data.push_back(0);
    return t;
}

writer::RawTag ShortTag(uint16_t tag, uint16_t value) {
    return writer::RawTag{tag, 3, 1, {static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8)}};
}

writer::RawTag RationalTag(uint16_t tag, double value) {
    // libtiff stores the float GDAL sets as a rational: integers as n / 1.
    uint32_t num, den = 1;
    if (value >= 0 && value == std::floor(value) && value < 4294967296.0) {
        num = static_cast<uint32_t>(value);
    } else {
        den = 1000000;
        num = static_cast<uint32_t>(std::max(0.0, std::min(4294967295.0, std::round(value * den))));
    }
    writer::RawTag t{tag, 5, 1, std::vector<uint8_t>(8)};
    for (int i = 0; i < 4; ++i) {
        t.data[static_cast<size_t>(i)] = static_cast<uint8_t>(num >> (8 * i));
        t.data[static_cast<size_t>(4 + i)] = static_cast<uint8_t>(den >> (8 * i));
    }
    return t;
}

}  // namespace ddbt::cog::build
