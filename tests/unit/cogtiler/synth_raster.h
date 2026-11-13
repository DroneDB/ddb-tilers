// SPDX-License-Identifier: MPL-2.0
//
// Synthetic GeoTIFFs for the COG tiler tests.
#pragma once

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "../tiff/tiff_test_util.h"

namespace ddbt::cog::tiler::testing {

// A synthetic uncompressed GeoTIFF in EPSG:3857: w x h pixels in tiles of `tile` x `tile`
// (multiples of 16; 0: one tile of w x h, which must then be multiples of 16), `bands` samples
// of 1 (UInt8) or 4 (Float32) bytes, pixel or band interleaved, 0.25 m pixels with the
// upper-left corner at (kOriginX, kOriginY). Tile padding past the image holds value(x, y) too.
inline constexpr double kOriginX = -10240777.0, kOriginY = 5916486.0, kPixel = 0.25;

struct SynthSpec {
    uint32_t width = 256, height = 256;
    uint32_t tile = 0;
    int bands = 1;
    int bytes = 4;  // 1: UInt8, 4: Float32
    bool planar = false;
    std::optional<std::string> nodata;
    std::function<float(int band0, uint32_t x, uint32_t y)> value = [](int, uint32_t, uint32_t) { return 1.0f; };
};

inline SynthSpec Spec(uint32_t size, int bands, int bytes = 4) {
    SynthSpec s;
    s.width = s.height = size;
    s.bands = bands;
    s.bytes = bytes;
    return s;
}

inline void WriteSynth(const std::filesystem::path& path, const SynthSpec& s) {
    using ddbt::cog::tiff::testing::TiffBuilder;
    TiffBuilder t(false, false);
    const uint32_t tw = s.tile != 0 ? s.tile : s.width, th = s.tile != 0 ? s.tile : s.height;
    const uint32_t across = (s.width + tw - 1) / tw, down = (s.height + th - 1) / th;
    const size_t bytes = static_cast<size_t>(s.bytes);
    const size_t bands = static_cast<size_t>(s.bands);
    // One block: the samples of `planeBands` bands from band0 on, pixel interleaved.
    const auto block = [&](uint32_t tx, uint32_t ty, int band0, size_t planeBands) {
        std::vector<uint8_t> out(static_cast<size_t>(tw) * th * planeBands * bytes);
        for (uint32_t y = 0; y < th; ++y)
            for (uint32_t x = 0; x < tw; ++x)
                for (size_t b = 0; b < planeBands; ++b) {
                    const float v = s.value(band0 + static_cast<int>(b), tx * tw + x, ty * th + y);
                    uint8_t* at = out.data() + ((static_cast<size_t>(y) * tw + x) * planeBands + b) * bytes;
                    if (s.bytes == 1) *at = static_cast<uint8_t>(v);
                    else std::memcpy(at, &v, 4);
                }
        return out;
    };
    std::vector<uint32_t> offsets, counts;
    for (int plane = 0; plane < (s.planar ? s.bands : 1); ++plane)
        for (uint32_t ty = 0; ty < down; ++ty)
            for (uint32_t tx = 0; tx < across; ++tx) {
                const std::vector<uint8_t> b = block(tx, ty, plane, s.planar ? 1 : bands);
                offsets.push_back(static_cast<uint32_t>(t.AddData(b)));
                counts.push_back(static_cast<uint32_t>(b.size()));
            }
    std::vector<TiffBuilder::Entry> e = {
        t.Long(256, {s.width}),
        t.Long(257, {s.height}),
        t.Short(258, std::vector<uint16_t>(bands, static_cast<uint16_t>(s.bytes * 8))),
        t.Short(259, {1}),
        t.Short(262, {1}),
        t.Short(277, {static_cast<uint16_t>(s.bands)}),
        t.Short(284, {static_cast<uint16_t>(s.planar ? 2 : 1)}),
        t.Long(322, {tw}),
        t.Long(323, {th}),
        t.Long(324, offsets),
        t.Long(325, counts),
        t.Short(339, std::vector<uint16_t>(bands, static_cast<uint16_t>(s.bytes == 4 ? 3 : 1))),
        t.Double(33550, {kPixel, kPixel, 0}),
        t.Double(33922, {0, 0, 0, kOriginX, kOriginY, 0}),
        t.Short(34735, {1, 1, 0, 3, 1024, 0, 1, 1, 1025, 0, 1, 1, 3072, 0, 1, 3857}),
    };
    if (s.bands > 1) e.push_back(t.Short(338, std::vector<uint16_t>(bands - 1, 0)));
    if (s.nodata) e.push_back(t.Ascii(42113, *s.nodata));
    t.AddIfd(e);
    std::ofstream(path, std::ios::binary).write(reinterpret_cast<const char*>(t.bytes().data()),
                                                 static_cast<std::streamsize>(t.bytes().size()));
}

}  // namespace ddbt::cog::tiler::testing
