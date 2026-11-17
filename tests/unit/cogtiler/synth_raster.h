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
// Optionally: overviews of half the size each (rounded up, as GDAL's), the pixels of a level from
// levelValue; a photometric interpretation, extra samples and a colour map; an internal mask (of 1 or
// 8 bits a pixel).
inline constexpr double kOriginX = -10240777.0, kOriginY = 5916486.0, kPixel = 0.25;

struct SynthSpec {
    uint32_t width = 256, height = 256;
    uint32_t tile = 0;
    int bands = 1;
    int bytes = 4;  // 1: UInt8, 4: Float32, 8 with `raw`
    uint16_t sampleFormat = 0;  // TIFF SampleFormat (0: 3 for 4 bytes, else 1)
    bool planar = false;
    std::optional<std::string> nodata;
    bool georeferenced = true;  // false: no geotransform and no GeoTIFF keys
    std::function<float(int band0, uint32_t x, uint32_t y)> value = [](int, uint32_t, uint32_t) { return 1.0f; };
    /// The bits of each sample, when set (64-bit integers, which a float does not hold)
    std::function<uint64_t(int band0, uint32_t x, uint32_t y)> raw;
    /// Overviews (each half the previous level, rounded up); their pixels from levelValue (level
    /// 1 is the first overview), else from value at the full-resolution pixel of their corner.
    int overviews = 0;
    std::function<float(int band0, uint32_t x, uint32_t y, int level)> levelValue;
    uint16_t photometric = 1;              // TIFF PhotometricInterpretation (1: min-is-black)
    std::vector<uint16_t> extraSamples;    // ExtraSamples of the bands past the colour ones (0 each by default)
    std::vector<uint16_t> colorMap;        // TIFF ColorMap (3 x 2^bits values), with photometric 3
    /// An internal mask (all levels): 1 = valid.
    std::function<bool(uint32_t x, uint32_t y, int level)> mask;
    /// An internal mask of 8 bits a pixel instead (all levels): its values (0 transparent, 255 opaque).
    std::function<uint8_t(uint32_t x, uint32_t y, int level)> mask8;
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
    const size_t bytes = static_cast<size_t>(s.bytes);
    const size_t bands = static_cast<size_t>(s.bands);
    // One level (0: full resolution) and its mask: w x h pixels in blocks of tw x th.
    const auto writeLevel = [&](int level, uint32_t w, uint32_t h) {
        const uint32_t tw = s.tile != 0 ? s.tile : w, th = s.tile != 0 ? s.tile : h;
        const uint32_t across = (w + tw - 1) / tw, down = (h + th - 1) / th;
        const auto sample = [&](int band0, uint32_t x, uint32_t y) {
            if (level > 0 && s.levelValue) return s.levelValue(band0, x, y, level);
            return s.value(band0, x << level, y << level);
        };
        // One block: the samples of `planeBands` bands from band0 on, pixel interleaved.
        const auto block = [&](uint32_t tx, uint32_t ty, int band0, size_t planeBands) {
            std::vector<uint8_t> out(static_cast<size_t>(tw) * th * planeBands * bytes);
            for (uint32_t y = 0; y < th; ++y)
                for (uint32_t x = 0; x < tw; ++x)
                    for (size_t b = 0; b < planeBands; ++b) {
                        uint8_t* at = out.data() + ((static_cast<size_t>(y) * tw + x) * planeBands + b) * bytes;
                        if (s.raw) {
                            const uint64_t bits = s.raw(band0 + static_cast<int>(b), tx * tw + x, ty * th + y);
                            std::memcpy(at, &bits, bytes);
                            continue;
                        }
                        const float v = sample(band0 + static_cast<int>(b), tx * tw + x, ty * th + y);
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
            t.Long(256, {w}),
            t.Long(257, {h}),
            t.Short(258, std::vector<uint16_t>(bands, static_cast<uint16_t>(s.bytes * 8))),
            t.Short(259, {1}),
            t.Short(262, {s.photometric}),
            t.Short(277, {static_cast<uint16_t>(s.bands)}),
            t.Short(284, {static_cast<uint16_t>(s.planar ? 2 : 1)}),
            t.Long(322, {tw}),
            t.Long(323, {th}),
            t.Long(324, offsets),
            t.Long(325, counts),
            t.Short(339, std::vector<uint16_t>(bands, s.sampleFormat != 0 ? s.sampleFormat
                                                                           : static_cast<uint16_t>(s.bytes == 4 ? 3 : 1))),
        };
        if (level > 0) e.push_back(t.Long(254, {1}));
        if (level == 0 && s.georeferenced) {
            e.push_back(t.Double(33550, {kPixel, kPixel, 0}));
            e.push_back(t.Double(33922, {0, 0, 0, kOriginX, kOriginY, 0}));
            e.push_back(t.Short(34735, {1, 1, 0, 3, 1024, 0, 1, 1, 1025, 0, 1, 1, 3072, 0, 1, 3857}));
        }
        const size_t colours = s.photometric == 2 ? 3 : 1;
        if (!s.extraSamples.empty()) e.push_back(t.Short(338, s.extraSamples));
        else if (bands > colours) e.push_back(t.Short(338, std::vector<uint16_t>(bands - colours, 0)));
        if (!s.colorMap.empty()) e.push_back(t.Short(320, s.colorMap));
        if (s.nodata && level == 0) e.push_back(t.Ascii(42113, *s.nodata));
        t.AddIfd(e);
        if (!s.mask && !s.mask8) return;
        // The mask: 1 bit a pixel, rows padded to a byte (or a byte a pixel), blocks of the image's size.
        std::vector<uint32_t> moff, mcount;
        const uint16_t maskBits = s.mask8 ? 8 : 1;
        const uint32_t rowBytes = s.mask8 ? tw : (tw + 7) / 8;
        for (uint32_t ty = 0; ty < down; ++ty)
            for (uint32_t tx = 0; tx < across; ++tx) {
                std::vector<uint8_t> b(static_cast<size_t>(rowBytes) * th, 0);
                for (uint32_t y = 0; y < th; ++y)
                    for (uint32_t x = 0; x < tw; ++x) {
                        uint8_t& at = b[static_cast<size_t>(y) * rowBytes + (s.mask8 ? x : x / 8)];
                        if (s.mask8) at = s.mask8(tx * tw + x, ty * th + y, level);
                        else if (s.mask(tx * tw + x, ty * th + y, level)) at |= static_cast<uint8_t>(0x80u >> (x % 8));
                    }
                moff.push_back(static_cast<uint32_t>(t.AddData(b)));
                mcount.push_back(static_cast<uint32_t>(b.size()));
            }
        t.AddIfd({t.Long(254, {level > 0 ? 5u : 4u}), t.Long(256, {w}), t.Long(257, {h}), t.Short(258, {maskBits}),
                  t.Short(259, {1}), t.Short(262, {4}), t.Short(277, {1}), t.Long(322, {tw}), t.Long(323, {th}),
                  t.Long(324, moff), t.Long(325, mcount)});
    };
    uint32_t w = s.width, h = s.height;
    for (int level = 0; level <= s.overviews; ++level) {
        writeLevel(level, w, h);
        w = (w + 1) / 2;
        h = (h + 1) / 2;
    }
    std::ofstream(path, std::ios::binary).write(reinterpret_cast<const char*>(t.bytes().data()),
                                                 static_cast<std::streamsize>(t.bytes().size()));
}

}  // namespace ddbt::cog::tiler::testing
