// SPDX-License-Identifier: MPL-2.0
#include "core/encode/tile_image.h"

#include <cstddef>
#include <cstdint>

#include "core/encode/kernels.h"

namespace ddbt::encode {

bool TileImage::HasAlphaChannel() const noexcept {
    if (channels == 2 || channels == 4) return true;
    if (IsPalette()) {
        for (uint32_t i = 0; i < paletteSize; ++i) {
            if (palette[i].a != 255) return true;
        }
    }
    return false;
}

const char* ValidateTileImage(const TileImage& image) noexcept {
    if (image.pixels == nullptr) return "image has no pixels";
    if (image.width == 0 || image.height == 0) return "image has an empty size";
    // Tiles are small; the bound keeps every size computation far from overflow.
    if (image.width > 65535 || image.height > 65535) return "image is larger than 65535 pixels";
    if (image.channels < 1 || image.channels > 4) return "image must have 1 to 4 channels";
    if (image.stride != 0 && image.stride < static_cast<size_t>(image.width) * image.channels)
        return "image stride is shorter than a row";
    if (image.palette != nullptr) {
        if (image.channels != 1) return "a palette image must have 1 channel";
        if (image.paletteSize < 1 || image.paletteSize > 256) return "palette must have 1 to 256 entries";
    }
    return nullptr;
}

void ScanAlpha(TileImage& image) noexcept {
    image.alphaKnown = true;
    image.allOpaque = true;
    image.allTransparent = false;
    if (image.IsPalette()) {
        if (!image.HasAlphaChannel()) return;
        uint8_t alphaOf[256];
        for (uint32_t i = 0; i < 256; ++i) alphaOf[i] = i < image.paletteSize ? image.palette[i].a : uint8_t{255};
        uint8_t mn = 255, mx = 0;
        for (uint32_t y = 0; y < image.height && !(mn < 255 && mx > 0); ++y) {
            const uint8_t* row = image.Row(y);
            for (uint32_t x = 0; x < image.width; ++x) {
                const uint8_t a = alphaOf[row[x]];
                mn = a < mn ? a : mn;
                mx = a > mx ? a : mx;
            }
        }
        image.allOpaque = mn == 255;
        image.allTransparent = mx == 0;
        return;
    }
    if (image.channels != 2 && image.channels != 4) return;
    uint8_t mn = 255, mx = 0;
    // Stops at the first row that proves the alpha mixed (edge tiles: early).
    for (uint32_t y = 0; y < image.height && !(mn < 255 && mx > 0); ++y) {
        kernels::AlphaRange(image.Row(y), image.width, image.channels, mn, mx);
    }
    image.allOpaque = mn == 255;
    image.allTransparent = mx == 0;
}

}  // namespace ddbt::encode
