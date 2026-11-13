// SPDX-License-Identifier: MPL-2.0
#pragma once

#include <cstddef>
#include <cstdint>

namespace ddbt::encode {

/// One palette entry of a colormapped image (PNG colour type 3).
struct PaletteEntry {
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    uint8_t a = 255;
};

/// Read-only view of an 8-bit, pixel-interleaved tile image; the pixels are not owned.
///
/// channels: 1 grey (or palette indices when palette is set), 2 grey + alpha, 3 RGB, 4 RGBA.
/// stride: bytes from one row to the next (0 = width * channels). Rows need no alignment.
///
/// The alpha summary (alphaKnown, allOpaque, allTransparent) is either given by the caller (who
/// often knows it for free, for example a tile fully inside the raster) or computed by ScanAlpha.
/// The encoders call ScanAlpha themselves when alphaKnown is false.
struct TileImage {
    const uint8_t* pixels = nullptr;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t channels = 0;
    size_t stride = 0;

    /// Colormap of a 1-channel image (pixel values are indices); nullptr for a grey image.
    const PaletteEntry* palette = nullptr;
    uint32_t paletteSize = 0;  // 1..256

    bool alphaKnown = false;
    bool allOpaque = false;       // every pixel has alpha 255 (always true without alpha)
    bool allTransparent = false;  // every pixel has alpha 0 (never true without alpha)

    size_t RowStride() const noexcept {
        return stride != 0 ? stride : static_cast<size_t>(width) * channels;
    }
    const uint8_t* Row(uint32_t y) const noexcept { return pixels + RowStride() * y; }
    bool IsPalette() const noexcept { return palette != nullptr && channels == 1; }
    /// True when the image carries an alpha channel (or a palette with a non-opaque entry).
    bool HasAlphaChannel() const noexcept;
    /// Colour channels of the image: 1 (grey) or 3 (RGB; also palette images).
    uint32_t ColorChannels() const noexcept { return (channels >= 3 || IsPalette()) ? 3u : 1u; }

    /// A view over a tightly packed buffer (stride = width * channels).
    static TileImage Packed(const uint8_t* pixels, uint32_t width, uint32_t height, uint32_t channels) {
        TileImage t;
        t.pixels = pixels;
        t.width = width;
        t.height = height;
        t.channels = channels;
        return t;
    }
};

/// Validates the shape of image (sizes, channels, stride, palette); returns nullptr when it is
/// usable, else a static message.
const char* ValidateTileImage(const TileImage& image) noexcept;

/// Computes allOpaque / allTransparent with a Highway SIMD scan of the alpha channel (palette
/// images: of the alpha of every index) and sets alphaKnown. Images without alpha are opaque.
void ScanAlpha(TileImage& image) noexcept;

}  // namespace ddbt::encode
