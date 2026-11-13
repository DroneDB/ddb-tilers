// SPDX-License-Identifier: MPL-2.0
#include "core/encode/tile_encoder.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace ddbt::encode {

namespace {

bool EqualsIgnoreCase(std::string_view a, std::string_view b) noexcept {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        const char x = (a[i] >= 'A' && a[i] <= 'Z') ? static_cast<char>(a[i] - 'A' + 'a') : a[i];
        if (x != b[i]) return false;
    }
    return true;
}

JpegOptions JpegFor(const EncodeOptions& options) noexcept {
    JpegOptions j = options.jpeg;
    j.quality = EffectiveQuality(options);
    return j;
}

WebpOptions WebpFor(const EncodeOptions& options) noexcept {
    WebpOptions w = options.webp;
    w.quality = static_cast<float>(EffectiveQuality(options));
    return w;
}

void EncodeAs(const TileImage& image, const EncodeOptions& options, ByteBuffer& out) {
    switch (options.format) {
        case TileFormat::kJpeg: EncodeJpeg(image, JpegFor(options), out); break;
        case TileFormat::kWebp: EncodeWebp(image, WebpFor(options), out); break;
        default: EncodePng(image, options.png, out); break;
    }
}

// Everything the bytes of a blank tile depend on.
using BlankKey = std::tuple<int, uint32_t, uint32_t, uint32_t, int, int, int, int, int>;

BlankKey KeyOf(uint32_t width, uint32_t height, uint32_t colorChannels, const EncodeOptions& options) {
    const int format = static_cast<int>(options.format);
    switch (options.format) {
        case TileFormat::kJpeg: {
            const JpegOptions j = JpegFor(options);
            return {format, width, height, colorChannels, j.quality, static_cast<int>(j.subsampling),
                    (j.optimizeHuffman ? 1 : 0) | (j.fastDct ? 2 : 0) | (j.progressive ? 4 : 0), 0, 0};
        }
        case TileFormat::kWebp: {
            const WebpOptions w = WebpFor(options);
            // WebP always has 3 colour bands
            return {format, width, height, 3u, static_cast<int>(std::lround(w.quality * 1000.0f)), ResolveWebpMethod(w.method),
                    w.lossless ? 1 : 0, w.exact ? 1 : 0, 0};
        }
        default:
            // Always encoded at the strongest level: the key does not depend on options.png
            return {format, width, height, colorChannels, 0, 0, 0, 0, 0};
    }
}

}  // namespace

std::optional<TileFormat> ParseTileFormat(std::string_view name) noexcept {
    if (name.empty() || EqualsIgnoreCase(name, "png") || EqualsIgnoreCase(name, "image/png")) return TileFormat::kPng;
    if (EqualsIgnoreCase(name, "jpeg") || EqualsIgnoreCase(name, "jpg") || EqualsIgnoreCase(name, "image/jpeg") ||
        EqualsIgnoreCase(name, "image/jpg"))
        return TileFormat::kJpeg;
    if (EqualsIgnoreCase(name, "webp") || EqualsIgnoreCase(name, "image/webp")) return TileFormat::kWebp;
    return std::nullopt;
}

const char* TileFormatExtension(TileFormat format) noexcept {
    switch (format) {
        case TileFormat::kJpeg: return "jpg";
        case TileFormat::kWebp: return "webp";
        default: return "png";
    }
}

const char* TileFormatMimeType(TileFormat format) noexcept {
    switch (format) {
        case TileFormat::kJpeg: return "image/jpeg";
        case TileFormat::kWebp: return "image/webp";
        default: return "image/png";
    }
}

int EffectiveQuality(const EncodeOptions& options) noexcept {
    switch (options.format) {
        case TileFormat::kJpeg: return options.quality > 0 ? std::min(options.quality, 100) : 75;
        case TileFormat::kWebp: return options.quality > 0 ? std::min(options.quality, 100) : 80;
        default: return 0;
    }
}

const std::vector<uint8_t>& BlankTileBytes(uint32_t width, uint32_t height, uint32_t colorChannels,
                                           const EncodeOptions& options) {
    if (colorChannels != 1 && colorChannels != 3) throw std::invalid_argument("blank tile: 1 or 3 colour channels");
    static std::mutex mutex;
    static std::map<BlankKey, std::vector<uint8_t>> cache;

    const BlankKey key = KeyOf(width, height, colorChannels, options);
    std::lock_guard<std::mutex> lock(mutex);
    if (const auto it = cache.find(key); it != cache.end()) return it->second;

    // As DroneDB: zeros under alpha 0 (the same bytes for every blank image of this key),
    // with the alpha kept. JPEG turns them white; WebP takes RGBA.
    const uint32_t channels = (options.format == TileFormat::kWebp ? 3u : colorChannels) + 1u;
    const std::vector<uint8_t> zeros(static_cast<size_t>(width) * height * channels, 0);
    TileImage blank = TileImage::Packed(zeros.data(), width, height, channels);
    blank.alphaKnown = true;
    blank.allOpaque = false;
    blank.allTransparent = true;

    EncodeOptions blankOptions = options;
    // Encoded once, so at the smallest size
    blankOptions.png.engine = PngEngine::kDeflate;
    blankOptions.png.level = 12;
    blankOptions.png.filter = PngRowFilter::kNone;
    ByteBuffer bytes;
    EncodeAs(blank, blankOptions, bytes);
    return cache.emplace(key, std::vector<uint8_t>(bytes.data(), bytes.data() + bytes.size())).first->second;
}

void EncodeTile(const TileImage& input, const EncodeOptions& options, ByteBuffer& out) {
    if (const char* err = ValidateTileImage(input)) throw std::invalid_argument(std::string("tile: ") + err);
    TileImage image = input;
    if (!image.alphaKnown) ScanAlpha(image);
    if (image.allTransparent && image.HasAlphaChannel()) {
        const auto& blank = BlankTileBytes(image.width, image.height, image.ColorChannels(), options);
        out.assign(blank.data(), blank.size());
        return;
    }
    EncodeAs(image, options, out);
}

ByteBuffer EncodeTile(const TileImage& image, const EncodeOptions& options) {
    ByteBuffer out;
    EncodeTile(image, options, out);
    return out;
}

}  // namespace ddbt::encode
