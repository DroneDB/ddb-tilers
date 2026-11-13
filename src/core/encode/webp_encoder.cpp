// SPDX-License-Identifier: MPL-2.0
#include "core/encode/webp_encoder.h"

#include <webp/encode.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>
#include <stdexcept>
#include <string>

#include "core/encode/kernels.h"

namespace ddbt::encode {

namespace {

// The WebPPicture of the thread: libwebp reuses nothing across imports (each import reallocates
// the YUV planes), but keeping it saves the init and frees the planes only at thread exit.
class WebpPictureHolder {
public:
    WebpPictureHolder() {
        if (!WebPPictureInit(&picture_)) throw std::runtime_error("WebP: WebPPictureInit failed (ABI mismatch)");
    }
    WebpPictureHolder(const WebpPictureHolder&) = delete;
    WebpPictureHolder& operator=(const WebpPictureHolder&) = delete;
    ~WebpPictureHolder() { WebPPictureFree(&picture_); }
    WebPPicture& Get() noexcept { return picture_; }

private:
    WebPPicture picture_{};
};

thread_local ByteBuffer tlsWebpPixels;

struct WriterState {
    ByteBuffer* out = nullptr;
    bool failed = false;
};

int AppendWriter(const uint8_t* data, size_t size, const WebPPicture* picture) {
    auto* state = static_cast<WriterState*>(picture->custom_ptr);
    try {
        state->out->append(data, size);
        return 1;
    } catch (const std::bad_alloc&) {
        state->failed = true;
        return 0;
    }
}

const char* WebpErrorText(WebPEncodingError e) {
    switch (e) {
        case VP8_ENC_ERROR_OUT_OF_MEMORY: return "out of memory";
        case VP8_ENC_ERROR_BITSTREAM_OUT_OF_MEMORY: return "out of memory while flushing bits";
        case VP8_ENC_ERROR_NULL_PARAMETER: return "null parameter";
        case VP8_ENC_ERROR_INVALID_CONFIGURATION: return "invalid configuration";
        case VP8_ENC_ERROR_BAD_DIMENSION: return "bad dimension";
        case VP8_ENC_ERROR_PARTITION0_OVERFLOW: return "partition 0 overflow";
        case VP8_ENC_ERROR_PARTITION_OVERFLOW: return "partition overflow";
        case VP8_ENC_ERROR_BAD_WRITE: return "bad write";
        case VP8_ENC_ERROR_FILE_TOO_BIG: return "file too big";
        case VP8_ENC_ERROR_USER_ABORT: return "user abort";
        default: return "unknown error";
    }
}

int MethodFromEnvironment() noexcept {
    if (const char* e = std::getenv("DDBT_WEBP_METHOD"); e != nullptr && e[0] >= '0' && e[0] <= '6' && e[1] == '\0')
        return e[0] - '0';
    return 1;
}

}  // namespace

int DefaultWebpMethod() noexcept {
    static const int method = MethodFromEnvironment();
    return method;
}

void EncodeWebp(const TileImage& input, const WebpOptions& options, ByteBuffer& out) {
    if (const char* err = ValidateTileImage(input)) throw std::invalid_argument(std::string("WebP: ") + err);
    if (input.width > WEBP_MAX_DIMENSION || input.height > WEBP_MAX_DIMENSION)
        throw std::invalid_argument("WebP: image is larger than 16383 pixels");
    if (!(options.quality >= 0.0f && options.quality <= 100.0f))
        throw std::invalid_argument("WebP: quality must be 0 to 100");
    if (options.method < -1 || options.method > 6) throw std::invalid_argument("WebP: method must be 0 to 6 (or -1)");

    WebPConfig config;
    if (!WebPConfigInit(&config)) throw std::runtime_error("WebP: WebPConfigInit failed (ABI mismatch)");
    config.quality = options.quality;
    config.method = ResolveWebpMethod(options.method);
    config.lossless = options.lossless ? 1 : 0;
    config.exact = options.exact ? 1 : 0;
    if (!WebPValidateConfig(&config)) throw std::invalid_argument("WebP: invalid configuration");

    TileImage image = input;
    if (!image.alphaKnown) ScanAlpha(image);
    const uint32_t w = image.width;
    const uint32_t h = image.height;
    const size_t pixels = static_cast<size_t>(w) * h;

    // RGB / RGBA rows to import: the view itself when it already is RGB or RGBA
    const uint8_t* src = image.pixels;
    size_t stride = image.RowStride();
    uint32_t channels = image.channels;
    if (image.IsPalette()) {
        uint32_t lut[256];
        for (uint32_t i = 0; i < 256; ++i) {
            const PaletteEntry e = i < image.paletteSize ? image.palette[i] : PaletteEntry{};
            lut[i] = uint32_t{e.r} | (uint32_t{e.g} << 8) | (uint32_t{e.b} << 16) | (uint32_t{e.a} << 24);
        }
        tlsWebpPixels.resize(pixels * 4);
        uint8_t* dst = tlsWebpPixels.data();
        for (uint32_t y = 0; y < h; ++y) {
            const uint8_t* row = image.Row(y);
            for (uint32_t x = 0; x < w; ++x) std::memcpy(dst + (static_cast<size_t>(y) * w + x) * 4, &lut[row[x]], 4);
        }
        src = dst;
        stride = static_cast<size_t>(w) * 4;
        channels = 4;
    } else if (image.channels <= 2) {
        const uint32_t expanded = image.channels == 1 ? 3u : 4u;
        tlsWebpPixels.resize(pixels * expanded);
        for (uint32_t y = 0; y < h; ++y) {
            uint8_t* dst = tlsWebpPixels.data() + static_cast<size_t>(y) * w * expanded;
            if (image.channels == 1) {
                kernels::GreyToRgb(image.Row(y), w, dst);
            } else {
                kernels::GreyAlphaToRgba(image.Row(y), w, dst);
            }
        }
        src = tlsWebpPixels.data();
        stride = static_cast<size_t>(w) * expanded;
        channels = expanded;
    }

    thread_local WebpPictureHolder holder;
    WebPPicture& pic = holder.Get();
    pic.use_argb = config.lossless;
    pic.width = static_cast<int>(w);
    pic.height = static_cast<int>(h);
    const int istride = static_cast<int>(stride);
    int ok = 0;
    if (channels == 3) {
        ok = WebPPictureImportRGB(&pic, src, istride);
    } else if (image.allOpaque) {
        ok = WebPPictureImportRGBX(&pic, src, istride);  // alpha dropped
    } else {
        ok = WebPPictureImportRGBA(&pic, src, istride);
    }
    if (!ok) throw std::bad_alloc();

    out.clear();
    out.reserve(pixels / 2 + 4096);
    WriterState state{&out, false};
    pic.writer = AppendWriter;
    pic.custom_ptr = &state;
    const int encoded = WebPEncode(&config, &pic);
    pic.writer = nullptr;
    pic.custom_ptr = nullptr;
    if (state.failed) throw std::bad_alloc();
    if (!encoded) throw std::runtime_error(std::string("WebP: WebPEncode: ") + WebpErrorText(pic.error_code));
}

}  // namespace ddbt::encode
