// SPDX-License-Identifier: MPL-2.0
#include "core/encode/jpeg_encoder.h"

#include <turbojpeg.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include "core/encode/kernels.h"

namespace ddbt::encode {

namespace {

// One TurboJPEG compressor per thread, destroyed at thread exit.
class JpegHandle {
public:
    JpegHandle() = default;
    JpegHandle(const JpegHandle&) = delete;
    JpegHandle& operator=(const JpegHandle&) = delete;
    ~JpegHandle() {
        if (handle_ != nullptr) tj3Destroy(handle_);
    }

    tjhandle Get() {
        if (handle_ == nullptr) {
            handle_ = tj3Init(TJINIT_COMPRESS);
            if (handle_ == nullptr) throw std::runtime_error("JPEG: tj3Init failed");
        }
        return handle_;
    }

private:
    tjhandle handle_ = nullptr;
};

thread_local JpegHandle tlsJpeg;
thread_local ByteBuffer tlsJpegPixels;

int TjSubsampling(JpegSubsampling s) {
    switch (s) {
        case JpegSubsampling::k422: return TJSAMP_422;
        case JpegSubsampling::k444: return TJSAMP_444;
        default: return TJSAMP_420;
    }
}

void Set(tjhandle h, int param, int value) {
    if (tj3Set(h, param, value) != 0) throw std::runtime_error(std::string("JPEG: tj3Set: ") + tj3GetErrorStr(h));
}

}  // namespace

void EncodeJpeg(const TileImage& input, const JpegOptions& options, ByteBuffer& out) {
    if (const char* err = ValidateTileImage(input)) throw std::invalid_argument(std::string("JPEG: ") + err);
    if (options.quality < 1 || options.quality > 100) throw std::invalid_argument("JPEG: quality must be 1 to 100");

    TileImage image = input;
    const uint32_t w = image.width;
    const uint32_t h = image.height;
    const size_t pixels = static_cast<size_t>(w) * h;

    // Source pixels in a format TurboJPEG reads: white under alpha 0, alpha dropped
    const uint8_t* src = image.pixels;
    size_t pitch = image.RowStride();
    int pixelFormat = TJPF_GRAY;
    if (image.IsPalette()) {
        uint32_t lut[256];
        for (uint32_t i = 0; i < 256; ++i) {
            const PaletteEntry e = i < image.paletteSize ? image.palette[i] : PaletteEntry{};
            lut[i] = e.a == 0 ? 0xFFFFFFFFu
                              : (uint32_t{e.r} | (uint32_t{e.g} << 8) | (uint32_t{e.b} << 16) | 0xFF000000u);
        }
        tlsJpegPixels.resize(pixels * 4);
        uint8_t* dst = tlsJpegPixels.data();
        for (uint32_t y = 0; y < h; ++y) {
            const uint8_t* row = image.Row(y);
            for (uint32_t x = 0; x < w; ++x) std::memcpy(dst + (static_cast<size_t>(y) * w + x) * 4, &lut[row[x]], 4);
        }
        src = dst;
        pitch = static_cast<size_t>(w) * 4;
        pixelFormat = TJPF_RGBX;
    } else if (image.channels == 1) {
        pixelFormat = TJPF_GRAY;
    } else if (image.channels == 3) {
        pixelFormat = TJPF_RGB;
    } else {
        if (!image.alphaKnown) ScanAlpha(image);
        if (image.channels == 4) {
            pixelFormat = TJPF_RGBX;
            if (!image.allOpaque) {
                tlsJpegPixels.resize(pixels * 4);
                for (uint32_t y = 0; y < h; ++y) {
                    kernels::WhiteUnderTransparent4(image.Row(y), w,
                                                    tlsJpegPixels.data() + static_cast<size_t>(y) * w * 4);
                }
                src = tlsJpegPixels.data();
                pitch = static_cast<size_t>(w) * 4;
            }
        } else {
            pixelFormat = TJPF_GRAY;
            tlsJpegPixels.resize(pixels);
            for (uint32_t y = 0; y < h; ++y) {
                kernels::WhiteUnderTransparent2(image.Row(y), w, tlsJpegPixels.data() + static_cast<size_t>(y) * w);
            }
            src = tlsJpegPixels.data();
            pitch = w;
        }
    }

    const int subsamp = pixelFormat == TJPF_GRAY ? TJSAMP_GRAY : TjSubsampling(options.subsampling);
    tjhandle tj = tlsJpeg.Get();
    Set(tj, TJPARAM_QUALITY, options.quality);
    Set(tj, TJPARAM_SUBSAMP, subsamp);
    Set(tj, TJPARAM_OPTIMIZE, options.optimizeHuffman ? 1 : 0);
    Set(tj, TJPARAM_FASTDCT, options.fastDct ? 1 : 0);
    Set(tj, TJPARAM_PROGRESSIVE, options.progressive ? 1 : 0);
    Set(tj, TJPARAM_ARITHMETIC, 0);
    Set(tj, TJPARAM_NOREALLOC, 1);

    const size_t bound = tj3JPEGBufSize(static_cast<int>(w), static_cast<int>(h), subsamp);
    if (bound == 0) throw std::runtime_error(std::string("JPEG: tj3JPEGBufSize: ") + tj3GetErrorStr(nullptr));
    out.clear();
    out.reserve(bound);
    unsigned char* jpeg = out.data();
    size_t jpegSize = bound;
    if (tj3Compress8(tj, src, static_cast<int>(w), static_cast<int>(pitch), static_cast<int>(h), pixelFormat, &jpeg,
                     &jpegSize) != 0) {
        throw std::runtime_error(std::string("JPEG: tj3Compress8: ") + tj3GetErrorStr(tj));
    }
    out.resize(jpegSize);
}

}  // namespace ddbt::encode
