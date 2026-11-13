// SPDX-License-Identifier: MPL-2.0
//
// Independent decoders for the encoder tests: libspng (PNG, checks every CRC and the Adler-32),
// TurboJPEG (JPEG) and libwebp (WebP), plus test images.
#pragma once

#include <spng.h>
#include <turbojpeg.h>
#include <webp/decode.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <random>
#include <vector>

namespace ddbt::encode::testing {

struct DecodedPng {
    bool ok = false;
    uint32_t width = 0;
    uint32_t height = 0;
    uint8_t colorType = 0;
    uint8_t bitDepth = 0;
    std::vector<uint8_t> pixels;  // native format (palette: indices)
    std::vector<uint8_t> rgba;    // expanded to RGBA8 with tRNS applied
    std::vector<uint8_t> palette;  // 4 bytes per entry (alpha from tRNS)
    bool hasTrns = false;
};

inline DecodedPng DecodePng(const uint8_t* data, size_t size) {
    DecodedPng r;
    for (int pass = 0; pass < 2; ++pass) {
        spng_ctx* ctx = spng_ctx_new(0);
        if (ctx == nullptr) return r;
        spng_set_crc_action(ctx, SPNG_CRC_ERROR, SPNG_CRC_ERROR);
        spng_set_png_buffer(ctx, data, size);
        spng_ihdr ihdr{};
        if (spng_get_ihdr(ctx, &ihdr) != 0) {
            spng_ctx_free(ctx);
            return r;
        }
        r.width = ihdr.width;
        r.height = ihdr.height;
        r.colorType = ihdr.color_type;
        r.bitDepth = ihdr.bit_depth;
        const int fmt = pass == 0 ? SPNG_FMT_PNG : SPNG_FMT_RGBA8;
        size_t n = 0;
        if (spng_decoded_image_size(ctx, fmt, &n) != 0) {
            spng_ctx_free(ctx);
            return r;
        }
        auto& dst = pass == 0 ? r.pixels : r.rgba;
        dst.resize(n);
        if (spng_decode_image(ctx, dst.data(), n, fmt, pass == 0 ? 0 : SPNG_DECODE_TRNS) != 0) {
            spng_ctx_free(ctx);
            return r;
        }
        if (pass == 0 && ihdr.color_type == SPNG_COLOR_TYPE_INDEXED) {
            spng_plte plte{};
            if (spng_get_plte(ctx, &plte) == 0) {
                spng_trns trns{};
                r.hasTrns = spng_get_trns(ctx, &trns) == 0;
                for (uint32_t i = 0; i < plte.n_entries; ++i) {
                    r.palette.push_back(plte.entries[i].red);
                    r.palette.push_back(plte.entries[i].green);
                    r.palette.push_back(plte.entries[i].blue);
                    r.palette.push_back(r.hasTrns && i < trns.n_type3_entries ? trns.type3_alpha[i] : uint8_t{255});
                }
            }
        }
        spng_ctx_free(ctx);
    }
    r.ok = true;
    return r;
}

struct DecodedJpeg {
    bool ok = false;
    int width = 0;
    int height = 0;
    int subsampling = -1;
    int colorspace = -1;
    std::vector<uint8_t> pixels;  // RGB, or grey when the JPEG is greyscale
    int channels = 0;
};

inline DecodedJpeg DecodeJpeg(const uint8_t* data, size_t size) {
    DecodedJpeg r;
    tjhandle h = tj3Init(TJINIT_DECOMPRESS);
    if (h == nullptr) return r;
    if (tj3DecompressHeader(h, data, size) == 0) {
        r.width = tj3Get(h, TJPARAM_JPEGWIDTH);
        r.height = tj3Get(h, TJPARAM_JPEGHEIGHT);
        r.subsampling = tj3Get(h, TJPARAM_SUBSAMP);
        r.colorspace = tj3Get(h, TJPARAM_COLORSPACE);
        r.channels = r.colorspace == TJCS_GRAY ? 1 : 3;
        r.pixels.resize(static_cast<size_t>(r.width) * static_cast<size_t>(r.height) * static_cast<size_t>(r.channels));
        r.ok = tj3Decompress8(h, data, size, r.pixels.data(), 0, r.channels == 1 ? TJPF_GRAY : TJPF_RGB) == 0;
    }
    tj3Destroy(h);
    return r;
}

struct DecodedWebp {
    bool ok = false;
    int width = 0;
    int height = 0;
    bool hasAlpha = false;
    bool lossless = false;
    std::vector<uint8_t> rgba;
};

inline DecodedWebp DecodeWebp(const uint8_t* data, size_t size) {
    DecodedWebp r;
    WebPBitstreamFeatures f;
    if (WebPGetFeatures(data, size, &f) != VP8_STATUS_OK) return r;
    r.hasAlpha = f.has_alpha != 0;
    r.lossless = f.format == 2;
    uint8_t* px = WebPDecodeRGBA(data, size, &r.width, &r.height);
    if (px == nullptr) return r;
    r.rgba.assign(px, px + static_cast<size_t>(r.width) * static_cast<size_t>(r.height) * 4);
    WebPFree(px);
    r.ok = true;
    return r;
}

/// PSNR in dB of b against a (same size); infinity when equal.
inline double Psnr(const uint8_t* a, const uint8_t* b, size_t n) {
    double se = 0;
    for (size_t i = 0; i < n; ++i) {
        const double d = static_cast<double>(a[i]) - static_cast<double>(b[i]);
        se += d * d;
    }
    if (se == 0) return INFINITY;
    return 10.0 * std::log10(255.0 * 255.0 / (se / static_cast<double>(n)));
}

/// A photo-like test image: smooth gradients, a few edges and mild noise, `channels` per pixel,
/// rows of `stride` bytes (padding bytes filled with garbage). Alpha (channel 2 of 2 / 4 of 4)
/// is 255 unless alphaPattern: then a disc of alpha 0 in the top left, a ramp elsewhere.
inline std::vector<uint8_t> MakeImage(uint32_t w, uint32_t h, uint32_t channels, size_t stride, uint32_t seed,
                                      bool alphaPattern = false) {
    std::mt19937 rng(seed);
    std::vector<uint8_t> img(stride * h);
    for (auto& b : img) b = static_cast<uint8_t>(rng());  // padding garbage
    std::normal_distribution<double> noise(0.0, 3.0);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            uint8_t* p = img.data() + y * stride + static_cast<size_t>(x) * channels;
            const double fx = static_cast<double>(x) / w;
            const double fy = static_cast<double>(y) / h;
            const double edge = (x > w / 3 && y > h / 2) ? 60.0 : 0.0;
            const double base[3] = {40 + 150 * fx + edge, 60 + 120 * fy + 20 * std::sin(fx * 9), 90 + 80 * fx * fy};
            const uint32_t colors = (channels >= 3) ? 3 : 1;
            for (uint32_t c = 0; c < colors; ++c) {
                const double v = base[c] + noise(rng);
                p[c] = static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
            }
            if (channels == 2 || channels == 4) {
                uint8_t a = 255;
                if (alphaPattern) {
                    const double dx = static_cast<double>(x) - w * 0.25;
                    const double dy = static_cast<double>(y) - h * 0.25;
                    a = (dx * dx + dy * dy < (w * w) / 25.0) ? uint8_t{0} : static_cast<uint8_t>(55 + (200 * x) / w);
                }
                p[channels - 1] = a;
            }
        }
    }
    return img;
}

}  // namespace ddbt::encode::testing
