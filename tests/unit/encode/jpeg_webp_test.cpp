// SPDX-License-Identifier: MPL-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <utility>
#include <vector>

#include "core/encode/jpeg_encoder.h"
#include "core/encode/webp_encoder.h"
#include "decode_helpers.h"

using ddbt::encode::ByteBuffer;
using ddbt::encode::EncodeJpeg;
using ddbt::encode::EncodeWebp;
using ddbt::encode::JpegOptions;
using ddbt::encode::JpegSubsampling;
using ddbt::encode::PaletteEntry;
using ddbt::encode::TileImage;
using ddbt::encode::WebpOptions;
using ddbt::encode::testing::DecodeJpeg;
using ddbt::encode::testing::DecodeWebp;
using ddbt::encode::testing::MakeImage;
using ddbt::encode::testing::Psnr;

namespace {

// Colour channels of a view, packed (grey stays 1 channel unless expand).
std::vector<uint8_t> Colors(const std::vector<uint8_t>& img, uint32_t w, uint32_t h, uint32_t ch, size_t stride,
                            bool expandGrey = false) {
    const uint32_t colors = ch >= 3 ? 3 : 1;
    const uint32_t outCh = expandGrey ? 3 : colors;
    std::vector<uint8_t> v;
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            for (uint32_t c = 0; c < outCh; ++c) v.push_back(img[y * stride + x * ch + (c < colors ? c : 0)]);
        }
    }
    return v;
}

std::vector<uint8_t> Rgb(const std::vector<uint8_t>& rgba) {
    std::vector<uint8_t> v;
    for (size_t i = 0; i < rgba.size(); i += 4) v.insert(v.end(), rgba.begin() + static_cast<long>(i),
                                                         rgba.begin() + static_cast<long>(i) + 3);
    return v;
}

}  // namespace

// ------------------------------------------------------------------------------------------ JPEG

TEST(JpegEncoder, RgbRoundTripAboveThePsnrFloor) {
    for (const auto& [w, h] : std::vector<std::pair<uint32_t, uint32_t>>{{256, 256}, {512, 512}, {77, 33}, {1, 1}}) {
        const size_t stride = static_cast<size_t>(w) * 3 + 7;
        const auto img = MakeImage(w, h, 3, stride, w + h);
        TileImage t = TileImage::Packed(img.data(), w, h, 3);
        t.stride = stride;
        ByteBuffer out;
        EncodeJpeg(t, JpegOptions{}, out);
        ASSERT_GE(out.size(), 4u);
        EXPECT_EQ(out.data()[0], 0xFF);
        EXPECT_EQ(out.data()[1], 0xD8);
        const auto d = DecodeJpeg(out.data(), out.size());
        ASSERT_TRUE(d.ok);
        EXPECT_EQ(d.width, static_cast<int>(w));
        EXPECT_EQ(d.height, static_cast<int>(h));
        EXPECT_EQ(d.subsampling, TJSAMP_420);  // GDAL's default
        EXPECT_EQ(d.channels, 3);
        if (w > 8) {
            EXPECT_GT(Psnr(Colors(img, w, h, 3, stride).data(), d.pixels.data(), d.pixels.size()), 30.0) << w;
        }
    }
}

TEST(JpegEncoder, OpaqueRgbaGivesTheSameBytesAsRgb) {
    const uint32_t w = 256, h = 256;
    const auto rgba = MakeImage(w, h, 4, w * 4, 3, false);
    std::vector<uint8_t> rgb = Colors(rgba, w, h, 4, w * 4);
    ByteBuffer a, b;
    EncodeJpeg(TileImage::Packed(rgba.data(), w, h, 4), JpegOptions{}, a);
    EncodeJpeg(TileImage::Packed(rgb.data(), w, h, 3), JpegOptions{}, b);
    ASSERT_EQ(a.size(), b.size());
    EXPECT_EQ(std::memcmp(a.data(), b.data(), a.size()), 0);
}

TEST(JpegEncoder, TransparentPixelsBecomeWhite) {
    const uint32_t w = 128, h = 128;
    std::vector<uint8_t> img(static_cast<size_t>(w) * h * 4, 0);  // black
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) img[(y * w + x) * 4 + 3] = x < w / 2 ? 0 : 255;
    }
    ByteBuffer out;
    EncodeJpeg(TileImage::Packed(img.data(), w, h, 4), JpegOptions{}, out);
    const auto d = DecodeJpeg(out.data(), out.size());
    ASSERT_TRUE(d.ok);
    // Away from the edge: white on the left, black on the right
    for (uint32_t y = 0; y < h; ++y) {
        for (const uint32_t x : {2u, w / 4, w / 2 - 10}) {
            for (uint32_t c = 0; c < 3; ++c) EXPECT_GE(d.pixels[(y * w + x) * 3 + c], 245) << x << "," << y;
        }
        for (const uint32_t x : {w / 2 + 10, w - 3}) {
            for (uint32_t c = 0; c < 3; ++c) EXPECT_LE(d.pixels[(y * w + x) * 3 + c], 10) << x << "," << y;
        }
    }
}

TEST(JpegEncoder, GreyStaysOneBand) {
    for (const uint32_t ch : {1u, 2u}) {
        const uint32_t w = 100, h = 60;
        const size_t stride = static_cast<size_t>(w) * ch + 2;
        const auto img = MakeImage(w, h, ch, stride, 9, ch == 2);
        TileImage t = TileImage::Packed(img.data(), w, h, ch);
        t.stride = stride;
        ByteBuffer out;
        EncodeJpeg(t, JpegOptions{}, out);
        const auto d = DecodeJpeg(out.data(), out.size());
        ASSERT_TRUE(d.ok);
        EXPECT_EQ(d.colorspace, TJCS_GRAY);
        EXPECT_EQ(d.channels, 1);
        if (ch == 1) {
            EXPECT_GT(Psnr(Colors(img, w, h, 1, stride).data(), d.pixels.data(), d.pixels.size()), 30.0);
        } else {
            // Grey + alpha: alpha 0 is white. The centre of MakeImage's transparent disc (radius
            // w / 5 around (w / 4, h / 4)), away from its edge and the block ringing there
            const uint32_t cx = w / 4, cy = h / 4;
            for (uint32_t y = cy - 4; y <= cy + 4; ++y) {
                for (uint32_t x = cx - 4; x <= cx + 4; ++x) {
                    ASSERT_EQ(img[y * stride + x * 2 + 1], 0);
                    EXPECT_GE(d.pixels[y * w + x], 245);
                }
            }
        }
    }
}

TEST(JpegEncoder, OptionsChangeTheStream) {
    const uint32_t w = 256, h = 256;
    const auto img = MakeImage(w, h, 3, w * 3, 4);
    const TileImage t = TileImage::Packed(img.data(), w, h, 3);
    ByteBuffer optimized, plain, q90, s444;
    EncodeJpeg(t, JpegOptions{}, optimized);
    JpegOptions o;
    o.optimizeHuffman = false;
    EncodeJpeg(t, o, plain);
    EXPECT_LT(optimized.size(), plain.size());
    o = JpegOptions{};
    o.quality = 90;
    EncodeJpeg(t, o, q90);
    EXPECT_GT(q90.size(), optimized.size());
    o = JpegOptions{};
    o.subsampling = JpegSubsampling::k444;
    EncodeJpeg(t, o, s444);
    EXPECT_EQ(DecodeJpeg(s444.data(), s444.size()).subsampling, TJSAMP_444);
    o.quality = 0;
    EXPECT_THROW(EncodeJpeg(t, o, s444), std::invalid_argument);
}

TEST(JpegEncoder, PaletteIsExpanded) {
    const uint32_t w = 64, h = 64;
    const PaletteEntry pal[2] = {{255, 0, 0, 255}, {0, 0, 255, 0}};
    std::vector<uint8_t> idx(static_cast<size_t>(w) * h);
    for (uint32_t i = 0; i < idx.size(); ++i) idx[i] = (i % w) < w / 2 ? 0 : 1;
    TileImage t = TileImage::Packed(idx.data(), w, h, 1);
    t.palette = pal;
    t.paletteSize = 2;
    ByteBuffer out;
    EncodeJpeg(t, JpegOptions{}, out);
    const auto d = DecodeJpeg(out.data(), out.size());
    ASSERT_TRUE(d.ok);
    EXPECT_EQ(d.channels, 3);
    const uint8_t* red = d.pixels.data() + (32 * w + 4) * 3;
    EXPECT_GE(red[0], 230);
    EXPECT_LE(red[2], 30);
    const uint8_t* white = d.pixels.data() + (32 * w + 60) * 3;  // index 1 has alpha 0
    EXPECT_GE(white[0], 240);
    EXPECT_GE(white[2], 240);
}

// ------------------------------------------------------------------------------------------ WebP

TEST(WebpEncoder, RgbRoundTripAboveThePsnrFloor) {
    for (const auto& [w, h] : std::vector<std::pair<uint32_t, uint32_t>>{{256, 256}, {512, 512}, {77, 33}}) {
        const size_t stride = static_cast<size_t>(w) * 3 + 5;
        const auto img = MakeImage(w, h, 3, stride, w * 3 + h);
        TileImage t = TileImage::Packed(img.data(), w, h, 3);
        t.stride = stride;
        ByteBuffer out;
        EncodeWebp(t, WebpOptions{}, out);
        ASSERT_GE(out.size(), 12u);
        EXPECT_EQ(std::memcmp(out.data(), "RIFF", 4), 0);
        const auto d = DecodeWebp(out.data(), out.size());
        ASSERT_TRUE(d.ok);
        EXPECT_FALSE(d.hasAlpha);
        EXPECT_FALSE(d.lossless);
        EXPECT_EQ(d.width, static_cast<int>(w));
        EXPECT_GT(Psnr(Colors(img, w, h, 3, stride).data(), Rgb(d.rgba).data(), static_cast<size_t>(w) * h * 3), 30.0);
    }
}

TEST(WebpEncoder, KeepsAlphaExactlyAndDropsAnOpaqueOne) {
    const uint32_t w = 128, h = 96;
    const size_t stride = static_cast<size_t>(w) * 4 + 4;
    const auto img = MakeImage(w, h, 4, stride, 21, true);
    TileImage t = TileImage::Packed(img.data(), w, h, 4);
    t.stride = stride;
    ByteBuffer out;
    EncodeWebp(t, WebpOptions{}, out);
    auto d = DecodeWebp(out.data(), out.size());
    ASSERT_TRUE(d.ok);
    EXPECT_TRUE(d.hasAlpha);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) ASSERT_EQ(d.rgba[(y * w + x) * 4 + 3], img[y * stride + x * 4 + 3]);
    }

    const auto opaque = MakeImage(w, h, 4, stride, 21, false);
    t.pixels = opaque.data();
    EncodeWebp(t, WebpOptions{}, out);
    d = DecodeWebp(out.data(), out.size());
    ASSERT_TRUE(d.ok);
    EXPECT_FALSE(d.hasAlpha);
    EXPECT_GT(Psnr(Colors(opaque, w, h, 4, stride).data(), Rgb(d.rgba).data(), static_cast<size_t>(w) * h * 3), 30.0);
}

TEST(WebpEncoder, GreyIsExpandedToRgb) {
    for (const uint32_t ch : {1u, 2u}) {
        const uint32_t w = 90, h = 50;
        const size_t stride = static_cast<size_t>(w) * ch + 3;
        const auto img = MakeImage(w, h, ch, stride, 2, ch == 2);
        TileImage t = TileImage::Packed(img.data(), w, h, ch);
        t.stride = stride;
        ByteBuffer out;
        EncodeWebp(t, WebpOptions{}, out);
        const auto d = DecodeWebp(out.data(), out.size());
        ASSERT_TRUE(d.ok);
        EXPECT_EQ(d.hasAlpha, ch == 2);
        if (ch == 1) {
            EXPECT_GT(Psnr(Colors(img, w, h, 1, stride, true).data(), Rgb(d.rgba).data(), static_cast<size_t>(w) * h * 3),
                      30.0);
        } else {
            for (uint32_t y = 0; y < h; ++y) {
                for (uint32_t x = 0; x < w; ++x) ASSERT_EQ(d.rgba[(y * w + x) * 4 + 3], img[y * stride + x * 2 + 1]);
            }
        }
    }
}

TEST(WebpEncoder, MethodAndQualityKnobs) {
    const uint32_t w = 256, h = 256;
    const auto img = MakeImage(w, h, 3, w * 3, 6);
    const TileImage t = TileImage::Packed(img.data(), w, h, 3);
    ByteBuffer m0, m2, q95;
    WebpOptions o;
    o.method = 0;
    EncodeWebp(t, o, m0);
    o.method = 2;
    EncodeWebp(t, o, m2);
    o.quality = 95;
    EncodeWebp(t, o, q95);
    EXPECT_NE(m0.size(), m2.size());
    EXPECT_GT(q95.size(), m2.size());
    for (const ByteBuffer* b : {&m0, &m2, &q95}) EXPECT_TRUE(DecodeWebp(b->data(), b->size()).ok);
    o.method = 7;
    EXPECT_THROW(EncodeWebp(t, o, m0), std::invalid_argument);
    o.method = -2;
    EXPECT_THROW(EncodeWebp(t, o, m0), std::invalid_argument);
    // -1: the process default (DroneDB's 2 unless DDBT_WEBP_METHOD says otherwise), same bytes as it
    ByteBuffer def, explicitDefault;
    o.quality = 80;
    o.method = -1;
    EncodeWebp(t, o, def);
    o.method = ddbt::encode::DefaultWebpMethod();
    EncodeWebp(t, o, explicitDefault);
    EXPECT_EQ(std::vector<uint8_t>(def.data(), def.data() + def.size()),
              std::vector<uint8_t>(explicitDefault.data(), explicitDefault.data() + explicitDefault.size()));
}
