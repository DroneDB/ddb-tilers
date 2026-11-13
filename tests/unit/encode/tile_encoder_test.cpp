// SPDX-License-Identifier: MPL-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

#include "core/encode/tile_encoder.h"
#include "decode_helpers.h"

using ddbt::encode::BlankTileBytes;
using ddbt::encode::ByteBuffer;
using ddbt::encode::EffectiveQuality;
using ddbt::encode::EncodeOptions;
using ddbt::encode::EncodeTile;
using ddbt::encode::ParseTileFormat;
using ddbt::encode::ScanAlpha;
using ddbt::encode::TileFormat;
using ddbt::encode::TileImage;
using ddbt::encode::testing::DecodeJpeg;
using ddbt::encode::testing::DecodePng;
using ddbt::encode::testing::DecodeWebp;
using ddbt::encode::testing::MakeImage;

TEST(TileEncoder, ParsesFormatsAsDroneDB) {
    EXPECT_EQ(ParseTileFormat(""), TileFormat::kPng);
    EXPECT_EQ(ParseTileFormat("PNG"), TileFormat::kPng);
    EXPECT_EQ(ParseTileFormat("image/png"), TileFormat::kPng);
    EXPECT_EQ(ParseTileFormat("jpg"), TileFormat::kJpeg);
    EXPECT_EQ(ParseTileFormat("JPEG"), TileFormat::kJpeg);
    EXPECT_EQ(ParseTileFormat("image/jpg"), TileFormat::kJpeg);
    EXPECT_EQ(ParseTileFormat("WebP"), TileFormat::kWebp);
    EXPECT_EQ(ParseTileFormat("image/webp"), TileFormat::kWebp);
    EXPECT_FALSE(ParseTileFormat("gif").has_value());
}

TEST(TileEncoder, DefaultQualitiesAreDroneDBs) {
    EncodeOptions o;
    o.format = TileFormat::kJpeg;
    EXPECT_EQ(EffectiveQuality(o), 75);
    o.format = TileFormat::kWebp;
    EXPECT_EQ(EffectiveQuality(o), 80);
    o.quality = 250;
    EXPECT_EQ(EffectiveQuality(o), 100);
    EXPECT_EQ(EncodeOptions{}.png.level, 1);
    EXPECT_EQ(EncodeOptions{}.webp.method, -1);  // the process default
    if (std::getenv("DDBT_WEBP_METHOD") == nullptr) EXPECT_EQ(ddbt::encode::DefaultWebpMethod(), 1);
}

TEST(TileEncoder, ScanAlphaClassifiesImages) {
    for (const uint32_t ch : {2u, 4u}) {
        const uint32_t w = 300, h = 7;  // odd width: vector tails
        std::vector<uint8_t> img(static_cast<size_t>(w) * h * ch, 17);
        auto setAlpha = [&](uint8_t a) {
            for (size_t i = ch - 1; i < img.size(); i += ch) img[i] = a;
        };
        TileImage t = TileImage::Packed(img.data(), w, h, ch);
        setAlpha(255);
        ScanAlpha(t);
        EXPECT_TRUE(t.alphaKnown && t.allOpaque && !t.allTransparent);
        setAlpha(0);
        ScanAlpha(t);
        EXPECT_TRUE(!t.allOpaque && t.allTransparent);
        img[img.size() - 1] = 1;  // last pixel of the last row
        ScanAlpha(t);
        EXPECT_TRUE(!t.allOpaque && !t.allTransparent);
    }
    std::vector<uint8_t> rgb(30, 0);
    TileImage t = TileImage::Packed(rgb.data(), 10, 1, 3);
    ScanAlpha(t);
    EXPECT_TRUE(t.allOpaque && !t.allTransparent);
}

TEST(TileEncoder, TransparentTilesGiveTheCachedBlankBytes) {
    for (const auto format : {TileFormat::kPng, TileFormat::kJpeg, TileFormat::kWebp}) {
        for (const uint32_t ch : {2u, 4u}) {
            const uint32_t w = 256, h = 256;
            std::vector<uint8_t> img(static_cast<size_t>(w) * h * ch, 0);
            for (size_t i = 0; i + ch <= img.size(); i += ch) img[i] = 123;  // colour under alpha 0: ignored
            EncodeOptions o;
            o.format = format;
            ByteBuffer out;
            EncodeTile(TileImage::Packed(img.data(), w, h, ch), o, out);
            const auto& blank = BlankTileBytes(w, h, ch == 4 ? 3 : 1, o);
            ASSERT_EQ(out.size(), blank.size());
            EXPECT_EQ(std::memcmp(out.data(), blank.data(), blank.size()), 0);
            // Encoded once: the same object
            EXPECT_EQ(&BlankTileBytes(w, h, ch == 4 ? 3 : 1, o), &blank);

            if (format == TileFormat::kPng) {
                const auto d = DecodePng(out.data(), out.size());
                ASSERT_TRUE(d.ok);
                EXPECT_EQ(d.colorType, ch == 4 ? 6 : 4);
                for (size_t i = 3; i < d.rgba.size(); i += 4) ASSERT_EQ(d.rgba[i], 0);
            } else if (format == TileFormat::kJpeg) {
                const auto d = DecodeJpeg(out.data(), out.size());
                ASSERT_TRUE(d.ok);
                EXPECT_EQ(d.channels, ch == 4 ? 3 : 1);
                for (const uint8_t v : d.pixels) ASSERT_GE(v, 254);
            } else {
                const auto d = DecodeWebp(out.data(), out.size());
                ASSERT_TRUE(d.ok);
                EXPECT_TRUE(d.hasAlpha);
                for (size_t i = 3; i < d.rgba.size(); i += 4) ASSERT_EQ(d.rgba[i], 0);
            }
        }
    }
    // The key holds the quality: another quality is another blank
    EncodeOptions a, b;
    a.format = b.format = TileFormat::kJpeg;
    b.quality = 90;
    EXPECT_NE(&BlankTileBytes(256, 256, 3, a), &BlankTileBytes(256, 256, 3, b));
    EXPECT_NE(&BlankTileBytes(256, 256, 3, a), &BlankTileBytes(512, 512, 3, a));
}

TEST(TileEncoder, BlankCacheIsThreadSafe) {
    EncodeOptions o;
    o.format = TileFormat::kWebp;
    o.quality = 61;  // a key no other test uses
    std::vector<const std::vector<uint8_t>*> seen(8, nullptr);
    std::vector<std::thread> threads;
    for (size_t i = 0; i < seen.size(); ++i) {
        threads.emplace_back([&, i] { seen[i] = &BlankTileBytes(128, 128, 3, o); });
    }
    for (auto& t : threads) t.join();
    for (const auto* p : seen) EXPECT_EQ(p, seen[0]);
}

TEST(TileEncoder, EncodesEveryFormatWithDroneDBDefaults) {
    const uint32_t w = 256, h = 256;
    const auto img = MakeImage(w, h, 4, w * 4, 12, true);
    for (const auto format : {TileFormat::kPng, TileFormat::kJpeg, TileFormat::kWebp}) {
        EncodeOptions o;
        o.format = format;
        ByteBuffer out = EncodeTile(TileImage::Packed(img.data(), w, h, 4), o);
        ASSERT_FALSE(out.empty());
        if (format == TileFormat::kPng) {
            const auto d = DecodePng(out.data(), out.size());
            ASSERT_TRUE(d.ok);
            EXPECT_EQ(d.pixels, img);
        } else if (format == TileFormat::kJpeg) {
            EXPECT_TRUE(DecodeJpeg(out.data(), out.size()).ok);
        } else {
            EXPECT_TRUE(DecodeWebp(out.data(), out.size()).hasAlpha);
        }
    }
}

TEST(TileEncoder, ParallelEncodesAgree) {
    // Per-thread compressors, handles and scratch: concurrent encodes give the serial bytes.
    const uint32_t w = 256, h = 256;
    const auto img = MakeImage(w, h, 4, w * 4, 31, true);
    const TileImage t = TileImage::Packed(img.data(), w, h, 4);
    std::vector<std::vector<uint8_t>> ref;
    for (const auto format : {TileFormat::kPng, TileFormat::kJpeg, TileFormat::kWebp}) {
        EncodeOptions o;
        o.format = format;
        const ByteBuffer b = EncodeTile(t, o);
        ref.emplace_back(b.data(), b.data() + b.size());
    }
    std::vector<int> mismatches(6, 0);
    std::vector<std::thread> threads;
    for (size_t i = 0; i < mismatches.size(); ++i) {
        threads.emplace_back([&, i] {
            for (int rep = 0; rep < 4; ++rep) {
                for (size_t f = 0; f < 3; ++f) {
                    EncodeOptions o;
                    o.format = static_cast<TileFormat>(f);
                    const ByteBuffer b = EncodeTile(t, o);
                    if (b.size() != ref[f].size() || std::memcmp(b.data(), ref[f].data(), b.size()) != 0) ++mismatches[i];
                }
            }
        });
    }
    for (auto& th : threads) th.join();
    for (const int m : mismatches) EXPECT_EQ(m, 0);
}
