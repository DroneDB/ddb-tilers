// SPDX-License-Identifier: MPL-2.0
#include <gtest/gtest.h>
#include <libdeflate.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "core/encode/png_encoder.h"
#include "core/simd/simd.h"
#include "decode_helpers.h"

using ddbt::encode::ByteBuffer;
using ddbt::encode::EncodePng;
using ddbt::encode::PaletteEntry;
using ddbt::encode::PngEngine;
using ddbt::encode::PngOptions;
using ddbt::encode::PngRowFilter;
using ddbt::encode::TileImage;
using ddbt::encode::testing::DecodePng;
using ddbt::encode::testing::MakeImage;

namespace {

uint32_t ReadU32(const uint8_t* p) {
    return (uint32_t{p[0]} << 24) | (uint32_t{p[1]} << 16) | (uint32_t{p[2]} << 8) | p[3];
}

// Chunk types in order; checks lengths and CRCs on the way.
std::vector<std::string> Chunks(const ByteBuffer& png) {
    std::vector<std::string> types;
    size_t at = 8;
    while (at + 12 <= png.size()) {
        const uint32_t len = ReadU32(png.data() + at);
        types.emplace_back(reinterpret_cast<const char*>(png.data() + at + 4), 4);
        EXPECT_LE(at + 12 + len, png.size());
        EXPECT_EQ(ReadU32(png.data() + at + 8 + len), libdeflate_crc32(0, png.data() + at + 4, len + 4u))
            << types.back();
        at += 12 + len;
    }
    EXPECT_EQ(at, png.size());
    return types;
}

// The pixels of the view (strided, channels), packed, optionally without the alpha byte.
std::vector<uint8_t> Packed(const std::vector<uint8_t>& img, uint32_t w, uint32_t h, uint32_t ch, size_t stride,
                            bool dropAlpha) {
    const uint32_t outCh = dropAlpha ? ch - 1 : ch;
    std::vector<uint8_t> v;
    v.reserve(static_cast<size_t>(w) * h * outCh);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            for (uint32_t c = 0; c < outCh; ++c) v.push_back(img[y * stride + x * ch + c]);
        }
    }
    return v;
}

constexpr uint8_t kColorTypeOf[5] = {0, 0, 4, 2, 6};

}  // namespace

TEST(PngEncoder, RoundTripsEveryColourTypeFilterAndLevel) {
    ByteBuffer out;
    for (const uint32_t ch : {1u, 2u, 3u, 4u}) {
        for (const auto filter : {PngRowFilter::kNone, PngRowFilter::kSub, PngRowFilter::kUp, PngRowFilter::kAverage,
                                  PngRowFilter::kPaeth, PngRowFilter::kAdaptive}) {
            for (const int level : {0, 1, 6, 12}) {
                const uint32_t w = 61, h = 23;
                const size_t stride = static_cast<size_t>(w) * ch + 3;  // odd stride, garbage padding
                const auto img = MakeImage(w, h, ch, stride, ch * 100 + static_cast<uint32_t>(level), true);
                TileImage t = TileImage::Packed(img.data(), w, h, ch);
                t.stride = stride;
                PngOptions o;
                o.level = level;
                o.filter = filter;
                EncodePng(t, o, out);
                const auto d = DecodePng(out.data(), out.size());
                ASSERT_TRUE(d.ok) << "ch " << ch << " level " << level;
                EXPECT_EQ(d.width, w);
                EXPECT_EQ(d.height, h);
                EXPECT_EQ(d.bitDepth, 8);
                EXPECT_EQ(d.colorType, kColorTypeOf[ch]);
                EXPECT_EQ(d.pixels, Packed(img, w, h, ch, stride, false))
                    << "ch " << ch << " filter " << static_cast<int>(filter) << " level " << level;
                EXPECT_EQ(Chunks(out), (std::vector<std::string>{"IHDR", "IDAT", "IEND"}));
            }
        }
    }
}

TEST(PngEncoder, OddSizesRoundTrip) {
    ByteBuffer out;
    for (const auto& [w, h] : std::vector<std::pair<uint32_t, uint32_t>>{{1, 1}, {1, 9}, {9, 1}, {2, 3}, {255, 2},
                                                                         {257, 3}, {512, 512}, {256, 256}, {100, 1}}) {
        for (const uint32_t ch : {1u, 3u, 4u}) {
            const auto img = MakeImage(w, h, ch, static_cast<size_t>(w) * ch, w * 7 + h, true);
            const TileImage t = TileImage::Packed(img.data(), w, h, ch);
            EncodePng(t, PngOptions{}, out);
            const auto d = DecodePng(out.data(), out.size());
            ASSERT_TRUE(d.ok) << w << "x" << h << "x" << ch;
            // MakeImage(alphaPattern) can be opaque on tiny images: compare through RGBA
            const auto packed = Packed(img, w, h, ch, static_cast<size_t>(w) * ch, false);
            for (size_t i = 0; i < static_cast<size_t>(w) * h; ++i) {
                const uint8_t* p = packed.data() + i * ch;
                const uint8_t* q = d.rgba.data() + i * 4;
                const uint8_t r = p[0], g = ch >= 3 ? p[1] : p[0], b = ch >= 3 ? p[2] : p[0];
                const uint8_t a = ch == 4 ? p[3] : 255;
                ASSERT_EQ(q[0], r);
                ASSERT_EQ(q[1], g);
                ASSERT_EQ(q[2], b);
                ASSERT_EQ(q[3], a);
            }
        }
    }
}

TEST(PngEncoder, DropsAnOpaqueAlpha) {
    ByteBuffer out;
    for (const uint32_t ch : {2u, 4u}) {
        const uint32_t w = 40, h = 17;
        const size_t stride = static_cast<size_t>(w) * ch + 1;
        const auto img = MakeImage(w, h, ch, stride, 5, false);  // alpha 255 everywhere
        TileImage t = TileImage::Packed(img.data(), w, h, ch);
        t.stride = stride;
        EncodePng(t, PngOptions{}, out);
        auto d = DecodePng(out.data(), out.size());
        ASSERT_TRUE(d.ok);
        EXPECT_EQ(d.colorType, ch == 4 ? 2 : 0);
        EXPECT_EQ(d.pixels, Packed(img, w, h, ch, stride, true));

        PngOptions keep;
        keep.dropOpaqueAlpha = false;
        EncodePng(t, keep, out);
        d = DecodePng(out.data(), out.size());
        ASSERT_TRUE(d.ok);
        EXPECT_EQ(d.colorType, ch == 4 ? 6 : 4);
        EXPECT_EQ(d.pixels, Packed(img, w, h, ch, stride, false));
    }
}

// fpnge engine: every colour type, the opaque alpha dropped or kept, fpnge levels 1-5, odd sizes
// and strides with garbage padding: the decoded pixels are the image's, with the colour type the
// libdeflate engine writes. Palette images keep the libdeflate path.
TEST(PngEncoder, FpngeEngineRoundTrips) {
    ByteBuffer out;
    for (const auto& [w, h] : std::vector<std::pair<uint32_t, uint32_t>>{{1, 1}, {9, 1}, {61, 23}, {257, 3}, {256, 256}}) {
        for (const uint32_t ch : {1u, 2u, 3u, 4u}) {
            for (const bool alphaPattern : {true, false}) {
                for (const int level : {1, 4, 5}) {
                    const size_t stride = static_cast<size_t>(w) * ch + 3;
                    const auto img = MakeImage(w, h, ch, stride, w * 13 + h + ch, alphaPattern);
                    TileImage t = TileImage::Packed(img.data(), w, h, ch);
                    t.stride = stride;
                    PngOptions o;
                    o.engine = PngEngine::kFpnge;
                    o.fpngeLevel = level;
                    EncodePng(t, o, out);
                    const auto d = DecodePng(out.data(), out.size());
                    ASSERT_TRUE(d.ok) << w << "x" << h << "x" << ch << " level " << level;
                    ASSERT_EQ(d.width, w);
                    ASSERT_EQ(d.height, h);
                    PngOptions ref;
                    ref.engine = PngEngine::kDeflate;
                    ByteBuffer refOut;
                    EncodePng(t, ref, refOut);
                    const auto r = DecodePng(refOut.data(), refOut.size());
                    ASSERT_TRUE(r.ok);
                    EXPECT_EQ(d.colorType, r.colorType) << w << "x" << h << "x" << ch;
                    EXPECT_EQ(d.pixels, r.pixels) << w << "x" << h << "x" << ch << " level " << level;
                }
            }
        }
    }
    PngOptions bad;
    bad.engine = PngEngine::kFpnge;
    bad.fpngeLevel = 6;
    const auto img = MakeImage(4, 4, 3, 12, 1, false);
    EXPECT_THROW(EncodePng(TileImage::Packed(img.data(), 4, 4, 3), bad, out), std::invalid_argument);
}

TEST(PngEncoder, TrustsTheCallersAlphaSummary) {
    const uint32_t w = 16, h = 16;
    const auto img = MakeImage(w, h, 4, w * 4, 8, true);  // not opaque
    TileImage t = TileImage::Packed(img.data(), w, h, 4);
    t.alphaKnown = true;
    t.allOpaque = true;  // the caller says so: the alpha is not scanned and is dropped
    ByteBuffer out;
    EncodePng(t, PngOptions{}, out);
    const auto d = DecodePng(out.data(), out.size());
    ASSERT_TRUE(d.ok);
    EXPECT_EQ(d.colorType, 2);
}

TEST(PngEncoder, PaletteWithAndWithoutTransparency) {
    const uint32_t w = 33, h = 21;
    std::vector<PaletteEntry> pal(7);
    for (uint32_t i = 0; i < pal.size(); ++i) {
        pal[i] = PaletteEntry{static_cast<uint8_t>(i * 30), static_cast<uint8_t>(255 - i * 20),
                              static_cast<uint8_t>(i * 5), 255};
    }
    pal[2].a = 0;
    pal[4].a = 128;
    const size_t stride = w + 5;
    std::vector<uint8_t> idx(stride * h, 0xEE);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) idx[y * stride + x] = static_cast<uint8_t>((x / 3 + y) % 7);
    }
    TileImage t = TileImage::Packed(idx.data(), w, h, 1);
    t.stride = stride;
    t.palette = pal.data();
    t.paletteSize = static_cast<uint32_t>(pal.size());
    ByteBuffer out;
    EncodePng(t, PngOptions{}, out);
    auto d = DecodePng(out.data(), out.size());
    ASSERT_TRUE(d.ok);
    EXPECT_EQ(d.colorType, 3);
    EXPECT_TRUE(d.hasTrns);
    EXPECT_EQ(Chunks(out), (std::vector<std::string>{"IHDR", "PLTE", "tRNS", "IDAT", "IEND"}));
    ASSERT_EQ(d.palette.size(), pal.size() * 4);
    for (size_t i = 0; i < pal.size(); ++i) {
        EXPECT_EQ(d.palette[4 * i], pal[i].r);
        EXPECT_EQ(d.palette[4 * i + 1], pal[i].g);
        EXPECT_EQ(d.palette[4 * i + 2], pal[i].b);
        EXPECT_EQ(d.palette[4 * i + 3], pal[i].a);
    }
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            const uint8_t i = idx[y * stride + x];
            ASSERT_EQ(d.pixels[y * w + x], i);
            ASSERT_EQ(d.rgba[(y * w + x) * 4 + 3], pal[i].a);
            ASSERT_EQ(d.rgba[(y * w + x) * 4], pal[i].r);
        }
    }

    // Only opaque entries used: no tRNS (the alpha is dropped as for RGBA)
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) idx[y * stride + x] = (x + y) % 2 == 0 ? 1 : 5;
    }
    EncodePng(t, PngOptions{}, out);
    EXPECT_EQ(Chunks(out), (std::vector<std::string>{"IHDR", "PLTE", "IDAT", "IEND"}));
    d = DecodePng(out.data(), out.size());
    ASSERT_TRUE(d.ok);
    EXPECT_EQ(d.pixels[0], 1);
    EXPECT_EQ(d.pixels[1], 5);
}

TEST(PngEncoder, EveryTargetGivesTheSameBytes) {
    const uint32_t w = 256, h = 64;
    const auto img = MakeImage(w, h, 4, w * 4, 77, true);
    const TileImage t = TileImage::Packed(img.data(), w, h, 4);
    ByteBuffer ref, out;
    EncodePng(t, PngOptions{}, ref);
    struct Guard {
        ~Guard() { ddbt::simd::ForceTargetForTest(0); }
    } guard;
    for (const int64_t bit : ddbt::simd::SupportedTargetBits()) {
        ddbt::simd::ForceTargetForTest(bit);
        EncodePng(t, PngOptions{}, out);
        ASSERT_EQ(out.size(), ref.size()) << ddbt::simd::TargetNameOf(bit);
        EXPECT_EQ(std::memcmp(out.data(), ref.data(), ref.size()), 0) << ddbt::simd::TargetNameOf(bit);
    }
}

TEST(PngEncoder, RejectsBadInput) {
    ByteBuffer out;
    const uint8_t px[16] = {};
    TileImage t = TileImage::Packed(px, 2, 2, 4);
    PngOptions o;
    o.level = 13;
    EXPECT_THROW(EncodePng(t, o, out), std::invalid_argument);
    t.channels = 5;
    EXPECT_THROW(EncodePng(t, PngOptions{}, out), std::invalid_argument);
    t = TileImage::Packed(nullptr, 2, 2, 4);
    EXPECT_THROW(EncodePng(t, PngOptions{}, out), std::invalid_argument);
    t = TileImage::Packed(px, 0, 2, 4);
    EXPECT_THROW(EncodePng(t, PngOptions{}, out), std::invalid_argument);
    t = TileImage::Packed(px, 2, 2, 4);
    t.stride = 7;
    EXPECT_THROW(EncodePng(t, PngOptions{}, out), std::invalid_argument);
}

TEST(PngEncoder, ReleasedBytesAreMallocOwned) {
    const auto img = MakeImage(8, 8, 3, 24, 1);
    ByteBuffer out;
    EncodePng(TileImage::Packed(img.data(), 8, 8, 3), PngOptions{}, out);
    const size_t n = out.size();
    uint8_t* p = out.release();
    ASSERT_NE(p, nullptr);
    EXPECT_TRUE(out.empty());
    EXPECT_EQ(DecodePng(p, n).ok, true);
    std::free(p);  // ddbtFree
}
