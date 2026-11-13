// SPDX-License-Identifier: MPL-2.0
//
// Block decoders: round trips through reference encoders (an LZW encoder written from the TIFF
// 6.0 specification below, libdeflate, libzstd, LERC, libwebp, TurboJPEG), hand-made edge
// cases (LZW KwKwK, full table, Clear codes, missing EOI, bad codes; PackBits runs), JPEG
// scaled / cropped decodes against full decodes, and random or mutated inputs, which must give
// an error status and never touch memory outside the spans (run under ASan / UBSan).

#include <gtest/gtest.h>

#include <Lerc_c_api.h>
#include <libdeflate.h>
#include <turbojpeg.h>
#include <webp/encode.h>
#include <zstd.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "cog/codecs/codecs.h"
#include "cog/codecs/jpeg.h"

namespace ddbt::cog::codecs {
namespace {

// ------------------------------------------------------------------------------------------
// Reference TIFF LZW encoder (MSB-first codes, early change, Clear when the table is full).

class BitWriter {
public:
    void Put(uint32_t code, uint32_t bits) {
        acc_ = (acc_ << bits) | code;
        n_ += bits;
        while (n_ >= 8) {
            out.push_back(static_cast<uint8_t>(acc_ >> (n_ - 8)));
            n_ -= 8;
        }
    }
    void Flush() {
        if (n_ > 0) out.push_back(static_cast<uint8_t>(acc_ << (8 - n_)));
        n_ = 0;
    }
    std::vector<uint8_t> out;

private:
    uint64_t acc_ = 0;
    uint32_t n_ = 0;
};

// clearAt: emit Clear when the next free code reaches this value (4094 like libtiff; 4096 to
// keep coding at 12 bits with a full table).
std::vector<uint8_t> LzwEncode(const std::vector<uint8_t>& in, uint32_t clearAt = 4094, bool eoi = true) {
    BitWriter w;
    uint32_t bits = 9;
    std::unordered_map<uint64_t, uint32_t> table;  // (prefix << 8 | byte) -> code
    uint32_t next = 258;
    w.Put(256, bits);
    if (in.empty()) {
        if (eoi) w.Put(257, bits);
        w.Flush();
        return w.out;
    }
    uint32_t prefix = in[0];
    for (size_t i = 1; i < in.size(); ++i) {
        const uint64_t key = (uint64_t{prefix} << 8) | in[i];
        auto it = table.find(key);
        if (it != table.end()) {
            prefix = it->second;
            continue;
        }
        w.Put(prefix, bits);
        if (next < 4096) {
            table[key] = next++;
            // The decoder widens when its next free code reaches 2^bits - 1; it adds its
            // entries one code later than the encoder, hence 2^bits.
            if (next == (1u << bits) && bits < 12) ++bits;
        }
        if (next >= clearAt && clearAt < 4096) {
            w.Put(256, bits);
            table.clear();
            next = 258;
            bits = 9;
        }
        prefix = in[i];
    }
    w.Put(prefix, bits);
    // The decoder adds one more entry after the last code.
    if (next < 4096 && next + 1 == (1u << bits) && bits < 12) ++bits;
    if (eoi) w.Put(257, bits);
    w.Flush();
    return w.out;
}

std::vector<uint8_t> TestData(size_t n, uint32_t seed, int kind) {
    std::mt19937 rng(seed);
    std::vector<uint8_t> v(n);
    for (size_t i = 0; i < n; ++i) {
        switch (kind) {
            case 0: v[i] = static_cast<uint8_t>(rng()); break;                       // incompressible
            case 1: v[i] = static_cast<uint8_t>((i / 7) % 5); break;                 // runs
            case 2: v[i] = 'a'; break;                                               // one byte: KwKwK chains
            default: v[i] = static_cast<uint8_t>((i * 31 + (rng() % 3)) & 0x3F); break;  // mixed
        }
    }
    return v;
}

TEST(Lzw, RoundTripsAcrossSizesAndPatterns) {
    for (int kind = 0; kind < 4; ++kind) {
        for (size_t n : {size_t{1}, size_t{2}, size_t{3}, size_t{100}, size_t{4095}, size_t{65536}, size_t{300000}}) {
            const auto data = TestData(n, static_cast<uint32_t>(n) + static_cast<uint32_t>(kind), kind);
            const auto enc = LzwEncode(data);
            std::vector<uint8_t> out(n);
            ASSERT_TRUE(DecodeLzw(enc, out).ok()) << "kind " << kind << " n " << n;
            ASSERT_EQ(out, data) << "kind " << kind << " n " << n;
        }
    }
}

TEST(Lzw, FullTableWithoutClearKeepsTwelveBitCodes) {
    const auto data = TestData(200000, 9, 0);
    const auto enc = LzwEncode(data, 4096);
    std::vector<uint8_t> out(data.size());
    ASSERT_TRUE(DecodeLzw(enc, out).ok());
    EXPECT_EQ(out, data);
}

TEST(Lzw, MissingEoiIsTreatedAsEnd) {
    const auto data = TestData(5000, 3, 3);
    const auto enc = LzwEncode(data, 4094, false);
    std::vector<uint8_t> out(data.size());
    ASSERT_TRUE(DecodeLzw(enc, out).ok());
    EXPECT_EQ(out, data);
}

TEST(Lzw, OutputSmallerThanStreamStopsWhenFull) {
    const auto data = TestData(10000, 4, 1);
    const auto enc = LzwEncode(data);
    std::vector<uint8_t> out(4321);
    ASSERT_TRUE(DecodeLzw(enc, out).ok());
    EXPECT_TRUE(std::equal(out.begin(), out.end(), data.begin()));
}

TEST(Lzw, ShortStreamIsCorrupt) {
    const auto data = TestData(10000, 5, 3);
    auto enc = LzwEncode(data);
    enc.resize(enc.size() / 2);
    std::vector<uint8_t> out(data.size());
    EXPECT_EQ(DecodeLzw(enc, out).code(), StatusCode::kCorrupt);
}

TEST(Lzw, RejectsOldStyleAndBadCodes) {
    std::vector<uint8_t> out(16);
    // Old-style (LSB-first) streams start 0x00 0x01.
    const std::vector<uint8_t> old = {0x00, 0x01, 0x02, 0x03};
    EXPECT_EQ(DecodeLzw(old, out).code(), StatusCode::kUnsupported);
    // Clear then a non-literal first code (300).
    BitWriter w;
    w.Put(256, 9);
    w.Put(300, 9);
    w.Flush();
    EXPECT_EQ(DecodeLzw(w.out, out).code(), StatusCode::kCorrupt);
    // A literal then a code past the next free entry (258 is next; 300 is not defined).
    BitWriter w2;
    w2.Put(256, 9);
    w2.Put('x', 9);
    w2.Put(300, 9);
    w2.Flush();
    EXPECT_EQ(DecodeLzw(w2.out, out).code(), StatusCode::kCorrupt);
    // Empty input, non-empty output.
    EXPECT_EQ(DecodeLzw({}, out).code(), StatusCode::kCorrupt);
}

TEST(Lzw, KwKwKAtTheEndOfTheBuffer) {
    // "aaaa...": every code after the first is the KwKwK case; sizes around the wild-copy slack.
    for (size_t n = 1; n < 80; ++n) {
        std::vector<uint8_t> data(n, 'a');
        const auto enc = LzwEncode(data);
        std::vector<uint8_t> out(n);
        ASSERT_TRUE(DecodeLzw(enc, out).ok()) << n;
        ASSERT_EQ(out, data) << n;
    }
}

// ------------------------------------------------------------------------------------------

TEST(PackBits, SpecificationExample) {
    // Apple TN1023 example.
    const std::vector<uint8_t> enc = {0xFE, 0xAA, 0x02, 0x80, 0x00, 0x2A, 0xFD, 0xAA, 0x03, 0x80, 0x00,
                                      0x2A, 0x22, 0xF7, 0xAA};
    const std::vector<uint8_t> expect = {0xAA, 0xAA, 0xAA, 0x80, 0x00, 0x2A, 0xAA, 0xAA, 0xAA, 0xAA, 0x80, 0x00,
                                         0x2A, 0x22, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
    std::vector<uint8_t> out(expect.size());
    ASSERT_TRUE(DecodePackBits(enc, out).ok());
    EXPECT_EQ(out, expect);
    // -128 is a no-op; runs past the output are clipped; short input is Corrupt.
    const std::vector<uint8_t> nop = {0x80, 0x00, 0x05};
    std::vector<uint8_t> one(1);
    ASSERT_TRUE(DecodePackBits(nop, one).ok());
    EXPECT_EQ(one[0], 5);
    std::vector<uint8_t> small(5);
    ASSERT_TRUE(DecodePackBits(enc, small).ok());
    std::vector<uint8_t> big(expect.size() + 1);
    EXPECT_EQ(DecodePackBits(enc, big).code(), StatusCode::kCorrupt);
    const std::vector<uint8_t> truncatedLiteral = {0x05, 0x01, 0x02};
    EXPECT_EQ(DecodePackBits(truncatedLiteral, big).code(), StatusCode::kCorrupt);
}

TEST(Deflate, RoundTripAndExtraOutput) {
    const auto data = TestData(100000, 1, 3);
    libdeflate_compressor* c = libdeflate_alloc_compressor(6);
    std::vector<uint8_t> enc(libdeflate_zlib_compress_bound(c, data.size()));
    enc.resize(libdeflate_zlib_compress(c, data.data(), data.size(), enc.data(), enc.size()));
    libdeflate_free_compressor(c);
    std::vector<uint8_t> out(data.size());
    ASSERT_TRUE(DecodeDeflate(enc, out).ok());
    EXPECT_EQ(out, data);
    // The stream holds more than the block: the block gets the first bytes.
    std::vector<uint8_t> part(1234);
    ASSERT_TRUE(DecodeDeflate(enc, part).ok());
    EXPECT_TRUE(std::equal(part.begin(), part.end(), data.begin()));
    // Too short.
    std::vector<uint8_t> more(data.size() + 10);
    EXPECT_EQ(DecodeDeflate(enc, more).code(), StatusCode::kCorrupt);
    enc.resize(enc.size() / 2);
    EXPECT_EQ(DecodeDeflate(enc, out).code(), StatusCode::kCorrupt);
}

TEST(Zstd, RoundTripAndExtraOutput) {
    const auto data = TestData(100000, 2, 3);
    std::vector<uint8_t> enc(ZSTD_compressBound(data.size()));
    enc.resize(ZSTD_compress(enc.data(), enc.size(), data.data(), data.size(), 3));
    std::vector<uint8_t> out(data.size());
    ASSERT_TRUE(DecodeZstd(enc, out).ok());
    EXPECT_EQ(out, data);
    std::vector<uint8_t> part(777);
    ASSERT_TRUE(DecodeZstd(enc, part).ok());
    EXPECT_TRUE(std::equal(part.begin(), part.end(), data.begin()));
    std::vector<uint8_t> more(data.size() + 1);
    EXPECT_EQ(DecodeZstd(enc, more).code(), StatusCode::kCorrupt);
    // The thread's context still works after errors.
    ASSERT_TRUE(DecodeZstd(enc, out).ok());
}

TEST(None, ShortInputIsCorrupt) {
    std::vector<uint8_t> in(10, 1), out(11);
    EXPECT_EQ(DecodeNone(in, out).code(), StatusCode::kCorrupt);
    out.resize(10);
    EXPECT_TRUE(DecodeNone(in, out).ok());
}

TEST(Lerc, FloatWithMaskBecomesNaNAndBytesWithAlpha) {
    const int w = 37, h = 23;
    // Float32 with invalid pixels.
    std::vector<float> f(size_t(w) * h);
    std::vector<uint8_t> valid(f.size());
    for (size_t i = 0; i < f.size(); ++i) {
        f[i] = static_cast<float>(i) * 0.25f - 30.f;
        valid[i] = (i % 7) != 0;
    }
    std::vector<uint8_t> blob(f.size() * 8 + 1024);
    unsigned int blobSize = 0;
    ASSERT_EQ(lerc_encode(f.data(), 6, 1, w, h, 1, 1, valid.data(), 0.0, blob.data(), static_cast<unsigned>(blob.size()),
                          &blobSize),
              0u);
    blob.resize(blobSize);
    LercBlockInfo info;
    info.width = w;
    info.height = h;
    info.samples = 1;
    info.bitsPerSample = 32;
    info.sampleFormat = 3;
    std::vector<uint8_t> out(f.size() * 4);
    ASSERT_TRUE(DecodeLerc(blob, info, out).ok());
    for (size_t i = 0; i < f.size(); ++i) {
        float v;
        std::memcpy(&v, out.data() + i * 4, 4);
        if (valid[i]) ASSERT_EQ(v, f[i]) << i;
        else ASSERT_TRUE(std::isnan(v)) << i;
    }
    // ZSTD layer.
    std::vector<uint8_t> z(ZSTD_compressBound(blob.size()));
    z.resize(ZSTD_compress(z.data(), z.size(), blob.data(), blob.size(), 1));
    info.additionalCompression = 2;
    std::vector<uint8_t> out2(out.size());
    ASSERT_TRUE(DecodeLerc(z, info, out2).ok());
    EXPECT_EQ(0, std::memcmp(out.data(), out2.data(), out.size()));

    // RGB bytes + mask -> RGBA with alpha 0 / 255 (libtiff's interleave).
    std::vector<uint8_t> rgb(size_t(w) * h * 3);
    for (size_t i = 0; i < rgb.size(); ++i) rgb[i] = static_cast<uint8_t>(i * 13);
    std::vector<uint8_t> blob2(rgb.size() * 4 + 1024);
    ASSERT_EQ(lerc_encode(rgb.data(), 1, 3, w, h, 1, 1, valid.data(), 0.0, blob2.data(),
                          static_cast<unsigned>(blob2.size()), &blobSize),
              0u);
    blob2.resize(blobSize);
    LercBlockInfo bi;
    bi.width = w;
    bi.height = h;
    bi.samples = 4;
    bi.bitsPerSample = 8;
    bi.sampleFormat = 1;
    bi.lastSampleIsUnassociatedAlpha = true;
    std::vector<uint8_t> rgba(size_t(w) * h * 4);
    ASSERT_TRUE(DecodeLerc(blob2, bi, rgba).ok());
    for (size_t i = 0; i < size_t(w) * h; ++i) {
        ASSERT_EQ(rgba[i * 4 + 3], valid[i] ? 255 : 0);
        if (valid[i]) {
            ASSERT_EQ(rgba[i * 4], rgb[i * 3]);
            ASSERT_EQ(rgba[i * 4 + 2], rgb[i * 3 + 2]);
        }
    }
    // Wrong block size / type.
    bi.width = w + 1;
    std::vector<uint8_t> wrong(size_t(w + 1) * h * 4);
    EXPECT_EQ(DecodeLerc(blob2, bi, wrong).code(), StatusCode::kCorrupt);
}

TEST(Webp, LosslessRoundTrip) {
    const int w = 61, h = 45;
    std::vector<uint8_t> rgba(size_t(w) * h * 4);
    for (size_t i = 0; i < rgba.size(); ++i) rgba[i] = static_cast<uint8_t>(i * 7 + i / 5);
    uint8_t* enc = nullptr;
    const size_t n = WebPEncodeLosslessRGBA(rgba.data(), w, h, w * 4, &enc);
    ASSERT_GT(n, 0u);
    std::vector<uint8_t> bytes(enc, enc + n);
    WebPFree(enc);
    std::vector<uint8_t> out(rgba.size());
    ASSERT_TRUE(DecodeWebp(bytes, w, h, 4, out).ok());
    EXPECT_EQ(out, rgba);
    EXPECT_EQ(DecodeWebp(bytes, w + 1, h, 4, out).code(), StatusCode::kCorrupt);
    bytes.resize(bytes.size() / 2);
    EXPECT_FALSE(DecodeWebp(bytes, w, h, 4, out).ok());
}

// ------------------------------------------------------------------------------------------
// JPEG

std::vector<uint8_t> EncodeJpeg(int w, int h, int comps, int subsamp, int quality = 85) {
    const size_t W = static_cast<size_t>(w), H = static_cast<size_t>(h), C = static_cast<size_t>(comps);
    std::vector<uint8_t> px(W * H * C);
    for (size_t y = 0; y < H; ++y)
        for (size_t x = 0; x < W; ++x)
            for (size_t c = 0; c < C; ++c)
                px[(y * W + x) * C + c] = static_cast<uint8_t>((x * (c + 1) * 3 + y * 2 + ((x ^ y) & 15)) & 255);
    tjhandle t = tj3Init(TJINIT_COMPRESS);
    tj3Set(t, TJPARAM_QUALITY, quality);
    tj3Set(t, TJPARAM_SUBSAMP, comps == 1 ? TJSAMP_GRAY : subsamp);
    unsigned char* buf = nullptr;
    size_t size = 0;
    EXPECT_EQ(tj3Compress8(t, px.data(), w, 0, h, comps == 1 ? TJPF_GRAY : TJPF_RGB, &buf, &size), 0);
    std::vector<uint8_t> out(buf, buf + size);
    tj3Free(buf);
    tj3Destroy(t);
    return out;
}

TEST(Jpeg, FullDecodeMatchesTurboJpeg) {
    const int w = 512, h = 512;
    const auto jpg = EncodeJpeg(w, h, 3, TJSAMP_420);
    JpegDecodeOptions o;
    o.ycbcrToRgb = true;
    std::vector<uint8_t> ours(size_t(w) * h * 3);
    ASSERT_TRUE(DecodeJpeg(jpg, o, 3, 0, 0, w, h, ours.data(), size_t(w) * 3).ok());
    tjhandle t = tj3Init(TJINIT_DECOMPRESS);
    std::vector<uint8_t> ref(ours.size());
    ASSERT_EQ(tj3DecompressHeader(t, jpg.data(), jpg.size()), 0);
    ASSERT_EQ(tj3Decompress8(t, jpg.data(), jpg.size(), ref.data(), 0, TJPF_RGB), 0);
    tj3Destroy(t);
    EXPECT_EQ(ours, ref);

    auto hdr = ReadJpegHeader(jpg, o);
    ASSERT_TRUE(hdr.ok());
    EXPECT_EQ(hdr->width, 512u);
    EXPECT_EQ(hdr->hSampling, 2u);
    EXPECT_EQ(hdr->vSampling, 2u);
}

TEST(Jpeg, ScaledDecodesMatchTurboJpegScaling) {
    const int w = 512, h = 384;
    const auto jpg = EncodeJpeg(w, h, 3, TJSAMP_420);
    for (uint32_t denom : {2u, 4u, 8u}) {
        const uint32_t sw = JpegScaledSize(w, denom), sh = JpegScaledSize(h, denom);
        JpegDecodeOptions o;
        o.ycbcrToRgb = true;
        o.scaleDenom = denom;
        std::vector<uint8_t> ours(size_t(sw) * sh * 3);
        ASSERT_TRUE(DecodeJpeg(jpg, o, 3, 0, 0, sw, sh, ours.data(), size_t(sw) * 3).ok()) << denom;
        tjhandle t = tj3Init(TJINIT_DECOMPRESS);
        ASSERT_EQ(tj3DecompressHeader(t, jpg.data(), jpg.size()), 0);
        tjscalingfactor sf{1, static_cast<int>(denom)};
        ASSERT_EQ(tj3SetScalingFactor(t, sf), 0);
        std::vector<uint8_t> ref(ours.size());
        ASSERT_EQ(tj3Decompress8(t, jpg.data(), jpg.size(), ref.data(), 0, TJPF_RGB), 0);
        tj3Destroy(t);
        EXPECT_EQ(ours, ref) << "1/" << denom;
    }
}

TEST(Jpeg, CroppedWindowsMatchTheFullDecode) {
    // libjpeg-turbo computes the cropped columns with the same context as a full decode
    // (it starts at an iMCU boundary and our copy skips the extra columns), and skipped rows
    // are decoded for the upsampling context: windows must equal the full decode.
    const int w = 512, h = 512;
    for (int subsamp : {TJSAMP_420, TJSAMP_444, TJSAMP_422}) {
        const auto jpg = EncodeJpeg(w, h, 3, subsamp);
        JpegDecodeOptions o;
        o.ycbcrToRgb = true;
        std::vector<uint8_t> full(size_t(w) * h * 3);
        ASSERT_TRUE(DecodeJpeg(jpg, o, 3, 0, 0, w, h, full.data(), size_t(w) * 3).ok());
        const uint32_t wins[][4] = {{0, 0, 256, 256}, {256, 256, 256, 256}, {100, 37, 211, 300}, {1, 1, 1, 1},
                                    {511, 0, 1, 512}, {0, 511, 512, 1},     {17, 300, 64, 64}};
        for (const auto& win : wins) {
            std::vector<uint8_t> part(size_t(win[2]) * win[3] * 3);
            ASSERT_TRUE(DecodeJpeg(jpg, o, 3, win[0], win[1], win[2], win[3], part.data(), size_t(win[2]) * 3).ok());
            int maxDiff = 0;
            for (uint32_t y = 0; y < win[3]; ++y)
                for (uint32_t x = 0; x < win[2] * 3; ++x)
                    maxDiff = std::max(maxDiff, std::abs(int(part[size_t(y) * win[2] * 3 + x]) -
                                                         int(full[(size_t(win[1] + y) * w + win[0]) * 3 + x])));
            EXPECT_EQ(maxDiff, 0) << "subsamp " << subsamp << " window " << win[0] << "," << win[1] << " " << win[2]
                                  << "x" << win[3];
        }
        // A window outside the image.
        std::vector<uint8_t> part(16 * 16 * 3);
        EXPECT_FALSE(DecodeJpeg(jpg, o, 3, 500, 500, 16, 16, part.data(), 48).ok());
    }
}

TEST(Jpeg, ScaledCroppedWindowsMatchTheScaledDecode) {
    const auto jpg = EncodeJpeg(512, 512, 3, TJSAMP_420);
    for (uint32_t denom : {2u, 4u, 8u}) {
        const uint32_t s = JpegScaledSize(512, denom);
        JpegDecodeOptions o;
        o.ycbcrToRgb = true;
        o.scaleDenom = denom;
        std::vector<uint8_t> full(size_t(s) * s * 3);
        ASSERT_TRUE(DecodeJpeg(jpg, o, 3, 0, 0, s, s, full.data(), size_t(s) * 3).ok());
        const uint32_t x = s / 4, y = s / 3, w = s / 2, h = s / 2;
        std::vector<uint8_t> part(size_t(w) * h * 3);
        ASSERT_TRUE(DecodeJpeg(jpg, o, 3, x, y, w, h, part.data(), size_t(w) * 3).ok());
        for (uint32_t r = 0; r < h; ++r) {
            ASSERT_EQ(0, std::memcmp(part.data() + size_t(r) * w * 3, full.data() + (size_t(y + r) * s + x) * 3, size_t(w) * 3))
                << "1/" << denom << " row " << r;
        }
    }
}

TEST(Jpeg, ComponentMismatchAndGarbage) {
    const auto gray = EncodeJpeg(64, 64, 1, TJSAMP_GRAY);
    JpegDecodeOptions o;
    std::vector<uint8_t> out(64 * 64 * 3);
    EXPECT_EQ(DecodeJpeg(gray, o, 3, 0, 0, 64, 64, out.data(), 64 * 3).code(), StatusCode::kCorrupt);
    ASSERT_TRUE(DecodeJpeg(gray, o, 1, 0, 0, 64, 64, out.data(), 64).ok());
    std::vector<uint8_t> junk(500, 0x5A);
    EXPECT_FALSE(DecodeJpeg(junk, o, 1, 0, 0, 64, 64, out.data(), 64).ok());
    // Truncated datastream: libjpeg warns and fills; the decode still succeeds as in libtiff.
    auto trunc = gray;
    trunc.resize(trunc.size() * 2 / 3);
    const uint64_t warnings = JpegWarningCount();
    EXPECT_TRUE(DecodeJpeg(trunc, o, 1, 0, 0, 64, 64, out.data(), 64).ok());
    EXPECT_GT(JpegWarningCount(), warnings);
    // The decoder still works.
    ASSERT_TRUE(DecodeJpeg(gray, o, 1, 0, 0, 64, 64, out.data(), 64).ok());
}

// ------------------------------------------------------------------------------------------
// Malformed inputs: random bytes and mutations of valid streams must not crash any decoder.

TEST(CodecFuzz, RandomAndMutatedInputsNeverOverrun) {
    std::mt19937 rng(1234);
    const auto plain = TestData(3000, 11, 3);
    std::vector<std::vector<uint8_t>> seeds;
    seeds.push_back(LzwEncode(plain));
    {
        libdeflate_compressor* c = libdeflate_alloc_compressor(6);
        std::vector<uint8_t> enc(libdeflate_zlib_compress_bound(c, plain.size()));
        enc.resize(libdeflate_zlib_compress(c, plain.data(), plain.size(), enc.data(), enc.size()));
        libdeflate_free_compressor(c);
        seeds.push_back(enc);
    }
    {
        std::vector<uint8_t> enc(ZSTD_compressBound(plain.size()));
        enc.resize(ZSTD_compress(enc.data(), enc.size(), plain.data(), plain.size(), 1));
        seeds.push_back(enc);
    }
    seeds.push_back(EncodeJpeg(48, 32, 3, TJSAMP_420, 50));
    seeds.push_back(std::vector<uint8_t>(64, 0));
    for (int iter = 0; iter < 3000; ++iter) {
        std::vector<uint8_t> in;
        if (iter % 4 == 0) {
            in.resize(rng() % 300);
            for (auto& b : in) b = static_cast<uint8_t>(rng());
        } else {
            in = seeds[rng() % seeds.size()];
            const int flips = 1 + static_cast<int>(rng() % 8);
            for (int f = 0; f < flips && !in.empty(); ++f) in[rng() % in.size()] = static_cast<uint8_t>(rng());
            if (rng() % 3 == 0) in.resize(rng() % (in.size() + 1));
        }
        // Exact-size heap buffers, so that ASan sees any overrun.
        std::vector<uint8_t> out(plain.size());
        DecodeLzw(in, out).IgnoreError();
        DecodePackBits(in, out).IgnoreError();
        DecodeDeflate(in, out).IgnoreError();
        DecodeZstd(in, out).IgnoreError();
        DecodeNone(in, out).IgnoreError();
        std::vector<uint8_t> img(48 * 32 * 3);
        JpegDecodeOptions o;
        o.ycbcrToRgb = true;
        DecodeJpeg(in, o, 3, 0, 0, 48, 32, img.data(), 48 * 3).IgnoreError();
        DecodeWebp(in, 48, 32, 3, img).IgnoreError();
        LercBlockInfo li;
        li.width = 48;
        li.height = 32;
        li.samples = 3;
        li.bitsPerSample = 8;
        DecodeLerc(in, li, img).IgnoreError();
    }
    SUCCEED();
}

}  // namespace
}  // namespace ddbt::cog::codecs
