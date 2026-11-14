// SPDX-License-Identifier: MPL-2.0
//
// The TIFF compressions ddbt decodes through libtiff (CCITT RLE / Group 3 / Group 4, LZMA,
// 12-bit JPEG) and packed samples (NBITS), on files libtiff writes here: TiffReader gives back
// the samples that were written (12-bit JPEG: within the loss of quality 100), for strips and
// tiles, both byte orders, with and without predictors.

#include <gtest/gtest.h>

#include <tiffio.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

#include "cog/tiff/libtiff_codec.h"
#include "cog/tiff/tiff_reader.h"
#include "../test_env.h"

namespace ddbt::cog::tiff {
namespace {

struct Spec {
    uint32_t width = 67, height = 45;
    uint16_t spp = 1, bits = 8, sampleFormat = SAMPLEFORMAT_UINT;
    uint16_t compression = COMPRESSION_NONE, photometric = PHOTOMETRIC_MINISBLACK;
    uint16_t predictor = PREDICTOR_NONE, planar = PLANARCONFIG_CONTIG;
    uint32_t tile = 0;          // 0: strips of rowsPerStrip
    uint32_t rowsPerStrip = 8;
    bool bigEndian = false;
    int jpegQuality = 100;
};

std::filesystem::path Scratch(const std::string& name) {
    const auto d = std::filesystem::temp_directory_path() / ("ddbt-libtiff-test-" + test::ProcessIdString());
    std::filesystem::create_directories(d);
    return d / (name + ".tif");
}

// Row-major samples (pixel interleaved), each a value of `bits` bits in a uint32
std::vector<uint32_t> Samples(const Spec& s, uint32_t seed) {
    std::mt19937 rng(seed);
    const uint64_t range = uint64_t{1} << s.bits;  // values 0 .. range - 1
    std::vector<uint32_t> v(static_cast<size_t>(s.width) * s.height * s.spp);
    for (uint32_t y = 0; y < s.height; ++y)
        for (uint32_t x = 0; x < s.width; ++x)
            for (uint32_t c = 0; c < s.spp; ++c) {
                // smooth with noise (JPEG keeps it close), 1-bit: runs (fax codes)
                const uint64_t smooth = (uint64_t{x} * 7 + y * 3 + c * 50) % range;
                const uint32_t value =
                    s.bits == 1 ? ((x / 5 + y / 3) & 1) : static_cast<uint32_t>((smooth + rng() % 3) % range);
                v[(static_cast<size_t>(y) * s.width + x) * s.spp + c] = value;
            }
    return v;
}

// Packs rows of samples MSB first (TIFF's order for widths that are not whole bytes), or as
// whole little/big-endian words per libtiff's host order for 8/16/32 bits
std::vector<uint8_t> PackRows(const Spec& s, const std::vector<uint32_t>& v, uint32_t y0, uint32_t rows, uint32_t x0,
                              uint32_t cols) {
    const uint64_t rowBits = uint64_t{cols} * s.spp * s.bits;
    const size_t rowBytes = static_cast<size_t>((rowBits + 7) / 8);
    std::vector<uint8_t> out(rowBytes * rows, 0);
    for (uint32_t r = 0; r < rows; ++r) {
        uint8_t* dst = out.data() + r * rowBytes;
        uint64_t bit = 0;
        for (uint32_t x = 0; x < cols; ++x)
            for (uint32_t c = 0; c < s.spp; ++c) {
                const uint32_t yy = y0 + r, xx = x0 + x;
                const uint32_t value = (yy < s.height && xx < s.width) ? v[(static_cast<size_t>(yy) * s.width + xx) * s.spp + c] : 0;
                if (s.bits == 8 || s.bits == 16 || s.bits == 32) {
                    std::memcpy(dst + bit / 8, &value, s.bits / 8);  // host order: libtiff swaps on write
                } else {
                    for (int k = s.bits - 1; k >= 0; --k, ++bit)
                        if ((value >> k) & 1u) dst[bit >> 3] |= static_cast<uint8_t>(0x80u >> (bit & 7));
                    continue;
                }
                bit += s.bits;
            }
    }
    return out;
}

void Write(const std::filesystem::path& path, const Spec& s, const std::vector<uint32_t>& v) {
    TIFF* t = TIFFOpen(path.string().c_str(), s.bigEndian ? "wb" : "wl");
    ASSERT_NE(t, nullptr);
    TIFFSetField(t, TIFFTAG_IMAGEWIDTH, s.width);
    TIFFSetField(t, TIFFTAG_IMAGELENGTH, s.height);
    TIFFSetField(t, TIFFTAG_SAMPLESPERPIXEL, s.spp);
    TIFFSetField(t, TIFFTAG_BITSPERSAMPLE, s.bits);
    TIFFSetField(t, TIFFTAG_SAMPLEFORMAT, s.sampleFormat);
    TIFFSetField(t, TIFFTAG_PHOTOMETRIC, s.photometric);
    TIFFSetField(t, TIFFTAG_PLANARCONFIG, s.planar);
    TIFFSetField(t, TIFFTAG_COMPRESSION, s.compression);
    if (s.predictor != PREDICTOR_NONE) TIFFSetField(t, TIFFTAG_PREDICTOR, s.predictor);
    if (s.compression == COMPRESSION_JPEG) TIFFSetField(t, TIFFTAG_JPEGQUALITY, s.jpegQuality);
    // BAND interleave: one plane after the other, each a raster of one sample
    const bool separate = s.planar == PLANARCONFIG_SEPARATE;
    const uint32_t planes = separate ? s.spp : 1;
    if (s.tile != 0) {
        TIFFSetField(t, TIFFTAG_TILEWIDTH, s.tile);
        TIFFSetField(t, TIFFTAG_TILELENGTH, s.tile);
    } else {
        TIFFSetField(t, TIFFTAG_ROWSPERSTRIP, s.rowsPerStrip);
    }
    for (uint32_t plane = 0; plane < planes; ++plane) {
        Spec ps = s;
        std::vector<uint32_t> pv = v;
        if (separate) {
            ps.spp = 1;
            pv.resize(static_cast<size_t>(s.width) * s.height);
            for (size_t k = 0; k < pv.size(); ++k) pv[k] = v[k * s.spp + plane];
        }
        if (s.tile != 0) {
            for (uint32_t ty = 0; ty < s.height; ty += s.tile)
                for (uint32_t tx = 0; tx < s.width; tx += s.tile) {
                    auto block = PackRows(ps, pv, ty, s.tile, tx, s.tile);
                    ASSERT_GE(TIFFWriteEncodedTile(t, TIFFComputeTile(t, tx, ty, 0, static_cast<uint16_t>(plane)),
                                                   block.data(), static_cast<tmsize_t>(block.size())),
                              0);
                }
        } else {
            const uint32_t strips = (s.height + s.rowsPerStrip - 1) / s.rowsPerStrip;
            for (uint32_t y = 0, strip = 0; y < s.height; y += s.rowsPerStrip, ++strip) {
                const uint32_t rows = std::min(s.rowsPerStrip, s.height - y);
                auto block = PackRows(ps, pv, y, rows, 0, s.width);
                ASSERT_GE(TIFFWriteEncodedStrip(t, plane * strips + strip, block.data(),
                                                static_cast<tmsize_t>(block.size())),
                          0);
            }
        }
    }
    TIFFClose(t);
}

// Every sample TiffReader decodes, row-major and pixel interleaved, as uint32 (1-bit: 0 / 1)
std::vector<uint32_t> ReadAll(const std::filesystem::path& path) {
    auto reader = TiffReader::Open(path.string());
    EXPECT_TRUE(reader.ok()) << reader.status().message();
    if (!reader.ok()) return {};
    const TiffReader& r = **reader;
    const IfdInfo& i = r.Ifd(r.MainImage());
    std::vector<uint32_t> out(static_cast<size_t>(i.width) * i.height * i.samplesPerPixel);
    for (uint32_t b = 0; b < i.blockCount; ++b) {
        auto block = r.ReadBlock(i.index, b);
        EXPECT_TRUE(block.ok()) << block.status().message();
        if (!block.ok()) return {};
        const uint32_t x0 = i.BlockColumn(b) * i.blockWidth, y0 = i.BlockRow(b) * i.blockHeight;
        const uint32_t spp = i.SamplesPerBlockPixel();
        for (uint32_t y = 0; y < block->height && y0 + y < i.height; ++y)
            for (uint32_t x = 0; x < block->width && x0 + x < i.width; ++x)
                for (uint32_t c = 0; c < spp; ++c) {
                    const uint8_t* row = block->data() + y * block->rowStride;
                    uint32_t v = 0;
                    const size_t k = static_cast<size_t>(x) * spp + c;
                    switch (i.sampleType) {
                        case SampleType::kBit1: v = (row[x >> 3] >> (7 - (x & 7))) & 1u; break;
                        case SampleType::kU8: v = row[k]; break;
                        case SampleType::kU16: v = reinterpret_cast<const uint16_t*>(row)[k]; break;
                        case SampleType::kU32: v = reinterpret_cast<const uint32_t*>(row)[k]; break;
                        default: ADD_FAILURE() << "type"; return {};
                    }
                    const uint32_t plane = i.IsPixelInterleaved() ? c : i.BlockPlane(b);
                    out[(static_cast<size_t>(y0 + y) * i.width + x0 + x) * i.samplesPerPixel + plane] = v;
                }
    }
    return out;
}

void ExpectRoundTrip(const std::string& name, const Spec& s, uint32_t tolerance = 0) {
    if (s.compression != COMPRESSION_NONE && !LibtiffCodec::Supports(s.compression))
        GTEST_SKIP() << "libtiff has no codec " << s.compression;
    const auto path = Scratch(name);
    const auto v = Samples(s, 7);
    Write(path, s, v);
    const auto got = ReadAll(path);
    ASSERT_EQ(got.size(), v.size()) << name;
    uint32_t worst = 0;
    for (size_t k = 0; k < v.size(); ++k)
        worst = std::max(worst, got[k] > v[k] ? got[k] - v[k] : v[k] - got[k]);
    EXPECT_LE(worst, tolerance) << name;
    std::filesystem::remove(path);
}

TEST(LibtiffCodecs, LzmaStripsTilesBothByteOrdersAndPredictor) {
    ExpectRoundTrip("lzma-u8x3", {.spp = 3, .compression = COMPRESSION_LZMA});
    ExpectRoundTrip("lzma-u16-pred2-be", {.bits = 16, .compression = COMPRESSION_LZMA, .predictor = PREDICTOR_HORIZONTAL,
                                          .tile = 32, .bigEndian = true});
    ExpectRoundTrip("lzma-u32x2-band", {.spp = 2, .bits = 32, .compression = COMPRESSION_LZMA,
                                        .planar = PLANARCONFIG_SEPARATE});
}

TEST(LibtiffCodecs, CcittRleGroup3AndGroup4) {
    for (const uint16_t c : {COMPRESSION_CCITTRLE, COMPRESSION_CCITTFAX3, COMPRESSION_CCITTFAX4}) {
        ExpectRoundTrip("ccitt-" + std::to_string(c), {.bits = 1, .compression = c, .photometric = PHOTOMETRIC_MINISWHITE});
        ExpectRoundTrip("ccitt-tiled-" + std::to_string(c),
                        {.bits = 1, .compression = c, .photometric = PHOTOMETRIC_MINISBLACK, .tile = 32});
    }
}

TEST(LibtiffCodecs, PackedSamplesUnpackToTheirType) {
    // natively decoded codecs, then unpacked
    ExpectRoundTrip("nbits4-none", {.spp = 3, .bits = 4});
    ExpectRoundTrip("nbits2-deflate-tiled", {.bits = 2, .compression = COMPRESSION_ADOBE_DEFLATE, .tile = 16});
    ExpectRoundTrip("nbits12-lzw-be", {.spp = 2, .bits = 12, .compression = COMPRESSION_LZW, .bigEndian = true});
    ExpectRoundTrip("nbits12-band", {.spp = 3, .bits = 12, .planar = PLANARCONFIG_SEPARATE});
    ExpectRoundTrip("nbits24-deflate", {.bits = 24, .compression = COMPRESSION_ADOBE_DEFLATE});
    ExpectRoundTrip("nbits31", {.width = 13, .bits = 31});
    // through libtiff, then unpacked
    ExpectRoundTrip("nbits12-lzma", {.bits = 12, .compression = COMPRESSION_LZMA, .tile = 16});
}

TEST(LibtiffCodecs, TwelveBitJpeg) {
    // libtiff writes and reads 12-bit JPEG when libjpeg-turbo gives it the 12-bit API
    const auto probe = Scratch("jpeg12-probe");
    TIFF* t = TIFFOpen(probe.string().c_str(), "w");
    ASSERT_NE(t, nullptr);
    TIFFSetField(t, TIFFTAG_IMAGEWIDTH, 16);
    TIFFSetField(t, TIFFTAG_IMAGELENGTH, 16);
    TIFFSetField(t, TIFFTAG_BITSPERSAMPLE, 12);
    TIFFSetField(t, TIFFTAG_SAMPLESPERPIXEL, 1);
    TIFFSetField(t, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
    TIFFSetField(t, TIFFTAG_COMPRESSION, COMPRESSION_JPEG);
    TIFFSetField(t, TIFFTAG_ROWSPERSTRIP, 16);
    std::vector<uint8_t> zeros(16 * 16 * 2, 0);
    const bool writes = TIFFWriteEncodedStrip(t, 0, zeros.data(), 16 * 12 * 16 / 8) >= 0;
    TIFFClose(t);
    std::filesystem::remove(probe);
    if (!writes) GTEST_SKIP() << "libtiff has no 12-bit JPEG";
    ExpectRoundTrip("jpeg12", {.width = 64, .height = 48, .bits = 12, .compression = COMPRESSION_JPEG, .rowsPerStrip = 16}, 24);
    ExpectRoundTrip("jpeg12-tiled", {.width = 64, .height = 48, .bits = 12, .compression = COMPRESSION_JPEG, .tile = 32}, 24);
}

TEST(LibtiffCodecs, TheCodecsAreConfigured) {
    for (const uint16_t c : {COMPRESSION_CCITTRLE, COMPRESSION_CCITTFAX3, COMPRESSION_CCITTFAX4, COMPRESSION_OJPEG,
                             COMPRESSION_LZMA, COMPRESSION_JPEG})
        EXPECT_TRUE(LibtiffCodec::Supports(c)) << c;
}

}  // namespace
}  // namespace ddbt::cog::tiff
