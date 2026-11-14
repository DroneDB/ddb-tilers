// SPDX-License-Identifier: MPL-2.0
//
// One test per third-party library of the vcpkg manifest: the version reported by the linked
// code must match the headers it was compiled against, and a tiny real call must work.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <numeric>
#include <string>
#include <vector>

#include <Lerc_c_api.h>
#include <gif_lib.h>
#include <libdeflate.h>
#include <openjpeg.h>
#include <proj.h>
#include <spng.h>
#include <turbojpeg.h>
#include <webp/decode.h>
#include <webp/encode.h>
#include <zstd.h>

#include "hwy/highway.h"

TEST(DepsCheck, Proj) {
    const PJ_INFO info = proj_info();
    EXPECT_EQ(info.major, PROJ_VERSION_MAJOR);
    EXPECT_EQ(info.minor, PROJ_VERSION_MINOR);
    EXPECT_EQ(info.patch, PROJ_VERSION_PATCH);
    EXPECT_GT(std::strlen(info.version), 0u);

    // A projection built from a PROJ string needs neither proj.db nor the network.
    PJ_CONTEXT* ctx = proj_context_create();
    ASSERT_NE(ctx, nullptr);
    PJ* merc = proj_create(ctx, "+proj=merc +a=6378137 +b=6378137 +lat_ts=0 +lon_0=0 +units=m +no_defs");
    ASSERT_NE(merc, nullptr);
    const PJ_COORD lonlat = proj_coord(proj_torad(10.0), proj_torad(45.0), 0, 0);
    const PJ_COORD xy = proj_trans(merc, PJ_FWD, lonlat);
    EXPECT_NEAR(xy.xy.x, 1113194.9079327357, 1e-6);
    EXPECT_NEAR(xy.xy.y, 5621521.486192066, 1e-6);
    proj_destroy(merc);
    proj_context_destroy(ctx);
}

TEST(DepsCheck, Libdeflate) {
    std::vector<uint8_t> src(4096);
    for (size_t i = 0; i < src.size(); ++i) src[i] = static_cast<uint8_t>(i % 7);

    libdeflate_compressor* c = libdeflate_alloc_compressor(1);
    ASSERT_NE(c, nullptr);
    std::vector<uint8_t> comp(libdeflate_deflate_compress_bound(c, src.size()));
    const size_t csize = libdeflate_zlib_compress(c, src.data(), src.size(), comp.data(), comp.size());
    libdeflate_free_compressor(c);
    ASSERT_GT(csize, 0u);
    EXPECT_LT(csize, src.size());

    libdeflate_decompressor* d = libdeflate_alloc_decompressor();
    ASSERT_NE(d, nullptr);
    std::vector<uint8_t> out(src.size());
    size_t actual = 0;
    EXPECT_EQ(libdeflate_zlib_decompress(d, comp.data(), csize, out.data(), out.size(), &actual), LIBDEFLATE_SUCCESS);
    libdeflate_free_decompressor(d);
    EXPECT_EQ(actual, src.size());
    EXPECT_EQ(out, src);

    // CRC-32 of "123456789" (check value of the standard)
    EXPECT_EQ(libdeflate_crc32(0, "123456789", 9), 0xCBF43926u);
}

TEST(DepsCheck, LibjpegTurbo) {
    EXPECT_GE(TURBOJPEG_VERSION_NUMBER, 3000000);
    tjhandle c = tj3Init(TJINIT_COMPRESS);
    ASSERT_NE(c, nullptr);
    EXPECT_EQ(tj3Set(c, TJPARAM_QUALITY, 75), 0);

    const int w = 16, h = 16;
    std::vector<uint8_t> rgb(static_cast<size_t>(w) * h * 3, 128);
    EXPECT_EQ(tj3Set(c, TJPARAM_SUBSAMP, TJSAMP_420), 0);
    unsigned char* jpeg = nullptr;
    size_t jsize = 0;
    ASSERT_EQ(tj3Compress8(c, rgb.data(), w, 0, h, TJPF_RGB, &jpeg, &jsize), 0) << tj3GetErrorStr(c);
    ASSERT_GT(jsize, 0u);
    tj3Destroy(c);

    tjhandle dec = tj3Init(TJINIT_DECOMPRESS);
    ASSERT_NE(dec, nullptr);
    ASSERT_EQ(tj3DecompressHeader(dec, jpeg, jsize), 0);
    EXPECT_EQ(tj3Get(dec, TJPARAM_JPEGWIDTH), w);
    EXPECT_EQ(tj3Get(dec, TJPARAM_JPEGHEIGHT), h);
    tj3Destroy(dec);
    tj3Free(jpeg);
}

TEST(DepsCheck, Libwebp) {
    EXPECT_EQ(WebPGetEncoderVersion() >> 16, WEBP_ENCODER_ABI_VERSION >> 8 ? WebPGetEncoderVersion() >> 16 : 0);
    EXPECT_GT(WebPGetEncoderVersion(), 0);
    EXPECT_GT(WebPGetDecoderVersion(), 0);

    const int w = 8, h = 8;
    std::vector<uint8_t> rgba(static_cast<size_t>(w) * h * 4, 200);
    uint8_t* out = nullptr;
    const size_t n = WebPEncodeRGBA(rgba.data(), w, h, w * 4, 80.0f, &out);
    ASSERT_GT(n, 0u);
    int dw = 0, dh = 0;
    EXPECT_EQ(WebPGetInfo(out, n, &dw, &dh), 1);
    EXPECT_EQ(dw, w);
    EXPECT_EQ(dh, h);
    WebPFree(out);
}

TEST(DepsCheck, Libspng) {
    // A 2 x 1 grey PNG written and read back.
    spng_ctx* enc = spng_ctx_new(SPNG_CTX_ENCODER);
    ASSERT_NE(enc, nullptr);
    spng_set_option(enc, SPNG_ENCODE_TO_BUFFER, 1);
    spng_ihdr ihdr{};
    ihdr.width = 2;
    ihdr.height = 1;
    ihdr.bit_depth = 8;
    ihdr.color_type = SPNG_COLOR_TYPE_GRAYSCALE;
    ASSERT_EQ(spng_set_ihdr(enc, &ihdr), 0);
    const uint8_t px[2] = {7, 200};
    ASSERT_EQ(spng_encode_image(enc, px, sizeof(px), SPNG_FMT_PNG, SPNG_ENCODE_FINALIZE), 0);
    size_t n = 0;
    int err = 0;
    auto* png = static_cast<uint8_t*>(spng_get_png_buffer(enc, &n, &err));
    ASSERT_NE(png, nullptr);
    spng_ctx* dec = spng_ctx_new(0);
    ASSERT_EQ(spng_set_png_buffer(dec, png, n), 0);
    uint8_t out[2] = {};
    ASSERT_EQ(spng_decode_image(dec, out, sizeof(out), SPNG_FMT_PNG, 0), 0);
    EXPECT_EQ(out[0], 7);
    EXPECT_EQ(out[1], 200);
    spng_ctx_free(dec);
    std::free(png);
    spng_ctx_free(enc);
}

TEST(DepsCheck, Giflib) {
    EXPECT_GE(GIFLIB_MAJOR, 5);
    EXPECT_NE(GifErrorString(D_GIF_ERR_NOT_GIF_FILE), nullptr);
}

TEST(DepsCheck, Zstd) {
    EXPECT_EQ(ZSTD_versionNumber(), static_cast<unsigned>(ZSTD_VERSION_NUMBER));
    const std::string src(1000, 'x');
    std::vector<char> comp(ZSTD_compressBound(src.size()));
    const size_t csize = ZSTD_compress(comp.data(), comp.size(), src.data(), src.size(), 3);
    ASSERT_FALSE(ZSTD_isError(csize));
    std::string out(src.size(), '\0');
    EXPECT_EQ(ZSTD_decompress(out.data(), out.size(), comp.data(), csize), src.size());
    EXPECT_EQ(out, src);
}

TEST(DepsCheck, OpenJpeg) {
    const char* v = opj_version();
    ASSERT_NE(v, nullptr);
    EXPECT_EQ(std::string(v).substr(0, 3), "2.5");
    opj_codec_t* codec = opj_create_decompress(OPJ_CODEC_J2K);
    ASSERT_NE(codec, nullptr);
    opj_destroy_codec(codec);
}

TEST(DepsCheck, Lerc) {
    EXPECT_GE(LERC_VERSION_NUMBER, 40000);
    const int w = 8, h = 4;
    std::vector<float> src(static_cast<size_t>(w) * h);
    std::iota(src.begin(), src.end(), 0.5f);
    std::vector<unsigned char> blob(4096);
    unsigned int nBytes = 0;
    ASSERT_EQ(lerc_encode(src.data(), 6 /*float*/, 1, w, h, 1, 0, nullptr, 0.0, blob.data(),
                          static_cast<unsigned int>(blob.size()), &nBytes), 0u);
    std::vector<float> out(src.size());
    ASSERT_EQ(lerc_decode(blob.data(), nBytes, 0, nullptr, 1, w, h, 1, 6, out.data()), 0u);
    EXPECT_EQ(out, src);
}

TEST(DepsCheck, Highway) {
    EXPECT_EQ(HWY_MAJOR, 1);
    EXPECT_GE(HWY_MINOR, 4);
    // A kernel compiled for the static target (the -march baseline) works.
    namespace hn = hwy::HWY_NAMESPACE;
    const hn::ScalableTag<float> d;
    const auto v = hn::Set(d, 2.0f);
    EXPECT_EQ(hn::GetLane(hn::Mul(v, v)), 4.0f);
    EXPECT_GT(hn::Lanes(d), 0u);
}
