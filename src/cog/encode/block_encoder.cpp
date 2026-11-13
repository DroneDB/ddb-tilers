// SPDX-License-Identifier: MPL-2.0
#include "cog/encode/block_encoder.h"

#include <libdeflate.h>
#include <zstd.h>

#include <algorithm>
#include <csetjmp>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>

// jpeglib.h needs size_t and FILE declared first.
#include <jerror.h>
#include <jpeglib.h>

#include "cog/codecs/encoders.h"

namespace ddbt::cog::encode {

namespace {

// ---------------------------------------------------------------------------------------------
// Per-thread scratch and compressors

struct DeflateDeleter {
    void operator()(libdeflate_compressor* c) const noexcept { libdeflate_free_compressor(c); }
};
struct ZstdDeleter {
    void operator()(ZSTD_CCtx* c) const noexcept { ZSTD_freeCCtx(c); }
};

libdeflate_compressor* ThreadDeflate(int level) {
    thread_local std::unique_ptr<libdeflate_compressor, DeflateDeleter> c[13];
    if (level < 1 || level > 12) level = 6;
    auto& slot = c[level];
    if (!slot) {
        slot.reset(libdeflate_alloc_compressor(level));
        if (!slot) throw std::runtime_error("DEFLATE: cannot allocate a compressor");
    }
    return slot.get();
}

ZSTD_CCtx* ThreadZstd() {
    thread_local std::unique_ptr<ZSTD_CCtx, ZstdDeleter> c;
    if (!c) {
        c.reset(ZSTD_createCCtx());
        if (!c) throw std::runtime_error("ZSTD: cannot allocate a context");
    }
    return c.get();
}

ByteBuffer& ThreadScratch() {
    thread_local ByteBuffer b;
    return b;
}

void Deflate(const uint8_t* in, size_t n, int level, ByteBuffer& out) {
    libdeflate_compressor* c = ThreadDeflate(level);
    const size_t bound = libdeflate_zlib_compress_bound(c, n);
    const size_t at = out.size();
    out.resize(at + bound);
    const size_t got = libdeflate_zlib_compress(c, in, n, out.data() + at, bound);
    if (got == 0) throw std::runtime_error("DEFLATE: compression failed");
    out.resize(at + got);
}

void Zstd(const uint8_t* in, size_t n, int level, ByteBuffer& out) {
    ZSTD_CCtx* c = ThreadZstd();
    const size_t bound = ZSTD_compressBound(n);
    const size_t at = out.size();
    out.resize(at + bound);
    const size_t got = ZSTD_compressCCtx(c, out.data() + at, bound, in, n, level);
    if (ZSTD_isError(got)) throw std::runtime_error(std::string("ZSTD: ") + ZSTD_getErrorName(got));
    out.resize(at + got);
}

// ---------------------------------------------------------------------------------------------
// JPEG (libjpeg API: TurboJPEG cannot write abbreviated datastreams)

struct ErrorMgr {
    jpeg_error_mgr pub;
    std::jmp_buf jump;
    char message[JMSG_LENGTH_MAX];
};

extern "C" void OnJpegEncodeError(j_common_ptr cinfo) {
    auto* err = reinterpret_cast<ErrorMgr*>(cinfo->err);
    (*cinfo->err->format_message)(cinfo, err->message);
    std::longjmp(err->jump, 1);
}

extern "C" void OnJpegEncodeMessage(j_common_ptr, int) {}

// Destination manager appending to a ByteBuffer.
struct Dest {
    jpeg_destination_mgr pub;
    ByteBuffer* out;
    size_t start;
};

constexpr size_t kJpegChunk = 64 << 10;

extern "C" void DestInit(j_compress_ptr cinfo) {
    auto* d = reinterpret_cast<Dest*>(cinfo->dest);
    d->start = d->out->size();
    d->out->resize(d->start + kJpegChunk);
    d->pub.next_output_byte = d->out->data() + d->start;
    d->pub.free_in_buffer = kJpegChunk;
}

extern "C" boolean DestEmpty(j_compress_ptr cinfo) {
    auto* d = reinterpret_cast<Dest*>(cinfo->dest);
    // libjpeg calls this when the whole buffer is full (free_in_buffer is not trustworthy here).
    const size_t used = d->out->size();
    d->out->resize(used + used / 2 + kJpegChunk);
    d->pub.next_output_byte = d->out->data() + used;
    d->pub.free_in_buffer = d->out->size() - used;
    return TRUE;
}

extern "C" void DestTerm(j_compress_ptr cinfo) {
    auto* d = reinterpret_cast<Dest*>(cinfo->dest);
    const size_t used = static_cast<size_t>(d->pub.next_output_byte - d->out->data());
    d->out->resize(used);
}

struct Compressor {
    jpeg_compress_struct cinfo{};
    ErrorMgr err{};
    Dest dest{};
    bool created = false;

    Compressor() = default;
    Compressor(const Compressor&) = delete;
    Compressor& operator=(const Compressor&) = delete;
    ~Compressor() {
        if (created) jpeg_destroy_compress(&cinfo);
    }
    bool Ensure() {
        if (created) return true;
        std::memset(&cinfo, 0, sizeof(cinfo));
        cinfo.err = jpeg_std_error(&err.pub);
        err.pub.error_exit = OnJpegEncodeError;
        err.pub.emit_message = OnJpegEncodeMessage;
        if (setjmp(err.jump)) return false;
        jpeg_create_compress(&cinfo);
        dest.pub.init_destination = DestInit;
        dest.pub.empty_output_buffer = DestEmpty;
        dest.pub.term_destination = DestTerm;
        cinfo.dest = &dest.pub;
        created = true;
        return true;
    }
};

Compressor& ThreadJpeg() {
    thread_local Compressor c;
    if (!c.Ensure()) throw std::runtime_error("JPEG: cannot create a compressor");
    return c;
}

// The settings of libtiff's JPEGPreEncode for PHOTOMETRIC_YCBCR, JPEGCOLORMODE_RGB, 8-bit, PIXEL.
void SetupYCbCr(jpeg_compress_struct* cinfo, uint32_t width, uint32_t height, int quality) {
    cinfo->image_width = width;
    cinfo->image_height = height;
    cinfo->input_components = 3;
    cinfo->in_color_space = JCS_RGB;
    jpeg_set_defaults(cinfo);
    jpeg_set_colorspace(cinfo, JCS_YCbCr);
    cinfo->comp_info[0].h_samp_factor = 2;
    cinfo->comp_info[0].v_samp_factor = 2;
    cinfo->comp_info[1].h_samp_factor = 1;
    cinfo->comp_info[1].v_samp_factor = 1;
    cinfo->comp_info[2].h_samp_factor = 1;
    cinfo->comp_info[2].v_samp_factor = 1;
    cinfo->write_JFIF_header = FALSE;
    cinfo->write_Adobe_marker = FALSE;
    jpeg_set_quality(cinfo, quality, FALSE);
}

void EncodeJpegYCbCr(const BlockFormat& f, const uint8_t* pixels, ByteBuffer& out) {
    if (f.type != tiff::SampleType::kU8 || (f.samples != 3 && f.samples != 4))
        throw std::invalid_argument("JPEG: 8-bit RGB or RGBX blocks only");
    Compressor& c = ThreadJpeg();
    jpeg_compress_struct* cinfo = &c.cinfo;
    c.dest.out = &out;
    const size_t at = out.size();
    // RGBX (an alpha the caller keeps in the mask): packed to RGB rows first.
    ByteBuffer& rgb = ThreadScratch();
    const uint8_t* src = pixels;
    if (f.samples == 4) {
        rgb.resize(static_cast<size_t>(f.width) * f.height * 3);
        const size_t n = static_cast<size_t>(f.width) * f.height;
        for (size_t i = 0; i < n; ++i) {
            rgb.data()[3 * i] = pixels[4 * i];
            rgb.data()[3 * i + 1] = pixels[4 * i + 1];
            rgb.data()[3 * i + 2] = pixels[4 * i + 2];
        }
        src = rgb.data();
    }
    if (setjmp(c.err.jump)) {
        jpeg_abort_compress(cinfo);
        out.resize(at);
        throw std::runtime_error(std::string("JPEG: ") + c.err.message);
    }
    SetupYCbCr(cinfo, f.width, f.height, f.jpegQuality);
    cinfo->optimize_coding = TRUE;
    // JPEGTABLESMODE=1: the quantization tables are in JPEGTables, not in the tile.
    for (int i = 0; i < NUM_QUANT_TBLS; ++i)
        if (cinfo->quant_tbl_ptrs[i] != nullptr) cinfo->quant_tbl_ptrs[i]->sent_table = TRUE;
    jpeg_start_compress(cinfo, FALSE);
    const size_t stride = static_cast<size_t>(f.width) * 3;
    while (cinfo->next_scanline < cinfo->image_height) {
        JSAMPROW rows[16];
        const JDIMENSION n = std::min<JDIMENSION>(16, cinfo->image_height - cinfo->next_scanline);
        for (JDIMENSION i = 0; i < n; ++i)
            rows[i] = const_cast<JSAMPROW>(src + (cinfo->next_scanline + i) * stride);
        jpeg_write_scanlines(cinfo, rows, n);
    }
    jpeg_finish_compress(cinfo);
}

// ---------------------------------------------------------------------------------------------

// JPEG zigzag order: entry z of a DQT segment is the coefficient at natural index kZigzag[z].
constexpr int kZigzag[64] = {0,  1,  8,  16, 9,  2,  3,  10, 17, 24, 32, 25, 18, 11, 4,  5,
                             12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6,  7,  14, 21, 28,
                             35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
                             58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63};

template <typename T>
void HorizontalDiff(uint8_t* data, uint32_t width, uint32_t height, uint32_t spp) {
    const size_t rowSamples = static_cast<size_t>(width) * spp;
    for (uint32_t y = 0; y < height; ++y) {
        T* row = reinterpret_cast<T*>(data) + y * rowSamples;
        for (size_t i = rowSamples; i-- > spp;) row[i] = static_cast<T>(row[i] - row[i - spp]);
    }
}

}  // namespace

uint16_t CompressionTag(Codec c) noexcept {
    switch (c) {
        case Codec::kNone: return 1;
        case Codec::kLzw: return 5;
        case Codec::kDeflate: return 8;
        case Codec::kZstd: return 50000;
        case Codec::kJpeg: return 7;
    }
    return 1;
}

const char* CodecName(Codec c) noexcept {
    switch (c) {
        case Codec::kNone: return "NONE";
        case Codec::kLzw: return "LZW";
        case Codec::kDeflate: return "DEFLATE";
        case Codec::kZstd: return "ZSTD";
        case Codec::kJpeg: return "JPEG";
    }
    return "NONE";
}

void ApplyHorizontalPredictor(uint8_t* data, uint32_t width, uint32_t height, uint32_t samples, size_t sampleBytes) {
    switch (sampleBytes) {
        case 1: HorizontalDiff<uint8_t>(data, width, height, samples); break;
        case 2: HorizontalDiff<uint16_t>(data, width, height, samples); break;
        case 4: HorizontalDiff<uint32_t>(data, width, height, samples); break;
        case 8: HorizontalDiff<uint64_t>(data, width, height, samples); break;
        default: throw std::invalid_argument("predictor 2: unsupported sample size");
    }
}

void ApplyFloatingPointPredictor(const uint8_t* in, uint8_t* out, uint32_t width, uint32_t height, uint32_t samples,
                                 size_t bps) {
    // libtiff fpDiff (little-endian host): byte planes, most significant byte first, then the
    // bytes of the row differenced with stride spp.
    const size_t wc = static_cast<size_t>(width) * samples;
    const size_t rowBytes = wc * bps;
    for (uint32_t y = 0; y < height; ++y) {
        const uint8_t* src = in + y * rowBytes;
        uint8_t* dst = out + y * rowBytes;
        for (size_t b = 0; b < bps; ++b) {
            uint8_t* plane = dst + (bps - b - 1) * wc;
            for (size_t i = 0; i < wc; ++i) plane[i] = src[i * bps + b];
        }
        for (size_t i = rowBytes; i-- > samples;) dst[i] = static_cast<uint8_t>(dst[i] - dst[i - samples]);
    }
}

void EncodeBlock(const BlockFormat& f, const uint8_t* pixels, ByteBuffer& out) {
    if (f.codec == Codec::kJpeg) {
        EncodeJpegYCbCr(f, pixels, out);
        return;
    }
    const size_t ss = tiff::SampleTypeSize(f.type);
    if (ss == 0) throw std::invalid_argument("block encoder: unsupported sample type");
    const size_t bytes = static_cast<size_t>(f.width) * f.height * f.samples * ss;
    const uint8_t* data = pixels;
    ByteBuffer& scratch = ThreadScratch();
    if (f.predictor == 2) {
        scratch.assign(pixels, bytes);
        ApplyHorizontalPredictor(scratch.data(), f.width, f.height, f.samples, ss);
        data = scratch.data();
    } else if (f.predictor == 3) {
        scratch.resize(bytes);
        ApplyFloatingPointPredictor(pixels, scratch.data(), f.width, f.height, f.samples, ss);
        data = scratch.data();
    }
    switch (f.codec) {
        case Codec::kNone: out.append(data, bytes); break;
        case Codec::kLzw: codecs::EncodeLzw({data, bytes}, out); break;
        case Codec::kDeflate: Deflate(data, bytes, f.level > 0 ? f.level : 6, out); break;
        case Codec::kZstd: Zstd(data, bytes, f.level > 0 ? f.level : 9, out); break;
        case Codec::kJpeg: break;
    }
}

void SpreadJpegEdges(uint8_t* pixels, uint32_t bw, uint32_t bh, uint32_t samples, uint32_t vw, uint32_t vh) {
    if (vw == 0 || vh == 0) return;
    const size_t stride = static_cast<size_t>(bw) * samples;
    if (vw < bw) {
        for (uint32_t y = 0; y < vh; ++y) {
            uint8_t* row = pixels + y * stride;
            const uint8_t* last = row + static_cast<size_t>(vw - 1) * samples;
            for (uint32_t x = vw; x < bw; ++x) std::memcpy(row + static_cast<size_t>(x) * samples, last, samples);
        }
    }
    for (uint32_t y = vh; y < bh; ++y) std::memcpy(pixels + y * stride, pixels + (vh - 1) * stride, stride);
}

void EncodeMaskBlock(const uint8_t* mask, uint32_t width, uint32_t height, ByteBuffer& out) {
    const size_t rowBytes = (width + 7) / 8;
    ByteBuffer& bits = ThreadScratch();
    bits.resize(rowBytes * height);
    for (uint32_t y = 0; y < height; ++y) {
        const uint8_t* m = mask + static_cast<size_t>(y) * width;
        uint8_t* b = bits.data() + y * rowBytes;
        uint32_t x = 0;
        for (; x + 8 <= width; x += 8, ++b) {
            *b = static_cast<uint8_t>(((m[x] != 0) << 7) | ((m[x + 1] != 0) << 6) | ((m[x + 2] != 0) << 5) |
                                      ((m[x + 3] != 0) << 4) | ((m[x + 4] != 0) << 3) | ((m[x + 5] != 0) << 2) |
                                      ((m[x + 6] != 0) << 1) | (m[x + 7] != 0));
        }
        if (x < width) {
            uint8_t v = 0;
            for (uint32_t k = 0; x < width; ++x, ++k)
                if (m[x] != 0) v |= static_cast<uint8_t>(0x80 >> k);
            *b = v;
        }
    }
    Deflate(bits.data(), bits.size(), 6, out);
}

std::vector<uint8_t> JpegTables(int quality) {
    Compressor& c = ThreadJpeg();
    jpeg_compress_struct* cinfo = &c.cinfo;
    ByteBuffer buf;
    c.dest.out = &buf;
    if (setjmp(c.err.jump)) {
        jpeg_abort_compress(cinfo);
        throw std::runtime_error(std::string("JPEG: ") + c.err.message);
    }
    SetupYCbCr(cinfo, 16, 16, quality);
    // Tables-only datastream with the quantization tables alone (libtiff prepare_JPEGTables with
    // JPEGTABLESMODE_QUANT): the Huffman tables are marked sent so they are not written.
    jpeg_suppress_tables(cinfo, TRUE);
    for (int i = 0; i < 2; ++i)
        if (cinfo->quant_tbl_ptrs[i] != nullptr) cinfo->quant_tbl_ptrs[i]->sent_table = FALSE;
    jpeg_write_tables(cinfo);
    return std::vector<uint8_t>(buf.data(), buf.data() + buf.size());
}

int GuessJpegQuality(std::span<const uint8_t> t, int components) {
    // Quantization tables of the tag, in zigzag order as stored.
    std::vector<std::vector<uint16_t>> tables;
    size_t i = 2;
    if (t.size() < 4 || t[0] != 0xFF || t[1] != 0xD8) return -1;
    while (i + 4 <= t.size()) {
        if (t[i] != 0xFF) return -1;
        const uint8_t marker = t[i + 1];
        if (marker == 0xD9) break;
        const size_t len = (static_cast<size_t>(t[i + 2]) << 8) | t[i + 3];
        if (len < 2 || i + 2 + len > t.size()) return -1;
        if (marker == 0xDB) {
            size_t p = i + 4;
            const size_t end = i + 2 + len;
            while (p < end) {
                const int pq = t[p] >> 4;
                const size_t entries = 64 * (pq ? 2 : 1);
                if (p + 1 + entries > end) return -1;
                std::vector<uint16_t> q(64);
                for (size_t k = 0; k < 64; ++k)
                    q[k] = pq ? static_cast<uint16_t>((t[p + 1 + 2 * k] << 8) | t[p + 2 + 2 * k]) : t[p + 1 + k];
                tables.push_back(std::move(q));
                p += 1 + entries;
            }
        }
        i += 2 + len;
    }
    const size_t want = components == 1 ? 1 : 2;
    if (tables.size() < want) return -1;
    Compressor& c = ThreadJpeg();
    jpeg_compress_struct* cinfo = &c.cinfo;
    if (setjmp(c.err.jump)) {
        jpeg_abort_compress(cinfo);
        return -1;
    }
    cinfo->in_color_space = JCS_RGB;
    cinfo->input_components = 3;
    jpeg_set_defaults(cinfo);
    for (int q = 1; q <= 100; ++q) {
        jpeg_set_quality(cinfo, q, FALSE);
        bool same = true;
        for (size_t k = 0; k < want && same; ++k) {
            const JQUANT_TBL* qt = cinfo->quant_tbl_ptrs[k];
            if (qt == nullptr) return -1;
            for (size_t z = 0; z < 64 && same; ++z) same = qt->quantval[kZigzag[z]] == tables[k][z];
        }
        if (same) return q;
    }
    return -1;
}

}  // namespace ddbt::cog::encode
