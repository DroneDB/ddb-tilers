// SPDX-License-Identifier: MPL-2.0
//
// Uncompressed, PackBits, DEFLATE (libdeflate, whole buffer) and ZSTD block decoders.

#include <libdeflate.h>
#include <zstd.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "cog/codecs/codecs.h"

namespace ddbt::cog::codecs {

std::string_view CompressionName(uint16_t compression) noexcept {
    switch (static_cast<Compression>(compression)) {
        case Compression::kNone: return "NONE";
        case Compression::kCcittRle: return "CCITTRLE";
        case Compression::kCcittFax3: return "CCITTFAX3";
        case Compression::kCcittFax4: return "CCITTFAX4";
        case Compression::kLzw: return "LZW";
        case Compression::kOldJpeg: return "OJPEG";
        case Compression::kJpeg: return "JPEG";
        case Compression::kAdobeDeflate: return "DEFLATE";
        case Compression::kDeflate: return "DEFLATE";
        case Compression::kJbig: return "JBIG";
        case Compression::kNext: return "NEXT";
        case Compression::kPackBits: return "PACKBITS";
        case Compression::kThunderScan: return "THUNDERSCAN";
        case Compression::kLerc: return "LERC";
        case Compression::kLzma: return "LZMA";
        case Compression::kZstd: return "ZSTD";
        case Compression::kWebp: return "WEBP";
        case Compression::kJxl: return "JXL";
        case Compression::kJxlDng: return "JXL";
    }
    return "UNKNOWN";
}

bool IsSupportedCompression(uint16_t compression) noexcept {
    switch (static_cast<Compression>(compression)) {
        case Compression::kNone:
        case Compression::kLzw:
        case Compression::kJpeg:
        case Compression::kAdobeDeflate:
        case Compression::kDeflate:
        case Compression::kPackBits:
        case Compression::kLerc:
        case Compression::kZstd:
        case Compression::kWebp: return true;
        default: return false;
    }
}

Status DecodeNone(std::span<const uint8_t> in, std::span<uint8_t> out) {
    if (in.size() < out.size()) {
        return Corrupt("uncompressed block: " + std::to_string(in.size()) + " bytes stored, " +
                       std::to_string(out.size()) + " expected");
    }
    if (!out.empty()) std::memcpy(out.data(), in.data(), out.size());
    return Status::Ok();
}

Status DecodePackBits(std::span<const uint8_t> in, std::span<uint8_t> out) {
    const uint8_t* src = in.data();
    const uint8_t* const srcEnd = src + in.size();
    uint8_t* dst = out.data();
    uint8_t* const dstEnd = dst + out.size();
    while (dst < dstEnd && src < srcEnd) {
        const int n = static_cast<int8_t>(*src++);
        if (n >= 0) {
            // n + 1 literal bytes.
            size_t count = static_cast<size_t>(n) + 1;
            if (count > static_cast<size_t>(srcEnd - src)) {
                return Corrupt("PackBits: literal run past the end of the data");
            }
            if (count > static_cast<size_t>(dstEnd - dst)) count = static_cast<size_t>(dstEnd - dst);
            std::memcpy(dst, src, count);
            dst += count;
            src += static_cast<size_t>(n) + 1;
        } else if (n != -128) {
            // 1 - n copies of the next byte.
            if (src >= srcEnd) return Corrupt("PackBits: replicate run past the end of the data");
            size_t count = static_cast<size_t>(1 - n);
            if (count > static_cast<size_t>(dstEnd - dst)) count = static_cast<size_t>(dstEnd - dst);
            std::memset(dst, *src++, count);
            dst += count;
        }
        // -128: no-op.
    }
    if (dst < dstEnd) {
        return Corrupt("PackBits: not enough data (" + std::to_string(dstEnd - dst) + " bytes missing)");
    }
    return Status::Ok();
}

namespace {

struct DeflateDeleter {
    void operator()(libdeflate_decompressor* d) const noexcept { libdeflate_free_decompressor(d); }
};

libdeflate_decompressor* ThreadDecompressor() {
    thread_local std::unique_ptr<libdeflate_decompressor, DeflateDeleter> d;
    if (!d) {
        d.reset(libdeflate_alloc_decompressor());
        if (!d) throw std::bad_alloc();
    }
    return d.get();
}

struct ZstdDeleter {
    void operator()(ZSTD_DCtx* c) const noexcept { ZSTD_freeDCtx(c); }
};

ZSTD_DCtx* ThreadZstd() {
    thread_local std::unique_ptr<ZSTD_DCtx, ZstdDeleter> c;
    if (!c) {
        c.reset(ZSTD_createDCtx());
        if (!c) throw std::bad_alloc();
    }
    return c.get();
}

// Cap on the scratch buffer used when a DEFLATE stream decodes to more than the block.
constexpr size_t kMaxDeflateOverrun = size_t{256} << 20;

}  // namespace

Status DecodeDeflate(std::span<const uint8_t> in, std::span<uint8_t> out) {
    libdeflate_decompressor* d = ThreadDecompressor();
    size_t actual = 0;
    libdeflate_result r = libdeflate_zlib_decompress(d, in.data(), in.size(), out.data(), out.size(), &actual);
    if (r == LIBDEFLATE_SUCCESS) {
        if (actual < out.size()) {
            return Corrupt("DEFLATE: not enough data (" + std::to_string(actual) + " of " +
                           std::to_string(out.size()) + " bytes)");
        }
        return Status::Ok();
    }
    if (r == LIBDEFLATE_INSUFFICIENT_SPACE) {
        // The stream holds more than the block (some writers pad strips): libtiff stops when its
        // buffer is full. libdeflate needs room for the whole stream, so decode into scratch.
        size_t cap = out.size() < 4096 ? 8192 : out.size() * 2;
        while (cap <= kMaxDeflateOverrun) {
            std::vector<uint8_t> scratch(cap);
            r = libdeflate_zlib_decompress(d, in.data(), in.size(), scratch.data(), scratch.size(), &actual);
            if (r == LIBDEFLATE_SUCCESS) {
                if (actual < out.size()) return Corrupt("DEFLATE: not enough data");
                std::memcpy(out.data(), scratch.data(), out.size());
                return Status::Ok();
            }
            if (r != LIBDEFLATE_INSUFFICIENT_SPACE) break;
            cap *= 4;
        }
        if (r == LIBDEFLATE_INSUFFICIENT_SPACE) return Corrupt("DEFLATE: stream decodes to far more than the block");
    }
    return Corrupt("DEFLATE: invalid compressed data");
}

Status DecodeZstd(std::span<const uint8_t> in, std::span<uint8_t> out) {
    ZSTD_DCtx* ctx = ThreadZstd();
    const size_t r = ZSTD_decompressDCtx(ctx, out.data(), out.size(), in.data(), in.size());
    if (ZSTD_isError(r) && ZSTD_getErrorCode(r) == ZSTD_error_dstSize_tooSmall) {
        // More data than the block: libtiff streams and stops when its buffer is full.
        ZSTD_DCtx_reset(ctx, ZSTD_reset_session_only);
        ZSTD_inBuffer ib{in.data(), in.size(), 0};
        ZSTD_outBuffer ob{out.data(), out.size(), 0};
        while (ob.pos < ob.size && ib.pos < ib.size) {
            const size_t s = ZSTD_decompressStream(ctx, &ob, &ib);
            if (ZSTD_isError(s)) {
                ZSTD_DCtx_reset(ctx, ZSTD_reset_session_only);
                return Corrupt(std::string("ZSTD: ") + ZSTD_getErrorName(s));
            }
            if (s == 0 && ob.pos < ob.size && ib.pos >= ib.size) break;
        }
        ZSTD_DCtx_reset(ctx, ZSTD_reset_session_only);
        if (ob.pos < ob.size) return Corrupt("ZSTD: not enough data");
        return Status::Ok();
    }
    if (ZSTD_isError(r)) return Corrupt(std::string("ZSTD: ") + ZSTD_getErrorName(r));
    if (r < out.size()) {
        return Corrupt("ZSTD: not enough data (" + std::to_string(r) + " of " + std::to_string(out.size()) + " bytes)");
    }
    return Status::Ok();
}

}  // namespace ddbt::cog::codecs
