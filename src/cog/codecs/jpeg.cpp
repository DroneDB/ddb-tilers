// SPDX-License-Identifier: MPL-2.0
#include "cog/codecs/jpeg.h"

#include <algorithm>
#include <csetjmp>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

// jpeglib.h needs size_t and FILE declared first.
#include <jerror.h>
#include <jpeglib.h>

namespace ddbt::cog::codecs {

namespace {

struct ErrorMgr {
    jpeg_error_mgr pub;
    std::jmp_buf jump;
    char message[JMSG_LENGTH_MAX];
};

thread_local uint64_t tlsWarnings = 0;

extern "C" void OnJpegError(j_common_ptr cinfo) {
    auto* err = reinterpret_cast<ErrorMgr*>(cinfo->err);
    (*cinfo->err->format_message)(cinfo, err->message);
    std::longjmp(err->jump, 1);
}

extern "C" void OnJpegMessage(j_common_ptr cinfo, int level) {
    // level < 0: warning (corrupt data that libjpeg works around); >= 0: trace messages.
    if (level < 0) {
        ++tlsWarnings;
        ++cinfo->err->num_warnings;
    }
}

// One decompressor per thread. `hasTables` remembers that tables of some JPEGTables tag were
// loaded: a later datastream without JPEGTables must not see them, so the object is re-created.
struct Decoder {
    jpeg_decompress_struct cinfo{};
    ErrorMgr err{};
    bool created = false;
    bool hasTables = false;

    Decoder() = default;
    Decoder(const Decoder&) = delete;
    Decoder& operator=(const Decoder&) = delete;
    ~Decoder() { Destroy(); }

    void Destroy() noexcept {
        if (created) jpeg_destroy_decompress(&cinfo);
        created = false;
        hasTables = false;
    }

    // Returns false when libjpeg could not allocate the object.
    bool Ensure(bool needFresh) {
        if (created && !needFresh) return true;
        Destroy();
        std::memset(&cinfo, 0, sizeof(cinfo));
        cinfo.err = jpeg_std_error(&err.pub);
        err.pub.error_exit = OnJpegError;
        err.pub.emit_message = OnJpegMessage;
        if (setjmp(err.jump)) return false;
        jpeg_create_decompress(&cinfo);
        created = true;
        return true;
    }
};

thread_local Decoder tlsDecoder;
thread_local std::vector<uint8_t> tlsRow;

struct Request {
    const uint8_t* data;
    size_t size;
    const uint8_t* tables;
    size_t tablesSize;
    bool ycbcrToRgb;
    uint32_t scaleDenom;
    // Decode window (scaled coordinates); headerOnly skips decoding.
    bool headerOnly;
    uint32_t expectedComponents;
    uint32_t x, y, width, height;
    uint8_t* out;
    size_t outStride;
    uint8_t* rowBuf;  // row buffer of at least scaled width * components bytes
};

enum class Result { kOk, kLibjpegError, kNotTablesOnly, kNoImage, kBadComponents, kBadPrecision, kBadWindow, kUnsupported };

// Everything between setjmp and the return uses only trivially destructible locals.
Result Run(Decoder& dec, const Request& rq, JpegHeader* header) {
    j_decompress_ptr cinfo = &dec.cinfo;
    if (setjmp(dec.err.jump)) {
        jpeg_abort_decompress(cinfo);
        return Result::kLibjpegError;
    }
    if (rq.tablesSize > 0) {
        jpeg_mem_src(cinfo, rq.tables, static_cast<unsigned long>(rq.tablesSize));
        const int t = jpeg_read_header(cinfo, FALSE);
        dec.hasTables = true;
        if (t != JPEG_HEADER_TABLES_ONLY) {
            jpeg_abort_decompress(cinfo);
            if (t != JPEG_HEADER_OK) return Result::kNotTablesOnly;
            // A full image in JPEGTables: keep going, its tables are loaded too.
        }
    }
    jpeg_mem_src(cinfo, rq.data, static_cast<unsigned long>(rq.size));
    if (jpeg_read_header(cinfo, TRUE) != JPEG_HEADER_OK) {
        jpeg_abort_decompress(cinfo);
        return Result::kNoImage;
    }
    header->width = cinfo->image_width;
    header->height = cinfo->image_height;
    header->components = static_cast<uint32_t>(cinfo->num_components);
    header->precision = static_cast<uint32_t>(cinfo->data_precision);
    header->hSampling = cinfo->num_components > 0 ? static_cast<uint32_t>(cinfo->comp_info[0].h_samp_factor) : 1;
    header->vSampling = cinfo->num_components > 0 ? static_cast<uint32_t>(cinfo->comp_info[0].v_samp_factor) : 1;
    header->progressive = cinfo->progressive_mode != 0;
    if (rq.headerOnly) {
        jpeg_abort_decompress(cinfo);
        return Result::kOk;
    }
    if (static_cast<uint32_t>(cinfo->num_components) != rq.expectedComponents) {
        jpeg_abort_decompress(cinfo);
        return Result::kBadComponents;
    }
    if (cinfo->data_precision != 8) {
        jpeg_abort_decompress(cinfo);
        return Result::kBadPrecision;
    }
    if (rq.ycbcrToRgb && cinfo->num_components == 3) {
        cinfo->jpeg_color_space = JCS_YCbCr;
        cinfo->out_color_space = JCS_RGB;
    } else {
        // libtiff: no colour handling. Subsampled components would need its raw-data path.
        cinfo->jpeg_color_space = JCS_UNKNOWN;
        cinfo->out_color_space = JCS_UNKNOWN;
        for (int c = 0; c < cinfo->num_components; ++c) {
            if (cinfo->comp_info[c].h_samp_factor != cinfo->comp_info[0].h_samp_factor ||
                cinfo->comp_info[c].v_samp_factor != cinfo->comp_info[0].v_samp_factor) {
                jpeg_abort_decompress(cinfo);
                return Result::kUnsupported;
            }
        }
    }
    cinfo->scale_num = 1;
    cinfo->scale_denom = rq.scaleDenom;
    cinfo->dct_method = JDCT_ISLOW;
    jpeg_start_decompress(cinfo);
    const uint32_t outW = cinfo->output_width;
    const uint32_t outH = cinfo->output_height;
    if (rq.width == 0 || rq.height == 0 || rq.x > outW || rq.width > outW - rq.x || rq.y > outH ||
        rq.height > outH - rq.y) {
        jpeg_abort_decompress(cinfo);
        return Result::kBadWindow;
    }
    const uint32_t comps = static_cast<uint32_t>(cinfo->output_components);
    // Fancy upsampling reads the neighbouring chroma samples: a crop or a skip that starts or
    // ends exactly at the window would make libjpeg treat the window edge as the image edge.
    // Keep one iMCU (in output pixels) of context on every side that is not an image edge;
    // the window is then bit-identical to the same pixels of a full decode.
    const uint32_t mcuW = std::max<uint32_t>(1, static_cast<uint32_t>(cinfo->max_h_samp_factor) * 8 / rq.scaleDenom);
    const uint32_t mcuH = std::max<uint32_t>(1, static_cast<uint32_t>(cinfo->max_v_samp_factor) * 8 / rq.scaleDenom);
    const uint32_t left = rq.x > mcuW ? rq.x - mcuW : 0;
    const uint32_t right = std::min<uint32_t>(outW, rq.x + rq.width + mcuW);
    const uint32_t top = rq.y > mcuH ? rq.y - mcuH : 0;
    JDIMENSION cropX = left;
    JDIMENSION cropW = right - left;
    const bool crop = left != 0 || right != outW;
    if (crop) jpeg_crop_scanline(cinfo, &cropX, &cropW);
    if (top > 0) jpeg_skip_scanlines(cinfo, top);
    const size_t skipBytes = static_cast<size_t>(rq.x - cropX) * comps;
    const size_t copyBytes = static_cast<size_t>(rq.width) * comps;
    const bool direct = !crop;
    // Context rows between the skip and the window.
    for (uint32_t r = top; r < rq.y; ++r) {
        JSAMPROW ptr = rq.rowBuf;
        if (jpeg_read_scanlines(cinfo, &ptr, 1) != 1) {
            jpeg_abort_decompress(cinfo);
            return Result::kLibjpegError;
        }
    }
    uint32_t row = 0;
    while (row < rq.height) {
        JSAMPROW ptr = direct ? rq.out + row * rq.outStride : rq.rowBuf;
        const JDIMENSION got = jpeg_read_scanlines(cinfo, &ptr, 1);
        if (got != 1) break;  // suspension cannot happen with a memory source
        if (!direct) std::memcpy(rq.out + row * rq.outStride, rq.rowBuf + skipBytes, copyBytes);
        ++row;
    }
    // Rows past the window are not needed: abort instead of finishing (no tail decode).
    jpeg_abort_decompress(cinfo);
    return row == rq.height ? Result::kOk : Result::kLibjpegError;
}

Status ToStatus(Result r, const Decoder& dec, const JpegHeader& h, uint32_t expectedComponents) {
    switch (r) {
        case Result::kOk: return Status::Ok();
        case Result::kLibjpegError: return Corrupt(std::string("JPEG: ") + dec.err.message);
        case Result::kNotTablesOnly: return Corrupt("JPEG: JPEGTables is not a tables-only datastream");
        case Result::kNoImage: return Corrupt("JPEG: no image in the datastream");
        case Result::kBadComponents:
            return Corrupt("JPEG: " + std::to_string(h.components) + " components, " +
                           std::to_string(expectedComponents) + " expected");
        case Result::kBadPrecision:
            return Unsupported("JPEG: " + std::to_string(h.precision) + "-bit data (only 8-bit is supported)");
        case Result::kBadWindow:
            return Corrupt("JPEG: the " + std::to_string(h.width) + "x" + std::to_string(h.height) +
                           " datastream does not cover the requested window");
        case Result::kUnsupported:
            return Unsupported("JPEG: subsampled components without YCbCr to RGB conversion");
    }
    return InternalError("JPEG: unexpected result");
}

Status Prepare(Decoder& dec, const JpegDecodeOptions& options) {
    if (options.scaleDenom != 1 && options.scaleDenom != 2 && options.scaleDenom != 4 && options.scaleDenom != 8) {
        return InvalidArgument("JPEG: scale must be 1/1, 1/2, 1/4 or 1/8");
    }
    // Without JPEGTables, tables left by another file must not apply.
    if (!dec.Ensure(options.tables.empty() && dec.hasTables)) {
        return ResourceExhausted("JPEG: cannot create a decompressor");
    }
    dec.err.message[0] = '\0';
    return Status::Ok();
}

}  // namespace

Expected<JpegHeader> ReadJpegHeader(std::span<const uint8_t> in, const JpegDecodeOptions& options) {
    Decoder& dec = tlsDecoder;
    DDBT_TRY(Prepare(dec, options));
    Request rq{};
    rq.data = in.data();
    rq.size = in.size();
    rq.tables = options.tables.data();
    rq.tablesSize = options.tables.size();
    rq.ycbcrToRgb = options.ycbcrToRgb;
    rq.scaleDenom = options.scaleDenom;
    rq.headerOnly = true;
    JpegHeader h;
    const Result r = Run(dec, rq, &h);
    if (r != Result::kOk) return ToStatus(r, dec, h, 0);
    return h;
}

Status DecodeJpeg(std::span<const uint8_t> in, const JpegDecodeOptions& options, uint32_t expectedComponents,
                  uint32_t x, uint32_t y, uint32_t width, uint32_t height, uint8_t* out, size_t outStride) {
    if (in.empty()) return Corrupt("JPEG: empty datastream");
    Decoder& dec = tlsDecoder;
    DDBT_TRY(Prepare(dec, options));
    // The row buffer covers a full scaled row, whatever the crop gives back (at most 65535 px).
    const size_t rowBytes = size_t{65536} * (expectedComponents == 0 ? 1 : expectedComponents);
    if (tlsRow.size() < rowBytes) tlsRow.resize(rowBytes);
    Request rq{};
    rq.data = in.data();
    rq.size = in.size();
    rq.tables = options.tables.data();
    rq.tablesSize = options.tables.size();
    rq.ycbcrToRgb = options.ycbcrToRgb;
    rq.scaleDenom = options.scaleDenom;
    rq.headerOnly = false;
    rq.expectedComponents = expectedComponents;
    rq.x = x;
    rq.y = y;
    rq.width = width;
    rq.height = height;
    rq.out = out;
    rq.outStride = outStride;
    rq.rowBuf = tlsRow.data();
    JpegHeader h;
    return ToStatus(Run(dec, rq, &h), dec, h, expectedComponents);
}

uint64_t JpegWarningCount() noexcept { return tlsWarnings; }

}  // namespace ddbt::cog::codecs
