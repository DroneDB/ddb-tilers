// SPDX-License-Identifier: MPL-2.0
//
// The decoding rules are those of GDAL 3.13.3's JPEG driver (frmts/jpeg/jpgdataset.cpp: the
// colour spaces of JPGDataset::Open, the CMYK conversion of JPGRasterBand::IReadBlock, the memory
// check of StartDecompress, the warnings of EmitMessage, CheckForMask and DecompressMask), ported
// to C++20. Their notice:
//
//   Copyright (c) 2000, Frank Warmerdam
//   Copyright (c) 2007-2014, Even Rouault <even dot rouault at spatialys.com>
//
//   Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//   and associated documentation files (the "Software"), to deal in the Software without
//   restriction, including without limitation the rights to use, copy, modify, merge, publish,
//   distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
//   Software is furnished to do so, subject to the following conditions:
//
//   The above copyright notice and this permission notice shall be included in all copies or
//   substantial portions of the Software.
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//   BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
//   DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#include "cog/build/jpeg_source.h"

#include <algorithm>
#include <csetjmp>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <libdeflate.h>

// jpeglib.h needs size_t and FILE declared first.
#include <jerror.h>
#include <jpeglib.h>

namespace ddbt::cog::build {

namespace {

struct ErrorMgr {
    jpeg_error_mgr pub;
    std::jmp_buf jump;
    char message[JMSG_LENGTH_MAX];
    bool prematureEnd;
};

extern "C" void OnPhotoJpegError(j_common_ptr cinfo) {
    auto* err = reinterpret_cast<ErrorMgr*>(cinfo->err);
    (*cinfo->err->format_message)(cinfo, err->message);
    std::longjmp(err->jump, 1);
}

// GDAL's EmitMessage: a premature end of the file is an error, other warnings are not.
extern "C" void OnPhotoJpegMessage(j_common_ptr cinfo, int level) {
    if (level >= 0) return;
    auto* err = reinterpret_cast<ErrorMgr*>(cinfo->err);
    if (cinfo->err->msg_code == JWRN_JPEG_EOF) err->prematureEnd = true;
}

// A libjpeg decompressor over a memory buffer, destroyed with the object.
struct Decompressor {
    jpeg_decompress_struct cinfo{};
    ErrorMgr err{};
    bool created = false;

    Decompressor() {
        cinfo.err = jpeg_std_error(&err.pub);
        err.pub.error_exit = OnPhotoJpegError;
        err.pub.emit_message = OnPhotoJpegMessage;
    }
    Decompressor(const Decompressor&) = delete;
    Decompressor& operator=(const Decompressor&) = delete;
    ~Decompressor() {
        if (created) jpeg_destroy_decompress(&cinfo);
    }
};

// GDAL's SetMaxMemoryToUse: at least 500 MB before libjpeg would use a temporary file, and the
// refusal of StartDecompress beyond it (JPEGMEM unset).
constexpr uint64_t kMaxLibjpegMemory = uint64_t{500} * 1024 * 1024;

// The memory libjpeg needs for the coefficients of a multi-scan image (StartDecompress); 0 for
// invalid sampling factors.
uint64_t MultiScanMemory(const jpeg_decompress_struct& c) {
    uint64_t need = 1024 * 1024;
    for (int ci = 0; ci < c.num_components; ++ci) {
        const jpeg_component_info& comp = c.comp_info[ci];
        if (comp.h_samp_factor <= 0 || comp.v_samp_factor <= 0) return 0;
        const uint64_t w = (static_cast<uint64_t>(comp.width_in_blocks) + static_cast<uint64_t>(comp.h_samp_factor) - 1) /
                           static_cast<uint64_t>(comp.h_samp_factor);
        const uint64_t h = (static_cast<uint64_t>(comp.height_in_blocks) + static_cast<uint64_t>(comp.v_samp_factor) - 1) /
                           static_cast<uint64_t>(comp.v_samp_factor);
        need += w * h * sizeof(JBLOCK);
    }
    return need;
}

// The functions that call setjmp hold only trivially destructible locals: libjpeg's longjmp
// skips no destructor (and MSVC has nothing to warn about).
enum class Outcome : uint8_t { kOk, kLibjpegError, kPrematureEnd, kBadSampling, kTooMuchMemory, kBadOutput, kSinkError };

struct HeaderFacts {
    uint32_t width = 0, height = 0;
    int precision = 0;
    int colorSpace = 0;
};

Outcome ReadHeader(Decompressor& d, const uint8_t* data, size_t size, HeaderFacts& out) {
    j_decompress_ptr c = &d.cinfo;
    if (setjmp(d.err.jump)) return Outcome::kLibjpegError;
    jpeg_create_decompress(c);
    d.created = true;
    jpeg_mem_src(c, data, static_cast<unsigned long>(size));
    jpeg_read_header(c, TRUE);
    out.width = c->image_width;
    out.height = c->image_height;
    out.precision = c->data_precision;
    out.colorSpace = c->jpeg_color_space;
    return Outcome::kOk;
}

// What the decoding loop needs (all plain pointers and values).
struct DecodeJob {
    const uint8_t* data = nullptr;
    size_t size = 0;
    int outColorSpace = 0;
    int outComponents = 0;
    bool twelveBit = false;
    bool cmykToRgb = false;
    uint32_t width = 0, height = 0;
    uint8_t* scan = nullptr;  // a decoded scanline
    uint8_t* row = nullptr;   // a converted row (CMYK to RGB)
    RowSink* sink = nullptr;
    Status* sinkStatus = nullptr;
};

// The row to the sink, its failure into *sinkStatus.
bool EmitRow(RowSink& sink, uint32_t y, const uint8_t* row, Status* status) {
    *status = sink.Row(y, row);
    return status->ok();
}

Outcome RunDecode(Decompressor& d, const DecodeJob& j) {
    j_decompress_ptr c = &d.cinfo;
    if (setjmp(d.err.jump)) return Outcome::kLibjpegError;
    jpeg_create_decompress(c);
    d.created = true;
    c->mem->max_memory_to_use = std::max<long>(c->mem->max_memory_to_use, static_cast<long>(kMaxLibjpegMemory));
    jpeg_mem_src(c, j.data, static_cast<unsigned long>(j.size));
    jpeg_read_header(c, TRUE);
    c->out_color_space = static_cast<J_COLOR_SPACE>(j.outColorSpace);
    if (jpeg_has_multiple_scans(c)) {
        const uint64_t need = MultiScanMemory(*c);
        if (need == 0) return Outcome::kBadSampling;
        if (need > static_cast<uint64_t>(c->mem->max_memory_to_use)) return Outcome::kTooMuchMemory;
    }
    jpeg_start_decompress(c);
    if (c->output_components != j.outComponents || c->output_width != j.width) return Outcome::kBadOutput;
    for (uint32_t y = 0; y < j.height; ++y) {
        if (j.twelveBit) {
            J12SAMPROW r = reinterpret_cast<J12SAMPROW>(j.scan);
            jpeg12_read_scanlines(c, &r, 1);
        } else {
            JSAMPROW r = j.scan;
            jpeg_read_scanlines(c, &r, 1);
        }
        if (d.err.prematureEnd) return Outcome::kPrematureEnd;
        const uint8_t* out = j.scan;
        if (j.cmykToRgb) {
            // JPGRasterBand::IReadBlock: R = C * K / 255, G = M * K / 255, B = Y * K / 255.
            for (uint32_t x = 0; x < j.width; ++x) {
                const uint8_t* p = j.scan + static_cast<size_t>(x) * 4;
                const int k = p[3];
                j.row[static_cast<size_t>(x) * 3] = static_cast<uint8_t>((p[0] * k) / 255);
                j.row[static_cast<size_t>(x) * 3 + 1] = static_cast<uint8_t>((p[1] * k) / 255);
                j.row[static_cast<size_t>(x) * 3 + 2] = static_cast<uint8_t>((p[2] * k) / 255);
            }
            out = j.row;
        }
        if (!EmitRow(*j.sink, y, out, j.sinkStatus)) {
            jpeg_abort_decompress(c);
            return Outcome::kSinkError;
        }
    }
    jpeg_abort_decompress(c);
    return Outcome::kOk;
}

}  // namespace

bool IsJpegFile(const io::ByteSource& bytes) {
    uint8_t head[10];
    if (bytes.Size() < sizeof(head) || !bytes.Read(0, head).ok()) return false;
    return head[0] == 0xFF && head[1] == 0xD8 && head[2] == 0xFF;
}

Expected<std::unique_ptr<JpegSource>> JpegSource::Open(const std::string& path, uint64_t memoryBudget) {
    auto src = io::OpenByteSource(path, io::SourceKind::kPread);
    if (!src.ok()) return src.status();
    const uint64_t size = (*src)->Size();
    if (size > (uint64_t{1} << 40)) return Status(StatusCode::kUnsupported, "JPEG file too large: " + path);
    auto bytes = (*src)->ReadVector(0, static_cast<size_t>(size));
    if (!bytes.ok()) return bytes.status();
    std::unique_ptr<JpegSource> s(new JpegSource(path, memoryBudget));
    s->bytes_ = std::move(*bytes);
    s->imageBytes_ = s->bytes_.size();

    HeaderFacts h;
    {
        Decompressor d;
        if (ReadHeader(d, s->bytes_.data(), s->bytes_.size(), h) != Outcome::kOk)
            return Status(StatusCode::kCorrupt, std::string("JPEG: ") + d.err.message + ": " + path);
    }
    if (h.precision != 8 && h.precision != 12)
        return Status(StatusCode::kUnsupported,
                      "JPEG files with a precision other than 8 or 12 bits are not supported: " + path);
    s->precision_ = h.precision;
    s->jpegColorSpace_ = h.colorSpace;
    s->outColorSpace_ = h.colorSpace;

    ImageInfo info;
    info.width = h.width;
    info.height = h.height;
    info.type = s->precision_ == 12 ? tiff::SampleType::kU16 : tiff::SampleType::kU8;
    if (s->precision_ == 12) info.nbits = 12;
    const bool eightBit = s->precision_ == 8;
    switch (h.colorSpace) {
        case JCS_GRAYSCALE:
            info.bands = 1;
            info.interps = {"Gray"};
            break;
        case JCS_RGB:
            info.bands = 3;
            info.interps = {"Red", "Green", "Blue"};
            break;
        case JCS_YCbCr:
            info.bands = 3;
            info.interps = {"Red", "Green", "Blue"};
            s->outColorSpace_ = JCS_RGB;
            break;
        case JCS_CMYK:
            if (eightBit) {
                info.bands = 3;
                info.interps = {"Red", "Green", "Blue"};
                s->cmykToRgb_ = true;
            } else {
                info.bands = 4;
                info.interps = {"Cyan", "Magenta", "Yellow", "Black"};
            }
            break;
        case JCS_YCCK:
            if (eightBit) {
                // libjpeg converts YCCK to CMYK, then GDAL's arithmetic to RGB.
                info.bands = 3;
                info.interps = {"Red", "Green", "Blue"};
                s->outColorSpace_ = JCS_CMYK;
                s->cmykToRgb_ = true;
            } else {
                info.bands = 4;
                info.interps = {"YCbCr_Y", "YCbCr_Cb", "YCbCr_Cr", "Black"};
            }
            break;
        default:
            return Status(StatusCode::kUnsupported,
                          "Unrecognized JPEG colour space " + std::to_string(h.colorSpace) + ": " + path);
    }

    s->outComponents_ = s->outColorSpace_ == JCS_GRAYSCALE                             ? 1
                        : (s->outColorSpace_ == JCS_CMYK || s->outColorSpace_ == JCS_YCCK) ? 4
                                                                                          : 3;
    // CheckForMask: the last four bytes are the size of the JPEG datastream, which ends with EOI.
    const size_t n = s->bytes_.size();
    if (n >= 4) {
        const uint8_t* t = s->bytes_.data() + n - 4;
        const uint64_t imageSize =
            uint64_t{t[0]} | uint64_t{t[1]} << 8 | uint64_t{t[2]} << 16 | uint64_t{t[3]} << 24;
        if (imageSize >= 2 && imageSize >= n / 2 && imageSize <= n - 4 && s->bytes_[imageSize - 2] == 0xFF &&
            s->bytes_[imageSize - 1] == 0xD9) {
            s->maskOffset_ = static_cast<size_t>(imageSize);
            s->maskSize_ = n - static_cast<size_t>(imageSize) - 4;
            info.mask = true;
        }
    }
    DDBT_TRY(s->SetInfo(std::move(info)));
    return s;
}

Status JpegSource::Decode(RowSink& sink) const {
    const ImageInfo& info = Info();
    std::vector<uint8_t> scan(static_cast<size_t>(info.width) * static_cast<size_t>(outComponents_) *
                              (precision_ == 12 ? 2 : 1));
    std::vector<uint8_t> row(RowBytes());
    Status sinkStatus;
    DecodeJob j;
    j.data = bytes_.data();
    j.size = imageBytes_;
    j.outColorSpace = outColorSpace_;
    j.outComponents = outComponents_;
    j.twelveBit = precision_ == 12;
    j.cmykToRgb = cmykToRgb_;
    j.width = info.width;
    j.height = info.height;
    j.scan = scan.data();
    j.row = row.data();
    j.sink = &sink;
    j.sinkStatus = &sinkStatus;
    Decompressor d;
    switch (RunDecode(d, j)) {
        case Outcome::kOk: break;
        case Outcome::kLibjpegError:
            return Status(StatusCode::kCorrupt, std::string("JPEG: ") + d.err.message + ": " + Path());
        case Outcome::kPrematureEnd:
            return Status(StatusCode::kCorrupt, "JPEG: Premature end of JPEG file: " + Path());
        case Outcome::kBadSampling: return Status(StatusCode::kCorrupt, "JPEG: invalid sampling factor(s): " + Path());
        case Outcome::kTooMuchMemory:
            return Status(StatusCode::kUnsupported,
                          "Reading this JPEG would require libjpeg to allocate more than 500 MB: " + Path());
        case Outcome::kBadOutput: return Status(StatusCode::kInternal, "JPEG: unexpected output of libjpeg: " + Path());
        case Outcome::kSinkError: return sinkStatus;
    }
    if (info.mask) return DecodeMask(sink);
    return Status::Ok();
}

Status JpegSource::DecodeMask(RowSink& sink) const {
    // DecompressMask: the bits of every pixel in a row-major sequence (no row padding).
    const ImageInfo& info = Info();
    const uint64_t pixels = static_cast<uint64_t>(info.width) * info.height;
    const size_t bufSize = static_cast<size_t>(info.height) * ((static_cast<size_t>(info.width) + 7) / 8);
    std::vector<uint8_t> bits(bufSize);
    libdeflate_decompressor* dec = libdeflate_alloc_decompressor();
    if (dec == nullptr) return Status(StatusCode::kResourceExhausted, "libdeflate");
    size_t got = 0;
    const libdeflate_result r =
        libdeflate_zlib_decompress(dec, bytes_.data() + maskOffset_, maskSize_, bits.data(), bits.size(), &got);
    libdeflate_free_decompressor(dec);
    if (r != LIBDEFLATE_SUCCESS && r != LIBDEFLATE_SHORT_OUTPUT)
        return Status(StatusCode::kCorrupt, "Failure decoding JPEG validity bitmask: " + Path());
    (void)pixels;
    const int W = static_cast<int>(info.width), H = static_cast<int>(info.height);
    auto msbBit = [&](size_t i) { return (bits[i >> 3] & (0x1 << (7 - (i & 7)))) != 0; };
    // The bit order: LSB first, unless the mask is "obviously" MSB first (GDAL's heuristic: every
    // row the same, one change of value, which the LSB reading would see mid-byte).
    bool lsb = true;
    if (W > 8 && H > 1) {
        int prev = 0, changes = 0, x = 0;
        for (; x < W; ++x) {
            const int v = msbBit(static_cast<size_t>(x));
            if (x == 0) {
                prev = v;
            } else if (v != prev) {
                prev = v;
                ++changes;
                if (changes == 1) {
                    if (x % 8 == 0 && W % 8 == 0) break;
                } else {
                    break;
                }
            }
            if (v != static_cast<int>(msbBit(static_cast<size_t>(x + W)))) break;
        }
        lsb = !(x == W && changes == 1);
    }
    std::vector<uint8_t> row(info.width);
    for (uint32_t y = 0; y < info.height; ++y) {
        size_t bit = static_cast<size_t>(y) * info.width;
        for (uint32_t x = 0; x < info.width; ++x, ++bit) {
            const bool set = lsb ? (bits[bit >> 3] & (0x1 << (bit & 7))) != 0 : msbBit(bit);
            row[x] = set ? 255 : 0;
        }
        DDBT_TRY(sink.MaskRow(y, row.data()));
    }
    return Status::Ok();
}

}  // namespace ddbt::cog::build
