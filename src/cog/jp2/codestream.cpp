// SPDX-License-Identifier: MPL-2.0
//
// Follows GDAL 3.13.3's OpenJPEG decoding (frmts/openjpeg/opjdatasetbase.h, MIT licence:
// setUpDecompress, readBlockInit): the codestream decoder, its parameters and the checks of the
// header. The copyright notices and the permission notice:
//
//   frmts/openjpeg/opjdatasetbase.h:
//     Copyright (c) 2010-2014, Even Rouault <even dot rouault at spatialys dot com>
//     Copyright (c) 2015, European Union (European Environment Agency)
//     Copyright (c) 2023, Grok Image Compression Inc.
//
//   Permission is hereby granted, free of charge, to any person obtaining a copy of this
//   software and associated documentation files (the "Software"), to deal in the Software
//   without restriction, including without limitation the rights to use, copy, modify, merge,
//   publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons
//   to whom the Software is furnished to do so, subject to the following conditions:
//
//   The above copyright notice and this permission notice shall be included in all copies or
//   substantial portions of the Software.
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//   INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
//   PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
//   FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
//   OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//   DEALINGS IN THE SOFTWARE.
#include "cog/jp2/codestream.h"

#include <openjpeg.h>

#include <algorithm>
#include <cstring>

namespace ddbt::cog::jp2 {

namespace {

// The codestream bytes OpenJPEG reads through its stream callbacks.
struct StreamState {
    std::shared_ptr<const io::ByteSource> src;
    uint64_t base = 0, length = 0, pos = 0;
};

OPJ_SIZE_T ReadCallback(void* buffer, OPJ_SIZE_T bytes, void* user) {
    auto* s = static_cast<StreamState*>(user);
    if (s->pos >= s->length) return static_cast<OPJ_SIZE_T>(-1);
    const uint64_t n = std::min<uint64_t>(bytes, s->length - s->pos);
    if (!s->src->Read(s->base + s->pos, {static_cast<uint8_t*>(buffer), static_cast<size_t>(n)}).ok())
        return static_cast<OPJ_SIZE_T>(-1);
    s->pos += n;
    return static_cast<OPJ_SIZE_T>(n);
}

OPJ_OFF_T SkipCallback(OPJ_OFF_T bytes, void* user) {
    auto* s = static_cast<StreamState*>(user);
    if (bytes < 0 && static_cast<uint64_t>(-bytes) > s->pos) s->pos = 0;
    else s->pos = static_cast<uint64_t>(static_cast<int64_t>(s->pos) + bytes);
    return bytes;
}

OPJ_BOOL SeekCallback(OPJ_OFF_T offset, void* user) {
    auto* s = static_cast<StreamState*>(user);
    s->pos = offset < 0 ? 0 : static_cast<uint64_t>(offset);
    return OPJ_TRUE;
}

void ErrorCallback(const char* message, void* user) {
    auto* errors = static_cast<std::string*>(user);
    std::string m(message != nullptr ? message : "");
    while (!m.empty() && (m.back() == '\n' || m.back() == '\r')) m.pop_back();
    if (!errors->empty()) *errors += "; ";
    *errors += m;
}

void QuietCallback(const char*, void*) {}

}  // namespace

struct Codestream::Impl {
    StreamState state;
    std::string errors;
    opj_codec_t* codec = nullptr;
    opj_stream_t* stream = nullptr;
    opj_image_t* image = nullptr;

    ~Impl() {
        if (image != nullptr) opj_image_destroy(image);
        if (stream != nullptr) opj_stream_destroy(stream);
        if (codec != nullptr) opj_destroy_codec(codec);
    }

    Status Fail(const std::string& what) const {
        return Status(StatusCode::kCorrupt, what + (errors.empty() ? std::string() : ": " + errors) + " (" +
                                                std::string(state.src->Name()) + ")");
    }
};

Codestream::Codestream(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
Codestream::~Codestream() = default;

Expected<std::unique_ptr<Codestream>> Codestream::Open(std::shared_ptr<const io::ByteSource> src, uint64_t offset,
                                                       uint64_t length, int threads) {
    auto impl = std::make_unique<Impl>();
    impl->state.src = std::move(src);
    impl->state.base = offset;
    impl->state.length = length;
    Impl& d = *impl;
    // Always the raw codestream: the boxes of a JP2 file are GDAL's to interpret.
    d.codec = opj_create_decompress(OPJ_CODEC_J2K);
    if (d.codec == nullptr) return Status(StatusCode::kInternal, "opj_create_decompress() failed");
    opj_set_error_handler(d.codec, ErrorCallback, &d.errors);
    opj_set_warning_handler(d.codec, QuietCallback, nullptr);
    opj_set_info_handler(d.codec, QuietCallback, nullptr);
    opj_dparameters_t parameters;
    opj_set_default_decoder_parameters(&parameters);
    if (!opj_setup_decoder(d.codec, &parameters)) return d.Fail("opj_setup_decoder() failed");
    if (threads > 1 && opj_has_thread_support()) opj_codec_set_threads(d.codec, threads);
    d.stream = opj_stream_create(64 * 1024, OPJ_TRUE);
    if (d.stream == nullptr) return Status(StatusCode::kInternal, "opj_stream_create() failed");
    opj_stream_set_user_data(d.stream, &d.state, nullptr);
    opj_stream_set_user_data_length(d.stream, length);
    opj_stream_set_read_function(d.stream, ReadCallback);
    opj_stream_set_skip_function(d.stream, SkipCallback);
    opj_stream_set_seek_function(d.stream, SeekCallback);
    if (!opj_read_header(d.stream, d.codec, &d.image) || d.image == nullptr) return d.Fail("opj_read_header() failed");

    CodestreamHeader h;
    opj_codestream_info_v2_t* info = opj_get_cstr_info(d.codec);
    if (info == nullptr) return d.Fail("opj_get_cstr_info() failed");
    h.tileWidth = info->tdx;
    h.tileHeight = info->tdy;
    h.tileX0 = info->tx0;
    h.tileY0 = info->ty0;
    h.tilesAcross = info->tw;
    h.tilesDown = info->th;
    h.resolutions = info->m_default_tile_info.tccp_info != nullptr
                        ? static_cast<int>(info->m_default_tile_info.tccp_info[0].numresolutions)
                        : 0;
    opj_destroy_cstr_info(&info);
    const opj_image_t* im = d.image;
    h.x0 = im->x0;
    h.y0 = im->y0;
    h.x1 = im->x1;
    h.y1 = im->y1;
    for (OPJ_UINT32 c = 0; c < im->numcomps; ++c) {
        const opj_image_comp_t& k = im->comps[c];
        h.components.push_back(ComponentInfo{k.dx, k.dy, k.w, k.h, k.prec, k.sgnd != 0});
    }
    // GDAL's JP2OpenJPEG refuses what it cannot address with 31-bit sizes, and an image whose
    // first component does not cover the image area.
    if (im->x1 <= im->x0 || im->y1 <= im->y0 || im->numcomps == 0 || (im->comps[0].w >> 31) != 0 ||
        (im->comps[0].h >> 31) != 0 || (h.tileWidth >> 31) != 0 || (h.tileHeight >> 31) != 0 ||
        im->comps[0].w != im->x1 - im->x0 || im->comps[0].h != im->y1 - im->y0)
        return Status(StatusCode::kUnsupported,
                      "Unable to handle that JPEG 2000 image (" + std::string(d.state.src->Name()) + ")");
    std::unique_ptr<Codestream> out(new Codestream(std::move(impl)));
    out->header_ = std::move(h);
    return out;
}

Status Codestream::DecodeArea(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1) {
    Impl& d = *impl_;
    d.errors.clear();
    if (!opj_set_decoded_resolution_factor(d.codec, 0)) return d.Fail("opj_set_decoded_resolution_factor() failed");
    for (OPJ_UINT32 c = 0; c < d.image->numcomps; ++c) d.image->comps[c].factor = 0;
    if (!opj_set_decode_area(d.codec, d.image, static_cast<OPJ_INT32>(x0), static_cast<OPJ_INT32>(y0),
                             static_cast<OPJ_INT32>(x1), static_cast<OPJ_INT32>(y1)))
        return d.Fail("opj_set_decode_area() failed");
    if (!opj_decode(d.codec, d.stream, d.image)) return d.Fail("opj_decode() failed");
    return Status::Ok();
}

Status Codestream::DecodeTile(uint32_t index) {
    Impl& d = *impl_;
    d.errors.clear();
    if (!opj_set_decoded_resolution_factor(d.codec, 0)) return d.Fail("opj_set_decoded_resolution_factor() failed");
    if (!opj_get_decoded_tile(d.codec, d.stream, d.image, index)) return d.Fail("opj_get_decoded_tile() failed");
    return Status::Ok();
}

DecodedComponent Codestream::Component(size_t c) const noexcept {
    const opj_image_t* im = impl_->image;
    if (im == nullptr || c >= im->numcomps) return {};
    const opj_image_comp_t& k = im->comps[c];
    return DecodedComponent{k.data, k.w, k.h};
}

}  // namespace ddbt::cog::jp2
