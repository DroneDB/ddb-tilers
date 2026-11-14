// SPDX-License-Identifier: MPL-2.0
//
// Ported from GDAL 3.13.3, MIT licence: the band types, colour interpretation, 1-bit alpha
// promotion, 4:2:0 conversion, block layout and block reading of
// frmts/opjlike/jp2opjlikedataset.cpp and frmts/openjpeg/opjdatasetbase.h (JP2OPJLikeDataset,
// JP2OPJLikeRasterBand, JP2OPJDatasetBase). The copyright notices and the permission notice of
// the ported code:
//
//   frmts/opjlike/jp2opjlikedataset.cpp, frmts/openjpeg/opjdatasetbase.h:
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
#include "cog/build/jp2_source.h"

#include <algorithm>
#include <cstring>
#include <limits>

#include "core/exec/thread_pool.h"

namespace ddbt::cog::build {

namespace {

// GDALCopyWords from the decoder's 32-bit integers: values clamped into the band type.
template <typename T>
void CopyClamped(const int32_t* src, size_t n, uint8_t* out, size_t stride) {
    constexpr int64_t lo = static_cast<int64_t>(std::numeric_limits<T>::lowest());
    constexpr int64_t hi = static_cast<int64_t>(std::numeric_limits<T>::max());
    for (size_t i = 0; i < n; ++i) {
        const T v = static_cast<T>(std::clamp<int64_t>(src[i], lo, hi));
        std::memcpy(out + i * stride, &v, sizeof(T));
    }
}

uint8_t Clamp255(int v) { return static_cast<uint8_t>(std::clamp(v, 0, 255)); }

}  // namespace

Expected<std::unique_ptr<Jp2Source>> Jp2Source::Open(const std::string& path, uint64_t memoryBudget) {
    // pread, not a mapping, as for GeoTIFFs: a build reads the file once.
    auto bytes = io::OpenByteSource(path, io::SourceKind::kPread);
    if (!bytes.ok()) return bytes.status();
    std::unique_ptr<Jp2Source> s(new Jp2Source());
    s->path_ = path;
    s->bytes_ = std::shared_ptr<const io::ByteSource>(std::move(*bytes));
    auto file = jp2::ReadJp2File(*s->bytes_);
    if (!file.ok()) return file.status();
    s->file_ = std::move(*file);
    {
        auto cs = s->OpenCodestream(1);
        if (!cs.ok()) return cs.status();
        s->header_ = (*cs)->Header();
    }
    const jp2::CodestreamHeader& h = s->header_;
    const std::vector<jp2::ComponentInfo>& c = h.components;
    const int bands = static_cast<int>(c.size());
    if (c[0].prec > 16) s->type_ = c[0].sgnd ? tiff::SampleType::kI32 : tiff::SampleType::kU32;
    else if (c[0].prec > 8) s->type_ = c[0].sgnd ? tiff::SampleType::kI16 : tiff::SampleType::kU16;
    else s->type_ = tiff::SampleType::kU8;

    // The colour boxes (the codestream alone has no colour space GDAL knows).
    if (s->file_.colourSpace) s->colourSpace_ = *s->file_.colourSpace;
    s->roles_ = jp2::ReadChannelDefinitions(s->file_.channelDefinitions, bands);
    // GDAL's work-around for the NRW orthophotos (dop10rgbi) that declare a grey colour space
    // for their RGB + infrared components.
    if (s->file_.jp2 && s->colourSpace_ == jp2::ColourSpace::kGray && bands == 4 && s->roles_.red == 0 &&
        s->roles_.green == 1 && s->roles_.blue == 2 && path.find("dop10rgbi") != std::string::npos)
        s->colourSpace_ = jp2::ColourSpace::kSrgb;

    s->is420_ = s->colourSpace_ != jp2::ColourSpace::kSrgb && s->type_ == tiff::SampleType::kU8 &&
                (bands == 3 || bands == 4) && c[1].w == c[0].w / 2 && c[1].h == c[0].h / 2 && c[2].w == c[0].w / 2 &&
                c[2].h == c[0].h / 2 && (bands == 3 || (c[3].w == c[0].w && c[3].h == c[0].h));
    if (!s->is420_) {
        for (const jp2::ComponentInfo& k : c)
            if (k.w != c[0].w || k.h != c[0].h)
                return Status(StatusCode::kUnsupported,
                              "Unable to handle that JPEG 2000 image: components of different sizes (" + path + ")");
    }
    const int alpha = s->roles_.alpha;
    s->promoteAlpha_ = alpha >= 0 && alpha < bands && c[(alpha == 0 && bands > 1) ? 1 : 0].prec == 8 &&
                       c[static_cast<size_t>(alpha)].prec == 1;
    if (s->file_.palette) {
        std::vector<std::array<uint8_t, 3>> table;
        for (const auto& e : *s->file_.palette) table.push_back({e[0], e[1], e[2]});
        s->colorTable_ = std::move(table);
    }

    // GDAL's blocks (preferPerBlockDeCompress).
    const uint32_t W = h.Width(), H = h.Height();
    const bool singleTiled = W == h.tileWidth && H == h.tileHeight;
    uint32_t bw = h.tileWidth, bh = h.tileHeight;
    bool area = false;
    if (W <= 1024 && H <= 1024 && h.tileWidth < 32 && h.tileHeight < 32) {
        // Tiny tiles (some Sentinel-2 previews): the whole image as one block.
        area = true;
        bw = W;
        bh = H;
    } else {
        area = singleTiled && (W > 1024 || H > 1024);
        if (W < h.tileWidth && H < h.tileHeight) {
            area = true;
            bw = std::min<uint32_t>(W, 2048);
            bh = std::min<uint32_t>(H, 2048);
        } else if (area && !(h.tileHeight == 1 && h.tileWidth < 20 * 1024 * 1024)) {
            bw = std::min<uint32_t>(bw, 1024);
            bh = std::min<uint32_t>(bh, 1024);
        }
    }
    s->blockWidth_ = bw;
    s->blockHeight_ = bh;
    s->mode_ = !area ? Mode::kTile : singleTiled ? Mode::kSharedArea : Mode::kArea;

    // Threads: the pool decodes blocks in parallel; a single codestream kept open, or fewer
    // blocks than workers, get OpenJPEG's own threads.
    const uint64_t pool = std::max<size_t>(1, exec::DefaultPool().Size());
    const uint64_t blocks = std::max<uint64_t>(1, uint64_t{s->BlocksAcross()} * s->BlocksDown());
    s->threadsPerDecode_ = static_cast<int>(s->mode_ == Mode::kSharedArea ? pool : std::max<uint64_t>(1, pool / blocks));
    // A decode holds the 32-bit samples of its area or tile and the block: at most a quarter of
    // the budget for the decodes running at the same time.
    const uint64_t decodeW = s->mode_ == Mode::kTile ? h.tileWidth : bw;
    const uint64_t decodeH = s->mode_ == Mode::kTile ? h.tileHeight : bh;
    const uint64_t perDecode = decodeW * decodeH * static_cast<uint64_t>(bands) * 4 + s->BlockBytes();
    s->freeSlots_ = static_cast<int>(std::clamp<uint64_t>((memoryBudget / 4) / std::max<uint64_t>(perDecode, 1), 1, pool));

    s->georef_ = jp2::ReadJp2Georeferencing(s->file_, path, &s->rpc_);
    GtiffMetadata(jp2::Jp2Metadata(s->file_), s->tags_, s->gdalMetadata_);
    return s;
}

Expected<std::unique_ptr<jp2::Codestream>> Jp2Source::OpenCodestream(int threads) const {
    return jp2::Codestream::Open(bytes_, file_.codestreamOffset, file_.codestreamLength, threads);
}

int Jp2Source::AlphaBand() const noexcept {
    // A colour table makes the first band a palette whatever the cdef box says.
    if (roles_.alpha < 0 || roles_.alpha >= Bands() || (roles_.alpha == 0 && colorTable_)) return -1;
    return roles_.alpha;
}

std::string Jp2Source::ColorInterp(int b) const {
    if (b == 0 && colorTable_) return "Palette";
    if (b == roles_.alpha) return "Alpha";
    if (Bands() <= 2 && colourSpace_ == jp2::ColourSpace::kGray) return "Gray";
    if (colourSpace_ == jp2::ColourSpace::kSrgb || colourSpace_ == jp2::ColourSpace::kSycc) {
        if (b == roles_.red) return "Red";
        if (b == roles_.green) return "Green";
        if (b == roles_.blue) return "Blue";
    }
    return "Undefined";
}

std::optional<std::vector<std::array<uint8_t, 3>>> Jp2Source::ColorTable() const { return colorTable_; }

std::optional<int> Jp2Source::Nbits(int b) const {
    if (b < 0 || b >= Bands()) return std::nullopt;
    const int bits = promoteAlpha_ && b == roles_.alpha ? 8 : static_cast<int>(header_.components[static_cast<size_t>(b)].prec);
    if (bits % 8 == 0) return std::nullopt;
    return bits;
}

void Jp2Source::AcquireSlot() const {
    std::unique_lock<std::mutex> lock(slotMutex_);
    slotFree_.wait(lock, [&] { return freeSlots_ > 0; });
    --freeSlots_;
}

void Jp2Source::ReleaseSlot() const {
    {
        std::lock_guard<std::mutex> lock(slotMutex_);
        ++freeSlots_;
    }
    slotFree_.notify_one();
}

Status Jp2Source::Decode(jp2::Codestream& cs, uint32_t col, uint32_t row) const {
    if (mode_ == Mode::kTile) return cs.DecodeTile(col + row * BlocksAcross());
    const uint32_t x = col * blockWidth_, y = row * blockHeight_;
    const uint32_t w = std::min(blockWidth_, Width() - x), h = std::min(blockHeight_, Height() - y);
    return cs.DecodeArea(header_.x0 + x, header_.y0 + y, header_.x0 + x + w, header_.y0 + y + h);
}

Status Jp2Source::Convert(const jp2::Codestream& cs, uint32_t width, uint32_t rows, uint8_t* out) const {
    const int bands = Bands();
    const size_t ss = SampleSize();
    const size_t pb = PixelBytes();
    std::vector<jp2::DecodedComponent> comp(static_cast<size_t>(bands));
    for (int b = 0; b < bands; ++b) {
        comp[static_cast<size_t>(b)] = cs.Component(static_cast<size_t>(b));
        if (comp[static_cast<size_t>(b)].data == nullptr)
            return Status(StatusCode::kCorrupt, "JPEG 2000 component " + std::to_string(b) + " not decoded in " + path_);
    }
    const size_t rowBytes = static_cast<size_t>(blockWidth_) * pb;
    if (is420_) {
        // 4:2:0 YCbCr to RGB as GDAL converts it (YCbCr420ToBand), the fourth component copied.
        const jp2::DecodedComponent &Y = comp[0], &Cb = comp[1], &Cr = comp[2];
        if (Y.w < width || Y.h < rows || Cb.w != (Y.w + 1) / 2 || Cb.h != (Y.h + 1) / 2 || Cr.w != (Y.w + 1) / 2 ||
            Cr.h != (Y.h + 1) / 2 || (bands == 4 && (comp[3].w < width || comp[3].h < rows)))
            return Status(StatusCode::kCorrupt, "Unexpected 4:2:0 JPEG 2000 block sizes in " + path_);
        for (uint32_t j = 0; j < rows; ++j) {
            uint8_t* dst = out + j * rowBytes;
            for (uint32_t i = 0; i < width; ++i) {
                const int y = Y.data[static_cast<size_t>(j) * Y.w + i];
                const int cb = Cb.data[static_cast<size_t>(j / 2) * Cb.w + i / 2];
                const int cr = Cr.data[static_cast<size_t>(j / 2) * Cr.w + i / 2];
                uint8_t* px = dst + static_cast<size_t>(i) * pb;
                px[0] = Clamp255(static_cast<int>(y + 1.402 * (cr - 128)));
                px[1] = Clamp255(static_cast<int>(y - 0.34414 * (cb - 128) - 0.71414 * (cr - 128)));
                px[2] = Clamp255(static_cast<int>(y + 1.772 * (cb - 128)));
                if (bands == 4) {
                    int a = comp[3].data[static_cast<size_t>(j) * comp[3].w + i];
                    if (promoteAlpha_ && roles_.alpha == 3) a *= 255;
                    px[3] = Clamp255(a);
                }
            }
        }
        return Status::Ok();
    }
    thread_local std::vector<int32_t> promoted;
    for (int b = 0; b < bands; ++b) {
        const jp2::DecodedComponent& k = comp[static_cast<size_t>(b)];
        if (k.w < width || k.h < rows)
            return Status(StatusCode::kCorrupt, "JPEG 2000 block smaller than expected in " + path_);
        const bool promote = promoteAlpha_ && b == roles_.alpha;
        for (uint32_t j = 0; j < rows; ++j) {
            const int32_t* src = k.data + static_cast<size_t>(j) * k.w;
            if (promote) {
                // A 1-bit alpha of an 8-bit image as 0 / 255.
                promoted.assign(src, src + width);
                for (int32_t& v : promoted) v *= 255;
                src = promoted.data();
            }
            uint8_t* dst = out + j * rowBytes + static_cast<size_t>(b) * ss;
            switch (type_) {
                case tiff::SampleType::kU8: CopyClamped<uint8_t>(src, width, dst, pb); break;
                case tiff::SampleType::kI16: CopyClamped<int16_t>(src, width, dst, pb); break;
                case tiff::SampleType::kU16: CopyClamped<uint16_t>(src, width, dst, pb); break;
                case tiff::SampleType::kI32: CopyClamped<int32_t>(src, width, dst, pb); break;
                case tiff::SampleType::kU32: CopyClamped<uint32_t>(src, width, dst, pb); break;
                default: return Status(StatusCode::kInternal, "unexpected JPEG 2000 band type");
            }
        }
    }
    return Status::Ok();
}

Expected<uint32_t> Jp2Source::ReadBlock(uint32_t col, uint32_t row, uint8_t* out) const {
    if (col >= BlocksAcross() || row >= BlocksDown())
        return Status(StatusCode::kOutOfRange, "JPEG 2000 block out of range in " + path_);
    const uint32_t width = std::min(blockWidth_, Width() - col * blockWidth_);
    const uint32_t rows = std::min(blockHeight_, Height() - row * blockHeight_);
    std::memset(out, 0, BlockBytes());
    AcquireSlot();
    struct Release {
        const Jp2Source* s;
        ~Release() { s->ReleaseSlot(); }
    } release{this};
    if (mode_ == Mode::kSharedArea) {
        // One codestream for every area: its compressed data is read once.
        std::lock_guard<std::mutex> lock(sharedMutex_);
        if (!shared_) {
            auto cs = OpenCodestream(threadsPerDecode_);
            if (!cs.ok()) return cs.status();
            shared_ = std::move(*cs);
        }
        DDBT_TRY(Decode(*shared_, col, row));
        DDBT_TRY(Convert(*shared_, width, rows, out));
        return rows;
    }
    auto cs = OpenCodestream(threadsPerDecode_);
    if (!cs.ok()) return cs.status();
    DDBT_TRY(Decode(**cs, col, row));
    DDBT_TRY(Convert(**cs, width, rows, out));
    return rows;
}

Status Jp2Source::ReadMaskBlock(uint32_t, uint32_t, uint8_t*) const {
    return Status(StatusCode::kInvalidArgument, "no mask");
}

}  // namespace ddbt::cog::build
