// SPDX-License-Identifier: MPL-2.0
//
// The rules are those of GDAL 3.13.3's PNG driver (PNGDataset::Open, LoadScanline,
// CollectMetadata): see png_source.h.
#include "cog/build/png_source.h"

#include <spng.h>

#include <cstring>
#include <string>
#include <vector>

namespace ddbt::cog::build {

namespace {

// A libspng context over a buffer, freed with the object.
class Context {
public:
    Context() : ctx_(spng_ctx_new(0)) {}
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;
    ~Context() {
        if (ctx_ != nullptr) spng_ctx_free(ctx_);
    }
    spng_ctx* get() const noexcept { return ctx_; }

private:
    spng_ctx* ctx_;
};

constexpr uint8_t kGray = 0, kRgb = 2, kPalette = 3, kGrayAlpha = 4, kRgba = 6;

Status SpngError(int err, const std::string& path) {
    return Status(StatusCode::kCorrupt, std::string("PNG: ") + spng_strerror(err) + ": " + path);
}

// A context set up as libpng reads (critical chunks with a bad CRC fail, ancillary ones are
// skipped), over `bytes`.
Status Start(const Context& c, const std::vector<uint8_t>& bytes, const std::string& path) {
    if (c.get() == nullptr) return Status(StatusCode::kResourceExhausted, "PNG: cannot create a decoder");
    if (const int e = spng_set_crc_action(c.get(), SPNG_CRC_ERROR, SPNG_CRC_DISCARD)) return SpngError(e, path);
    if (const int e = spng_set_png_buffer(c.get(), bytes.data(), bytes.size())) return SpngError(e, path);
    return Status::Ok();
}

}  // namespace

bool IsPngFile(const io::ByteSource& bytes) {
    static constexpr uint8_t kSignature[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    uint8_t head[8];
    if (bytes.Size() < sizeof(head) || !bytes.Read(0, head).ok()) return false;
    return std::memcmp(head, kSignature, sizeof(head)) == 0;
}

Expected<std::unique_ptr<PngSource>> PngSource::Open(const std::string& path, uint64_t memoryBudget) {
    auto src = io::OpenByteSource(path, io::SourceKind::kPread);
    if (!src.ok()) return src.status();
    const uint64_t size = (*src)->Size();
    if (size > (uint64_t{1} << 40)) return Status(StatusCode::kUnsupported, "PNG file too large: " + path);
    auto bytes = (*src)->ReadVector(0, static_cast<size_t>(size));
    if (!bytes.ok()) return bytes.status();
    std::unique_ptr<PngSource> s(new PngSource(path, memoryBudget));
    s->bytes_ = std::move(*bytes);

    Context c;
    DDBT_TRY(Start(c, s->bytes_, path));
    spng_ihdr ihdr{};
    if (const int e = spng_get_ihdr(c.get(), &ihdr)) return SpngError(e, path);
    s->bitDepth_ = ihdr.bit_depth;
    s->interlaced_ = ihdr.interlace_method != SPNG_INTERLACE_NONE;

    ImageInfo info;
    info.width = ihdr.width;
    info.height = ihdr.height;
    info.type = ihdr.bit_depth == 16 ? tiff::SampleType::kU16 : tiff::SampleType::kU8;
    if (ihdr.bit_depth < 8) info.nbits = ihdr.bit_depth;
    spng_trns trns{};
    const bool hasTrns = spng_get_trns(c.get(), &trns) == 0;
    switch (ihdr.color_type) {
        case kGray:
            info.bands = 1;
            info.interps = {"Gray"};
            if (hasTrns) info.noData = trns.gray;
            break;
        case kGrayAlpha:
            info.bands = 2;
            info.interps = {"Gray", "Alpha"};
            info.alphaBand = 1;
            break;
        case kRgb:
            info.bands = 3;
            info.interps = {"Red", "Green", "Blue"};
            if (hasTrns) {
                // GDAL sets each band's nodata to its value of the tRNS colour.
                if (trns.red != trns.green || trns.red != trns.blue)
                    return Status(StatusCode::kUnsupported,
                                  "PNG: an RGB transparency colour of different red, green and blue values (one "
                                  "nodata value per band) is not supported: " + path);
                info.noData = trns.red;
            }
            break;
        case kRgba:
            info.bands = 4;
            info.interps = {"Red", "Green", "Blue", "Alpha"};
            info.alphaBand = 3;
            break;
        case kPalette: {
            info.bands = 1;
            info.interps = {"Palette"};
            spng_plte plte{};
            if (spng_get_plte(c.get(), &plte) != 0) plte.n_entries = 0;
            std::vector<std::array<uint8_t, 3>> table(plte.n_entries);
            int nodataIndex = -1;
            for (uint32_t i = 0; i < plte.n_entries; ++i) {
                table[i] = {plte.entries[i].red, plte.entries[i].green, plte.entries[i].blue};
            }
            // The index of the only fully transparent entry (GDAL scans from the last one).
            if (hasTrns) {
                for (int i = static_cast<int>(plte.n_entries) - 1; i >= 0; --i) {
                    if (static_cast<uint32_t>(i) < trns.n_type3_entries && trns.type3_alpha[i] == 0)
                        nodataIndex = nodataIndex == -1 ? i : -2;
                }
            }
            if (nodataIndex > -1) info.noData = nodataIndex;
            info.colorTable = std::move(table);
            break;
        }
        default: return Status(StatusCode::kCorrupt, "PNG: invalid colour type: " + path);
    }
    if (info.noData) info.noDataText = std::to_string(static_cast<long long>(*info.noData));
    DDBT_TRY(s->SetInfo(std::move(info)));
    return s;
}

Status PngSource::Decode(RowSink& sink) const {
    const ImageInfo& info = Info();
    Context c;
    DDBT_TRY(Start(c, bytes_, Path()));
    // The samples as stored, host order (libpng's png_set_swap of GDAL on little-endian hosts).
    const int fmt = SPNG_FMT_PNG;
    size_t imageBytes = 0;
    if (const int e = spng_decoded_image_size(c.get(), fmt, &imageBytes)) return SpngError(e, Path());
    const size_t packedRow = imageBytes / info.height;
    std::vector<uint8_t> unpacked(RowBytes());
    // Samples of 1, 2 or 4 bits are unpacked (png_set_packing: one byte per sample, unscaled).
    auto emit = [&](uint32_t y, const uint8_t* packed) -> Status {
        if (bitDepth_ >= 8) return sink.Row(y, packed);
        const size_t n = static_cast<size_t>(info.width) * static_cast<size_t>(info.bands);
        const unsigned mask = (1u << bitDepth_) - 1u;
        for (size_t i = 0; i < n; ++i) {
            const size_t bit = i * static_cast<size_t>(bitDepth_);
            const unsigned shift = 8u - static_cast<unsigned>(bitDepth_) - static_cast<unsigned>(bit & 7);
            unpacked[i] = static_cast<uint8_t>((packed[bit >> 3] >> shift) & mask);
        }
        return sink.Row(y, unpacked.data());
    };
    if (interlaced_) {
        // Adam7: the whole image, then its rows.
        std::vector<uint8_t> image(imageBytes);
        if (const int e = spng_decode_image(c.get(), image.data(), image.size(), fmt, 0)) return SpngError(e, Path());
        for (uint32_t y = 0; y < info.height; ++y) DDBT_TRY(emit(y, image.data() + static_cast<size_t>(y) * packedRow));
        return Status::Ok();
    }
    if (const int e = spng_decode_image(c.get(), nullptr, 0, fmt, SPNG_DECODE_PROGRESSIVE)) return SpngError(e, Path());
    std::vector<uint8_t> row(packedRow);
    for (uint32_t y = 0; y < info.height; ++y) {
        const int e = spng_decode_row(c.get(), row.data(), row.size());
        if (e != 0 && e != SPNG_EOI) return SpngError(e, Path());
        DDBT_TRY(emit(y, row.data()));
        if (e == SPNG_EOI && y + 1 < info.height) return Status(StatusCode::kCorrupt, "PNG: truncated image: " + Path());
    }
    return Status::Ok();
}

}  // namespace ddbt::cog::build
