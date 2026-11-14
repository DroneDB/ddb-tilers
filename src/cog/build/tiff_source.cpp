// SPDX-License-Identifier: MPL-2.0
#include "cog/build/tiff_source.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "cog/encode/block_encoder.h"
#include "core/crs/crs.h"
#include "core/io/byte_source.h"

namespace ddbt::cog::build {

Expected<std::unique_ptr<TiffSource>> TiffSource::Open(const std::string& path) {
    // pread, not a mapping: a build reads the whole file once, and mapped pages would count in
    // the process's resident memory (GDAL reads with pread too).
    auto bytes = io::OpenByteSource(path, io::SourceKind::kPread);
    if (!bytes.ok()) return bytes.status();
    auto reader = tiff::TiffReader::Open(std::shared_ptr<const io::ByteSource>(std::move(*bytes)));
    if (!reader.ok()) return reader.status();
    std::unique_ptr<TiffSource> s(new TiffSource());
    s->path_ = path;
    s->reader_ = std::move(*reader);
    const tiff::IfdInfo& m = s->Main();
    DDBT_TRY(s->reader_->CheckDecodable(m.index));
    if (tiff::SampleTypeSize(s->Type()) == 0 || (m.sampleType == tiff::SampleType::kBit1 && m.samplesPerPixel != 1))
        return Status(StatusCode::kUnsupported, "Unsupported sample type in " + path);
    auto geo = tiff::ReadGeoreferencing(*s->reader_, path);
    if (!geo.ok()) return geo.status();
    s->georef_ = std::move(*geo);
    s->maskIfd_ = s->reader_->MaskOf(m.index);
    // GDAL: an associated or unassociated alpha extra sample makes that band GCI_AlphaBand.
    const size_t nExtra = m.extraSamples.size();
    for (size_t i = 0; i < nExtra; ++i) {
        const uint16_t v = m.extraSamples[i];
        if (v == tiff::extrasample::kAssociatedAlpha || v == tiff::extrasample::kUnassociatedAlpha) {
            const int band = static_cast<int>(m.samplesPerPixel - nExtra + i);
            if (band >= 0) s->alphaBand_ = band;
        }
    }
    return s;
}

std::string TiffSource::ColorInterp(int b) const {
    const tiff::IfdInfo& m = Main();
    if (b == alphaBand_) return "Alpha";
    // A colour table (the ColorMap, GDAL's black and white table of a 1-bit image, its inverted
    // ramp of a min-is-white one) makes the band a palette
    if (b == 0 && ColorTable()) return "Palette";
    const int nExtra = static_cast<int>(m.extraSamples.size());
    const int colour = m.samplesPerPixel - nExtra;
    switch (m.photometric) {
        case tiff::photometric::kRgb:
        case tiff::photometric::kYCbCr:
            if (b == 0) return "Red";
            if (b == 1) return "Green";
            if (b == 2) return "Blue";
            return "Undefined";
        case tiff::photometric::kPalette: return b == 0 ? "Palette" : "Undefined";
        default:
            if (b == 0 && colour >= 1) return "Gray";
            return "Undefined";
    }
}

std::optional<std::vector<std::array<uint8_t, 3>>> TiffSource::ColorTable() const {
    const tiff::IfdInfo& m = Main();
    if (m.samplesPerPixel != 1 || m.bitsPerSample > 16) return std::nullopt;
    std::vector<std::array<uint8_t, 3>> table;
    if (!m.colorMap.empty()) {
        // TIFFColorMapTagToColorTable: values in [0, 255] are 8-bit already, multiples of 256 (and
        // not all of 257) are divided by 256, the others by 257
        const size_t n = m.colorMap.size() / 3;
        uint16_t maxColor = 0;
        bool any = false, all256 = true;
        for (const uint16_t v : m.colorMap) {
            maxColor = std::max(maxColor, v);
            any = any || v > 0;
            all256 = all256 && v % 256 == 0;
        }
        const unsigned divisor = maxColor > 0 && maxColor < 256 ? 1u : (any && all256 ? 256u : 257u);
        table.resize(n);
        for (size_t i = 0; i < n; ++i)
            for (size_t c = 0; c < 3; ++c) table[i][c] = static_cast<uint8_t>(m.colorMap[c * n + i] / divisor);
        return table;
    }
    if (m.sampleType == tiff::SampleType::kBit1) {
        // GTiffBitmapBand
        const bool white0 = m.photometric == tiff::photometric::kMinIsWhite;
        table = {white0 ? std::array<uint8_t, 3>{255, 255, 255} : std::array<uint8_t, 3>{0, 0, 0},
                 white0 ? std::array<uint8_t, 3>{0, 0, 0} : std::array<uint8_t, 3>{255, 255, 255}};
        return table;
    }
    if (m.photometric == tiff::photometric::kMinIsWhite) {
        const unsigned n = 1u << m.bitsPerSample;
        table.resize(n);
        for (unsigned i = 0; i < n; ++i) {
            const auto v = static_cast<uint8_t>((255u * (n - 1 - i)) / (n - 1));
            table[i] = {v, v, v};
        }
        return table;
    }
    return std::nullopt;
}

std::optional<int> TiffSource::Nbits(int b) const {
    const int bits = Main().bitsPerSample;
    if (b < 0 || b >= Bands() || bits == 8 || bits == 16 || bits == 32 || bits == 64) return std::nullopt;
    return bits;
}

bool TiffSource::IsOptimizedCog() const {
    // GDAL reports no CRS for a raster georeferenced by GCPs (only a GCP CRS): it is warped.
    if (!georef_.crs || !georef_.geoTransform) return false;
    try {
        static const crs::Crs webMercator = crs::Crs::FromEpsg(3857);
        if (!georef_.crs->Equivalent(webMercator)) return false;
    } catch (...) {
        return false;
    }
    const tiff::IfdInfo& m = Main();
    if (!m.tiled || m.blockWidth != m.blockHeight || (m.blockWidth != 256 && m.blockWidth != 512)) return false;
    return !reader_->Overviews().empty();
}

std::optional<int> TiffSource::JpegQuality() const {
    const tiff::IfdInfo& m = Main();
    if (m.compression != 7) return std::nullopt;
    const int components = m.samplesPerPixel >= 3 ? 3 : 1;
    const int q = encode::GuessJpegQuality(m.jpegTables, components);
    if (q <= 0) return std::nullopt;
    return q;
}

bool TiffSource::HasRpc() const noexcept {
    constexpr uint16_t kRpcCoefficientTag = 50844;  // GDAL's RPC metadata
    const tiff::TagEntry* rpc = Main().FindTag(kRpcCoefficientTag);
    return rpc != nullptr && rpc->count == 92;
}

std::vector<writer::RawTag> TiffSource::MetadataTags() const {
    std::vector<writer::RawTag> out;
    const tiff::IfdInfo& m = Main();
    auto ascii = [&](uint16_t tag) {
        const tiff::TagEntry* e = m.FindTag(tag);
        if (e == nullptr) return;
        auto v = reader_->ReadTagString(*e, 1 << 20);
        if (v.ok()) out.push_back(AsciiTag(tag, *v));
    };
    auto rational = [&](uint16_t tag) {
        const tiff::TagEntry* e = m.FindTag(tag);
        if (e == nullptr) return;
        auto v = reader_->ReadTagDoubles(*e, 1);
        if (v.ok() && !v->empty()) out.push_back(RationalTag(tag, GdalFloatTag((*v)[0])));
    };
    auto shortTag = [&](uint16_t tag) {
        const tiff::TagEntry* e = m.FindTag(tag);
        if (e == nullptr) return;
        auto v = reader_->ReadTagUInts(*e, 1);
        if (v.ok() && !v->empty()) out.push_back(ShortTag(tag, static_cast<uint16_t>((*v)[0])));
    };
    ascii(269);
    ascii(270);
    rational(282);
    rational(283);
    shortTag(280);
    shortTag(281);
    shortTag(296);
    ascii(305);
    ascii(306);
    ascii(315);
    ascii(316);
    ascii(33432);
    return out;
}

Expected<uint32_t> TiffSource::ReadBlock(uint32_t col, uint32_t row, uint8_t* out) const {
    const tiff::IfdInfo& m = Main();
    const uint32_t ifd = m.index;
    if (m.sampleType == tiff::SampleType::kBit1) {
        // A 1-bit image: bytes 0 / 1
        const uint32_t index = m.BlockIndex(col, row, 0);
        auto b = reader_->ReadBlock(ifd, index);
        if (!b.ok()) return b.status();
        tiff::ExpandBits(b->data(), b->rowStride, m.blockWidth, b->height, 1, out, m.blockWidth);
        return b->height;
    }
    if (m.IsPixelInterleaved() || m.samplesPerPixel == 1) {
        const uint32_t index = m.BlockIndex(col, row, 0);
        const uint64_t size = reader_->DecodedSize(ifd, index);
        auto r = reader_->ReadBlockInto(ifd, index, {out, static_cast<size_t>(size)});
        if (!r.ok()) return r.status();
        return m.BlockRows(index);
    }
    // BAND interleave: decode each plane (a band: one value, or the two of a complex sample) and
    // interleave.
    const size_t vs = SampleSize() * m.Components();  // bytes of a band's sample
    const size_t planes = static_cast<size_t>(m.samplesPerPixel);
    thread_local std::vector<uint8_t> plane;
    uint32_t rows = 0;
    for (size_t b = 0; b < planes; ++b) {
        const uint32_t index = m.BlockIndex(col, row, static_cast<uint32_t>(b));
        const uint64_t size = reader_->DecodedSize(ifd, index);
        plane.resize(static_cast<size_t>(size));
        auto r = reader_->ReadBlockInto(ifd, index, {plane.data(), plane.size()});
        if (!r.ok()) return r.status();
        rows = m.BlockRows(index);
        const size_t n = static_cast<size_t>(m.blockWidth) * rows;
        for (size_t i = 0; i < n; ++i) std::memcpy(out + (i * planes + b) * vs, plane.data() + i * vs, vs);
    }
    return rows;
}

Status TiffSource::ReadMaskBlock(uint32_t col, uint32_t row, uint8_t* out) const {
    if (!maskIfd_) return Status(StatusCode::kInvalidArgument, "no mask");
    const tiff::IfdInfo& m = Main();
    const tiff::IfdInfo& k = reader_->Ifd(*maskIfd_);
    if (k.blockWidth != m.blockWidth || k.blockHeight != m.blockHeight || k.sampleType != tiff::SampleType::kBit1)
        return Status(StatusCode::kUnsupported, "Mask blocks differ from the image blocks in " + path_);
    const uint32_t index = k.BlockIndex(col, row, 0);
    auto b = reader_->ReadBlock(*maskIfd_, index);
    if (!b.ok()) return b.status();
    tiff::ExpandBits(b->data(), b->rowStride, k.blockWidth, b->height, 255, out, k.blockWidth);
    return Status::Ok();
}

}  // namespace ddbt::cog::build
