// SPDX-License-Identifier: MPL-2.0
//
// TiffReader block reads: raw bytes (ghost header leader / trailer checks), decoding through
// the codecs, byte order, predictors, sparse fills.

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "cog/codecs/codecs.h"
#include "cog/codecs/jpeg.h"
#include "cog/codecs/predictor.h"
#include "cog/tiff/tiff_reader.h"
#include "core/util/checked_math.h"
#include "core/util/endian.h"

namespace ddbt::cog::tiff {

namespace {

using codecs::Compression;

thread_local std::vector<uint8_t> tlsCompressed;  // pread staging
thread_local std::vector<uint8_t> tlsReversed;    // FillOrder=2 bit reversal
thread_local std::vector<uint8_t> tlsPacked;      // packed (NBITS) samples before unpacking

uint8_t ReverseBits(uint8_t b) {
    b = static_cast<uint8_t>((b & 0xF0) >> 4 | (b & 0x0F) << 4);
    b = static_cast<uint8_t>((b & 0xCC) >> 2 | (b & 0x33) << 2);
    b = static_cast<uint8_t>((b & 0xAA) >> 1 | (b & 0x55) << 1);
    return b;
}

template <typename T>
T ConvertNoData(double v) {
    if constexpr (std::is_floating_point_v<T>) {
        return static_cast<T>(v);
    } else {
        // GDALCopyWords: round to nearest, saturate; NaN -> 0.
        if (std::isnan(v)) return T{0};
        const double r = std::round(v);
        if (r <= static_cast<double>(std::numeric_limits<T>::lowest())) return std::numeric_limits<T>::lowest();
        if (r >= static_cast<double>(std::numeric_limits<T>::max())) return std::numeric_limits<T>::max();
        return static_cast<T>(r);
    }
}

template <typename T>
void FillTyped(std::span<uint8_t> out, T value) {
    const size_t n = out.size() / sizeof(T);
    uint8_t pattern[sizeof(T)];
    std::memcpy(pattern, &value, sizeof(T));
    bool allSame = true;
    for (size_t i = 1; i < sizeof(T); ++i) allSame &= pattern[i] == pattern[0];
    if (allSame) {
        std::memset(out.data(), pattern[0], out.size());
        return;
    }
    for (size_t i = 0; i < n; ++i) std::memcpy(out.data() + i * sizeof(T), pattern, sizeof(T));
}

uint16_t FloatToHalfBits(float f) {
    // Round to nearest even (as GDAL's CPLFloatToHalf for nodata).
    uint32_t x;
    std::memcpy(&x, &f, 4);
    const uint32_t sign = (x >> 16) & 0x8000u;
    const uint32_t exp = (x >> 23) & 0xFFu;
    uint32_t mant = x & 0x7FFFFFu;
    if (exp == 0xFF) return static_cast<uint16_t>(sign | 0x7C00u | (mant ? 0x200u : 0));
    int e = static_cast<int>(exp) - 127 + 15;
    if (e >= 31) return static_cast<uint16_t>(sign | 0x7C00u);
    if (e <= 0) {
        if (e < -10) return static_cast<uint16_t>(sign);
        mant |= 0x800000u;
        const uint32_t shift = static_cast<uint32_t>(14 - e);
        uint32_t h = mant >> shift;
        const uint32_t rem = mant & ((1u << shift) - 1);
        const uint32_t half = 1u << (shift - 1);
        if (rem > half || (rem == half && (h & 1))) ++h;
        return static_cast<uint16_t>(sign | h);
    }
    uint32_t h = (static_cast<uint32_t>(e) << 10) | (mant >> 13);
    const uint32_t rem = mant & 0x1FFFu;
    if (rem > 0x1000u || (rem == 0x1000u && (h & 1))) ++h;
    return static_cast<uint16_t>(sign | h);
}

}  // namespace

void FillSamples(std::span<uint8_t> out, SampleType type, std::optional<double> value) {
    if (!value.has_value() || type == SampleType::kBit1 || type == SampleType::kUnknown) {
        std::memset(out.data(), 0, out.size());
        return;
    }
    const double v = *value;
    switch (type) {
        case SampleType::kU8: FillTyped(out, ConvertNoData<uint8_t>(v)); break;
        case SampleType::kI8: FillTyped(out, ConvertNoData<int8_t>(v)); break;
        case SampleType::kU16: FillTyped(out, ConvertNoData<uint16_t>(v)); break;
        case SampleType::kI16: FillTyped(out, ConvertNoData<int16_t>(v)); break;
        case SampleType::kU32: FillTyped(out, ConvertNoData<uint32_t>(v)); break;
        case SampleType::kI32: FillTyped(out, ConvertNoData<int32_t>(v)); break;
        case SampleType::kU64: FillTyped(out, ConvertNoData<uint64_t>(v)); break;
        case SampleType::kI64: FillTyped(out, ConvertNoData<int64_t>(v)); break;
        case SampleType::kF16: FillTyped(out, FloatToHalfBits(static_cast<float>(v))); break;
        case SampleType::kF32: FillTyped(out, static_cast<float>(v)); break;
        case SampleType::kF64: FillTyped(out, v); break;
        default: std::memset(out.data(), 0, out.size()); break;
    }
}

namespace {

// Byte b of the packed bits -> 8 bytes 0x00 / 0xFF, MSB first (the first pixel in the low byte of
// the little-endian word, as memcpy stores it).
constexpr std::array<uint64_t, 256> MakeExpandTable() {
    std::array<uint64_t, 256> t{};
    for (unsigned b = 0; b < 256; ++b) {
        uint64_t v = 0;
        for (unsigned i = 0; i < 8; ++i)
            if ((b >> (7 - i)) & 1u) v |= uint64_t{0xFF} << (8 * i);
        t[b] = v;
    }
    return t;
}
constexpr std::array<uint64_t, 256> kExpandTable = MakeExpandTable();
static_assert(std::endian::native == std::endian::little, "kExpandTable assumes a little-endian host");

}  // namespace

void ExpandBits(const uint8_t* in, size_t inRowStride, uint32_t width, uint32_t rows, uint8_t one, uint8_t* out,
                size_t outRowStride) {
    // Eight pixels per table lookup (a 512 x 512 mask block was 2.6 ms bit by bit).
    const uint64_t oneMask = uint64_t{one} * 0x0101010101010101ULL;
    const uint32_t fullBytes = width / 8;
    for (uint32_t r = 0; r < rows; ++r) {
        const uint8_t* src = in + r * inRowStride;
        uint8_t* dst = out + r * outRowStride;
        for (uint32_t b = 0; b < fullBytes; ++b) {
            const uint64_t v = kExpandTable[src[b]] & oneMask;
            std::memcpy(dst + 8 * static_cast<size_t>(b), &v, 8);
        }
        for (uint32_t x = fullBytes * 8; x < width; ++x) dst[x] = (src[x >> 3] >> (7 - (x & 7))) & 1 ? one : 0;
    }
}

BitsSummary SummarizeBits(const uint8_t* in, size_t inRowStride, uint32_t width, uint32_t rows) {
    const uint32_t fullBytes = width / 8;
    const uint32_t rem = width % 8;
    const uint8_t remMask = rem != 0 ? static_cast<uint8_t>(0xFFu << (8 - rem)) : uint8_t{0};
    bool any0 = false, any1 = false;
    for (uint32_t r = 0; r < rows && !(any0 && any1); ++r) {
        const uint8_t* src = in + r * inRowStride;
        uint8_t orAcc = 0, andAcc = 0xFF;
        for (uint32_t b = 0; b < fullBytes; ++b) {
            orAcc = static_cast<uint8_t>(orAcc | src[b]);
            andAcc = static_cast<uint8_t>(andAcc & src[b]);
        }
        any1 = any1 || orAcc != 0;
        any0 = any0 || andAcc != 0xFF;
        if (rem != 0) {
            const uint8_t v = static_cast<uint8_t>(src[fullBytes] & remMask);
            any1 = any1 || v != 0;
            any0 = any0 || v != remMask;
        }
    }
    return {any0, any1};
}

Status TiffReader::CheckDecodable(uint32_t ifd) const {
    if (ifd >= ifds_.size()) return OutOfRange("IFD " + std::to_string(ifd) + " does not exist");
    const IfdInfo& i = ifds_[ifd];
    if (i.sampleType == SampleType::kUnknown) {
        return Unsupported(std::to_string(i.bitsPerSample) + "-bit samples of SampleFormat " + std::to_string(i.sampleFormat));
    }
    if (!codecs::IsSupportedCompression(i.compression) && !LibtiffCodec::Supports(i.compression)) {
        return Unsupported(std::string("compression ") + std::string(codecs::CompressionName(i.compression)) + " (" +
                           std::to_string(i.compression) + ")");
    }
    const auto c = static_cast<Compression>(i.compression);
    if (i.IsPacked() && i.predictor != 1) return Unsupported("predictor on " + std::to_string(i.bitsPerSample) + "-bit data");
    if (i.sampleType == SampleType::kBit1) {
        if (c == Compression::kJpeg || c == Compression::kWebp || c == Compression::kLerc) {
            return Unsupported("1-bit data with " + std::string(codecs::CompressionName(i.compression)));
        }
        if (i.predictor != 1) return Unsupported("predictor on 1-bit data");
    }
    if (i.predictor != 1 && i.predictor != 2 && i.predictor != 3) return Unsupported("predictor " + std::to_string(i.predictor));
    if (i.predictor == 3 && i.sampleFormat != 3) return Unsupported("floating-point predictor on non-float data");
    if (i.predictor == 3 && i.bitsPerSample != 16 && i.bitsPerSample != 32 && i.bitsPerSample != 64) {
        return Unsupported("floating-point predictor on " + std::to_string(i.bitsPerSample) + "-bit data");
    }
    if (c == Compression::kJpeg) {
        // 12-bit JPEG goes to libtiff (built with libjpeg-turbo's 12-bit API)
        if (i.bitsPerSample != 8 && i.bitsPerSample != 12)
            return Unsupported("JPEG with " + std::to_string(i.bitsPerSample) + "-bit samples");
        if (i.photometric == photometric::kYCbCr && (!i.IsPixelInterleaved() || i.samplesPerPixel != 3)) {
            return Unsupported("JPEG YCbCr needs 3 pixel-interleaved samples");
        }
    } else if (i.photometric == photometric::kYCbCr && i.samplesPerPixel >= 3) {
        return Unsupported("YCbCr data without JPEG compression");
    }
    if (c == Compression::kWebp && i.IsPacked()) return Unsupported("WebP with packed samples");
    if (c == Compression::kWebp) {
        if (i.bitsPerSample != 8 || !i.IsPixelInterleaved() || (i.samplesPerPixel != 3 && i.samplesPerPixel != 4)) {
            return Unsupported("WebP needs 3 or 4 pixel-interleaved 8-bit samples");
        }
    }
    if (c == Compression::kLerc && i.hasLercParameters && i.lercParameters[1] > 2) {
        return Unsupported("LERC additional compression " + std::to_string(i.lercParameters[1]));
    }
    return Status::Ok();
}

uint64_t TiffReader::DecodedSize(uint32_t ifd, uint32_t index) const noexcept {
    if (ifd >= ifds_.size() || index >= ifds_[ifd].blockCount) return 0;
    return ifds_[ifd].DecodedBlockBytes(index);
}

void TiffReader::FillSparse(uint32_t ifd, std::span<uint8_t> out) const {
    const IfdInfo& i = ifds_.at(ifd);
    FillSamples(out, i.sampleType, i.noData);
}

Expected<std::span<const uint8_t>> TiffReader::ReadRaw(uint32_t ifd, uint32_t index, const BlockLocation& loc,
                                                     std::vector<uint8_t>& storage) const {
    const uint64_t fileSize = source_->Size();
    if (loc.size > options_.maxCompressedBlockBytes) {
        return ResourceExhausted("block " + std::to_string(index) + " of IFD " + std::to_string(ifd) + ": " +
                                 std::to_string(loc.size) + " compressed bytes");
    }
    if (!RangeInside(loc.offset, loc.size, fileSize)) {
        return Corrupt("block " + std::to_string(index) + " of IFD " + std::to_string(ifd) + " lies past the end of the file");
    }
    // GDAL ghost header: the 4 bytes before the data hold its size (little-endian), the 4 after
    // repeat its last 4 bytes. Read them with the data and check.
    const bool leader = ghost_.blockLeaderSizeAsUint4 && options_.validateGhostLeader &&
                        !ghost_.knownIncompatibleEdition && loc.offset >= 4;
    const bool trailer = leader && ghost_.blockTrailerLast4BytesRepeated && loc.size >= 4 &&
                         RangeInside(loc.offset + loc.size, 4, fileSize);
    const uint64_t start = leader ? loc.offset - 4 : loc.offset;
    const uint64_t len = loc.size + (leader ? 4 : 0) + (trailer ? 4 : 0);
    const uint8_t* p = nullptr;
    if (const uint8_t* base = source_->Data()) {
        p = base + start;
    } else {
        storage.resize(static_cast<size_t>(len));
        DDBT_TRY(source_->Read(start, std::span<uint8_t>(storage.data(), static_cast<size_t>(len))));
        p = storage.data();
    }
    if (leader) {
        const uint32_t stated = LoadLE<uint32_t>(p);
        if (stated != loc.size) {
            return Corrupt("block " + std::to_string(index) + " of IFD " + std::to_string(ifd) + ": ghost header leader says " +
                           std::to_string(stated) + " bytes, ByteCounts says " + std::to_string(loc.size));
        }
        p += 4;
        if (trailer && std::memcmp(p + loc.size - 4, p + loc.size, 4) != 0) {
            return Corrupt("block " + std::to_string(index) + " of IFD " + std::to_string(ifd) + ": ghost header trailer mismatch");
        }
    }
    return std::span<const uint8_t>(p, static_cast<size_t>(loc.size));
}

Expected<RawBlock> TiffReader::ReadBlockRaw(uint32_t ifd, uint32_t index) const {
    auto loc = Locate(ifd, index);
    if (!loc.ok()) return std::move(loc).TakeStatus();
    RawBlock raw;
    raw.location = *loc;
    if (loc->IsSparse()) return raw;
    auto bytes = ReadRaw(ifd, index, *loc, raw.owned);
    if (!bytes.ok()) return std::move(bytes).TakeStatus();
    raw.bytes = *bytes;
    return raw;
}

Status TiffReader::DecodeBlock(uint32_t ifd, uint32_t index, std::span<const uint8_t> compressed,
                               std::span<uint8_t> out) const {
    DDBT_TRY(CheckDecodable(ifd));
    const IfdInfo& i = ifds_[ifd];
    if (index >= i.blockCount) return OutOfRange("block " + std::to_string(index));
    const uint64_t expected = i.DecodedBlockBytes(index);
    if (expected == 0) return OverflowError("block size overflow");
    if (expected > options_.maxBlockBytes) {
        return ResourceExhausted("block of " + std::to_string(expected) + " bytes (limit " +
                                 std::to_string(options_.maxBlockBytes) + ")");
    }
    if (out.size() != expected) {
        return InvalidArgument("output buffer of " + std::to_string(out.size()) + " bytes, block needs " + std::to_string(expected));
    }
    const size_t bps = SampleTypeSize(i.sampleType);
    if (bps > 1 && reinterpret_cast<uintptr_t>(out.data()) % bps != 0) return InvalidArgument("output buffer misaligned");

    const uint32_t rows = i.BlockRows(index);
    const uint32_t spp = i.SamplesPerBlockPixel();
    const auto c = static_cast<Compression>(i.compression);

    // Packed samples are decoded as stored, then unpacked into `out`
    std::span<uint8_t> stored = out;
    if (i.IsPacked()) {
        std::vector<uint8_t>& packed = tlsPacked;
        packed.resize(static_cast<size_t>(i.StoredRowBytes() * rows));
        stored = std::span<uint8_t>(packed);
    }

    // The compressions ddbt decodes itself, and the others (libtiff_codec.h): old-style LZW
    // (its first code is a byte-swapped 9-bit Clear, libtiff's test) and 12-bit JPEG go to libtiff
    const bool oldLzw = c == Compression::kLzw && compressed.size() >= 2 && compressed[0] == 0 && (compressed[1] & 1);
    const bool viaLibtiff = !codecs::IsSupportedCompression(i.compression) || oldLzw ||
                            (c == Compression::kJpeg && i.bitsPerSample != 8);
    Status st;
    bool nativeOrder = false;  // decoder output is already in host order
    if (viaLibtiff) {
        // libtiff restores FillOrder 2, swaps to host order and undoes the predictor itself
        st = Libtiff().Decode(i.offset, index, compressed, stored);
        nativeOrder = true;
    } else {
        if (i.fillOrder == 2) {
            // libtiff reverses the bits of the raw data before decoding.
            std::vector<uint8_t>& rev = tlsReversed;
            rev.resize(compressed.size());
            for (size_t k = 0; k < compressed.size(); ++k) rev[k] = ReverseBits(compressed[k]);
            compressed = std::span<const uint8_t>(rev);
        }
        switch (c) {
            case Compression::kNone: st = codecs::DecodeNone(compressed, stored); break;
            case Compression::kLzw: st = codecs::DecodeLzw(compressed, stored); break;
            case Compression::kPackBits: st = codecs::DecodePackBits(compressed, stored); break;
            case Compression::kAdobeDeflate:
            case Compression::kDeflate: st = codecs::DecodeDeflate(compressed, stored); break;
            case Compression::kZstd: st = codecs::DecodeZstd(compressed, stored); break;
            case Compression::kLerc: {
                codecs::LercBlockInfo li;
                li.width = i.blockWidth;
                li.height = rows;
                li.samples = spp;
                li.bitsPerSample = i.bitsPerSample;
                li.sampleFormat = i.sampleFormat;
                li.lastSampleIsUnassociatedAlpha = i.IsPixelInterleaved() && !i.extraSamples.empty() &&
                                                   i.extraSamples.back() == extrasample::kUnassociatedAlpha;
                li.additionalCompression = i.hasLercParameters ? i.lercParameters[1] : 0;
                st = codecs::DecodeLerc(compressed, li, stored);
                break;
            }
            case Compression::kWebp:
                st = codecs::DecodeWebp(compressed, i.blockWidth, rows, i.samplesPerPixel, stored);
                nativeOrder = true;
                break;
            case Compression::kJpeg: {
                codecs::JpegDecodeOptions jo;
                jo.tables = i.jpegTables;
                jo.ycbcrToRgb = i.photometric == photometric::kYCbCr && i.IsPixelInterleaved() && i.samplesPerPixel == 3;
                st = codecs::DecodeJpeg(compressed, jo, spp, 0, 0, i.blockWidth, rows, stored.data(),
                                        static_cast<size_t>(i.BlockRowBytes()));
                nativeOrder = true;
                break;
            }
            default: st = Unsupported("compression " + std::to_string(i.compression)); break;
        }
    }
    if (!st.ok()) return st.WithContext("block " + std::to_string(index) + " of IFD " + std::to_string(ifd));
    if (i.IsPacked()) {
        // Bits of a packed stream are in file order whatever the byte order (libtiff swaps only
        // whole 16, 32 and 64-bit samples), and no predictor applies to them
        UnpackSamples(stored.data(), static_cast<size_t>(i.StoredRowBytes()), uint64_t{i.blockWidth} * spp, rows,
                      i.bitsPerSample, bps, out.data(), static_cast<size_t>(i.BlockRowBytes()));
        return Status::Ok();
    }
    if (nativeOrder || bps == 0) return Status::Ok();

    const size_t rowStride = static_cast<size_t>(i.BlockRowBytes());
    if (i.predictor == 3) {
        // fpAcc produces host-order samples: no byte swap.
        codecs::UndoFloatingPointPredictor(out.data(), rows, rowStride, i.blockWidth, spp, bps);
        return Status::Ok();
    }
    if (bigEndian_ && bps > 1) codecs::ByteSwapSamples(out.data(), out.size() / bps, bps);
    if (i.predictor == 2) codecs::UndoHorizontalPredictor(out.data(), rows, rowStride, i.blockWidth, spp, bps);
    return Status::Ok();
}

const LibtiffCodec& TiffReader::Libtiff() const {
    std::call_once(libtiffOnce_, [this] { libtiff_ = std::make_unique<LibtiffCodec>(source_); });
    return *libtiff_;
}

void UnpackSamples(const uint8_t* in, size_t inRowStride, uint64_t samples, uint32_t rows, uint32_t bits,
                   size_t outBytes, uint8_t* out, size_t outRowStride) {
    for (uint32_t r = 0; r < rows; ++r) {
        const uint8_t* src = in + static_cast<size_t>(r) * inRowStride;
        uint8_t* dst = out + static_cast<size_t>(r) * outRowStride;
        uint64_t bit = 0;
        for (uint64_t s = 0; s < samples; ++s) {
            uint32_t v = 0;
            for (uint32_t k = 0; k < bits; ++k, ++bit) v = (v << 1) | ((src[bit >> 3] >> (7 - (bit & 7))) & 1u);
            switch (outBytes) {
                case 1: dst[s] = static_cast<uint8_t>(v); break;
                case 2: reinterpret_cast<uint16_t*>(dst)[s] = static_cast<uint16_t>(v); break;
                default: reinterpret_cast<uint32_t*>(dst)[s] = v; break;
            }
        }
    }
}

Expected<bool> TiffReader::ReadBlockInto(uint32_t ifd, uint32_t index, std::span<uint8_t> out) const {
    DDBT_TRY(CheckDecodable(ifd));
    auto loc = Locate(ifd, index);
    if (!loc.ok()) return std::move(loc).TakeStatus();
    const IfdInfo& i = ifds_[ifd];
    if (out.size() != i.DecodedBlockBytes(index)) {
        return InvalidArgument("output buffer of " + std::to_string(out.size()) + " bytes, block needs " +
                               std::to_string(i.DecodedBlockBytes(index)));
    }
    if (loc->IsSparse()) {
        FillSparse(ifd, out);
        return true;
    }
    // pread sources stage through a per-thread buffer instead of a fresh vector per block.
    auto bytes = ReadRaw(ifd, index, *loc, tlsCompressed);
    if (!bytes.ok()) return std::move(bytes).TakeStatus();
    DDBT_TRY(DecodeBlock(ifd, index, *bytes, out));
    return false;
}

Expected<DecodedBlock> TiffReader::ReadBlock(uint32_t ifd, uint32_t index) const {
    DDBT_TRY(CheckDecodable(ifd));
    if (ifd >= ifds_.size()) return OutOfRange("IFD " + std::to_string(ifd) + " does not exist");
    const IfdInfo& i = ifds_[ifd];
    if (index >= i.blockCount) return OutOfRange("block " + std::to_string(index));
    const uint64_t size = i.DecodedBlockBytes(index);
    if (size == 0) return OverflowError("block size overflow");
    if (size > options_.maxBlockBytes) {
        return ResourceExhausted("block of " + std::to_string(size) + " bytes (limit " + std::to_string(options_.maxBlockBytes) + ")");
    }
    DecodedBlock b;
    b.buffer = BlockBuffer::Allocate(static_cast<size_t>(size), options_.pool);
    auto sparse = ReadBlockInto(ifd, index, b.buffer.span());
    if (!sparse.ok()) return std::move(sparse).TakeStatus();
    b.width = i.blockWidth;
    b.height = i.BlockRows(index);
    b.samples = i.SamplesPerBlockPixel();
    b.bitsPerSample = i.bitsPerSample;
    b.sampleType = i.sampleType;
    b.rowStride = static_cast<size_t>(i.BlockRowBytes());
    b.sparse = *sparse;
    return b;
}

}  // namespace ddbt::cog::tiff
