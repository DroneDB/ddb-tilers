// SPDX-License-Identifier: MPL-2.0
#include "core/encode/png_encoder.h"

#include <fpnge.h>
#include <libdeflate.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>

#include "core/encode/kernels.h"

namespace ddbt::encode {

namespace {

constexpr int kMaxLevel = 12;
// Levels 10-12 (near-optimal parsing) keep megabytes of state: only blank tiles use them, so they
// are allocated per call instead of living in every thread.
constexpr int kMaxCachedLevel = 9;

struct CompressorDeleter {
    void operator()(libdeflate_compressor* c) const noexcept { libdeflate_free_compressor(c); }
};
using CompressorPtr = std::unique_ptr<libdeflate_compressor, CompressorDeleter>;

// One libdeflate compressor per level and thread: allocated on first use, freed at thread exit.
class CompressorCache {
public:
    CompressorCache() = default;
    CompressorCache(const CompressorCache&) = delete;
    CompressorCache& operator=(const CompressorCache&) = delete;
    ~CompressorCache() {
        for (auto* c : compressors_) {
            if (c != nullptr) libdeflate_free_compressor(c);
        }
    }

    libdeflate_compressor* Get(int level) {
        auto& c = compressors_[static_cast<size_t>(level)];
        if (c == nullptr) {
            c = libdeflate_alloc_compressor(level);
            if (c == nullptr) throw std::bad_alloc();
        }
        return c;
    }

private:
    std::array<libdeflate_compressor*, kMaxCachedLevel + 1> compressors_{};
};

// Per-thread buffers: the filtered rows (deflate input) and conversion scratch.
struct PngScratch {
    ByteBuffer filtered;
    ByteBuffer rows;
    ByteBuffer packed;  // fpnge input when the alpha channel is dropped
};

thread_local CompressorCache tlsCompressors;
thread_local PngScratch tlsScratch;

inline void PutU32(uint8_t* p, uint32_t v) noexcept {
    p[0] = static_cast<uint8_t>(v >> 24);
    p[1] = static_cast<uint8_t>(v >> 16);
    p[2] = static_cast<uint8_t>(v >> 8);
    p[3] = static_cast<uint8_t>(v);
}

// Writes a whole chunk (length, type, data, CRC of type + data) at out's end.
void AppendChunk(ByteBuffer& out, const char type[4], const uint8_t* data, uint32_t length) {
    const size_t at = out.size();
    out.resize(at + 12 + length);
    uint8_t* p = out.data() + at;
    PutU32(p, length);
    std::memcpy(p + 4, type, 4);
    if (length != 0) std::memcpy(p + 8, data, length);
    PutU32(p + 8 + length, libdeflate_crc32(0, p + 4, size_t{length} + 4));
}

constexpr uint8_t kSignature[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};

// fpnge: grey, grey + alpha, RGB or RGBA rows (dstChannels per pixel) straight from the image,
// or through a packed copy when the opaque alpha channel is dropped. fpnge writes the signature,
// IHDR (8-bit, the colour type of the channel count), IDAT and IEND.
void EncodeFpnge(const TileImage& image, uint32_t dstChannels, int level, ByteBuffer& out) {
    const uint8_t* pixels = image.pixels;
    size_t stride = image.RowStride();
    if (dstChannels != image.channels) {
        ByteBuffer& packed = tlsScratch.packed;
        const size_t rowBytes = static_cast<size_t>(image.width) * dstChannels;
        packed.resize(rowBytes * image.height);
        for (uint32_t y = 0; y < image.height; ++y) {
            const uint8_t* src = image.pixels + static_cast<size_t>(y) * image.RowStride();
            uint8_t* dst = packed.data() + static_cast<size_t>(y) * rowBytes;
            if (image.channels == 4)
                kernels::DropAlpha4(src, image.width, dst);
            else
                kernels::DropAlpha2(src, image.width, dst);
        }
        pixels = packed.data();
        stride = rowBytes;
    }
    FPNGEOptions o;
    FPNGEFillOptions(&o, level, FPNGE_CICP_NONE);
    out.clear();
    out.resize(FPNGEOutputAllocSize(1, dstChannels, image.width, image.height));
    const size_t n = FPNGEEncode(1, dstChannels, pixels, image.width, stride, image.height, out.data(), &o);
    if (n == 0) throw std::runtime_error("PNG: fpnge failed");
    out.resize(n);
}

PngEngine EngineFromEnvironment() noexcept {
    if (const char* e = std::getenv("DDBT_PNG_ENGINE")) {
        if (std::strcmp(e, "deflate") == 0) return PngEngine::kDeflate;
        if (std::strcmp(e, "fpnge") == 0) return PngEngine::kFpnge;
    }
    // libdeflate until the end-to-end comparison settles the default: fpnge encodes raster tiles
    // 4.6-5.3x faster but writes 20-25 % more bytes (26-65 % on sparse point-cloud tiles).
    return PngEngine::kDeflate;
}

}  // namespace

PngEngine DefaultPngEngine() noexcept {
    static const PngEngine engine = EngineFromEnvironment();
    return engine;
}

void EncodePng(const TileImage& input, const PngOptions& options, ByteBuffer& out) {
    if (const char* err = ValidateTileImage(input)) throw std::invalid_argument(std::string("PNG: ") + err);
    if (options.level < 0 || options.level > kMaxLevel)
        throw std::invalid_argument("PNG: compression level must be 0 to 12");
    if (options.fpngeLevel < 1 || options.fpngeLevel > 5) throw std::invalid_argument("PNG: fpnge level must be 1 to 5");

    TileImage image = input;
    if (!image.alphaKnown) ScanAlpha(image);

    // Colour type and the channels written per pixel
    uint8_t colorType = 0;
    uint32_t dstChannels = image.channels;
    const bool dropAlpha = options.dropOpaqueAlpha && image.allOpaque;
    if (image.IsPalette()) {
        colorType = 3;
    } else {
        switch (image.channels) {
            case 1: colorType = 0; break;
            case 2: colorType = dropAlpha ? 0 : 4; break;
            case 3: colorType = 2; break;
            default: colorType = dropAlpha ? 2 : 6; break;
        }
        if (dropAlpha && (image.channels == 2 || image.channels == 4)) dstChannels = image.channels - 1;
    }

    const PngEngine engine = options.engine == PngEngine::kDefault ? DefaultPngEngine() : options.engine;
    if (engine == PngEngine::kFpnge && !image.IsPalette()) {
        EncodeFpnge(image, dstChannels, options.fpngeLevel, out);
        return;
    }

    // Filter the rows into the deflate input
    const size_t rowBytes = static_cast<size_t>(image.width) * dstChannels;
    const size_t rawSize = (rowBytes + 1) * image.height;
    ByteBuffer& filtered = tlsScratch.filtered;
    filtered.resize(rawSize);
    ByteBuffer& rows = tlsScratch.rows;
    rows.resize(3 * rowBytes);
    kernels::PngFilterJob job;
    job.src = image.pixels;
    job.srcStride = image.RowStride();
    job.width = image.width;
    job.height = image.height;
    job.srcChannels = image.channels;
    job.dstChannels = dstChannels;
    job.filter = image.IsPalette() ? PngRowFilter::kNone : options.filter;
    job.out = filtered.data();
    job.scratch = rows.data();
    kernels::PngFilterImage(job);

    // Header chunks
    CompressorPtr oneShot;
    libdeflate_compressor* compressor = nullptr;
    if (options.level <= kMaxCachedLevel) {
        compressor = tlsCompressors.Get(options.level);
    } else {
        oneShot.reset(libdeflate_alloc_compressor(options.level));
        if (!oneShot) throw std::bad_alloc();
        compressor = oneShot.get();
    }
    const size_t bound = libdeflate_zlib_compress_bound(compressor, rawSize);
    out.clear();
    out.reserve(sizeof(kSignature) + 25 + (12 + 768) + (12 + 256) + 12 + bound + 12);
    out.append(kSignature, sizeof(kSignature));

    uint8_t ihdr[13];
    PutU32(ihdr, image.width);
    PutU32(ihdr + 4, image.height);
    ihdr[8] = 8;  // bit depth
    ihdr[9] = colorType;
    ihdr[10] = 0;  // deflate
    ihdr[11] = 0;  // adaptive filtering
    ihdr[12] = 0;  // no interlace
    AppendChunk(out, "IHDR", ihdr, sizeof(ihdr));

    if (image.IsPalette()) {
        uint8_t plte[768];
        uint8_t trns[256];
        uint32_t trnsLength = 0;
        for (uint32_t i = 0; i < image.paletteSize; ++i) {
            plte[3 * i] = image.palette[i].r;
            plte[3 * i + 1] = image.palette[i].g;
            plte[3 * i + 2] = image.palette[i].b;
            trns[i] = image.palette[i].a;
            if (trns[i] != 255) trnsLength = i + 1;
        }
        AppendChunk(out, "PLTE", plte, 3 * image.paletteSize);
        if (trnsLength != 0 && !dropAlpha) AppendChunk(out, "tRNS", trns, trnsLength);
    }

    // IDAT: compressed straight into out
    const size_t idat = out.size();
    out.resize(idat + 8 + bound);
    const size_t compressed =
        libdeflate_zlib_compress(compressor, filtered.data(), rawSize, out.data() + idat + 8, bound);
    if (compressed == 0) throw std::runtime_error("PNG: libdeflate_zlib_compress failed");
    if (compressed > 0x7FFFFFFFu) throw std::runtime_error("PNG: image data exceeds one IDAT chunk");
    uint8_t* p = out.data() + idat;
    PutU32(p, static_cast<uint32_t>(compressed));
    std::memcpy(p + 4, "IDAT", 4);
    out.resize(idat + 8 + compressed + 4);
    PutU32(out.data() + idat + 8 + compressed, libdeflate_crc32(0, out.data() + idat + 4, compressed + 4));

    AppendChunk(out, "IEND", nullptr, 0);
}

}  // namespace ddbt::encode
