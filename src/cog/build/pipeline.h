// SPDX-License-Identifier: MPL-2.0
//
// Internals of the COG builder shared by its producers: the output plan, strips (one row of
// blocks of a level, full width, pixel-interleaved), and the producer interface that fills the
// full-resolution strips (own grid copy or warp).
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "cog/encode/block_encoder.h"
#include "cog/tiff/tiff_reader.h"
#include "core/mem/aligned.h"

namespace ddbt::cog::build {

enum class Transparency : uint8_t {
    kNone,    // every pixel valid (outside the source: 0)
    kMask,    // a 1-bit mask (JPEG + source alpha / mask, or a source's internal mask)
    kAlpha,   // an alpha band kept as the last band
    kNoData,  // the nodata value
};

const char* TransparencyName(Transparency t) noexcept;

struct Plan {
    bool ownGrid = false;
    uint32_t width = 0, height = 0;
    uint32_t blockSize = 512;
    std::array<double, 6> geoTransform{};
    std::vector<std::pair<uint32_t, uint32_t>> overviews;
    int zoom = -1;  // warped grids: the tile matrix level

    int bands = 1;  // bands of the image (the alpha band included when kept)
    tiff::SampleType type = tiff::SampleType::kU8;
    Transparency transparency = Transparency::kNone;
    int alphaBand = -1;                // kAlpha: index of the alpha band
    bool alphaMasksOverviews = false;  // kAlpha: GDAL's alpha mask rule (2 / 4 bands, 8 / 16-bit)
    std::optional<double> nodata;
    std::string nodataText;

    encode::Codec codec = encode::Codec::kLzw;
    int predictor = 1;
    int level = 0;
    int jpegQuality = 75;
    uint16_t photometric = 1;
    std::vector<uint16_t> extraSamples;
    std::vector<uint16_t> colorMap;  // the ColorMap of a palette output (TIFF's R, G, B values)
    bool nearestOverviews = false;   // the COG driver's NEAREST (palette), else CUBIC
    std::string gdalMetadata;

    bool MaskOut() const noexcept { return transparency == Transparency::kMask; }
    /// Strips carry an 8-bit mask (the 1-bit mask's values, or the alpha as GDAL's mask band).
    bool StripMask() const noexcept { return MaskOut() || (transparency == Transparency::kAlpha && alphaMasksOverviews); }
    size_t SampleSize() const noexcept { return tiff::SampleTypeSize(type); }
    size_t PixelBytes() const noexcept { return SampleSize() * static_cast<size_t>(bands); }
};

/// A large buffer mapped from the system (mmap / VirtualAlloc) and returned to it when freed:
/// strips and decoded source blocks are tens to hundreds of MiB, and malloc's arenas would keep
/// such freed blocks (its mmap threshold grows with every large free).
class LargeBytes {
public:
    LargeBytes() noexcept = default;
    explicit LargeBytes(size_t n) : p_(n ? static_cast<uint8_t*>(mem::LargeAlloc(n)) : nullptr), n_(n) {}
    LargeBytes(LargeBytes&& o) noexcept : p_(o.p_), n_(o.n_) {
        o.p_ = nullptr;
        o.n_ = 0;
    }
    LargeBytes& operator=(LargeBytes&& o) noexcept {
        if (this != &o) {
            Reset();
            p_ = o.p_;
            n_ = o.n_;
            o.p_ = nullptr;
            o.n_ = 0;
        }
        return *this;
    }
    LargeBytes(const LargeBytes&) = delete;
    LargeBytes& operator=(const LargeBytes&) = delete;
    ~LargeBytes() { Reset(); }
    void Reset() noexcept {
        if (p_ != nullptr) mem::LargeFree(p_, n_);
        p_ = nullptr;
        n_ = 0;
    }
    uint8_t* get() const noexcept { return p_; }
    size_t size() const noexcept { return n_; }
    explicit operator bool() const noexcept { return p_ != nullptr; }

private:
    uint8_t* p_ = nullptr;
    size_t n_ = 0;
};

/// One row of blocks of a level: rows [y0, y0 + rows) of the full width.
struct Strip {
    uint32_t level = 0, row = 0;
    uint32_t y0 = 0, rows = 0, width = 0;
    size_t pixelBytes = 0;
    LargeBytes pixels;  // rows * width * pixelBytes
    LargeBytes mask;    // rows * width bytes (0..255), when the plan has a strip mask

    uint8_t* Pixel(uint32_t x, uint32_t y) const noexcept {
        return pixels.get() + (static_cast<size_t>(y) * width + x) * pixelBytes;
    }
    uint8_t* MaskAt(uint32_t x, uint32_t y) const noexcept { return mask.get() + static_cast<size_t>(y) * width + x; }
    size_t Stride() const noexcept { return static_cast<size_t>(width) * pixelBytes; }
    size_t Bytes() const noexcept { return static_cast<size_t>(rows) * width * (pixelBytes + (mask ? 1 : 0)); }
};

bool IsFloatType(tiff::SampleType t);
/// A palette output (GTiff's CreateCopy of a 1-band raster with a colour table): photometric
/// palette, the ColorMap GDAL writes for `table` (8-bit entries times 257, 2^bits entries, the
/// missing ones black), overviews by nearest neighbour (the COG driver's rule).
void SetPalette(Plan& p, const std::vector<std::array<uint8_t, 3>>& table);
/// GDAL_METADATA of the output: the source's items (minus its IMAGE_STRUCTURE domain, without
/// the band statistics when `dropStatistics`, and without the colour interpretations when the
/// photometric gives them: RGB of YCbCr JPEG, palette), and the COG driver's
/// OVERVIEW_RESAMPLING when the output has overviews.
std::string OutputMetadata(const std::string& source, const Plan& plan, bool dropStatistics);
/// Photometric (RGB when the first three bands are red, green, blue; else min-is-black) and
/// ExtraSamples (alpha bands: unassociated alpha) GDAL's GTiff CreateCopy writes for bands with
/// these colour interpretations ("Red", "Alpha", ...).
void SetPhotometric(Plan& p, const std::vector<std::string>& interps);
/// GDAL's GetMaskBand alpha rule: the last band of a 2 / 4-band 8 / 16-bit raster masks the others.
bool AlphaMaskRule(int bands, int alphaBand, tiff::SampleType t);

/// Fills the full-resolution strips. Produce() is called concurrently for the columns of a row;
/// BeginRow / EndRow on the driving thread, rows in order.
class TileProducer {
public:
    virtual ~TileProducer() = default;
    virtual void BeginRow(uint32_t /*row*/) {}
    /// Fills columns [col * blockSize, ...) of `strip` (its pixels and, with MaskOut(), its mask).
    virtual void Produce(uint32_t col, Strip& strip) = 0;
    virtual void EndRow(uint32_t /*row*/) {}
    /// Source blocks decoded so far (reporting).
    virtual uint64_t BlocksDecoded() const { return 0; }
    /// Peak bytes of decoded source blocks held (reporting).
    virtual uint64_t PeakCacheBytes() const { return 0; }
};

}  // namespace ddbt::cog::build
