// SPDX-License-Identifier: MPL-2.0
//
// The two point rasterizers of the COPC tiler.
//
// CompatRasterizer is DroneDB 2.4.1's PointRasterizer (src/library/point_rasterizer.cpp), line
// for line: a float z-buffer of the padded tile initialised to -99999, a point wins when the
// buffer at its centre pixel is lower (strict <, the float promoted to double), and then draws
// the baseline's disc (offsets -r..r-1 within r: 11 px for r = 2, asymmetric) at
// (px - drawOffset, py - drawOffset) over whatever is there, so the result depends on the point
// order; colours are kept as 16-bit values and reduced to 8 bits at the end, with v >> 8 when
// any point added had a component above 255. The file is compiled without floating-point
// contraction, so the pixel of every point is the baseline's.
//
// SplatRasterizer serves the hybrid and modern modes: every point is a key (orderable z << 32 |
// RGBA) and every pixel of its disc takes the maximum key of the points covering it: a true
// per-pixel z-test, independent of the point order and of how the points are split
// (deterministic across threads and runs). Two discs:
//  - kStamp11 (hybrid): PointRasterizer's 11-pixel stamp (offsets dx, dy in -2..1 with
//    dx^2 + dy^2 <= 4) around the anchor pixel;
//  - kDisc13 (modern): the 13-pixel symmetric disc of radius 2 (dx^2 + dy^2 <= 4).
// Points are accumulated as the maximum key per anchor pixel; Resolve() then gives each pixel the
// maximum over the anchors whose disc covers it (the reflected disc around the pixel), only over
// the rows and columns touched. Buffers are kept between tiles (one rasterizer per thread): no
// allocation in steady state.
#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace ddbt::copc::tiler {

/// PointRasterizer's PixelMapping: px = round(((x - originX) + shiftX) * scaleX + offsetX),
/// py = height - 1 - round(((y - originY) + shiftY) * scaleY + offsetY).
struct CompatMapping {
    double originX = 0, shiftX = 0, scaleX = 1, offsetX = 0;
    double originY = 0, shiftY = 0, scaleY = 1, offsetY = 0;
    int width = 0;
    int height = 0;
};

class CompatRasterizer {
public:
    /// Starts a tile (PointRasterizer's constructor). Throws std::invalid_argument for the sizes
    /// the baseline refuses.
    void Reset(int imageSize, const CompatMapping& mapping, int drawOffset, int radius);

    /// PointRasterizer::add for n points (x, y in the mapping's coordinates; non-finite x or y
    /// are skipped after the 16-bit check, as in the baseline).
    void Add(const double* x, const double* y, const double* z, const uint16_t* r, const uint16_t* g,
             const uint16_t* b, size_t n);

    size_t drawn() const noexcept { return drawn_; }
    size_t added() const noexcept { return added_; }
    bool sixteenBit() const noexcept { return sixteenBit_; }

    /// toImage(): 3 colour planes of imageSize^2 bytes and the alpha plane.
    void ToPlanes(std::vector<uint8_t>& color, std::vector<uint8_t>& alpha) const;
    /// The same image, interleaved RGBA (imageSize^2 * 4 bytes). Returns the number of opaque
    /// pixels.
    size_t ToRgba(uint8_t* rgba) const;

private:
    void DrawDisc(int px, int py, uint16_t r, uint16_t g, uint16_t b);

    int imageSize_ = 0;
    CompatMapping m_{};
    int drawOffset_ = 0;
    int radius_ = 0;
    std::vector<uint16_t> color_;  // 3 planes of imageSize^2
    std::vector<uint8_t> alpha_;
    std::vector<float> zBuffer_;
    bool sixteenBit_ = false;
    size_t drawn_ = 0;
    size_t added_ = 0;
};

/// z as a 32-bit key with the same order as the floats (total order, -0 below +0).
inline uint32_t OrderableZ(float z) noexcept {
    const uint32_t b = std::bit_cast<uint32_t>(z);
    return (b & 0x80000000u) != 0 ? ~b : (b | 0x80000000u);
}

/// The splat key of a point: z first, then the colour (ties are broken by the larger RGBA).
inline uint64_t SplatKey(float z, uint32_t rgba) noexcept {
    return (uint64_t{OrderableZ(z)} << 32) | rgba;
}

class SplatRasterizer {
public:
    /// Disc radius: both discs lie within offsets -kRadius..kRadius of their anchor pixel.
    static constexpr int kRadius = 2;

    enum class Disc : uint8_t {
        /// The 13-pixel disc dx^2 + dy^2 <= 4 (dx, dy in -2..2), symmetric around the anchor.
        kDisc13,
        /// PointRasterizer's stamp: dx, dy in -2..1 with dx^2 + dy^2 <= 4 (11 pixels; its 4 x 4
        /// box spans pixels anchor - 2 .. anchor + 1, centred on the anchor's top-left corner).
        kStamp11,
    };

    /// Whether offset (dx, dy) from the anchor pixel belongs to `disc` (dy grows downwards).
    static constexpr bool Covers(Disc disc, int dx, int dy) noexcept {
        if (dx * dx + dy * dy > kRadius * kRadius) return false;
        if (disc == Disc::kStamp11) return dx < kRadius && dy < kRadius;
        return true;
    }

    /// Starts a tile of tileSize x tileSize pixels drawn with `disc`. The anchor buffer covers
    /// the tile plus kRadius pixels on each side (points anchored there still cover tile pixels).
    void Reset(int tileSize, Disc disc = Disc::kDisc13);

    /// Width (and height) of the anchor buffer: tileSize + 2 * kRadius. Anchor pixel (cx, cy)
    /// of the buffer is tile pixel (cx - kRadius, cy - kRadius).
    int PaddedSize() const noexcept { return padded_; }

    /// A point anchored on buffer pixel (cx, cy), 0 <= cx, cy < PaddedSize(). key != 0.
    void Add(int cx, int cy, uint64_t key) noexcept {
        uint64_t& c = centre_[static_cast<size_t>(cy) * static_cast<size_t>(padded_) + static_cast<size_t>(cx)];
        if (key > c) c = key;
        if (cx < dirtyMinX_) dirtyMinX_ = cx;
        if (cx > dirtyMaxX_) dirtyMaxX_ = cx;
        if (cy < dirtyMinY_) dirtyMinY_ = cy;
        if (cy > dirtyMaxY_) dirtyMaxY_ = cy;
        ++points_;
    }

    size_t points() const noexcept { return points_; }

    /// Writes the tile as RGBA (tileSize^2 * 4 bytes; transparent black where no disc reaches)
    /// and clears the touched part of the buffer for the next tile. Returns the number of opaque
    /// pixels.
    size_t Resolve(uint8_t* rgba);

private:
    int tile_ = 0;
    int padded_ = 0;
    Disc disc_ = Disc::kDisc13;
    std::vector<uint64_t> centre_;  // padded_^2, all zero between tiles
    int dirtyMinX_ = 0, dirtyMaxX_ = -1, dirtyMinY_ = 0, dirtyMaxY_ = -1;
    size_t points_ = 0;
};

}  // namespace ddbt::copc::tiler
