// SPDX-License-Identifier: MPL-2.0
// Compiled with -ffp-contract=off -fno-lto (sources.cmake): the pixel mapping must round as
// DroneDB's build (generic x86-64, no FMA) does.
#include "copc/tiler/rasterizer.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace ddbt::copc::tiler {

namespace {

// static_cast<int>(d) where the baseline converts a rounded double: on x86-64 a value outside
// the int range becomes INT_MIN (cvttsd2si), which every caller then rejects as off the
// buffer; report it as "outside" instead of relying on undefined behaviour.
inline bool ToInt(double d, int& out) noexcept {
    if (!(d >= static_cast<double>(INT_MIN) && d <= static_cast<double>(INT_MAX))) return false;
    out = static_cast<int>(d);
    return true;
}

}  // namespace

void CompatRasterizer::Reset(int imageSize, const CompatMapping& mapping, int drawOffset, int radius) {
    if (imageSize <= 0 || mapping.width <= 0 || mapping.height <= 0 || radius < 0)
        throw std::invalid_argument("Invalid point rasterizer size");
    imageSize_ = imageSize;
    m_ = mapping;
    drawOffset_ = drawOffset;
    radius_ = radius;
    const size_t pixels = static_cast<size_t>(imageSize) * static_cast<size_t>(imageSize);
    color_.assign(pixels * 3, 0);
    alpha_.assign(pixels, 0);
    zBuffer_.assign(static_cast<size_t>(mapping.width) * static_cast<size_t>(mapping.height), -99999.0f);
    sixteenBit_ = false;
    drawn_ = 0;
    added_ = 0;
}

void CompatRasterizer::Add(const double* xs, const double* ys, const double* zs, const uint16_t* r,
                           const uint16_t* g, const uint16_t* b, size_t n) {
    const CompatMapping& m = m_;
    added_ += n;
    for (size_t i = 0; i < n; i++) {
        if (r[i] > 255 || g[i] > 255 || b[i] > 255) sixteenBit_ = true;

        const double x = xs[i];
        const double y = ys[i];
        if (!std::isfinite(x) || !std::isfinite(y)) continue;  // Not transformable

        int px = 0, pyr = 0;
        if (!ToInt(std::round(((x - m.originX) + m.shiftX) * m.scaleX + m.offsetX), px)) continue;
        if (!ToInt(std::round(((y - m.originY) + m.shiftY) * m.scaleY + m.offsetY), pyr)) continue;
        const int py = m.height - 1 - pyr;
        if (px < 0 || px >= m.width || py < 0 || py >= m.height) continue;

        float& depth = zBuffer_[static_cast<size_t>(py) * static_cast<size_t>(m.width) + static_cast<size_t>(px)];
        if (static_cast<double>(depth) < zs[i]) {  // the float promoted, as in the baseline
            depth = static_cast<float>(zs[i]);
            DrawDisc(px - drawOffset_, py - drawOffset_, r[i], g[i], b[i]);
            drawn_++;
        }
    }
}

void CompatRasterizer::DrawDisc(int px, int py, uint16_t r, uint16_t g, uint16_t b) {
    // Same footprint as the original drawCircle: offsets -radius..radius-1 within radius
    const size_t pixels = static_cast<size_t>(imageSize_) * static_cast<size_t>(imageSize_);
    const int r2 = radius_ * radius_;
    const int rr = radius_ << 1;
    const int area = r2 << 2;
    for (int i = 0; i < area; i++) {
        const int tx = (i % rr) - radius_;
        const int ty = (i / rr) - radius_;
        if (tx * tx + ty * ty > r2) continue;
        const int dx = px + tx;
        const int dy = py + ty;
        if (dx < 0 || dx >= imageSize_ || dy < 0 || dy >= imageSize_) continue;
        const size_t idx = static_cast<size_t>(dy) * static_cast<size_t>(imageSize_) + static_cast<size_t>(dx);
        color_[idx] = r;
        color_[idx + pixels] = g;
        color_[idx + pixels * 2] = b;
        alpha_[idx] = 255;
    }
}

void CompatRasterizer::ToPlanes(std::vector<uint8_t>& color, std::vector<uint8_t>& alpha) const {
    color.resize(color_.size());
    for (size_t i = 0; i < color_.size(); i++)
        color[i] = sixteenBit_ ? static_cast<uint8_t>(color_[i] >> 8) : static_cast<uint8_t>(color_[i]);
    alpha = alpha_;
}

size_t CompatRasterizer::ToRgba(uint8_t* rgba) const {
    const size_t pixels = alpha_.size();
    const int shift = sixteenBit_ ? 8 : 0;
    size_t opaque = 0;
    for (size_t i = 0; i < pixels; i++) {
        uint8_t* p = rgba + i * 4;
        p[0] = static_cast<uint8_t>(color_[i] >> shift);
        p[1] = static_cast<uint8_t>(color_[i + pixels] >> shift);
        p[2] = static_cast<uint8_t>(color_[i + 2 * pixels] >> shift);
        p[3] = alpha_[i];
        opaque += alpha_[i] != 0;
    }
    return opaque;
}

void SplatRasterizer::Reset(int tileSize, Disc disc) {
    if (tileSize <= 0) throw std::invalid_argument("Invalid tile size");
    disc_ = disc;
    if (tileSize != tile_) {
        tile_ = tileSize;
        padded_ = tileSize + 2 * kRadius;
        centre_.assign(static_cast<size_t>(padded_) * static_cast<size_t>(padded_), 0);
        dirtyMinX_ = dirtyMinY_ = padded_;
        dirtyMaxX_ = dirtyMaxY_ = -1;
    } else if (dirtyMaxX_ >= 0) {
        // A tile left unresolved (an exception between Reset and Resolve): clear what it touched.
        for (int y = dirtyMinY_; y <= dirtyMaxY_; ++y)
            std::fill_n(centre_.data() + static_cast<size_t>(y) * static_cast<size_t>(padded_) + dirtyMinX_,
                        dirtyMaxX_ - dirtyMinX_ + 1, uint64_t{0});
        dirtyMinX_ = dirtyMinY_ = padded_;
        dirtyMaxX_ = dirtyMaxY_ = -1;
    }
    points_ = 0;
}

size_t SplatRasterizer::Resolve(uint8_t* rgba) {
    const int T = tile_;
    const size_t P = static_cast<size_t>(padded_);
    std::memset(rgba, 0, static_cast<size_t>(T) * static_cast<size_t>(T) * 4);
    if (dirtyMaxX_ < 0) return 0;

    // Tile pixel (ox, oy) is anchor pixel (ox + 2, oy + 2); it can only be reached by anchors
    // within 2 pixels of it.
    const int ox0 = std::max(0, dirtyMinX_ - 2 * kRadius), ox1 = std::min(T - 1, dirtyMaxX_);
    const int oy0 = std::max(0, dirtyMinY_ - 2 * kRadius), oy1 = std::min(T - 1, dirtyMaxY_);
    const uint64_t* c = centre_.data();
    size_t opaque = 0;
    const bool stamp = disc_ == Disc::kStamp11;
    for (int oy = oy0; oy <= oy1; ++oy) {
        // Rows cy - 2 .. cy + 2 of the anchor buffer, cy = oy + 2.
        const uint64_t* r0 = c + static_cast<size_t>(oy) * P;
        const uint64_t* r1 = r0 + P;
        const uint64_t* r2 = r1 + P;
        const uint64_t* r3 = r2 + P;
        const uint64_t* r4 = r3 + P;
        uint32_t* out = reinterpret_cast<uint32_t*>(rgba) + static_cast<size_t>(oy) * static_cast<size_t>(T);
        if (stamp) {
            // The anchors whose stamp covers the pixel: the stamp reflected around it (anchor
            // offsets -dx, -dy for every stamp offset dx, dy in -2..1): one anchor 2 rows below,
            // 3 on the rows 1 below and 1 above, 4 on the pixel's row (1 west .. 2 east).
            for (int ox = ox0; ox <= ox1; ++ox) {
                uint64_t m = std::max(r4[ox + 2], r2[ox + 4]);
                m = std::max(m, std::max(std::max(r1[ox + 1], r1[ox + 2]), r1[ox + 3]));
                m = std::max(m, std::max(std::max(r2[ox + 1], r2[ox + 2]), r2[ox + 3]));
                m = std::max(m, std::max(std::max(r3[ox + 1], r3[ox + 2]), r3[ox + 3]));
                const uint32_t px = static_cast<uint32_t>(m);  // RGBA, A = 255 for every key
                out[ox] = px;
                opaque += px != 0;
            }
            continue;
        }
        for (int ox = ox0; ox <= ox1; ++ox) {
            // Disc of radius 2: the 3 x 3 square plus the four pixels 2 away on the axes.
            uint64_t m = std::max(r2[ox], r2[ox + 4]);
            m = std::max(m, std::max(r0[ox + 2], r4[ox + 2]));
            m = std::max(m, std::max(std::max(r1[ox + 1], r1[ox + 2]), r1[ox + 3]));
            m = std::max(m, std::max(std::max(r2[ox + 1], r2[ox + 2]), r2[ox + 3]));
            m = std::max(m, std::max(std::max(r3[ox + 1], r3[ox + 2]), r3[ox + 3]));
            const uint32_t px = static_cast<uint32_t>(m);  // RGBA, A = 255 for every key
            out[ox] = px;
            opaque += px != 0;
        }
    }
    for (int y = dirtyMinY_; y <= dirtyMaxY_; ++y)
        std::fill_n(centre_.data() + static_cast<size_t>(y) * P + static_cast<size_t>(dirtyMinX_),
                    dirtyMaxX_ - dirtyMinX_ + 1, uint64_t{0});
    dirtyMinX_ = dirtyMinY_ = padded_;
    dirtyMaxX_ = dirtyMaxY_ = -1;
    return opaque;
}

}  // namespace ddbt::copc::tiler
