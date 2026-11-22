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

void SplatRasterizer::Reset(int tileSize, Disc disc, int pad) {
    if (tileSize <= 0) throw std::invalid_argument("Invalid tile size");
    if (pad < kRadius || pad > 4096) throw std::invalid_argument("Invalid anchor pad");
    disc_ = disc;
    if (tileSize != tile_ || pad != pad_) {
        tile_ = tileSize;
        pad_ = pad;
        padded_ = tileSize + 2 * pad;
        centre_.assign(static_cast<size_t>(padded_) * static_cast<size_t>(padded_), 0);
        dirtyMinX_ = dirtyMinY_ = padded_;
        dirtyMaxX_ = dirtyMaxY_ = -1;
    } else {
        // A tile left unresolved (an exception between Reset and Resolve): clear what it touched.
        ClearAnchors();
    }
    points_ = 0;
}

void SplatRasterizer::ClearAnchors() noexcept {
    if (dirtyMaxX_ < 0) return;
    for (int y = dirtyMinY_; y <= dirtyMaxY_; ++y)
        std::fill_n(centre_.data() + static_cast<size_t>(y) * static_cast<size_t>(padded_) +
                        static_cast<size_t>(dirtyMinX_),
                    dirtyMaxX_ - dirtyMinX_ + 1, uint64_t{0});
    dirtyMinX_ = dirtyMinY_ = padded_;
    dirtyMaxX_ = dirtyMaxY_ = -1;
}

template <typename Emit>
void SplatRasterizer::GatherDiscs(bool stamp, Emit&& emit) const {
    const int T = tile_;
    const size_t P = static_cast<size_t>(padded_);
    // Tile pixel (ox, oy) is anchor pixel (ox + pad, oy + pad); it can only be reached by anchors
    // within kRadius pixels of it. The rows below are read from the anchor buffer shifted by
    // pad - kRadius, so that anchor (ox + pad + dx, oy + pad + dy) is r[2 + dy][ox + 2 + dx].
    const int shift = pad_ - kRadius;
    const int ox0 = std::max(0, dirtyMinX_ - shift - 2 * kRadius), ox1 = std::min(T - 1, dirtyMaxX_ - shift);
    const int oy0 = std::max(0, dirtyMinY_ - shift - 2 * kRadius), oy1 = std::min(T - 1, dirtyMaxY_ - shift);
    const uint64_t* c = centre_.data() + static_cast<size_t>(shift) * P + static_cast<size_t>(shift);
    for (int oy = oy0; oy <= oy1; ++oy) {
        // Rows cy - 2 .. cy + 2 of the anchor buffer, cy = oy + pad.
        const uint64_t* r0 = c + static_cast<size_t>(oy) * P;
        const uint64_t* r1 = r0 + P;
        const uint64_t* r2 = r1 + P;
        const uint64_t* r3 = r2 + P;
        const uint64_t* r4 = r3 + P;
        const size_t row = static_cast<size_t>(oy) * static_cast<size_t>(T);
        if (stamp) {
            // The anchors whose stamp covers the pixel: the stamp reflected around it (anchor
            // offsets -dx, -dy for every stamp offset dx, dy in -2..1): one anchor 2 rows below,
            // 3 on the rows 1 below and 1 above, 4 on the pixel's row (1 west .. 2 east).
            for (int ox = ox0; ox <= ox1; ++ox) {
                uint64_t m = std::max(r4[ox + 2], r2[ox + 4]);
                m = std::max(m, std::max(std::max(r1[ox + 1], r1[ox + 2]), r1[ox + 3]));
                m = std::max(m, std::max(std::max(r2[ox + 1], r2[ox + 2]), r2[ox + 3]));
                m = std::max(m, std::max(std::max(r3[ox + 1], r3[ox + 2]), r3[ox + 3]));
                emit(row + static_cast<size_t>(ox), m);
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
            emit(row + static_cast<size_t>(ox), m);
        }
    }
}

size_t SplatRasterizer::Resolve(uint8_t* rgba) {
    std::memset(rgba, 0, static_cast<size_t>(tile_) * static_cast<size_t>(tile_) * 4);
    if (dirtyMaxX_ < 0) return 0;
    uint32_t* out = reinterpret_cast<uint32_t*>(rgba);
    size_t opaque = 0;
    GatherDiscs(disc_ == Disc::kStamp11, [&](size_t i, uint64_t key) {
        const uint32_t px = static_cast<uint32_t>(key);  // RGBA, A = 255 for every key
        out[i] = px;
        opaque += px != 0;
    });
    ClearAnchors();
    return opaque;
}

int SplatRasterizer::AdaptiveRadius(int kthSquared, int maxRadius) noexcept {
    if (kthSquared <= 0 || maxRadius <= kRadius) return kRadius;
    const double spacing = std::sqrt(static_cast<double>(kthSquared));
    if (spacing <= kGaplessSpacing) return kRadius;
    const double r = std::ceil(kSpacingToRadius * spacing);
    return r >= maxRadius ? maxRadius : std::max(kRadius, static_cast<int>(r));
}

void SplatRasterizer::CountAnchors() {
    const size_t w = static_cast<size_t>(dirtyMaxX_ - dirtyMinX_ + 1) + 1;
    const size_t h = static_cast<size_t>(dirtyMaxY_ - dirtyMinY_ + 1) + 1;
    counts_.assign(w * h, 0);
    const size_t P = static_cast<size_t>(padded_);
    for (size_t y = 1; y < h; ++y) {
        const uint64_t* row = centre_.data() + (static_cast<size_t>(dirtyMinY_) + y - 1) * P + static_cast<size_t>(dirtyMinX_);
        uint32_t* out = counts_.data() + y * w;
        const uint32_t* above = out - w;
        uint32_t inRow = 0;
        for (size_t x = 1; x < w; ++x) {
            inRow += row[x - 1] != 0;
            out[x] = above[x] + inRow;
        }
    }
}

int SplatRasterizer::AnchorsIn(int x0, int y0, int x1, int y1) const noexcept {
    x0 = std::max(x0, dirtyMinX_) - dirtyMinX_;
    y0 = std::max(y0, dirtyMinY_) - dirtyMinY_;
    x1 = std::min(x1, dirtyMaxX_) - dirtyMinX_;
    y1 = std::min(y1, dirtyMaxY_) - dirtyMinY_;
    if (x0 > x1 || y0 > y1) return 0;
    const size_t w = static_cast<size_t>(dirtyMaxX_ - dirtyMinX_ + 1) + 1;
    const uint32_t* c = counts_.data();
    const auto at = [&](int x, int y) { return c[static_cast<size_t>(y) * w + static_cast<size_t>(x)]; };
    return static_cast<int>(at(x1 + 1, y1 + 1) - at(x0, y1 + 1) - at(x1 + 1, y0) + at(x0, y0));
}

int SplatRasterizer::KthNeighbourSquared(int x, int y, int search) const noexcept {
    // The kNeighbours smallest squared distances found so far, ascending.
    int nearest[kNeighbours];
    int found = 0;
    const uint64_t* c = centre_.data();
    const size_t P = static_cast<size_t>(padded_);
    auto visit = [&](int cx, int cy) {
        if (cx < dirtyMinX_ || cx > dirtyMaxX_ || cy < dirtyMinY_ || cy > dirtyMaxY_) return;
        if (c[static_cast<size_t>(cy) * P + static_cast<size_t>(cx)] == 0) return;
        const int d2 = (cx - x) * (cx - x) + (cy - y) * (cy - y);
        if (found == kNeighbours && d2 >= nearest[kNeighbours - 1]) return;
        int i = found < kNeighbours ? found++ : kNeighbours - 1;
        for (; i > 0 && nearest[i - 1] > d2; --i) nearest[i] = nearest[i - 1];
        nearest[i] = d2;
    };
    // Square rings of growing Chebyshev radius: every anchor of ring k is at least k pixels away,
    // so the search stops once the kNeighbours-th distance is within the next ring. A ring without
    // anchors (the counts of the boxes inside and around it are equal) is not scanned.
    int inside = 1;  // the anchor itself
    for (int k = 1; k <= search; ++k) {
        const int box = AnchorsIn(x - k, y - k, x + k, y + k);
        if (box > inside) {
            for (int dx = -k; dx <= k; ++dx) {
                visit(x + dx, y - k);
                visit(x + dx, y + k);
            }
            for (int dy = -k + 1; dy <= k - 1; ++dy) {
                visit(x - k, y + dy);
                visit(x + k, y + dy);
            }
        }
        inside = box;
        if (found == kNeighbours && nearest[kNeighbours - 1] <= (k + 1) * (k + 1)) break;
        // No anchor lies beyond the touched rectangle.
        if (x - k <= dirtyMinX_ && x + k >= dirtyMaxX_ && y - k <= dirtyMinY_ && y + k >= dirtyMaxY_) break;
    }
    return found == kNeighbours ? nearest[kNeighbours - 1] : 0;
}

size_t SplatRasterizer::ResolveAdaptive(uint8_t* rgba, int maxRadius) {
    const int T = tile_;
    const size_t pixels = static_cast<size_t>(T) * static_cast<size_t>(T);
    if (dirtyMaxX_ < 0) {
        std::memset(rgba, 0, pixels * 4);
        return 0;
    }
    maxRadius = std::max(kRadius, maxRadius);
    const int search = AdaptiveSearch(maxRadius);
    // Every anchor's 13-pixel disc, then the larger discs over them (the maximum key either way).
    best_.assign(pixels, 0);
    GatherDiscs(false, [&](size_t i, uint64_t key) { best_[i] = key; });
    CountAnchors();
    const uint64_t* c = centre_.data();
    const size_t P = static_cast<size_t>(padded_);
    auto drawRow = [&](int py, int tx, int w, uint64_t key) {
        if (py < 0 || py >= T) return;
        const int x0 = std::max(0, tx - w), x1 = std::min(T - 1, tx + w);
        uint64_t* out = best_.data() + static_cast<size_t>(py) * static_cast<size_t>(T);
        for (int px = x0; px <= x1; ++px) out[px] = std::max(out[px], key);
    };
    for (int y = dirtyMinY_; y <= dirtyMaxY_; ++y) {
        const uint64_t* row = c + static_cast<size_t>(y) * P;
        for (int x = dirtyMinX_; x <= dirtyMaxX_; ++x) {
            const uint64_t key = row[x];
            if (key == 0) continue;
            // The anchor's tile pixel; a disc that cannot reach the tile is skipped unmeasured.
            const int tx = x - pad_, ty = y - pad_;
            if (tx < -maxRadius || ty < -maxRadius || tx >= T + maxRadius || ty >= T + maxRadius) continue;
            // kNeighbours anchors in the 5 x 5 box around it are at most kGaplessSpacing away: the
            // 13-pixel disc, drawn already.
            if (AnchorsIn(x - kRadius, y - kRadius, x + kRadius, y + kRadius) > kNeighbours) continue;
            const int r = AdaptiveRadius(KthNeighbourSquared(x, y, search), maxRadius);
            if (r == kRadius || ty + r < 0 || ty - r >= T || tx + r < 0 || tx - r >= T) continue;
            // Row dy of the disc spans tx - w .. tx + w, the largest w with w^2 + dy^2 <= r^2.
            int w = r;
            for (int dy = 0; dy <= r; ++dy) {
                while (w * w + dy * dy > r * r) --w;
                drawRow(ty - dy, tx, w, key);
                if (dy > 0) drawRow(ty + dy, tx, w, key);
            }
        }
    }
    size_t opaque = 0;
    uint32_t* out = reinterpret_cast<uint32_t*>(rgba);
    for (size_t i = 0; i < pixels; ++i) {
        const uint32_t px = static_cast<uint32_t>(best_[i]);  // RGBA, A = 255 for every key
        out[i] = px;
        opaque += px != 0;
    }
    ClearAnchors();
    return opaque;
}

}  // namespace ddbt::copc::tiler
