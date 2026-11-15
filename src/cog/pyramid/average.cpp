// SPDX-License-Identifier: MPL-2.0
#include "cog/pyramid/average.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ddbt::cog::pyramid {

int AverageTaps::SrcBegin() const {
    int b = std::numeric_limits<int>::max();
    for (size_t i = 0; i < start.size(); ++i)
        if (count[i] > 0) b = std::min(b, start[i]);
    return b == std::numeric_limits<int>::max() ? 0 : b;
}

int AverageTaps::SrcEnd() const {
    int e = 0;
    for (size_t i = 0; i < start.size(); ++i) e = std::max(e, start[i] + count[i]);
    return e;
}

AverageTaps AverageAxis(int srcSize, int dstSize, int dst0, int dst1) {
    AverageTaps t;
    t.dst0 = dst0;
    const size_t n = static_cast<size_t>(std::max(0, dst1 - dst0));
    t.start.resize(n);
    t.count.resize(n);
    t.first.resize(n);
    t.last.resize(n);
    t.total.resize(n);
    const double ratio = static_cast<double>(srcSize) / dstSize;
    for (int i = dst0; i < dst1; ++i) {
        const size_t k = static_cast<size_t>(i - dst0);
        // The precomputation of GDALResampleChunk_AverageOrRMS_T (with an epsilon against
        // rounding), the chunk being the whole source
        const double off = i * ratio;
        int s0 = static_cast<int>(off + 1e-8);
        const double off2 = (i + 1) * ratio;
        int s1 = static_cast<int>(std::ceil(off2 - 1e-8));
        if (s0 < 0) s0 = 0;
        if (s1 == s0) ++s1;
        if (s1 > srcSize) s1 = srcSize;
        t.start[k] = s0;
        t.count[k] = s1 - s0;
        t.first[k] = (s1 == s0 + 1) ? 1.0 : 1 - (off - s0);
        t.last[k] = 1 - (s1 - off2);
        double total = t.first[k];
        if (s0 + 1 < s1) {
            total += s1 - s0 - 2;
            total += t.last[k];
        }
        t.total[k] = total;
    }
    return t;
}

namespace {

// GDALGetNoDataReplacementValue for UInt8
uint8_t Replacement(uint8_t nodata) { return nodata == 255 ? 254 : static_cast<uint8_t>(nodata + 1); }

uint8_t Rounded(double total, double weight) { return static_cast<uint8_t>(total / weight + 0.5); }

}  // namespace

void Average(const DownsampleSpec& spec, const uint8_t* src, size_t srcStride, int srcX0, int srcY0, int srcW,
             int srcH, const uint8_t* srcMask, const AverageTaps& cols, const AverageTaps& rows, int dstW, int dstH,
             uint8_t* dst, size_t dstStride) {
    if (spec.type != tiff::SampleType::kU8) throw std::invalid_argument("Average: 8-bit samples only");
    (void)srcH;
    const int bands = spec.bands;
    const bool hasNoData = spec.hasNoData;
    const uint8_t nodata = hasNoData ? static_cast<uint8_t>(spec.noData) : 0;
    const uint8_t replacement = Replacement(nodata);
    // A sample at (x, y) of the source window (absolute coordinates) of band b
    const auto sample = [&](int x, int y, int b) {
        return src[static_cast<size_t>(y - srcY0) * srcStride + static_cast<size_t>(x - srcX0) * bands + b];
    };
    const auto valid = [&](int x, int y, int b) {
        switch (spec.mask) {
            case MaskMode::kExternal:
                return srcMask == nullptr ||
                       srcMask[static_cast<size_t>(y - srcY0) * static_cast<size_t>(srcW) + (x - srcX0)] != 0;
            case MaskMode::kNoData:
                return !hasNoData || sample(x, y, b) != nodata;
            default:
                return true;
        }
    };
    for (int dy = 0; dy < dstH; ++dy) {
        const size_t ry = static_cast<size_t>(dy);
        const int y0 = rows.start[ry], y1 = y0 + rows.count[ry];
        const double bottom = rows.first[ry], top = rows.last[ry];
        uint8_t* out = dst + static_cast<size_t>(dy) * dstStride;
        for (int dx = 0; dx < dstW; ++dx) {
            const size_t cx = static_cast<size_t>(dx);
            const int x0 = cols.start[cx], x1 = x0 + cols.count[cx];
            const double left = cols.first[cx], right = cols.last[cx];
            for (int b = 0; b < bands; ++b) {
                uint8_t value;
                if (spec.mask == MaskMode::kNone) {
                    double total = 0;
                    for (int y = y0; y < y1; ++y) {
                        double line = sample(x0, y, b) * left;
                        if (x0 + 1 < x1) {
                            for (int x = x0 + 1; x < x1 - 1; ++x) line += sample(x, y, b);
                            line += sample(x1 - 1, y, b) * right;
                        }
                        const double wy = y == y0 ? bottom : (y + 1 == y1 ? top : 1.0);
                        total += line * wy;
                    }
                    value = Rounded(total, cols.total[cx] * rows.total[ry]);
                } else {
                    double total = 0, weight = 0;
                    size_t counted = 0;
                    for (int y = y0; y < y1; ++y) {
                        double line = 0, lineWeight = 0;
                        if (valid(x0, y, b)) {
                            ++counted;
                            lineWeight = left;
                            line = sample(x0, y, b) * left;
                        }
                        if (x0 < x1 - 1) {
                            for (int x = x0 + 1; x < x1 - 1; ++x) {
                                if (valid(x, y, b)) {
                                    ++counted;
                                    lineWeight += 1;
                                    line += sample(x, y, b);
                                }
                            }
                            if (valid(x1 - 1, y, b)) {
                                ++counted;
                                lineWeight += right;
                                line += sample(x1 - 1, y, b) * right;
                            }
                        }
                        const double wy = y == y0 ? bottom : (y + 1 == y1 ? top : 1.0);
                        total += line * wy;
                        weight += lineWeight * wy;
                    }
                    if (counted == 0) {
                        out[cx * static_cast<size_t>(bands) + static_cast<size_t>(b)] = nodata;
                        continue;
                    }
                    value = Rounded(total, weight);
                }
                if (hasNoData && value == nodata) value = replacement;
                out[cx * static_cast<size_t>(bands) + static_cast<size_t>(b)] = value;
            }
        }
    }
}

}  // namespace ddbt::cog::pyramid
