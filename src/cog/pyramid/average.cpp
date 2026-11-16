// SPDX-License-Identifier: MPL-2.0
#include "cog/pyramid/average.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

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

AverageTaps AverageAxis(int srcSize, int dstSize, int dst0, int dst1, int chunkRows) {
    AverageTaps t;
    t.dst0 = dst0;
    t.half = srcSize == 2 * dstSize;
    const size_t n = static_cast<size_t>(std::max(0, dst1 - dst0));
    t.start.resize(n);
    t.count.resize(n);
    t.first.resize(n);
    t.last.resize(n);
    t.total.resize(n);
    const double ratio = static_cast<double>(srcSize) / dstSize;
    const int chunks = chunkRows > 0 ? (srcSize + chunkRows - 1) / chunkRows : 1;
    // GDALRegenerateOverviewsEx: the first destination row of the chunk that starts at source row y
    const auto chunkDst = [&](int y) { return static_cast<int>(0.5 + y / ratio); };
    for (int i = dst0; i < dst1; ++i) {
        const size_t k = static_cast<size_t>(i - dst0);
        // The source rows of the chunk that computes destination row i: the last chunk whose first
        // destination row is at most i (the whole axis without chunks)
        int lo = 0, hi = srcSize;
        if (chunkRows > 0) {
            int j = std::min(chunks - 1, static_cast<int>(i * ratio / chunkRows));
            while (j + 1 < chunks && chunkDst((j + 1) * chunkRows) <= i) ++j;
            while (j > 0 && chunkDst(j * chunkRows) > i) --j;
            lo = j * chunkRows;
            hi = std::min(srcSize, lo + chunkRows);
        }
        // The precomputation of GDALResampleChunk_AverageOrRMS_T (with an epsilon against
        // rounding), clipped to the chunk
        const double off = i * ratio;
        int s0 = static_cast<int>(off + 1e-8);
        const double off2 = (i + 1) * ratio;
        int s1 = static_cast<int>(std::ceil(off2 - 1e-8));
        if (s0 < lo) s0 = lo;
        if (s1 == s0) ++s1;
        if (s1 > hi) s1 = hi;
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

constexpr int64_t kChunkMaxSize = 10485760;  // GDAL_OVR_CHUNK_MAX_SIZE

// UpdateChunkHeightAndGetChunkSize's first chunk: 1-byte work samples, no kernel radius
int64_t FirstChunkRows(int factor, int blockHeight) { return std::max<int64_t>(blockHeight, 2 * int64_t{factor}); }

}  // namespace

int OverviewFactor(int srcWidth, int srcHeight, int dstWidth, int dstHeight) {
    return std::max({1, static_cast<int>(static_cast<double>(srcWidth) / dstWidth + 0.5),
                     static_cast<int>(static_cast<double>(srcHeight) / dstHeight + 0.5)});
}

int RegenerateChunkRowsOfFactor(int srcWidth, int factor, int blockHeight) {
    int64_t rows = FirstChunkRows(factor, blockHeight);
    // Larger than the maximum: kept as it is; else doubled to get close to the maximum
    if (rows * srcWidth <= kChunkMaxSize) {
        while (rows * srcWidth < kChunkMaxSize / 2) rows *= 2;
    }
    return static_cast<int>(rows);
}

int RegenerateChunkRows(int srcWidth, int srcHeight, int dstWidth, int dstHeight, int blockHeight) {
    return RegenerateChunkRowsOfFactor(srcWidth, OverviewFactor(srcWidth, srcHeight, dstWidth, dstHeight), blockHeight);
}

int RegenerateChunkRowsOrBlocks(int srcWidth, int srcHeight, int dstWidth, int dstHeight, int blockWidth,
                                int blockHeight) {
    const int factor = OverviewFactor(srcWidth, srcHeight, dstWidth, dstHeight);
    if (FirstChunkRows(factor, blockHeight) * srcWidth > kChunkMaxSize && blockWidth < srcWidth) return 0;
    return RegenerateChunkRowsOfFactor(srcWidth, factor, blockHeight);
}

uint8_t CastNoData(double nodata) {
    if (std::isnan(nodata) || nodata <= -2147483648.0 || nodata >= 2147483648.0) return 0;
    return static_cast<uint8_t>(static_cast<int32_t>(nodata));
}

namespace {

// GDALGetNoDataReplacementValue for UInt8: 0 for a nodata value Byte cannot hold
uint8_t Replacement(double nodata) {
    if (!(nodata >= 0 && nodata <= 255)) return 0;
    return nodata == 255 ? 254 : static_cast<uint8_t>(nodata + 1);
}

uint8_t Rounded(double total, double weight) { return static_cast<uint8_t>(total / weight + 0.5); }

}  // namespace

void Average(const DownsampleSpec& spec, const uint8_t* src, size_t srcStride, int srcX0, int srcY0, int srcW,
             int srcH, const uint8_t* srcMask, const AverageTaps& cols, const AverageTaps& rows, int dstW, int dstH,
             uint8_t* dst, size_t dstStride) {
    if (spec.type != tiff::SampleType::kU8) throw std::invalid_argument("Average: 8-bit samples only");
    (void)srcH;
    const int bands = spec.bands;
    const bool hasNoData = spec.hasNoData;
    const uint8_t nodata = hasNoData ? CastNoData(spec.noData) : 0;
    const uint8_t replacement = Replacement(spec.noData);
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
                // GDALNoDataMaskBand: a nodata value Byte cannot hold masks nothing
                return !hasNoData || !(spec.noData >= 0 && spec.noData <= 255) || sample(x, y, b) != nodata;
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
                if (spec.mask == MaskMode::kNone && cols.half && rows.count[ry] == 2) {
                    // GDAL's fast path: the four samples, no replacement
                    const unsigned sum = sample(x0, y0, b) + sample(x0 + 1, y0, b) + sample(x0, y0 + 1, b) +
                                         sample(x0 + 1, y0 + 1, b);
                    out[cx * static_cast<size_t>(bands) + static_cast<size_t>(b)] = static_cast<uint8_t>((sum + 2) / 4);
                    continue;
                } else if (spec.mask == MaskMode::kNone) {
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

bool IsIdentityTable(const std::vector<ColorEntry>& entries) {
    for (size_t i = 0; i < entries.size(); ++i) {
        const ColorEntry& e = entries[i];
        if (size_t{e[0]} != i || size_t{e[1]} != i || size_t{e[2]} != i || e[3] != 255) return false;
    }
    return true;
}

PaletteTable AveragePalette(std::vector<ColorEntry> entries, std::optional<double> nodata) {
    PaletteTable t;
    t.entries = std::move(entries);
    // ReadColorTable's first transparent entry, before the nodata entry is made transparent
    int transparent = -1;
    for (size_t i = 0; i < t.entries.size() && transparent < 0; ++i)
        if (t.entries[i][3] == 0) transparent = static_cast<int>(i);
    t.noData = nodata ? CastNoData(*nodata) : 0;
    if (nodata && *nodata >= 0.0 && size_t{t.noData} < t.entries.size())
        t.entries[t.noData][3] = 0;
    else if (!nodata && transparent >= 0)
        t.noData = static_cast<uint8_t>(transparent);
    return t;
}

int BestColorEntry(const std::vector<ColorEntry>& entries, int r, int g, int b) {
    int minDist = std::numeric_limits<int>::max();
    size_t best = 0;
    for (size_t i = 0; i < entries.size(); ++i) {
        const ColorEntry& e = entries[i];
        if (e[3] == 0) continue;
        const int dist = (r - e[0]) * (r - e[0]) + (g - e[1]) * (g - e[1]) + (b - e[2]) * (b - e[2]);
        if (dist < minDist) {
            minDist = dist;
            best = i;
        }
    }
    return static_cast<int>(best);
}

void PaletteAverage(const PaletteTable& table, const uint8_t* src, size_t srcStride, int srcX0, int srcY0,
                    const AverageTaps& cols, const AverageTaps& rows, int dstW, int dstH, uint8_t* dst,
                    size_t dstStride) {
    const std::vector<ColorEntry>& entries = table.entries;
    // The entry of the last colour averaged: neighbouring pixels of a palette image often average
    // to the same colour, and BestColorEntry scans the whole table
    int lastR = -1, lastG = -1, lastB = -1;
    uint8_t lastEntry = 0;
    for (int dy = 0; dy < dstH; ++dy) {
        const size_t ry = static_cast<size_t>(dy);
        const int y0 = rows.start[ry], y1 = y0 + rows.count[ry];
        uint8_t* out = dst + static_cast<size_t>(dy) * dstStride;
        for (int dx = 0; dx < dstW; ++dx) {
            const size_t cx = static_cast<size_t>(dx);
            const int x0 = cols.start[cx], x1 = x0 + cols.count[cx];
            uint64_t totalR = 0, totalG = 0, totalB = 0;
            size_t count = 0;
            for (int y = y0; y < y1; ++y) {
                const uint8_t* line = src + static_cast<size_t>(y - srcY0) * srcStride;
                for (int x = x0; x < x1; ++x) {
                    const uint8_t v = line[x - srcX0];
                    if (size_t{v} >= entries.size()) continue;
                    const ColorEntry& e = entries[v];
                    if (e[3] == 0) continue;
                    totalR += e[0];
                    totalG += e[1];
                    totalB += e[2];
                    ++count;
                }
            }
            if (count == 0) {
                out[cx] = table.noData;
                continue;
            }
            const int r = static_cast<int>((totalR + count / 2) / count);
            const int g = static_cast<int>((totalG + count / 2) / count);
            const int b = static_cast<int>((totalB + count / 2) / count);
            if (r != lastR || g != lastG || b != lastB) {
                lastR = r;
                lastG = g;
                lastB = b;
                // GDAL stores the index in the band's 8-bit type
                lastEntry = static_cast<uint8_t>(BestColorEntry(entries, r, g, b));
            }
            out[cx] = lastEntry;
        }
    }
}

}  // namespace ddbt::cog::pyramid
