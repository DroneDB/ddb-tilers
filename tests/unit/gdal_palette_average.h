// SPDX-License-Identifier: MPL-2.0
#pragma once

// GDALBuildOverviews AVERAGE of a 1-band 8-bit palette GTiff, written the way GDAL 3.13 computes it,
// loop for loop, to hold ddb-tilers' palette average to (pyramid/average.h PaletteAverage):
//   - the colour table GTiff reads back from the file (TIFFColorMapTagToColorTable: the ColorMap of
//     2^8 entries, the missing ones black, every entry opaque but the one of the nodata value);
//   - GDALRegenerateOverviewsEx: the rows of a level read in chunks (UpdateChunkHeightAndGetChunkSize,
//     GDAL_OVR_CHUNK_MAX_SIZE 10 MiB), the destination rows of each chunk, each level from the one
//     above (GDALRegenerateCascadingOverviews), the overviews tiled as GTIFFGetOverviewBlockSize says;
//   - GDALResampleChunk_AverageOrRMS_T's palette branch and BestColorEntry (gcore/overview.cpp).
// GDAL's types are kept (GDALColorEntry's shorts, the int distances, the 64-bit totals).

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace ddbt::test {

struct GdalColorEntry {
    short c1 = 0, c2 = 0, c3 = 0, c4 = 0;
};

// static_cast<GByte> of a double as x86-64 makes it (cvttsd2si to 32 bits, the low byte kept)
inline uint8_t GdalByteOfDouble(double v) {
    if (!(v > -2147483649.0 && v < 2147483648.0)) return 0;
    return static_cast<uint8_t>(static_cast<int32_t>(v));
}

// The colour table GTiff gives band 1 of a file whose ColorMap GDAL wrote from `rgb` (each value
// times 257, read back divided by 257), with a nodata value or not
inline std::vector<GdalColorEntry> GtiffReadBackTable(const std::vector<std::array<uint8_t, 3>>& rgb, bool bNoDataSet,
                                                      double dfNoDataValue) {
    const int nColorCount = 1 << 8;
    std::vector<GdalColorEntry> table(static_cast<size_t>(nColorCount));
    for (int iColor = nColorCount - 1; iColor >= 0; iColor--) {
        const size_t i = static_cast<size_t>(iColor);
        const unsigned short r = i < rgb.size() ? static_cast<unsigned short>(rgb[i][0] * 257) : 0;
        const unsigned short g = i < rgb.size() ? static_cast<unsigned short>(rgb[i][1] * 257) : 0;
        const unsigned short b = i < rgb.size() ? static_cast<unsigned short>(rgb[i][2] * 257) : 0;
        const bool transparent = bNoDataSet && std::isfinite(dfNoDataValue) && std::fabs(dfNoDataValue) < 2147483648.0 &&
                                 static_cast<int>(dfNoDataValue) == iColor;
        table[i] = {static_cast<short>(r / 257), static_cast<short>(g / 257), static_cast<short>(b / 257),
                    static_cast<short>(transparent ? 0 : 255)};
    }
    return table;
}

inline int GdalBestColorEntry(const std::vector<GdalColorEntry>& entries, const GdalColorEntry& test) {
    int nMinDist = std::numeric_limits<int>::max();
    size_t bestEntry = 0;
    for (size_t i = 0; i < entries.size(); ++i) {
        const GdalColorEntry& entry = entries[i];
        if (entry.c4 == 0) continue;
        const int nDist = ((test.c1 - entry.c1) * (test.c1 - entry.c1)) + ((test.c2 - entry.c2) * (test.c2 - entry.c2)) +
                          ((test.c3 - entry.c3) * (test.c3 - entry.c3));
        if (nDist < nMinDist) {
            nMinDist = nDist;
            bestEntry = i;
        }
    }
    return static_cast<int>(bestEntry);
}

// GDALResampleChunk_AverageOrRMS_T for GByte with a colour table: the destination rows
// [nDstYOff, nDstYOff2) of the chunk [nChunkYOff, + nChunkYSize) (pChunk: its rows, nChunkXSize wide)
inline void GdalAveragePaletteChunk(const uint8_t* pChunk, int nChunkXSize, int nChunkYOff, int nChunkYSize,
                                    int nDstXWidth, int nDstYOff, int nDstYOff2, double dfXRatioDstToSrc,
                                    double dfYRatioDstToSrc, const std::vector<GdalColorEntry>& table, bool bHasNoData,
                                    double dfNoDataValue, uint8_t* pDstBuffer) {
    const int nChunkXOff = 0;
    const double dfSrcXDelta = 0, dfSrcYDelta = 0;
    uint8_t tNoDataValue = !bHasNoData ? 0 : GdalByteOfDouble(dfNoDataValue);
    const int nChunkRightXOff = nChunkXOff + nChunkXSize;
    const int nChunkBottomYOff = nChunkYOff + nChunkYSize;

    struct PrecomputedXValue {
        int nLeftXOffShifted, nRightXOffShifted;
    };
    std::vector<PrecomputedXValue> pasSrcX(static_cast<size_t>(nDstXWidth));

    int nTransparentIdx = -1;
    std::vector<GdalColorEntry> colorEntries = table;
    for (size_t i = 0; i < colorEntries.size(); ++i)
        if (nTransparentIdx < 0 && colorEntries[i].c4 == 0) nTransparentIdx = static_cast<int>(i);
    if (bHasNoData && dfNoDataValue >= 0.0 && tNoDataValue < colorEntries.size())
        colorEntries[static_cast<int>(tNoDataValue)].c4 = 0;
    else if (!bHasNoData && nTransparentIdx >= 0) {
        bHasNoData = true;
        tNoDataValue = static_cast<uint8_t>(nTransparentIdx);
    }

    for (int iDstPixel = 0; iDstPixel < nDstXWidth; ++iDstPixel) {
        const double dfSrcXOff = dfSrcXDelta + iDstPixel * dfXRatioDstToSrc;
        int nSrcXOff = static_cast<int>(dfSrcXOff + 1e-8);
        const double dfSrcXOff2 = dfSrcXDelta + (iDstPixel + 1) * dfXRatioDstToSrc;
        int nSrcXOff2 = static_cast<int>(std::ceil(dfSrcXOff2 - 1e-8));
        if (nSrcXOff < nChunkXOff) nSrcXOff = nChunkXOff;
        if (nSrcXOff2 == nSrcXOff) nSrcXOff2++;
        if (nSrcXOff2 > nChunkRightXOff) nSrcXOff2 = nChunkRightXOff;
        pasSrcX[static_cast<size_t>(iDstPixel)] = {nSrcXOff - nChunkXOff, nSrcXOff2 - nChunkXOff};
    }

    for (int iDstLine = nDstYOff; iDstLine < nDstYOff2; ++iDstLine) {
        const double dfSrcYOff = dfSrcYDelta + iDstLine * dfYRatioDstToSrc;
        int nSrcYOff = static_cast<int>(dfSrcYOff + 1e-8);
        if (nSrcYOff < nChunkYOff) nSrcYOff = nChunkYOff;
        const double dfSrcYOff2 = dfSrcYDelta + (iDstLine + 1) * dfYRatioDstToSrc;
        int nSrcYOff2 = static_cast<int>(std::ceil(dfSrcYOff2 - 1e-8));
        if (nSrcYOff2 == nSrcYOff) ++nSrcYOff2;
        if (nSrcYOff2 > nChunkBottomYOff) nSrcYOff2 = nChunkBottomYOff;
        uint8_t* const pDstScanline = pDstBuffer + static_cast<size_t>(iDstLine - nDstYOff) * static_cast<size_t>(nDstXWidth);
        nSrcYOff -= nChunkYOff;
        nSrcYOff2 -= nChunkYOff;
        for (int iDstPixel = 0; iDstPixel < nDstXWidth; ++iDstPixel) {
            const int nSrcXOff = pasSrcX[static_cast<size_t>(iDstPixel)].nLeftXOffShifted;
            const int nSrcXOff2 = pasSrcX[static_cast<size_t>(iDstPixel)].nRightXOffShifted;
            uint64_t nTotalR = 0, nTotalG = 0, nTotalB = 0;
            size_t nCount = 0;
            for (int iY = nSrcYOff; iY < nSrcYOff2; ++iY) {
                for (int iX = nSrcXOff; iX < nSrcXOff2; ++iX) {
                    const uint8_t val = pChunk[static_cast<size_t>(iX) + static_cast<size_t>(iY) * static_cast<size_t>(nChunkXSize)];
                    if (val >= colorEntries.size()) continue;
                    const auto& entry = colorEntries[val];
                    if (entry.c4) {
                        nTotalR += static_cast<uint64_t>(entry.c1);
                        nTotalG += static_cast<uint64_t>(entry.c2);
                        nTotalB += static_cast<uint64_t>(entry.c3);
                        ++nCount;
                    }
                }
            }
            if (nCount == 0) {
                pDstScanline[iDstPixel] = tNoDataValue;
            } else {
                GdalColorEntry color;
                color.c1 = static_cast<short>((nTotalR + nCount / 2) / nCount);
                color.c2 = static_cast<short>((nTotalG + nCount / 2) / nCount);
                color.c3 = static_cast<short>((nTotalB + nCount / 2) / nCount);
                pDstScanline[iDstPixel] = static_cast<uint8_t>(GdalBestColorEntry(colorEntries, color));
            }
        }
    }
}

// GDALRegenerateOverviewsEx of one overview (nDstWidth x nDstHeight) of a band of nWidth x nHeight
// samples whose blocks are nFRYBlockSize rows high
inline std::vector<uint8_t> GdalPaletteOverview(const std::vector<uint8_t>& src, int nWidth, int nHeight, int nDstWidth,
                                                int nDstHeight, int nFRYBlockSize,
                                                const std::vector<GdalColorEntry>& table, bool bHasNoData,
                                                double dfNoDataValue) {
    const int nMaxOvrFactor =
        std::max({1, static_cast<int>(static_cast<double>(nWidth) / nDstWidth + 0.5),
                  static_cast<int>(static_cast<double>(nHeight) / nDstHeight + 0.5)});
    int nFullResYChunk = nFRYBlockSize;
    const auto UpdateChunkHeightAndGetChunkSize = [&]() {
        nFullResYChunk = std::max(nFullResYChunk, 2 * nMaxOvrFactor);
        return static_cast<int64_t>(nFullResYChunk) * nWidth;
    };
    const int nChunkMaxSize = 10485760;
    int64_t nChunkSize = UpdateChunkHeightAndGetChunkSize();
    if (nChunkSize <= nChunkMaxSize) {
        while (nChunkSize < nChunkMaxSize / 2) {
            nFullResYChunk *= 2;
            nChunkSize = UpdateChunkHeightAndGetChunkSize();
        }
    }
    std::vector<uint8_t> dst(static_cast<size_t>(nDstWidth) * static_cast<size_t>(nDstHeight));
    for (int nChunkYOff = 0; nChunkYOff < nHeight; nChunkYOff += nFullResYChunk) {
        if (nFullResYChunk + nChunkYOff > nHeight) nFullResYChunk = nHeight - nChunkYOff;
        const double dfXRatioDstToSrc = static_cast<double>(nWidth) / nDstWidth;
        const double dfYRatioDstToSrc = static_cast<double>(nHeight) / nDstHeight;
        const int nDstYOff = static_cast<int>(0.5 + nChunkYOff / dfYRatioDstToSrc);
        if (nDstYOff == nDstHeight) continue;
        int nDstYOff2 = static_cast<int>(0.5 + (nChunkYOff + nFullResYChunk) / dfYRatioDstToSrc);
        if (nChunkYOff + nFullResYChunk == nHeight) nDstYOff2 = nDstHeight;
        GdalAveragePaletteChunk(src.data() + static_cast<size_t>(nChunkYOff) * static_cast<size_t>(nWidth), nWidth,
                                nChunkYOff, nFullResYChunk, nDstWidth, nDstYOff, nDstYOff2, dfXRatioDstToSrc,
                                dfYRatioDstToSrc, table, bHasNoData, dfNoDataValue,
                                dst.data() + static_cast<size_t>(nDstYOff) * static_cast<size_t>(nDstWidth));
    }
    return dst;
}

// GDALBuildOverviews AVERAGE of the palette band of a tiled GTiff (square blocks of blockSize):
// every level of `factors` (ceil(size / factor)), each from the one above
inline std::vector<std::vector<uint8_t>> GdalPaletteOverviews(const std::vector<uint8_t>& src, int width, int height,
                                                              int blockSize, const std::vector<int>& factors,
                                                              const std::vector<GdalColorEntry>& table, bool bHasNoData,
                                                              double dfNoDataValue) {
    // GTIFFGetOverviewBlockSize
    const bool keep = blockSize >= 64 && blockSize <= 4096 && (blockSize & (blockSize - 1)) == 0;
    const int ovrBlockSize = keep ? blockSize : 128;
    std::vector<std::vector<uint8_t>> levels;
    levels.reserve(factors.size());  // `above` points into it
    const std::vector<uint8_t>* above = &src;
    int w = width, h = height, block = blockSize;
    for (const int f : factors) {
        const int dw = (width + f - 1) / f, dh = (height + f - 1) / f;
        levels.push_back(GdalPaletteOverview(*above, w, h, dw, dh, block, table, bHasNoData, dfNoDataValue));
        above = &levels.back();
        w = dw;
        h = dh;
        block = ovrBlockSize;
    }
    return levels;
}

}  // namespace ddbt::test
