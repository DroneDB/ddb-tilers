// SPDX-License-Identifier: MPL-2.0
//
// LERC (with GDAL's DEFLATE / ZSTD layering) and WebP block decoders, following libtiff's
// tif_lerc.c and tif_webp.c so that the decoded bytes are those GDAL returns.

#include <Lerc_c_api.h>
#include <libdeflate.h>
#include <webp/decode.h>
#include <zstd.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "cog/codecs/codecs.h"
#include "core/util/checked_math.h"

namespace ddbt::cog::codecs {

namespace {

// libtiff's GetLercDataType: -1 for combinations LERC in TIFF does not define.
int LercDataType(uint16_t sampleFormat, uint32_t bits) {
    if (sampleFormat == 1 && bits == 8) return 1;
    if (sampleFormat == 2 && bits == 16) return 2;
    if (sampleFormat == 1 && bits == 16) return 3;
    if (sampleFormat == 2 && bits == 32) return 4;
    if (sampleFormat == 1 && bits == 32) return 5;
    if (sampleFormat == 3 && bits == 32) return 6;
    if (sampleFormat == 3 && bits == 64) return 7;
    return -1;
}

struct Scratch {
    std::vector<uint8_t> inflated;
    std::vector<uint8_t> mask;
    std::vector<uint8_t> multiband;
};

thread_local Scratch tlsLerc;

}  // namespace

Status DecodeLerc(std::span<const uint8_t> in, const LercBlockInfo& info, std::span<uint8_t> out) {
    const int dataType = LercDataType(info.sampleFormat, info.bitsPerSample);
    if (dataType < 0) return Unsupported("LERC: unsupported combination of SampleFormat and BitsPerSample");
    if (info.width == 0 || info.height == 0 || info.samples == 0) return Corrupt("LERC: empty block");
    const uint64_t bytesPerSample = info.bitsPerSample / 8;
    uint64_t pixels = 0, expected = 0;
    if (!CheckedMul<uint64_t>(info.width, info.height, &pixels) ||
        !CheckedMul<uint64_t>(pixels, info.samples * bytesPerSample, &expected) || expected != out.size()) {
        return InvalidArgument("LERC: output size does not match the block");
    }
    if (expected > UINT32_MAX / 2) return Unsupported("LERC: block too large");

    Scratch& s = tlsLerc;
    const uint8_t* blob = in.data();
    size_t blobSize = in.size();
    if (info.additionalCompression != 0) {
        // libtiff's buffer: 256 + size + size / 3, at least ZSTD_compressBound(size).
        size_t cap = static_cast<size_t>(256 + expected + expected / 3);
        cap = std::max(cap, ZSTD_compressBound(static_cast<size_t>(expected)));
        if (s.inflated.size() < cap) s.inflated.resize(cap);
        if (info.additionalCompression == 1) {
            thread_local std::unique_ptr<libdeflate_decompressor, void (*)(libdeflate_decompressor*)> d(
                libdeflate_alloc_decompressor(), libdeflate_free_decompressor);
            if (!d) throw std::bad_alloc();
            size_t actual = 0;
            if (libdeflate_zlib_decompress(d.get(), in.data(), in.size(), s.inflated.data(), cap, &actual) !=
                LIBDEFLATE_SUCCESS) {
                return Corrupt("LERC: DEFLATE layer: invalid compressed data");
            }
            blobSize = actual;
        } else if (info.additionalCompression == 2) {
            const size_t r = ZSTD_decompress(s.inflated.data(), cap, in.data(), in.size());
            if (ZSTD_isError(r)) return Corrupt(std::string("LERC: ZSTD layer: ") + ZSTD_getErrorName(r));
            blobSize = r;
        } else {
            return Unsupported("LERC: unknown additional compression " + std::to_string(info.additionalCompression));
        }
        blob = s.inflated.data();
    }
    if (blobSize > UINT32_MAX) return Corrupt("LERC: blob too large");

    unsigned int infoArray[9] = {};
    if (lerc_getBlobInfo(blob, static_cast<unsigned>(blobSize), infoArray, nullptr, 9, 0) != 0) {
        return Corrupt("LERC: lerc_getBlobInfo failed");
    }
    const uint32_t spp = info.samples;
    uint32_t nomaskBands = spp;
    bool useMask = false;
    if (spp > 1 && info.lastSampleIsUnassociatedAlpha && dataType == 1 && infoArray[2] == spp - 1) {
        useMask = true;
        --nomaskBands;
    } else if (info.sampleFormat == 3) {
        useMask = true;
    }
    const uint32_t ndims = nomaskBands;
    if (infoArray[1] != static_cast<unsigned>(dataType)) return Corrupt("LERC: unexpected data type");
    const unsigned foundDims = infoArray[2];
    const bool floatMulti = info.sampleFormat == 3 && spp > 1;
    if (floatMulti) {
        if (foundDims != 1 && foundDims != ndims) return Corrupt("LERC: unexpected nDim");
    } else if (foundDims != ndims) {
        return Corrupt("LERC: unexpected nDim");
    }
    if (infoArray[3] != info.width || infoArray[4] != info.height) return Corrupt("LERC: unexpected block size");
    const unsigned foundBands = infoArray[5];
    if (floatMulti && foundDims == 1) {
        if (foundBands != spp) return Corrupt("LERC: unexpected nBands");
    } else if (foundBands != 1) {
        return Corrupt("LERC: unexpected nBands");
    }
    if (infoArray[7] != blobSize) return Corrupt("LERC: unexpected blob size");
    int requestedMasks = useMask ? 1 : 0;
    const int foundMasks = static_cast<int>(infoArray[8]);
    if (floatMulti && foundDims == 1) {
        if (foundMasks != 0 && foundMasks != static_cast<int>(spp)) return Corrupt("LERC: unexpected mask count");
        requestedMasks = foundMasks;
    } else if (foundMasks != 0 && foundMasks != 1) {
        return Corrupt("LERC: unexpected mask count");
    }
    if (info.sampleFormat == 3 && foundMasks == 0) {
        requestedMasks = 0;
        useMask = false;
    }

    const size_t npix = static_cast<size_t>(pixels);
    if (useMask && s.mask.size() < npix * static_cast<size_t>(std::max(requestedMasks, 1))) {
        s.mask.resize(npix * static_cast<size_t>(std::max(requestedMasks, 1)));
    }
    lerc_status rc;
    if (requestedMasks > 1) {
        if (s.multiband.size() < expected) s.multiband.resize(static_cast<size_t>(expected));
        rc = lerc_decode(blob, static_cast<unsigned>(blobSize), requestedMasks, s.mask.data(), static_cast<int>(foundDims),
                         static_cast<int>(info.width), static_cast<int>(info.height), static_cast<int>(foundBands),
                         static_cast<unsigned>(dataType), s.multiband.data());
    } else {
        rc = lerc_decode(blob, static_cast<unsigned>(blobSize), requestedMasks, useMask ? s.mask.data() : nullptr,
                         static_cast<int>(foundDims), static_cast<int>(info.width), static_cast<int>(info.height),
                         static_cast<int>(foundBands), static_cast<unsigned>(dataType), out.data());
    }
    if (rc != 0) return Corrupt("LERC: lerc_decode failed");

    uint8_t* const o = out.data();
    if (useMask && dataType == 1) {
        // Interleave the mask as the alpha sample, from the end so that it moves in place.
        const size_t srcStride = spp - 1;
        const size_t dstStride = spp;
        for (size_t i = npix; i-- > 0;) {
            std::memmove(o + i * dstStride, o + i * srcStride, srcStride);
            o[i * dstStride + spp - 1] = static_cast<uint8_t>(255 * s.mask[i]);
        }
    } else if (useMask && info.sampleFormat == 3) {
        const float nan32 = std::numeric_limits<float>::quiet_NaN();
        const double nan64 = static_cast<double>(nan32);
        auto setNan = [&](size_t k) {
            if (info.bitsPerSample == 32) std::memcpy(o + k * 4, &nan32, 4);
            else std::memcpy(o + k * 8, &nan64, 8);
        };
        if (spp == 1) {
            for (size_t i = 0; i < npix; ++i)
                if (s.mask[i] == 0) setNan(i);
        } else if (requestedMasks == 1) {
            size_t k = 0;
            for (size_t i = 0; i < npix; ++i)
                for (uint32_t j = 0; j < spp; ++j, ++k)
                    if (s.mask[i] == 0) setNan(k);
        } else {
            // One mask per band, band-sequential data: interleave.
            const size_t bps = info.bitsPerSample / 8;
            size_t k = 0;
            for (size_t i = 0; i < npix; ++i) {
                for (uint32_t j = 0; j < spp; ++j, ++k) {
                    if (s.mask[i + j * npix] == 0) setNan(k);
                    else std::memcpy(o + k * bps, s.multiband.data() + (i + j * npix) * bps, bps);
                }
            }
        }
    }
    return Status::Ok();
}

Status DecodeWebp(std::span<const uint8_t> in, uint32_t width, uint32_t height, uint32_t samples,
                  std::span<uint8_t> out) {
    if (samples != 3 && samples != 4) return Unsupported("WebP: only 3 or 4 samples per pixel");
    int w = 0, h = 0;
    if (!WebPGetInfo(in.data(), in.size(), &w, &h)) return Corrupt("WebP: invalid bitstream header");
    if (static_cast<uint32_t>(w) != width || static_cast<uint32_t>(h) != height) {
        return Corrupt("WebP: image is " + std::to_string(w) + "x" + std::to_string(h) + ", block is " +
                       std::to_string(width) + "x" + std::to_string(height));
    }
    const size_t stride = static_cast<size_t>(width) * samples;
    if (out.size() < stride * height) return InvalidArgument("WebP: output buffer too small");
    const int istride = static_cast<int>(stride);
    const uint8_t* r = samples == 4 ? WebPDecodeRGBAInto(in.data(), in.size(), out.data(), out.size(), istride)
                                    : WebPDecodeRGBInto(in.data(), in.size(), out.data(), out.size(), istride);
    if (r == nullptr) return Corrupt("WebP: decoding failed");
    return Status::Ok();
}

}  // namespace ddbt::cog::codecs
