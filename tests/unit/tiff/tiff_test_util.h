// SPDX-License-Identifier: MPL-2.0
//
// Helpers of the TIFF reader tests: a window read assembled from decoded blocks (what GDAL's
// band ReadRaster returns), CRC-32 / Adler-32 as Python's zlib computes them, and a minimal
// TIFF writer for synthetic and malformed files.
#pragma once

#include <libdeflate.h>

#include <algorithm>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "cog/tiff/tiff_reader.h"
#include "core/util/endian.h"

namespace ddbt::cog::tiff::testing {

inline uint32_t Crc32(const std::vector<uint8_t>& v) {
    return libdeflate_crc32(0, v.data(), v.size());
}
inline uint32_t Adler32(const std::vector<uint8_t>& v) {
    return libdeflate_adler32(1, v.data(), v.size());
}

/// Reads samples `bands` (0-based sample indexes) of the window [x0, x0 + w) x [y0, y0 + h)
/// of `ifd`, each as a packed array of the sample type (1-bit samples expanded to 0 / `bitOne`),
/// i.e. what GDAL's band.ReadRaster returns. Blocks are decoded once for all bands.
inline Expected<std::vector<std::vector<uint8_t>>> ReadWindow(const TiffReader& r, uint32_t ifd,
                                                             const std::vector<uint32_t>& bands, uint32_t x0,
                                                             uint32_t y0, uint32_t w, uint32_t h, uint8_t bitOne) {
    const IfdInfo& info = r.Ifd(ifd);
    const bool bit = info.sampleType == SampleType::kBit1;
    // A complex band is its two components (GDAL's CInt16 ... CFloat64 values)
    const size_t comps = info.Components();
    const size_t ss = bit ? 1 : SampleTypeSize(info.sampleType) * comps;
    std::vector<std::vector<uint8_t>> out(bands.size(), std::vector<uint8_t>(size_t{w} * h * ss));
    const uint32_t bx0 = x0 / info.blockWidth, bx1 = (x0 + w - 1) / info.blockWidth;
    const uint32_t by0 = y0 / info.blockHeight, by1 = (y0 + h - 1) / info.blockHeight;
    std::vector<uint8_t> expanded;
    for (uint32_t by = by0; by <= by1; ++by) {
        for (uint32_t bx = bx0; bx <= bx1; ++bx) {
            // One decode per plane needed (1 for PIXEL interleave).
            std::map<uint32_t, DecodedBlock> planes;
            for (size_t bi = 0; bi < bands.size(); ++bi) {
                const uint32_t plane = info.IsPixelInterleaved() ? 0 : bands[bi];
                auto it = planes.find(plane);
                if (it == planes.end()) {
                    auto blk = r.ReadBlock(ifd, info.BlockIndex(bx, by, plane));
                    if (!blk.ok()) return std::move(blk).TakeStatus();
                    it = planes.emplace(plane, std::move(*blk)).first;
                }
                const DecodedBlock& b = it->second;
                const uint32_t spp = b.samples / static_cast<uint32_t>(comps);
                const uint32_t sample = info.IsPixelInterleaved() ? bands[bi] : 0;
                const uint8_t* src = b.data();
                size_t srcStride = b.rowStride;
                if (bit) {
                    expanded.resize(size_t{b.width} * b.height);
                    ExpandBits(b.data(), b.rowStride, b.width, b.height, bitOne, expanded.data(), b.width);
                    src = expanded.data();
                    srcStride = b.width;
                }
                const uint32_t bxStart = bx * info.blockWidth, byStart = by * info.blockHeight;
                const uint32_t cx0 = std::max(x0, bxStart), cx1 = std::min(x0 + w, bxStart + b.width);
                const uint32_t cy0 = std::max(y0, byStart), cy1 = std::min(y0 + h, byStart + b.height);
                for (uint32_t y = cy0; y < cy1; ++y) {
                    const uint8_t* row = src + size_t{y - byStart} * srcStride;
                    uint8_t* dst = out[bi].data() + (size_t{y - y0} * w + (cx0 - x0)) * ss;
                    for (uint32_t x = cx0; x < cx1; ++x) {
                        std::memcpy(dst, row + (size_t{x - bxStart} * spp + sample) * ss, ss);
                        dst += ss;
                    }
                }
            }
        }
    }
    return out;
}

/// Minimal TIFF writer: one or more IFDs with arbitrary entries (for malformed-file tests).
class TiffBuilder {
public:
    TiffBuilder(bool bigTiff, bool bigEndian) : big_(bigTiff), be_(bigEndian), linkPos_(bigTiff ? 8 : 4) {
        bytes_.push_back(be_ ? 'M' : 'I');
        bytes_.push_back(be_ ? 'M' : 'I');
        Put16(big_ ? 43 : 42);
        if (big_) {
            Put16(8);
            Put16(0);
            Put64(0);
        } else {
            Put32(0);
        }
    }

    struct Entry {
        uint16_t tag;
        uint16_t type;
        uint64_t count;
        std::vector<uint8_t> data;  // values, file byte order
    };

    template <typename T>
    std::vector<uint8_t> Values(const std::vector<T>& v) const {
        std::vector<uint8_t> out(v.size() * sizeof(T));
        for (size_t i = 0; i < v.size(); ++i) StoreEndian<T>(out.data() + i * sizeof(T), v[i], be_);
        return out;
    }
    Entry Short(uint16_t tag, std::vector<uint16_t> v) { return {tag, 3, v.size(), Values(v)}; }
    Entry Long(uint16_t tag, std::vector<uint32_t> v) { return {tag, 4, v.size(), Values(v)}; }
    Entry Long8(uint16_t tag, std::vector<uint64_t> v) { return {tag, 16, v.size(), Values(v)}; }
    Entry Double(uint16_t tag, std::vector<double> v) { return {tag, 12, v.size(), Values(v)}; }
    Entry Ascii(uint16_t tag, const std::string& s) {
        std::vector<uint8_t> d(s.begin(), s.end());
        d.push_back(0);
        return {tag, 2, d.size(), d};
    }
    Entry Bytes(uint16_t tag, uint16_t type, std::vector<uint8_t> d) { return {tag, type, d.size(), std::move(d)}; }

    /// Appends raw data (block contents); returns its offset.
    uint64_t AddData(const std::vector<uint8_t>& d) {
        Align();
        const uint64_t off = bytes_.size();
        bytes_.insert(bytes_.end(), d.begin(), d.end());
        return off;
    }

    /// Appends GDAL's ghost header (call first, right after construction).
    void AddGhost(const std::string& text) {
        char head[64];
        std::snprintf(head, sizeof(head), "GDAL_STRUCTURAL_METADATA_SIZE=%06zu bytes\n", text.size());
        bytes_.insert(bytes_.end(), head, head + std::strlen(head));
        bytes_.insert(bytes_.end(), text.begin(), text.end());
    }

    /// Appends an IFD; links it from the previous linked one (or the header) unless `link` is
    /// false (SubIFDs). Returns its offset.
    uint64_t AddIfd(std::vector<Entry> entries, bool link = true) {
        std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) { return a.tag < b.tag; });
        // Out-of-line values first.
        const size_t field = big_ ? 8 : 4;
        std::vector<uint64_t> offsets(entries.size(), 0);
        for (size_t i = 0; i < entries.size(); ++i) {
            if (entries[i].data.size() > field) offsets[i] = AddData(entries[i].data);
        }
        Align();
        const uint64_t ifd = bytes_.size();
        if (big_) Put64(entries.size());
        else Put16(static_cast<uint16_t>(entries.size()));
        for (size_t i = 0; i < entries.size(); ++i) {
            const Entry& e = entries[i];
            Put16(e.tag);
            Put16(e.type);
            if (big_) Put64(e.count);
            else Put32(static_cast<uint32_t>(e.count));
            if (e.data.size() > field) {
                if (big_) Put64(offsets[i]);
                else Put32(static_cast<uint32_t>(offsets[i]));
            } else {
                std::vector<uint8_t> f(field, 0);
                std::copy(e.data.begin(), e.data.end(), f.begin());
                bytes_.insert(bytes_.end(), f.begin(), f.end());
            }
        }
        const uint64_t nextPos = bytes_.size();
        if (big_) Put64(0);
        else Put32(0);
        if (link) {
            if (big_) StoreEndian<uint64_t>(bytes_.data() + linkPos_, ifd, be_);
            else StoreEndian<uint32_t>(bytes_.data() + linkPos_, static_cast<uint32_t>(ifd), be_);
            linkPos_ = nextPos;
        }
        return ifd;
    }

    std::vector<uint8_t>& bytes() { return bytes_; }
    bool big() const { return big_; }
    bool be() const { return be_; }

private:
    void Align() {
        while (bytes_.size() % 2) bytes_.push_back(0);
    }
    void Put16(uint16_t v) {
        uint8_t b[2];
        StoreEndian<uint16_t>(b, v, be_);
        bytes_.insert(bytes_.end(), b, b + 2);
    }
    void Put32(uint32_t v) {
        uint8_t b[4];
        StoreEndian<uint32_t>(b, v, be_);
        bytes_.insert(bytes_.end(), b, b + 4);
    }
    void Put64(uint64_t v) {
        uint8_t b[8];
        StoreEndian<uint64_t>(b, v, be_);
        bytes_.insert(bytes_.end(), b, b + 8);
    }

    bool big_;
    bool be_;
    std::vector<uint8_t> bytes_;
    uint64_t linkPos_;  // where the offset of the next IFD goes (header field first)
};

}  // namespace ddbt::cog::tiff::testing
