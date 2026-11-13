// SPDX-License-Identifier: MPL-2.0
//
// Synthetic LAS / LAZ files for the builder and reader tests: any point format 0-10, extra
// bytes, LAS 1.2 or 1.4, uncompressed or LAZ (formats 0-3 and 6-8, fixed-size chunks, the
// chunk table offset optionally stored as -1 with the real one at the end of the file).
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "copc/las/point_format.h"
#include "copc/laz/chunk_decoder.h"
#include "core/util/endian.h"

namespace ddbt::copc::testing {

struct SynthLas {
    int minor = 2;
    int format = 3;
    uint16_t extraBytes = 0;
    bool laz = false;
    bool chunkTableMinusOne = false;
    uint32_t chunkSize = 5000;
    std::array<double, 3> scale{0.01, 0.01, 0.01};
    std::array<double, 3> offset{500000, 4000000, 0};
    std::vector<uint8_t> records;  // count * RecordSize()
    uint64_t count = 0;
    bool overrideBounds = false;
    std::array<double, 3> min{}, max{};
    std::string wkt;
    std::vector<uint8_t> extraBytesVlr;

    size_t RecordSize() const { return las::BaseRecordSize(format) + extraBytes; }
};

/// Random records of `s.format` (every field varies; 16-bit colour when color16), points in a
/// box of `extent` units of the scale around the offset, clustered so the octree has depth.
inline void FillRandom(SynthLas& s, uint64_t count, uint32_t seed, int32_t extent = 200000, bool color16 = true) {
    std::mt19937_64 rng(seed);
    const las::PointLayout L = las::LayoutOf(s.format);
    const size_t rs = s.RecordSize();
    s.count = count;
    s.records.assign(count * rs, 0);
    std::uniform_int_distribution<int32_t> coord(-extent, extent);
    std::normal_distribution<double> blob(0.0, extent / 20.0);
    std::array<std::array<int32_t, 3>, 8> centres;
    for (auto& c : centres)
        for (auto& v : c) v = coord(rng) / 2;
    double gps = 1000.0;
    uint64_t waveOffset = 1u << 20;
    uint32_t waveSize = 120;
    for (uint64_t i = 0; i < count; ++i) {
        uint8_t* r = s.records.data() + i * rs;
        int32_t xyz[3];
        if (i % 3 == 0) {
            for (int a = 0; a < 3; ++a) xyz[a] = coord(rng);
        } else {
            const auto& c = centres[rng() % 8];
            for (int a = 0; a < 3; ++a)
                xyz[a] = std::clamp<int32_t>(c[static_cast<size_t>(a)] + static_cast<int32_t>(blob(rng)), -extent, extent);
        }
        if (i % 97 == 5 && i > 0) std::memcpy(xyz, s.records.data() + (i - 1) * rs, 12);  // duplicates
        for (int a = 0; a < 3; ++a) StoreLE<int32_t>(r + 4 * a, xyz[a]);
        StoreLE<uint16_t>(r + 12, static_cast<uint16_t>(rng()));
        if (L.extended) {
            const uint8_t nr = static_cast<uint8_t>(1 + rng() % 15);
            const uint8_t rn = static_cast<uint8_t>(1 + rng() % nr);
            r[14] = static_cast<uint8_t>(rn | (nr << 4));
            r[15] = static_cast<uint8_t>(rng());
            r[16] = static_cast<uint8_t>(rng() % 64);
            r[17] = static_cast<uint8_t>(rng());
            StoreLE<int16_t>(r + 18, static_cast<int16_t>(static_cast<int32_t>(rng() % 30001) - 15000));
            StoreLE<uint16_t>(r + 20, static_cast<uint16_t>(rng() % 7));
        } else {
            const uint8_t nr = static_cast<uint8_t>(1 + rng() % 7);
            const uint8_t rn = static_cast<uint8_t>(1 + rng() % nr);
            r[14] = static_cast<uint8_t>(rn | (nr << 3) | ((rng() & 1) << 6) | ((rng() & 1) << 7));
            const uint8_t cls = static_cast<uint8_t>(rng() % 20);  // 12 included (legacy overlap)
            r[15] = static_cast<uint8_t>(cls | ((rng() % 8) << 5));
            r[16] = static_cast<uint8_t>(static_cast<int8_t>(static_cast<int32_t>(rng() % 181) - 90));
            r[17] = static_cast<uint8_t>(rng());
            StoreLE<uint16_t>(r + 18, static_cast<uint16_t>(rng() % 7));
        }
        if (L.HasGpsTime()) {
            gps += (rng() % 10 == 0) ? 0.5 : 0.0;
            StoreLE<double>(r + L.gpsTime, gps);
        }
        if (L.HasRgb())
            for (int c = 0; c < 3; ++c)
                StoreLE<uint16_t>(r + L.rgb + 2 * c, static_cast<uint16_t>(color16 ? rng() : (rng() % 256)));
        if (L.HasNir()) StoreLE<uint16_t>(r + L.nir, static_cast<uint16_t>(rng()));
        if (L.HasWavePacket()) {
            // Offsets that take every path of the WAVEPACKET13 coder: the same, contiguous, a
            // 32-bit difference, a 64-bit jump.
            uint8_t* w = r + L.wavePacket;
            w[0] = static_cast<uint8_t>(rng() % 3);
            const int pick = static_cast<int>(rng() % 4);
            if (pick == 1) waveOffset += waveSize;
            else if (pick == 2) waveOffset += rng() % 100000;
            else if (pick == 3) waveOffset = rng();
            waveSize = static_cast<uint32_t>(100 + rng() % 50);
            StoreLE<uint64_t>(w + 1, waveOffset);
            StoreLE<uint32_t>(w + 9, waveSize);
            StoreLE<float>(w + 13, static_cast<float>(rng() % 1000) / 7.0f);
            for (int k = 0; k < 3; ++k) StoreLE<float>(w + 17 + 4 * k, static_cast<float>(static_cast<int>(rng() % 2001) - 1000) * 1e-4f);
        }
        for (size_t k = L.baseSize; k < rs; ++k) r[k] = static_cast<uint8_t>(rng() % 251);
    }
}

namespace detail {
inline void Put(std::vector<uint8_t>& b, const void* p, size_t n) {
    const uint8_t* c = static_cast<const uint8_t*>(p);
    b.insert(b.end(), c, c + n);
}
template <typename T>
void PutLE(std::vector<uint8_t>& b, T v) {
    uint8_t t[sizeof(T)];
    StoreLE<T>(t, v);
    Put(b, t, sizeof(T));
}
inline void PutStr(std::vector<uint8_t>& b, const std::string& s, size_t w) {
    std::vector<uint8_t> z(w, 0);
    std::memcpy(z.data(), s.data(), std::min(w, s.size()));
    Put(b, z.data(), w);
}
inline void Vlr(std::vector<uint8_t>& b, const std::string& user, uint16_t id, const std::vector<uint8_t>& payload) {
    PutLE<uint16_t>(b, 0);
    PutStr(b, user, 16);
    PutLE<uint16_t>(b, id);
    PutLE<uint16_t>(b, static_cast<uint16_t>(payload.size()));
    PutStr(b, "", 32);
    Put(b, payload.data(), payload.size());
}
inline std::vector<uint8_t> LazVlr(int format, uint16_t eb, uint32_t chunk) {
    std::vector<uint8_t> v;
    const bool l14 = format >= 6;
    PutLE<uint16_t>(v, l14 ? 3 : 2);
    PutLE<uint16_t>(v, 0);
    v.push_back(l14 ? 3 : 2);
    v.push_back(l14 ? 4 : 2);
    PutLE<uint16_t>(v, 0);
    PutLE<uint32_t>(v, 0);
    PutLE<uint32_t>(v, chunk);
    PutLE<int64_t>(v, -1);
    PutLE<int64_t>(v, -1);
    std::vector<std::array<uint16_t, 3>> items;
    if (!l14) {
        items.push_back({6, 20, 2});
        if (format == 1 || format == 3 || format == 4 || format == 5) items.push_back({7, 8, 2});
        if (format == 2 || format == 3 || format == 5) items.push_back({8, 6, 2});
        if (format == 4 || format == 5) items.push_back({9, 29, 1});  // WAVEPACKET13
        if (eb) items.push_back({0, eb, 2});
    } else {
        items.push_back({10, 30, 3});
        if (format == 7) items.push_back({11, 6, 3});
        if (format == 8 || format == 10) items.push_back({12, 8, 3});
        if (format >= 9) items.push_back({13, 29, 3});  // WAVEPACKET14
        if (eb) items.push_back({14, eb, 3});
    }
    PutLE<uint16_t>(v, static_cast<uint16_t>(items.size()));
    for (const auto& it : items)
        for (uint16_t x : it) PutLE<uint16_t>(v, x);
    return v;
}
/// A layered chunk of format 9 / 10: laz-perf's chunk of the records without their wave packets
/// (format 6 / 8), with a WAVEPACKET14 layer spliced in where LASzip puts it (raw first packet
/// after the colour, its size after the colour sizes, its bytes after the colour bytes). Its
/// content is arbitrary: readers skip the layer.
inline std::vector<uint8_t> WaveChunk(int format, uint16_t eb, const uint8_t* recs, uint32_t n) {
    const int base = format == 9 ? 6 : 8;
    const size_t rs = las::BaseRecordSize(format) + eb, brs = las::BaseRecordSize(base) + eb;
    const size_t waveAt = las::BaseRecordSize(base);  // the packet follows the base fields
    std::vector<uint8_t> b(size_t{n} * brs);
    for (uint32_t i = 0; i < n; ++i) {
        std::memcpy(b.data() + i * brs, recs + i * rs, waveAt);
        std::memcpy(b.data() + i * brs + waveAt, recs + i * rs + waveAt + 29, eb);
    }
    auto c = laz::CompressChunk(base, eb, b, n);
    if (!c.ok()) return {};
    const std::vector<uint8_t>& in = *c;
    std::vector<uint8_t> out;
    size_t at = 0;
    Put(out, in.data(), waveAt);  // first point: base fields
    at = waveAt;
    Put(out, recs + waveAt, 29);  // raw first wave packet
    Put(out, in.data() + at, eb);  // first point: extra bytes
    at += eb;
    Put(out, in.data() + at, 4);  // point count
    at += 4;
    const size_t colourSizes = base == 8 ? 2 : 0;
    std::vector<uint32_t> sizes;
    for (size_t k = 0; k < 9 + colourSizes + eb; ++k) sizes.push_back(LoadLE<uint32_t>(in.data() + at + 4 * k));
    Put(out, in.data() + at, 4 * (9 + colourSizes));
    const std::vector<uint8_t> wave = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
    PutLE<uint32_t>(out, static_cast<uint32_t>(wave.size()));
    Put(out, in.data() + at + 4 * (9 + colourSizes), 4 * eb);
    at += 4 * (9 + colourSizes + eb);
    size_t before = 0;
    for (size_t k = 0; k < 9 + colourSizes; ++k) before += sizes[k];
    Put(out, in.data() + at, before);
    Put(out, wave.data(), wave.size());
    Put(out, in.data() + at + before, in.size() - at - before);
    return out;
}

}  // namespace detail

inline std::vector<uint8_t> BuildLasBytes(const SynthLas& s) {
    using namespace detail;
    const size_t rs = s.RecordSize();
    const las::PointLayout L = las::LayoutOf(s.format);
    std::array<double, 3> mn{1e300, 1e300, 1e300}, mx{-1e300, -1e300, -1e300};
    std::array<uint64_t, 15> byReturn{};
    for (uint64_t i = 0; i < s.count; ++i) {
        const uint8_t* r = s.records.data() + i * rs;
        for (size_t a = 0; a < 3; ++a) {
            const double v = LoadLE<int32_t>(r + 4 * a) * s.scale[a] + s.offset[a];
            mn[a] = std::min(mn[a], v);
            mx[a] = std::max(mx[a], v);
        }
        const int rn = L.extended ? (r[14] & 15) : (r[14] & 7);
        if (rn >= 1 && rn <= 15) ++byReturn[static_cast<size_t>(rn - 1)];
    }
    if (s.overrideBounds) {
        mn = s.min;
        mx = s.max;
    }
    std::vector<uint8_t> vlrs;
    uint32_t vlrCount = 0;
    if (!s.wkt.empty()) {
        std::vector<uint8_t> w(s.wkt.begin(), s.wkt.end());
        w.push_back(0);
        Vlr(vlrs, "LASF_Projection", 2112, w);
        ++vlrCount;
    }
    if (!s.extraBytesVlr.empty()) {
        Vlr(vlrs, "LASF_Spec", 4, s.extraBytesVlr);
        ++vlrCount;
    }
    if (s.laz) {
        Vlr(vlrs, "laszip encoded", 22204, LazVlr(s.format, s.extraBytes, s.chunkSize));
        ++vlrCount;
    }
    const size_t headerSize = s.minor >= 4 ? 375 : s.minor == 3 ? 235 : 227;
    std::vector<uint8_t> b;
    PutStr(b, "LASF", 4);
    PutLE<uint16_t>(b, 7);  // file source id
    PutLE<uint16_t>(b, static_cast<uint16_t>(s.minor >= 4 && s.format >= 6 && !s.wkt.empty() ? 16 : 0));
    for (int i = 0; i < 16; ++i) b.push_back(static_cast<uint8_t>(i));
    b.push_back(1);
    b.push_back(static_cast<uint8_t>(s.minor));
    PutStr(b, "synthetic", 32);
    PutStr(b, "ddbt test", 32);
    PutLE<uint16_t>(b, 123);
    PutLE<uint16_t>(b, 2024);
    PutLE<uint16_t>(b, static_cast<uint16_t>(headerSize));
    PutLE<uint32_t>(b, static_cast<uint32_t>(headerSize + vlrs.size()));
    PutLE<uint32_t>(b, vlrCount);
    b.push_back(static_cast<uint8_t>(s.format | (s.laz ? 0x80 : 0)));
    PutLE<uint16_t>(b, static_cast<uint16_t>(rs));
    const bool legacyOk = s.format < 6 && s.count <= UINT32_MAX;
    PutLE<uint32_t>(b, legacyOk ? static_cast<uint32_t>(s.count) : 0);
    for (size_t i = 0; i < 5; ++i) PutLE<uint32_t>(b, legacyOk ? static_cast<uint32_t>(byReturn[i]) : 0);
    for (size_t a = 0; a < 3; ++a) PutLE<double>(b, s.scale[a]);
    for (size_t a = 0; a < 3; ++a) PutLE<double>(b, s.offset[a]);
    for (size_t a = 0; a < 3; ++a) {
        PutLE<double>(b, mx[a]);
        PutLE<double>(b, mn[a]);
    }
    if (s.minor == 3) PutLE<uint64_t>(b, 0);  // waveform data
    if (s.minor >= 4) {
        PutLE<uint64_t>(b, 0);
        PutLE<uint64_t>(b, 0);
        PutLE<uint32_t>(b, 0);
        PutLE<uint64_t>(b, s.count);
        for (size_t i = 0; i < 15; ++i) PutLE<uint64_t>(b, byReturn[i]);
    }
    b.resize(headerSize, 0);
    Put(b, vlrs.data(), vlrs.size());
    if (!s.laz) {
        Put(b, s.records.data(), s.records.size());
        return b;
    }
    const size_t tableOffsetPos = b.size();
    PutLE<int64_t>(b, 0);
    std::vector<laz::ChunkTableEntry> table;
    for (uint64_t first = 0; first < s.count; first += s.chunkSize) {
        const uint32_t n = static_cast<uint32_t>(std::min<uint64_t>(s.chunkSize, s.count - first));
        std::vector<uint8_t> chunk;
        if (s.format >= 9) {
            chunk = WaveChunk(s.format, s.extraBytes, s.records.data() + first * rs, n);
        } else {
            auto c = laz::CompressChunk(s.format, s.extraBytes,
                                        std::span<const uint8_t>(s.records.data() + first * rs, size_t{n} * rs), n);
            if (!c.ok()) return {};
            chunk = std::move(*c);
        }
        if (chunk.empty()) return {};
        table.push_back({n, chunk.size()});
        Put(b, chunk.data(), chunk.size());
    }
    const uint64_t tableOffset = b.size();
    PutLE<uint32_t>(b, 0);
    PutLE<uint32_t>(b, static_cast<uint32_t>(table.size()));
    const std::vector<uint8_t> body = laz::CompressChunkTable(table, false);
    Put(b, body.data(), body.size());
    if (s.chunkTableMinusOne) {
        StoreLE<int64_t>(b.data() + tableOffsetPos, -1);
        PutLE<uint64_t>(b, tableOffset);
    } else {
        StoreLE<int64_t>(b.data() + tableOffsetPos, static_cast<int64_t>(tableOffset));
    }
    return b;
}

inline bool WriteBytes(const std::string& path, const std::vector<uint8_t>& b) {
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f.write(reinterpret_cast<const char*>(b.data()), static_cast<std::streamsize>(b.size()));
    return static_cast<bool>(f);
}

inline std::vector<uint8_t> ReadBytes(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

/// Order-independent hash of `count` records of `rs` bytes (sum of a 64-bit hash per record).
inline uint64_t MultisetHash(const uint8_t* recs, size_t count, size_t rs) {
    uint64_t sum = 0;
    for (size_t i = 0; i < count; ++i) {
        uint64_t h = 1469598103934665603ull;
        for (size_t k = 0; k < rs; ++k) h = (h ^ recs[i * rs + k]) * 1099511628211ull;
        h ^= h >> 31;
        h *= 0x7fb5d329728ea185ull;
        h ^= h >> 27;
        sum += h;
    }
    return sum;
}

}  // namespace ddbt::copc::testing
