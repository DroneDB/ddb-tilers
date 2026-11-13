// SPDX-License-Identifier: MPL-2.0
//
// Test helpers for the LAS / LAZ / COPC readers: synthetic point records of formats 6-8 that
// exercise every layer (scanner channel switches, return changes, GPS time jumps, 8- and 16-bit
// colour, extra bytes), a builder of in-memory COPC files with child hierarchy pages, the
// corpus file list, and an order-independent hash of point sets.
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <map>
#include <random>
#include <string>
#include <tuple>
#include <vector>

#include "copc/copc/hierarchy.h"
#include "copc/las/point_format.h"
#include "copc/laz/chunk_decoder.h"

namespace ddbt::copc::testing {

template <typename T>
inline void Put(std::vector<uint8_t>& v, size_t at, T x) {
    if (at > v.size() || v.size() - at < sizeof(T)) std::abort();
    std::memcpy(v.data() + at, &x, sizeof(T));
}
template <typename T>
inline void Append(std::vector<uint8_t>& v, T x) {
    const size_t at = v.size();
    v.resize(at + sizeof(T));
    std::memcpy(v.data() + at, &x, sizeof(T));
}
template <typename T>
inline T Get(const uint8_t* p) {
    T v;
    std::memcpy(&v, p, sizeof(T));
    return v;
}

struct SynthOptions {
    int format = 7;
    int extraBytes = 0;
    uint32_t seed = 1;
    bool multiChannel = true;  // scanner channel runs over 0..3
    bool color16 = true;       // colours above 255
    int32_t base[3] = {1000, 2000, 300};
    int32_t spread = 5000;     // random walk bound per axis
};

/// `n` records of format 6/7/8 + extra bytes. Every field changes now and then, with runs.
inline std::vector<uint8_t> SynthRecords(size_t n, const SynthOptions& o) {
    const las::PointLayout l = las::LayoutOf(o.format);
    const size_t rs = l.baseSize + static_cast<size_t>(o.extraBytes);
    std::vector<uint8_t> v(n * rs, 0);
    std::mt19937 rng(o.seed);
    auto uni = [&rng](int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); };
    int32_t x = o.base[0], y = o.base[1], z = o.base[2];
    int channel = 0, nret = 1, ret = 1;
    double gps = 1.0e8 + uni(0, 1000);
    uint16_t intensity = 100, psid = 7;
    int16_t scan = 0;
    uint8_t cls = 2, user = 0, flags = 0;
    uint16_t rgb[3] = {200, 100, 50}, nir = 300;
    std::vector<uint8_t> eb(static_cast<size_t>(o.extraBytes), 0);
    for (size_t i = 0; i < n; ++i) {
        uint8_t* p = v.data() + i * rs;
        x = std::clamp(x + uni(-300, 300), o.base[0] - o.spread, o.base[0] + o.spread);
        y = std::clamp(y + uni(-300, 300), o.base[1] - o.spread, o.base[1] + o.spread);
        z = std::clamp(z + uni(-50, 50), o.base[2] - o.spread, o.base[2] + o.spread);
        if (o.multiChannel && uni(0, 9) == 0) channel = uni(0, 3);
        if (uni(0, 3) == 0) {
            nret = uni(1, 15);
            ret = uni(1, nret);
        } else if (ret < nret) {
            ++ret;
        } else {
            ret = 1;
        }
        const int g = uni(0, 19);
        if (g < 10) gps += 0.00001 * uni(1, 5);          // small steps
        else if (g == 10) gps += uni(1000, 100000);       // jump
        else if (g == 11) gps -= uni(1, 50);              // backwards
        // else: same time (multiple returns)
        if (uni(0, 4) == 0) intensity = static_cast<uint16_t>(uni(0, 65535));
        if (uni(0, 30) == 0) psid = static_cast<uint16_t>(uni(0, 65535));
        if (uni(0, 10) == 0) scan = static_cast<int16_t>(uni(-15000, 15000));
        if (uni(0, 6) == 0) cls = static_cast<uint8_t>(uni(0, 255));
        if (uni(0, 20) == 0) user = static_cast<uint8_t>(uni(0, 255));
        if (uni(0, 8) == 0) flags = static_cast<uint8_t>(uni(0, 15) | (uni(0, 1) << 6) | (uni(0, 1) << 7));
        for (uint16_t& c : rgb)
            if (uni(0, 2) == 0) c = static_cast<uint16_t>(o.color16 ? uni(0, 65535) : uni(0, 255));
        if (uni(0, 3) == 0) nir = static_cast<uint16_t>(uni(0, 65535));
        for (uint8_t& b : eb)
            if (uni(0, 3) == 0) b = static_cast<uint8_t>(uni(0, 255));

        std::memcpy(p + 0, &x, 4);
        std::memcpy(p + 4, &y, 4);
        std::memcpy(p + 8, &z, 4);
        std::memcpy(p + 12, &intensity, 2);
        p[14] = static_cast<uint8_t>(ret | (nret << 4));
        p[15] = static_cast<uint8_t>((flags & 0xCF) | (channel << 4));
        p[16] = cls;
        p[17] = user;
        std::memcpy(p + 18, &scan, 2);
        std::memcpy(p + 20, &psid, 2);
        std::memcpy(p + 22, &gps, 8);
        if (l.HasRgb()) std::memcpy(p + l.rgb, rgb, 6);
        if (l.HasNir()) std::memcpy(p + l.nir, &nir, 2);
        if (o.extraBytes) std::memcpy(p + l.baseSize, eb.data(), eb.size());
    }
    return v;
}

/// A synthetic COPC file. Nodes are given by key with their records; nodes at depth
/// `pageDepth` (when > 0) and their descendants go into child hierarchy pages, linked from
/// the root page.
struct SynthNode {
    NodeKey key;
    std::vector<uint8_t> records;  // format records
    uint32_t count = 0;            // 0: a node without points
};

struct SynthCopc {
    int format = 7;
    int extraBytes = 0;
    double scale[3] = {0.01, 0.01, 0.01};
    double offset[3] = {0, 0, 0};
    double center[3] = {50, 50, 50};
    double halfSize = 50;
    double spacing = 1;
    int pageDepth = 0;
    std::string wkt;
    std::vector<SynthNode> nodes;
};

inline std::vector<uint8_t> BuildCopcBytes(const SynthCopc& s) {
    const las::PointLayout l = las::LayoutOf(s.format);
    const uint16_t rs = static_cast<uint16_t>(l.baseSize + s.extraBytes);
    std::vector<uint8_t> f(375, 0);
    std::memcpy(f.data(), "LASF", 4);
    Put<uint16_t>(f, 6, 16 | 1);  // WKT + standard GPS time
    f[24] = 1;
    f[25] = 4;
    std::memcpy(f.data() + 26, "ddbt test", 9);
    std::memcpy(f.data() + 58, "ddbt synthetic copc", 19);
    Put<uint16_t>(f, 94, 375);
    f[104] = static_cast<uint8_t>(s.format | 0x80);
    Put<uint16_t>(f, 105, rs);
    for (int i = 0; i < 3; ++i) {
        Put<double>(f, 131 + 8 * static_cast<size_t>(i), s.scale[i]);
        Put<double>(f, 155 + 8 * static_cast<size_t>(i), s.offset[i]);
        Put<double>(f, 179 + 16 * static_cast<size_t>(i), s.center[i] + s.halfSize);
        Put<double>(f, 187 + 16 * static_cast<size_t>(i), s.center[i] - s.halfSize);
    }
    auto addVlr = [&f](const char* user, uint16_t record, const std::vector<uint8_t>& data) {
        const size_t at = f.size();
        f.resize(at + 54, 0);
        std::strncpy(reinterpret_cast<char*>(f.data() + at + 2), user, 16);
        Put<uint16_t>(f, at + 18, record);
        Put<uint16_t>(f, at + 20, static_cast<uint16_t>(data.size()));
        f.insert(f.end(), data.begin(), data.end());
    };
    // COPC info (rewritten once the hierarchy is placed).
    std::vector<uint8_t> info(160, 0);
    addVlr("copc", 1, info);
    const size_t infoAt = 375 + 54;
    // LAZ VLR.
    std::vector<uint8_t> laz;
    Append<uint16_t>(laz, 3);
    Append<uint16_t>(laz, 0);
    laz.push_back(3);
    laz.push_back(4);
    Append<uint16_t>(laz, 3);
    Append<uint32_t>(laz, 0);
    Append<uint32_t>(laz, 0xFFFFFFFFu);
    Append<int64_t>(laz, -1);
    Append<int64_t>(laz, -1);
    std::vector<std::array<uint16_t, 3>> items{{10, 30, 3}};
    if (s.format == 7) items.push_back({11, 6, 3});
    if (s.format == 8) items.push_back({12, 8, 3});
    if (s.extraBytes) items.push_back({14, static_cast<uint16_t>(s.extraBytes), 3});
    Append<uint16_t>(laz, static_cast<uint16_t>(items.size()));
    for (auto& it : items)
        for (uint16_t w : it) Append<uint16_t>(laz, w);
    addVlr("laszip encoded", 22204, laz);
    if (!s.wkt.empty()) {
        std::vector<uint8_t> w(s.wkt.begin(), s.wkt.end());
        w.push_back(0);
        addVlr("LASF_Projection", 2112, w);
    }
    Put<uint32_t>(f, 100, s.wkt.empty() ? 2u : 3u);
    Put<uint32_t>(f, 96, static_cast<uint32_t>(f.size()));
    Append<int64_t>(f, -1);  // chunk table offset (none)

    struct Placed {
        const SynthNode* node;
        uint64_t offset;
        int32_t size;
    };
    std::vector<Placed> placed;
    uint64_t total = 0;
    for (const SynthNode& n : s.nodes) {
        Placed p{&n, f.size(), 0};
        if (n.count > 0) {
            auto chunk = laz::CompressChunk(s.format, s.extraBytes, n.records, n.count);
            if (!chunk.ok()) std::abort();
            f.insert(f.end(), chunk->begin(), chunk->end());
            p.size = static_cast<int32_t>(chunk->size());
        }
        total += n.count;
        placed.push_back(p);
    }
    Put<uint64_t>(f, 247, total);

    // Hierarchy: root page, then one page per subtree rooted at depth pageDepth.
    auto entry = [](std::vector<uint8_t>& page, const NodeKey& k, uint64_t off, int32_t size, int32_t count) {
        Append<int32_t>(page, k.d);
        Append<int32_t>(page, k.x);
        Append<int32_t>(page, k.y);
        Append<int32_t>(page, k.z);
        Append<uint64_t>(page, off);
        Append<int32_t>(page, size);
        Append<int32_t>(page, count);
    };
    auto subtreeRoot = [&s](const NodeKey& k) {
        NodeKey r = k;
        while (r.d > s.pageDepth) r = r.Parent();
        return r;
    };
    std::vector<uint8_t> root;
    std::map<std::tuple<int, int, int, int>, std::vector<uint8_t>> children;
    for (const Placed& p : placed) {
        const NodeKey& k = p.node->key;
        if (s.pageDepth > 0 && k.d >= s.pageDepth) {
            const NodeKey r = subtreeRoot(k);
            entry(children[{r.d, r.x, r.y, r.z}], k, p.offset, p.size, static_cast<int32_t>(p.node->count));
        } else {
            entry(root, k, p.offset, p.size, static_cast<int32_t>(p.node->count));
        }
    }
    const uint64_t evlrAt = f.size();
    f.resize(f.size() + 60, 0);
    std::strncpy(reinterpret_cast<char*>(f.data() + evlrAt + 2), "copc", 16);
    Put<uint16_t>(f, evlrAt + 18, 1000);
    const uint64_t rootAt = f.size();
    const uint64_t rootSize = root.size() + 32 * children.size();
    const uint64_t firstChildAt = rootAt + rootSize;
    uint64_t childAt = firstChildAt;
    for (auto& [k, page] : children) {
        entry(root, {std::get<0>(k), std::get<1>(k), std::get<2>(k), std::get<3>(k)}, childAt,
              static_cast<int32_t>(page.size()), -1);
        childAt += page.size();
    }
    f.insert(f.end(), root.begin(), root.end());
    for (auto& [k, page] : children) f.insert(f.end(), page.begin(), page.end());
    Put<uint64_t>(f, evlrAt + 20, f.size() - evlrAt - 60);
    Put<uint64_t>(f, 235, evlrAt);
    Put<uint32_t>(f, 243, 1);

    Put<double>(f, infoAt + 0, s.center[0]);
    Put<double>(f, infoAt + 8, s.center[1]);
    Put<double>(f, infoAt + 16, s.center[2]);
    Put<double>(f, infoAt + 24, s.halfSize);
    Put<double>(f, infoAt + 32, s.spacing);
    Put<uint64_t>(f, infoAt + 40, rootAt);
    Put<uint64_t>(f, infoAt + 48, rootSize);
    return f;
}

/// COPC files of the corpus that exist on this machine (P2, P3, 20160402, the baseline-built
/// P1 COPCs, a PDRF 8 + extra bytes COPC built from E5). Override with DDBT_COPC_CORPUS
/// (colon-separated paths); DDBT_COPC_CORPUS=none disables.
inline std::vector<std::string> CorpusCopcFiles() {
    std::vector<std::string> files;
    if (const char* e = std::getenv("DDBT_COPC_CORPUS")) {
        std::string s = e;
        if (s == "none") return files;
        size_t pos = 0;
        while (pos <= s.size()) {
            const size_t end = s.find(':', pos);
            const std::string f = s.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
            if (!f.empty()) files.push_back(f);
            if (end == std::string::npos) break;
            pos = end + 1;
        }
        return files;
    }
    const char* defaults[] = {
        "/mnt/data/datasets/point-clouds/test-crash/autzen-classified.copc.laz",
        "/mnt/data/datasets/point-clouds/test-crash/LHD_FXX_0658_6859_PTS_O_LAMB93_IGN69.copc.laz",
        "/mnt/data/datasets/point-clouds/test-crash/20160402_11SMS46053676.copc.laz",
        "/mnt/data/ddb-tilers-work/prepared/P1-b9c0ec9b47b4/cloud.copc.laz",
        "/mnt/data/ddb-tilers-work/out/baseline/smoke/copc-pdal/cloud.copc.laz",
        "/mnt/data/ddb-tilers-work/out/baseline/smoke/copc-untwine/cloud.copc.laz",
        "/mnt/data/ddb-tilers-work/fixtures/copc/E5-pdrf8-eb.copc.laz",
    };
    for (const char* f : defaults)
        if (std::filesystem::exists(f)) files.push_back(f);
    return files;
}

/// Order-independent hash of a multiset of points: sum and xor of a 64-bit mix of each point.
struct MultisetHash {
    uint64_t count = 0, sum = 0, xr = 0;
    static uint64_t Mix(uint64_t v) {
        v ^= v >> 33;
        v *= 0xff51afd7ed558ccdULL;
        v ^= v >> 33;
        v *= 0xc4ceb9fe1a85ec53ULL;
        v ^= v >> 33;
        return v;
    }
    void Add(int32_t x, int32_t y, int32_t z, uint16_t r, uint16_t g, uint16_t b) {
        uint64_t h = Mix(static_cast<uint32_t>(x) ^ (uint64_t{static_cast<uint32_t>(y)} << 32));
        h = Mix(h ^ static_cast<uint32_t>(z) ^ (uint64_t{r} << 32) ^ (uint64_t{g} << 48));
        h = Mix(h ^ b);
        ++count;
        sum += h;
        xr ^= h;
    }
    friend bool operator==(const MultisetHash&, const MultisetHash&) = default;
};

}  // namespace ddbt::copc::testing
