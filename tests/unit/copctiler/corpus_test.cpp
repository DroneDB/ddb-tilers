// SPDX-License-Identifier: MPL-2.0
// The COPC tiler on corpus clouds (skipped when absent; DDBT_COPC_TILER_CORPUS=a:b overrides the
// list, "none" disables): seams at z21+ (a 512 px tile at z against the mosaic of its four 256 px
// tiles of z + 1: same pixels, same octree depth) in the hybrid and modern modes, and their
// presence in compat mode (the baseline truncates its padding to 0 m from z21); determinism of
// hybrid and modern tiles whatever the order and the threads; hybrid's coverage against compat's
// (compat reproducing DroneDB 2.4.1's tiles) moved by (4, 1) px.
#include <gtest/gtest.h>

#include <atomic>
#include <cstring>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include "copc/cache/node_cache.h"
#include "copc/tiler/cloud.h"
#include "copc/tiler/copc_tiler.h"
#include "core/tile/compat.h"

namespace {

using namespace ddbt::copc::tiler;
namespace compat = ddbt::tile::compat;

std::vector<std::string> CorpusClouds() {
    std::vector<std::string> out;
    std::string list;
    if (const char* e = std::getenv("DDBT_COPC_TILER_CORPUS")) {
        list = e;
        if (list == "none") return out;
    } else {
        list =
            "/mnt/data/datasets/point-clouds/test-crash/autzen-classified.copc.laz:"
            "/mnt/data/datasets/point-clouds/test-crash/LHD_FXX_0658_6859_PTS_O_LAMB93_IGN69.copc.laz:"
            "/mnt/data/ddb-tilers-work/prepared/P1-b9c0ec9b47b4/cloud.copc.laz:"
            "/mnt/data/ddb-tilers-work/prepared/E5-f3f745895662/cloud.copc.laz";
    }
    size_t pos = 0;
    while (pos <= list.size()) {
        const size_t end = list.find(':', pos);
        const std::string f = list.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
        if (!f.empty() && std::filesystem::exists(f)) out.push_back(f);
        if (end == std::string::npos) break;
        pos = end + 1;
    }
    return out;
}

std::vector<uint8_t> Render(const Cloud& cloud, int z, int x, int y, int size, Mode mode) {
    TileRequest r;
    r.path = cloud.path();
    r.z = z, r.x = x, r.y = y, r.tileSize = size, r.tms = true, r.mode = mode;
    std::vector<uint8_t> rgba;
    RenderRgba(cloud, r, rgba);
    return rgba;
}

size_t Opaque(const std::vector<uint8_t>& rgba) {
    size_t n = 0;
    for (size_t i = 3; i < rgba.size(); i += 4) n += rgba[i] != 0;
    return n;
}

struct SeamStats {
    size_t tiles = 0, pixels = 0, differing = 0, seamPixels = 0, seamHoles = 0;
};

// 512 px tiles of zoom z around the centre of the cloud, against their four 256 px tiles.
SeamStats Seams(const Cloud& cloud, int z, Mode mode, int span) {
    SeamStats st;
    const auto& f = *cloud.compatFootprint();
    const double cx = (f.minX + f.maxX) / 2, cy = (f.minY + f.maxY) / 2;
    const compat::GlobalMercator m(512);
    const auto c = m.metersToTile(cx, cy, z);  // TMS row
    for (int dy = -span; dy <= span; ++dy)
        for (int dx = -span; dx <= span; ++dx) {
            const int x = c.x + dx, y = compat::FlipRow(c.y + dy, z);  // XYZ row
            std::vector<uint8_t> big;
            try {
                big = Render(cloud, z, x, y, 512, mode);
            } catch (const std::exception&) {
                continue;
            }
            if (Opaque(big) < 512 * 512 / 10) continue;  // tiles with data
            std::vector<uint8_t> mosaic(big.size(), 0);
            bool inRange = true;
            for (int q = 0; q < 4 && inRange; ++q) {
                std::vector<uint8_t> small;
                try {
                    small = Render(cloud, z + 1, 2 * x + (q & 1), 2 * y + (q >> 1), 256, mode);
                } catch (const std::exception&) {  // out of the footprint's range
                    inRange = false;
                    break;
                }
                for (size_t row = 0; row < 256; ++row)
                    std::memcpy(mosaic.data() + ((static_cast<size_t>(q >> 1) * 256 + row) * 512 +
                                                 static_cast<size_t>(q & 1) * 256) *
                                                    4,
                                small.data() + row * 256 * 4, 256 * 4);
            }
            if (!inRange) continue;
            ++st.tiles;
            for (size_t i = 0; i < big.size(); i += 4) {
                ++st.pixels;
                st.differing += std::memcmp(big.data() + i, mosaic.data() + i, 4) != 0;
            }
            for (size_t k = 0; k < 512; ++k)
                for (const size_t seam : {size_t{255}, size_t{256}}) {
                    for (const size_t a : {(k * 512 + seam) * 4 + 3, (seam * 512 + k) * 4 + 3}) {
                        ++st.seamPixels;
                        st.seamHoles += big[a] != 0 && mosaic[a] == 0;
                    }
                }
        }
    return st;
}

TEST(CopcTilerCorpus, SplatModesHaveNoSeamsAtZ21AndAbove) {
    const auto clouds = CorpusClouds();
    if (clouds.empty()) GTEST_SKIP() << "no corpus cloud";
    for (const auto& path : clouds) {
        const auto cloud = Cloud::Get(path);
        if (!cloud->compatFootprint()) continue;
        for (const int z : {20, 21}) {  // 256 px tiles of z21 and z22
            const SeamStats c = Seams(*cloud, z, Mode::kCompat, 2);
            for (const Mode mode : {Mode::kHybrid, Mode::kModern}) {
                const SeamStats d = Seams(*cloud, z, mode, 2);
                std::printf("%s z%d+1: %zu tiles; %s: %zu of %zu pixels differ, %zu holes on %zu seam pixels; "
                            "compat: %zu differ, %zu holes\n",
                            std::filesystem::path(path).filename().c_str(), z, d.tiles, ModeName(mode), d.differing,
                            d.pixels, d.seamHoles, d.seamPixels, c.differing, c.seamHoles);
                if (d.tiles == 0) continue;
                // A point on a pixel edge may land on either side in the two renders (separate fits).
                EXPECT_LE(d.differing, d.pixels / 500) << path << " z" << z << " " << ModeName(mode);
                EXPECT_LE(d.seamHoles, d.seamPixels / 1000) << path << " z" << z << " " << ModeName(mode);
                if (z == 20) {
                    EXPECT_GT(c.seamHoles, d.seamHoles * 10 + 10) << "compat seams at z21 " << path;
                }
            }
        }
    }
}

struct Tile {
    int z, x, y;  // XYZ row
};

// Tiles of z16, z18, z20 and z22 on the diagonal through the centre of the cloud (in range).
std::vector<Tile> CentreTiles(const Cloud& cloud) {
    const auto& f = *cloud.compatFootprint();
    const double cx = (f.minX + f.maxX) / 2, cy = (f.minY + f.maxY) / 2;
    std::vector<Tile> tiles;
    for (const int z : {16, 18, 20, 22}) {
        const compat::GlobalMercator m(256);
        const auto c = m.metersToTile(cx, cy, z);
        for (int d = -1; d <= 1; ++d) {
            const Tile t{z, c.x + d, compat::FlipRow(c.y + d, z)};
            try {
                Render(cloud, t.z, t.x, t.y, 256, Mode::kCompat);
                tiles.push_back(t);
            } catch (const std::exception&) {  // out of the footprint's range
            }
        }
    }
    return tiles;
}

TEST(CopcTilerCorpus, SplatModeTilesDoNotDependOnThreadsOrOrder) {
    const auto clouds = CorpusClouds();
    if (clouds.empty()) GTEST_SKIP() << "no corpus cloud";
    for (const auto& path : clouds) {
        const auto cloud = Cloud::Get(path);
        if (!cloud->compatFootprint()) continue;
        const auto tiles = CentreTiles(*cloud);
        for (const Mode mode : {Mode::kHybrid, Mode::kModern}) {
            ddbt::copc::cache::NodeCache::Global().Clear();
            std::vector<std::vector<uint8_t>> serial;
            for (const auto& t : tiles) serial.push_back(Render(*cloud, t.z, t.x, t.y, 256, mode));
            for (const int threads : {8, 16}) {
                ddbt::copc::cache::NodeCache::Global().Clear();
                std::atomic<size_t> next{0}, bad{0};
                std::vector<std::thread> pool;
                for (int t = 0; t < threads; ++t)
                    pool.emplace_back([&] {
                        for (size_t i; (i = next.fetch_add(1)) < tiles.size() * 3;) {
                            const size_t k = tiles.size() - 1 - i % tiles.size();  // reverse order
                            const auto& tl = tiles[k];
                            if (Render(*cloud, tl.z, tl.x, tl.y, 256, mode) != serial[k]) ++bad;
                        }
                    });
                for (auto& th : pool) th.join();
                EXPECT_EQ(bad.load(), 0u) << path << " " << ModeName(mode) << " " << threads << " threads";
            }
        }
    }
}

TEST(CopcTilerCorpus, HybridCoversWhatCompatCoversMovedOntoThePoint) {
    // Compat is DroneDB 2.4.1's tiles (pixel-identical on the coloured clouds' sweeps);
    // hybrid draws the same stamps at the same depth (4, 1) px away: alpha IoU ~1 at every zoom,
    // where the stamps overlap as where they do not (z20-z22).
    const auto clouds = CorpusClouds();
    if (clouds.empty()) GTEST_SKIP() << "no corpus cloud";
    for (const auto& path : clouds) {
        const auto cloud = Cloud::Get(path);
        if (!cloud->compatFootprint()) continue;
        size_t inter = 0, uni = 0;
        for (const auto& t : CentreTiles(*cloud)) {
            const auto c = Render(*cloud, t.z, t.x, t.y, 256, Mode::kCompat);
            const auto h = Render(*cloud, t.z, t.x, t.y, 256, Mode::kHybrid);
            size_t ti = 0, tu = 0;
            for (size_t row = 8; row < 248; ++row)
                for (size_t col = 8; col < 248; ++col) {
                    const bool a = c[(row * 256 + col) * 4 + 3] != 0;
                    const bool b = h[((row + 1) * 256 + col + 4) * 4 + 3] != 0;
                    ti += a && b;
                    tu += a || b;
                }
            if (tu > 0) {
                EXPECT_GE(static_cast<double>(ti) / static_cast<double>(tu), 0.99)
                    << path << " z" << t.z << " " << t.x << " " << t.y;
            }
            inter += ti;
            uni += tu;
        }
        std::printf("%s: hybrid vs compat moved (4, 1): alpha IoU %.5f on %zu pixels\n",
                    std::filesystem::path(path).filename().c_str(),
                    uni ? static_cast<double>(inter) / static_cast<double>(uni) : 1.0, uni);
        if (uni > 0) {
            EXPECT_GE(static_cast<double>(inter) / static_cast<double>(uni), 0.999) << path;
        }
    }
}

}  // namespace
