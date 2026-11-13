// SPDX-License-Identifier: MPL-2.0
//
// Decoded blocks against GDAL: for every raster listed by tests/unit/tiff/data/gdal_reference.py
// (corpus, test_data GeoTIFFs, synthetic files covering every codec / type / layout), each
// recorded window of each directory (main image, overviews, masks) is read through TiffReader
// and its CRC-32 / Adler-32 compared with GDAL's band.ReadRaster of the same window.
//
// The fixtures live outside the repository (DDBT_TIFF_FIXTURES, default
// /mnt/data/ddb-tilers-work/fixtures/tiff); the test skips when they are absent. Tiers:
// DDBT_TIFF_PARITY_TIERS (default "synthetic,testdata,corpus,bench"; add "corpus-big" for R7..R10).

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <tuple>
#include <vector>

#include "cog/tiff/tiff_reader.h"
#include "tiff_test_util.h"

namespace ddbt::cog::tiff {
namespace {

std::string FixturesDir() {
    const char* e = std::getenv("DDBT_TIFF_FIXTURES");
    return e != nullptr ? e : "/mnt/data/ddb-tilers-work/fixtures/tiff";
}

std::set<std::string> Tiers() {
    const char* e = std::getenv("DDBT_TIFF_PARITY_TIERS");
    std::string s = e != nullptr ? e : "synthetic,testdata,corpus,bench";
    std::set<std::string> out;
    std::stringstream ss(s);
    std::string t;
    while (std::getline(ss, t, ',')) out.insert(t);
    return out;
}

std::vector<std::string> Split(const std::string& line) {
    std::vector<std::string> f;
    std::string cur;
    for (char c : line) {
        if (c == '\t') {
            f.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    f.push_back(cur);
    return f;
}

struct Window {
    std::string key;
    uint32_t dirW, dirH, x0, y0, w, h;
    bool operator<(const Window& o) const {
        return std::tie(key, dirW, dirH, y0, x0, w, h) < std::tie(o.key, o.dirW, o.dirH, o.y0, o.x0, o.w, o.h);
    }
};
struct Expect {
    uint32_t band, crc, adler;
};

// The IFD of the reader that GDAL shows as (key, width, height).
std::optional<uint32_t> FindIfd(const TiffReader& r, const std::string& key, uint32_t w, uint32_t h) {
    if (key == "main") return r.MainImage();
    SubfileKind want = key == "ovr" ? SubfileKind::kOverview : key == "mask" ? SubfileKind::kMask : SubfileKind::kOverviewMask;
    for (const IfdInfo& i : r.Ifds()) {
        if (i.kind == want && i.width == w && i.height == h) return i.index;
    }
    return std::nullopt;
}

// Max |a - b| over the samples of a JPEG dump (bytes).
int MaxAbsDiff(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
    int m = 0;
    for (size_t i = 0; i < std::min(a.size(), b.size()); ++i) m = std::max(m, std::abs(int(a[i]) - int(b[i])));
    return m;
}

TEST(TiffGdalParity, DecodedWindowsMatchGdal) {
    const std::string dir = FixturesDir();
    std::ifstream index(dir + "/index.tsv");
    if (!index) GTEST_SKIP() << "no fixtures in " << dir << " (run tests/unit/tiff/data/gdal_reference.py)";
    const auto tiers = Tiers();
    std::string line;
    size_t files = 0, totalWindows = 0, totalBad = 0;
    std::ostringstream report;
    while (std::getline(index, line)) {
        const auto f = Split(line);
        if (f.size() < 5) continue;
        const std::string& id = f[0];
        const std::string& path = f[1];
        if (!tiers.count(f[4])) continue;
        std::ifstream blocks(dir + "/blocks/" + id + ".tsv");
        if (!blocks) continue;
        struct stat st {};
        if (::stat(path.c_str(), &st) != 0 || std::to_string(st.st_size) != f[2]) {
            ADD_FAILURE() << id << ": input changed since the fixtures were made";
            continue;
        }
        std::map<Window, std::vector<Expect>> wins;
        std::string bl;
        while (std::getline(blocks, bl)) {
            const auto g = Split(bl);
            if (g.size() < 10) continue;
            Window w{g[0], uint32_t(std::stoul(g[1])), uint32_t(std::stoul(g[2])), uint32_t(std::stoul(g[4])),
                     uint32_t(std::stoul(g[5])), uint32_t(std::stoul(g[6])), uint32_t(std::stoul(g[7]))};
            wins[w].push_back({uint32_t(std::stoul(g[3])), uint32_t(std::stoul(g[8])), uint32_t(std::stoul(g[9]))});
        }
        auto reader = TiffReader::Open(path);
        if (!reader.ok()) {
            ADD_FAILURE() << id << ": " << reader.status().ToString();
            continue;
        }
        const TiffReader& r = **reader;
        size_t n = 0, bad = 0, errors = 0;
        int maxDiff = 0;
        std::string firstError;
        for (const auto& [w, exps] : wins) {
            auto ifd = FindIfd(r, w.key, w.dirW, w.dirH);
            if (!ifd) {
                ++errors;
                if (firstError.empty()) firstError = "no IFD for " + w.key + " " + std::to_string(w.dirW) + "x" + std::to_string(w.dirH);
                continue;
            }
            std::vector<uint32_t> bands;
            for (const Expect& e : exps) bands.push_back(e.band == 0 ? 0 : e.band - 1);
            const bool isMask = w.key == "mask" || w.key == "ovrmask";
            auto got = testing::ReadWindow(r, *ifd, bands, w.x0, w.y0, w.w, w.h, isMask ? 255 : 1);
            if (!got.ok()) {
                errors += exps.size();
                if (firstError.empty()) firstError = got.status().ToString();
                continue;
            }
            for (size_t k = 0; k < exps.size(); ++k) {
                ++n;
                const auto& data = (*got)[k];
                if (testing::Crc32(data) == exps[k].crc && testing::Adler32(data) == exps[k].adler) continue;
                ++bad;
                // JPEG: compare with GDAL's dump of the window when there is one.
                char name[256];
                std::snprintf(name, sizeof(name), "%s_%ux%u_b%u_%u_%u_%ux%u.raw", w.key.c_str(), w.dirW, w.dirH,
                              exps[k].band, w.x0, w.y0, w.w, w.h);
                std::ifstream dump(dir + "/dumps/" + id + "/" + name, std::ios::binary);
                if (dump) {
                    std::vector<uint8_t> ref((std::istreambuf_iterator<char>(dump)), {});
                    maxDiff = std::max(maxDiff, MaxAbsDiff(ref, data));
                }
            }
        }
        ++files;
        totalWindows += n;
        totalBad += bad + errors;
        report << "  " << id << ": " << n << " windows, " << bad << " mismatched, " << errors << " errors";
        if (maxDiff > 0) report << ", JPEG max abs diff " << maxDiff;
        if (!firstError.empty()) report << " (" << firstError << ")";
        report << "\n";
        EXPECT_EQ(bad, 0u) << id;
        EXPECT_EQ(errors, 0u) << id << ": " << firstError;
    }
    std::cout << "GDAL parity: " << files << " rasters, " << totalWindows << " windows, " << totalBad
              << " mismatched or failed\n"
              << report.str();
}

}  // namespace
}  // namespace ddbt::cog::tiff
