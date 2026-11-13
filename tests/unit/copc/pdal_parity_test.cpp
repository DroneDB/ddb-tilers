// SPDX-License-Identifier: MPL-2.0
//
// The points of a window at a resolution, our way (HierarchyIndex::Select with PDAL's
// depthEnd, layered XYZ+RGB decode, readers.copc's point test) against PDAL's readers.copc
// with `bounds` and `resolution`, as DroneDB's copc_node_cache_test samePointsAsReadersCopc
// does: same count and same order-independent hash of (X, Y, Z integers, R, G, B).
//
// PDAL runs as the `pdal` CLI (PDAL 2.10.1 of DroneDB's vcpkg tree, or $DDBT_PDAL):
// `pdal translate in.copc.laz out.las --readers.copc.bounds=... --readers.copc.resolution=...
// --writers.las.forward=all`, whose output keeps the input's scale and offset, so its integers
// are read back exactly. PDAL's answers are cached in $DDBT_COPC_FIXTURES/pdal-parity (default
// /mnt/data/ddb-tilers-work/fixtures/copc/pdal-parity), keyed by file size and case, so only
// the first run pays for PDAL. Cases above 12 M points are skipped (counted in the output).
// The test skips without pdal or corpus files.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

#include "copc/copc/copc_file.h"
#include "copc/copc/node_decoder.h"
#include "copc/las/las_header.h"
#include "copc_test_util.h"
#include "core/exec/parallel_for.h"
#include "core/io/byte_source.h"

namespace ddbt::copc {
namespace {

namespace fs = std::filesystem;
using testing::MultisetHash;

std::string PdalBinary() {
    if (const char* e = std::getenv("DDBT_PDAL")) return e;
    return "/mnt/data/repos/DroneDB/build/vcpkg_installed/x64-linux/tools/pdal/pdal";
}

fs::path CacheDir() {
    const char* e = std::getenv("DDBT_COPC_FIXTURES");
    return fs::path(e != nullptr ? e : "/mnt/data/ddb-tilers-work/fixtures/copc") / "pdal-parity";
}

struct Case {
    std::string name;
    Box3 box;
    bool threeD = false;
    double resolution = 0;
};

std::string Num(double v) {
    char b[64];
    std::snprintf(b, sizeof(b), "%.17g", v);
    return b;
}

std::string BoundsArg(const Case& c) {
    std::string s = "([" + Num(c.box.minX) + "," + Num(c.box.maxX) + "],[" + Num(c.box.minY) + "," + Num(c.box.maxY) + "]";
    if (c.threeD) s += ",[" + Num(c.box.minZ) + "," + Num(c.box.maxZ) + "]";
    return s + ")";
}

// Hash of the points of an uncompressed LAS file written by PDAL, in the input's scale/offset.
Expected<MultisetHash> HashLas(const std::string& path, const las::LasHeader& input) {
    DDBT_ASSIGN_OR_RETURN(std::unique_ptr<io::ByteSource> src, io::OpenByteSource(path));
    DDBT_ASSIGN_OR_RETURN(las::LasMetadata m, las::ReadMetadata(*src));
    const las::LasHeader& h = m.header;
    // forward=all passes the offsets through metadata text, which can round them (the
    // 20160402 file): then the integers are mapped back to the input's through the doubles
    // (exact as long as the offsets differ by much less than half a scale step).
    bool same = true;
    for (size_t a = 0; a < 3; ++a) {
        if (h.scale[a] != input.scale[a] || h.offset[a] != input.offset[a]) same = false;
        if (std::fabs(h.offset[a] - input.offset[a]) > 0.01 * input.scale[a] || h.scale[a] != input.scale[a])
            return InternalError("PDAL output has another scale / offset");
    }
    auto toInput = [&](int32_t v, size_t a) -> int32_t {
        if (same) return v;
        const double x = static_cast<double>(v) * h.scale[a] + h.offset[a];
        return static_cast<int32_t>(std::llround((x - input.offset[a]) / input.scale[a]));
    };
    if (h.compressed) return InternalError("PDAL output is compressed");
    const las::PointLayout l = h.Layout();
    MultisetHash out;
    const uint64_t n = h.pointCount;
    std::vector<uint8_t> rec(h.pointRecordLength);
    for (uint64_t i = 0; i < n; ++i) {
        DDBT_TRY(src->Read(h.pointDataOffset + i * h.pointRecordLength, rec));
        const uint8_t* p = rec.data();
        uint16_t r = 0, g = 0, b = 0;
        if (l.HasRgb()) {
            r = testing::Get<uint16_t>(p + l.rgb);
            g = testing::Get<uint16_t>(p + l.rgb + 2);
            b = testing::Get<uint16_t>(p + l.rgb + 4);
        }
        out.Add(toInput(testing::Get<int32_t>(p), 0), toInput(testing::Get<int32_t>(p + 4), 1),
                toInput(testing::Get<int32_t>(p + 8), 2), r, g, b);
    }
    return out;
}

fs::path TmpDir() {
    return fs::path("/mnt/data/ddb-tilers-work/tmp") / ("copc-parity-" + std::to_string(::getpid()));
}

// PDAL's hash for a case (cached unless `fresh`).
Expected<MultisetHash> PdalHash(const std::string& file, const las::LasHeader& header, const Case& c, bool fresh = false) {
    const std::string key = fs::path(file).filename().string() + "-" + std::to_string(fs::file_size(file)) + "-" +
                            std::to_string(std::hash<std::string>{}(BoundsArg(c) + "|" + Num(c.resolution))) + ".txt";
    const fs::path cache = CacheDir() / key;
    if (!fresh && fs::exists(cache)) {
        std::ifstream in(cache);
        MultisetHash h;
        in >> h.count >> h.sum >> h.xr;
        if (in) return h;
    }
    std::error_code ec;
    const fs::path tmpDir = TmpDir();
    fs::create_directories(tmpDir, ec);
    const fs::path out = tmpDir / (key + ".las");
    const std::string cmd = PdalBinary() + " translate '" + file + "' '" + out.string() + "' --readers.copc.bounds='" +
                            BoundsArg(c) + "' --readers.copc.resolution=" + Num(c.resolution) +
                            " --writers.las.forward=all > /dev/null 2>&1";
    if (std::system(cmd.c_str()) != 0) return InternalError("pdal failed: " + cmd);
    auto h = HashLas(out.string(), header);
    fs::remove(out, ec);
    if (!h.ok()) return h.status().WithContext(cmd);
    fs::create_directories(CacheDir(), ec);
    std::ofstream(cache) << h->count << " " << h->sum << " " << h->xr << "\n";
    return h;
}

TEST(CopcPdalParity, SamePointsAsReadersCopc) {
    if (!fs::exists(PdalBinary())) GTEST_SKIP() << "no pdal at " << PdalBinary() << " (set DDBT_PDAL)";
    const auto files = testing::CorpusCopcFiles();
    if (files.empty()) GTEST_SKIP() << "no corpus COPC files";
    constexpr uint64_t kMaxPoints = 12'000'000;
    for (const std::string& path : files) {
        auto file = CopcFile::Open(path);
        ASSERT_TRUE(file.ok()) << file.status().ToString();
        const CopcFile& f = **file;
        const las::LasHeader& h = f.header();
        const double w = h.max[0] - h.min[0], ht = h.max[1] - h.min[1], dz = h.max[2] - h.min[2];
        const Box3 nb = f.hierarchy().NodeBounds({2, 1, 1, 0});
        std::vector<Case> windows = {
            {"all", Box3::Xy(h.min[0], h.min[1], h.max[0], h.max[1])},
            {"quarter", Box3::Xy(h.min[0] + w * 0.25, h.min[1] + ht * 0.25, h.min[0] + w * 0.5, h.min[1] + ht * 0.5)},
            {"small", Box3::Xy(h.min[0] + w * 0.6, h.min[1] + ht * 0.1, h.min[0] + w * 0.62, h.min[1] + ht * 0.13)},
            {"outside", Box3::Xy(h.max[0] + 10, h.max[1] + 10, h.max[0] + 20, h.max[1] + 20)},
            {"node-edges", Box3::Xy(nb.minX, nb.minY, nb.maxX, nb.maxY)},
            {"3d", {h.min[0] + w * 0.2, h.min[1] + ht * 0.3, h.min[2] + dz * 0.3, h.min[0] + w * 0.7, h.min[1] + ht * 0.6,
                    h.min[2] + dz * 0.6}, true},
        };
        std::vector<Case> cases;
        const double s = f.spacing();
        for (const Case& win : windows)
            for (double r : {s * 2, s / 3, s / 20, s / 500}) {
                Case c = win;
                c.resolution = r;
                c.name = win.name + " res " + Num(r);
                cases.push_back(c);
            }

        // Ours (cases in parallel: NodeDecoder is thread-safe).
        NodeDecoder dec(*file);
        PointFilter base;
        for (size_t a = 0; a < 3; ++a) {
            base.scale[a] = h.scale[a];
            base.offset[a] = h.offset[a];
        }
        std::vector<MultisetHash> ours(cases.size());
        std::vector<char> skipped(cases.size(), 0), failed(cases.size(), 0);
        exec::ParallelFor(0, cases.size(), 1, [&](size_t i) {
            const Case& c = cases[i];
            SelectQuery q{c.box, DepthEndForResolution(s, c.resolution)};
            auto sel = f.hierarchy().Select(q);
            if (!sel.ok()) {
                failed[i] = 1;
                return;
            }
            if (sel->points > kMaxPoints) {
                skipped[i] = 1;
                return;
            }
            PointFilter filter = base;
            filter.box = c.box;
            RawPoints raw;
            for (const HierarchyNode* n : sel->nodes) {
                if (!dec.DecodeRaw(*n, laz::kLayersXyzRgb, raw).ok()) {
                    failed[i] = 1;
                    return;
                }
                for (size_t k = 0; k < raw.size(); ++k) {
                    if (!filter.Contains(raw.x[k], raw.y[k], raw.z[k])) continue;
                    ours[i].Add(raw.x[k], raw.y[k], raw.z[k], raw.r.empty() ? 0 : raw.r[k], raw.g.empty() ? 0 : raw.g[k],
                                raw.b.empty() ? 0 : raw.b[k]);
                }
            }
        });
        for (size_t i = 0; i < cases.size(); ++i) ASSERT_FALSE(failed[i]) << path << " " << cases[i].name;

        // PDAL's, several pdal processes at a time.
        std::vector<Expected<MultisetHash>> theirs(cases.size(), Expected<MultisetHash>(InternalError("not run")));
        exec::ParallelFor(0, cases.size(), 1, [&](size_t i) {
            if (!skipped[i]) theirs[i] = PdalHash(path, h, cases[i]);
        });

        size_t compared = 0, nonEmpty = 0, skips = 0, pdalUnstable = 0;
        uint64_t pointsCompared = 0;
        for (size_t i = 0; i < cases.size(); ++i) {
            if (skipped[i]) {
                ++skips;
                continue;
            }
            ASSERT_TRUE(theirs[i].ok()) << theirs[i].status().ToString();
            if (!(ours[i] == *theirs[i])) {
                // readers.copc (streaming, 2 reader threads) was seen to return another point set of
                // the same size, once in a few dozen runs, only while many pdal processes ran at
                // once; never reproduced alone. Ask PDAL again, alone: if PDAL disagrees with
                // itself and one answer is ours, count it as PDAL instability.
                std::vector<MultisetHash> again;
                for (int k = 0; k < 3; ++k) {
                    auto h2 = PdalHash(path, f.header(), cases[i], true);
                    ASSERT_TRUE(h2.ok()) << h2.status().ToString();
                    again.push_back(*h2);
                }
                const bool pdalAgrees = again[0] == *theirs[i] && again[1] == *theirs[i] && again[2] == *theirs[i];
                const bool oursSeen = again[0] == ours[i] || again[1] == ours[i] || again[2] == ours[i];
                if (!pdalAgrees && oursSeen) {
                    ++pdalUnstable;
                    std::cout << "[ pdal parity ] " << path << " " << cases[i].name
                              << ": PDAL gave different point sets across runs; ours equals a PDAL run\n";
                    theirs[i] = ours[i];
                }
            }
            EXPECT_EQ(ours[i].count, theirs[i]->count) << path << " " << cases[i].name;
            EXPECT_TRUE(ours[i] == *theirs[i]) << path << " " << cases[i].name;
            ++compared;
            nonEmpty += ours[i].count > 0;
            pointsCompared += ours[i].count;
        }
        std::cout << "[ pdal parity ] " << path << ": " << compared << " cases equal to readers.copc (" << nonEmpty
                  << " non-empty, " << pointsCompared << " points), " << skips << " skipped (> 12 M points), "
                  << pdalUnstable << " with unstable PDAL answers\n";
    }
    std::error_code ec;
    fs::remove_all(TmpDir(), ec);
}

}  // namespace
}  // namespace ddbt::copc
