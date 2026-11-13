// SPDX-License-Identifier: MPL-2.0
//
// HierarchyIndex: Morton order, PDAL's depthEnd and node-bound arithmetic, selection against a
// brute-force replica of readers.copc's traversal (synthetic octrees with child pages, random
// windows / depths, 2-D and 3-D, and the corpus), eager vs lazy pages over a range-callback
// source (pages read only when reached, once, also under concurrent queries), and malformed
// hierarchies.

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <map>
#include <mutex>
#include <random>
#include <set>
#include <thread>
#include <tuple>
#include <vector>

#include "copc/copc/copc_file.h"
#include "copc/copc/hierarchy.h"
#include "copc_test_util.h"
#include "core/io/byte_source.h"

namespace ddbt::copc {
namespace {

using testing::BuildCopcBytes;
using testing::SynthCopc;
using testing::SynthNode;

uint64_t Morton64(const NodeKey& k) {
    uint64_t m = 0;
    for (int b = 0; b < 21; ++b) {
        m |= ((static_cast<uint64_t>(k.x) >> b) & 1u) << (3 * b);
        m |= ((static_cast<uint64_t>(k.y) >> b) & 1u) << (3 * b + 1);
        m |= ((static_cast<uint64_t>(k.z) >> b) & 1u) << (3 * b + 2);
    }
    return m;
}

TEST(CopcHierarchy, MortonLessMatchesInterleavedCodes) {
    std::mt19937 rng(3);
    for (int i = 0; i < 200000; ++i) {
        const int d = static_cast<int>(rng() % 22);
        const uint32_t w = 1u << d;
        NodeKey a{d, static_cast<int32_t>(rng() % w), static_cast<int32_t>(rng() % w), static_cast<int32_t>(rng() % w)};
        NodeKey b{d, static_cast<int32_t>(rng() % w), static_cast<int32_t>(rng() % w), static_cast<int32_t>(rng() % w)};
        if (i % 7 == 0) b = a;
        ASSERT_EQ(MortonLess(a, b), Morton64(a) < Morton64(b)) << i;
    }
    // Children come in direction order after their parent's earlier siblings' children.
    const NodeKey p{3, 5, 2, 6};
    for (int dir = 0; dir < 7; ++dir) EXPECT_TRUE(MortonLess(p.Child(dir), p.Child(dir + 1)));
    EXPECT_TRUE(DepthMortonLess(NodeKey{1, 1, 1, 1}, NodeKey{2, 0, 0, 0}));
}

TEST(CopcHierarchy, DepthEndForResolutionIsPdals) {
    EXPECT_EQ(DepthEndForResolution(10, 0), 0);
    EXPECT_EQ(DepthEndForResolution(10, -1), 0);
    EXPECT_EQ(DepthEndForResolution(10, 20), 1);   // max(1, ceil(-1) + 1 = 0)
    EXPECT_EQ(DepthEndForResolution(10, 10), 1);   // ceil(0) + 1
    EXPECT_EQ(DepthEndForResolution(10, 5), 2);
    EXPECT_EQ(DepthEndForResolution(10, 4.9), 3);  // ceil(1.03) + 1
    EXPECT_EQ(DepthEndForResolution(36.37117188, 36.37117188 / 500), 10);
    for (double s : {0.7, 6.8, 36.4}) {
        for (double r = s / 1000; r < 4 * s; r *= 1.37) {
            const int ref = std::max(1, static_cast<int>(std::ceil(std::log2(s / r))) + 1);
            EXPECT_EQ(DepthEndForResolution(s, r), ref);
        }
    }
}

// The PDAL Key::bounds arithmetic, with the products forced to round before the add.
Box3 PdalBounds(const HierarchyIndex::Cube& root, const NodeKey& k) {
    const int width = static_cast<int>(std::pow(2, k.d));
    const double cellWidth = (root.maxX - root.minX) / width;
    auto at = [&](double base, int i) {
        volatile double p = cellWidth * i;
        return base + p;
    };
    Box3 b;
    b.minX = k.x == 0 ? root.minX : at(root.minX, k.x);
    b.maxX = k.x == width - 1 ? root.maxX : at(root.minX, k.x + 1);
    b.minY = k.y == 0 ? root.minY : at(root.minY, k.y);
    b.maxY = k.y == width - 1 ? root.maxY : at(root.minY, k.y + 1);
    b.minZ = k.z == 0 ? root.minZ : at(root.minZ, k.z);
    b.maxZ = k.z == width - 1 ? root.maxZ : at(root.minZ, k.z + 1);
    return b;
}

// A synthetic octree: random nodes down to `depth` (each node's ancestors present, some with
// 0 points), points inside each node's cube.
SynthCopc RandomOctree(uint32_t seed, int depth, int pageDepth, size_t maxNodes = 400) {
    std::mt19937 rng(seed);
    SynthCopc s;
    s.format = 7;
    s.center[0] = 1000.25;
    s.center[1] = -2000.5;
    s.center[2] = 300.125;
    s.halfSize = 512.375;
    s.offset[0] = 1000;
    s.offset[1] = -2000;
    s.offset[2] = 300;
    s.spacing = s.halfSize * 2 / 128;
    s.pageDepth = pageDepth;
    std::set<std::tuple<int, int, int, int>> keys{{0, 0, 0, 0}};
    std::vector<NodeKey> frontier{{0, 0, 0, 0}};
    while (!frontier.empty() && keys.size() < maxNodes) {
        const NodeKey k = frontier.back();
        frontier.pop_back();
        if (k.d >= depth) continue;
        for (int dir = 0; dir < 8; ++dir) {
            if (rng() % 3 != 0) continue;
            const NodeKey c = k.Child(dir);
            if (keys.insert({c.d, c.x, c.y, c.z}).second) frontier.insert(frontier.begin(), c);
        }
    }
    for (const auto& [d, x, y, z] : keys) {
        SynthNode n;
        n.key = {d, x, y, z};
        n.count = rng() % 5 == 0 ? 0u : static_cast<uint32_t>(1u + rng() % 40);
        if (n.count) {
            testing::SynthOptions o;
            o.format = 7;
            o.seed = static_cast<uint32_t>(rng());
            // Points in the node's cube (integers of scale 0.01 relative to the offset).
            const double cw = 2 * s.halfSize / (1 << d);
            for (int a = 0; a < 3; ++a) {
                const int idx = a == 0 ? x : a == 1 ? y : z;
                const double lo = s.center[a] - s.halfSize + cw * idx;
                o.base[a] = static_cast<int32_t>(std::lround((lo + cw / 2 - s.offset[a]) / s.scale[a]));
            }
            o.spread = static_cast<int32_t>(cw / 2 / 0.01) - 1;
            n.records = testing::SynthRecords(n.count, o);
        }
        s.nodes.push_back(std::move(n));
    }
    return s;
}

// readers.copc's traversal as written in CopcReader::loadHierarchy, on a key -> count map.
std::set<std::tuple<int, int, int, int>> PdalSelect(const std::map<std::tuple<int, int, int, int>, int32_t>& entries,
                                                    const HierarchyIndex::Cube& root, const SelectQuery& q) {
    std::set<std::tuple<int, int, int, int>> out;
    auto passes = [&](const NodeKey& k) {
        return (q.depthEnd == 0 || k.d < q.depthEnd) && PdalBounds(root, k).Overlaps(q.box);
    };
    std::vector<NodeKey> todo;
    if (passes({0, 0, 0, 0}) && entries.count({0, 0, 0, 0})) todo.push_back({0, 0, 0, 0});
    while (!todo.empty()) {
        const NodeKey k = todo.back();
        todo.pop_back();
        if (entries.at({k.d, k.x, k.y, k.z}) > 0) out.insert({k.d, k.x, k.y, k.z});
        for (int dir = 0; dir < 8; ++dir) {
            const NodeKey c = k.Child(dir);
            if (passes(c) && entries.count({c.d, c.x, c.y, c.z})) todo.push_back(c);
        }
    }
    return out;
}

std::set<std::tuple<int, int, int, int>> Keys(const Selection& s) {
    std::set<std::tuple<int, int, int, int>> out;
    for (const HierarchyNode* n : s.nodes) out.insert({n->key.d, n->key.x, n->key.y, n->key.z});
    return out;
}

SelectQuery RandomQuery(std::mt19937& rng, const Box3& cube, bool threeD) {
    std::uniform_real_distribution<double> u(-0.1, 1.1);
    auto axis = [&](double lo, double hi, double& a, double& b) {
        a = lo + (hi - lo) * u(rng);
        b = a + (hi - lo) * std::pow(10.0, -3.0 * std::uniform_real_distribution<double>(0, 1)(rng));
    };
    SelectQuery q;
    axis(cube.minX, cube.maxX, q.box.minX, q.box.maxX);
    axis(cube.minY, cube.maxY, q.box.minY, q.box.maxY);
    if (threeD) axis(cube.minZ, cube.maxZ, q.box.minZ, q.box.maxZ);
    q.depthEnd = static_cast<int>(rng() % 8);
    return q;
}

TEST(CopcHierarchy, SelectEqualsReadersCopcTraversal) {
    for (uint32_t seed = 1; seed <= 6; ++seed) {
        for (int pageDepth : {0, 1, 2, 3}) {
            const SynthCopc spec = RandomOctree(seed, 6, pageDepth);
            const std::vector<uint8_t> bytes = BuildCopcBytes(spec);
            auto file = CopcFile::Open(std::make_shared<io::MemorySource>(bytes, "synthetic"));
            ASSERT_TRUE(file.ok()) << file.status().ToString();
            const HierarchyIndex& h = (*file)->hierarchy();
            std::map<std::tuple<int, int, int, int>, int32_t> entries;
            uint64_t total = 0;
            for (const SynthNode& n : spec.nodes) {
                entries[{n.key.d, n.key.x, n.key.y, n.key.z}] = static_cast<int32_t>(n.count);
                total += n.count;
            }
            const HierarchyStats st = h.Stats();
            EXPECT_EQ(st.points, total);
            auto all = h.All();
            ASSERT_TRUE(all.ok());
            EXPECT_EQ(all->points, total);

            const Box3 cube = (*file)->RootCube();
            // Node bounds equal PDAL's for every node.
            for (const auto& [k, c] : entries) {
                const NodeKey key{std::get<0>(k), std::get<1>(k), std::get<2>(k), std::get<3>(k)};
                const Box3 a = h.NodeBounds(key), b = PdalBounds(h.RootCube(), key);
                ASSERT_TRUE(a.minX == b.minX && a.maxX == b.maxX && a.minY == b.minY && a.maxY == b.maxY &&
                            a.minZ == b.minZ && a.maxZ == b.maxZ);
            }
            std::mt19937 rng(seed * 100 + static_cast<uint32_t>(pageDepth));
            for (int i = 0; i < 300; ++i) {
                SelectQuery q = RandomQuery(rng, cube, i % 3 == 0);
                if (i % 10 == 0) {  // a window ending exactly on node boundaries
                    const Box3 nb = h.NodeBounds({2, 1, 2, 0});
                    q.box = Box3::Xy(nb.minX, nb.minY, nb.maxX, nb.maxY);
                }
                auto sel = h.Select(q);
                ASSERT_TRUE(sel.ok());
                ASSERT_EQ(Keys(*sel), PdalSelect(entries, h.RootCube(), q)) << "seed " << seed << " query " << i;
                uint64_t pts = 0;
                for (const HierarchyNode* n : sel->nodes) pts += static_cast<uint64_t>(n->pointCount);
                EXPECT_EQ(pts, sel->points);
                EXPECT_TRUE(std::is_sorted(sel->nodes.begin(), sel->nodes.end(),
                                           [](const HierarchyNode* a, const HierarchyNode* b) {
                                               return DepthMortonLess(a->key, b->key);
                                           }));
                auto byFile = h.Select(q, NodeOrder::kFileOrder);
                ASSERT_TRUE(byFile.ok());
                EXPECT_EQ(Keys(*byFile), Keys(*sel));
                EXPECT_TRUE(std::is_sorted(byFile->nodes.begin(), byFile->nodes.end(),
                                           [](const HierarchyNode* a, const HierarchyNode* b) { return a->offset < b->offset; }));
            }
        }
    }
}

// A source that counts reads per offset (a stand-in for HTTP range requests).
struct CountingRemote {
    std::shared_ptr<std::vector<uint8_t>> bytes;
    std::shared_ptr<std::mutex> mutex = std::make_shared<std::mutex>();
    std::shared_ptr<std::map<uint64_t, int>> reads = std::make_shared<std::map<uint64_t, int>>();
    std::shared_ptr<io::ByteSource> Source() {
        auto b = bytes;
        auto m = mutex;
        auto r = reads;
        return std::make_shared<io::RangeCallbackSource>(
            b->size(),
            [b, m, r](uint64_t off, std::span<uint8_t> out) -> Status {
                if (off + out.size() > b->size()) return OutOfRange("past the end");
                std::memcpy(out.data(), b->data() + off, out.size());
                std::lock_guard<std::mutex> lock(*m);
                ++(*r)[off];
                return Status();
            },
            "remote");
    }
    int ReadsAt(uint64_t off) {
        std::lock_guard<std::mutex> lock(*mutex);
        auto it = reads->find(off);
        return it == reads->end() ? 0 : it->second;
    }
};

TEST(CopcHierarchy, LazyPagesOverRangeCallbacks) {
    const SynthCopc spec = RandomOctree(11, 6, 2, 2000);
    CountingRemote remote{std::make_shared<std::vector<uint8_t>>(BuildCopcBytes(spec))};
    auto file = CopcFile::Open(remote.Source());
    ASSERT_TRUE(file.ok()) << file.status().ToString();
    const HierarchyIndex& h = (*file)->hierarchy();
    HierarchyStats st = h.Stats();
    EXPECT_EQ(st.pagesLoaded, 1u) << "lazy for a non-mapped source";
    EXPECT_GT(st.pagesKnown, 2u);

    // A query that stops at depth 2 needs no child page.
    SelectQuery shallow;
    shallow.depthEnd = 2;
    ASSERT_TRUE(h.Select(shallow).ok());
    EXPECT_EQ(h.Stats().pagesLoaded, 1u);

    // A small window loads only the pages it reaches.
    const Box3 nb = h.NodeBounds({2, 0, 0, 0});
    SelectQuery corner;
    corner.box = Box3::Xy(nb.minX + 1, nb.minY + 1, nb.minX + 2, nb.minY + 2);
    auto c = h.Select(corner);
    ASSERT_TRUE(c.ok());
    const uint32_t afterCorner = h.Stats().pagesLoaded;
    EXPECT_LE(afterCorner, 2u);

    // The same results as the eager index, from many threads at once; each page read once.
    auto eager = CopcFile::Open(std::make_shared<io::MemorySource>(*remote.bytes, "eager"));
    ASSERT_TRUE(eager.ok());
    std::vector<SelectQuery> queries;
    std::mt19937 rng(5);
    for (int i = 0; i < 64; ++i) queries.push_back(RandomQuery(rng, (*file)->RootCube(), i % 2 == 0));
    std::atomic<int> mismatches{0};
    std::vector<std::thread> threads;
    for (int t = 0; t < 8; ++t)
        threads.emplace_back([&, t] {
            for (size_t i = static_cast<size_t>(t); i < queries.size(); i += 2) {
                auto a = h.Select(queries[i]);
                auto b = (*eager)->hierarchy().Select(queries[i]);
                if (!a.ok() || !b.ok() || Keys(*a) != Keys(*b)) mismatches.fetch_add(1);
            }
        });
    for (auto& t : threads) t.join();
    EXPECT_EQ(mismatches.load(), 0);
    ASSERT_TRUE(h.LoadAll().ok());
    st = h.Stats();
    EXPECT_EQ(st.pagesLoaded, st.pagesKnown);
    EXPECT_EQ(st.points, (*eager)->hierarchy().Stats().points);
    for (const auto& [off, n] : *remote.reads) EXPECT_EQ(n, 1) << "offset " << off << " read " << n << " times";
}

TEST(CopcHierarchy, MalformedHierarchies) {
    const SynthCopc spec = RandomOctree(2, 4, 2);
    const std::vector<uint8_t> good = BuildCopcBytes(spec);
    using testing::Get;
    using testing::Put;
    const uint64_t rootAt = Get<uint64_t>(good.data() + 375 + 54 + 40);
    const uint64_t rootSize = Get<uint64_t>(good.data() + 375 + 54 + 48);
    auto open = [](std::vector<uint8_t> b, HierarchyLoad load = HierarchyLoad::kEager) {
        CopcOpenOptions o;
        o.hierarchy = load;
        return CopcFile::Open(std::make_shared<io::MemorySource>(std::move(b), "bad"), o);
    };
    ASSERT_TRUE(open(good).ok());
    {
        auto b = good;  // root page without the root key
        Put<int32_t>(b, rootAt, 1);
        Put<int32_t>(b, rootAt + 4, 0);
        // the first entry is 0-0-0-0 in this builder only if sorted; find and break it
        for (uint64_t e = rootAt; e < rootAt + rootSize; e += 32)
            if (Get<int32_t>(b.data() + e) == 0) Put<int32_t>(b, e, 9);
        EXPECT_EQ(open(b).code(), StatusCode::kCorrupt);
    }
    {
        auto b = good;  // root page size not a multiple of 32
        Put<uint64_t>(b, 375 + 54 + 48, rootSize - 1);
        EXPECT_EQ(open(b).code(), StatusCode::kCorrupt);
    }
    {
        auto b = good;  // root page past the end
        Put<uint64_t>(b, 375 + 54 + 40, b.size());
        EXPECT_EQ(open(b).code(), StatusCode::kCorrupt);
    }
    {
        auto b = good;  // a link to a page outside the file: corrupt when reached (eager: at open)
        for (uint64_t e = rootAt; e < rootAt + rootSize; e += 32)
            if (Get<int32_t>(b.data() + e + 28) == -1) Put<uint64_t>(b, e + 16, b.size() + 100);
        EXPECT_EQ(open(b).code(), StatusCode::kCorrupt);
    }
    {
        auto b = good;  // a child page whose entries lack its own key
        for (uint64_t e = rootAt; e < rootAt + rootSize; e += 32) {
            if (Get<int32_t>(b.data() + e + 28) != -1) continue;
            const uint64_t page = Get<uint64_t>(b.data() + e + 16);
            Put<int32_t>(b, page, 7);  // first entry of the child page: depth 7 (another key)
            Put<int32_t>(b, page + 4, 0);
            break;
        }
        auto lazy = open(b, HierarchyLoad::kLazy);
        ASSERT_TRUE(lazy.ok()) << "lazy: the child page is not read at open";
        EXPECT_FALSE((*lazy)->hierarchy().All().ok());
        EXPECT_FALSE(open(b).ok());
    }
    {
        // Entries with out-of-range keys are ignored, as unreachable.
        auto b = good;
        for (uint64_t e = rootAt; e < rootAt + rootSize; e += 32) {
            if (Get<int32_t>(b.data() + e) == 1) {
                Put<int32_t>(b, e + 4, 5);  // x = 5 at depth 1
                break;
            }
        }
        EXPECT_TRUE(open(b).ok());
    }
}

// Indexed selection against the brute-force replica on the corpus files.
TEST(CopcHierarchy, CorpusSelectEqualsReadersCopcTraversal) {
    const auto files = testing::CorpusCopcFiles();
    if (files.empty()) GTEST_SKIP() << "no corpus COPC files";
    for (const std::string& path : files) {
        auto file = CopcFile::Open(path);
        ASSERT_TRUE(file.ok()) << file.status().ToString();
        const HierarchyIndex& h = (*file)->hierarchy();
        auto all = h.All();
        ASSERT_TRUE(all.ok());
        EXPECT_EQ(all->points, (*file)->header().pointCount) << path;
        std::map<std::tuple<int, int, int, int>, int32_t> entries;
        // Every key of the loaded pages (structural nodes too): through Select with no filter
        // only data nodes come back, so rebuild from a depth-first walk of all keys instead.
        SelectQuery everything;
        auto sel = h.Select(everything);
        ASSERT_TRUE(sel.ok());
        for (const HierarchyNode* n : sel->nodes) entries[{n->key.d, n->key.x, n->key.y, n->key.z}] = n->pointCount;
        // Ancestors without points (if any) as 0-point entries.
        for (auto it = entries.begin(); it != entries.end(); ++it) {
            NodeKey k{std::get<0>(it->first), std::get<1>(it->first), std::get<2>(it->first), std::get<3>(it->first)};
            while (k.d > 0) {
                k = k.Parent();
                entries.emplace(std::make_tuple(k.d, k.x, k.y, k.z), 0);
            }
        }
        std::mt19937 rng(77);
        for (int i = 0; i < 200; ++i) {
            const SelectQuery q = RandomQuery(rng, (*file)->RootCube(), i % 4 == 0);
            auto s = h.Select(q);
            ASSERT_TRUE(s.ok());
            ASSERT_EQ(Keys(*s), PdalSelect(entries, h.RootCube(), q)) << path << " query " << i;
        }
        const HierarchyStats st = h.Stats();
        std::cout << "[ hierarchy ] " << path << ": " << st.pagesLoaded << " pages, " << st.nodes << " entries, "
                  << st.dataNodes << " data nodes, depth " << st.maxDepth << "\n";
    }
}

}  // namespace
}  // namespace ddbt::copc
