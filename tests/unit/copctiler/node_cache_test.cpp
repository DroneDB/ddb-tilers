// SPDX-License-Identifier: MPL-2.0
// NodeCache: decoded nodes equal a direct decode, hits after the first fetch, single flight
// under concurrent fetches, the byte budget, a zero budget, the per-node "above 255" flag, and
// decode errors reported without caching.
#include <gtest/gtest.h>

#include <atomic>
#include <cstring>
#include <filesystem>
#include <thread>
#include <vector>

#include "copc/cache/node_cache.h"
#include "copc/copc/copc_file.h"
#include "synth_cloud.h"

namespace {

using namespace ddbt::copc;
using ddbt::copc::cache::NodeCache;
using ddbt::copc::tiler::test::CloudSpec;

class CopcNodeCacheTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        dir_ = new std::filesystem::path(tiler::test::ScratchDir("nodecache"));
        CloudSpec s;
        s.sixteenBit = false;
        rgb8_ = new std::string(tiler::test::WriteCloud(*dir_, "rgb8", s));
        s.sixteenBit = true;
        s.seed = 8;
        rgb16_ = new std::string(tiler::test::WriteCloud(*dir_, "rgb16", s));
    }
    static void TearDownTestSuite() {
        std::error_code ec;
        std::filesystem::remove_all(*dir_, ec);
        delete dir_;
        delete rgb8_;
        delete rgb16_;
    }
    static std::filesystem::path* dir_;
    static std::string* rgb8_;
    static std::string* rgb16_;
};
std::filesystem::path* CopcNodeCacheTest::dir_ = nullptr;
std::string* CopcNodeCacheTest::rgb8_ = nullptr;
std::string* CopcNodeCacheTest::rgb16_ = nullptr;

bool SameNode(const CompactNode& a, const CompactNode& b) {
    if (a.size() != b.size() || a.hasColor() != b.hasColor()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        for (int ax = 0; ax < 3; ++ax)
            if (a.Coord(ax, i) != b.Coord(ax, i)) return false;
        if (a.hasColor() && a.rgba()[i] != b.rgba()[i]) return false;
    }
    return true;
}

TEST_F(CopcNodeCacheTest, FetchEqualsDirectDecodeAndHits) {
    auto file = CopcFile::Open(*rgb8_).value();
    const auto all = file->hierarchy().All().value();
    NodeCache cache(64 << 20);
    std::vector<NodeCache::NodePtr> nodes;
    cache.Fetch(*file, all.nodes, nodes);
    ASSERT_EQ(nodes.size(), all.nodes.size());
    NodeDecoder dec(file);
    for (size_t i = 0; i < nodes.size(); ++i) {
        auto direct = dec.DecodeCompact(*all.nodes[i]);
        ASSERT_TRUE(direct.ok());
        EXPECT_TRUE(SameNode(nodes[i]->points, *direct)) << i;
        EXPECT_FALSE(nodes[i]->over255);
    }
    const auto s1 = cache.Stats();
    EXPECT_EQ(s1.loads, all.nodes.size());
    std::vector<NodeCache::NodePtr> again;
    cache.Fetch(*file, all.nodes, again);
    for (size_t i = 0; i < nodes.size(); ++i) EXPECT_EQ(again[i].get(), nodes[i].get());
    EXPECT_EQ(cache.Stats().loads, all.nodes.size());
    EXPECT_GT(cache.Stats().bytes, 0u);
}

TEST_F(CopcNodeCacheTest, ConcurrentFetchesDecodeEachNodeOnce) {
    auto file = CopcFile::Open(*rgb16_).value();
    const auto all = file->hierarchy().All().value();
    NodeCache cache(64 << 20);
    std::vector<std::thread> threads;
    std::atomic<int> bad{0};
    std::vector<std::vector<NodeCache::NodePtr>> results(16);
    for (int t = 0; t < 16; ++t)
        threads.emplace_back([&, t] {
            cache.Fetch(*file, all.nodes, results[static_cast<size_t>(t)]);
            for (const auto& n : results[static_cast<size_t>(t)]) bad += n == nullptr;
        });
    for (auto& th : threads) th.join();
    EXPECT_EQ(bad.load(), 0);
    EXPECT_EQ(cache.Stats().loads, all.nodes.size());
    for (int t = 1; t < 16; ++t)
        for (size_t i = 0; i < all.nodes.size(); ++i)
            EXPECT_EQ(results[static_cast<size_t>(t)][i].get(), results[0][i].get());
    // 16-bit cloud: colours above 255 flagged, compact colours are v >> 8.
    bool any = false;
    for (const auto& n : results[0]) any = any || n->over255;
    EXPECT_TRUE(any);
}

TEST_F(CopcNodeCacheTest, BudgetBoundsTheBytesAndZeroKeepsNothing) {
    auto file = CopcFile::Open(*rgb8_).value();
    const auto all = file->hierarchy().All().value();
    {
        NodeCache cache(0);
        std::vector<NodeCache::NodePtr> nodes;
        cache.Fetch(*file, all.nodes, nodes);
        cache.Fetch(*file, all.nodes, nodes);
        EXPECT_EQ(cache.Stats().bytes, 0u);
        EXPECT_EQ(cache.Stats().loads, 2 * all.nodes.size());
        for (const auto& n : nodes) EXPECT_NE(n, nullptr);
    }
    {
        const size_t budget = 256 << 10;
        NodeCache cache(budget, 4);
        std::vector<NodeCache::NodePtr> nodes;
        cache.Fetch(*file, all.nodes, nodes);
        EXPECT_LE(cache.Stats().bytes, budget);
        EXPECT_GT(cache.Stats().evictions + cache.Stats().rejected, 0u);
    }
}

TEST_F(CopcNodeCacheTest, DecodeErrorsAreThrownAndNotCached) {
    auto file = CopcFile::Open(*rgb8_).value();
    const auto all = file->hierarchy().All().value();
    HierarchyNode broken = *all.nodes.back();
    broken.offset = file->source().Size() + 100;  // outside the file
    NodeCache cache(64 << 20);
    EXPECT_THROW(cache.Get(*file, broken), std::runtime_error);
    EXPECT_EQ(cache.Stats().entries, 0u);
}

}  // namespace
