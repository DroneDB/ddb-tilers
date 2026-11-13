// SPDX-License-Identifier: MPL-2.0
#include "copc/cache/node_cache.h"

#include <cstdlib>
#include <stdexcept>
#include <string>

#include "copc/laz/chunk_decoder.h"
#include "core/cache/sharded_lru.h"
#include "core/exec/task_group.h"
#include "core/exec/thread_pool.h"

namespace ddbt::copc::cache {

namespace {

constexpr size_t kDefaultBudgetMb = 512;

struct Key {
    uint64_t cloud = 0;
    uint64_t node = 0;
    friend bool operator==(const Key&, const Key&) = default;
};

struct KeyHash {
    size_t operator()(const Key& k) const noexcept {
        uint64_t x = k.cloud * 0x9E3779B97F4A7C15ull ^ k.node;
        x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
        x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
        return static_cast<size_t>(x ^ (x >> 31));
    }
};

struct Cost {
    size_t operator()(const CachedNode& n) const noexcept { return n.points.Bytes(); }
};

std::string NodeName(const HierarchyNode& n) {
    return std::to_string(n.key.d) + "-" + std::to_string(n.key.x) + "-" + std::to_string(n.key.y) + "-" +
           std::to_string(n.key.z);
}

}  // namespace

struct NodeCache::Impl {
    explicit Impl(size_t budget, size_t shards) : lru(budget, shards) {}
    ddbt::cache::ShardedLru<Key, CachedNode, KeyHash, Cost> lru;
};

size_t NodeCache::BudgetFromEnvironment() {
    const char* e = std::getenv("DDBT_COPC_NODE_CACHE_MB");
    if (e == nullptr || *e == '\0') return kDefaultBudgetMb << 20;
    char* end = nullptr;
    const unsigned long long mb = std::strtoull(e, &end, 10);
    if (end == e || *end != '\0' || mb > (1ull << 20)) return kDefaultBudgetMb << 20;
    return static_cast<size_t>(mb) << 20;
}

NodeCache& NodeCache::Global() {
    static NodeCache cache(BudgetFromEnvironment());
    return cache;
}

NodeCache::NodeCache(size_t budgetBytes, size_t shards) : impl_(std::make_unique<Impl>(budgetBytes, shards)) {}
NodeCache::~NodeCache() = default;

Expected<CachedNode> NodeCache::Decode(const CopcFile& file, const HierarchyNode& node) {
    bool color16 = false;
    if (file.hasRgb()) {
        DDBT_ASSIGN_OR_RETURN(color16, file.Uses16BitColor());
    }
    thread_local RawPoints raw;
    DDBT_TRY(DecodeNodeRaw(file, node, file.hasRgb() ? laz::kLayersXyzRgb : laz::kLayersXyz, raw));
    CachedNode c;
    if (!raw.r.empty()) {
        const size_t n = raw.size();
        uint16_t any = 0;
        for (size_t i = 0; i < n; ++i) any |= static_cast<uint16_t>(raw.r[i] | raw.g[i] | raw.b[i]);
        c.over255 = any > 255;
    }
    c.points = CompactNode::FromRaw(raw, color16);
    return c;
}

NodeCache::NodePtr NodeCache::Get(const CopcFile& file, const HierarchyNode& node) {
    return impl_->lru.GetOrLoad(Key{file.id(), node.Id()}, [&]() -> CachedNode {
        auto c = Decode(file, node);
        if (!c.ok())
            throw std::runtime_error("Cannot decompress COPC node " + NodeName(node) + ": " +
                                     std::string(c.status().message()));
        return std::move(c).value();
    });
}

void NodeCache::Fetch(const CopcFile& file, std::span<const HierarchyNode* const> nodes, std::vector<NodePtr>& out) {
    out.assign(nodes.size(), nullptr);
    thread_local std::vector<size_t> misses;
    misses.clear();
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (impl_->lru.Contains(Key{file.id(), nodes[i]->Id()}))
            out[i] = Get(file, *nodes[i]);  // a hit (or a load another thread finishes)
        else
            misses.push_back(i);
    }
    if (misses.empty()) return;
    if (misses.size() == 1) {
        out[misses[0]] = Get(file, *nodes[misses[0]]);
        return;
    }
    exec::TaskGroup group(exec::DefaultPool(), exec::Priority::kInteractive);
    // The calling thread decodes the first miss itself while the pool takes the others.
    for (size_t k = 1; k < misses.size(); ++k) {
        const size_t i = misses[k];
        group.Spawn([this, &file, &nodes, &out, i] { out[i] = Get(file, *nodes[i]); });
    }
    const size_t first = misses[0];
    try {
        out[first] = Get(file, *nodes[first]);
    } catch (...) {
        group.Cancel();
        group.Wait();
        throw;
    }
    group.Wait();
}

::ddbt::cache::CacheStats NodeCache::Stats() const { return impl_->lru.Stats(); }
void NodeCache::SetBudget(size_t bytes) { impl_->lru.SetBudget(bytes); }
size_t NodeCache::BudgetBytes() const { return impl_->lru.BudgetBytes(); }
void NodeCache::Clear() { impl_->lru.Clear(); }

}  // namespace ddbt::copc::cache
