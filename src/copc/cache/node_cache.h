// SPDX-License-Identifier: MPL-2.0
//
// NodeCache: the process-wide cache of decoded COPC nodes the tiler reads from.
//
//   auto& cache = NodeCache::Global();
//   std::vector<NodeCache::NodePtr> nodes;
//   cache.Fetch(*file, selection.nodes, nodes);     // misses decoded in parallel, single flight
//
// - Entries are compact nodes (copc::CompactNode: uint16 offsets or int32 values per axis, RGBA8
//   with the per-cloud 16-bit decision; 6-16 bytes per point) keyed by (CopcFile::id(), node
//   id = chunk offset). A file that changes on disk is opened again (a new CopcFile with a new
//   id), so its old nodes are never served and age out of the LRU.
// - One core ShardedLru (16 shards) with a byte budget: DDBT_COPC_NODE_CACHE_MB megabytes
//   (default 512; 0 keeps nothing, every request decodes). An entry costs CompactNode::Bytes().
// - Concurrent misses on one node decode it once (the others wait, helping the pool). The misses
//   of one Fetch are decoded in parallel on the default pool at interactive priority; the
//   calling thread takes part.
// - Each entry also records whether a raw colour component of the node was above 255 (the
//   compat tiler needs it to reproduce DroneDB's per-tile 16-bit decision exactly).
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include "copc/copc/copc_file.h"
#include "copc/copc/hierarchy.h"
#include "copc/copc/node_decoder.h"
#include "core/cache/cache_stats.h"

namespace ddbt::copc::cache {

/// A decoded node as the cache keeps it.
struct CachedNode {
    CompactNode points;
    /// A red, green or blue value of the node was above 255 before the 8 / 16-bit reduction.
    bool over255 = false;
};

class NodeCache {
public:
    using NodePtr = std::shared_ptr<const CachedNode>;

    /// The process-wide cache (budget from DDBT_COPC_NODE_CACHE_MB, read once).
    static NodeCache& Global();

    explicit NodeCache(size_t budgetBytes, size_t shards = 16);
    ~NodeCache();
    NodeCache(const NodeCache&) = delete;
    NodeCache& operator=(const NodeCache&) = delete;

    /// The node, from the cache or decoded now (single flight). Throws std::runtime_error when
    /// the node cannot be decoded (nothing is cached then).
    NodePtr Get(const CopcFile& file, const HierarchyNode& node);

    /// The nodes of `nodes`, in the same order, into `out`. Misses are decoded in parallel on
    /// exec::DefaultPool() (interactive priority, the calling thread helps). Throws like Get.
    void Fetch(const CopcFile& file, std::span<const HierarchyNode* const> nodes, std::vector<NodePtr>& out);

    /// Decodes a node (XYZ + RGB layers) into its cached form, without caching it.
    static Expected<CachedNode> Decode(const CopcFile& file, const HierarchyNode& node);

    ::ddbt::cache::CacheStats Stats() const;
    void SetBudget(size_t bytes);
    size_t BudgetBytes() const;
    void Clear();

    /// Budget of Global(): DDBT_COPC_NODE_CACHE_MB megabytes, default 512.
    static size_t BudgetFromEnvironment();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace ddbt::copc::cache
