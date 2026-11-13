// SPDX-License-Identifier: MPL-2.0
//
// The level of detail of the COPC builder: a deterministic, bottom-up octree thinning whose
// levels hold about as many points as Untwine's (per-depth counts within 8 % on the test corpus).
//
//   - Leaves. The cloud's leaf level L is the first depth (at least 1) at which its points,
//     divided by 2 for the first level and by 8 for each further one, are at most `leafMax`
//     (Untwine's level: its cube test, `side >= max side` per axis after the cube was widened
//     by one scale step, holds for one axis only at the first level, so it halves the count
//     there). A node of depth L
//     holding c points of the cloud (its subtree) is a leaf when c <= leafMax; otherwise its
//     region is cut m levels deeper, m the smallest with c <= leafMax x 4^m, and every node of
//     depth L + m in it is a leaf (`RegionLeafDepth`). Never deeper than `maxDepth`. A leaf
//     keeps all its points.
//   - Any other node is built from its children: its candidates are the points its children
//     hold once built. Children holding together fewer than `hoistTotal` points give all of
//     them; a child holding fewer than `hoistMin` gives all of its own. From the other
//     children the node takes one point per occupied cell of a grid of `gridCells`^3 cells over
//     the node (`rootGridCells`^3 at the root): the candidate closest to the cell centre (exact
//     integers in key space; ties: the smaller key, then the smaller record bytes). Both cell
//     counts are even, so a cell never straddles two children.
//   - The children are then final; they are emitted (encoded) in Morton order. The root is
//     emitted last.
//
// The result depends on the points only: a subtree is built the same whether it is a
// partition processed alone or part of a bigger one, so the output does not depend on how the
// build was partitioned (memory budget) nor on the number of threads.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "core/util/status.h"

namespace ddbt::copc::build {

struct LodParams {
    /// Points of a leaf (Untwine's MaxPointsPerNode).
    uint32_t leafMax = 100000;
    /// The cloud's leaf level L: LeafLevel(points, leafMax). Set by the builder.
    int leafLevel = 1;
    /// A child holding fewer points gives all of them to its parent (Untwine's MinimumPoints).
    uint32_t hoistMin = 100;
    /// Children holding fewer points together give all of them (Untwine's MinimumTotalPoints).
    uint32_t hoistTotal = 1500;
    /// Cells per axis of a node's grid (even): Untwine's 128 sqrt(3) / 1.5 = 147.8.
    int gridCells = 148;
    /// Cells per axis of the root's grid (even): Untwine's 128 sqrt(3) = 221.7.
    int rootGridCells = 222;
    /// No node deeper (the 21 key bits leave 7 bits per axis below a node of depth 14).
    int maxDepth = 14;
};

/// The leaf level L of a cloud of `points` points (at least 1).
int LeafLevel(uint64_t points, uint32_t leafMax);

/// Depth of the leaves below a node of depth `params.leafLevel` whose subtree holds `points`.
int RegionLeafDepth(uint64_t points, const LodParams& params);

/// Whether node (depth, subtree points) is a leaf; `regionLeafDepth` is RegionLeafDepth of its
/// ancestor at the leaf level when depth > leafLevel (ignored otherwise).
bool IsLeaf(int depth, uint64_t points, int regionLeafDepth, const LodParams& params);

/// Points of a node that is not final yet, sorted by (key, record bytes).
struct NodePoints {
    int depth = 0;
    uint64_t prefix = 0;
    std::vector<uint64_t> keys;
    std::vector<uint8_t> records;
    /// The node has children in the hierarchy (emitted, possibly with 0 points).
    bool hasChildren = false;
    size_t size() const noexcept { return keys.size(); }
};

/// Receives a final node: its depth, Morton prefix and contiguous records (count may be 0 for a
/// node kept only because it has children).
using EmitFn = std::function<Status(int depth, uint64_t prefix, const uint8_t* records, uint32_t count)>;

/// A record and its key, as the partition sort takes them.
struct SortEntry {
    uint64_t key;
    const uint8_t* rec;
};

/// Sorts `entries` by key (LSD radix, stable), then equal keys by record bytes. `scratch` is
/// resized to entries.size().
void SortEntries(std::vector<SortEntry>& entries, std::vector<SortEntry>& scratch, size_t recordSize, int fixedTopBits);

/// Builds the subtree of partition root (depth, prefix) from its points (`entries`, keys within
/// the root's prefix; consumed). `regionLeafDepth`: RegionLeafDepth of the root's ancestor at
/// the leaf level when depth > params.leafLevel (else ignored). Emits every node below the root
/// through `emit` (in the global emission order) and returns the root's points in `root`.
/// `afterGather` runs once the records were copied out of `entries` (the source may be freed).
Status BuildPartition(int depth, uint64_t prefix, int regionLeafDepth, std::vector<SortEntry>& entries,
                      size_t recordSize, const LodParams& params, const EmitFn& emit, NodePoints& root,
                      const std::function<void()>& afterGather = {});

/// Builds node (depth, prefix) above the partitions (an inner node) from its children's points
/// (`children[c]` for child direction c; null or empty when absent). Emits the children (in
/// Morton order) and returns the node's points in `out`. The children's points are consumed.
Status BuildTopNode(int depth, uint64_t prefix, const std::array<NodePoints*, 8>& children, size_t recordSize,
                    const LodParams& params, const EmitFn& emit, NodePoints& out);

/// The winners among the sorted keys of one node of depth `depth` (they share its prefix) on a
/// grid of `cells`^3 cells over the node: win[i] = 1 for the candidate closest to the centre of
/// each occupied cell, the first one on ties (win must be zeroed). `scratch` holds the grid
/// (cells^3 entries, all ~0u between calls; grown as needed). Exposed for tests.
void SelectWinners(const uint64_t* keys, size_t n, int depth, int cells, uint8_t* win, std::vector<uint32_t>& scratch);

/// Bytes of the grid scratch of SelectWinners for `cells` cells per axis.
inline size_t GridScratchBytes(int cells) noexcept {
    return size_t(cells) * size_t(cells) * size_t(cells) * sizeof(uint32_t);
}

}  // namespace ddbt::copc::build
