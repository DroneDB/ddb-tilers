// SPDX-License-Identifier: MPL-2.0
//
// HierarchyIndex: the COPC octree as flattened arrays, one per hierarchy page, each sorted by
// (depth, Morton code) with an 8-bit child mask and the index of the first child per node, so
// children are found without hashing and a query descends only into nodes it overlaps:
// O(selected + boundary) instead of the linear scan with a pow() per node of today's tiler.
//
//   const HierarchyIndex& h = file->hierarchy();
//   auto sel = h.Select({window, depthEnd}, NodeOrder::kDepthMorton);   // Expected<Selection>
//   for (const HierarchyNode* n : sel->nodes) ... n->key, n->offset, n->pointCount
//
// Node bounds use PDAL's arithmetic (io/private/copc/Key.hpp Key::bounds, also DroneDB's
// copc_utils.cpp nodeBounds) with the cell width of each depth precomputed: the same nodes as
// readers.copc are selected for the same bounds and resolution, bit for bit. Pages are loaded
// eagerly at open (local files) or lazily on the first query that reaches them (remote
// sources); loading is thread-safe and each page is read once.
#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

#include "core/io/byte_source.h"
#include "core/util/status.h"

namespace ddbt::copc {

/// Deepest octree level accepted (keys deeper than this are ignored, as unreachable).
inline constexpr int kMaxDepth = 30;

struct NodeKey {
    int32_t d = 0, x = 0, y = 0, z = 0;
    friend bool operator==(const NodeKey&, const NodeKey&) = default;
    NodeKey Child(int dir) const noexcept {
        return {d + 1, (x << 1) | (dir & 1), (y << 1) | ((dir >> 1) & 1), (z << 1) | ((dir >> 2) & 1)};
    }
    NodeKey Parent() const noexcept { return {d - 1, x >> 1, y >> 1, z >> 1}; }
};

/// Order of (x, y, z) in the Morton (Z-order) curve, z the most significant axis of each bit
/// triple, as COPC child numbers (dir = x | y << 1 | z << 2). Works at every depth (no
/// 64-bit code is formed).
bool MortonLess(const NodeKey& a, const NodeKey& b) noexcept;
/// (depth, Morton) order.
inline bool DepthMortonLess(const NodeKey& a, const NodeKey& b) noexcept {
    return a.d != b.d ? a.d < b.d : MortonLess(a, b);
}

/// a * b + c with two roundings, never fused into an FMA whatever the compiler flags (LTO
/// included): PDAL computes node bounds and point coordinates this way (its builds do not
/// contract), and a selection equal to readers.copc's needs the same doubles.
inline double MulAddUnfused(double a, double b, double c) noexcept {
    double p = a * b;
#if defined(__GNUC__) && (defined(__x86_64__) || defined(__i386__))
    __asm__("" : "+x"(p));
#elif defined(__GNUC__) && defined(__aarch64__)
    __asm__("" : "+w"(p));
#elif defined(__GNUC__)
    volatile double v = p;
    p = v;
#endif
    return p + c;
}

struct Box3 {
    double minX = 0, minY = 0, minZ = 0, maxX = 0, maxY = 0, maxZ = 0;
    /// PDAL BOX3D::overlaps (closed intervals).
    bool Overlaps(const Box3& o) const noexcept {
        return minX <= o.maxX && maxX >= o.minX && minY <= o.maxY && maxY >= o.minY && minZ <= o.maxZ &&
               maxZ >= o.minZ;
    }
    /// PDAL BOX3D::contains(x, y, z) (closed intervals).
    bool Contains(double x, double y, double z) const noexcept {
        return minX <= x && x <= maxX && minY <= y && y <= maxY && minZ <= z && z <= maxZ;
    }
    /// A 2-D window: z spans [lowest, max] as PDAL sets for 2-D `bounds`.
    static Box3 Xy(double minX, double minY, double maxX, double maxY) noexcept {
        return {minX, minY, std::numeric_limits<double>::lowest(), maxX, maxY, std::numeric_limits<double>::max()};
    }
};

/// readers.copc's point test: the point (LAS integers) scaled as PDAL scales it
/// (int * scale + offset, unfused) lies in `box` (closed intervals).
struct PointFilter {
    Box3 box;
    double scale[3] = {1, 1, 1};
    double offset[3] = {0, 0, 0};
    bool Contains(int32_t x, int32_t y, int32_t z) const noexcept {
        return box.Contains(MulAddUnfused(x, scale[0], offset[0]), MulAddUnfused(y, scale[1], offset[1]),
                            MulAddUnfused(z, scale[2], offset[2]));
    }
};

/// One hierarchy entry. Data nodes have pointCount >= 0 (0: a node without points whose
/// children may have some); pointCount == -1 marks a link to a child page (internal).
struct HierarchyNode {
    NodeKey key;
    uint64_t offset = 0;     // file offset of the LAZ chunk (of the page for links)
    int32_t byteSize = 0;    // chunk (page) bytes
    int32_t pointCount = 0;
    uint32_t firstChild = 0; // index (same page) of the child with the lowest direction
    uint8_t childMask = 0;   // bit dir: child `dir` exists (data node or link)
    uint32_t link = 0;       // links: index of the page slot in the owning page

    bool IsLink() const noexcept { return pointCount == -1; }
    /// Stable identity of a data node within its file (the chunk offset is unique).
    uint64_t Id() const noexcept { return offset; }
};

/// PDAL's depthEnd for a resolution: max(1, ceil(log2(spacing / resolution)) + 1); 0 (no
/// limit) when resolution <= 0.
int DepthEndForResolution(double spacing, double resolution) noexcept;

enum class NodeOrder : uint8_t {
    kDepthMorton,  // (depth, Morton): parents before children, spatially coherent
    kFileOrder,    // by chunk offset (sequential reads)
};

struct SelectQuery {
    Box3 box = Box3::Xy(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
                        std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    int depthEnd = 0;  // nodes with depth < depthEnd; 0: no limit (PDAL: no resolution)
};

struct Selection {
    std::vector<const HierarchyNode*> nodes;  // data nodes with points, in the requested order
    uint64_t points = 0;
    uint32_t visited = 0;  // nodes whose bounds were tested
};

struct HierarchyStats {
    uint32_t pagesLoaded = 0;
    uint32_t pagesKnown = 0;
    uint64_t nodes = 0;      // entries of the loaded pages (data nodes and links)
    uint64_t dataNodes = 0;  // with points
    uint64_t points = 0;     // in the loaded pages
    int maxDepth = 0;
};

class HierarchyIndex {
public:
    struct Cube {
        double minX, minY, minZ, maxX, maxY, maxZ;
    };

    /// Reads the root page [rootOffset, rootOffset + rootSize) of `source` (and every page
    /// when `eager`). The root page must hold the root key 0-0-0-0.
    static Expected<std::unique_ptr<HierarchyIndex>> Load(std::shared_ptr<const io::ByteSource> source,
                                                          uint64_t rootOffset, uint64_t rootSize, Cube rootCube,
                                                          bool eager);
    ~HierarchyIndex();

    /// Nodes overlapping `q.box` (PDAL overlap test on the node cube) with depth < q.depthEnd,
    /// reached from the root by descending only into overlapping nodes, as readers.copc does.
    /// Loads the pages it reaches if needed (errors: I/O, corrupt page).
    Expected<Selection> Select(const SelectQuery& q, NodeOrder order = NodeOrder::kDepthMorton) const;

    /// Every data node with points (loads every page).
    Expected<Selection> All(NodeOrder order = NodeOrder::kDepthMorton) const;

    /// Bounds of a node, PDAL's arithmetic.
    Box3 NodeBounds(const NodeKey& k) const noexcept;
    /// Cell width at depth d (root width / 2^d).
    double CellWidth(int d) const noexcept { return cellWidth_[static_cast<size_t>(d)]; }
    const Cube& RootCube() const noexcept { return root_; }

    /// Loads every page not loaded yet.
    Status LoadAll() const;
    HierarchyStats Stats() const;

    struct Page;

private:
    HierarchyIndex() = default;
    Expected<const Page*> Resolve(const Page& parent, const HierarchyNode& link) const;

    std::shared_ptr<const io::ByteSource> source_;
    Cube root_{};
    double cellWidth_[kMaxDepth + 1] = {};
    std::unique_ptr<Page> rootPage_;
    mutable std::atomic<uint32_t> pagesLoaded_{0};
};

}  // namespace ddbt::copc
