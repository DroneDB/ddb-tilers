// SPDX-License-Identifier: MPL-2.0
#include "copc/copc/hierarchy.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <string>

#include "core/util/checked_math.h"
#include "core/util/endian.h"

namespace ddbt::copc {
namespace {

constexpr size_t kEntrySize = 32;
constexpr uint64_t kMaxPageBytes = uint64_t{64} << 20;

// msb(a) < msb(b), msb(0) = -1 (Chan's comparison of the most significant set bits).
inline bool LessMsb(uint32_t a, uint32_t b) noexcept { return a < b && a < (a ^ b); }

std::string KeyString(const NodeKey& k) {
    return std::to_string(k.d) + "-" + std::to_string(k.x) + "-" + std::to_string(k.y) + "-" + std::to_string(k.z);
}

bool ValidKey(const NodeKey& k) noexcept {
    if (k.d < 0 || k.d > kMaxDepth) return false;
    const int64_t width = int64_t{1} << k.d;
    return k.x >= 0 && k.y >= 0 && k.z >= 0 && k.x < width && k.y < width && k.z < width;
}

}  // namespace

bool MortonLess(const NodeKey& a, const NodeKey& b) noexcept {
    const uint32_t dx = static_cast<uint32_t>(a.x ^ b.x);
    const uint32_t dy = static_cast<uint32_t>(a.y ^ b.y);
    const uint32_t dz = static_cast<uint32_t>(a.z ^ b.z);
    if (!LessMsb(dz, dy) && !LessMsb(dz, dx)) return a.z < b.z;
    if (!LessMsb(dy, dx)) return a.y < b.y;
    return a.x < b.x;
}

int DepthEndForResolution(double spacing, double resolution) noexcept {
    if (!(resolution > 0)) return 0;
    // CopcReader::initialize: (std::max)(1, (int)ceil(log2(spacing / resolution)) + 1)
    return std::max(1, static_cast<int>(std::ceil(std::log2(spacing / resolution))) + 1);
}

// One hierarchy page as a flattened octree.
struct HierarchyIndex::Page {
    struct Slot {
        NodeKey key;
        uint64_t offset = 0;
        uint64_t size = 0;
        std::mutex mutex;
        std::atomic<const Page*> page{nullptr};
        std::unique_ptr<Page> owned;
    };

    NodeKey key;
    uint64_t offset = 0;
    uint64_t size = 0;
    uint32_t root = 0;                      // index of the node with `key`
    std::vector<HierarchyNode> nodes;       // sorted by (depth, Morton)
    std::vector<std::unique_ptr<Slot>> slots;  // one per link node (HierarchyNode::link)
};

namespace {

using Page = HierarchyIndex::Page;

// Builds a page from its bytes: entries parsed, invalid keys dropped, duplicates resolved
// first-wins (PDAL's unordered_set insert), sorted, child links set.
Expected<std::unique_ptr<Page>> BuildPage(std::span<const uint8_t> bytes, const NodeKey& key, uint64_t offset,
                                          uint64_t fileSize) {
    auto page = std::make_unique<Page>();
    page->key = key;
    page->offset = offset;
    page->size = bytes.size();
    const size_t n = bytes.size() / kEntrySize;
    std::vector<HierarchyNode> all;
    all.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const uint8_t* e = bytes.data() + i * kEntrySize;
        HierarchyNode node;
        node.key = {LoadLE<int32_t>(e), LoadLE<int32_t>(e + 4), LoadLE<int32_t>(e + 8), LoadLE<int32_t>(e + 12)};
        node.offset = LoadLE<uint64_t>(e + 16);
        node.byteSize = LoadLE<int32_t>(e + 24);
        node.pointCount = LoadLE<int32_t>(e + 28);
        if (!ValidKey(node.key) || node.pointCount < -1) continue;  // unreachable / meaningless
        if (node.IsLink()) {
            if (node.byteSize <= 0 || node.byteSize % static_cast<int32_t>(kEntrySize) != 0 ||
                !RangeInside(node.offset, static_cast<uint64_t>(node.byteSize), fileSize))
                return Corrupt("COPC hierarchy page " + KeyString(key) + ": child page " + KeyString(node.key) +
                               " has an invalid range");
        }
        all.push_back(node);
    }
    // First occurrence wins: stable sort, then drop later duplicates.
    std::stable_sort(all.begin(), all.end(),
                     [](const HierarchyNode& a, const HierarchyNode& b) { return DepthMortonLess(a.key, b.key); });
    all.erase(std::unique(all.begin(), all.end(),
                          [](const HierarchyNode& a, const HierarchyNode& b) { return a.key == b.key; }),
              all.end());
    page->nodes = std::move(all);
    std::vector<HierarchyNode>& nodes = page->nodes;

    auto find = [&nodes](const NodeKey& k) -> int64_t {
        auto it = std::lower_bound(nodes.begin(), nodes.end(), k,
                                   [](const HierarchyNode& a, const NodeKey& b) { return DepthMortonLess(a.key, b); });
        if (it == nodes.end() || !(it->key == k)) return -1;
        return it - nodes.begin();
    };
    const int64_t root = find(key);
    if (root < 0 || nodes[static_cast<size_t>(root)].IsLink())
        return Corrupt("COPC hierarchy page " + KeyString(key) + " has no entry for its own key");
    page->root = static_cast<uint32_t>(root);

    // Children of a node are consecutive in (depth, Morton) order, by direction: one pass over
    // the nodes, each finding its parent by binary search.
    for (size_t i = 0; i < nodes.size(); ++i) {
        HierarchyNode& c = nodes[i];
        if (c.key.d == 0) continue;
        const int64_t p = find(c.key.Parent());
        if (p < 0) continue;  // orphan: unreachable
        HierarchyNode& parent = nodes[static_cast<size_t>(p)];
        if (parent.IsLink()) continue;  // the child belongs to the linked page
        const int dir = (c.key.x & 1) | ((c.key.y & 1) << 1) | ((c.key.z & 1) << 2);
        if (parent.childMask == 0) parent.firstChild = static_cast<uint32_t>(i);
        parent.childMask = static_cast<uint8_t>(parent.childMask | (1u << dir));
    }
    for (HierarchyNode& node : nodes) {
        if (!node.IsLink()) continue;
        auto slot = std::make_unique<Page::Slot>();
        slot->key = node.key;
        slot->offset = node.offset;
        slot->size = static_cast<uint64_t>(node.byteSize);
        node.link = static_cast<uint32_t>(page->slots.size());
        page->slots.push_back(std::move(slot));
    }
    return page;
}

Expected<std::unique_ptr<Page>> ReadPage(const io::ByteSource& source, const NodeKey& key, uint64_t offset,
                                         uint64_t size) {
    if (size % kEntrySize != 0 || size > kMaxPageBytes || !RangeInside(offset, size, source.Size()))
        return Corrupt("COPC hierarchy page " + KeyString(key) + ": invalid range " + std::to_string(offset) + "+" +
                       std::to_string(size));
    std::span<const uint8_t> view = source.View(offset, static_cast<size_t>(size));
    std::vector<uint8_t> owned;
    if (!view.empty()) source.Prefetch(offset, size);  // one read of a cold page (see NodeChunk)
    if (view.empty() && size > 0) {
        owned.resize(static_cast<size_t>(size));
        DDBT_TRY(source.Read(offset, owned));
        view = owned;
    }
    return BuildPage(view, key, offset, source.Size());
}

inline uint32_t ChildIndex(const HierarchyNode& n, int dir) noexcept {
    return n.firstChild + static_cast<uint32_t>(std::popcount(static_cast<unsigned>(n.childMask) & ((1u << dir) - 1u)));
}

void SortSelection(std::vector<const HierarchyNode*>& v, NodeOrder order) {
    if (order == NodeOrder::kFileOrder)
        std::sort(v.begin(), v.end(), [](const HierarchyNode* a, const HierarchyNode* b) { return a->offset < b->offset; });
    else
        std::sort(v.begin(), v.end(),
                  [](const HierarchyNode* a, const HierarchyNode* b) { return DepthMortonLess(a->key, b->key); });
}

}  // namespace

HierarchyIndex::~HierarchyIndex() = default;

Expected<std::unique_ptr<HierarchyIndex>> HierarchyIndex::Load(std::shared_ptr<const io::ByteSource> source,
                                                               uint64_t rootOffset, uint64_t rootSize, Cube rootCube,
                                                               bool eager) {
    std::unique_ptr<HierarchyIndex> h(new HierarchyIndex());
    h->source_ = std::move(source);
    h->root_ = rootCube;
    for (int d = 0; d <= kMaxDepth; ++d) {
        const int width = 1 << d;  // PDAL: int width = pow(2, d)
        h->cellWidth_[d] = (rootCube.maxX - rootCube.minX) / width;
    }
    DDBT_ASSIGN_OR_RETURN(h->rootPage_, ReadPage(*h->source_, NodeKey{}, rootOffset, rootSize));
    h->pagesLoaded_.store(1, std::memory_order_relaxed);
    if (eager) DDBT_TRY(h->LoadAll());
    return h;
}

Box3 HierarchyIndex::NodeBounds(const NodeKey& k) const noexcept {
    // io/private/copc/Key.hpp Key::bounds (root.minx + (cellWidth * x), unfused): the test in
    // each of these avoids rounding where the value is known.
    const int width = 1 << k.d;
    const double cw = cellWidth_[k.d];
    Box3 b;
    b.minX = k.x == 0 ? root_.minX : MulAddUnfused(cw, k.x, root_.minX);
    b.maxX = k.x == width - 1 ? root_.maxX : MulAddUnfused(cw, k.x + 1, root_.minX);
    b.minY = k.y == 0 ? root_.minY : MulAddUnfused(cw, k.y, root_.minY);
    b.maxY = k.y == width - 1 ? root_.maxY : MulAddUnfused(cw, k.y + 1, root_.minY);
    b.minZ = k.z == 0 ? root_.minZ : MulAddUnfused(cw, k.z, root_.minZ);
    b.maxZ = k.z == width - 1 ? root_.maxZ : MulAddUnfused(cw, k.z + 1, root_.minZ);
    return b;
}

Expected<const HierarchyIndex::Page*> HierarchyIndex::Resolve(const Page& parent, const HierarchyNode& link) const {
    Page::Slot& slot = *parent.slots[link.link];
    if (const Page* p = slot.page.load(std::memory_order_acquire)) return p;
    std::lock_guard<std::mutex> lock(slot.mutex);
    if (const Page* p = slot.page.load(std::memory_order_acquire)) return p;
    // Not cached on failure: a remote read may succeed on the next query.
    DDBT_ASSIGN_OR_RETURN(slot.owned, ReadPage(*source_, slot.key, slot.offset, slot.size));
    slot.page.store(slot.owned.get(), std::memory_order_release);
    pagesLoaded_.fetch_add(1, std::memory_order_relaxed);
    return slot.owned.get();
}

Expected<Selection> HierarchyIndex::Select(const SelectQuery& q, NodeOrder order) const {
    Selection sel;
    auto passes = [&](const NodeKey& k) {
        ++sel.visited;
        return (q.depthEnd <= 0 || k.d < q.depthEnd) && NodeBounds(k).Overlaps(q.box);
    };
    struct Item {
        const Page* page;
        uint32_t index;
    };
    std::vector<Item> stack;
    stack.reserve(64);
    if (passes(rootPage_->key)) stack.push_back({rootPage_.get(), rootPage_->root});
    while (!stack.empty()) {
        Item it = stack.back();
        stack.pop_back();
        const HierarchyNode* n = &it.page->nodes[it.index];
        if (n->IsLink()) {
            DDBT_ASSIGN_OR_RETURN(const Page* child, Resolve(*it.page, *n));
            it = {child, child->root};
            n = &child->nodes[child->root];
        }
        if (n->pointCount > 0) {
            sel.nodes.push_back(n);
            sel.points += static_cast<uint64_t>(n->pointCount);
        }
        for (unsigned m = n->childMask; m != 0; m &= m - 1) {
            const int dir = std::countr_zero(m);
            const uint32_t ci = ChildIndex(*n, dir);
            if (passes(it.page->nodes[ci].key)) stack.push_back({it.page, ci});
        }
    }
    SortSelection(sel.nodes, order);
    return sel;
}

Expected<Selection> HierarchyIndex::All(NodeOrder order) const { return Select(SelectQuery{}, order); }

Status HierarchyIndex::LoadAll() const {
    std::vector<const Page*> todo{rootPage_.get()};
    while (!todo.empty()) {
        const Page* p = todo.back();
        todo.pop_back();
        for (const HierarchyNode& n : p->nodes) {
            if (!n.IsLink()) continue;
            DDBT_ASSIGN_OR_RETURN(const Page* child, Resolve(*p, n));
            todo.push_back(child);
        }
    }
    return Status();
}

HierarchyStats HierarchyIndex::Stats() const {
    HierarchyStats s;
    std::vector<const Page*> todo{rootPage_.get()};
    while (!todo.empty()) {
        const Page* p = todo.back();
        todo.pop_back();
        ++s.pagesLoaded;
        ++s.pagesKnown;
        for (const HierarchyNode& n : p->nodes) {
            ++s.nodes;
            s.maxDepth = std::max(s.maxDepth, n.key.d);
            if (n.IsLink()) {
                if (const Page* c = p->slots[n.link]->page.load(std::memory_order_acquire))
                    todo.push_back(c);
                else
                    ++s.pagesKnown;
            } else if (n.pointCount > 0) {
                ++s.dataNodes;
                s.points += static_cast<uint64_t>(n.pointCount);
            }
        }
    }
    return s;
}

}  // namespace ddbt::copc
