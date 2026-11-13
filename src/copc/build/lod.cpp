// SPDX-License-Identifier: MPL-2.0
#include "copc/build/lod.h"

#include <algorithm>
#include <cstring>
#include <numeric>

#include "copc/build/morton.h"

namespace ddbt::copc::build {

namespace {

constexpr uint32_t kNoCell = ~uint32_t{0};

/// Squared distance, in units of 1 / (2 g) key cells, from the centre of key cell l to the
/// centre of grid cell c (g grid cells over 2^b key cells, per axis).
inline uint64_t GridDistance(uint32_t lx, uint32_t ly, uint32_t lz, uint32_t cx, uint32_t cy, uint32_t cz, int64_t g,
                             int b) noexcept {
    const int64_t dx = g * (2 * int64_t{lx} + 1) - ((2 * int64_t{cx} + 1) << b);
    const int64_t dy = g * (2 * int64_t{ly} + 1) - ((2 * int64_t{cy} + 1) << b);
    const int64_t dz = g * (2 * int64_t{lz} + 1) - ((2 * int64_t{cz} + 1) << b);
    return static_cast<uint64_t>(dx * dx + dy * dy + dz * dz);
}

/// The candidates of one child of a node being built.
struct ChildView {
    bool present = false;
    bool hasChildren = false;
    size_t n = 0;
    const uint64_t* keys = nullptr;
};

/// Which candidates go up to the node: take[c][i] = 1 for the winners and for every point of a
/// hoisted child.
void DecideNode(int depth, const std::array<ChildView, 8>& ch, const LodParams& p, std::vector<uint32_t>& scratch,
                std::array<std::vector<uint8_t>, 8>& take, std::array<uint32_t, 8>& remaining) {
    const int cells = (depth == 0 ? p.rootGridCells : p.gridCells) / 2;  // per axis of a child
    size_t total = 0;
    for (size_t c = 0; c < 8; ++c)
        if (ch[c].present) total += ch[c].n;
    const bool hoistAll = total < p.hoistTotal;
    for (size_t c = 0; c < 8; ++c) {
        take[c].assign(ch[c].n, 0);
        remaining[c] = 0;
        if (!ch[c].present || ch[c].n == 0) continue;
        if (hoistAll || ch[c].n < p.hoistMin) {
            std::fill(take[c].begin(), take[c].end(), uint8_t{1});
            continue;
        }
        SelectWinners(ch[c].keys, ch[c].n, depth + 1, cells, take[c].data(), scratch);
        size_t up = 0;
        for (size_t i = 0; i < ch[c].n; ++i) up += take[c][i];
        remaining[c] = static_cast<uint32_t>(ch[c].n - up);
    }
}

struct NodeState {
    std::vector<uint32_t> pts;  // indices into the partition's sorted arrays, ascending
    bool hasChildren = false;
    bool present = false;
};

class PartitionBuilder {
public:
    PartitionBuilder(const uint64_t* keys, const uint8_t* recs, size_t rs, const LodParams& p, const EmitFn& emit)
        : keys_(keys), recs_(recs), rs_(rs), p_(p), emit_(emit) {}

    Status Build(int d, uint64_t prefix, int regionLeaf, uint32_t lo, uint32_t hi, NodeState& out) {
        out.present = true;
        if (d == p_.leafLevel) regionLeaf = RegionLeafDepth(hi - lo, p_);
        if (IsLeaf(d, hi - lo, regionLeaf, p_)) {
            out.pts.resize(hi - lo);
            std::iota(out.pts.begin(), out.pts.end(), lo);
            out.hasChildren = false;
            return Status();
        }
        std::array<NodeState, 8> child;
        const int shift = 3 * (kKeyBits - (d + 1));
        uint32_t start = lo;
        for (size_t c = 0; c < 8; ++c) {
            const uint64_t childPrefix = prefix * 8 + static_cast<uint64_t>(c);
            const uint64_t endKey = (childPrefix + 1) << shift;
            const uint32_t end = (c == 7) ? hi
                                          : static_cast<uint32_t>(std::lower_bound(keys_ + start, keys_ + hi, endKey) - keys_);
            if (end > start) DDBT_TRY(Build(d + 1, childPrefix, regionLeaf, start, end, child[c]));
            start = end;
        }
        return Select(d, prefix, child, out);
    }

private:
    Status Select(int d, uint64_t prefix, std::array<NodeState, 8>& child, NodeState& out) {
        std::array<ChildView, 8> view;
        for (size_t c = 0; c < 8; ++c) {
            ckeys_[c].resize(child[c].pts.size());
            for (size_t i = 0; i < child[c].pts.size(); ++i) ckeys_[c][i] = keys_[child[c].pts[i]];
            view[c] = {child[c].present, child[c].hasChildren, child[c].pts.size(), ckeys_[c].data()};
        }
        std::array<uint32_t, 8> remaining;
        DecideNode(d, view, p_, scratch_, take_, remaining);
        out.pts.clear();
        for (size_t c = 0; c < 8; ++c)
            for (size_t i = 0; i < child[c].pts.size(); ++i)
                if (take_[c][i]) out.pts.push_back(child[c].pts[i]);
        out.hasChildren = false;
        for (size_t c = 0; c < 8; ++c) {
            if (!child[c].present) continue;
            const uint64_t childPrefix = prefix * 8 + static_cast<uint64_t>(c);
            if (remaining[c] == 0 && !child[c].hasChildren) continue;
            buf_.resize(size_t{remaining[c]} * rs_);
            uint8_t* w = buf_.data();
            for (size_t i = 0; i < child[c].pts.size(); ++i) {
                if (take_[c][i]) continue;
                std::memcpy(w, recs_ + size_t{child[c].pts[i]} * rs_, rs_);
                w += rs_;
            }
            DDBT_TRY(emit_(d + 1, childPrefix, buf_.data(), remaining[c]));
            out.hasChildren = true;
            std::vector<uint32_t>().swap(child[c].pts);
        }
        return Status();
    }

    const uint64_t* keys_;
    const uint8_t* recs_;
    size_t rs_;
    const LodParams& p_;
    const EmitFn& emit_;
    std::array<std::vector<uint64_t>, 8> ckeys_;
    std::array<std::vector<uint8_t>, 8> take_;
    std::vector<uint32_t> scratch_;
    std::vector<uint8_t> buf_;
};

}  // namespace

int LeafLevel(uint64_t points, uint32_t leafMax) {
    // The first level splits the points in 2, the others in 8 (Untwine's).
    int level = 0;
    uint64_t div = 2;
    while (points > leafMax) {
        points /= div;
        div = 8;
        ++level;
    }
    return std::max(level, 1);
}

int RegionLeafDepth(uint64_t points, const LodParams& p) {
    int m = 0;
    uint64_t cap = std::max<uint64_t>(p.leafMax, 1);
    while (points > cap && p.leafLevel + m < p.maxDepth) {
        cap *= 4;
        ++m;
    }
    return std::min(p.leafLevel + m, p.maxDepth);
}

bool IsLeaf(int depth, uint64_t points, int regionLeafDepth, const LodParams& p) {
    if (depth >= p.maxDepth) return true;
    if (depth < p.leafLevel) return false;
    if (depth == p.leafLevel) regionLeafDepth = RegionLeafDepth(points, p);
    return depth >= regionLeafDepth;
}

void SelectWinners(const uint64_t* keys, size_t n, int depth, int cells, uint8_t* win, std::vector<uint32_t>& scratch) {
    const size_t g = static_cast<size_t>(cells);
    if (scratch.size() < g * g * g) scratch.assign(g * g * g, kNoCell);
    const int b = kKeyBits - depth;
    const uint64_t mask = (uint64_t{1} << (3 * b)) - 1;
    const int64_t gi = cells;
    auto cellOf = [&](uint64_t key, uint32_t& lx, uint32_t& ly, uint32_t& lz, uint32_t& cx, uint32_t& cy, uint32_t& cz) {
        MortonDecode(key & mask, lx, ly, lz);
        cx = static_cast<uint32_t>((uint64_t{lx} * g) >> b);
        cy = static_cast<uint32_t>((uint64_t{ly} * g) >> b);
        cz = static_cast<uint32_t>((uint64_t{lz} * g) >> b);
        return static_cast<uint32_t>(cx + g * (cy + g * cz));
    };
    uint32_t lx, ly, lz, cx, cy, cz;
    for (size_t i = 0; i < n; ++i) {
        const uint32_t id = cellOf(keys[i], lx, ly, lz, cx, cy, cz);
        uint32_t& best = scratch[id];
        if (best == kNoCell) {
            best = static_cast<uint32_t>(i);
            continue;
        }
        const uint64_t d = GridDistance(lx, ly, lz, cx, cy, cz, gi, b);
        uint32_t bx, by, bz, ux, uy, uz;
        cellOf(keys[best], bx, by, bz, ux, uy, uz);
        if (d < GridDistance(bx, by, bz, ux, uy, uz, gi, b)) best = static_cast<uint32_t>(i);
    }
    // Mark the winners and leave the grid empty again.
    for (size_t i = 0; i < n; ++i) {
        const uint32_t id = cellOf(keys[i], lx, ly, lz, cx, cy, cz);
        if (scratch[id] == static_cast<uint32_t>(i)) {
            win[i] = 1;
            scratch[id] = kNoCell;
        }
    }
}

void SortEntries(std::vector<SortEntry>& entries, std::vector<SortEntry>& scratch, size_t recordSize, int fixedTopBits) {
    const size_t n = entries.size();
    scratch.resize(n);
    if (n > 1) {
        const int bits = std::max(0, 63 - fixedTopBits);
        SortEntry* src = entries.data();
        SortEntry* dst = scratch.data();
        for (int shift = 0; shift < bits; shift += 8) {
            size_t count[256] = {};
            for (size_t i = 0; i < n; ++i) ++count[(src[i].key >> shift) & 0xFF];
            bool single = false;
            for (int b = 0; b < 256; ++b)
                if (count[b] == n) single = true;
            if (single) continue;
            size_t pos = 0;
            for (int b = 0; b < 256; ++b) {
                const size_t c = count[b];
                count[b] = pos;
                pos += c;
            }
            for (size_t i = 0; i < n; ++i) dst[count[(src[i].key >> shift) & 0xFF]++] = src[i];
            std::swap(src, dst);
        }
        if (src != entries.data()) std::memcpy(entries.data(), src, n * sizeof(SortEntry));
        // Equal keys: by record bytes, so the order does not depend on the arrival order.
        for (size_t i = 0; i < n;) {
            size_t j = i + 1;
            while (j < n && entries[j].key == entries[i].key) ++j;
            if (j - i > 1)
                std::sort(entries.begin() + static_cast<ptrdiff_t>(i), entries.begin() + static_cast<ptrdiff_t>(j),
                          [recordSize](const SortEntry& a, const SortEntry& b) {
                              return std::memcmp(a.rec, b.rec, recordSize) < 0;
                          });
            i = j;
        }
    }
}

Status BuildPartition(int depth, uint64_t prefix, int regionLeafDepth, std::vector<SortEntry>& entries,
                      size_t recordSize, const LodParams& params, const EmitFn& emit, NodePoints& root,
                      const std::function<void()>& afterGather) {
    if (depth > params.leafLevel && regionLeafDepth <= params.leafLevel)
        return InternalError("a partition below the leaf level needs the depth of its region's leaves");
    const size_t n = entries.size();
    {
        std::vector<SortEntry> scratch;
        SortEntries(entries, scratch, recordSize, 3 * depth);
    }
    std::vector<uint64_t> keys(n);
    std::vector<uint8_t> recs(n * recordSize);
    for (size_t i = 0; i < n; ++i) {
        keys[i] = entries[i].key;
        std::memcpy(recs.data() + i * recordSize, entries[i].rec, recordSize);
    }
    std::vector<SortEntry>().swap(entries);
    if (afterGather) afterGather();
    root = NodePoints();
    root.depth = depth;
    root.prefix = prefix;
    if (n == 0) return Status();
    PartitionBuilder b(keys.data(), recs.data(), recordSize, params, emit);
    NodeState st;
    DDBT_TRY(b.Build(depth, prefix, regionLeafDepth, 0, static_cast<uint32_t>(n), st));
    root.hasChildren = st.hasChildren;
    root.keys.resize(st.pts.size());
    root.records.resize(st.pts.size() * recordSize);
    for (size_t i = 0; i < st.pts.size(); ++i) {
        root.keys[i] = keys[st.pts[i]];
        std::memcpy(root.records.data() + i * recordSize, recs.data() + size_t{st.pts[i]} * recordSize, recordSize);
    }
    return Status();
}

Status BuildTopNode(int depth, uint64_t prefix, const std::array<NodePoints*, 8>& children, size_t recordSize,
                    const LodParams& params, const EmitFn& emit, NodePoints& out) {
    std::array<ChildView, 8> view;
    for (size_t c = 0; c < 8; ++c) {
        NodePoints* ch = children[c];
        const bool present = ch != nullptr && (ch->size() > 0 || ch->hasChildren);
        view[c] = {present, ch != nullptr && ch->hasChildren, ch != nullptr ? ch->size() : 0,
                   ch != nullptr ? ch->keys.data() : nullptr};
    }
    std::array<std::vector<uint8_t>, 8> take;
    std::array<uint32_t, 8> remaining;
    std::vector<uint32_t> scratch;
    DecideNode(depth, view, params, scratch, take, remaining);
    std::vector<uint32_t>().swap(scratch);
    out = NodePoints();
    out.depth = depth;
    out.prefix = prefix;
    for (size_t c = 0; c < 8; ++c) {
        if (!view[c].present) continue;
        const NodePoints& ch = *children[c];
        for (size_t i = 0; i < ch.size(); ++i) {
            if (!take[c][i]) continue;
            out.keys.push_back(ch.keys[i]);
            out.records.insert(out.records.end(), ch.records.begin() + static_cast<ptrdiff_t>(i * recordSize),
                               ch.records.begin() + static_cast<ptrdiff_t>((i + 1) * recordSize));
        }
    }
    std::vector<uint8_t> buf;
    for (size_t c = 0; c < 8; ++c) {
        if (!view[c].present) continue;
        NodePoints& ch = *children[c];
        if (remaining[c] == 0 && !ch.hasChildren) {
            ch = NodePoints();
            continue;
        }
        buf.resize(size_t{remaining[c]} * recordSize);
        uint8_t* w = buf.data();
        for (size_t i = 0; i < ch.size(); ++i) {
            if (take[c][i]) continue;
            std::memcpy(w, ch.records.data() + i * recordSize, recordSize);
            w += recordSize;
        }
        DDBT_TRY(emit(depth + 1, prefix * 8 + static_cast<uint64_t>(c), buf.data(), remaining[c]));
        out.hasChildren = true;
        ch = NodePoints();
    }
    return Status();
}

}  // namespace ddbt::copc::build
