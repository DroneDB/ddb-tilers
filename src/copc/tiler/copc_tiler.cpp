// SPDX-License-Identifier: MPL-2.0
#include "copc/tiler/copc_tiler.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <unordered_set>
#include <vector>

#include "cog/tiler/tile_error.h"
#include "copc/cache/node_cache.h"
#include "copc/copc/hierarchy.h"
#include "copc/copc/node_decoder.h"
#include "copc/laz/chunk_decoder.h"
#include "copc/tiler/cloud.h"
#include "copc/tiler/kernels.h"
#include "copc/tiler/rasterizer.h"
#include "core/cache/sharded_lru.h"
#include "core/crs/polyfit.h"
#include "core/tile/compat.h"

namespace ddbt::copc::tiler {

using cog::tiler::TileError;
using cog::tiler::TileErrorCode;
using cache::NodeCache;

namespace {

constexpr size_t kBatch = 2048;
// Below this many points in the selected nodes, transforming them exactly is cheaper than a fit.
constexpr uint64_t kExactBelow = 1500;
constexpr double kFitMaxResidualPx = 0.01;
constexpr int kFitSamples = 7;

// ---------------------------------------------------------------------------------------------
// Settings

int ReadEnvInt(const char* name, int fallback) {
    const char* e = std::getenv(name);
    if (e == nullptr || *e == '\0') return fallback;
    char* end = nullptr;
    const long v = std::strtol(e, &end, 10);
    if (end == e || *end != '\0' || v < -64 || v > 64) return fallback;
    return static_cast<int>(v);
}

constexpr Mode kDefaultMode = Mode::kHybrid;

std::optional<Mode> ParseModeEnv(const char* e) {
    if (e == nullptr) return std::nullopt;
    return ParseMode(e);
}

std::atomic<int>& ModeSetting() {
    static std::atomic<int> m = [] {
        auto mode = ParseModeEnv(std::getenv("DDBT_COPC_TILER_MODE"));
        if (!mode) mode = ParseModeEnv(std::getenv("DDBT_TILER_MODE"));
        return static_cast<int>(mode.value_or(kDefaultMode));
    }();
    return m;
}

// ---------------------------------------------------------------------------------------------
// Encoded-tile cache

struct TileCacheKey {
    uint64_t cloud = 0;
    int z = 0, x = 0, y = 0, size = 0;
    uint8_t format = 0, mode = 0;
    friend bool operator==(const TileCacheKey&, const TileCacheKey&) = default;
};

struct TileCacheKeyHash {
    size_t operator()(const TileCacheKey& k) const noexcept {
        uint64_t h = k.cloud * 0x9E3779B97F4A7C15ull;
        for (const uint64_t v : {uint64_t(uint32_t(k.z)), uint64_t(uint32_t(k.x)), uint64_t(uint32_t(k.y)),
                                 uint64_t(uint32_t(k.size)), uint64_t(k.format) << 8 | k.mode}) {
            h ^= v + 0x9E3779B97F4A7C15ull + (h << 6) + (h >> 2);
        }
        h = (h ^ (h >> 30)) * 0xBF58476D1CE4E5B9ull;
        return static_cast<size_t>(h ^ (h >> 27));
    }
};

using TileCache = ddbt::cache::ShardedLru<TileCacheKey, std::vector<uint8_t>, TileCacheKeyHash>;

TileCache& EncodedTiles() {
    static TileCache cache([] {
        const char* e = std::getenv("DDBT_COPC_TILE_CACHE_MB");
        size_t mb = 64;
        if (e != nullptr && *e != '\0') {
            char* end = nullptr;
            const unsigned long long v = std::strtoull(e, &end, 10);
            if (end != e && *end == '\0' && v <= (1ull << 20)) mb = static_cast<size_t>(v);
        }
        return mb << 20;
    }());
    return cache;
}

// ---------------------------------------------------------------------------------------------
// Per-thread scratch: buffers kept from tile to tile (no allocation in steady state).

struct Scratch {
    CompatRasterizer compat;
    SplatRasterizer splat;
    std::vector<double> xs, ys, zs, us, vs;
    std::vector<uint32_t> idx, rgba;
    std::vector<uint16_t> r, g, b;
    std::vector<NodeCache::NodePtr> nodes;
    std::vector<const HierarchyNode*> order;
    RawPoints raw;
    bool inUse = false;

    void Reserve() {
        if (xs.size() < kBatch) {
            for (auto* v : {&xs, &ys, &zs, &us, &vs}) v->resize(kBatch);
            idx.resize(kBatch);
            rgba.resize(kBatch);
            r.resize(kBatch);
            g.resize(kBatch);
            b.resize(kBatch);
        }
    }
};

// The thread's scratch; a nested render on the same thread (a pool task run while waiting)
// gets its own.
class ScratchLease {
public:
    ScratchLease() {
        thread_local Scratch tls;
        if (!tls.inUse) {
            s_ = &tls;
        } else {
            own_ = std::make_unique<Scratch>();
            s_ = own_.get();
        }
        s_->inUse = true;
        s_->Reserve();
    }
    ~ScratchLease() { s_->inUse = false; }
    Scratch& operator*() const noexcept { return *s_; }
    Scratch* operator->() const noexcept { return s_; }

private:
    Scratch* s_ = nullptr;
    std::unique_ptr<Scratch> own_;
};

kernels::NodeView ViewOf(const CompactNode& n) {
    kernels::NodeView v;
    for (int a = 0; a < 3; ++a) {
        v.off16[a] = n.offsets16(a);
        v.val32[a] = n.values32(a);
        v.base[a] = n.base()[static_cast<size_t>(a)];
    }
    v.count = n.size();
    return v;
}

kernels::NodeView ViewOf(const RawPoints& p) {
    kernels::NodeView v;
    v.val32[0] = p.x.data();
    v.val32[1] = p.y.data();
    v.val32[2] = p.z.data();
    v.count = static_cast<uint32_t>(p.size());
    return v;
}

// ---------------------------------------------------------------------------------------------
// Node orders of the baseline

// Pre-order of a depth-first descent visiting children in direction order 0..7.
bool DfsLess(const NodeKey& a, const NodeKey& b) noexcept {
    const int d = std::max(a.d, b.d);
    const NodeKey sa{d, a.x << (d - a.d), a.y << (d - a.d), a.z << (d - a.d)};
    const NodeKey sb{d, b.x << (d - b.d), b.y << (d - b.d), b.z << (d - b.d)};
    if (sa == sb) return a.d < b.d;
    return MortonLess(sa, sb);
}

// readers.copc keeps the selected entries in an std::unordered_set<copc::Entry> (hash of
// io/private/copc/Key.hpp), filled by a depth-first descent, and submits their loads in its
// iteration order. Same container, same hash, same insertions: the same iteration order
// (libstdc++).
struct PdalEntry {
    const HierarchyNode* node;
};
struct PdalHash {
    size_t operator()(const PdalEntry& e) const noexcept {
        const NodeKey& k = e.node->key;
        std::hash<uint64_t> h;
        const uint64_t k1 = (static_cast<uint64_t>(k.d) << 32) | static_cast<uint64_t>(k.x);
        const uint64_t k2 = (static_cast<uint64_t>(k.y) << 32) | static_cast<uint64_t>(k.z);
        return h(k1) ^ (h(k2) << 1);
    }
};
struct PdalEq {
    bool operator()(const PdalEntry& a, const PdalEntry& b) const noexcept { return a.node->key == b.node->key; }
};

// readers.copc's load order, then the order its tiles reach the stream: the entries are
// submitted in the set's order to a pool of 4 threads (DroneDB's copcReadThreads; readers.copc's
// ThreadPool takes tasks first in, first out), and each decoded tile is queued when it completes.
// The completion order is modelled with the cost of a tile in PDAL 2.10.1 (laz-perf 3.4's
// chunk_decompressor: ~500 us to set up, then ~0.34 us per point of format 6, measured on one
// thread over the corpus COPC files). On P3 this gives the baseline's exact tile for 126 of 163 sweep tiles; the
// baseline itself reproduces 131 of 163 from one run to the next.
void PdalOrder(std::vector<const HierarchyNode*>& nodes) {
    std::sort(nodes.begin(), nodes.end(),
              [](const HierarchyNode* a, const HierarchyNode* b) { return DfsLess(a->key, b->key); });
    std::unordered_set<PdalEntry, PdalHash, PdalEq> set;
    for (const HierarchyNode* n : nodes) set.insert(PdalEntry{n});
    nodes.clear();
    for (const PdalEntry& e : set) nodes.push_back(e.node);

    constexpr int kThreads = 4;
    constexpr double kSetupUs = 500.0, kPerPointUs = 0.34;
    double freeAt[kThreads] = {};
    std::vector<std::pair<double, const HierarchyNode*>> done;
    done.reserve(nodes.size());
    for (const HierarchyNode* n : nodes) {
        double* w = std::min_element(std::begin(freeAt), std::end(freeAt));
        *w += kSetupUs + kPerPointUs * n->pointCount;
        done.emplace_back(*w, n);
    }
    std::stable_sort(done.begin(), done.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    for (size_t i = 0; i < done.size(); ++i) nodes[i] = done[i].second;
}

void DroneDbOrder(const Cloud& cloud, std::vector<const HierarchyNode*>& nodes) {
    std::stable_sort(nodes.begin(), nodes.end(), [&cloud](const HierarchyNode* a, const HierarchyNode* b) {
        return cloud.DroneDbRank(a->Id()) < cloud.DroneDbRank(b->Id());
    });
}

// ---------------------------------------------------------------------------------------------

std::string OutOfBounds(const tile::compat::Box<tile::compat::PointI>& b) {
    return "Out of bounds [(" + std::to_string(b.min.x) + "; " + std::to_string(b.min.y) + ") - (" +
           std::to_string(b.max.x) + "; " + std::to_string(b.max.y) + ")]";
}

Selection SelectOrThrow(const Cloud& cloud, const SelectQuery& q, NodeOrder order) {
    auto sel = cloud.file().hierarchy().Select(q, order);
    if (!sel.ok()) cog::tiler::ThrowIfError(sel.status(), "Cannot read the hierarchy of " + cloud.path());
    return std::move(sel).value();
}

void FetchNodes(const Cloud& cloud, std::span<const HierarchyNode* const> nodes, std::vector<NodeCache::NodePtr>& out) {
    try {
        NodeCache::Global().Fetch(cloud.file(), nodes, out);
    } catch (const TileError&) {
        throw;
    } catch (const std::exception& e) {
        throw TileError(TileErrorCode::kException, e.what());
    }
}

// ---------------------------------------------------------------------------------------------
// compat

// The colours of the baseline: raw 16-bit values. From a compact node: 8-bit clouds keep them
// (no value above 255 in these nodes), 16-bit clouds kept v >> 8, given back as (v >> 8) << 8
// (the per-tile decision then sees "above 255" exactly when the raw value was).
void CompactColors(const CompactNode& n, bool color16, const uint32_t* idx, size_t count, uint16_t* r, uint16_t* g,
                   uint16_t* b) {
    const uint32_t* c = n.rgba();
    const int shift = color16 ? 8 : 0;
    for (size_t i = 0; i < count; ++i) {
        const uint32_t v = c[idx[i]];
        r[i] = static_cast<uint16_t>((v & 0xFFu) << shift);
        g[i] = static_cast<uint16_t>(((v >> 8) & 0xFFu) << shift);
        b[i] = static_cast<uint16_t>(((v >> 16) & 0xFFu) << shift);
    }
}

void RampColors(const double* zs, size_t n, double min, double max, uint32_t* rgba, uint16_t* r, uint16_t* g,
                uint16_t* b) {
    kernels::RampRgba(zs, n, min, max, rgba);
    for (size_t i = 0; i < n; ++i) {
        r[i] = static_cast<uint16_t>(rgba[i] & 0xFFu);
        g[i] = static_cast<uint16_t>((rgba[i] >> 8) & 0xFFu);
        b[i] = static_cast<uint16_t>((rgba[i] >> 16) & 0xFFu);
    }
}

void RenderCompat(const Cloud& cloud, const tile::compat::GlobalMercator& m, int tz, int tx, int tmsRow, int tileSize,
                  Scratch& s, std::vector<uint8_t>& rgba, RenderInfo& info) {
    // PointCloudTiler::tile
    const auto tileBounds = m.tileBounds(tx, tmsRow, tz);
    auto bounds = tileBounds;
    const int boundsBufSize = tile::compat::PointCloudBoundsBuffer(m, tz);
    bounds.min.x -= boundsBufSize;
    bounds.max.x += boundsBufSize;
    bounds.min.y -= boundsBufSize;
    bounds.max.y += boundsBufSize;
    if (!cloud.toCloud().TransformPoint(bounds.min.x, bounds.min.y) ||
        !cloud.toCloud().TransformPoint(bounds.max.x, bounds.max.y))
        throw TileError(TileErrorCode::kException, "Transform failed");
    const kernels::Window window{bounds.min.x, bounds.min.y, bounds.max.x, bounds.max.y};

    const double copcResolution = m.resolution(tz - ResolutionOffset());
    const auto pm = tile::compat::PointCloudPixelMapping(m, tileBounds, tz);
    CompatMapping mapping;
    mapping.originX = pm.originX;
    mapping.shiftX = pm.shiftX;
    mapping.scaleX = pm.scaleX;
    mapping.originY = pm.originY;
    mapping.shiftY = pm.shiftY;
    mapping.scaleY = pm.scaleY;
    mapping.width = pm.width;
    mapping.height = pm.height;
    const int pointRadius = tile::compat::kPointRadius;

    const bool colored = cloud.hasColor();
    // CopcNodeReader takes the resolution as is; readers.copc (colourless clouds) through its
    // option text.
    const double resolution = colored ? copcResolution : PdalOptionDouble(copcResolution);
    SelectQuery q;
    q.box = Box3::Xy(window.minX, window.minY, window.maxX, window.maxY);
    q.depthEnd = DepthEndForResolution(cloud.file().spacing(), resolution);
    info.depthEnd = q.depthEnd;
    Selection sel = SelectOrThrow(cloud, q, NodeOrder::kFileOrder);
    s.order.assign(sel.nodes.begin(), sel.nodes.end());
    if (colored)
        DroneDbOrder(cloud, s.order);
    else
        PdalOrder(s.order);
    info.nodes = static_cast<uint32_t>(s.order.size());
    info.points = sel.points;

    if (!colored && !(cloud.compatRampMax() > cloud.compatRampMin()))
        throw TileError(TileErrorCode::kException,
                        "filters.colorinterp: Specified 'minimum' value must be less than 'maximum' value.");

    FetchNodes(cloud, s.order, s.nodes);
    const kernels::Scaling& sc = cloud.scaling();
    const crs::Transform& toMercator = cloud.toMercator();

    auto draw = [&](size_t n) {
        toMercator.TransformPoints(s.xs.data(), s.ys.data(), nullptr, n);
        s.compat.Add(s.xs.data(), s.ys.data(), s.zs.data(), s.r.data(), s.g.data(), s.b.data(), n);
        info.inWindow += n;
    };

    bool raw = false;
    if (colored && !cloud.color16()) {
        // 8-bit cloud: the compact colours are the raw ones unless a node had values above 255.
        for (const auto& n : s.nodes) raw = raw || n->over255;
    }
    if (!raw) {
        s.compat.Reset(tileSize, mapping, pointRadius, pointRadius);
        for (const auto& np : s.nodes) {
            const CompactNode& node = np->points;
            const kernels::NodeView view = ViewOf(node);
            for (size_t at = 0; at < node.size(); at += kBatch) {
                const size_t end = std::min<size_t>(node.size(), at + kBatch);
                const size_t n =
                    kernels::DecodeCull(view, at, end, sc, window, s.xs.data(), s.ys.data(), s.zs.data(), s.idx.data());
                if (n == 0) continue;
                if (colored)
                    CompactColors(node, cloud.color16(), s.idx.data(), n, s.r.data(), s.g.data(), s.b.data());
                else
                    RampColors(s.zs.data(), n, cloud.compatRampMin(), cloud.compatRampMax(), s.rgba.data(), s.r.data(),
                               s.g.data(), s.b.data());
                draw(n);
            }
        }
        // A 16-bit cloud whose tile has points but none above 255: the baseline shows the low
        // bytes, which the compact nodes do not keep.
        raw = colored && cloud.color16() && s.compat.added() > 0 && !s.compat.sixteenBit();
    }
    if (raw) {
        info.rawFallback = true;
        info.inWindow = 0;
        s.compat.Reset(tileSize, mapping, pointRadius, pointRadius);
        for (const HierarchyNode* node : s.order) {
            const Status st = DecodeNodeRaw(cloud.file(), *node, laz::kLayersXyzRgb, s.raw);
            if (!st.ok()) cog::tiler::ThrowIfError(st, "Cannot decompress a COPC node of " + cloud.path());
            const kernels::NodeView view = ViewOf(s.raw);
            for (size_t at = 0; at < s.raw.size(); at += kBatch) {
                const size_t end = std::min(s.raw.size(), at + kBatch);
                const size_t n =
                    kernels::DecodeCull(view, at, end, sc, window, s.xs.data(), s.ys.data(), s.zs.data(), s.idx.data());
                for (size_t i = 0; i < n; ++i) {
                    s.r[i] = s.raw.r[s.idx[i]];
                    s.g[i] = s.raw.g[s.idx[i]];
                    s.b[i] = s.raw.b[s.idx[i]];
                }
                if (n > 0) draw(n);
            }
        }
    }
    rgba.resize(static_cast<size_t>(tileSize) * static_cast<size_t>(tileSize) * 4);
    info.opaque = s.compat.ToRgba(rgba.data());
    info.transform = RenderInfo::Transform::kExact;
}

// ---------------------------------------------------------------------------------------------
// hybrid and modern

struct PixelMap {
    // u = (mx - minX) / res + kRadius, v = (maxY - my) / res + kRadius: anchor-buffer pixels.
    double minX, maxY, invRes;
    void ToPixels(double* mx, double* my, size_t n) const {
        constexpr double r = SplatRasterizer::kRadius;
        for (size_t i = 0; i < n; ++i) {
            mx[i] = (mx[i] - minX) * invRes + r;
            my[i] = (maxY - my[i]) * invRes + r;
        }
    }
};

// Fits cloud (x, y) -> centre-buffer pixel (u, v) over the window: degree 1, 2, then 3, the
// first whose max error <= 0.01 px on the fit samples and on the centres of the sample cells.
std::optional<crs::PolyFit> FitWindow(const crs::Transform& toMercator, const kernels::Window& w, const PixelMap& pm,
                                      double& residual) {
    constexpr int n = kFitSamples;
    constexpr int nc = n - 1;
    double xs[n * n + nc * nc], ys[n * n + nc * nc], us[n * n + nc * nc], vs[n * n + nc * nc];
    size_t k = 0;
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i, ++k) {
            xs[k] = w.minX + (w.maxX - w.minX) * i / (n - 1);
            ys[k] = w.minY + (w.maxY - w.minY) * j / (n - 1);
        }
    for (int j = 0; j < nc; ++j)
        for (int i = 0; i < nc; ++i, ++k) {
            xs[k] = w.minX + (w.maxX - w.minX) * (i + 0.5) / (n - 1);
            ys[k] = w.minY + (w.maxY - w.minY) * (j + 0.5) / (n - 1);
        }
    std::memcpy(us, xs, sizeof xs);
    std::memcpy(vs, ys, sizeof ys);
    if (toMercator.TransformPoints(us, vs, nullptr, k) != 0) return std::nullopt;
    pm.ToPixels(us, vs, k);
    for (int degree = 1; degree <= 3; ++degree) {
        auto fit = crs::FitPolynomial(xs, ys, us, vs, static_cast<size_t>(n * n), degree);
        if (!fit) continue;
        double worst = fit->MaxResidual();
        if (!(worst <= kFitMaxResidualPx)) continue;
        for (size_t c = static_cast<size_t>(n * n); c < k && worst <= kFitMaxResidualPx; ++c) {
            double u, v;
            fit->Eval(xs[c], ys[c], u, v);
            worst = std::max(worst, std::hypot(u - us[c], v - vs[c]));
        }
        if (worst <= kFitMaxResidualPx) {
            residual = worst;
            return fit;
        }
    }
    return std::nullopt;
}

// hybrid and modern differ in the depth limit, the ramp range, the disc and where it is anchored
// (see copc_tiler.h); the window, the transform, the colours and the z-test are the same.
void RenderSplat(const Cloud& cloud, const tile::compat::GlobalMercator& m, int tz, int tx, int tmsRow, int tileSize,
                 Mode mode, Scratch& s, std::vector<uint8_t>& rgba, RenderInfo& info) {
    const bool hybrid = mode == Mode::kHybrid;
    const size_t bytes = static_cast<size_t>(tileSize) * static_cast<size_t>(tileSize) * 4;
    rgba.resize(bytes);
    auto blank = [&] {
        std::memset(rgba.data(), 0, bytes);
        info.blank = true;
    };

    const double res = m.resolution(tz);
    const auto tb = m.tileBounds(tx, tmsRow, tz);
    // Discs reach kRadius px; one more pixel of margin for the rounding of the window.
    const double pad = (SplatRasterizer::kRadius + 1) * res;
    const Rect padded{tb.min.x - pad, tb.min.y - pad, tb.max.x + pad, tb.max.y + pad};
    const auto& fp = cloud.footprint();
    if (!fp || fp->maxX < padded.minX || fp->minX > padded.maxX || fp->maxY < padded.minY || fp->minY > padded.maxY)
        return blank();

    // Read window: the padded tile's outline (32 points) in the cloud CRS.
    constexpr int kSide = 8;
    double wx[8 * kSide], wy[8 * kSide], wz[8 * kSide];
    const bool geo = cloud.geocentric();
    const int passes = geo ? 2 : 1;
    size_t k = 0;
    for (int pass = 0; pass < passes; ++pass)
        for (int side = 0; side < 4; ++side)
            for (int i = 0; i < kSide; ++i, ++k) {
                const double t = static_cast<double>(i) / kSide;
                const double w = padded.maxX - padded.minX, h = padded.maxY - padded.minY;
                switch (side) {
                    case 0: wx[k] = padded.minX + t * w; wy[k] = padded.minY; break;
                    case 1: wx[k] = padded.maxX; wy[k] = padded.minY + t * h; break;
                    case 2: wx[k] = padded.maxX - t * w; wy[k] = padded.maxY; break;
                    default: wx[k] = padded.minX; wy[k] = padded.maxY - t * h; break;
                }
                wz[k] = pass == 0 ? cloud.heightMin() : cloud.heightMax();
            }
    kernels::Window window;
    if (cloud.toCloud().TransformPoints(wx, wy, geo ? wz : nullptr, k) == 0) {
        window = {wx[0], wy[0], wx[0], wy[0]};
        for (size_t i = 1; i < k; ++i) {
            window.minX = std::min(window.minX, wx[i]);
            window.maxX = std::max(window.maxX, wx[i]);
            window.minY = std::min(window.minY, wy[i]);
            window.maxY = std::max(window.maxY, wy[i]);
        }
        // The outline is sampled: allow for its curvature between samples.
        const double mx = (window.maxX - window.minX) * 0.01, my = (window.maxY - window.minY) * 0.01;
        window.minX -= mx;
        window.maxX += mx;
        window.minY -= my;
        window.maxY += my;
    } else {
        // Part of the tile has no preimage (beyond the CRS's domain): read the whole cloud there.
        const las::LasHeader& h = cloud.file().header();
        window = {h.min[0], h.min[1], h.max[0], h.max[1]};
    }

    // Level of detail. hybrid: the baseline's (the spacing in the cloud's units against
    // res(z - 2), the resolution passed through readers.copc's option text for colourless
    // clouds). modern: the spacing projected to EPSG:3857 (sqrt |det J| at the window centre).
    double scale = 1.0;
    if (!hybrid && !geo) {
        const double cx = (window.minX + window.maxX) / 2, cy = (window.minY + window.maxY) / 2;
        const double hx = std::max((window.maxX - window.minX) * 0.01, 1e-9);
        const double hy = std::max((window.maxY - window.minY) * 0.01, 1e-9);
        double px[3] = {cx, cx + hx, cx}, py[3] = {cy, cy, cy + hy};
        if (cloud.toMercator().TransformPoints(px, py, nullptr, 3) == 0) {
            const double det = ((px[1] - px[0]) * (py[2] - py[0]) - (px[2] - px[0]) * (py[1] - py[0])) / (hx * hy);
            if (std::isfinite(det) && det != 0) scale = std::sqrt(std::fabs(det));
        }
    } else if (!hybrid) {
        // Geocentric: metres on the ground, EPSG:3857 scale 1 / cos(latitude) = cosh(y / R).
        scale = std::cosh(((tb.min.y + tb.max.y) / 2) / 6378137.0);
    }
    const bool colored = cloud.hasColor();
    double lodResolution = m.resolution(tz - ResolutionOffset());
    if (hybrid && !colored) lodResolution = PdalOptionDouble(lodResolution);
    SelectQuery q;
    q.box = Box3::Xy(window.minX, window.minY, window.maxX, window.maxY);
    q.depthEnd = DepthEndForResolution(cloud.file().spacing() * scale, lodResolution);
    info.depthEnd = q.depthEnd;
    Selection sel = SelectOrThrow(cloud, q, NodeOrder::kFileOrder);
    info.nodes = static_cast<uint32_t>(sel.nodes.size());
    info.points = sel.points;
    if (sel.nodes.empty()) return blank();

    const PixelMap pm{tb.min.x, tb.max.y, 1.0 / res};
    std::optional<crs::PolyFit> fit;
    if (!geo && sel.points >= kExactBelow) {
        double residual = 0;
        fit = FitWindow(cloud.toMercator(), window, pm, residual);
        if (fit) {
            info.fitResidual = residual;
            info.transform = fit->Degree() == 1   ? RenderInfo::Transform::kAffine
                             : fit->Degree() == 2 ? RenderInfo::Transform::kQuadratic
                                                  : RenderInfo::Transform::kCubic;
        }
    }
    if (!fit) info.transform = RenderInfo::Transform::kExact;

    // Ramp range of colourless clouds. hybrid: filters.colorinterp's (the baseline's colours),
    // unless that range is empty (the baseline then fails; hybrid colours the cloud as modern).
    double rampMin = cloud.zMin(), rampMax = cloud.zMax();
    if (hybrid && cloud.compatRampMax() > cloud.compatRampMin()) {
        rampMin = cloud.compatRampMin();
        rampMax = cloud.compatRampMax();
    }

    FetchNodes(cloud, sel.nodes, s.nodes);
    // The disc and its anchor. modern: the 13-pixel disc on the pixel holding the point
    // (floor). hybrid: the baseline's 11-pixel stamp on the pixel whose top-left corner is the
    // pixel corner nearest the point (round, as PointRasterizer), so that the stamp's 4 x 4 box
    // is centred on the point up to half a pixel.
    s.splat.Reset(tileSize, hybrid ? SplatRasterizer::Disc::kStamp11 : SplatRasterizer::Disc::kDisc13);
    const double anchorBias = hybrid ? 0.5 : 0.0;
    const int P = s.splat.PaddedSize();
    const kernels::Scaling& sc = cloud.scaling();
    const double zOrigin = geo ? cloud.heightMin() : cloud.zMin();
    for (const auto& np : s.nodes) {
        const CompactNode& node = np->points;
        const kernels::NodeView view = ViewOf(node);
        const uint32_t* colors = node.rgba();
        for (size_t at = 0; at < node.size(); at += kBatch) {
            const size_t end = std::min<size_t>(node.size(), at + kBatch);
            const size_t n =
                kernels::DecodeCull(view, at, end, sc, window, s.xs.data(), s.ys.data(), s.zs.data(), s.idx.data());
            if (n == 0) continue;
            info.inWindow += n;
            if (colored) {
                for (size_t i = 0; i < n; ++i) s.rgba[i] = colors[s.idx[i]];
            } else {
                kernels::RampRgba(s.zs.data(), n, rampMin, rampMax, s.rgba.data());
            }
            const double* heights = s.zs.data();
            if (fit) {
                fit->Eval(s.xs.data(), s.ys.data(), s.us.data(), s.vs.data(), n);
            } else {
                std::memcpy(s.us.data(), s.xs.data(), n * sizeof(double));
                std::memcpy(s.vs.data(), s.ys.data(), n * sizeof(double));
                if (geo) {
                    // Heights come out in the z array.
                    cloud.toMercator().TransformPoints(s.us.data(), s.vs.data(), s.zs.data(), n);
                } else {
                    cloud.toMercator().TransformPoints(s.us.data(), s.vs.data(), nullptr, n);
                }
                pm.ToPixels(s.us.data(), s.vs.data(), n);
            }
            for (size_t i = 0; i < n; ++i) {
                const double u = s.us[i] + anchorBias, v = s.vs[i] + anchorBias;
                // Also rejects NaN and the HUGE_VAL of failed transforms.
                if (!(u >= 0.0 && u < P && v >= 0.0 && v < P)) continue;
                const float z = static_cast<float>(heights[i] - zOrigin);
                s.splat.Add(static_cast<int>(u), static_cast<int>(v), SplatKey(z, s.rgba[i]));
            }
        }
    }
    info.opaque = s.splat.Resolve(rgba.data());
    if (info.opaque == 0) info.blank = true;
}


// The deepest depth whose points, with those of every shallower depth, fit the budget (0 when even
// the root exceeds it): what a thumbnail reads.
int ThumbnailDepth(const Cloud& cloud, uint64_t budget) {
    auto all = cloud.file().hierarchy().All(NodeOrder::kFileOrder);
    if (!all.ok()) cog::tiler::ThrowIfError(all.status(), "Cannot read the hierarchy of " + cloud.path());
    std::vector<uint64_t> perDepth;
    for (const HierarchyNode* n : all->nodes) {
        const auto d = static_cast<size_t>(n->key.d);
        if (perDepth.size() <= d) perDepth.resize(d + 1, 0);
        perDepth[d] += static_cast<uint64_t>(n->pointCount);
    }
    int depth = 0;
    uint64_t cumulative = 0;
    for (size_t d = 0; d < perDepth.size(); ++d) {
        cumulative += perDepth[d];
        if (cumulative > budget) break;
        depth = static_cast<int>(d);
    }
    return depth;
}

}  // namespace

void RenderThumbnail(const ThumbnailRequest& r, std::vector<uint8_t>& rgba, RenderInfo* infoOut) {
    if (r.size <= 0 || r.size > 16384) throw TileError(TileErrorCode::kInvalidArgs, "Invalid thumbnail size");
    const auto cloud = Cloud::Get(r.path);
    const las::LasHeader& h = cloud->file().header();

    // The area drawn: the footprint in EPSG:3857, else the header bounds in the cloud's x, y
    const bool mercator = cloud->crsError().empty() && cloud->footprint().has_value();
    const Rect area = mercator ? *cloud->footprint() : Rect{h.min[0], h.min[1], h.max[0], h.max[1]};
    const double width = area.maxX - area.minX, height = area.maxY - area.minY;
    if (!(std::max(width, height) > 0) || !std::isfinite(width) || !std::isfinite(height))
        throw TileError(TileErrorCode::kException, "Cannot calculate length: point cloud has zero extent");
    const double scale = r.size / std::max(width, height);
    // Centred: the margins of the shorter side split evenly
    const double offX = (r.size - width * scale) / 2, offY = (r.size - height * scale) / 2;
    const PixelMap pm{area.minX - offX / scale, area.maxY + offY / scale, scale};

    RenderInfo info;
    const uint64_t budget = r.pointBudget > 0 ? r.pointBudget
                                              : 2 * static_cast<uint64_t>(r.size) * static_cast<uint64_t>(r.size);
    SelectQuery q;
    q.box = Box3::Xy(h.min[0], h.min[1], h.max[0], h.max[1]);
    q.depthEnd = ThumbnailDepth(*cloud, budget) + 1;
    info.depthEnd = q.depthEnd;
    Selection sel = SelectOrThrow(*cloud, q, NodeOrder::kFileOrder);
    info.nodes = static_cast<uint32_t>(sel.nodes.size());
    info.points = sel.points;
    if (sel.points == 0) throw TileError(TileErrorCode::kException, "No points fetched from cloud, check zoom level");

    const bool geo = mercator && cloud->geocentric();
    const kernels::Window window{h.min[0], h.min[1], h.max[0], h.max[1]};
    std::optional<crs::PolyFit> fit;
    if (mercator && !geo && sel.points >= kExactBelow) {
        double residual = 0;
        fit = FitWindow(cloud->toMercator(), window, pm, residual);
        if (fit) info.fitResidual = residual;
    }
    info.transform = !mercator ? RenderInfo::Transform::kNone
                     : !fit    ? RenderInfo::Transform::kExact
                     : fit->Degree() == 1 ? RenderInfo::Transform::kAffine
                     : fit->Degree() == 2 ? RenderInfo::Transform::kQuadratic
                                          : RenderInfo::Transform::kCubic;

    ScratchLease s;
    FetchNodes(*cloud, sel.nodes, s->nodes);
    s->splat.Reset(r.size, SplatRasterizer::Disc::kDisc13);
    const int P = s->splat.PaddedSize();
    const kernels::Scaling& sc = cloud->scaling();
    const bool colored = cloud->hasColor();
    const double zOrigin = geo ? cloud->heightMin() : cloud->zMin();
    for (const auto& np : s->nodes) {
        const CompactNode& node = np->points;
        const kernels::NodeView view = ViewOf(node);
        const uint32_t* colors = node.rgba();
        for (size_t at = 0; at < node.size(); at += kBatch) {
            const size_t end = std::min<size_t>(node.size(), at + kBatch);
            const size_t n = kernels::DecodeCull(view, at, end, sc, window, s->xs.data(), s->ys.data(), s->zs.data(),
                                                 s->idx.data());
            if (n == 0) continue;
            info.inWindow += n;
            if (colored) {
                for (size_t i = 0; i < n; ++i) s->rgba[i] = colors[s->idx[i]];
            } else {
                kernels::RampRgba(s->zs.data(), n, cloud->zMin(), cloud->zMax(), s->rgba.data());
            }
            if (fit) {
                fit->Eval(s->xs.data(), s->ys.data(), s->us.data(), s->vs.data(), n);
            } else {
                std::memcpy(s->us.data(), s->xs.data(), n * sizeof(double));
                std::memcpy(s->vs.data(), s->ys.data(), n * sizeof(double));
                if (mercator) cloud->toMercator().TransformPoints(s->us.data(), s->vs.data(), geo ? s->zs.data() : nullptr, n);
                pm.ToPixels(s->us.data(), s->vs.data(), n);
            }
            for (size_t i = 0; i < n; ++i) {
                const double u = s->us[i], v = s->vs[i];
                if (!(u >= 0.0 && u < P && v >= 0.0 && v < P)) continue;
                s->splat.Add(static_cast<int>(u), static_cast<int>(v),
                             SplatKey(static_cast<float>(s->zs[i] - zOrigin), s->rgba[i]));
            }
        }
    }
    rgba.resize(static_cast<size_t>(r.size) * static_cast<size_t>(r.size) * 4);
    info.opaque = s->splat.Resolve(rgba.data());
    s->nodes.clear();
    if (info.opaque == 0) info.blank = true;
    if (infoOut) *infoOut = info;
}

std::optional<Mode> ParseMode(std::string_view name) {
    if (name == "compat") return Mode::kCompat;
    if (name == "hybrid" || name == "default") return Mode::kHybrid;
    if (name == "modern") return Mode::kModern;
    return std::nullopt;
}

const char* ModeName(Mode mode) {
    switch (mode) {
        case Mode::kCompat: return "compat";
        case Mode::kHybrid: return "hybrid";
        case Mode::kModern: return "modern";
    }
    return "?";
}

Mode GetMode() { return static_cast<Mode>(ModeSetting().load(std::memory_order_relaxed)); }
void SetMode(Mode mode) { ModeSetting().store(static_cast<int>(mode), std::memory_order_relaxed); }

int ResolutionOffset() {
    static const int offset = ReadEnvInt("DDBT_PCTILE_RESOLUTION_OFFSET", 2);
    return offset;
}

namespace {
void RequireTileSize(int tileSize) {
    if (tileSize <= 0 || (tileSize & (tileSize - 1)) != 0)
        throw TileError(TileErrorCode::kInvalidArgs, "Tile size must be a power of 2 greater than 0");
}

// The footprint the tiles of `mode` are bounded by (Tiler::getMinMaxCoordsForZ): the baseline's,
// else (outside compat) the densified outline's. Throws when there is none.
const Rect& TilingFootprint(const Cloud& cloud, Mode mode) {
    if (!cloud.crsError().empty()) throw TileError(TileErrorCode::kInvalidArgs, cloud.crsError());
    const std::optional<Rect>* footprint = &cloud.compatFootprint();
    if (!*footprint && mode != Mode::kCompat) footprint = &cloud.footprint();
    if (!*footprint) {
        const std::string why =
            cloud.footprintError().empty() ? std::string("its bounds do not map to EPSG:3857") : cloud.footprintError();
        throw TileError(TileErrorCode::kInvalidArgs, "Cannot tile " + cloud.path() + ": footprint unavailable: " + why);
    }
    return **footprint;
}
}  // namespace

tile::compat::TileExtent Extent(const std::string& path, int tileSize) {
    RequireTileSize(tileSize);
    const auto cloud = Cloud::Get(path);
    const Rect& f = TilingFootprint(*cloud, GetMode());
    tile::compat::TileExtent e;
    e.minX = f.minX;
    e.minY = f.minY;
    e.maxX = f.maxX;
    e.maxY = f.maxY;
    e.zooms = tile::compat::PointCloudZoomRange(tile::compat::GlobalMercator(tileSize), f.minX, f.minY, f.maxX,
                                                f.maxY, tile::compat::kDroneDbCopcSpan);
    return e;
}

void RenderRgba(const Cloud& cloud, const TileRequest& r, std::vector<uint8_t>& rgba, RenderInfo* infoOut) {
    RequireTileSize(r.tileSize);
    const Mode mode = r.mode.value_or(GetMode());
    const tile::compat::GlobalMercator m(r.tileSize);
    const int tmsRow = tile::compat::InternalRow(r.y, r.z, r.tms);

    // The footprint's tile range (Tiler::getMinMaxCoordsForZ): any zoom, out of it an error.
    const Rect& f = TilingFootprint(cloud, mode);
    const auto range = tile::compat::MinMaxCoordsForZ(m, f.minX, f.minY, f.maxX, f.maxY, r.z);
    if (!range.Contains(r.x, tmsRow)) throw TileError(TileErrorCode::kException, OutOfBounds(range));

    RenderInfo info;
    ScratchLease s;
    if (mode == Mode::kCompat)
        RenderCompat(cloud, m, r.z, r.x, tmsRow, r.tileSize, *s, rgba, info);
    else
        RenderSplat(cloud, m, r.z, r.x, tmsRow, r.tileSize, mode, *s, rgba, info);
    if (info.opaque == 0) info.blank = true;
    s->nodes.clear();  // drop the node references (the cache owns them)
    if (infoOut) *infoOut = info;
}

encode::ByteBuffer RenderTile(const TileRequest& r) {
    const auto cloud = Cloud::Get(r.path);
    const Mode mode = r.mode.value_or(GetMode());
    encode::EncodeOptions options;
    options.format = r.format;

    TileCache& tiles = EncodedTiles();
    const bool useCache = !r.noCache && tiles.BudgetBytes() > 0;
    TileCacheKey key;
    if (useCache) {
        key = {cloud->file().id(), r.z, r.x, tile::compat::InternalRow(r.y, r.z, r.tms), r.tileSize,
               static_cast<uint8_t>(r.format), static_cast<uint8_t>(mode)};
        if (auto hit = tiles.Get(key)) {
            encode::ByteBuffer out;
            out.assign(hit->data(), hit->size());
            return out;
        }
    }

    thread_local std::vector<uint8_t> rgba;
    RenderInfo info;
    TileRequest req = r;
    req.mode = mode;
    RenderRgba(*cloud, req, rgba, &info);

    encode::ByteBuffer out;
    const uint32_t size = static_cast<uint32_t>(r.tileSize);
    if (info.opaque == 0) {
        const auto& b = encode::BlankTileBytes(size, size, 3, options);
        out.assign(b.data(), b.size());
    } else {
        encode::TileImage img = encode::TileImage::Packed(rgba.data(), size, size, 4);
        img.alphaKnown = true;
        img.allOpaque = info.opaque == static_cast<uint64_t>(size) * size;
        img.allTransparent = false;
        encode::EncodeTile(img, options, out);
    }
    if (useCache) tiles.Put(key, std::vector<uint8_t>(out.data(), out.data() + out.size()));
    return out;
}

::ddbt::cache::CacheStats TileCacheStats() { return EncodedTiles().Stats(); }
void SetTileCacheBudget(size_t bytes) { EncodedTiles().SetBudget(bytes); }
size_t TileCacheBudget() { return EncodedTiles().BudgetBytes(); }
void ClearTileCache() { EncodedTiles().Clear(); }

}  // namespace ddbt::copc::tiler
