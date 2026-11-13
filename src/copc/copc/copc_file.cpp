// SPDX-License-Identifier: MPL-2.0
#include "copc/copc/copc_file.h"

#include <algorithm>
#include <atomic>
#include <string>
#include <vector>

#include "copc/copc/node_decoder.h"
#include "copc/las/las_crs.h"
#include "core/exec/parallel_for.h"
#include "core/exec/thread_pool.h"
#include "core/util/checked_math.h"

namespace ddbt::copc {
namespace {

std::atomic<uint64_t> g_nextFileId{1};

}  // namespace

CopcFile::~CopcFile() = default;

Expected<std::shared_ptr<const CopcFile>> CopcFile::Open(std::string_view utf8Path, const CopcOpenOptions& options) {
    DDBT_ASSIGN_OR_RETURN(std::unique_ptr<io::ByteSource> source, io::OpenByteSource(utf8Path, options.sourceKind));
    return Open(std::shared_ptr<const io::ByteSource>(std::move(source)), options);
}

Expected<std::shared_ptr<const CopcFile>> CopcFile::Open(std::shared_ptr<const io::ByteSource> source,
                                                         const CopcOpenOptions& options) {
    if (!source) return InvalidArgument("CopcFile::Open: null source");
    std::shared_ptr<CopcFile> f(new CopcFile());
    f->source_ = std::move(source);
    const std::string name(f->source_->Name());
    auto meta = las::ReadMetadata(*f->source_, options.las);
    if (!meta.ok()) return meta.status().WithContext(name);
    f->meta_ = std::move(*meta);
    const las::LasHeader& h = f->meta_.header;

    // CopcReader::fetchHeader / validateHeader / validateVlrInfo, plus the LAZ VLR.
    if (!f->meta_.copcInfo) return Unsupported(name + ": not a COPC file (no COPC info VLR at offset 375)");
    if (!h.VersionAtLeast(1, 4)) return Unsupported(name + ": COPC needs LAS 1.4");
    if (h.pointFormat < 6 || h.pointFormat > 8)
        return Unsupported(name + ": COPC point format " + std::to_string(h.pointFormat) + " (must be 6-8)");
    if (!f->meta_.laz) return Corrupt(name + ": COPC file without a LAZ VLR");
    if (f->meta_.laz->compressor != 3)
        return Unsupported(name + ": LAZ compressor " + std::to_string(f->meta_.laz->compressor) +
                           " (COPC chunks are layered, compressor 3)");
    const las::CopcInfo& info = *f->meta_.copcInfo;
    if (!RangeInside(info.rootHierOffset, info.rootHierSize, f->source_->Size()))
        return Corrupt(name + ": COPC root hierarchy page outside the file");

    const Box3 cube = f->RootCube();
    const bool eager = options.hierarchy == HierarchyLoad::kEager ||
                       (options.hierarchy == HierarchyLoad::kAuto && f->source_->Data() != nullptr);
    auto hier = HierarchyIndex::Load(f->source_, info.rootHierOffset, info.rootHierSize,
                                     {cube.minX, cube.minY, cube.minZ, cube.maxX, cube.maxY, cube.maxZ}, eager);
    if (!hier.ok()) return hier.status().WithContext(name);
    f->hierarchy_ = std::move(*hier);
    f->id_ = g_nextFileId.fetch_add(1, std::memory_order_relaxed);
    return std::shared_ptr<const CopcFile>(std::move(f));
}

Box3 CopcFile::RootCube() const noexcept {
    const las::CopcInfo& i = *meta_.copcInfo;
    // CopcReader::fetchHeader: rootNodeExtent = center -+ halfsize.
    return {i.centerX - i.halfSize, i.centerY - i.halfSize, i.centerZ - i.halfSize,
            i.centerX + i.halfSize, i.centerY + i.halfSize, i.centerZ + i.halfSize};
}

Expected<crs::Crs> CopcFile::Crs() const {
    std::call_once(crsOnce_, [this] { crs_.emplace(las::ResolveCrs(meta_)); });
    return *crs_;
}

Expected<bool> CopcFile::Uses16BitColor() const {
    const int known = color16_.load(std::memory_order_acquire);
    if (known >= 0) return known == 1;
    if (!hasRgb()) {
        color16_.store(0, std::memory_order_release);
        return false;
    }
    std::lock_guard<std::mutex> lock(colorMutex_);
    if (const int k = color16_.load(std::memory_order_acquire); k >= 0) return k == 1;

    SelectQuery q;
    q.depthEnd = kColorDecisionMaxDepth + 1;
    DDBT_ASSIGN_OR_RETURN(Selection sel, hierarchy_->Select(q, NodeOrder::kFileOrder));

    // The sample: whole depths from the root down, until they hold kColorDecisionMinPoints
    // points (or depth kColorDecisionMaxDepth is in). The shallow levels of a COPC are a uniform
    // subsample of the whole cloud, so a cloud with 16-bit colours shows them there. Reading
    // every node of depths 0-2 instead cost an 8-bit cloud 70 ms (P1: 505 k of its 0.9 M
    // points) on the first tile request, while every other request for the cloud waited.
    std::vector<const HierarchyNode*> nodes;
    {
        uint64_t points = 0;
        for (int d = 0; d <= kColorDecisionMaxDepth && points < kColorDecisionMinPoints; ++d) {
            for (const HierarchyNode* node : sel.nodes) {
                if (node->key.d != d) continue;
                nodes.push_back(node);
                points += static_cast<uint64_t>(std::max<int64_t>(node->pointCount, 0));
            }
        }
    }

    // The answer is that of the first node, in file order, that is decisive: a component above
    // 255 (16-bit) or a decoding error; no decisive node means 8-bit. The nodes are decoded in
    // parallel on the default pool; a node after the first decisive one found so far is
    // skipped, nodes before it are still decoded, so the answer is the serial loop's.
    std::stable_sort(nodes.begin(), nodes.end(),
                     [](const HierarchyNode* a, const HierarchyNode* b) { return a->offset < b->offset; });
    const size_t n = nodes.size();
    enum : uint8_t { kEightBit, kSixteenBit, kError };
    std::vector<uint8_t> outcome(n, kEightBit);
    std::vector<Status> errors(n);
    std::atomic<size_t> firstDecisive{n};
    auto decide = [&](size_t i) {
        if (i > firstDecisive.load(std::memory_order_relaxed)) return;
        thread_local RawPoints raw;
        Status s = DecodeNodeRaw(*this, *nodes[i], laz::kLayerXY | laz::kLayerRgb, raw);
        if (!s.ok()) {
            outcome[i] = kError;
            errors[i] = std::move(s);
        } else {
            bool sixteen = false;
            for (size_t k = 0; k < raw.size() && !sixteen; ++k)
                sixteen = raw.r[k] > 255 || raw.g[k] > 255 || raw.b[k] > 255;
            if (!sixteen) return;
            outcome[i] = kSixteenBit;
        }
        size_t cur = firstDecisive.load(std::memory_order_relaxed);
        while (i < cur && !firstDecisive.compare_exchange_weak(cur, i, std::memory_order_relaxed)) {
        }
    };
    {
        // Isolated: while this thread waits for the decision's tasks (holding colorMutex_) it runs
        // only those, never a queued task that could ask this file for its decision again.
        exec::IsolationScope scope(this);
        exec::ParallelFor(exec::DefaultPool(), 0, n, 1, decide, nullptr, exec::Priority::kInteractive);
    }

    const size_t first = firstDecisive.load(std::memory_order_relaxed);
    if (first < n && outcome[first] == kError) return errors[first];
    const bool sixteen = first < n;
    color16_.store(sixteen ? 1 : 0, std::memory_order_release);
    return sixteen;
}

}  // namespace ddbt::copc
