// SPDX-License-Identifier: MPL-2.0
#include "copc/build/copc_check.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <unordered_set>

#include "copc/copc/copc_file.h"
#include "copc/copc/node_decoder.h"
#include "copc/laz/chunk_decoder.h"
#include "copc/readers/las_reader.h"
#include "core/exec/parallel_for.h"
#include "core/exec/thread_pool.h"

namespace ddbt::copc::build {

namespace {
uint64_t KeyOf(const NodeKey& k) {
    return (static_cast<uint64_t>(k.d) << 57) ^ (static_cast<uint64_t>(k.x) << 38) ^ (static_cast<uint64_t>(k.y) << 19) ^
           static_cast<uint64_t>(k.z);
}
}  // namespace

Expected<CopcCheckReport> CheckCopc(const std::string& path, size_t threads) {
    CopcCheckReport r;
    CopcOpenOptions oo;
    oo.hierarchy = HierarchyLoad::kEager;
    DDBT_ASSIGN_OR_RETURN(std::shared_ptr<const CopcFile> file, CopcFile::Open(path, oo));
    const las::LasHeader& h = file->header();
    const las::CopcInfo& info = file->info();
    r.headerPoints = h.pointCount;
    const double spacing = 2 * info.halfSize / 128;
    if (std::abs(info.spacing - spacing) > 1e-12 * std::max(1.0, spacing))
        r.errors.push_back("spacing " + std::to_string(info.spacing) + " is not 2 halfsize / 128 = " + std::to_string(spacing));
    DDBT_ASSIGN_OR_RETURN(Selection all, file->hierarchy().All(NodeOrder::kFileOrder));
    r.dataNodes = all.nodes.size();
    std::unordered_set<uint64_t> keys;
    for (const HierarchyNode* n : all.nodes) {
        r.hierarchyPoints += static_cast<uint64_t>(n->pointCount);
        r.maxDepth = std::max(r.maxDepth, n->key.d);
        keys.insert(KeyOf(n->key));
    }
    if (r.hierarchyPoints != r.headerPoints)
        r.errors.push_back("hierarchy holds " + std::to_string(r.hierarchyPoints) + " points, the header " +
                           std::to_string(r.headerPoints));
    std::unordered_set<uint64_t> parents;
    for (const HierarchyNode* n : all.nodes)
        if (n->key.d > 0) parents.insert(KeyOf({n->key.d - 1, n->key.x >> 1, n->key.y >> 1, n->key.z >> 1}));

    // Chunk table against the hierarchy (file order).
    {
        auto meta = las::ReadMetadata(file->source());
        if (!meta.ok()) return meta.status();
        auto chunks = readers::ReadLazChunkTable(file->source(), *meta);
        if (!chunks.ok()) {
            r.errors.push_back("chunk table: " + std::string(chunks.status().message()));
        } else {
            std::vector<const HierarchyNode*> byOffset = all.nodes;
            std::sort(byOffset.begin(), byOffset.end(),
                      [](const HierarchyNode* a, const HierarchyNode* b) { return a->offset < b->offset; });
            bool same = chunks->size() == byOffset.size();
            for (size_t i = 0; same && i < byOffset.size(); ++i) {
                const auto& c = (*chunks)[i];
                same = c.offset == byOffset[i]->offset && c.bytes == static_cast<uint64_t>(byOffset[i]->byteSize) &&
                       c.count == static_cast<uint32_t>(byOffset[i]->pointCount);
            }
            r.chunkTableMatches = same;
            if (!same) r.errors.push_back("the chunk table does not list the hierarchy's chunks in file order");
        }
    }

    std::unique_ptr<exec::ThreadPool> own;
    exec::ThreadPool* pool = &exec::DefaultPool();
    if (threads > 0) {
        exec::ThreadPoolOptions po;
        po.threads = threads - 1;
        po.zeroWorkers = threads == 1;
        own = std::make_unique<exec::ThreadPool>(po);
        pool = own.get();
    }
    std::mutex mu;
    r.depths.resize(static_cast<size_t>(r.maxDepth) + 1);
    const double halfStep[3] = {h.scale[0] / 2, h.scale[1] / 2, h.scale[2] / 2};
    exec::ParallelFor(*pool, 0, all.nodes.size(), 1, [&](size_t i) {
        const HierarchyNode& n = *all.nodes[i];
        RawPoints pts;
        Status s = DecodeNodeRaw(*file, n, laz::kLayersXyz, pts);
        const Box3 b = file->hierarchy().NodeBounds(n.key);
        uint64_t outside = 0, outsideTol = 0, shared = 0;
        const bool inner = parents.count(KeyOf(n.key)) != 0;
        if (s.ok()) {
            std::unordered_set<uint64_t> cells;
            const double w = (b.maxX - b.minX) / 128;
            for (size_t k = 0; k < pts.size(); ++k) {
                const double x = MulAddUnfused(pts.x[k], h.scale[0], h.offset[0]);
                const double y = MulAddUnfused(pts.y[k], h.scale[1], h.offset[1]);
                const double z = MulAddUnfused(pts.z[k], h.scale[2], h.offset[2]);
                if (!b.Contains(x, y, z)) {
                    ++outside;
                    if (x < b.minX - halfStep[0] || x > b.maxX + halfStep[0] || y < b.minY - halfStep[1] ||
                        y > b.maxY + halfStep[1] || z < b.minZ - halfStep[2] || z > b.maxZ + halfStep[2])
                        ++outsideTol;
                }
                if (inner) {
                    const auto cx = static_cast<uint64_t>(std::clamp((x - b.minX) / w, 0.0, 127.0));
                    const auto cy = static_cast<uint64_t>(std::clamp((y - b.minY) / w, 0.0, 127.0));
                    const auto cz = static_cast<uint64_t>(std::clamp((z - b.minZ) / w, 0.0, 127.0));
                    if (!cells.insert((cx << 14) | (cy << 7) | cz).second) ++shared;
                }
            }
        }
        std::lock_guard lock(mu);
        if (!s.ok()) {
            r.errors.push_back("node " + std::to_string(n.key.d) + "-" + std::to_string(n.key.x) + "-" +
                               std::to_string(n.key.y) + "-" + std::to_string(n.key.z) + ": " + std::string(s.message()));
            return;
        }
        r.decodedPoints += pts.size();
        r.outside += outside;
        r.outsideTolerance += outsideTol;
        CopcDepthStats& d = r.depths[static_cast<size_t>(n.key.d)];
        ++d.nodes;
        d.points += pts.size();
        if (inner) {
            ++d.innerNodes;
            d.innerPoints += pts.size();
            d.innerSharedCells += shared;
        }
    });
    if (r.outsideTolerance)
        r.errors.push_back(std::to_string(r.outsideTolerance) + " points lie outside their node by more than half a scale step");
    if (r.decodedPoints != r.hierarchyPoints && r.errors.empty())
        r.errors.push_back("decoded " + std::to_string(r.decodedPoints) + " points");
    return r;
}

}  // namespace ddbt::copc::build
