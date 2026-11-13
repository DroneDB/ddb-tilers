// SPDX-License-Identifier: MPL-2.0
//
// CopcFile: an open COPC file, immutable and shared by every thread (std::shared_ptr<const>).
//
//   auto file = CopcFile::Open(path);                         // Expected<shared_ptr<const CopcFile>>
//   auto sel = (*file)->hierarchy().Select({Box3::Xy(...), depthEnd});
//   NodeDecoder dec(*file);
//   auto node = dec.DecodeCompact(*sel->nodes[0]);            // Expected<CompactNode>
//
// The bytes come from a ByteSource: a local path opens with OpenByteSource(kAuto), i.e. a
// mapping on Linux and positional reads on Windows; a remote object comes as a
// RangeCallbackSource, whose hierarchy pages are then read lazily, on the first query that
// reaches them. Open() validates what readers.copc validates (LAS 1.4, formats 6-8, the COPC
// info VLR first at offset 375) plus the LAZ VLR and the hierarchy range.
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string_view>

#include "copc/copc/hierarchy.h"
#include "copc/las/las_header.h"
#include "core/crs/crs.h"
#include "core/io/byte_source.h"
#include "core/util/status.h"

namespace ddbt::copc {

enum class HierarchyLoad : uint8_t {
    kAuto,   // eager when the source is memory-resident (ByteSource::Data()), else lazy
    kEager,  // every page at Open()
    kLazy,   // root page at Open(), the others on first use
};

struct CopcOpenOptions {
    HierarchyLoad hierarchy = HierarchyLoad::kAuto;
    io::SourceKind sourceKind = io::SourceKind::kAuto;  // for Open(path)
    las::ReadOptions las;
};

class CopcFile {
public:
    static Expected<std::shared_ptr<const CopcFile>> Open(std::string_view utf8Path, const CopcOpenOptions& options = {});
    static Expected<std::shared_ptr<const CopcFile>> Open(std::shared_ptr<const io::ByteSource> source,
                                                          const CopcOpenOptions& options = {});
    ~CopcFile();

    const io::ByteSource& source() const noexcept { return *source_; }
    const std::shared_ptr<const io::ByteSource>& sharedSource() const noexcept { return source_; }
    const las::LasMetadata& metadata() const noexcept { return meta_; }
    const las::LasHeader& header() const noexcept { return meta_.header; }
    const las::CopcInfo& info() const noexcept { return *meta_.copcInfo; }
    const HierarchyIndex& hierarchy() const noexcept { return *hierarchy_; }

    int pointFormat() const noexcept { return meta_.header.pointFormat; }
    int extraBytes() const noexcept { return meta_.header.ExtraBytesPerPoint(); }
    bool hasRgb() const noexcept { return meta_.header.pointFormat >= 7; }
    bool hasNir() const noexcept { return meta_.header.pointFormat == 8; }
    double spacing() const noexcept { return meta_.copcInfo->spacing; }
    /// The octree cube (center +- halfsize), as PDAL's rootNodeExtent.
    Box3 RootCube() const noexcept;

    /// Process-unique id of this open file (cache keys: (id, node offset)).
    uint64_t id() const noexcept { return id_; }

    /// The CRS (las::ResolveCrs), resolved on first call and cached; an invalid Crs when the
    /// file has none.
    Expected<crs::Crs> Crs() const;

    /// The per-cloud colour depth decision: true when a red, green or blue value above 255
    /// occurs in the sample: whole depths from the root down until they hold
    /// kColorDecisionMinPoints points, at most depths 0 to kColorDecisionMaxDepth (decoded once,
    /// XY + RGB layers only, then cached). Colours of 16-bit clouds are reduced with v >> 8,
    /// others are taken as 8-bit (clamped to 255). False for format 6.
    Expected<bool> Uses16BitColor() const;
    static constexpr uint64_t kColorDecisionMinPoints = 4096;
    static constexpr int kColorDecisionMaxDepth = 2;

private:
    CopcFile() = default;

    std::shared_ptr<const io::ByteSource> source_;
    las::LasMetadata meta_;
    std::unique_ptr<HierarchyIndex> hierarchy_;
    uint64_t id_ = 0;

    mutable std::once_flag crsOnce_;
    mutable std::optional<Expected<crs::Crs>> crs_;
    mutable std::mutex colorMutex_;
    mutable std::atomic<int> color16_{-1};  // -1 unknown, 0 / 1
};

}  // namespace ddbt::copc
