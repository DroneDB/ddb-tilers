// SPDX-License-Identifier: MPL-2.0
//
// The summary of a point cloud a host indexes (DroneDB's point-cloud entries): its point count,
// bounds, CRS and dimensions, read with the readers of the COPC builder. A reader whose header
// gives a trusted count and bounds (LAS / LAZ, so a COPC) answers from the header alone; the other
// formats are read once, in parallel, when their count or (if asked) their bounds is needed.
#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include "core/exec/thread_pool.h"
#include "core/util/status.h"

namespace ddbt::copc::readers {

struct PointCloudSummary {
    std::string kind;  // "las", "laz", "ply", "e57", "xyz", "pts"
    uint64_t pointCount = 0;
    /// Bounds of the points (minimum, maximum) when hasBounds.
    bool hasBounds = false;
    std::array<double, 3> min{}, max{};
    /// CRS as WKT as PDAL reports it (a WKT record as stored), empty when unknown.
    std::string wkt;
    /// Dimension names as PDAL calls them, in PDAL's order.
    std::vector<std::string> dimensions;
};

/// The summary of the point cloud at `path` (any input OpenReader opens). Its points are read only
/// when the header has no exact count, or with `readBounds` when it has no bounds; `hasBounds` says
/// whether the bounds are known. Errors: those of OpenReader and of the reads (a damaged file, a
/// non-finite coordinate).
Expected<PointCloudSummary> SummarizePointCloud(const std::string& path, exec::ThreadPool& pool, bool readBounds);

}  // namespace ddbt::copc::readers
