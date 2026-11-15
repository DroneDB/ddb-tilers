// SPDX-License-Identifier: MPL-2.0
#include "copc/readers/point_info.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "copc/readers/point_reader.h"
#include "core/exec/parallel_for.h"

namespace ddbt::copc::readers {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

/// Bounds and count of the points of one task.
struct TaskBounds {
    std::array<double, 3> min{kInf, kInf, kInf};
    std::array<double, 3> max{-kInf, -kInf, -kInf};
    uint64_t points = 0;
};

class BoundsSink final : public BatchSink {
public:
    explicit BoundsSink(TaskBounds& bounds) : bounds_(bounds) {}
    Status Consume(Batch& b) override {
        if (b.xyz == nullptr) return InternalError("point summary: a reader gave quantised coordinates");
        for (size_t i = 0; i < b.count; ++i) {
            const double* p = b.xyz + 3 * i;
            for (size_t a = 0; a < 3; ++a) {
                if (!std::isfinite(p[a])) return Corrupt("a point has a non-finite coordinate");
                bounds_.min[a] = std::min(bounds_.min[a], p[a]);
                bounds_.max[a] = std::max(bounds_.max[a], p[a]);
            }
        }
        bounds_.points += b.count;
        return Status();
    }

private:
    TaskBounds& bounds_;
};

}  // namespace

Expected<PointCloudSummary> SummarizePointCloud(const std::string& path, exec::ThreadPool& pool, bool readBounds) {
    DDBT_ASSIGN_OR_RETURN(std::unique_ptr<PointReader> reader, OpenReader(path));
    const SourceInfo& info = reader->Info();
    PointCloudSummary summary;
    summary.kind = info.kind;
    summary.wkt = info.statedWkt.empty() ? info.wkt : info.statedWkt;
    summary.dimensions = info.dimensions;
    if (info.countExact && (info.boundsKnown || !readBounds)) {
        summary.pointCount = info.pointCount;
        summary.hasBounds = info.boundsKnown;
        summary.min = info.min;
        summary.max = info.max;
        return summary;
    }

    // One pass over every task for the exact count and bounds
    OutputSpec spec;
    spec.quantized = false;
    std::vector<TaskBounds> bounds(reader->TaskCount());
    std::vector<Status> status(reader->TaskCount());
    exec::ParallelFor(pool, 0, reader->TaskCount(), 1, [&](size_t t) {
        BoundsSink sink(bounds[t]);
        status[t] = reader->Read(t, spec, sink);
    });
    TaskBounds all;
    for (size_t t = 0; t < bounds.size(); ++t) {
        DDBT_TRY(status[t]);
        for (size_t a = 0; a < 3; ++a) {
            all.min[a] = std::min(all.min[a], bounds[t].min[a]);
            all.max[a] = std::max(all.max[a], bounds[t].max[a]);
        }
        all.points += bounds[t].points;
    }
    summary.pointCount = all.points;
    summary.hasBounds = all.points > 0;
    if (all.points > 0) {
        summary.min = all.min;
        summary.max = all.max;
    }
    return summary;
}

}  // namespace ddbt::copc::readers
