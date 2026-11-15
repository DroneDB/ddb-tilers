// SPDX-License-Identifier: MPL-2.0
#include "copc/readers/point_stream.h"

#include <algorithm>
#include <cstring>
#include <memory>
#include <vector>

#include "copc/readers/point_reader.h"
#include "core/exec/parallel_for.h"

namespace ddbt::copc::readers {
namespace {

/// The points of one task: coordinates and colours, decoded before they are handed over
struct TaskPoints {
    std::vector<double> xyz;
    std::vector<uint16_t> rgb;
};

class CollectSink final : public BatchSink {
public:
    CollectSink(TaskPoints& out, size_t recordSize, bool colour)
        : out_(out), recordSize_(recordSize), colour_(colour) {}
    Status Consume(Batch& b) override {
        if (b.xyz == nullptr) return InternalError("point stream: a reader gave quantised coordinates");
        out_.xyz.insert(out_.xyz.end(), b.xyz, b.xyz + 3 * b.count);
        if (colour_) {
            for (size_t i = 0; i < b.count; ++i) {
                const uint8_t* rec = b.records + i * recordSize_ + rec::kRgb;
                for (size_t c = 0; c < 3; ++c) {
                    uint16_t v;
                    std::memcpy(&v, rec + 2 * c, 2);
                    out_.rgb.push_back(v);
                }
            }
        }
        return Status();
    }

private:
    TaskPoints& out_;
    size_t recordSize_;
    bool colour_;
};

}  // namespace

Status ForEachPoint(const std::string& path, exec::ThreadPool& pool, const PointBatchFn& batch) {
    DDBT_ASSIGN_OR_RETURN(std::unique_ptr<PointReader> reader, OpenReader(path));
    const bool colour = reader->Info().hasRgb;
    OutputSpec spec;
    spec.format = 7;  // the colours of a record of format 7 follow its base fields
    spec.quantized = false;
    const size_t tasks = reader->TaskCount();
    // A window of tasks decoded at once, then handed over in order: at most a few hundred MB
    const size_t window = std::max<size_t>(1, std::min<size_t>(pool.Size(), 8));
    for (size_t first = 0; first < tasks; first += window) {
        const size_t n = std::min(window, tasks - first);
        std::vector<TaskPoints> points(n);
        std::vector<Status> status(n);
        exec::ParallelFor(pool, 0, n, 1, [&](size_t k) {
            CollectSink sink(points[k], spec.RecordSize(), colour);
            status[k] = reader->Read(first + k, spec, sink);
        });
        for (size_t k = 0; k < n; ++k) {
            DDBT_TRY(status[k]);
            const size_t count = points[k].xyz.size() / 3;
            if (count == 0) continue;
            if (!batch(count, points[k].xyz.data(), colour ? points[k].rgb.data() : nullptr))
                return Status(StatusCode::kCancelled, "the reading of the points was stopped");
        }
    }
    return Status();
}

}  // namespace ddbt::copc::readers
