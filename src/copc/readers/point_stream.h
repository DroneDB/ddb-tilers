// SPDX-License-Identifier: MPL-2.0
//
// The points of a point cloud in file order, for a host that reads them itself (a sample of them):
// the readers of the COPC builder decode a few tasks at once and hand their points over task after
// task, so the order is the order of the file whatever the threads.
#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include "core/exec/thread_pool.h"
#include "core/util/status.h"

namespace ddbt::copc::readers {

/// A batch of points: `count` coordinates (x, y, z interleaved) and, when the cloud has colours,
/// their red, green and blue (16-bit: LAS as stored, 8-bit formats scaled by 257); null otherwise.
/// Return false to stop the reading.
using PointBatchFn = std::function<bool(size_t count, const double* xyz, const uint16_t* rgb)>;

/// Reads every point of `path` (any input OpenReader opens) in file order. Errors: those of
/// OpenReader and of the reads; kCancelled when `batch` returned false.
Status ForEachPoint(const std::string& path, exec::ThreadPool& pool, const PointBatchFn& batch);

}  // namespace ddbt::copc::readers
