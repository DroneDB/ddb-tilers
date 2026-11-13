// SPDX-License-Identifier: MPL-2.0
//
// Structure checks of a COPC file (any writer): the COPC info VLR first at 375 (CopcFile
// validates it), spacing = 2 halfsize / 128, the hierarchy counts sum to the header count,
// every node decodes to its count, every point lies inside its node (PDAL's Key::bounds
// arithmetic, closed; also counted with a tolerance of half a scale step), the chunk table
// lists the data nodes in file order with their counts and sizes, and per depth how many points
// of the inner nodes share a cell of the node's 128^3 grid (0 for ddbt's LOD except points
// hoisted from small leaves).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "core/util/status.h"

namespace ddbt::copc::build {

struct CopcDepthStats {
    uint64_t nodes = 0;
    uint64_t points = 0;
    uint64_t innerNodes = 0;
    uint64_t innerPoints = 0;
    uint64_t innerSharedCells = 0;  // points of inner nodes in a cell already taken in that node
};

struct CopcCheckReport {
    uint64_t headerPoints = 0;
    uint64_t hierarchyPoints = 0;
    uint64_t decodedPoints = 0;
    uint64_t dataNodes = 0;
    int maxDepth = 0;
    uint64_t outside = 0;           // strictly outside the node bounds
    uint64_t outsideTolerance = 0;  // further than half a scale step
    bool chunkTableMatches = false;
    std::vector<CopcDepthStats> depths;
    std::vector<std::string> errors;
    bool ok() const noexcept { return errors.empty(); }
};

/// Checks `path` (decoding every node on `threads` threads, 0: the default pool).
Expected<CopcCheckReport> CheckCopc(const std::string& path, size_t threads = 0);

}  // namespace ddbt::copc::build
