// SPDX-License-Identifier: MPL-2.0
//
// The COPC builder:
//
//   1. Plan: open the inputs (LAS / LAZ / PLY / E57 / XYZ / PTS), choose the output point
//      format (8 with NIR, 7 with colour, 6 otherwise: a colourless cloud stays colourless),
//      the scale / offset (a single LAS input keeps its own), the cube (from the header bounds
//      of LAS inputs, else from a bounds pass), then estimate the point density from a sample
//      (the first points of every LAZ chunk, strided LAS records, the bounds pass) as a Morton
//      histogram and cut the octree into bins.
//   2. Scatter: one parallel pass over the inputs (a task per LAZ chunk, LAS range, text range,
//      PLY stride, E57 packet range) decodes, converts, keys (BMI2 pdep) and appends every
//      point to its bin, in memory or in one segmented spill file when the memory budget is
//      too small.
//   3. Process: bins are grouped into partitions by their exact counts; partitions are sorted
//      and built in parallel (lod.h: deterministic bottom-up thinning), their nodes encoded as
//      soon as final; a committer writes them in the global emission order and builds the top
//      of the tree as partitions complete.
//   4. Finish: chunk table, hierarchy pages, header and VLRs; a JSON sidecar with the bounds,
//      the count, the WKT, the EPSG:4326 footprint, the dimensions and the 16-bit colour hint.
//
// The output is byte-identical whatever the number of threads and the memory budget.
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "copc/build/key_mapper.h"
#include "copc/build/lod.h"
#include "core/util/status.h"

namespace ddbt::copc::build {

/// Progress: fraction 0..1 and the phase ("planning", "reading", "building", "writing").
using ProgressFn = std::function<void(double fraction, const char* phase)>;

struct CopcBuildOptions {
    /// More inputs merged with the first one.
    std::vector<std::string> extraInputs;
    /// Threads (0: the default pool of the process, DDBT_THREADS or every CPU).
    size_t threads = 0;
    /// Memory budget in bytes (0: the default, `DDBT_COPC_BUILD_MEM_MB` MiB when set, else
    /// 4 GiB, at most half the physical memory). A cap on the build's own allocations: records
    /// in memory, spill staging, partitions in flight and waiting to be committed, reader and
    /// encoder state per thread (the model is at the top of copc_builder.cpp).
    uint64_t memoryBudget = 0;
    /// Spill directory ("" = the directory of the output).
    std::string tempDir;
    /// Spill even when the points fit the budget (tests).
    bool forceSpill = false;
    /// Spilling: the share of the budget (after the fixed part and the threads) the bins kept
    /// in memory may take (< 0: 0.7, or 0 with forceSpill; tests).
    double spillMemoryFraction = -1;
    /// glibc: map blocks of 1 MiB or more and give them back when freed (mallopt
    /// M_MMAP_THRESHOLD, process-wide, set once), so freed partition buffers do not stay
    /// resident in per-thread arenas.
    bool tuneAllocator = true;
    /// Upper bound on the points of one partition (0: from the budget and the thread count).
    uint64_t maxPartitionPoints = 0;
    LodParams lod;
    /// Write the metadata sidecar (`metadataPath`, default "<output>.meta.json").
    bool metadata = true;
    std::string metadataPath;
    /// Creation date of the header for inputs that are not LAS (0: today, UTC).
    uint16_t creationDoy = 0, creationYear = 0;
    ProgressFn progress;
};

struct CopcBuildReport {
    uint64_t points = 0;
    uint64_t nodes = 0;
    uint64_t bins = 0;
    uint64_t partitions = 0;
    uint64_t topNodes = 0;
    int maxDepth = 0;
    int leafLevel = 0;  // the LOD's leaf level (lod.h)
    int format = 6;
    int extraBytes = 0;
    std::array<double, 3> scale{}, offset{};
    Cube cube;
    double spacing = 0;
    bool spilled = false;
    uint64_t spillBytes = 0;
    uint64_t memoryBudget = 0;
    uint64_t peakStoreBytes = 0;
    uint64_t peakProcessBytes = 0;  // largest reservation of the partitions and top nodes held
    uint64_t maxPartitionPoints = 0;
    uint64_t largestPartition = 0;
    bool restarted = false;  // header bounds were wrong: planned again from the real ones
    bool color16 = false;
    uint64_t fileSize = 0;
    double secondsPlan = 0, secondsScatter = 0, secondsProcess = 0, secondsTotal = 0;
    double secondsCommitWait = 0;  // the committer waiting for the next partition
    double secondsTopNodes = 0;    // the committer building the top nodes
};

/// Builds the COPC `output` from `input` (and options.extraInputs). The output appears
/// atomically; on failure nothing is left behind.
Status BuildCopc(const std::string& input, const std::string& output, const CopcBuildOptions& options,
                 CopcBuildReport* report = nullptr);

/// The default memory budget: the one set by SetDefaultCopcMemoryBudget, else
/// `DDBT_COPC_BUILD_MEM_MB` MiB when set (an error when it is not a positive integer), else 4 GiB
/// but at most half the physical memory.
Expected<uint64_t> DefaultCopcMemoryBudget();

/// Sets the process-wide default memory budget of the builds that do not give one (a host's
/// configuration, which wins over `DDBT_COPC_BUILD_MEM_MB`); 0 restores the default order.
void SetDefaultCopcMemoryBudget(uint64_t bytes) noexcept;

}  // namespace ddbt::copc::build
