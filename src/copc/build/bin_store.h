// SPDX-License-Identifier: MPL-2.0
//
// Where the scatter pass of the COPC builder puts the records of each bin (a subtree of the
// octree chosen by the plan) until its partition is processed:
//
//   - in memory: per bin, a list of blocks (64 KiB growing to 8 MiB) appended under the bin's
//     mutex;
//   - out of core: one segmented spill file: records are staged per bin (one buffer per bin
//     shared by the writers, under the bin's mutex) and a full buffer is written at an offset
//     reserved atomically (pwrite); the segments of each bin are listed in memory, a partition
//     is read back with one pread per segment.
//     Bins flagged `inMemory` are kept in memory blocks instead (the share of the budget the
//     records may use; the plan flags the first bins, processed first).
//
// The order of the records of a bin depends on the scheduling; the partition sort makes it
// canonical (lod.h).
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "core/util/status.h"

namespace ddbt::copc::build {

/// Records of some bins, loaded for processing: `pieces` point into the store (memory) or into
/// `owned` (spill).
struct LoadedRecords {
    std::vector<std::span<const uint8_t>> pieces;
    std::vector<uint8_t> owned;
    uint64_t count = 0;
};

class BinStore {
public:
    /// A writer context: not thread-safe, one per running task.
    class Writer {
    public:
        virtual ~Writer() = default;
        virtual Status Append(uint32_t bin, const uint8_t* records, size_t count) = 0;
        /// Writes what is staged (spill). Called before the context is reused by another
        /// thread and at the end of the pass.
        virtual Status Flush() = 0;
    };

    virtual ~BinStore() = default;
    virtual std::unique_ptr<Writer> NewWriter() = 0;
    virtual uint64_t Count(uint32_t bin) const = 0;
    /// Records of `bins`, in bin order.
    virtual Status Load(std::span<const uint32_t> bins, LoadedRecords& out) = 0;
    /// Frees the memory of `bins` (memory store).
    virtual void Release(std::span<const uint32_t> bins) = 0;
    virtual uint64_t SpilledBytes() const noexcept { return 0; }
    /// Bytes held in memory by the store right now (blocks or staging buffers).
    virtual uint64_t MemoryBytes() const noexcept = 0;

    static std::unique_ptr<BinStore> CreateMemory(size_t bins, size_t recordSize);
    /// Spill file created in `directory` (deleted when the store is destroyed, or at once where
    /// the OS allows). `stagingBytes`: per bin. `inMemory[b]` != 0: bin b
    /// stays in memory (empty: none).
    static Expected<std::unique_ptr<BinStore>> CreateSpill(const std::string& directory, size_t bins, size_t recordSize,
                                                           size_t stagingBytes, std::vector<uint8_t> inMemory = {});
};

}  // namespace ddbt::copc::build
