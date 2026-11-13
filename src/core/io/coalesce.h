// SPDX-License-Identifier: MPL-2.0
#pragma once

// Read coalescing: requests for nearby byte ranges (TIFF tiles of one row, COPC nodes of one
// subtree) are merged into fewer, larger reads. Worth it for remote sources (one HTTP request
// instead of many) and for cold local reads (fewer syscalls, better read-ahead).

#include <cstdint>
#include <span>
#include <vector>

#include "core/util/status.h"

namespace ddbt::io {

class ByteSource;

struct ByteRange {
    uint64_t offset = 0;
    uint64_t length = 0;
};

struct CoalesceOptions {
    /// Ranges separated by at most this many unrequested bytes are merged.
    uint64_t maxGap = uint64_t{64} << 10;
    /// A merged read never grows past this (a single larger range is still read whole).
    uint64_t maxLength = uint64_t{16} << 20;
};

/// One merged read: covers requests order[first .. first + count) (indexes into the request
/// array, sorted by offset).
struct CoalescedRead {
    uint64_t offset = 0;
    uint64_t length = 0;
    uint32_t first = 0;
    uint32_t count = 0;
};

/// Merges `ranges`. `order` receives the request indexes sorted by offset; each read refers to
/// a slice of it. Overlapping and duplicate ranges are fine. Empty ranges are skipped.
std::vector<CoalescedRead> CoalesceRanges(std::span<const ByteRange> ranges, const CoalesceOptions& options,
                                          std::vector<uint32_t>& order);

/// Reads every range into its output (outs[i].size() must equal ranges[i].length), merging
/// nearby ranges. Memory-resident sources are served by memcpy directly.
Status ReadRanges(const ByteSource& source, std::span<const ByteRange> ranges, std::span<const std::span<uint8_t>> outs,
                  const CoalesceOptions& options = {});

}  // namespace ddbt::io
