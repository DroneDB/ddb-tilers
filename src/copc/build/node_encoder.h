// SPDX-License-Identifier: MPL-2.0
//
// Encodes the points of one COPC node as one LAZ chunk (layered, point format 6-8 + extra
// bytes, laz-perf fork's compressor), as COPC requires: the chunk of a node is a stand-alone
// LAZ chunk that any LAZ 1.4 reader decodes.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "core/util/status.h"

namespace ddbt::copc::build {

/// Compresses `count` records (point format `recordFormat`, 6, 7 or 8, then `extraBytes` bytes;
/// contiguous) into `out` (cleared first) as point format `format`, 6 to `recordFormat`. A lower
/// format keeps the fields the formats share, which lead every record (formats 6, 7 and 8 differ
/// only by the colour and the near infrared after them), and the extra bytes: a cloud whose
/// records were read with colour can be written without it. Thread-safe (a compressor per call,
/// or the calling thread's).
Status EncodeNode(int format, int recordFormat, int extraBytes, const uint8_t* records, size_t count,
                  std::vector<uint8_t>& out);

}  // namespace ddbt::copc::build
