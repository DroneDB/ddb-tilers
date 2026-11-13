// SPDX-License-Identifier: MPL-2.0
//
// The point-cloud branch of ddbtMemoryTileFmt (internal: the tile dispatch of src/api calls it
// for COPC inputs). Same arguments, checks, codes and messages as ddbtMemoryTileFmt; the tile
// comes from copc::tiler::RenderTile (DroneDB 2.4.1's PointCloudTiler semantics).
#pragma once

#include <cstdint>

#include "ddbt/ddbt.h"

namespace ddbt::api {

/// DroneDB's isCopcPath: the name ends with ".copc.laz" (case-insensitive).
bool IsCopcPath(const char* path) noexcept;

/// ddbtMemoryTileFmt for a COPC file. Never throws.
ddbtErr CopcMemoryTile(const char* inputPath, int tz, int tx, int ty, uint8_t** outBuffer, int* outBufferSize,
                       int tileSize, bool tms, bool forceRecreate, const char* inputPathHash,
                       const char* outputFormat) noexcept;

}  // namespace ddbt::api
