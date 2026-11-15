// SPDX-License-Identifier: MPL-2.0
//
// The sources of the region warps (region_warp.h). The renders of a map read the same blocks of the
// same rasters again and again, as GDAL's block cache keeps them for gdalwarp: a level of a TIFF is
// parsed once per version of the file (FileIdentity) and shared by the warps that read it, and its
// decoded blocks come from the tiler's block cache (shared with the tiles and the reads, one
// budget), which the warp producer reads in place (RasterSource::SharedDataBlock).
#pragma once

#include <memory>
#include <string>

#include "cog/build/tiff_source.h"
#include "core/util/status.h"

namespace ddbt::cog::warp {

/// Level `level` of the TIFF at `path` (0: the image, i: overview i - 1), shared by the warps of this
/// version of the file.
Expected<std::shared_ptr<const build::TiffSource>> SharedTiffLevel(const std::string& path, int level);

/// `source` (level `level` of `path`) reading its blocks and mask blocks from the tiler's cache.
std::unique_ptr<build::RasterSource> WithCachedBlocks(std::shared_ptr<const build::TiffSource> source,
                                                      const std::string& path, int level);

/// Drops the shared levels (tests, benchmarks).
void ClearRegionSources();

}  // namespace ddbt::cog::warp
