// SPDX-License-Identifier: MPL-2.0
//
// The CRS of a LAS / COPC file, chosen as PDAL's LAS readers do (io/private/las/Srs.cpp):
// WKT2 (LASF_Projection 4224), then PDAL's PROJJSON (PDAL 4225), then WKT (LASF_Projection
// 2112) when the WKT global-encoding bit is set or the format is 6-10, else the GeoTIFF keys
// (LASF_Projection 34735-34737, interpreted by cog/tiff's port of libgeotiff / GDAL). When the
// preferred record is absent, the other of WKT / GeoTIFF keys is used.
#pragma once

#include <string>

#include "copc/las/las_header.h"
#include "core/crs/crs.h"
#include "core/util/status.h"

namespace ddbt::copc::las {

enum class CrsSource { kNone, kWkt2, kProjJson, kWkt, kGeoKeys };

struct CrsDefinition {
    CrsSource source = CrsSource::kNone;
    /// What crs::Crs::FromUserInput / FromWkt gets: WKT, PROJJSON, or the definition built
    /// from the GeoTIFF keys ("EPSG:2154", "EPSG:2154+5720", WKT). Empty when kNone.
    std::string text;
};

/// The CRS definition of the file (source kNone when it has none). Errors when GeoTIFF keys
/// are present but cannot be interpreted.
Expected<CrsDefinition> FindCrsDefinition(const LasMetadata& meta);

/// The CRS of the file (an invalid Crs when it has none). Errors when PROJ cannot parse it.
Expected<crs::Crs> ResolveCrs(const LasMetadata& meta);

}  // namespace ddbt::copc::las
