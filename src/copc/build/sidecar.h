// SPDX-License-Identifier: MPL-2.0
//
// The metadata sidecar of a COPC build ("<output>.meta.json"), so DroneDB can index the
// result without opening it again: what its getCopcInfo / getPointCloudInfo compute
// (pointCount, bounds, projection, dimensions as PDAL names them, the EPSG:4326 footprint
// polygon and centroid of populatePointCloudFootprint: the min / max corners of the horizontal
// CRS, heights in metres) and what the tiler needs (16-bit colour, spacing, root cube).
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "copc/build/key_mapper.h"
#include "core/util/status.h"

namespace ddbt::copc::build {

struct SidecarInput {
    uint64_t points = 0;
    std::array<double, 3> min{}, max{};
    std::string wkt;
    int format = 6;
    bool color = false;
    bool color16 = false;
    double spacing = 0;
    Cube cube;
    std::vector<std::string> extraDimensions;
};

/// PDAL's dimension names of a LAS 1.4 point format 6-8 file with these extra dimensions.
std::vector<std::string> PdalDimensions(int format, const std::vector<std::string>& extra);

/// The sidecar JSON text.
std::string SidecarJson(const SidecarInput& in);

Status WriteSidecarFile(const std::string& path, const SidecarInput& in);

}  // namespace ddbt::copc::build
