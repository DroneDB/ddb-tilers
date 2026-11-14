// SPDX-License-Identifier: MPL-2.0
//
// Cloud: one COPC file as the tiler sees it, immutable and shared by every request for the same
// (path, size, mtime), or (URL, size, version) for a remote file read through the host's remote
// reader (io/remote_source.h). Cloud::Get() looks it up in a process-wide registry (one stat per
// call; a changed file gets a new Cloud, and its CopcFile a new id, so the node cache never serves the
// old nodes; the old Cloud lives on while requests hold it). Concurrent first requests open the
// file once.
//
// What PointCloudTiler's constructor and getCopcInfo compute once (DroneDB 2.4.1 pctiler.cpp,
// pointcloud.cpp), and the extras of the default mode:
//  - the CRS (copc::CopcFile::Crs: WKT2, PROJJSON, WKT or GeoTIFF keys, PDAL's order) and the two
//    transforms of the baseline, cloud -> EPSG:3857 and EPSG:3857 -> cloud, each prepared as
//    OGRCreateCoordinateTransformation prepares it (no area of interest);
//  - the baseline's footprint: the two corners (min, max) of the header bounds through the
//    horizontal part of the CRS, with its refusals (heights beyond +-30 km, a transform that
//    fails); its tile zoom range;
//  - a densified footprint (32 points of the bounds' outline; for a geocentric CRS the 12 edges
//    of the 3-D box) for the default mode, which also tiles clouds the baseline refuses;
//  - the ramp range of colourless clouds: the header z bounds, also as filters.colorinterp
//    receives them (through PDAL's option text, 15 significant digits);
//  - the per-cloud 16-bit colour decision (CopcFile::Uses16BitColor);
//  - the order in which DroneDB's CopcNodeReader visits the nodes (hierarchy pages read with a
//    stack, entries in file order), for the compat mode.
#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "copc/copc/copc_file.h"
#include "copc/tiler/kernels.h"
#include "core/crs/crs.h"
#include "core/crs/transform.h"

namespace ddbt::copc::tiler {

struct Rect {
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
};

class Cloud {
public:
    /// The shared Cloud of `path` (a file or an http(s) URL) for its current (size, mtime or version).
    /// Throws cog::tiler::TileError: kNotFound when the file does not exist, kInvalidArgs when it
    /// is not a COPC file DroneDB can tile ("Cannot get COPC info for ...", "COPC file has no WKT
    /// SRS: ..."), kNotImplemented for a URL without a remote reader, kException when a remote read
    /// fails.
    static std::shared_ptr<const Cloud> Get(const std::string& path);
    /// Drops every registry entry (tests; Clouds in use stay alive).
    static void ClearRegistry();
    /// Opens `path` without the registry. Throws like Get.
    static std::shared_ptr<const Cloud> Open(const std::string& path);

    const std::string& path() const noexcept { return path_; }
    const CopcFile& file() const noexcept { return *file_; }
    const std::shared_ptr<const CopcFile>& sharedFile() const noexcept { return file_; }
    const crs::Crs& crs() const noexcept { return crs_; }
    bool geocentric() const noexcept { return geocentric_; }
    /// Cloud CRS -> EPSG:3857 (DroneDB's toMercator).
    const crs::Transform& toMercator() const noexcept { return *toMercator_; }
    /// EPSG:3857 -> cloud CRS (DroneDB's toCloud).
    const crs::Transform& toCloud() const noexcept { return *toCloud_; }

    /// The baseline's footprint in EPSG:3857 (oMinX = transformed min corner x, ... as
    /// PointCloudTiler takes them, not re-sorted); nullopt when the baseline has none, with the
    /// reason in footprintError().
    const std::optional<Rect>& compatFootprint() const noexcept { return compatFootprint_; }
    const std::string& footprintError() const noexcept { return footprintError_; }
    /// The baseline's tMinZ (zoomForLength of the footprint's smaller side) for 256 px tiles.
    int compatMinZoom() const noexcept { return compatMinZoom_; }
    /// Bounding box in EPSG:3857 of the densified outline of the cloud (default mode); nullopt
    /// when no point of it transforms.
    const std::optional<Rect>& footprint() const noexcept { return footprint_; }

    /// Point formats 7 and 8 (Red, Green, Blue dimensions).
    bool hasColor() const noexcept { return hasColor_; }
    /// The per-cloud 16-bit decision (CompactNode colours are reduced with it).
    bool color16() const noexcept { return color16_; }

    const kernels::Scaling& scaling() const noexcept { return scaling_; }
    /// Header z bounds (the ramp range of the default mode, the z origin of splat keys).
    double zMin() const noexcept { return zMin_; }
    double zMax() const noexcept { return zMax_; }
    /// The ramp range filters.colorinterp gets in the baseline: the header z bounds written with
    /// 15 significant digits (pdal::Options) and read back.
    double compatRampMin() const noexcept { return compatRampMin_; }
    double compatRampMax() const noexcept { return compatRampMax_; }
    /// For a geocentric CRS: the ellipsoidal heights spanned by the corners of the header box.
    double heightMin() const noexcept { return heightMin_; }
    double heightMax() const noexcept { return heightMax_; }

    /// Position of a data node (by HierarchyNode::Id) in the order DroneDB's CopcNodeReader
    /// visits nodes; ~0u when unknown. Computed on first use (reads every hierarchy page).
    uint32_t DroneDbRank(uint64_t nodeId) const;

private:
    Cloud() = default;
    static std::shared_ptr<const Cloud> OpenImpl(const std::string& path, uint64_t size);

    std::string path_;
    std::shared_ptr<const CopcFile> file_;
    crs::Crs crs_;
    bool geocentric_ = false;
    std::shared_ptr<const crs::Transform> toMercator_, toCloud_;
    std::optional<Rect> compatFootprint_;
    std::string footprintError_;
    int compatMinZoom_ = 0;
    std::optional<Rect> footprint_;
    bool hasColor_ = false;
    bool color16_ = false;
    kernels::Scaling scaling_;
    double zMin_ = 0, zMax_ = 0;
    double compatRampMin_ = 0, compatRampMax_ = 0;
    double heightMin_ = 0, heightMax_ = 0;

    mutable std::once_flag rankOnce_;
    mutable std::unordered_map<uint64_t, uint32_t> rank_;
};

/// The horizontal part of a compound CRS (its first component); the CRS itself otherwise.
crs::Crs HorizontalCrs(const crs::Crs& c);
/// Metres per unit of the vertical axis of a compound CRS (GDAL's GetTargetLinearUnits
/// ("VERT_CS")); 1 for other CRSs.
double VerticalUnitInMeters(const crs::Crs& c);

/// A double as pdal::Options passes it on: written with 15 significant digits and read back.
double PdalOptionDouble(double v);

}  // namespace ddbt::copc::tiler
