// SPDX-License-Identifier: MPL-2.0
//
// GeoTIFF georeferencing: GeoKeys, the geotransform GDAL derives from ModelPixelScale /
// ModelTiepoint / ModelTransformation (PixelIsArea / PixelIsPoint), ground control points,
// world-file sidecars, and the CRS of the keys built the way libgeotiff's GTIFGetDefn and
// GDAL's GTIFGetOGISDefnAsOSR do (EPSG codes, user-defined projections, ESRI PE strings,
// TOWGS84, vertical CRSs).
#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "cog/tiff/tiff_reader.h"
#include "core/crs/crs.h"
#include "core/util/status.h"

namespace ddbt::cog::tiff {

/// GeoKey ids (GeoTIFF 1.1).
namespace geokey {
inline constexpr uint16_t kGTModelType = 1024;
inline constexpr uint16_t kGTRasterType = 1025;
inline constexpr uint16_t kGTCitation = 1026;
inline constexpr uint16_t kGeographicType = 2048;
inline constexpr uint16_t kGeogCitation = 2049;
inline constexpr uint16_t kGeogGeodeticDatum = 2050;
inline constexpr uint16_t kGeogPrimeMeridian = 2051;
inline constexpr uint16_t kGeogLinearUnits = 2052;
inline constexpr uint16_t kGeogLinearUnitSize = 2053;
inline constexpr uint16_t kGeogAngularUnits = 2054;
inline constexpr uint16_t kGeogAngularUnitSize = 2055;
inline constexpr uint16_t kGeogEllipsoid = 2056;
inline constexpr uint16_t kGeogSemiMajorAxis = 2057;
inline constexpr uint16_t kGeogSemiMinorAxis = 2058;
inline constexpr uint16_t kGeogInvFlattening = 2059;
inline constexpr uint16_t kGeogAzimuthUnits = 2060;
inline constexpr uint16_t kGeogPrimeMeridianLong = 2061;
inline constexpr uint16_t kGeogTOWGS84 = 2062;
inline constexpr uint16_t kProjectedCSType = 3072;
inline constexpr uint16_t kPCSCitation = 3073;
inline constexpr uint16_t kProjection = 3074;
inline constexpr uint16_t kProjCoordTrans = 3075;
inline constexpr uint16_t kProjLinearUnits = 3076;
inline constexpr uint16_t kProjLinearUnitSize = 3077;
inline constexpr uint16_t kProjStdParallel1 = 3078;
inline constexpr uint16_t kProjStdParallel2 = 3079;
inline constexpr uint16_t kProjNatOriginLong = 3080;
inline constexpr uint16_t kProjNatOriginLat = 3081;
inline constexpr uint16_t kProjFalseEasting = 3082;
inline constexpr uint16_t kProjFalseNorthing = 3083;
inline constexpr uint16_t kProjFalseOriginLong = 3084;
inline constexpr uint16_t kProjFalseOriginLat = 3085;
inline constexpr uint16_t kProjFalseOriginEasting = 3086;
inline constexpr uint16_t kProjFalseOriginNorthing = 3087;
inline constexpr uint16_t kProjCenterLong = 3088;
inline constexpr uint16_t kProjCenterLat = 3089;
inline constexpr uint16_t kProjCenterEasting = 3090;
inline constexpr uint16_t kProjCenterNorthing = 3091;
inline constexpr uint16_t kProjScaleAtNatOrigin = 3092;
inline constexpr uint16_t kProjScaleAtCenter = 3093;
inline constexpr uint16_t kProjAzimuthAngle = 3094;
inline constexpr uint16_t kProjStraightVertPoleLong = 3095;
inline constexpr uint16_t kProjRectifiedGridAngle = 3096;
inline constexpr uint16_t kVerticalCSType = 4096;
inline constexpr uint16_t kVerticalCitation = 4097;
inline constexpr uint16_t kVerticalDatum = 4098;
inline constexpr uint16_t kVerticalUnits = 4099;
inline constexpr uint16_t kCoordinateEpoch = 5120;
inline constexpr uint16_t kUserDefined = 32767;
}  // namespace geokey

struct GeoKey {
    uint16_t id = 0;
    uint16_t location = 0;  // 0 (inline short), 34735 (shorts), 34736 (doubles), 34737 (ASCII)
    std::vector<uint16_t> shorts;
    std::vector<double> doubles;
    std::string ascii;
};

/// The GeoKeyDirectory of an IFD with its values resolved.
struct GeoKeyDirectory {
    uint16_t version = 0, revision = 0, minorRevision = 0;
    std::map<uint16_t, GeoKey> keys;

    bool empty() const noexcept { return keys.empty(); }
    bool Has(uint16_t id) const noexcept { return keys.count(id) != 0; }
    /// First value as a short (doubles are truncated, as libgeotiff's conversion does).
    std::optional<int> Short(uint16_t id) const;
    /// Value `index` as a double (shorts converted).
    std::optional<double> Double(uint16_t id, size_t index = 0) const;
    /// All double values.
    std::vector<double> Doubles(uint16_t id) const;
    std::optional<std::string> Ascii(uint16_t id) const;
};

/// GeoKeys of `ifd` (empty when the IFD has no GeoKeyDirectory). Malformed directories give
/// Corrupt; keys whose values point outside their arrays are dropped (libgeotiff ignores them).
Expected<GeoKeyDirectory> ReadGeoKeys(const TiffReader& reader, uint32_t ifd);

struct GroundControlPoint {
    double pixel = 0, line = 0, x = 0, y = 0, z = 0;
};

enum class GeoTransformSource : uint8_t { kNone, kInternal, kWorldFile };

struct Georeferencing {
    /// GDAL's geotransform: x = gt[0] + col * gt[1] + row * gt[2], y = gt[3] + col * gt[4] + row * gt[5].
    std::optional<std::array<double, 6>> geoTransform;
    GeoTransformSource source = GeoTransformSource::kNone;
    std::string worldFile;  // the sidecar used, when source == kWorldFile
    bool pixelIsPoint = false;  // GTRasterTypeGeoKey = RasterPixelIsPoint (already applied)
    std::vector<GroundControlPoint> gcps;

    GeoKeyDirectory keys;
    /// What the CRS was built from: "EPSG:xxxx", "EPSG:h+v", or WKT; empty when none.
    std::string crsDefinition;
    std::optional<crs::Crs> crs;
    /// Why no CRS could be built from keys that exist (unsupported transform, ...).
    std::string crsError;
};

/// Geotransform as GDAL's GTiff driver reads it (internal tags first, then .tfw / .tifw /
/// .wld next to `path` when given), GCPs, GeoKeys and the CRS. Errors only for I/O failures;
/// missing or unusable georeferencing leaves the fields empty.
Expected<Georeferencing> ReadGeoreferencing(const TiffReader& reader, std::string_view path = {});

/// The georeferencing GDAL reads from a GeoTIFF embedded in another file (the degenerate GeoTIFF
/// of a GeoJP2 box: GTIFWktFromMemBufEx), which differs from the GTiff driver's: a pixel scale
/// alone gives a geotransform (origin 0), tie points without a scale are GCPs (PixelIsPoint does
/// not move them), a ModelTransformation is not shifted for PixelIsPoint, and a geotransform is
/// reported only when it is not (0, 1, 0, 0, 0, 1). No world file.
Georeferencing ReadEmbeddedGeoreferencing(const TiffReader& reader);

/// The world file GDAL reads for `rasterPath` (FindWorldFile, ParseWorldFile): the geotransform
/// and the sidecar's path, or nothing.
std::optional<std::pair<std::array<double, 6>, std::string>> ReadWorldFile(std::string_view rasterPath);

/// The CRS definition the keys describe ("" when there are no keys or no model). Errors for
/// keys we cannot interpret (the message says which).
Expected<std::string> GeoKeysToCrsDefinition(const GeoKeyDirectory& keys);

/// Parses a world file (6 numbers: A D B E C F, pixel-centre convention) into a geotransform.
std::optional<std::array<double, 6>> ParseWorldFile(std::string_view text);

/// The world file GDAL would use for `rasterPath` (.tfw, .tifw, then .wld, any letter case),
/// or "" when there is none.
std::string FindWorldFile(std::string_view rasterPath);

/// Reads a .prj sidecar (WKT1, ESRI WKT or anything PROJ accepts) into a CRS. GDAL's GTiff
/// driver does not read .prj; this is for other raster formats.
Expected<crs::Crs> ReadPrjFile(std::string_view prjPath);

}  // namespace ddbt::cog::tiff
