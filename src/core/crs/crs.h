// SPDX-License-Identifier: MPL-2.0
#pragma once

// A coordinate reference system, parsed once by PROJ and kept as a canonical definition that
// any thread's context can re-create. Immutable and cheap to copy (shared state).

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace ddbt::crs {

/// Error of the crs module (a CRS that PROJ cannot parse, no operation between two CRSs).
class CrsError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/// Sets the directory that holds proj.db for the PROJ contexts created from now on ("" restores
/// the default order: env DDBT_PROJ_DATA, then the share/proj of the PROJ this library was built
/// with, then PROJ's own search). See core/crs/proj_context.h.
void SetProjDataDir(std::string dir);

/// The directory in use ("" when PROJ's own search applies).
std::string ProjDataDir();

enum class CrsType {
    Geographic2D,
    Geographic3D,
    Geocentric,
    Projected,
    Vertical,
    Compound,
    Engineering,
    Bound,
    Other,
};

/// Longitude / latitude box in degrees. west > east crosses the antimeridian (as GDAL's and
/// PROJ's areas of interest do).
struct LonLatBox {
    double west = 0, south = 0, east = 0, north = 0;
    friend bool operator==(const LonLatBox&, const LonLatBox&) = default;
};

enum class WktFlavor { Wkt2_2019, Wkt1Gdal, Wkt1Esri };

class Crs {
public:
    /// An empty CRS (Valid() false).
    Crs() = default;

    /// EPSG:code. Throws CrsError if proj.db does not know it.
    static Crs FromEpsg(int code);
    /// Anything proj_create accepts: "EPSG:31370", "AUTH:CODE", WKT1/WKT2, PROJJSON, a PROJ string.
    static Crs FromUserInput(std::string_view text);
    /// WKT (WKT2, WKT1 GDAL or ESRI flavour), parsed leniently as GDAL does (STRICT=NO,
    /// identifiers kept): the WKT a GeoTIFF's keys or a LAS WKT VLR give.
    static Crs FromWkt(std::string_view wkt);
    /// The WKT GDAL / libgeotiff derive from GeoTIFF keys (same lenient parsing as FromWkt).
    static Crs FromGeoTiffWkt(std::string_view wkt) { return FromWkt(wkt); }
    /// PROJJSON.
    static Crs FromProjJson(std::string_view json);

    bool Valid() const noexcept { return d_ != nullptr; }

    /// What PROJ is given to re-create this CRS: "AUTH:CODE" when the CRS has a top-level
    /// identifier and is equivalent to the authority's definition (GDAL's
    /// OGR_CT_PREFER_OFFICIAL_SRS_DEF: the database definition keeps the area of use), PROJJSON
    /// otherwise (lossless).
    const std::string& Definition() const;
    const std::string& Name() const;
    CrsType Type() const;
    /// The top-level EPSG code, when the definition carries one (not an identification guess).
    std::optional<int> Epsg() const;
    bool IsGeographic() const { return Type() == CrsType::Geographic2D || Type() == CrsType::Geographic3D; }
    bool IsProjected() const { return Type() == CrsType::Projected; }
    /// GDAL's OGRSpatialReference::IsGeographic: geographic, or a bound or compound CRS whose
    /// horizontal part is geographic.
    bool IsHorizontallyGeographic() const;
    bool IsGeocentric() const { return Type() == CrsType::Geocentric; }
    bool IsCompound() const { return Type() == CrsType::Compound; }
    /// Metres per unit of the first horizontal axis (1 for metres, 0.30480060960121924 for US
    /// survey feet); for geographic CRSs, radians per unit (0.0174532925199433 for degrees).
    double HorizontalUnitFactor() const;
    /// The area of use of the CRS (from the definition or proj.db), if any.
    std::optional<LonLatBox> AreaOfUse() const;

    std::string ToWkt(WktFlavor flavor = WktFlavor::Wkt2_2019) const;
    std::string ToProjJson() const;

    /// PROJ's EQUIVALENT criterion, ignoring the axis order of geographic CRSs (GDAL's IsSame).
    bool Equivalent(const Crs& other) const;
    /// Same definition string (cheap).
    friend bool operator==(const Crs& a, const Crs& b) { return a.d_ == b.d_ || (a.d_ && b.d_ && a.Definition() == b.Definition()); }

    struct Data;

private:
    explicit Crs(std::shared_ptr<const Data> d) : d_(std::move(d)) {}
    static Crs FromText(std::string_view text, bool wkt);
    std::shared_ptr<const Data> d_;
};

}  // namespace ddbt::crs
