// SPDX-License-Identifier: MPL-2.0
//
// The WKT of a CRS read from GeoKeys, against the text GDAL 3.13.3 reports for the same keys.

#include <gtest/gtest.h>

#include <string>

#include "cog/tiff/geotiff.h"
#include "core/crs/crs.h"

namespace ddbt::cog::tiff {
namespace {

GeoKeyDirectory Keys(std::initializer_list<std::pair<uint16_t, int>> shorts,
                     std::initializer_list<std::pair<uint16_t, double>> doubles,
                     std::initializer_list<std::pair<uint16_t, const char*>> texts) {
    GeoKeyDirectory d;
    d.version = 1;
    d.revision = 1;
    for (const auto& [id, v] : shorts) d.keys[id] = GeoKey{id, 0, {static_cast<uint16_t>(v)}, {}, {}};
    for (const auto& [id, v] : doubles) d.keys[id] = GeoKey{id, 34736, {}, {v}, {}};
    for (const auto& [id, v] : texts) d.keys[id] = GeoKey{id, 34737, {}, {}, v};
    return d;
}

// Keys that name an EPSG geographic CRS, its datum and its ellipsoid, give the ellipsoid's axes
// and inverse flattening again and a user-defined projection of an EPSG projected CRS (a DEM
// exported by a desktop GIS): GDAL keeps the identifiers of the datum and the ellipsoid (they
// choose the operations PROJ finds), writes the prime meridian without one and the degree as
// EPSG 9122.
TEST(GeoTiffCrsWkt, KeysThatRepeatAnEpsgGeographicCrsKeepItsIdentifiers) {
    using namespace geokey;
    const GeoKeyDirectory keys = Keys(
        {{kGTModelType, 1}, {kGTRasterType, 1}, {kGeographicType, 4619}, {kGeogGeodeticDatum, 6619},
         {kGeogLinearUnits, 9001}, {kGeogAngularUnits, 9102}, {kGeogEllipsoid, 7019}, {kProjectedCSType, 3008},
         {kProjection, kUserDefined}, {kProjCoordTrans, 1}, {kProjLinearUnits, 9001}},
        {{kGeogSemiMajorAxis, 6378137.0}, {kGeogSemiMinorAxis, 6356752.314140356}, {kGeogInvFlattening, 298.257222101},
         {kProjNatOriginLong, 13.5}, {kProjNatOriginLat, 0.0}, {kProjFalseEasting, 150000.0},
         {kProjFalseNorthing, 0.0}, {kProjScaleAtNatOrigin, 1.0}},
        {{kGTCitation, "SWEREF99 13 30|"}, {kGeogCitation, "SWEREF99|"}, {kPCSCitation, "SWEREF99 13 30|"}});
    const auto definition = GeoKeysToCrsDefinition(keys);
    ASSERT_TRUE(definition.ok()) << definition.status().ToString();
    const crs::Crs c = definition->rfind("EPSG:", 0) == 0 ? crs::Crs::FromUserInput(*definition)
                                                          : crs::Crs::FromWkt(*definition);
    EXPECT_EQ(c.ToWkt(crs::WktFlavor::GdalDefault),
              "PROJCS[\"ETRS89-SWE [SWEREF 99 13 30]\",GEOGCS[\"ETRS89-SWE [SWEREF 99]\",DATUM[\"SWEREF_99\","
              "SPHEROID[\"GRS 1980\",6378137,298.257222101004,AUTHORITY[\"EPSG\",\"7019\"]],AUTHORITY[\"EPSG\",\"6619\"]],"
              "PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],"
              "AUTHORITY[\"EPSG\",\"4619\"]],PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"latitude_of_origin\",0],"
              "PARAMETER[\"central_meridian\",13.5],PARAMETER[\"scale_factor\",1],PARAMETER[\"false_easting\",150000],"
              "PARAMETER[\"false_northing\",0],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Easting\",EAST],"
              "AXIS[\"Northing\",NORTH],AUTHORITY[\"EPSG\",\"3008\"]]");
}

}  // namespace
}  // namespace ddbt::cog::tiff
