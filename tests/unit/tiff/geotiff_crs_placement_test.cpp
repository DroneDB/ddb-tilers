// SPDX-License-Identifier: MPL-2.0
//
// Where the CRS read from GeoKeys puts coordinates, against GDAL 3.12 for the same keys: the keys
// of GeoTIFFs that GDAL / libgeotiff ship (NTF Paris in grads, ERDAS IMAGINE citations that name
// the linear unit, an EPSG projected CRS with another linear unit) and of rasters written by
// hand (a user-defined prime meridian under grads). Points are mapped to the geographic CRS of
// the same datum, so no datum transformation is involved; the expected values are GDAL's.

#include <gtest/gtest.h>
#include <proj.h>

#include <cmath>
#include <string>
#include <vector>

#include "cog/tiff/geotiff.h"
#include "core/crs/crs.h"
#include "core/crs/proj_context.h"
#include "core/crs/transform.h"
#include "geokeys_test_util.h"

namespace ddbt::cog::tiff {
namespace {

using namespace geokey;
using testing::Keys;

struct Point {
    double x, y;      // in the CRS of the keys
    double lon, lat;  // GDAL's, in the target CRS
};

crs::Crs CrsOf(const GeoKeyDirectory& keys) {
    const auto definition = GeoKeysToCrsDefinition(keys);
    EXPECT_TRUE(definition.ok()) << definition.status().ToString();
    if (!definition.ok()) return {};
    return definition->rfind("EPSG:", 0) == 0 ? crs::Crs::FromUserInput(*definition) : crs::Crs::FromWkt(*definition);
}

// Every point of `points`, mapped from the CRS of `keys` to EPSG:`target`, is within 1 cm of
// GDAL's (1e-7 degree).
void ExpectPlacedAsGdal(const GeoKeyDirectory& keys, int target, const std::vector<Point>& points) {
    const crs::Crs c = CrsOf(keys);
    ASSERT_TRUE(c.Valid());
    const auto t = crs::Transform::Create(c, crs::Crs::FromEpsg(target));
    for (const Point& p : points) {
        double x = p.x, y = p.y;
        ASSERT_TRUE(t->TransformPoint(x, y)) << p.x << " " << p.y;
        EXPECT_NEAR(x, p.lon, 1e-7) << "longitude of " << p.x << " " << p.y << "\n" << c.ToWkt();
        EXPECT_NEAR(y, p.lat, 1e-7) << "latitude of " << p.x << " " << p.y << "\n" << c.ToWkt();
    }
}

// The prime meridian of the CRS of `keys`, in degrees east of Greenwich.
double PrimeMeridianDegrees(const GeoKeyDirectory& keys) {
    const crs::Crs c = CrsOf(keys);
    PJ_CONTEXT* ctx = crs::detail::ThreadContext();
    crs::detail::PjPtr pj = crs::detail::CreateCrs(ctx, c.Definition());
    if (pj && proj_get_type(pj.get()) == PJ_TYPE_BOUND_CRS) pj.reset(proj_get_source_crs(ctx, pj.get()));
    crs::detail::PjPtr pm(pj ? proj_get_prime_meridian(ctx, pj.get()) : nullptr);
    double longitude = 0, toRadians = 0;
    if (!pm || !proj_prime_meridian_get_parameters(ctx, pm.get(), &longitude, &toRadians, nullptr)) return NAN;
    return longitude * toRadians * 180.0 / 3.14159265358979323846;
}

void SetDoubles(GeoKeyDirectory& keys, uint16_t id, std::vector<double> values) {
    keys.keys[id] = GeoKey{id, 34736, {}, std::move(values), {}};
}

// test_gf.tif: EPSG:27582 NTF (Paris) / France II with its geographic CRS, EPSG:4807 (grads,
// Paris meridian). GDAL builds it from the keys: the Paris meridian (2.33722917 degrees) given in
// degrees under a grad unit, which PROJ recognises; it was read as 2.33722917 * 0.9 grad.
TEST(GeoTiffCrsPlacement, NtfParisInGradsKeepsTheParisMeridian) {
    const GeoKeyDirectory keys = Keys(
        {{kGTModelType, 1}, {kGTRasterType, 1}, {kGeographicType, 4807}, {kGeogLinearUnits, 9001}, {kProjectedCSType, 27582}},
        {},
        {{kGTCitation, "Image Data"}, {kGeogCitation, "NTF (Paris)(France - onshore)"},
         {kPCSCitation, "NTF (Paris) / France II"}});
    ExpectPlacedAsGdal(keys, 4275,
                       {{827000, 2224000, 5.321528649378, 46.977140575225},
                        {827005, 2223995, 5.321591825002, 46.977093918634}});
}

// epsg_27563_allgeokeys.tif: every key of EPSG:27563 written out, the Paris meridian by code and
// as 2.5969213 grad, a TOWGS84 and a vertical unit.
TEST(GeoTiffCrsPlacement, NtfParisWithAllItsKeysKeepsTheParisMeridian) {
    GeoKeyDirectory keys = Keys(
        {{kGTModelType, 1}, {kGTRasterType, 1}, {kGeographicType, 4807}, {kGeogGeodeticDatum, 6807},
         {kGeogPrimeMeridian, 8903}, {kGeogAngularUnits, 9105}, {kGeogEllipsoid, 7011}, {kProjectedCSType, 27563},
         {kProjection, 18093}, {kProjCoordTrans, 9}, {kProjLinearUnits, 9001}, {kVerticalUnits, 9001}},
        {{kGeogAngularUnitSize, 0.01570796326794895}, {kGeogSemiMajorAxis, 6378249.2},
         {kGeogInvFlattening, 293.466021293627}, {kGeogPrimeMeridianLong, 2.5969213000000027},
         {kProjLinearUnitSize, 1.0}, {kProjNatOriginLong, 0.0}, {kProjNatOriginLat, 49.0},
         {kProjFalseEasting, 600000.0}, {kProjFalseNorthing, 200000.0}, {kProjScaleAtNatOrigin, 0.999877499}},
        {{kGTCitation, "NTF (Paris) / Lambert Sud France"}, {kGeogCitation, "NTF (Paris)"},
         {kPCSCitation, "NTF (Paris) / Lambert Sud France"}});
    SetDoubles(keys, kGeogTOWGS84, {-168, -60, 320, 0, 0, 0, 0});
    ExpectPlacedAsGdal(keys, 4275,
                       {{827294.14144377259, 523985.64443166065, 5.321528536303, 46.977140578031},
                        {827299.14781377255, 523980.63806166063, 5.321591829168, 46.977093995983}});
}

// citation_mixedcase.tif: EPSG:2838 (metres) whose IMAGINE citation says "Units =
// international_FeeT": GDAL takes the unit of the citation (international foot) and re-expresses
// the EPSG parameters in it.
TEST(GeoTiffCrsPlacement, AnImagineCitationUnitChangesTheUnitOfAnEpsgProjectedCrs) {
    const GeoKeyDirectory keys = Keys(
        {{kGTModelType, 1}, {kGTRasterType, 1}, {kGeogAngularUnits, 9102}, {kProjectedCSType, 2838}}, {},
        {{kGTCitation,
          "IMAGINE GeoTIFF Support\nCopyright 1991 - 2005 by Leica Geosystems Geospatial Imaging, LLC. All Rights "
          "Reserved\n@(#)$RCSfile: egtf.c $ IMAGINE 9.0 $Revision: 10.0 $ $Date: 2005/07/26 15:10:00 EST $\nProjection "
          "Name = NAD_1983_HARN_StatePlane_Oregon_North_FIPS_3601\nUnits = international_FeeT\nGeoTIFF Units = "
          "international_FeeT"},
         {kGeogCitation, "NAD83(HARN)"},
         {kPCSCitation,
          "IMAGINE GeoTIFF Support\nCopyright 1991 - 2005 by Leica Geosystems Geospatial Imaging, LLC. All Rights "
          "Reserved\n@(#)$RCSfile: egtf.c $ IMAGINE 9.0 $Revision: 10.0 $ $Date: 2005/07/26 15:10:00 EST $\nState Plane "
          "Zone -3601\nNAD = HARN"}});
    ExpectPlacedAsGdal(keys, 4152,
                       {{7656814, 714289.5, -122.630917075808, 45.606170828507},
                        {7656824, 714279.5, -122.630876984547, 45.606144135269}});
}

// epsg_2853_with_us_feet.tif: EPSG:2853 (metres) with ProjLinearUnitsGeoKey = US survey foot:
// the coordinates are in US survey feet, at the place the EPSG parameters give (the parameters
// were re-expressed in feet but the axes kept the metre).
TEST(GeoTiffCrsPlacement, ALinearUnitKeyChangesTheUnitOfTheAxesOfAnEpsgProjectedCrs) {
    const GeoKeyDirectory keys = Keys(
        {{kGTModelType, 1}, {kGTRasterType, 1}, {kProjectedCSType, 2853}, {kProjLinearUnits, 9003}}, {},
        {{kGTCitation,
          "IMAGINE GeoTIFF Support\nCopyright 1991 - 2005 by Leica Geosystems Geospatial Imaging, LLC. All Rights "
          "Reserved\n@(#)$RCSfile: egtf.c $ IMAGINE 9.0 $Revision: 10.0 $ $Date: 2005/07/26 15:10:00 EST $\nProjection "
          "Name = NAD_1983_HARN_StatePlane_Virginia_North_FIPS_4501_Feet\nUnits = us_survey_feet\nGeoTIFF Units = "
          "us_survey_feet"},
         {kPCSCitation,
          "IMAGINE GeoTIFF Support\nCopyright 1991 - 2005 by Leica Geosystems Geospatial Imaging, LLC. All Rights "
          "Reserved\n@(#)$RCSfile: egtf.c $ IMAGINE 9.0 $Revision: 10.0 $ $Date: 2005/07/26 15:10:00 EST $\nState Plane "
          "Zone -4501\nNAD = HARN"}});
    ExpectPlacedAsGdal(keys, 4152,
                       {{11882500, 7011250, -77.096044838297, 38.892809575888},
                        {11882505, 7011245, -77.096027541946, 38.892795638900}});
}

// ProjLinearUnitsGeoKey_9036.tif: EPSG:32064 (US survey feet) with ProjLinearUnitsGeoKey =
// kilometre and no citation unit.
TEST(GeoTiffCrsPlacement, AKilometreKeyChangesTheUnitOfAnEpsgProjectedCrsInFeet) {
    const GeoKeyDirectory keys = Keys(
        {{kGTModelType, 1}, {kGTRasterType, 1}, {kGeogAngularUnits, 9102}, {kProjectedCSType, 32064},
         {kProjLinearUnits, 9036}},
        {}, {{kGTCitation, "NAD27 / BLM 14N (ftUS)"}, {kGeogCitation, "NAD27"}});
    ExpectPlacedAsGdal(keys, 4267,
                       {{500, 4000, -99.000000011294, 36.146529212821}, {400, 3500, -100.054467890288, 31.632512350632}});
}

// A user-defined prime meridian under grads, without a datum or ellipsoid code: GDAL's one WKT1
// round trip (the grad normalisation) reads the meridian it has in degrees (2.33722917, from
// GeogPrimeMeridianLongGeoKey = 2.5969213 grad) as grads, so it ends up at 2.103506253 degrees.
TEST(GeoTiffCrsPlacement, AUserPrimeMeridianInGradsGoesThroughOneWkt1RoundTrip) {
    const GeoKeyDirectory keys = Keys(
        {{kGTModelType, 2}, {kGTRasterType, 1}, {kGeographicType, kUserDefined}, {kGeogPrimeMeridian, kUserDefined},
         {kGeogAngularUnits, 9105}},
        {{kGeogSemiMajorAxis, 6378249.2}, {kGeogInvFlattening, 293.4660212936269}, {kGeogPrimeMeridianLong, 2.5969213}},
        {});
    EXPECT_NEAR(PrimeMeridianDegrees(keys), 2.103506253, 1e-9);
    ExpectPlacedAsGdal(keys, 4326, {{2.0, 50.0, 3.903506253, 45.0}, {2.5, 49.5, 4.353506253, 44.55}});
}

// A prime meridian named by the GeogCitation under grads with a datum code (the file GDAL writes
// for a GEOGCS in grads with a TOWGS84): setting the code of the datum is a second WKT1 round
// trip in GDAL, so the meridian ends up at value * 0.9 * 0.9 (0.0330417989560251 grad).
TEST(GeoTiffCrsPlacement, ACitedPrimeMeridianInGradsWithADatumCodeGoesThroughTwoWkt1RoundTrips) {
    GeoKeyDirectory keys = Keys(
        {{kGTModelType, 2}, {kGTRasterType, 1}, {kGeographicType, kUserDefined}, {kGeogGeodeticDatum, 6807},
         {kGeogAngularUnits, 9105}},
        {{kGeogSemiMajorAxis, 6378249.2}, {kGeogInvFlattening, 293.4660212936269},
         {kGeogPrimeMeridianLong, 0.03671310995113886}},
        {{kGeogCitation, "GCS Name = NTF (Paris)|Ellipsoid = Clarke 1880 (IGN)|Primem = Paris|AUnits = grad|"}});
    SetDoubles(keys, kGeogTOWGS84, {-168, -60, 320});
    EXPECT_NEAR(PrimeMeridianDegrees(keys), 0.0330417989560251 * 0.9, 1e-12);
}

}  // namespace
}  // namespace ddbt::cog::tiff
