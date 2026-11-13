// SPDX-License-Identifier: MPL-2.0
//
// GeoKeys -> CRS definition, following libgeotiff's GTIFGetDefn (geo_normalize.c: key
// defaults, EPSG lookups, GTIFFetchProjParms) and GDAL's GTIFGetOGISDefnAsOSR (gt_wkt_srs.cpp:
// LOCAL_CS, ESRI PE strings, citations, EPSG import when only ProjectedCSTypeGeoKey is set,
// TOWGS84, the projection switch, vertical CRSs). EPSG lookups go to PROJ's database.
//
// Output: "EPSG:code" (or "EPSG:h+v") when the keys are an EPSG code GDAL would import as is,
// otherwise WKT (WKT1 GDAL flavour for key-built definitions, as OGR builds them; WKT2 when
// PROJ objects had to be combined).

#include <proj.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "cog/tiff/geotiff.h"
#include "core/crs/proj_context.h"

namespace ddbt::cog::tiff {

namespace {

constexpr int kUser = geokey::kUserDefined;
constexpr double kDegToRad = 0.0174532925199433;  // SRS_UA_DEGREE_CONV
constexpr double kPi = 3.14159265358979323846;

// libgeotiff projection method codes (CT_*).
enum Ct : int {
    kCtTransverseMercator = 1,
    kCtObliqueMercator = 3,
    kCtObliqueMercatorLaborde = 4,
    kCtMercator = 7,
    kCtLcc2SP = 8,
    kCtLcc1SP = 9,
    kCtLaea = 10,
    kCtAlbers = 11,
    kCtAzimuthalEquidistant = 12,
    kCtEquidistantConic = 13,
    kCtStereographic = 14,
    kCtPolarStereographic = 15,
    kCtObliqueStereographic = 16,
    kCtEquirectangular = 17,
    kCtCassini = 18,
    kCtGnomonic = 19,
    kCtMiller = 20,
    kCtOrthographic = 21,
    kCtPolyconic = 22,
    kCtRobinson = 23,
    kCtSinusoidal = 24,
    kCtVanDerGrinten = 25,
    kCtNzmg = 26,
    kCtTmSouthOriented = 27,
    kCtCea = 28,
    kCtHomAzimuthCenter = 9815,
};

using crs::detail::PjPtr;

// PROJ database lookups (EPSG), on the calling thread's context.
class Db {
public:
    Db() : ctx_(crs::detail::ThreadContext()) {}
    PJ_CONTEXT* ctx() const { return ctx_; }

    PjPtr Create(int code, PJ_CATEGORY cat) const {
        if (code <= 0 || code >= kUser) return nullptr;
        const std::string c = std::to_string(code);
        return PjPtr(proj_create_from_database(ctx_, "EPSG", c.c_str(), cat, 0, nullptr));
    }
    static int IdCode(const PJ* p) {
        if (p == nullptr) return kUser;
        const char* auth = proj_get_id_auth_name(p, 0);
        const char* code = proj_get_id_code(p, 0);
        if (auth == nullptr || code == nullptr || std::string(auth) != "EPSG") return kUser;
        return std::atoi(code);
    }
    static std::string Name(const PJ* p) {
        const char* n = p != nullptr ? proj_get_name(p) : nullptr;
        return n != nullptr ? n : "";
    }

    // GTIFGetPCSInfoEx: name, conversion code, linear unit code, base GCS code.
    bool PcsInfo(int code, std::string* name, int* projCode, int* uom, int* gcs) const {
        PjPtr crs = Create(code, PJ_CATEGORY_CRS);
        if (!crs || proj_get_type(crs.get()) != PJ_TYPE_PROJECTED_CRS) return false;
        if (name) *name = Name(crs.get());
        PjPtr base(proj_crs_get_geodetic_crs(ctx_, crs.get()));
        if (gcs) *gcs = IdCode(base.get());
        PjPtr conv(proj_crs_get_coordoperation(ctx_, crs.get()));
        if (projCode) *projCode = IdCode(conv.get());
        PjPtr cs(proj_crs_get_coordinate_system(ctx_, crs.get()));
        if (uom && cs) {
            const char* unitCode = nullptr;
            if (proj_cs_get_axis_info(ctx_, cs.get(), 0, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, &unitCode) &&
                unitCode != nullptr) {
                *uom = std::atoi(unitCode);
            }
        }
        return true;
    }
    // GTIFGetGCSInfoEx: name, datum code, prime meridian code, angular unit code.
    bool GcsInfo(int code, std::string* name, int* datum, int* pm, int* uomAngle) const {
        PjPtr crs = Create(code, PJ_CATEGORY_CRS);
        if (!crs) return false;
        const PJ_TYPE t = proj_get_type(crs.get());
        if (t != PJ_TYPE_GEOGRAPHIC_2D_CRS && t != PJ_TYPE_GEOGRAPHIC_3D_CRS) return false;
        if (name) *name = Name(crs.get());
        if (datum) {
            PjPtr d(proj_crs_get_datum(ctx_, crs.get()));
            if (!d) d.reset(proj_crs_get_datum_ensemble(ctx_, crs.get()));
            *datum = IdCode(d.get());
        }
        if (pm) {
            PjPtr p(proj_get_prime_meridian(ctx_, crs.get()));
            *pm = IdCode(p.get());
        }
        if (uomAngle) {
            PjPtr cs(proj_crs_get_coordinate_system(ctx_, crs.get()));
            const char* unitCode = nullptr;
            if (cs && proj_cs_get_axis_info(ctx_, cs.get(), 0, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, &unitCode) &&
                unitCode != nullptr) {
                *uomAngle = std::atoi(unitCode);
            }
        }
        return true;
    }
    bool DatumInfo(int code, std::string* name, int* ellipsoid) const {
        PjPtr d = Create(code, PJ_CATEGORY_DATUM);
        if (!d) d = Create(code, PJ_CATEGORY_DATUM_ENSEMBLE);
        if (!d) return false;
        if (name) *name = Name(d.get());
        if (ellipsoid) {
            PjPtr e(proj_get_ellipsoid(ctx_, d.get()));
            if (!e && proj_get_type(d.get()) == PJ_TYPE_DATUM_ENSEMBLE) {
                PjPtr first(proj_datum_ensemble_get_member(ctx_, d.get(), 0));
                if (first) e.reset(proj_get_ellipsoid(ctx_, first.get()));
            }
            *ellipsoid = IdCode(e.get());
        }
        return true;
    }
    bool EllipsoidInfo(int code, std::string* name, double* a, double* b) const {
        PjPtr e = Create(code, PJ_CATEGORY_ELLIPSOID);
        if (!e) return false;
        if (name) *name = Name(e.get());
        double sa = 0, sb = 0, invf = 0;
        int computed = 0;
        if (!proj_ellipsoid_get_parameters(ctx_, e.get(), &sa, &sb, &computed, &invf)) return false;
        if (a) *a = sa;
        if (b) *b = sb;
        return true;
    }
    bool PmInfo(int code, std::string* name, double* degrees) const {
        PjPtr p = Create(code, PJ_CATEGORY_PRIME_MERIDIAN);
        if (!p) return false;
        if (name) *name = Name(p.get());
        double lon = 0, conv = 0;
        if (!proj_prime_meridian_get_parameters(ctx_, p.get(), &lon, &conv, nullptr)) return false;
        if (degrees) *degrees = lon * conv * 180.0 / kPi;
        return true;
    }
    bool Uom(int code, std::string* name, double* factor, std::string* category) const {
        if (code <= 0 || code >= kUser) return false;
        const std::string c = std::to_string(code);
        const char* n = nullptr;
        const char* cat = nullptr;
        double f = 0;
        if (!proj_uom_get_info_from_database(ctx_, "EPSG", c.c_str(), &n, &f, &cat)) return false;
        if (name && n) *name = n;
        if (factor) *factor = f;
        if (category && cat) *category = cat;
        return true;
    }

private:
    PJ_CONTEXT* ctx_;
};

// libgeotiff's GTIFDefn.
struct Defn {
    int model = kUser, pcs = kUser, gcs = kUser;
    int uomLength = kUser;
    double uomLengthInMeters = 1.0;
    int uomAngle = kUser;
    double uomAngleInDegrees = 1.0;
    int datum = kUser, ellipsoid = kUser;
    double semiMajor = 0, semiMinor = 0;
    int pm = kUser;
    double pmLongToGreenwich = 0;
    std::vector<double> towgs84;
    int projCode = kUser, ctProjection = kUser;
    double parm[10] = {};
    int parmId[10] = {};
    int nParms = 0;
};

double AngleToDD(double angle, int uomAngle, double uomAngleInDegrees) {
    if (uomAngle == 9110) {
        // DDD.MMSSsss
        const double sign = angle < 0 ? -1.0 : 1.0;
        double a = std::fabs(angle);
        const double deg = std::floor(a);
        a = (a - deg) * 100.0;
        const double min = std::floor(a + 1e-9);
        const double sec = (a - min) * 100.0;
        return sign * (deg + min / 60.0 + sec / 3600.0);
    }
    if (uomAngle != kUser && uomAngleInDegrees != 0) return angle * uomAngleInDegrees;
    return angle;
}

// GTIFFetchProjParms.
void FetchProjParms(const GeoKeyDirectory& k, Defn& d) {
    using namespace geokey;
    auto get = [&](std::initializer_list<uint16_t> ids, double fallback, bool* found = nullptr) {
        for (uint16_t id : ids) {
            if (auto v = k.Double(id)) {
                if (found) *found = true;
                return *v;
            }
        }
        if (found) *found = false;
        return fallback;
    };
    const double fe = get({kProjFalseEasting, kProjCenterEasting, kProjFalseOriginEasting}, 0.0);
    const double fn = get({kProjFalseNorthing, kProjCenterNorthing, kProjFalseOriginNorthing}, 0.0);
    const double natLong = get({kProjNatOriginLong, kProjFalseOriginLong, kProjCenterLong}, 0.0);
    const double natLat = get({kProjNatOriginLat, kProjFalseOriginLat, kProjCenterLat}, 0.0);
    auto set = [&](int i, double v, int id) {
        d.parm[i] = v;
        d.parmId[i] = id;
    };
    for (int& id : d.parmId) id = 0;
    for (double& p : d.parm) p = 0;
    d.nParms = 7;
    set(5, fe, kProjFalseEasting);
    set(6, fn, kProjFalseNorthing);
    switch (d.ctProjection) {
        case kCtStereographic:
            set(0, natLat, kProjCenterLat);
            set(1, natLong, kProjCenterLong);
            set(4, get({kProjScaleAtNatOrigin}, 1.0), kProjScaleAtNatOrigin);
            break;
        case kCtMercator: {
            set(0, natLat, kProjNatOriginLat);
            set(1, natLong, kProjNatOriginLong);
            bool haveSp1 = false, haveNos = false;
            const double sp1 = get({kProjStdParallel1}, 0.0, &haveSp1);
            double nos = get({kProjScaleAtNatOrigin}, 1.0, &haveNos);
            if (!haveNos && !haveSp1) {
                haveNos = true;
                nos = 1.0;
            }
            if (haveSp1) set(2, sp1, kProjStdParallel1);
            if (haveNos) set(4, nos, kProjScaleAtNatOrigin);
            break;
        }
        case kCtLcc1SP:
        case kCtObliqueStereographic:
        case kCtTransverseMercator:
        case kCtTmSouthOriented:
            set(0, natLat, kProjNatOriginLat);
            set(1, natLong, kProjNatOriginLong);
            set(4, get({kProjScaleAtNatOrigin, kProjScaleAtCenter}, 1.0), kProjScaleAtNatOrigin);
            break;
        case kCtObliqueMercator:
        case kCtHomAzimuthCenter:
            set(0, natLat, kProjCenterLat);
            set(1, natLong, kProjCenterLong);
            set(2, get({kProjAzimuthAngle}, 0.0), kProjAzimuthAngle);
            set(3, get({kProjRectifiedGridAngle}, 90.0), kProjRectifiedGridAngle);
            set(4, get({kProjScaleAtNatOrigin, kProjScaleAtCenter}, 1.0), kProjScaleAtCenter);
            break;
        case kCtObliqueMercatorLaborde:
            set(0, natLat, kProjCenterLat);
            set(1, natLong, kProjCenterLong);
            set(2, get({kProjAzimuthAngle}, 0.0), kProjAzimuthAngle);
            set(4, get({kProjScaleAtNatOrigin, kProjScaleAtCenter}, 1.0), kProjScaleAtCenter);
            break;
        case kCtCassini:
        case kCtPolyconic:
            set(0, natLat, kProjNatOriginLat);
            set(1, natLong, kProjNatOriginLong);
            set(4, get({kProjScaleAtNatOrigin, kProjScaleAtCenter}, 1.0), kProjScaleAtNatOrigin);
            break;
        case kCtAzimuthalEquidistant:
        case kCtMiller:
        case kCtGnomonic:
        case kCtLaea:
        case kCtOrthographic:
        case kCtNzmg:
            set(0, natLat, kProjCenterLat);
            set(1, natLong, kProjCenterLong);
            break;
        case kCtEquirectangular:
            set(0, natLat, kProjCenterLat);
            set(1, natLong, kProjCenterLong);
            set(2, get({kProjStdParallel1}, 0.0), kProjStdParallel1);
            break;
        case kCtRobinson:
        case kCtSinusoidal:
        case kCtVanDerGrinten:
            set(1, natLong, kProjCenterLong);
            break;
        case kCtPolarStereographic:
            set(0, natLat, kProjNatOriginLat);
            set(1, get({kProjStraightVertPoleLong, kProjNatOriginLong, kProjFalseOriginLong, kProjCenterLong}, 0.0),
                kProjStraightVertPoleLong);
            set(4, get({kProjScaleAtNatOrigin, kProjScaleAtCenter}, 1.0), kProjScaleAtNatOrigin);
            break;
        case kCtLcc2SP: {
            bool have2 = false;
            double sp1 = get({kProjStdParallel1}, 0.0);
            const double sp2 = get({kProjStdParallel2}, 0.0, &have2);
            if (!have2) sp1 = 0.0;  // libgeotiff quirk: a missing StdParallel2 zeroes StdParallel1
            set(0, natLat, kProjFalseOriginLat);
            set(1, natLong, kProjFalseOriginLong);
            set(2, sp1, kProjStdParallel1);
            set(3, sp2, kProjStdParallel2);
            break;
        }
        case kCtAlbers:
        case kCtEquidistantConic:
            set(0, get({kProjStdParallel1}, 0.0), kProjStdParallel1);
            set(1, get({kProjStdParallel2}, 0.0), kProjStdParallel2);
            set(2, natLat, kProjNatOriginLat);
            set(3, natLong, kProjNatOriginLong);
            break;
        case kCtCea:
            set(0, get({kProjStdParallel1}, 0.0), kProjStdParallel1);
            set(1, natLong, kProjNatOriginLong);
            break;
        default: d.nParms = 0; return;
    }
    // Linear parameters to metres, angular ones to degrees (libgeotiff >= 1.7.4).
    for (int i = 0; i < d.nParms; ++i) {
        switch (d.parmId[i]) {
            case kProjFalseEasting:
            case kProjFalseNorthing:
            case kProjFalseOriginEasting:
            case kProjFalseOriginNorthing:
            case kProjCenterEasting:
            case kProjCenterNorthing:
                if (d.uomLengthInMeters != 0 && d.uomLengthInMeters != 1.0) d.parm[i] *= d.uomLengthInMeters;
                break;
            case kProjStdParallel1:
            case kProjStdParallel2:
            case kProjNatOriginLong:
            case kProjNatOriginLat:
            case kProjFalseOriginLong:
            case kProjFalseOriginLat:
            case kProjCenterLong:
            case kProjCenterLat:
            case kProjStraightVertPoleLong:
            case kProjRectifiedGridAngle:
                if (d.uomAngleInDegrees != 0 && d.uomAngleInDegrees != 1.0) d.parm[i] *= d.uomAngleInDegrees;
                break;
            default: break;
        }
    }
}

// GTIFGetDefn.
Defn GetDefn(const GeoKeyDirectory& k, const Db& db) {
    using namespace geokey;
    Defn d;
    if (auto v = k.Short(kGTModelType)) d.model = *v;
    if (auto v = k.Short(kGeogLinearUnits)) d.uomLength = *v;
    if (auto v = k.Short(kProjectedCSType); v && *v != kUser) {
        d.pcs = *v;
        int projCode = kUser, uom = d.uomLength, gcs = kUser;
        if (db.PcsInfo(d.pcs, nullptr, &projCode, &uom, &gcs)) {
            d.projCode = projCode;
            d.uomLength = uom;
            d.gcs = gcs;
        }
    } else if (v) {
        d.pcs = kUser;
    }
    if (d.projCode == kUser) {
        if (auto v = k.Short(kProjection)) d.projCode = *v;
    }
    if (auto v = k.Short(kGeographicType)) d.gcs = *v;
    if (d.gcs < 1 || d.gcs >= kUser) d.gcs = kUser;
    if (d.gcs != kUser) db.GcsInfo(d.gcs, nullptr, &d.datum, &d.pm, &d.uomAngle);
    if (auto v = k.Short(kGeogAngularUnits)) d.uomAngle = *v;
    if (d.uomAngle != kUser) {
        double f = 0;
        std::string cat;
        if (d.uomAngle == 9110) d.uomAngleInDegrees = 1.0;
        else if (db.Uom(d.uomAngle, nullptr, &f, &cat) && f > 0) d.uomAngleInDegrees = f * 180.0 / kPi;
    }
    if (auto v = k.Short(kGeogGeodeticDatum)) d.datum = *v;
    if (d.datum != kUser) db.DatumInfo(d.datum, nullptr, &d.ellipsoid);
    if (auto v = k.Short(kGeogEllipsoid)) d.ellipsoid = *v;
    if (d.ellipsoid != kUser) db.EllipsoidInfo(d.ellipsoid, nullptr, &d.semiMajor, &d.semiMinor);
    if (auto v = k.Double(kGeogSemiMajorAxis)) d.semiMajor = *v;
    if (auto v = k.Double(kGeogSemiMinorAxis)) d.semiMinor = *v;
    if (auto v = k.Double(kGeogInvFlattening)) d.semiMinor = *v != 0.0 ? d.semiMajor * (1 - 1.0 / *v) : d.semiMajor;
    if (auto v = k.Short(kGeogPrimeMeridian)) d.pm = *v;
    if (d.pm != kUser) {
        db.PmInfo(d.pm, nullptr, &d.pmLongToGreenwich);
    } else if (auto v = k.Double(kGeogPrimeMeridianLong)) {
        d.pmLongToGreenwich = AngleToDD(*v, d.uomAngle, d.uomAngleInDegrees);
    }
    d.towgs84 = k.Doubles(kGeogTOWGS84);
    if (d.towgs84.size() > 7) d.towgs84.resize(7);
    if (auto v = k.Short(kProjLinearUnits)) d.uomLength = *v;
    if (d.uomLength != kUser) {
        double f = 0;
        if (db.Uom(d.uomLength, nullptr, &f, nullptr) && f > 0) d.uomLengthInMeters = f;
    } else if (auto v = k.Double(kProjLinearUnitSize)) {
        d.uomLengthInMeters = *v;
    }
    if (auto v = k.Short(kProjCoordTrans)) {
        d.ctProjection = *v;
        FetchProjParms(k, d);
    }
    return d;
}

// Citation strings ("GCS Name = ...|Datum = ...|").
struct Citation {
    std::string pcs, projection, lunits, gcs, datum, ellipsoid, primem, aunits;
};

Citation ParseCitation(const std::string& s, bool geogKey) {
    Citation c;
    bool found = false;
    size_t pos = 0;
    while (pos < s.size()) {
        size_t bar = s.find('|', pos);
        if (bar == std::string::npos) bar = s.size();
        const std::string part = s.substr(pos, bar - pos);
        pos = bar + 1;
        auto take = [&](const char* key, std::string& dst) {
            const size_t at = part.find(key);
            if (at != std::string::npos && dst.empty()) {
                dst = part.substr(at + std::char_traits<char>::length(key));
                found = true;
            }
        };
        take("PCS Name = ", c.pcs);
        take("PRJ Name = ", c.projection);
        take("LUnits = ", c.lunits);
        take("GCS Name = ", c.gcs);
        take("Datum = ", c.datum);
        take("Ellipsoid = ", c.ellipsoid);
        take("Primem = ", c.primem);
        take("AUnits = ", c.aunits);
    }
    if (!found && geogKey) c.gcs = s;
    return c;
}

std::string Num(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.17g", v);
    // Shortest representation that round-trips.
    for (int p = 15; p <= 17; ++p) {
        char t[64];
        std::snprintf(t, sizeof(t), "%.*g", p, v);
        if (std::strtod(t, nullptr) == v) return t;
    }
    return buf;
}

std::string Quote(const std::string& s) {
    std::string out = "\"";
    for (char ch : s) out += ch == '"' ? '\'' : ch;
    return out + "\"";
}

std::string Authority(int code) { return code > 0 && code < kUser ? ",AUTHORITY[\"EPSG\",\"" + std::to_string(code) + "\"]" : ""; }

std::string Param(const char* name, double v) { return ",PARAMETER[\"" + std::string(name) + "\"," + Num(v) + "]"; }

// The PROJECTION and PARAMETER nodes of GDAL's projection switch. Angles in degrees on input,
// written in `angUnitDeg` units; lengths in metres, written in `linUnitM` units.
bool ProjectionWkt(const Defn& d, double angUnitDeg, double linUnitM, std::string* out) {
    const double* p = d.parm;
    auto A = [&](double deg) { return deg / angUnitDeg; };
    auto L = [&](double m) { return m / linUnitM; };
    std::string s;
    auto proj = [&](const char* n) { s = ",PROJECTION[\"" + std::string(n) + "\"]"; };
    switch (d.ctProjection) {
        case kCtTransverseMercator:
        case kCtTmSouthOriented:
            proj(d.ctProjection == kCtTransverseMercator ? "Transverse_Mercator" : "Transverse_Mercator_South_Orientated");
            s += Param("latitude_of_origin", A(p[0])) + Param("central_meridian", A(p[1])) + Param("scale_factor", p[4]) +
                 Param("false_easting", L(p[5])) + Param("false_northing", L(p[6]));
            break;
        case kCtMercator:
            if (d.parmId[2] == geokey::kProjStdParallel1) {
                proj("Mercator_2SP");
                s += Param("standard_parallel_1", A(p[2])) + Param("latitude_of_origin", A(p[0])) +
                     Param("central_meridian", A(p[1])) + Param("false_easting", L(p[5])) + Param("false_northing", L(p[6]));
            } else {
                proj("Mercator_1SP");
                s += Param("latitude_of_origin", A(p[0])) + Param("central_meridian", A(p[1])) + Param("scale_factor", p[4]) +
                     Param("false_easting", L(p[5])) + Param("false_northing", L(p[6]));
            }
            break;
        case kCtObliqueStereographic:
        case kCtStereographic:
        case kCtPolarStereographic:
            proj(d.ctProjection == kCtObliqueStereographic ? "Oblique_Stereographic"
                 : d.ctProjection == kCtStereographic      ? "Stereographic"
                                                           : "Polar_Stereographic");
            s += Param("latitude_of_origin", A(p[0])) + Param("central_meridian", A(p[1])) + Param("scale_factor", p[4]) +
                 Param("false_easting", L(p[5])) + Param("false_northing", L(p[6]));
            break;
        case kCtObliqueMercator:
        case kCtHomAzimuthCenter:
            proj(d.ctProjection == kCtObliqueMercator ? "Hotine_Oblique_Mercator" : "Hotine_Oblique_Mercator_Azimuth_Center");
            s += Param("latitude_of_center", A(p[0])) + Param("longitude_of_center", A(p[1])) + Param("azimuth", A(p[2])) +
                 Param("rectified_grid_angle", A(p[3])) + Param("scale_factor", p[4]) + Param("false_easting", L(p[5])) +
                 Param("false_northing", L(p[6]));
            break;
        case kCtObliqueMercatorLaborde:
            proj("Laborde_Oblique_Mercator");
            s += Param("latitude_of_center", A(p[0])) + Param("longitude_of_center", A(p[1])) + Param("azimuth", A(p[2])) +
                 Param("scale_factor", p[4]) + Param("false_easting", L(p[5])) + Param("false_northing", L(p[6]));
            break;
        case kCtEquidistantConic:
            proj("Equidistant_Conic");
            s += Param("standard_parallel_1", A(p[0])) + Param("standard_parallel_2", A(p[1])) +
                 Param("latitude_of_center", A(p[2])) + Param("longitude_of_center", A(p[3])) +
                 Param("false_easting", L(p[5])) + Param("false_northing", L(p[6]));
            break;
        case kCtAlbers:
            proj("Albers_Conic_Equal_Area");
            s += Param("standard_parallel_1", A(p[0])) + Param("standard_parallel_2", A(p[1])) +
                 Param("latitude_of_center", A(p[2])) + Param("longitude_of_center", A(p[3])) +
                 Param("false_easting", L(p[5])) + Param("false_northing", L(p[6]));
            break;
        case kCtLcc2SP:
            proj("Lambert_Conformal_Conic_2SP");
            s += Param("standard_parallel_1", A(p[2])) + Param("standard_parallel_2", A(p[3])) +
                 Param("latitude_of_origin", A(p[0])) + Param("central_meridian", A(p[1])) +
                 Param("false_easting", L(p[5])) + Param("false_northing", L(p[6]));
            break;
        case kCtLcc1SP:
            proj("Lambert_Conformal_Conic_1SP");
            s += Param("latitude_of_origin", A(p[0])) + Param("central_meridian", A(p[1])) + Param("scale_factor", p[4]) +
                 Param("false_easting", L(p[5])) + Param("false_northing", L(p[6]));
            break;
        case kCtCassini:
        case kCtPolyconic:
        case kCtGnomonic:
        case kCtOrthographic:
        case kCtNzmg:
            proj(d.ctProjection == kCtCassini      ? "Cassini_Soldner"
                 : d.ctProjection == kCtPolyconic  ? "Polyconic"
                 : d.ctProjection == kCtGnomonic   ? "Gnomonic"
                 : d.ctProjection == kCtOrthographic ? "Orthographic"
                                                     : "New_Zealand_Map_Grid");
            s += Param("latitude_of_origin", A(p[0])) + Param("central_meridian", A(p[1])) +
                 Param("false_easting", L(p[5])) + Param("false_northing", L(p[6]));
            break;
        case kCtAzimuthalEquidistant:
        case kCtMiller:
        case kCtLaea:
            proj(d.ctProjection == kCtAzimuthalEquidistant ? "Azimuthal_Equidistant"
                 : d.ctProjection == kCtMiller             ? "Miller_Cylindrical"
                                                           : "Lambert_Azimuthal_Equal_Area");
            s += Param("latitude_of_center", A(p[0])) + Param("longitude_of_center", A(p[1])) +
                 Param("false_easting", L(p[5])) + Param("false_northing", L(p[6]));
            break;
        case kCtEquirectangular:
            proj("Equirectangular");
            s += Param("latitude_of_origin", A(p[0])) + Param("central_meridian", A(p[1])) +
                 Param("standard_parallel_1", A(p[2])) + Param("false_easting", L(p[5])) +
                 Param("false_northing", L(p[6]));
            break;
        case kCtRobinson:
        case kCtSinusoidal:
            proj(d.ctProjection == kCtRobinson ? "Robinson" : "Sinusoidal");
            s += Param("longitude_of_center", A(p[1])) + Param("false_easting", L(p[5])) + Param("false_northing", L(p[6]));
            break;
        case kCtVanDerGrinten:
            proj("VanDerGrinten");
            s += Param("central_meridian", A(p[1])) + Param("false_easting", L(p[5])) + Param("false_northing", L(p[6]));
            break;
        case kCtCea:
            proj("Cylindrical_Equal_Area");
            s += Param("standard_parallel_1", A(p[0])) + Param("central_meridian", A(p[1])) +
                 Param("false_easting", L(p[5])) + Param("false_northing", L(p[6]));
            break;
        default: return false;
    }
    *out = s;
    return true;
}

bool ExistsInDb(const Db& db, int code, PJ_CATEGORY cat) { return static_cast<bool>(db.Create(code, cat)); }

std::string WktOf(const Db& db, PJ* obj, PJ_WKT_TYPE type) {
    const char* opts[] = {"MULTILINE=NO", nullptr};
    const char* w = proj_as_wkt(db.ctx(), obj, type, opts);
    return w != nullptr ? w : "";
}

}  // namespace

Expected<std::string> GeoKeysToCrsDefinition(const GeoKeyDirectory& keys) {
    using namespace geokey;
    if (keys.empty()) return std::string();
    Db db;
    Defn d = GetDefn(keys, db);
    if (d.model == kUser && d.pcs != kUser) d.model = 1;

    // Vertical keys, with GDAL's fixes for GeoTIFF 1.0 files.
    int vcs = keys.Short(kVerticalCSType).value_or(0);
    int vdatum = keys.Short(kVerticalDatum).value_or(0);
    int vunits = keys.Short(kVerticalUnits).value_or(0);
    if (vcs != 0 || vdatum != 0 || vunits != 0) {
        if (keys.version == 1 && keys.revision == 1 && keys.minorRevision == 0) {
            if (vcs >= 5101 && vcs <= 5112 && vdatum == 0) {
                vdatum = vcs;
                vcs = vdatum + 600;
            }
            if (vcs == 5171) {
                vdatum = 5171;
                vcs = 5773;
            }
        }
        if (vcs >= 5001 && vcs <= 5033 && vdatum == 0) {
            vdatum = vcs + 1000;
            vcs = 0;
        }
    }
    if (vcs > kUser || vdatum > kUser || vunits > kUser) vcs = vdatum = vunits = 0;

    const std::string gtCitation = keys.Ascii(kGTCitation).value_or("");
    const std::string pcsCitation = keys.Ascii(kPCSCitation).value_or("");
    const std::string geogCitation = keys.Ascii(kGeogCitation).value_or("");
    static constexpr std::string_view kPe = "ESRI PE String = ";

    // ---- Not projected / geographic / geocentric: LOCAL_CS (or an ESRI PE string) ----------
    if (d.model != 1 && d.model != 2 && d.model != 3) {
        if (const size_t at = pcsCitation.find(kPe); at != std::string::npos) return pcsCitation.substr(at + kPe.size());
        std::string name = !gtCitation.empty() ? gtCitation : !geogCitation.empty() ? geogCitation : "unnamed";
        std::string unitName = "unknown";
        if (d.uomLength != kUser) db.Uom(d.uomLength, &unitName, nullptr, nullptr);
        return "LOCAL_CS[" + Quote(name) + ",UNIT[" + Quote(unitName) + "," + Num(d.uomLengthInMeters) +
               Authority(d.uomLength) + "]]";
    }

    std::string horizontal;  // definition of the horizontal CRS
    bool horizontalIsEpsg = false;
    int horizontalCode = 0;

    // ---- Projected CRS that GDAL imports from EPSG ------------------------------------------
    const bool onlyPcs = d.model == 1 && d.pcs != kUser && !keys.Has(kProjection) && !keys.Has(kProjCoordTrans) &&
                         !keys.Has(kGeographicType) && !keys.Has(kGeogGeodeticDatum) && !keys.Has(kGeogEllipsoid);
    if (onlyPcs) {
        PjPtr pcs = db.Create(d.pcs, PJ_CATEGORY_CRS);
        if (pcs && proj_get_type(pcs.get()) == PJ_TYPE_PROJECTED_CRS) {
            // A ProjLinearUnitsGeoKey different from the EPSG unit changes the units (#6210).
            int epsgUom = kUser;
            db.PcsInfo(d.pcs, nullptr, nullptr, &epsgUom, nullptr);
            double epsgFactor = 1.0;
            db.Uom(epsgUom, nullptr, &epsgFactor, nullptr);
            if (std::fabs(d.uomLengthInMeters - epsgFactor) > 1e-15 * d.uomLengthInMeters) {
                std::string unitName = "unknown";
                if (d.uomLength != kUser) db.Uom(d.uomLength, &unitName, nullptr, nullptr);
                const std::string code = std::to_string(d.uomLength);
                PjPtr altered(proj_crs_alter_parameters_linear_unit(
                    db.ctx(), pcs.get(), unitName.c_str(), d.uomLengthInMeters, d.uomLength != kUser ? "EPSG" : nullptr,
                    d.uomLength != kUser ? code.c_str() : nullptr, 1));
                if (!altered) return Corrupt("cannot change the linear unit of EPSG:" + std::to_string(d.pcs));
                horizontal = WktOf(db, altered.get(), PJ_WKT2_2019);
            } else {
                horizontal = "EPSG:" + std::to_string(d.pcs);
                horizontalIsEpsg = true;
                horizontalCode = d.pcs;
            }
        }
    }

    // ---- Geographic CRS (part of everything else) ------------------------------------------
    std::string geogWkt;
    bool geogIsPlainEpsg = false;
    if (horizontal.empty()) {
        std::string geogName, datumName, pmName, spheroidName, angularUnits;
        bool namesFromCitation = false;
        std::string tmp;
        if (!(d.gcs != kUser && db.GcsInfo(d.gcs, &geogName, nullptr, nullptr, nullptr)) && !geogCitation.empty()) {
            const Citation c = ParseCitation(geogCitation, true);
            geogName = c.gcs;
            datumName = c.datum;
            pmName = c.primem;
            spheroidName = c.ellipsoid;
            angularUnits = c.aunits;
            namesFromCitation = true;
        }
        if (datumName.empty() && d.datum != kUser) db.DatumInfo(d.datum, &datumName, nullptr);
        double semiMajor = d.semiMajor;
        double invFlattening = 0.0;
        if (spheroidName.empty()) {
            if (d.ellipsoid != kUser) db.EllipsoidInfo(d.ellipsoid, &spheroidName, nullptr, nullptr);
        } else {
            if (auto v = keys.Double(kGeogSemiMajorAxis)) semiMajor = *v;
            if (auto v = keys.Double(kGeogInvFlattening)) invFlattening = std::isinf(*v) ? 0.0 : *v;
        }
        double pmLong = d.pmLongToGreenwich;
        if (pmName.empty()) {
            if (d.pm != kUser) db.PmInfo(d.pm, &pmName, nullptr);
        } else if (auto v = keys.Double(kGeogPrimeMeridianLong)) {
            pmLong = *v;
        }
        double angInDeg = d.uomAngleInDegrees;
        if (angularUnits.empty()) {
            if (d.uomAngle == 9110) angularUnits = "degree";
            else if (!(d.uomAngle != kUser && db.Uom(d.uomAngle, &angularUnits, nullptr, nullptr))) angularUnits = "unknown";
        } else if (auto v = keys.Double(kGeogAngularUnitSize)) {
            angInDeg = *v / kDegToRad;
        }
        if (angInDeg == 0) angInDeg = 1;
        if (semiMajor == 0.0) {
            spheroidName = "unretrievable - using WGS84";
            semiMajor = 6378137.0;
            invFlattening = 298.257223563;
        } else if (invFlattening == 0.0 && d.semiMajor != 0 &&
                   ((d.semiMinor / d.semiMajor) < 0.99999999999999999 || (d.semiMinor / d.semiMajor) > 1.00000000000000001)) {
            invFlattening = d.semiMajor / (d.semiMajor - d.semiMinor);
            if (std::fabs(semiMajor - 6378137.0) < 1e-10 && std::fabs(invFlattening - 298.257223563) < 1e-10) {
                invFlattening = 298.257223563;
            }
        }
        if (geogName.empty()) geogName = !datumName.empty() ? datumName : "unknown";
        (void)namesFromCitation;

        // GDAL: a valid GCS code is attached as the GEOGCS authority; datum and ellipsoid codes
        // likewise. A known (EPSG) datum loses its TOWGS84 (OSR_STRIP_TOWGS84).
        const bool gcsValid = d.gcs != kUser && d.model != 3 && ExistsInDb(db, d.gcs, PJ_CATEGORY_CRS);
        const bool keysOverride = keys.Has(kGeogGeodeticDatum) || keys.Has(kGeogEllipsoid) || keys.Has(kGeogSemiMajorAxis) ||
                                  keys.Has(kGeogSemiMinorAxis) || keys.Has(kGeogInvFlattening) ||
                                  keys.Has(kGeogPrimeMeridian) || keys.Has(kGeogPrimeMeridianLong) ||
                                  (keys.Has(kGeogAngularUnits) && d.uomAngle != 9102 && d.uomAngle != 9122);
        std::vector<double> towgs = d.towgs84;
        const bool datumKnown = d.datum != kUser && db.DatumInfo(d.datum, nullptr, nullptr);
        if (datumKnown) towgs.clear();
        if (towgs.size() == 3) towgs.resize(7, 0.0);
        if (towgs.size() != 7) towgs.clear();
        if (gcsValid && !keysOverride && towgs.empty()) geogIsPlainEpsg = true;

        std::string w = "GEOGCS[" + Quote(geogName) + ",DATUM[" + Quote(datumName.empty() ? "unknown" : datumName) +
                        ",SPHEROID[" + Quote(spheroidName.empty() ? "unnamed" : spheroidName) + "," + Num(semiMajor) + "," +
                        Num(invFlattening) + Authority(d.ellipsoid) + "]";
        if (!towgs.empty()) {
            w += ",TOWGS84[";
            for (size_t i = 0; i < 7; ++i) w += (i ? "," : "") + Num(towgs[i]);
            w += "]";
        }
        // GDAL passes the prime meridian (degrees) to OGRSpatialReference::SetGeogCS together with
        // the angular unit, then re-imports its WKT1 when that unit is not the degree (the
        // "test_tiff_grads" normalisation): the WKT1 value ends up as offset * unit-in-degrees.
        w += Authority(d.datum) + "],PRIMEM[" + Quote(pmName.empty() ? "Greenwich" : pmName) + "," +
             Num(pmLong * angInDeg) + Authority(d.pm) + "],UNIT[" + Quote(angularUnits) + "," + Num(angInDeg * kDegToRad) +
             Authority(d.uomAngle == 9110 ? kUser : d.uomAngle) + "]";
        if (gcsValid) w += Authority(d.gcs);
        w += "]";
        geogWkt = w;
        if (d.model == 2) {
            if (geogIsPlainEpsg) {
                horizontal = "EPSG:" + std::to_string(d.gcs);
                horizontalIsEpsg = true;
                horizontalCode = d.gcs;
            } else {
                horizontal = geogWkt;
            }
        }
    }

    // ---- Geocentric --------------------------------------------------------------------------
    if (horizontal.empty() && d.model == 3) {
        std::string name = !gtCitation.empty() ? gtCitation : !geogCitation.empty() ? geogCitation : "unnamed";
        std::string unitName = "unknown";
        if (d.uomLength != kUser) db.Uom(d.uomLength, &unitName, nullptr, nullptr);
        // GEOGCS[...] -> its DATUM and PRIMEM, inside a GEOCCS.
        const size_t datumAt = geogWkt.find(",DATUM[");
        const size_t unitAt = geogWkt.rfind(",UNIT[");
        if (datumAt == std::string::npos || unitAt == std::string::npos) return Corrupt("geocentric CRS");
        horizontal = "GEOCCS[" + Quote(name) + geogWkt.substr(datumAt, unitAt - datumAt) + ",UNIT[" + Quote(unitName) + "," +
                     Num(d.uomLengthInMeters) + Authority(d.uomLength) + "]]";
    }

    // ---- Projected from keys -------------------------------------------------------------------
    if (horizontal.empty() && d.model == 1) {
        // Name: the PCS name, or the PCS / GT citation.
        std::string name;
        if (d.pcs != kUser) db.PcsInfo(d.pcs, &name, nullptr, nullptr, nullptr);
        if (name.empty() && !pcsCitation.empty()) {
            if (const size_t at = pcsCitation.find(kPe); at != std::string::npos) return pcsCitation.substr(at + kPe.size());
            const Citation c = ParseCitation(pcsCitation, false);
            if (!c.pcs.empty()) name = c.pcs;
            else if (pcsCitation.rfind("LUnits = ", 0) != 0) name = pcsCitation;
        }
        if (const size_t at = gtCitation.find(kPe); at != std::string::npos) return gtCitation.substr(at + kPe.size());
        if (name.empty() && !gtCitation.empty()) {
            const Citation c = ParseCitation(gtCitation, false);
            name = !c.pcs.empty() ? c.pcs : gtCitation;
        }
        if (name.empty()) name = "unnamed";
        std::string unitName = "unknown";
        if (d.uomLength != kUser) db.Uom(d.uomLength, &unitName, nullptr, nullptr);
        const std::string unit = ",UNIT[" + Quote(unitName) + "," + Num(d.uomLengthInMeters) + Authority(d.uomLength) + "]";
        const bool pcsValid = d.pcs != kUser && ExistsInDb(db, d.pcs, PJ_CATEGORY_CRS);
        const double angUnitDeg = d.uomAngleInDegrees == 0 ? 1.0 : d.uomAngleInDegrees;

        if (d.ctProjection == kUser && d.projCode != kUser) {
            // Projection from an EPSG conversion code (ProjectionGeoKey): combine PROJ objects.
            PjPtr conv = db.Create(d.projCode, PJ_CATEGORY_COORDINATE_OPERATION);
            PjPtr geog(proj_create(db.ctx(), geogWkt.c_str()));
            if (!conv || !geog) return Unsupported("projection EPSG:" + std::to_string(d.projCode));
            PjPtr cs(proj_create_cartesian_2D_cs(db.ctx(), PJ_CART2D_EASTING_NORTHING, unitName.c_str(), d.uomLengthInMeters));
            PjPtr base(proj_crs_get_geodetic_crs(db.ctx(), geog.get()));
            PjPtr pcs(proj_create_projected_crs(db.ctx(), name.c_str(), base ? base.get() : geog.get(), conv.get(), cs.get()));
            if (!pcs) return Unsupported("cannot build a projected CRS from EPSG conversion " + std::to_string(d.projCode));
            horizontal = WktOf(db, pcs.get(), PJ_WKT2_2019);
        } else {
            std::string projWkt;
            if (!ProjectionWkt(d, angUnitDeg, d.uomLengthInMeters == 0 ? 1.0 : d.uomLengthInMeters, &projWkt)) {
                // GDAL: an unknown transform makes a LOCAL_CS of the same name.
                horizontal = "LOCAL_CS[" + Quote(name) + unit + "]";
            } else {
                horizontal = "PROJCS[" + Quote(name) + "," + geogWkt + projWkt + unit + (pcsValid ? Authority(d.pcs) : "") + "]";
            }
        }
    }
    if (horizontal.empty()) return std::string();

    // ---- Vertical ------------------------------------------------------------------------------
    if (vcs == 0 && vdatum == 0 && vunits == 0) return horizontal;
    std::string vertName;
    if (auto c = keys.Ascii(kVerticalCitation)) {
        if (c->rfind("VCS Name = ", 0) == 0) {
            vertName = c->substr(11);
            const size_t bar = vertName.find('|');
            if (bar != std::string::npos) vertName.resize(bar);
        }
    }
    // A Geographic 3D CRS from GeographicTypeGeoKey + VerticalCSTypeGeoKey.
    if (d.model == 2 && d.gcs != kUser) {
        const int code3d = vdatum == 6030 && d.gcs == 4326 ? 4979 : vcs;
        if (code3d > 0 && code3d != kUser) {
            PjPtr v3 = db.Create(code3d, PJ_CATEGORY_CRS);
            if (v3 && proj_get_type(v3.get()) == PJ_TYPE_GEOGRAPHIC_3D_CRS) {
                int datum2d = kUser, datum3d = kUser;
                db.GcsInfo(d.gcs, nullptr, &datum2d, nullptr, nullptr);
                db.GcsInfo(code3d, nullptr, &datum3d, nullptr, nullptr);
                if (datum2d == datum3d && datum2d != kUser) return "EPSG:" + std::to_string(code3d);
            }
        }
    }
    if (vcs != kUser && vcs > 0) {
        PjPtr v = db.Create(vcs, PJ_CATEGORY_CRS);
        if (v && proj_get_type(v.get()) == PJ_TYPE_VERTICAL_CRS) {
            if (horizontalIsEpsg) return "EPSG:" + std::to_string(horizontalCode) + "+" + std::to_string(vcs);
            PjPtr h(proj_create(db.ctx(), horizontal.c_str()));
            if (!h) return horizontal;
            const std::string compName = Db::Name(h.get()) + " + " + Db::Name(v.get());
            PjPtr comp(proj_create_compound_crs(db.ctx(), compName.c_str(), h.get(), v.get()));
            return comp ? WktOf(db, comp.get(), PJ_WKT2_2019) : horizontal;
        }
        return horizontal;  // GDAL cannot build the compound CRS either
    }
    // Manual VERT_CS (FillCompoundCRSWithManualVertCS).
    std::string vdatumName = "unknown";
    std::string vdatumAuth;
    if (vdatum > 0 && vdatum != kUser) {
        PjPtr dd = db.Create(vdatum, PJ_CATEGORY_DATUM);
        if (dd) vdatumName = Db::Name(dd.get());
        vdatumAuth = Authority(vdatum);
    }
    std::string vunit = "UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]]";
    if (vunits > 0 && vunits != kUser && vunits != 9001) {
        std::string un;
        double f = 0;
        if (db.Uom(vunits, &un, &f, nullptr)) vunit = "UNIT[" + Quote(un) + "," + Num(f) + Authority(vunits) + "]";
    }
    if (vertName.empty()) vertName = "unknown";
    std::string horizWkt1 = horizontal;
    if (horizontalIsEpsg) {
        PjPtr h(proj_create(db.ctx(), horizontal.c_str()));
        if (!h) return horizontal;
        horizWkt1 = WktOf(db, h.get(), PJ_WKT1_GDAL);
        if (horizWkt1.empty()) return horizontal;
    } else if (horizontal.rfind("PROJCS[", 0) != 0 && horizontal.rfind("GEOGCS[", 0) != 0 &&
               horizontal.rfind("LOCAL_CS[", 0) != 0) {
        PjPtr h(proj_create(db.ctx(), horizontal.c_str()));
        if (!h) return horizontal;
        horizWkt1 = WktOf(db, h.get(), PJ_WKT1_GDAL);
        if (horizWkt1.empty()) return horizontal;
    }
    std::string hname = "unnamed";
    {
        PjPtr h(proj_create(db.ctx(), horizWkt1.c_str()));
        if (h) hname = Db::Name(h.get());
    }
    const std::string compName = !gtCitation.empty() && gtCitation.find(" = ") == std::string::npos
                                     ? gtCitation
                                     : hname + " + " + vertName;
    return "COMPD_CS[" + Quote(compName) + "," + horizWkt1 + ",VERT_CS[" + Quote(vertName) + ",VERT_DATUM[" +
           Quote(vdatumName) + ",2005" + vdatumAuth + "]," + vunit + ",AXIS[\"Up\",UP]]]";
}

}  // namespace ddbt::cog::tiff
