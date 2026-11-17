// SPDX-License-Identifier: MPL-2.0

#include "core/crs/crs.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

#include "core/crs/proj_context.h"

namespace ddbt::crs {

using detail::PjPtr;

struct Crs::Data {
    std::string definition;
    std::string name;
    CrsType type = CrsType::Other;
    std::optional<int> epsg;
    double unitFactor = 1.0;
    bool horizontalGeographic = false;
    bool northingFirst = false;
    int axisCount = 0;
    std::optional<LonLatBox> area;
};

namespace {

CrsType TypeOf(PJ_TYPE t) {
    switch (t) {
        case PJ_TYPE_GEOGRAPHIC_2D_CRS: return CrsType::Geographic2D;
        case PJ_TYPE_GEOGRAPHIC_3D_CRS: return CrsType::Geographic3D;
        case PJ_TYPE_GEOCENTRIC_CRS: return CrsType::Geocentric;
        case PJ_TYPE_PROJECTED_CRS:
        case PJ_TYPE_DERIVED_PROJECTED_CRS: return CrsType::Projected;
        case PJ_TYPE_VERTICAL_CRS: return CrsType::Vertical;
        case PJ_TYPE_COMPOUND_CRS: return CrsType::Compound;
        case PJ_TYPE_ENGINEERING_CRS: return CrsType::Engineering;
        case PJ_TYPE_BOUND_CRS: return CrsType::Bound;
        default: return CrsType::Other;
    }
}

// The CRS whose first axis gives the horizontal unit: the base of a BoundCRS, the horizontal
// part of a compound CRS.
PjPtr HorizontalOf(PJ_CONTEXT* ctx, const PJ* crs) {
    PjPtr cur(proj_clone(ctx, crs));
    for (int guard = 0; cur && guard < 4; ++guard) {
        const PJ_TYPE t = proj_get_type(cur.get());
        if (t == PJ_TYPE_BOUND_CRS) {
            cur.reset(proj_get_source_crs(ctx, cur.get()));
        } else if (t == PJ_TYPE_COMPOUND_CRS) {
            cur.reset(proj_crs_get_sub_crs(ctx, cur.get(), 0));
        } else {
            break;
        }
    }
    return cur;
}

// The axes of a CRS's coordinates: a bound CRS's source's, the sum of a compound CRS's parts.
int AxisCountOf(PJ_CONTEXT* ctx, const PJ* crs, int guard = 0) {
    if (guard > 4) return 0;
    const PJ_TYPE t = proj_get_type(crs);
    if (t == PJ_TYPE_BOUND_CRS) {
        const PjPtr source(proj_get_source_crs(ctx, crs));
        return source ? AxisCountOf(ctx, source.get(), guard + 1) : 0;
    }
    if (t == PJ_TYPE_COMPOUND_CRS) {
        int n = 0;
        for (int i = 0;; ++i) {
            const PjPtr part(proj_crs_get_sub_crs(ctx, crs, i));
            if (!part) break;
            n += AxisCountOf(ctx, part.get(), guard + 1);
        }
        detail::ClearLastError();  // the index past the last part
        return n;
    }
    const PjPtr cs(proj_crs_get_coordinate_system(ctx, crs));
    return cs ? proj_cs_get_axis_count(ctx, cs.get()) : 0;
}

// GDAL's EPSGTreatsAsLatLong (a geographic CRS whose first axis points north) or
// EPSGTreatsAsNorthingEasting (a projected CRS whose axes are north then east, or northing then
// easting both pointing north or south, as in polar projections), on the horizontal CRS.
bool NorthingFirst(PJ_CONTEXT* ctx, const PJ* cs, PJ_TYPE horizontalType) {
    auto axis = [&](int i, const char** name, const char** direction) {
        *name = nullptr;
        *direction = nullptr;
        return proj_cs_get_axis_count(ctx, cs) > i &&
               proj_cs_get_axis_info(ctx, cs, i, name, nullptr, direction, nullptr, nullptr, nullptr, nullptr) &&
               *direction != nullptr;
    };
    auto is = [](const char* s, const char* what) {
        if (s == nullptr) return false;
        for (; *s != '\0' && *what != '\0'; ++s, ++what)
            if (std::tolower(static_cast<unsigned char>(*s)) != *what) return false;
        return *s == '\0' && *what == '\0';
    };
    auto startsWith = [](const char* s, const char* prefix) {
        if (s == nullptr) return false;
        for (; *prefix != '\0'; ++s, ++prefix)
            if (*s == '\0' || std::tolower(static_cast<unsigned char>(*s)) != *prefix) return false;
        return true;
    };
    const char *name0, *dir0, *name1, *dir1;
    if (!axis(0, &name0, &dir0)) return false;
    if (horizontalType == PJ_TYPE_GEOGRAPHIC_2D_CRS || horizontalType == PJ_TYPE_GEOGRAPHIC_3D_CRS)
        return is(dir0, "north");
    if (horizontalType != PJ_TYPE_PROJECTED_CRS || !axis(1, &name1, &dir1)) return false;
    if (is(dir0, "north") && is(dir1, "east")) return true;
    return ((is(dir0, "north") && is(dir1, "north")) || (is(dir0, "south") && is(dir1, "south"))) &&
           startsWith(name0, "northing") && startsWith(name1, "easting");
}

// wkt: the text the CRS was read from when it is a WKT ("" otherwise)
Crs::Data Describe(PJ_CONTEXT* ctx, PJ* pj, const std::string& wkt) {
    Crs::Data d;
    // Canonical definition: AUTH:CODE when equivalent to the authority's definition.
    const char* auth = proj_get_id_auth_name(pj, 0);
    const char* code = proj_get_id_code(pj, 0);
    PjPtr official;
    if (auth != nullptr && code != nullptr) {
        official.reset(proj_create_from_database(ctx, auth, code, PJ_CATEGORY_CRS, 0, nullptr));
        if (official && !proj_is_equivalent_to_with_ctx(ctx, official.get(), pj,
                                                        PJ_COMP_EQUIVALENT_EXCEPT_AXIS_ORDER_GEOGCRS)) {
            official.reset();
        }
        proj_errno_reset(pj);
    }
    const PJ* use = official ? official.get() : pj;
    if (official) {
        d.definition = std::string(auth) + ":" + code;
    } else if (!wkt.empty()) {
        // The WKT itself: PROJJSON leaves out the identifiers of the datum and the ellipsoid
        // under a CRS that has one, which GDAL's import keeps (and which choose the operations)
        d.definition = wkt;
    } else {
        const char* json = proj_as_projjson(ctx, pj, nullptr);
        if (json == nullptr) throw CrsError("cannot export the CRS to PROJJSON: " + detail::LastError(ctx));
        d.definition = json;
    }
    if (const char* n = proj_get_name(use)) d.name = n;
    PJ_TYPE t = proj_get_type(use);
    if (t == PJ_TYPE_BOUND_CRS) {
        PjPtr base(proj_get_source_crs(ctx, use));
        if (base) t = proj_get_type(base.get());
    }
    d.type = TypeOf(t);
    d.axisCount = AxisCountOf(ctx, use);
    if (auth != nullptr && code != nullptr && std::strcmp(auth, "EPSG") == 0) {
        char* end = nullptr;
        const long v = std::strtol(code, &end, 10);
        if (end != code && *end == '\0') d.epsg = static_cast<int>(v);
    }
    if (PjPtr h = HorizontalOf(ctx, use)) {
        const PJ_TYPE ht = proj_get_type(h.get());
        d.horizontalGeographic = ht == PJ_TYPE_GEOGRAPHIC_2D_CRS || ht == PJ_TYPE_GEOGRAPHIC_3D_CRS;
        if (PjPtr cs{proj_crs_get_coordinate_system(ctx, h.get())}) {
            double factor = 0;
            if (proj_cs_get_axis_count(ctx, cs.get()) > 0 &&
                proj_cs_get_axis_info(ctx, cs.get(), 0, nullptr, nullptr, nullptr, &factor, nullptr, nullptr,
                                      nullptr)) {
                d.unitFactor = factor;
            }
            d.northingFirst = NorthingFirst(ctx, cs.get(), ht);
        }
    }
    double w = 0, s = 0, e = 0, n = 0;
    if (proj_get_area_of_use(ctx, use, &w, &s, &e, &n, nullptr) && w > -1000 && s > -1000 && e > -1000 &&
        n > -1000) {
        d.area = LonLatBox{w, s, e, n};
    }
    return d;
}

PjPtr Create(PJ_CONTEXT* ctx, const std::string& def) {
    PjPtr pj = detail::CreateCrs(ctx, def);
    if (!pj) throw CrsError("PROJ cannot create the CRS: " + detail::LastError(ctx));
    return pj;
}

const Crs::Data& Need(const std::shared_ptr<const Crs::Data>& d) {
    if (!d) throw CrsError("empty CRS");
    return *d;
}

}  // namespace

Crs Crs::FromText(std::string_view text, bool wkt) {
    PJ_CONTEXT* ctx = detail::ThreadContext();
    detail::ClearLastError();
    const std::string s(text);
    PjPtr pj;
    if (wkt) {
        std::string err;
        pj = detail::CreateCrs(ctx, s, &err);
        if (!pj) throw CrsError("invalid WKT: " + (err.empty() ? detail::LastError(ctx) : err));
    } else {
        pj.reset(proj_create(ctx, s.c_str()));
        if (!pj) throw CrsError("PROJ cannot parse '" + s.substr(0, 80) + "': " + detail::LastError(ctx));
    }
    if (!proj_is_crs(pj.get())) throw CrsError("not a CRS: '" + s.substr(0, 80) + "'");
    return Crs(std::make_shared<const Data>(Describe(ctx, pj.get(), wkt ? s : std::string())));
}

Crs Crs::FromEpsg(int code) { return FromText("EPSG:" + std::to_string(code), false); }
Crs Crs::FromUserInput(std::string_view text) { return FromText(text, false); }
Crs Crs::FromWkt(std::string_view wkt) { return FromText(wkt, true); }
Crs Crs::FromProjJson(std::string_view json) { return FromText(json, false); }

const std::string& Crs::Definition() const { return Need(d_).definition; }
const std::string& Crs::Name() const { return Need(d_).name; }
CrsType Crs::Type() const { return Need(d_).type; }
std::optional<int> Crs::Epsg() const { return Need(d_).epsg; }
double Crs::HorizontalUnitFactor() const { return Need(d_).unitFactor; }
bool Crs::IsHorizontallyGeographic() const { return Need(d_).horizontalGeographic; }
bool Crs::NorthingFirst() const { return Need(d_).northingFirst; }
int Crs::AxisCount() const { return Need(d_).axisCount; }

namespace {

// Part `index` of a compound CRS (an empty CRS when there is none).
Crs CompoundPart(const Crs& c, int index) {
    PJ_CONTEXT* ctx = detail::ThreadContext();
    const PjPtr pj = Create(ctx, c.Definition());
    const PjPtr part(proj_crs_get_sub_crs(ctx, pj.get(), index));
    if (!part) {
        detail::ClearLastError();
        return Crs();
    }
    const char* const options[] = {"MULTILINE=NO", nullptr};
    const char* json = proj_as_projjson(ctx, part.get(), options);
    if (json == nullptr) throw CrsError("cannot export a part of the CRS: " + detail::LastError(ctx));
    return Crs::FromProjJson(json);
}

}  // namespace

std::string BoundsMisfit(const Crs& c, double minX, double minY, double maxX, double maxY) {
    if (!c.Valid() || !c.IsHorizontallyGeographic()) return "";
    if (minX >= -180 && maxX <= 180 && minY >= -90 && maxY <= 90) return "";
    const std::string& name = c.Name();
    return "the coordinates do not fit " + (name.empty() ? std::string("the geographic CRS of the file") : name);
}

// A compound CRS whose parts PROJ cannot split keeps itself as its horizontal part, so a caller can
// still transform its x, y
Crs Crs::HorizontalPart() const {
    if (!IsCompound()) return *this;
    Crs part = CompoundPart(*this, 0);
    return part.Valid() ? part : *this;
}
Crs Crs::VerticalPart() const { return IsCompound() ? CompoundPart(*this, 1) : Crs(); }
std::optional<LonLatBox> Crs::AreaOfUse() const { return Need(d_).area; }

namespace {

// OGRSpatialReference::exportToWkt's choice of WKT2 when no format is asked for: a derived
// geographic CRS, and a geographic or projected CRS of 3 axes that is not compound
bool GdalExportsWkt2(PJ_CONTEXT* ctx, PJ* pj) {
    // A bound CRS is judged by its source CRS, as GDAL does
    PjPtr source;
    PJ* crs = pj;
    if (proj_get_type(pj) == PJ_TYPE_BOUND_CRS) {
        source.reset(proj_get_source_crs(ctx, pj));
        if (source) crs = source.get();
    }
    const PJ_TYPE type = proj_get_type(crs);
    const bool geographic = type == PJ_TYPE_GEOGRAPHIC_2D_CRS || type == PJ_TYPE_GEOGRAPHIC_3D_CRS;
    if (geographic && proj_is_derived_crs(ctx, crs)) return true;
    if (!geographic && type != PJ_TYPE_PROJECTED_CRS) return false;
    const PjPtr cs(proj_crs_get_coordinate_system(ctx, crs));
    return cs && proj_cs_get_axis_count(ctx, cs.get()) == 3;
}

}  // namespace

Crs Crs::GeographicBase() const {
    PJ_CONTEXT* ctx = detail::ThreadContext();
    const PjPtr pj = Create(ctx, Definition());
    const PjPtr horizontal = HorizontalOf(ctx, pj.get());
    const PjPtr geodetic(proj_crs_get_geodetic_crs(ctx, horizontal ? horizontal.get() : pj.get()));
    if (!geodetic) throw CrsError("the CRS has no geographic CRS: " + detail::LastError(ctx));
    const char* const options[] = {"MULTILINE=NO", nullptr};
    const char* wkt = proj_as_wkt(ctx, geodetic.get(), PJ_WKT2_2019, options);
    if (wkt == nullptr) throw CrsError("cannot export the geographic CRS: " + detail::LastError(ctx));
    return FromWkt(wkt);
}

namespace {

// The text of the first `key["` node of a WKT1 text ("" if none)
std::string WktNodeText(const std::string& wkt, const std::string& key) {
    const size_t at = wkt.find(key + "[\"");
    if (at == std::string::npos) return {};
    const size_t from = at + key.size() + 2, to = wkt.find('"', from);
    return to == std::string::npos ? std::string() : wkt.substr(from, to - from);
}

// GetProjParm: the value of PARAMETER["name", value] (the name compared without case), else 0
double WktParameter(const std::string& wkt, const std::string& name) {
    for (size_t at = wkt.find("PARAMETER[\""); at != std::string::npos; at = wkt.find("PARAMETER[\"", at + 1)) {
        const size_t from = at + 11, to = wkt.find('"', from);
        if (to == std::string::npos) break;
        const std::string key = wkt.substr(from, to - from);
        if (key.size() == name.size() &&
            std::equal(key.begin(), key.end(), name.begin(), [](char a, char b) {
                return std::tolower(static_cast<unsigned char>(a)) == std::tolower(static_cast<unsigned char>(b));
            }))
            return std::strtod(wkt.c_str() + to + 2, nullptr);
    }
    return 0.0;
}

}  // namespace

std::optional<Crs::ProjectionMethod> Crs::Projection() const {
    const std::string wkt = ToWkt(WktFlavor::Wkt1Gdal);
    ProjectionMethod m;
    m.name = WktNodeText(wkt, "PROJECTION");
    if (m.name.empty()) return std::nullopt;
    // GetNormProjParm: the angular parameters in degrees, from the unit of the geographic CRS
    constexpr double kDegree = 0.0174532925199433;
    double toDegrees = GeographicBase().HorizontalUnitFactor() / kDegree;
    if (std::fabs(toDegrees - 1.0) < 0.000000001) toDegrees = 1.0;
    const auto norm = [&](double v) { return v * toDegrees; };
    m.centralMeridian = norm(WktParameter(wkt, "central_meridian"));
    m.latitudeOfOrigin = norm(WktParameter(wkt, "latitude_of_origin"));
    return m;
}

std::string Crs::ToWkt(WktFlavor flavor) const {
    PJ_CONTEXT* ctx = detail::ThreadContext();
    const PjPtr pj = Create(ctx, Definition());
    PJ_WKT_TYPE t = flavor == WktFlavor::Wkt2_2019 ? PJ_WKT2_2019
                    : flavor == WktFlavor::Wkt1Esri ? PJ_WKT1_ESRI
                                                    : PJ_WKT1_GDAL;
    if (flavor == WktFlavor::GdalDefault && GdalExportsWkt2(ctx, pj.get())) t = PJ_WKT2_2019;
    const char* const plain[] = {"MULTILINE=NO", nullptr};
    // GDAL writes the axes but in ESRI's WKT (OUTPUT_AXIS=YES)
    const char* const withAxes[] = {"MULTILINE=NO", "OUTPUT_AXIS=YES", nullptr};
    const char* wkt = proj_as_wkt(ctx, pj.get(), t, flavor == WktFlavor::GdalDefault ? withAxes : plain);
    if (wkt == nullptr) throw CrsError("cannot export the CRS to WKT: " + detail::LastError(ctx));
    return wkt;
}

std::string Crs::ToProjJson() const {
    PJ_CONTEXT* ctx = detail::ThreadContext();
    const PjPtr pj = Create(ctx, Definition());
    const char* const options[] = {"MULTILINE=NO", nullptr};
    const char* json = proj_as_projjson(ctx, pj.get(), options);
    if (json == nullptr) throw CrsError("cannot export the CRS to PROJJSON: " + detail::LastError(ctx));
    return json;
}

bool Crs::Equivalent(const Crs& other) const {
    if (*this == other) return true;
    if (!Valid() || !other.Valid()) return false;
    PJ_CONTEXT* ctx = detail::ThreadContext();
    const PjPtr a = Create(ctx, Definition());
    const PjPtr b = Create(ctx, other.Definition());
    return proj_is_equivalent_to_with_ctx(ctx, a.get(), b.get(), PJ_COMP_EQUIVALENT_EXCEPT_AXIS_ORDER_GEOGCRS) != 0;
}

}  // namespace ddbt::crs
