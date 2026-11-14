// SPDX-License-Identifier: MPL-2.0

#include "core/crs/crs.h"

#include <cctype>
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

Crs::Data Describe(PJ_CONTEXT* ctx, PJ* pj) {
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
    PjPtr pj(proj_create(ctx, def.c_str()));
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
        const char* const options[] = {"STRICT=NO", "UNSET_IDENTIFIERS_IF_INCOMPATIBLE_DEF=NO", nullptr};
        PROJ_STRING_LIST warnings = nullptr;
        PROJ_STRING_LIST errors = nullptr;
        pj.reset(proj_create_from_wkt(ctx, s.c_str(), options, &warnings, &errors));
        std::string err;
        for (auto it = errors; it != nullptr && *it != nullptr; ++it) err += std::string(err.empty() ? "" : "; ") + *it;
        proj_string_list_destroy(warnings);
        proj_string_list_destroy(errors);
        if (!pj) throw CrsError("invalid WKT: " + (err.empty() ? detail::LastError(ctx) : err));
    } else {
        pj.reset(proj_create(ctx, s.c_str()));
        if (!pj) throw CrsError("PROJ cannot parse '" + s.substr(0, 80) + "': " + detail::LastError(ctx));
    }
    if (!proj_is_crs(pj.get())) throw CrsError("not a CRS: '" + s.substr(0, 80) + "'");
    return Crs(std::make_shared<const Data>(Describe(ctx, pj.get())));
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
std::optional<LonLatBox> Crs::AreaOfUse() const { return Need(d_).area; }

std::string Crs::ToWkt(WktFlavor flavor) const {
    PJ_CONTEXT* ctx = detail::ThreadContext();
    const PjPtr pj = Create(ctx, Definition());
    const PJ_WKT_TYPE t = flavor == WktFlavor::Wkt2_2019  ? PJ_WKT2_2019
                          : flavor == WktFlavor::Wkt1Gdal ? PJ_WKT1_GDAL
                                                          : PJ_WKT1_ESRI;
    const char* const options[] = {"MULTILINE=NO", nullptr};
    const char* wkt = proj_as_wkt(ctx, pj.get(), t, options);
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
