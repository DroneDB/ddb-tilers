// SPDX-License-Identifier: MPL-2.0

#include "core/crs/transform.h"

#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <exception>
#include <limits>
#include <vector>

#include "core/crs/proj_context.h"
#include "core/crs/web_mercator_analytic.h"

namespace ddbt::crs {

using detail::PjPtr;

struct Transform::Impl {
    // The prepared operation ("prototype"). It is created in the creating thread's context and
    // used only under the mutex, after proj_assign_context() to the calling thread's context
    // (as GDAL's OGRProjCT does): no context, hence no proj.db connection, per transform.
    std::mutex mutex;
    PJ* proto = nullptr;
    std::string definition;

    Impl() = default;
    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    ~Impl() {
        if (proto == nullptr) return;
        // The creating thread may be gone: destroy through a temporary context (as PROJ's own
        // PJ_OPERATION_LIST destructor does). A bare context opens no database.
        PJ_CONTEXT* tmp = proj_context_create();
        proj_assign_context(proto, tmp);
        proj_destroy(proto);
        proj_context_destroy(tmp);
    }
};

namespace {

// EPSG:3857 only. GDAL's shortcut also takes the deprecated EPSG:3785 as a source, but the
// inverse of that transform would be PROJ's 4326 -> 3785 pipeline, not checked here: PROJ does it.
bool IsWebMercator(const Crs& c) {
    const auto e = c.Epsg();
    return e && *e == 3857;
}
bool IsWgs84LonLat(const Crs& c) {
    const auto e = c.Epsg();
    return e && *e == 4326;
}

PJ* CloneProto(const void* owner, PJ_CONTEXT* ctx) {
    auto* impl = static_cast<Transform::Impl*>(const_cast<void*>(owner));
    std::lock_guard lock(impl->mutex);
    proj_assign_context(impl->proto, ctx);
    return proj_clone(ctx, impl->proto);
}

// Marks failed points as GDAL does: x = y = HUGE_VAL (NaN included).
size_t MarkFailures(double* xs, double* ys, size_t n) noexcept {
    size_t failed = 0;
    for (size_t i = 0; i < n; ++i) {
        if (!std::isfinite(xs[i]) || !std::isfinite(ys[i])) {
            xs[i] = HUGE_VAL;
            ys[i] = HUGE_VAL;
            ++failed;
        }
    }
    return failed;
}

std::string Fmt(double v) {
    char buf[40];
    std::snprintf(buf, sizeof buf, "%.17g", v);
    return buf;
}

}  // namespace

std::string TransformOptions::Key() const {
    std::string k;
    if (areaOfInterest) {
        k += "aoi=" + Fmt(areaOfInterest->west) + "," + Fmt(areaOfInterest->south) + "," +
             Fmt(areaOfInterest->east) + "," + Fmt(areaOfInterest->north) + ";";
    }
    k += allowBallpark ? "bp=1;" : "bp=0;";
    if (onlyBest) k += *onlyBest ? "best=1;" : "best=0;";
    if (accuracy) k += "acc=" + Fmt(*accuracy) + ";";
    k += analyticWebMercator ? "wm=1" : "wm=0";
    return k;
}

Transform::Transform(Crs src, Crs dst, TransformOptions options)
    : src_(std::move(src)), dst_(std::move(dst)), options_(std::move(options)) {}

Transform::~Transform() = default;

std::shared_ptr<const Transform> Transform::Create(const Crs& src, const Crs& dst, const TransformOptions& options) {
    if (!src.Valid() || !dst.Valid()) throw CrsError("transform between empty CRSs");
    std::shared_ptr<Transform> t(new Transform(src, dst, options));
    t->id_ = detail::NextId();
    t->alive_ = std::make_shared<int>(0);

    if (src.Equivalent(dst)) {
        t->kind_ = Kind::Identity;
        return t;
    }
    if (options.analyticWebMercator && IsWgs84LonLat(src) && IsWebMercator(dst)) {
        t->kind_ = Kind::WebMercatorFromLonLat;
        return t;
    }
    if (options.analyticWebMercator && IsWebMercator(src) && IsWgs84LonLat(dst)) {
        t->kind_ = Kind::WebMercatorToLonLatGdal;
        return t;
    }

    auto impl = std::make_unique<Impl>();
    PJ_CONTEXT* ctx = detail::ThreadContext();
    detail::ClearLastError();
    PjPtr s(proj_create(ctx, src.Definition().c_str()));
    PjPtr d(proj_create(ctx, dst.Definition().c_str()));
    if (!s || !d) throw CrsError("PROJ cannot create the CRSs: " + detail::LastError(ctx));

    PJ_AREA* area = nullptr;
    if (options.areaOfInterest) {
        area = proj_area_create();
        proj_area_set_bbox(area, options.areaOfInterest->west, options.areaOfInterest->south,
                           options.areaOfInterest->east, options.areaOfInterest->north);
    }
    // GDAL's option list (OGRProjCT::Initialize, Strategy::PROJ)
    std::vector<std::string> opts;
    if (options.accuracy) opts.push_back("ACCURACY=" + Fmt(*options.accuracy));
    if (!options.allowBallpark) opts.emplace_back("ALLOW_BALLPARK=NO");
    if (options.onlyBest) opts.push_back(std::string("ONLY_BEST=") + (*options.onlyBest ? "YES" : "NO"));
    std::vector<const char*> optPtrs;
    for (const auto& o : opts) optPtrs.push_back(o.c_str());
    optPtrs.push_back(nullptr);

    PjPtr op(proj_create_crs_to_crs_from_pj(ctx, s.get(), d.get(), area, optPtrs.data()));
    if (area != nullptr) proj_area_destroy(area);
    if (!op) {
        throw CrsError("no coordinate operation from '" + src.Name() + "' to '" + dst.Name() +
                       "': " + detail::LastError(ctx));
    }
    PjPtr norm(proj_normalize_for_visualization(ctx, op.get()));
    if (!norm) throw CrsError("proj_normalize_for_visualization failed: " + detail::LastError(ctx));
    // A set of candidate operations (PROJ picks per point by area of use) has no single
    // definition: LastUsedOperation() tells which one a point used.
    if (proj_get_type(norm.get()) != PJ_TYPE_UNKNOWN) {
        const PJ_PROJ_INFO info = proj_pj_info(norm.get());
        if (info.definition != nullptr) impl->definition = info.definition;
    }
    impl->proto = norm.release();
    t->impl_ = std::move(impl);
    t->kind_ = Kind::Proj;
    return t;
}

void* Transform::ThreadPj() const {
    PJ* pj = detail::ThreadCopy(id_, alive_, impl_.get(), &CloneProto);
    if (pj == nullptr) throw CrsError("proj_clone failed: " + detail::LastError(detail::ThreadContext()));
    return pj;
}

size_t Transform::TransformPoints(double* xs, double* ys, double* zs, size_t n, Direction dir) const {
    if (n == 0) return 0;
    switch (kind_) {
        case Kind::Identity:
            return 0;
        case Kind::WebMercatorFromLonLat:
            return dir == Direction::Forward ? analytic::LonLatToWebMercator(xs, ys, xs, ys, n)
                                             : analytic::WebMercatorToLonLatProj(xs, ys, xs, ys, n);
        case Kind::WebMercatorToLonLatGdal:
            // Forward: GDAL's shortcut; inverse: PROJ's 4326 -> 3857 (GDAL creates that one with PROJ).
            return dir == Direction::Forward ? analytic::WebMercatorToLonLatGdal(xs, ys, xs, ys, n)
                                             : analytic::LonLatToWebMercator(xs, ys, xs, ys, n);
        case Kind::Proj:
            break;
    }
    PJ* pj = static_cast<PJ*>(ThreadPj());
    const PJ_DIRECTION pd = dir == Direction::Forward ? PJ_FWD : PJ_INV;
    // GDAL marks a non-finite input as failed without calling PROJ.
    if (n == 1) {
        // proj_trans_generic treats arrays of length 1 as constants and does not write them back
        if (!std::isfinite(xs[0]) || !std::isfinite(ys[0])) {
            xs[0] = ys[0] = HUGE_VAL;
            return 1;
        }
        PJ_COORD c = proj_coord(xs[0], ys[0], zs != nullptr ? zs[0] : 0.0, HUGE_VAL);
        c = proj_trans(pj, pd, c);
        xs[0] = c.xyzt.x;
        ys[0] = c.xyzt.y;
        if (zs != nullptr) zs[0] = c.xyzt.z;
    } else {
        proj_trans_generic(pj, pd, xs, sizeof(double), n, ys, sizeof(double), n, zs, zs != nullptr ? sizeof(double) : 0,
                           zs != nullptr ? n : 0, nullptr, 0, 0);
    }
    proj_errno_reset(pj);
    return MarkFailures(xs, ys, n);
}

std::string Transform::Definition() const { return impl_ ? impl_->definition : std::string(); }

std::string Transform::LastUsedOperation() const {
    if (kind_ != Kind::Proj) return {};
    PJ* pj = static_cast<PJ*>(ThreadPj());
    PjPtr last(proj_trans_get_last_used_operation(pj));
    if (!last) return {};
    const char* s = proj_as_proj_string(detail::ThreadContext(), last.get(), PJ_PROJ_5, nullptr);
    return s != nullptr ? s : std::string();
}

std::optional<LonLatBox> Transform::AreaOfInterestFor(const Crs& src, double minX, double minY, double maxX,
                                                      double maxY) {
    PJ_CONTEXT* ctx = detail::ThreadContext();
    PjPtr crs(proj_create(ctx, src.Definition().c_str()));
    if (!crs) return std::nullopt;
    // StripVertical()
    if (proj_get_type(crs.get()) == PJ_TYPE_COMPOUND_CRS) crs.reset(proj_crs_get_sub_crs(ctx, crs.get(), 0));
    if (!crs) return std::nullopt;
    // CloneGeogCS(), degrees, 2D
    PjPtr geog(proj_crs_get_geodetic_crs(ctx, crs.get()));
    if (!geog) return std::nullopt;
    const PJ_TYPE gt = proj_get_type(geog.get());
    if (gt != PJ_TYPE_GEOGRAPHIC_2D_CRS && gt != PJ_TYPE_GEOGRAPHIC_3D_CRS) return std::nullopt;
    if (gt == PJ_TYPE_GEOGRAPHIC_3D_CRS) geog.reset(proj_crs_demote_to_2D(ctx, nullptr, geog.get()));
    if (geog) geog.reset(proj_crs_alter_cs_angular_unit(ctx, geog.get(), "degree", 0.0174532925199433, "EPSG", "9122"));
    if (!geog) return std::nullopt;
    PjPtr op(proj_create_crs_to_crs_from_pj(ctx, crs.get(), geog.get(), nullptr, nullptr));
    if (!op) return std::nullopt;
    PjPtr norm(proj_normalize_for_visualization(ctx, op.get()));
    if (!norm) return std::nullopt;

    // Corners in GDAL's order: top-left, top-right, bottom-left, bottom-right.
    double x[4] = {minX, maxX, minX, maxX};
    double y[4] = {maxY, maxY, minY, minY};
    bool valid[4];
    for (int i = 0; i < 4; ++i) {
        const PJ_COORD c = proj_trans(norm.get(), PJ_FWD, proj_coord(x[i], y[i], 0, HUGE_VAL));
        valid[i] = std::isfinite(c.xy.x) && std::isfinite(c.xy.y) && c.xy.x != HUGE_VAL;
        x[i] = c.xy.x;
        y[i] = c.xy.y;
    }
    proj_errno_reset(norm.get());
    LonLatBox b{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()};
    bool any = false;
    for (int i = 0; i < 4; ++i) {
        if (!valid[i]) continue;
        any = true;
        b.west = std::min(b.west, x[i]);
        b.south = std::min(b.south, y[i]);
        b.east = std::max(b.east, x[i]);
        b.north = std::max(b.north, y[i]);
    }
    if (valid[0] && valid[1] && x[0] > x[1]) {
        b.west = x[0];
        b.east = x[1];
    }
    if (!any || std::fabs(b.west) > 180 || std::fabs(b.east) > 180 || std::fabs(b.south) > 90 ||
        std::fabs(b.north) > 90) {
        return std::nullopt;
    }
    return b;
}

// ---- cache --------------------------------------------------------------------------------

struct TransformCache::Slot {
    std::mutex m;
    std::condition_variable cv;
    bool done = false;
    std::shared_ptr<const Transform> value;
    std::exception_ptr error;
};

TransformCache& TransformCache::Global() {
    static TransformCache cache;
    return cache;
}

std::shared_ptr<const Transform> TransformCache::Get(const Crs& src, const Crs& dst, const TransformOptions& options) {
    if (!src.Valid() || !dst.Valid()) throw CrsError("transform between empty CRSs");
    std::string key = src.Definition();
    key += '\x1f';
    key += dst.Definition();
    key += '\x1f';
    key += options.Key();

    std::shared_ptr<Slot> slot;
    bool build = false;
    {
        std::lock_guard lock(mutex_);
        auto it = map_.find(key);
        if (it == map_.end()) {
            if (map_.size() >= 4096) map_.clear();  // bound; transforms in use stay alive (shared_ptr)
            slot = std::make_shared<Slot>();
            map_.emplace(key, slot);
            build = true;
        } else {
            slot = it->second;
        }
    }
    if (build) {
        std::shared_ptr<const Transform> value;
        std::exception_ptr error;
        try {
            value = Transform::Create(src, dst, options);
        } catch (...) {
            error = std::current_exception();
        }
        if (error) {
            std::lock_guard lock(mutex_);
            auto it = map_.find(key);
            if (it != map_.end() && it->second == slot) map_.erase(it);  // a later call retries
        }
        {
            std::lock_guard lock(slot->m);
            slot->value = value;
            slot->error = error;
            slot->done = true;
        }
        slot->cv.notify_all();
        if (error) std::rethrow_exception(error);
        return value;
    }
    std::unique_lock lock(slot->m);
    slot->cv.wait(lock, [&] { return slot->done; });
    if (slot->error) std::rethrow_exception(slot->error);
    return slot->value;
}

std::shared_ptr<const Transform> TransformCache::GetForExtent(const Crs& src, const Crs& dst, double minX, double minY,
                                                              double maxX, double maxY, TransformOptions options) {
    options.areaOfInterest = Transform::AreaOfInterestFor(src, minX, minY, maxX, maxY);
    return Get(src, dst, options);
}

size_t TransformCache::Size() const {
    std::lock_guard lock(mutex_);
    return map_.size();
}

void TransformCache::Clear() {
    std::lock_guard lock(mutex_);
    map_.clear();
}

}  // namespace ddbt::crs
