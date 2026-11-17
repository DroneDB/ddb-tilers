// SPDX-License-Identifier: MPL-2.0
#include "copc/build/point_transform.h"

#include <cmath>
#include <cstdio>
#include <vector>

namespace ddbt::copc::build {

namespace {

/// The calling thread's coordinates of a batch, one array per axis as PROJ takes them.
struct Axes {
    std::vector<double> x, y, z;
};
Axes& ThreadAxes() {
    thread_local Axes a;
    return a;
}

std::string PointText(const double* p) {
    char buf[96];
    std::snprintf(buf, sizeof buf, "(%.10g, %.10g, %.10g)", p[0], p[1], p[2]);
    return buf;
}

}  // namespace

Expected<std::shared_ptr<const PointTransform>> PointTransform::Create(const crs::Crs& source, const crs::Crs& target,
                                                                       const std::optional<std::array<double, 4>>& bounds,
                                                                       double maxError) {
    try {
        // Heights are kept when both CRSs give them in the same vertical CRS, or the target has
        // none: X / Y then transform between the horizontal parts. A geocentric CRS, a 3-D
        // geographic or projected one (ellipsoidal heights) and a change of vertical CRS transform
        // in 3-D. A geocentric source has no horizontal box to choose an operation by.
        const crs::Crs fromVertical = source.VerticalPart(), toVertical = target.VerticalPart();
        const bool ellipsoidal = (source.AxisCount() > 2 && !source.IsCompound()) ||
                                 (target.AxisCount() > 2 && !target.IsCompound());
        const bool heightsKept = !toVertical.Valid() || (fromVertical.Valid() && fromVertical.Equivalent(toVertical));
        const bool threeD = ellipsoidal || !heightsKept;
        const crs::Crs from = threeD ? source : source.HorizontalPart();
        const crs::Crs to = threeD ? target : target.HorizontalPart();
        crs::TransformCache& cache = crs::TransformCache::Global();
        std::shared_ptr<const crs::Transform> t =
            bounds && !source.IsGeocentric()
                ? cache.GetForExtent(from, to, (*bounds)[0], (*bounds)[1], (*bounds)[2], (*bounds)[3])
                : cache.Get(from, to);
        std::optional<crs::GridApprox> grid;
        const bool box = bounds && (*bounds)[2] > (*bounds)[0] && (*bounds)[3] > (*bounds)[1];
        if (!threeD && box && t->GetKind() != crs::Transform::Kind::Identity) {
            crs::GridApproxOptions go;
            go.maxError = maxError;
            grid = crs::GridApprox::Build(t, (*bounds)[0], (*bounds)[1], (*bounds)[2], (*bounds)[3], go);
        }
        return std::shared_ptr<const PointTransform>(new PointTransform(std::move(t), threeD, std::move(grid)));
    } catch (const crs::CrsError& e) {
        return InvalidArgument(std::string("no transform from ") + source.Name() + " to " + target.Name() + ": " + e.what());
    }
}

Status PointTransform::Apply(const double* xyz, size_t n, double* out) const {
    if (n == 0) return Status();
    Axes& a = ThreadAxes();
    a.x.resize(n);
    a.y.resize(n);
    if (threeD_) a.z.resize(n);
    for (size_t i = 0; i < n; ++i) {
        a.x[i] = xyz[3 * i];
        a.y[i] = xyz[3 * i + 1];
        if (threeD_) a.z[i] = xyz[3 * i + 2];
    }
    // A failed point is HUGE_VAL; PROJ may also give a height that is not finite without failing it.
    if (grid_) grid_->TransformPoints(a.x.data(), a.y.data(), n);
    else t_->TransformPoints(a.x.data(), a.y.data(), threeD_ ? a.z.data() : nullptr, n);
    for (size_t i = 0; i < n; ++i) {
        const double z = threeD_ ? a.z[i] : xyz[3 * i + 2];
        if (!(std::isfinite(a.x[i]) && std::isfinite(a.y[i]) && std::isfinite(z)))
            return InvalidArgument("the point " + PointText(xyz + 3 * i) + " does not transform from " + t_->Source().Name() +
                                   " to " + t_->Target().Name());
        out[3 * i] = a.x[i];
        out[3 * i + 1] = a.y[i];
        out[3 * i + 2] = z;
    }
    return Status();
}

std::string LasWkt(const crs::Crs& crs) {
    try {
        std::string wkt1 = crs.ToWkt(crs::WktFlavor::Wkt1Gdal);
        if (!wkt1.empty() && crs::Crs::FromWkt(wkt1).Equivalent(crs)) return wkt1;
    } catch (const crs::CrsError&) {
        // WKT1 cannot express it (PROJ refuses a 3-D projected CRS, for one).
    }
    return crs.ToWkt(crs::WktFlavor::Wkt2_2019);
}

}  // namespace ddbt::copc::build
