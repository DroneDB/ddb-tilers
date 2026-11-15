// SPDX-License-Identifier: MPL-2.0
#include "core/crs/transform_bounds.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>
#include <vector>

#include "core/crs/transform.h"

namespace ddbt::crs {

namespace {

constexpr double kDegree = 0.0174532925199433;

// The values of the points that transformed (GDAL's simple_min / simple_max)
double SimpleMin(const std::vector<double>& v, const std::vector<char>& ok) {
    double m = HUGE_VAL;
    for (size_t i = 0; i < v.size(); ++i)
        if (ok[i] && (v[i] < m || m == HUGE_VAL)) m = v[i];
    return m;
}

double SimpleMax(const std::vector<double>& v, const std::vector<char>& ok) {
    double m = HUGE_VAL;
    for (size_t i = 0; i < v.size(); ++i)
        if (ok[i] && (v[i] > m || m == HUGE_VAL)) m = v[i];
    return m;
}

// The nearest point before i that transformed, around the ring
size_t Previous(size_t i, const std::vector<char>& ok) {
    const size_t n = ok.size();
    size_t p = i == 0 ? n - 1 : i - 1;
    while (!ok[p] && p != i) p = p == 0 ? n - 1 : p - 1;
    return p;
}

// GDAL's antimeridian_min: a jump of 200 degrees or more between neighbours of the ring crosses
// the antimeridian; crossed twice, the west bound is the smallest longitude east of it
double AntimeridianMin(const std::vector<double>& v, const std::vector<char>& ok) {
    double positiveMin = HUGE_VAL, minValue = HUGE_VAL;
    int crossed = 0;
    bool positive = false;
    for (size_t i = 0; i < v.size(); ++i) {
        if (!ok[i]) continue;
        const double delta = v[Previous(i, ok)] - v[i];
        if (delta >= 200 && delta != HUGE_VAL) {
            if (crossed == 0) positiveMin = minValue;
            ++crossed;
            positive = false;
        } else if (delta <= -200 && delta != HUGE_VAL) {
            if (crossed == 0) positiveMin = v[i];
            ++crossed;
            positive = true;
        }
        if (positive && v[i] < positiveMin) positiveMin = v[i];
        if (v[i] < minValue) minValue = v[i];
    }
    if (crossed == 2) return positiveMin;
    if (crossed == 4) return -180;
    return minValue;
}

double AntimeridianMax(const std::vector<double>& v, const std::vector<char>& ok) {
    double negativeMax = -HUGE_VAL, maxValue = -HUGE_VAL;
    int crossed = 0;
    bool negative = false;
    for (size_t i = 0; i < v.size(); ++i) {
        if (!ok[i]) continue;
        const double delta = v[Previous(i, ok)] - v[i];
        if (delta >= 200 && delta != HUGE_VAL) {
            if (crossed == 0) negativeMax = v[i];
            ++crossed;
            negative = true;
        } else if (delta <= -200 && delta != HUGE_VAL) {
            if (crossed == 0) negativeMax = maxValue;
            negative = false;
            ++crossed;
        }
        if (negative && (v[i] > negativeMax || negativeMax == HUGE_VAL)) negativeMax = v[i];
        if (v[i] > maxValue || maxValue == HUGE_VAL) maxValue = v[i];
    }
    if (crossed == 2) return negativeMax;
    if (crossed == 4) return 180;
    return maxValue;
}

bool InDegrees(const Crs& c) {
    return c.IsHorizontallyGeographic() && std::fabs(c.HorizontalUnitFactor() - kDegree) < 1e-8;
}

// The special points of a projected target (its poles and the meridian opposite its central
// one) that fall in the box, added to the bounds as GDAL adds them
void AddProjectedSpecialPoints(const Crs& from, const Crs& to, const Crs::ProjectionMethod& projection,
                               const Transform& t, const Bounds& box, Bounds& out) {
    std::shared_ptr<const Transform> back;
    try {
        back = TransformCache::Global().Get(to.GeographicBase(), from);
    } catch (const CrsError&) {
        return;
    }
    const double lon0 = projection.centralMeridian, lat0 = projection.latitudeOfOrigin;
    double absPoleLat = 90;
    if (lon0 == 0) {
        std::string name = projection.name;
        std::transform(name.begin(), name.end(), name.begin(), [](unsigned char ch) { return std::tolower(ch); });
        if (name == "mercator_1sp") absPoleLat = 85.0511287798066;
    }
    constexpr double kEps = 1e-8;
    // A point of the target's geographic CRS that is in the box and transforms
    const auto add = [&](double x, double y) {
        if (!back->TransformPoint(x, y) || !(x >= box.minX && y >= box.minY && x <= box.maxX && y <= box.maxY) ||
            !t.TransformPoint(x, y))
            return false;
        out.minX = std::min(out.minX, x);
        out.minY = std::min(out.minY, y);
        out.maxX = std::max(out.maxX, x);
        out.maxY = std::max(out.maxY, y);
        return true;
    };
    double signedPoleLat = 0;
    bool includesPole = false;
    for (int sign = -1; sign <= 1; sign += 2) {
        if (add(lon0, sign * (absPoleLat - kEps))) {
            signedPoleLat = sign * absPoleLat;
            includesPole = true;
        }
    }
    const auto at180 = [&](double lat) {
        for (int sign = -1; sign <= 1; sign += 2) add(std::fmod(lon0 + sign * (180 - kEps) + 180, 360) - 180, lat);
    };
    if (lon0 != 0) at180(lat0);
    if (includesPole && lat0 != signedPoleLat) at180(signedPoleLat);
}

}  // namespace

std::optional<Bounds> TransformBounds(const Crs& from, const Crs& to, const Bounds& box, int densify) {
    if (from.Equivalent(to)) return box;
    if (densify < 0 || densify > 10000) return std::nullopt;
    const bool degreeInput = InDegrees(from), degreeOutput = InDegrees(to);
    if (degreeOutput && densify < 2) return std::nullopt;
    if (degreeInput && box.maxY < box.minY) return std::nullopt;  // latitude max < latitude min
    TransformOptions options;
    options.areaOfInterest = Transform::AreaOfInterestForBox(from, box.minX, box.minY, box.maxX, box.maxY);
    const std::shared_ptr<const Transform> t = TransformCache::Global().Get(from, to, options);
    const int side = densify + 1;
    const size_t n = static_cast<size_t>(side) * 4;
    // A box of longitudes whose east is under its west crosses the antimeridian
    const double dx = (degreeInput && box.maxX < box.minX ? box.maxX - box.minX + 360.0 : box.maxX - box.minX) / side;
    const double dy = (box.maxY - box.minY) / side;
    std::vector<double> xs(n), ys(n);
    for (int i = 0; i < side; ++i) {
        const size_t k = static_cast<size_t>(i), s = static_cast<size_t>(side);
        ys[k] = box.maxY - i * dy;
        xs[k] = box.minX;
        ys[k + s] = box.minY;
        xs[k + s] = box.minX + i * dx;
        ys[k + 2 * s] = box.minY + i * dy;
        xs[k + 2 * s] = box.maxX;
        ys[k + 3 * s] = box.maxY;
        xs[k + 3 * s] = box.maxX - i * dx;
    }
    // The poles of a geographic target in the box (sent back through the same transformation)
    const auto poleIn = [&](double lat) {
        double x = 0, y = lat;
        return t->TransformPoint(x, y, nullptr, Direction::Inverse) && box.minX < x && x < box.maxX &&
               box.maxY > y && y > box.minY;
    };
    const bool northPole = degreeOutput && poleIn(90);
    const bool southPole = degreeOutput && poleIn(-90);
    t->TransformPoints(xs.data(), ys.data(), nullptr, n);
    std::vector<char> ok(n);
    bool any = false;
    for (size_t i = 0; i < n; ++i) {
        ok[i] = xs[i] != HUGE_VAL && ys[i] != HUGE_VAL;
        any = any || ok[i];
    }
    if (!any) return std::nullopt;
    Bounds out;
    if (!degreeOutput) {
        out = {SimpleMin(xs, ok), SimpleMin(ys, ok), SimpleMax(xs, ok), SimpleMax(ys, ok)};
        if (const std::optional<Crs::ProjectionMethod> projection = to.Projection())
            AddProjectedSpecialPoints(from, to, *projection, *t, box, out);
    } else if (northPole) {
        out = {-180, SimpleMin(ys, ok), 180, 90};
    } else if (southPole) {
        out = {-180, -90, 180, SimpleMax(ys, ok)};
    } else {
        out = {AntimeridianMin(xs, ok), SimpleMin(ys, ok), AntimeridianMax(xs, ok), SimpleMax(ys, ok)};
    }
    if (out.minX == HUGE_VAL || out.minY == HUGE_VAL || out.maxX == HUGE_VAL || out.maxY == HUGE_VAL)
        return std::nullopt;
    return out;
}

}  // namespace ddbt::crs
