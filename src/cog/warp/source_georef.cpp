// SPDX-License-Identifier: MPL-2.0
//
// Ported from GDAL 3.13.3, MIT licence: GDALInvGeoTransform of alg/gdaltransformer.cpp
// (GdalInvGeoTransform). The copyright notices and the permission notice of the ported code:
//
//   alg/gdaltransformer.cpp:
//     Copyright (c) 2002, i3 - information integration and imaging, Fort Collin, CO
//     Copyright (c) 2008-2013, Even Rouault <even dot rouault at spatialys.com>
//     Copyright (c) 2021, CLS
//
//   Permission is hereby granted, free of charge, to any person obtaining a copy of this
//   software and associated documentation files (the "Software"), to deal in the Software
//   without restriction, including without limitation the rights to use, copy, modify, merge,
//   publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons
//   to whom the Software is furnished to do so, subject to the following conditions:
//
//   The above copyright notice and this permission notice shall be included in all copies or
//   substantial portions of the Software.
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//   INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
//   PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
//   FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
//   OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//   DEALINGS IN THE SOFTWARE.
//
// Compiled without floating-point contraction (sources.cmake): the geotransforms must round as
// GDAL's.
#include "cog/warp/source_georef.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ddbt::cog::warp {

bool InvertGeoTransform(const std::array<double, 6>& gt, std::array<double, 6>& inv) {
    const double det = gt[1] * gt[5] - gt[2] * gt[4];
    if (std::abs(det) < 1e-15) return false;
    const double invDet = 1.0 / det;
    inv[1] = gt[5] * invDet;
    inv[4] = -gt[4] * invDet;
    inv[2] = -gt[2] * invDet;
    inv[5] = gt[1] * invDet;
    inv[0] = (gt[2] * gt[3] - gt[0] * gt[5]) * invDet;
    inv[3] = (-gt[1] * gt[3] + gt[0] * gt[4]) * invDet;
    return true;
}

std::array<double, 6> GdalInvGeoTransform(const std::array<double, 6>& gt) {
    std::array<double, 6> inv{};
    if (gt[2] == 0.0 && gt[4] == 0.0 && gt[1] != 0.0 && gt[5] != 0.0) {
        inv[0] = -gt[0] / gt[1];
        inv[1] = 1.0 / gt[1];
        inv[2] = 0.0;
        inv[3] = -gt[3] / gt[5];
        inv[4] = 0.0;
        inv[5] = 1.0 / gt[5];
        return inv;
    }
    const double det = gt[1] * gt[5] - gt[2] * gt[4];
    const double magnitude = std::max(std::max(std::fabs(gt[1]), std::fabs(gt[2])), std::max(std::fabs(gt[4]), std::fabs(gt[5])));
    if (std::fabs(det) <= 1e-10 * magnitude * magnitude) return inv;
    const double invDet = 1.0 / det;
    inv[1] = gt[5] * invDet;
    inv[4] = -gt[4] * invDet;
    inv[2] = -gt[2] * invDet;
    inv[5] = gt[1] * invDet;
    inv[0] = (gt[2] * gt[3] - gt[0] * gt[5]) * invDet;
    inv[3] = (-gt[1] * gt[3] + gt[0] * gt[4]) * invDet;
    return inv;
}

namespace {

// GDALGenImgProjTransform's affine step: points an earlier step failed (HUGE_VAL) are left alone.
size_t Affine(const std::array<double, 6>& a, double* xs, double* ys, size_t n) noexcept {
    size_t failed = 0;
    for (size_t i = 0; i < n; ++i) {
        if (xs[i] == HUGE_VAL || ys[i] == HUGE_VAL) {
            xs[i] = ys[i] = HUGE_VAL;
            ++failed;
            continue;
        }
        const double x = a[0] + xs[i] * a[1] + ys[i] * a[2];
        const double y = a[3] + xs[i] * a[4] + ys[i] * a[5];
        xs[i] = x;
        ys[i] = y;
    }
    return failed;
}

}  // namespace

Expected<SourceGeoreference> SourceGeoreference::FromGeoTransform(const std::array<double, 6>& gt) {
    SourceGeoreference g;
    if (!InvertGeoTransform(gt, g.affine_)) return InvalidArgument("Cannot invert geotransform");
    g.gt_ = gt;
    return g;
}

Expected<SourceGeoreference> SourceGeoreference::FromGcps(std::span<const crs::Gcp> gcps, const crs::Crs& gcpCrs) {
    auto t = crs::GcpTransform::Create(gcps, gcpCrs);
    if (!t.ok()) return t.status();
    SourceGeoreference g;
    g.gcp_ = std::move(*t);
    return g;
}

Expected<SourceGeoreference> SourceGeoreference::Of(const tiff::Georeferencing& georef) {
    if (georef.geoTransform) return FromGeoTransform(*georef.geoTransform);
    if (georef.gcps.empty())
        return InvalidArgument("There is no affine transformation and no GCPs");
    std::vector<crs::Gcp> gcps;
    gcps.reserve(georef.gcps.size());
    for (const tiff::GroundControlPoint& p : georef.gcps) gcps.push_back({p.pixel, p.line, p.x, p.y, p.z});
    return FromGcps(gcps, georef.crs ? *georef.crs : crs::Crs());
}

size_t SourceGeoreference::PixelToCrs(double* xs, double* ys, size_t n) const noexcept {
    if (gcp_) return gcp_->TransformPoints(xs, ys, n, crs::Direction::Forward);
    return Affine(*gt_, xs, ys, n);
}

size_t SourceGeoreference::CrsToPixel(double* xs, double* ys, size_t n) const noexcept {
    if (gcp_) return gcp_->TransformPoints(xs, ys, n, crs::Direction::Inverse);
    return Affine(affine_, xs, ys, n);
}

}  // namespace ddbt::cog::warp
