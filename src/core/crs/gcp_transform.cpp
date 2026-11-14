// SPDX-License-Identifier: MPL-2.0
//
// Ported from GDAL 3.13.3, MIT licence: the polynomial GCP transformer of alg/gdal_crs.cpp
// (GDALCreateGCPTransformer, GDALGCPTransform and the CRS.C routines from GRASS it is built on),
// the homography of alg/gdal_homography.cpp (GDALGCPsToHomography, GDALInvHomography,
// GDALComposeHomographies, GDALApplyHomography, GDALHomographyTransform), the built-in linear
// solver of alg/gdallinearsystem.cpp, and the choice between them and the antimeridian unwrap of
// GDALCreateGenImgProjTransformer2 in alg/gdaltransformer.cpp. The arithmetic is kept in GDAL's
// order (this file is compiled without floating-point contraction) so that the results are the
// same bits. The copyright notices and the permission notice of the ported code:
//
//   alg/gdal_crs.cpp:
//     CRS.C - Center for Remote Sensing rectification routines, written by Brian J. Buckley,
//     The Center for Remote Sensing, Michigan State University; GCP refinement added by
//     C. F. Stallmann & R. van den Dool (South African National Space Agency).
//     Copyright (c) 1992, Michigan State University
//     Copyright (c) 2008-2013, Even Rouault <even dot rouault at spatialys.com>
//     GDAL transformer wrapper by Frank Warmerdam, warmerdam@pobox.com
//   alg/gdal_homography.cpp:
//     Copyright (c) 2025, Nathan Olson <nathanmolson at gmail dot com>
//   alg/gdallinearsystem.cpp:
//     Copyright (c) 2004, VIZRT Inc.
//     Copyright (c) 2008-2014, Even Rouault <even dot rouault at spatialys.com>
//     Copyright (c) 2019, Martin Franzke <martin dot franzke at telekom dot de>
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

#include "core/crs/gcp_transform.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace ddbt::crs {

namespace {

constexpr int kMaxOrder = 3;
constexpr double kDegreeInRadians = 0.0174532925199433;  // SRS_UA_DEGREE_CONV

// The n x n matrix (n = terms, at most 10) of the CRS.C routines, 1-based like GDAL's M(row, col).
struct CrsMatrix {
    int n = 0;
    std::array<double, 100> v{};
    double& operator()(int row, int col) noexcept { return v[static_cast<size_t>((row - 1) * n + col - 1)]; }
};

enum class CrsResult { Success, NotEnoughPoints, Unsolvable, ParameterError, InternalError };

Status CrsStatus(CrsResult r) {
    switch (r) {
        case CrsResult::NotEnoughPoints:
            return InvalidArgument("Failed to compute GCP transform: Not enough points available");
        case CrsResult::Unsolvable:
            return InvalidArgument("Failed to compute GCP transform: Transform is not solvable");
        case CrsResult::ParameterError:
            return InvalidArgument("Failed to compute GCP transform: Parameter error");
        case CrsResult::InternalError:
        case CrsResult::Success:
            break;
    }
    return InternalError("Failed to compute GCP transform: Internal error");
}

int TermsFor(int order) { return ((order + 1) * (order + 2)) / 2; }

// term(): the x / y term of a polynomial by its 1-based number.
double Term(int nTerm, double e, double n) {
    switch (nTerm) {
        case 1: return 1.0;
        case 2: return e;
        case 3: return n;
        case 4: return e * e;
        case 5: return e * n;
        case 6: return n * n;
        case 7: return e * e * e;
        case 8: return e * e * n;
        case 9: return e * n * n;
        case 10: return n * n * n;
        default: return 0.0;
    }
}

// solvemat(): Gauss-Jordan elimination with partial pivoting, for the two right-hand sides.
CrsResult SolveMat(CrsMatrix& m, double* a, double* b, double* E, double* N) {
    for (int i = 1; i <= m.n; i++) {
        const int j = i;
        double pivot = m(i, j);
        int imark = i;
        for (int i2 = i + 1; i2 <= m.n; i2++) {
            if (std::fabs(m(i2, j)) > std::fabs(pivot)) {
                pivot = m(i2, j);
                imark = i2;
            }
        }
        if (pivot == 0.0) return CrsResult::Unsolvable;
        if (imark != i) {
            for (int j2 = 1; j2 <= m.n; j2++) std::swap(m(imark, j2), m(i, j2));
            std::swap(a[imark - 1], a[i - 1]);
            std::swap(b[imark - 1], b[i - 1]);
        }
        for (int i2 = 1; i2 <= m.n; i2++) {
            if (i2 != i) {
                const double factor = m(i2, j) / pivot;
                for (int j2 = j; j2 <= m.n; j2++) m(i2, j2) -= factor * m(i, j2);
                a[i2 - 1] -= factor * a[i - 1];
                b[i2 - 1] -= factor * b[i - 1];
            }
        }
    }
    for (int i = 1; i <= m.n; i++) {
        E[i - 1] = a[i - 1] / m(i, i);
        N[i - 1] = b[i - 1] / m(i, i);
    }
    return CrsResult::Success;
}

// The control points of calccoef(): (e1, n1) -> (e2, n2), all active.
struct ControlPoints {
    size_t count = 0;
    const double* e1 = nullptr;
    const double* n1 = nullptr;
    const double* e2 = nullptr;
    const double* n2 = nullptr;
};

// exactdet(): as many points as terms, the system itself.
CrsResult ExactDet(const ControlPoints& cp, CrsMatrix& m, double xMean, double yMean, double* a, double* b,
                   double* E, double* N) {
    int currow = 1;
    for (size_t p = 0; p < cp.count; p++) {
        for (int j = 1; j <= m.n; j++) m(currow, j) = Term(j, cp.e1[p] - xMean, cp.n1[p] - yMean);
        a[currow - 1] = cp.e2[p];
        b[currow - 1] = cp.n2[p];
        currow++;
    }
    if (currow - 1 != m.n) return CrsResult::InternalError;
    return SolveMat(m, a, b, E, N);
}

// calcls(): more points than terms, the normal equations of the least squares fit.
CrsResult CalcLs(const ControlPoints& cp, CrsMatrix& m, double xMean, double yMean, double* a, double* b,
                 double* E, double* N) {
    for (int i = 1; i <= m.n; i++) {
        for (int j = i; j <= m.n; j++) m(i, j) = 0.0;
        a[i - 1] = b[i - 1] = 0.0;
    }
    size_t numactive = 0;
    for (size_t p = 0; p < cp.count; p++) {
        numactive++;
        const double e = cp.e1[p] - xMean;
        const double n = cp.n1[p] - yMean;
        for (int i = 1; i <= m.n; i++) {
            for (int j = i; j <= m.n; j++) m(i, j) += Term(i, e, n) * Term(j, e, n);
            a[i - 1] += cp.e2[p] * Term(i, e, n);
            b[i - 1] += cp.n2[p] * Term(i, e, n);
        }
    }
    if (numactive <= static_cast<size_t>(m.n)) return CrsResult::InternalError;
    for (int i = 2; i <= m.n; i++) {
        for (int j = 1; j < i; j++) m(i, j) = m(j, i);
    }
    return SolveMat(m, a, b, E, N);
}

// calccoef(): the coefficients of one direction.
CrsResult CalcCoef(const ControlPoints& cp, double xMean, double yMean, double* E, double* N, int order) {
    CrsMatrix m;
    m.n = TermsFor(order);
    if (cp.count < static_cast<size_t>(m.n)) return CrsResult::NotEnoughPoints;
    std::array<double, 10> a{}, b{};
    if (cp.count == static_cast<size_t>(m.n)) return ExactDet(cp, m, xMean, yMean, a.data(), b.data(), E, N);
    return CalcLs(cp, m, xMean, yMean, a.data(), b.data(), E, N);
}

// CRS_georef(): evaluates a polynomial at (e1, n1), relative to the means.
void CrsGeoref(double e1, double n1, double* e, double* n, const double* E, const double* N, int order) {
    switch (order) {
        case 1:
            *e = E[0] + E[1] * e1 + E[2] * n1;
            *n = N[0] + N[1] * e1 + N[2] * n1;
            break;
        case 2: {
            const double e2 = e1 * e1;
            const double n2 = n1 * n1;
            const double en = e1 * n1;
            *e = E[0] + E[1] * e1 + E[2] * n1 + E[3] * e2 + E[4] * en + E[5] * n2;
            *n = N[0] + N[1] * e1 + N[2] * n1 + N[3] * e2 + N[4] * en + N[5] * n2;
            break;
        }
        default: {
            const double e2 = e1 * e1;
            const double en = e1 * n1;
            const double n2 = n1 * n1;
            const double e3 = e1 * e2;
            const double e2n = e2 * n1;
            const double en2 = e1 * n2;
            const double n3 = n1 * n2;
            *e = E[0] + E[1] * e1 + E[2] * n1 + E[3] * e2 + E[4] * en + E[5] * n2 + E[6] * e3 + E[7] * e2n +
                 E[8] * en2 + E[9] * n3;
            *n = N[0] + N[1] * e1 + N[2] * n1 + N[3] * e2 + N[4] * en + N[5] * n2 + N[6] * e3 + N[7] * e2n +
                 N[8] * en2 + N[9] * n3;
            break;
        }
    }
}

// GDALApplyHomography().
bool ApplyHomography(const double* h, double pixel, double line, double* geoX, double* geoY) {
    const double w = h[6] + pixel * h[7] + line * h[8];
    if (std::abs(w) < 1.0e-15) return false;
    const double wx = h[0] + pixel * h[1] + line * h[2];
    const double wy = h[3] + pixel * h[4] + line * h[5];
    *geoX = wx / w;
    *geoY = wy / w;
    return true;
}

// GDALInvHomography().
bool InvHomography(const double* hIn, double* hOut) {
    // No rotation: no determinant (and its rounding).
    if (hIn[2] == 0.0 && hIn[4] == 0.0 && hIn[1] != 0.0 && hIn[5] != 0.0 && hIn[7] == 0.0 && hIn[8] == 0.0 &&
        hIn[6] != 0.0) {
        hOut[0] = -hIn[0] / hIn[1] / hIn[6];
        hOut[1] = 1.0 / hIn[1];
        hOut[2] = 0.0;
        hOut[3] = -hIn[3] / hIn[5] / hIn[6];
        hOut[4] = 0.0;
        hOut[5] = 1.0 / hIn[5];
        hOut[6] = 1.0 / hIn[6];
        hOut[7] = 0.0;
        hOut[8] = 0.0;
        return true;
    }
    const double det = hIn[1] * hIn[5] * hIn[6] - hIn[2] * hIn[4] * hIn[6] + hIn[2] * hIn[3] * hIn[7] -
                       hIn[0] * hIn[5] * hIn[7] + hIn[0] * hIn[4] * hIn[8] - hIn[1] * hIn[3] * hIn[8];
    const double magnitude =
        std::max(std::max(std::fabs(hIn[1]), std::fabs(hIn[2])), std::max(std::fabs(hIn[4]), std::fabs(hIn[5])));
    if (std::fabs(det) <= 1e-10 * magnitude * magnitude) return false;
    const double invDet = 1.0 / det;
    hOut[1] = (hIn[5] * hIn[6] - hIn[3] * hIn[8]) * invDet;
    hOut[4] = (hIn[3] * hIn[7] - hIn[4] * hIn[6]) * invDet;
    hOut[7] = (hIn[4] * hIn[8] - hIn[5] * hIn[7]) * invDet;
    hOut[2] = (hIn[0] * hIn[8] - hIn[2] * hIn[6]) * invDet;
    hOut[5] = (hIn[1] * hIn[6] - hIn[0] * hIn[7]) * invDet;
    hOut[8] = (hIn[2] * hIn[7] - hIn[1] * hIn[8]) * invDet;
    hOut[0] = (hIn[2] * hIn[3] - hIn[0] * hIn[5]) * invDet;
    hOut[3] = (hIn[0] * hIn[4] - hIn[1] * hIn[3]) * invDet;
    hOut[6] = (hIn[1] * hIn[5] - hIn[2] * hIn[4]) * invDet;
    return true;
}

// GDALComposeHomographies(): h1, then h2.
void ComposeHomographies(const double* h1, const double* h2, double* out) {
    double w[9];
    w[1] = h2[1] * h1[1] + h2[2] * h1[4] + h2[0] * h1[7];
    w[2] = h2[1] * h1[2] + h2[2] * h1[5] + h2[0] * h1[8];
    w[0] = h2[1] * h1[0] + h2[2] * h1[3] + h2[0] * h1[6];
    w[4] = h2[4] * h1[1] + h2[5] * h1[4] + h2[3] * h1[7];
    w[5] = h2[4] * h1[2] + h2[5] * h1[5] + h2[3] * h1[8];
    w[3] = h2[4] * h1[0] + h2[5] * h1[3] + h2[3] * h1[6];
    w[7] = h2[7] * h1[1] + h2[8] * h1[4] + h2[6] * h1[7];
    w[8] = h2[7] * h1[2] + h2[8] * h1[5] + h2[6] * h1[8];
    w[6] = h2[7] * h1[0] + h2[8] * h1[3] + h2[6] * h1[6];
    std::copy(w, w + 9, out);
}

// The built-in solver of GDALLinearSystemSolve() (LU decomposition with partial pivoting, eps 0)
// for the 9 x 9 system of the homography and one right-hand side. GDALMatrix is column-major;
// only the indices matter for the arithmetic.
bool SolveLinearSystem9(std::array<double, 81>& A, const std::array<double, 9>& rhs, std::array<double, 9>& x) {
    constexpr int m = 9;
    auto at = [&A](int row, int col) -> double& { return A[static_cast<size_t>(row + col * m)]; };
    std::array<int, m> perm{};
    for (int r = 0; r < m; ++r) perm[static_cast<size_t>(r)] = r;
    for (int step = 0; step < m - 1; ++step) {
        int iMax = step;
        double dMax = std::abs(at(step, step));
        for (int i = step + 1; i < m; ++i) {
            if (std::abs(at(i, step)) > dMax) {
                iMax = i;
                dMax = std::abs(at(i, step));
            }
        }
        if (dMax <= 0) return false;
        if (iMax != step) {
            std::swap(perm[static_cast<size_t>(iMax)], perm[static_cast<size_t>(step)]);
            for (int c = 0; c < m; ++c) std::swap(at(iMax, c), at(step, c));
        }
        for (int r = step + 1; r < m; ++r) at(r, step) /= at(step, step);
        for (int c = step + 1; c < m; ++c) {
            for (int r = step + 1; r < m; ++r) at(r, c) -= at(r, step) * at(step, c);
        }
    }
    for (int r = 0; r < m; ++r) {
        double& xr = x[static_cast<size_t>(r)];
        xr = rhs[static_cast<size_t>(perm[static_cast<size_t>(r)])];
        for (int k = 0; k < r; ++k) xr -= at(r, k) * x[static_cast<size_t>(k)];
    }
    for (int r = m - 1; r >= 0; --r) {
        double& xr = x[static_cast<size_t>(r)];
        for (int k = r + 1; k < m; ++k) xr -= at(r, k) * x[static_cast<size_t>(k)];
        xr /= at(r, r);
    }
    return true;
}

// GDALGCPsToHomography() for 4 GCPs or more.
Status GcpsToHomography(std::span<const Gcp> gcps, double* homography) {
    double minPixel = gcps[0].pixel, maxPixel = gcps[0].pixel;
    double minLine = gcps[0].line, maxLine = gcps[0].line;
    double minGeoX = gcps[0].x, maxGeoX = gcps[0].x;
    double minGeoY = gcps[0].y, maxGeoY = gcps[0].y;
    for (size_t i = 1; i < gcps.size(); ++i) {
        minPixel = std::min(minPixel, gcps[i].pixel);
        maxPixel = std::max(maxPixel, gcps[i].pixel);
        minLine = std::min(minLine, gcps[i].line);
        maxLine = std::max(maxLine, gcps[i].line);
        minGeoX = std::min(minGeoX, gcps[i].x);
        maxGeoX = std::max(maxGeoX, gcps[i].x);
        minGeoY = std::min(minGeoY, gcps[i].y);
        maxGeoY = std::max(maxGeoY, gcps[i].y);
    }
    constexpr double kEpsilon = 1.0e-12;
    if (std::abs(maxPixel - minPixel) < kEpsilon || std::abs(maxLine - minLine) < kEpsilon ||
        std::abs(maxGeoX - minGeoX) < kEpsilon || std::abs(maxGeoY - minGeoY) < kEpsilon) {
        return InvalidArgument("GDALGCPsToHomography() failed: GCPs degenerate in at least one dimension.");
    }

    double plNormalize[9], geoNormalize[9];
    plNormalize[0] = -minPixel / (maxPixel - minPixel);
    plNormalize[1] = 1.0 / (maxPixel - minPixel);
    plNormalize[2] = 0.0;
    plNormalize[3] = -minLine / (maxLine - minLine);
    plNormalize[4] = 0.0;
    plNormalize[5] = 1.0 / (maxLine - minLine);
    plNormalize[6] = 1.0;
    plNormalize[7] = 0.0;
    plNormalize[8] = 0.0;

    geoNormalize[0] = -minGeoX / (maxGeoX - minGeoX);
    geoNormalize[1] = 1.0 / (maxGeoX - minGeoX);
    geoNormalize[2] = 0.0;
    geoNormalize[3] = -minGeoY / (maxGeoY - minGeoY);
    geoNormalize[4] = 0.0;
    geoNormalize[5] = 1.0 / (maxGeoY - minGeoY);
    geoNormalize[6] = 1.0;
    geoNormalize[7] = 0.0;
    geoNormalize[8] = 0.0;

    double invGeoNormalize[9] = {};
    if (!InvHomography(geoNormalize, invGeoNormalize)) {
        return InvalidArgument("GDALGCPsToHomography() failed: GDALInvHomography() failed");
    }

    // Least squares fit of the normalised homography; rank(AtA) = 8, so the equation
    // h_normalized[6] = 1 is added.
    std::array<double, 81> AtA{};
    std::array<double, 9> rhs{};
    auto ata = [&AtA](int row, int col) -> double& { return AtA[static_cast<size_t>(row + col * 9)]; };
    rhs[6] = 1;
    ata(6, 6) = 1;
    for (size_t i = 0; i < gcps.size(); ++i) {
        double pixel = 0, line = 0, geox = 0, geoy = 0;
        if (!ApplyHomography(plNormalize, gcps[i].pixel, gcps[i].line, &pixel, &line) ||
            !ApplyHomography(geoNormalize, gcps[i].x, gcps[i].y, &geox, &geoy)) {
            return InvalidArgument("GDALGCPsToHomography() failed: GDALApplyHomography() failed on GCP " +
                                   std::to_string(i) + ".");
        }
        const double Ax[] = {1, pixel, line, 0, 0, 0, -geox, -geox * pixel, -geox * line};
        const double Ay[] = {0, 0, 0, 1, pixel, line, -geoy, -geoy * pixel, -geoy * line};
        for (int j = 0; j < 9; j++) {
            for (int k = j; k < 9; k++) ata(j, k) += Ax[j] * Ax[k] + Ay[j] * Ay[k];
        }
    }
    for (int j = 0; j < 9; j++) {
        for (int k = 0; k < j; k++) ata(j, k) = ata(k, j);
    }

    std::array<double, 9> hNormalized{};
    if (!SolveLinearSystem9(AtA, rhs, hNormalized)) {
        return InvalidArgument("GDALGCPsToHomography() failed: GDALLinearSystemSolve() failed");
    }
    if (std::abs(hNormalized[6]) < 1.0e-15) {
        return InvalidArgument("GDALGCPsToHomography() failed: h_normalized(6, 0) not zero");
    }

    // The unit square must map to a convex quadrilateral.
    double x[4] = {0, 1, 1, 0};
    double y[4] = {0, 0, 1, 1};
    for (int i = 0; i < 4; i++) {
        if (!ApplyHomography(hNormalized.data(), x[i], y[i], &x[i], &y[i])) {
            return InvalidArgument("GDALGCPsToHomography() failed: a corner of the GCPs' extent maps to infinity");
        }
    }
    for (int i = 3; i >= 0; i--) {
        x[i] -= x[0];
        y[i] -= y[0];
    }
    const double cross12 = x[1] * y[2] - x[2] * y[1];
    const double cross23 = x[2] * y[3] - x[3] * y[2];
    if (cross12 * cross23 <= 0.0) {
        return InvalidArgument("GDALGCPsToHomography() failed: cross12 * cross23 <= 0.0");
    }

    double h1p2[9] = {};
    ComposeHomographies(plNormalize, hNormalized.data(), h1p2);
    ComposeHomographies(h1p2, invGeoNormalize, homography);
    return Status::Ok();
}

// GDALGCPAntimeridianUnwrap() with GCP_ANTIMERIDIAN_UNWRAP=AUTO: GCPs in a geographic CRS in
// degrees whose longitudes are all within [-180, -170] or [170, 180].
bool UnwrapAntimeridian(std::vector<Gcp>& gcps, const Crs& crs) {
    if (!crs.Valid() || !crs.IsHorizontallyGeographic() ||
        !(std::fabs(crs.HorizontalUnitFactor() - kDegreeInRadians) < 1e-8)) {
        return false;
    }
    for (const Gcp& g : gcps) {
        const double longAbs = std::fabs(g.x);
        if (longAbs > 180 || longAbs < 170) return false;
    }
    bool done = false;
    for (Gcp& g : gcps) {
        if (g.x < 0) {
            done = true;
            g.x += 360;
        }
    }
    return done;
}

bool Finite(const std::array<double, 9>& h) {
    return std::all_of(h.begin(), h.end(), [](double v) { return std::isfinite(v); });
}

}  // namespace

Expected<GcpTransform> GcpTransform::Create(std::span<const Gcp> gcpsIn, const Crs& gcpCrs,
                                            const GcpTransformOptions& options) {
    const size_t count = gcpsIn.size();
    // GDALCreateGenImgProjTransformer2: the method.
    bool homography = false;
    switch (options.method) {
        case GcpMethod::Auto:
            homography = count >= 4 && count < 6;
            if (!homography && options.maxOrder < 0) {
                return Unsupported("thin plate spline GCP transforms (a negative GCP order) are not supported");
            }
            break;
        case GcpMethod::Homography: homography = true; break;
        case GcpMethod::Polynomial: break;
    }
    if (count == 0) return InvalidArgument("Failed to compute GCP transform: no GCP");

    std::vector<Gcp> gcps(gcpsIn.begin(), gcpsIn.end());
    GcpTransform t;
    t.unwrapped_ = UnwrapAntimeridian(gcps, gcpCrs);

    if (homography) {
        // GDALGCPsToHomography falls back to an exact geotransform fit below 4 GCPs; not ported.
        if (count < 4) return Unsupported("a GCP homography needs at least 4 GCPs");
        t.kind_ = Kind::Homography;
        DDBT_TRY(GcpsToHomography(gcps, t.forward_.data()));
        if (!InvHomography(t.forward_.data(), t.inverse_.data())) {
            return InvalidArgument("GDALCreateHomographyTransformer() failed, because GDALInvHomography() failed");
        }
        // GDAL's built-in solver does not test the last pivot: a singular system can give
        // coefficients that are not finite, and a transformer whose every point is NaN.
        if (!Finite(t.forward_) || !Finite(t.inverse_)) {
            return InvalidArgument("GCP homography: the GCPs give a singular system");
        }
        return t;
    }

    // GDALCreateGCPTransformerEx (no refinement, not reversed).
    int order = std::max(0, options.maxOrder);
    if (order == 0) order = count >= 6 ? 2 : 1;
    t.kind_ = Kind::Polynomial;
    t.order_ = order;
    if (gcps.size() == 2 && order == 1 && gcps[0].x != gcps[1].x && gcps[0].y != gcps[1].y) {
        // The 2 GCPs are opposite corners of a rectangle: a third corner.
        Gcp g;
        g.x = gcps[1].x;
        g.y = gcps[0].y;
        g.pixel = gcps[1].pixel;
        g.line = gcps[0].line;
        gcps.push_back(g);
    }

    const size_t n = gcps.size();
    std::vector<double> geoX(n), geoY(n), rasterX(n), rasterY(n);
    double x1Sum = 0, y1Sum = 0, x2Sum = 0, y2Sum = 0;
    for (size_t i = 0; i < n; i++) {
        geoX[i] = gcps[i].x;
        geoY[i] = gcps[i].y;
        rasterX[i] = gcps[i].pixel;
        rasterY[i] = gcps[i].line;
        x1Sum += gcps[i].pixel;
        y1Sum += gcps[i].line;
        x2Sum += gcps[i].x;
        y2Sum += gcps[i].y;
    }
    // GDAL divides by an int count.
    const double dn = static_cast<double>(static_cast<int>(n));
    t.pixelMean_ = x1Sum / dn;
    t.lineMean_ = y1Sum / dn;
    t.xMean_ = x2Sum / dn;
    t.yMean_ = y2Sum / dn;

    // CRS_compute_georef_equations().
    if (order < 1 || order > kMaxOrder) return CrsStatus(CrsResult::ParameterError);
    const ControlPoints forward{n, rasterX.data(), rasterY.data(), geoX.data(), geoY.data()};
    CrsResult r = CalcCoef(forward, t.pixelMean_, t.lineMean_, t.toGeoX_.data(), t.toGeoY_.data(), order);
    if (r != CrsResult::Success) return CrsStatus(r);
    const ControlPoints backward{n, geoX.data(), geoY.data(), rasterX.data(), rasterY.data()};
    r = CalcCoef(backward, t.xMean_, t.yMean_, t.fromGeoX_.data(), t.fromGeoY_.data(), order);
    if (r != CrsResult::Success) return CrsStatus(r);
    return t;
}

void GcpTransform::PolynomialPoints(double* xs, double* ys, size_t n, Direction dir, int* success,
                                    size_t& failed) const noexcept {
    const bool toPixel = dir == Direction::Inverse;
    const double meanX = toPixel ? xMean_ : pixelMean_;
    const double meanY = toPixel ? yMean_ : lineMean_;
    const double* E = toPixel ? fromGeoX_.data() : toGeoX_.data();
    const double* N = toPixel ? fromGeoY_.data() : toGeoY_.data();
    for (size_t i = 0; i < n; i++) {
        if (xs[i] == HUGE_VAL || ys[i] == HUGE_VAL) {
            xs[i] = ys[i] = HUGE_VAL;
            if (success) success[i] = 0;
            ++failed;
            continue;
        }
        CrsGeoref(xs[i] - meanX, ys[i] - meanY, xs + i, ys + i, E, N, order_);
        if (success) success[i] = 1;
    }
}

void GcpTransform::HomographyPoints(double* xs, double* ys, size_t n, Direction dir, int* success,
                                    size_t& failed) const noexcept {
    const double* h = dir == Direction::Inverse ? inverse_.data() : forward_.data();
    for (size_t i = 0; i < n; i++) {
        if (xs[i] == HUGE_VAL || ys[i] == HUGE_VAL || !ApplyHomography(h, xs[i], ys[i], xs + i, ys + i)) {
            xs[i] = ys[i] = HUGE_VAL;
            if (success) success[i] = 0;
            ++failed;
            continue;
        }
        if (success) success[i] = 1;
    }
}

size_t GcpTransform::TransformPoints(double* xs, double* ys, size_t n, Direction dir, int* success) const noexcept {
    size_t failed = 0;
    if (kind_ == Kind::Homography) {
        HomographyPoints(xs, ys, n, dir, success, failed);
    } else {
        PolynomialPoints(xs, ys, n, dir, success, failed);
    }
    return failed;
}

}  // namespace ddbt::crs
