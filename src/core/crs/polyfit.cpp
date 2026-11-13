// SPDX-License-Identifier: MPL-2.0

#include "core/crs/polyfit.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ddbt::crs {

namespace {

// Monomials of degree <= 3 in the order of PolyFit::CoeffsU.
inline void Monomials(double X, double Y, double* t) noexcept {
    t[0] = 1.0;
    t[1] = X;
    t[2] = Y;
    t[3] = X * X;
    t[4] = X * Y;
    t[5] = Y * Y;
    t[6] = t[3] * X;
    t[7] = t[3] * Y;
    t[8] = X * t[5];
    t[9] = t[5] * Y;
}

// Solves min |A c - b| for two right-hand sides by Householder QR. A is m x k, column-major.
// Returns false if A is rank deficient (relative to its largest column norm).
bool LeastSquares(std::vector<double>& A, size_t m, int k, std::vector<double>& b1, std::vector<double>& b2,
                  double* c1, double* c2) {
    double maxNorm = 0;
    for (int j = 0; j < k; ++j) {
        double* col = &A[static_cast<size_t>(j) * m];
        double norm = 0;
        for (size_t i = static_cast<size_t>(j); i < m; ++i) norm += col[i] * col[i];
        norm = std::sqrt(norm);
        maxNorm = std::max(maxNorm, norm);
        if (norm <= 1e-12 * maxNorm || norm == 0.0) return false;
        const double alpha = col[j] > 0 ? -norm : norm;
        // v = x - alpha e1, stored in col[j..m)
        col[j] -= alpha;
        double vnorm2 = 0;
        for (size_t i = static_cast<size_t>(j); i < m; ++i) vnorm2 += col[i] * col[i];
        // Apply H = I - 2 v v^T / (v^T v) to the remaining columns and to b1, b2.
        const auto apply = [&](double* y) {
            double dot = 0;
            for (size_t i = static_cast<size_t>(j); i < m; ++i) dot += col[i] * y[i];
            const double f = 2.0 * dot / vnorm2;
            for (size_t i = static_cast<size_t>(j); i < m; ++i) y[i] -= f * col[i];
        };
        for (int jj = j + 1; jj < k; ++jj) apply(&A[static_cast<size_t>(jj) * m]);
        apply(b1.data());
        apply(b2.data());
        // R(j, j) = alpha; the rest of column j below the diagonal is no longer needed.
        col[j] = alpha;
    }
    // Back substitution R c = Q^T b (first k rows).
    for (int j = k - 1; j >= 0; --j) {
        double s1 = b1[static_cast<size_t>(j)], s2 = b2[static_cast<size_t>(j)];
        for (int jj = j + 1; jj < k; ++jj) {
            const double r = A[static_cast<size_t>(jj) * m + static_cast<size_t>(j)];
            s1 -= r * c1[jj];
            s2 -= r * c2[jj];
        }
        const double d = A[static_cast<size_t>(j) * m + static_cast<size_t>(j)];
        c1[j] = s1 / d;
        c2[j] = s2 / d;
    }
    return true;
}

}  // namespace

void PolyFit::Eval(double x, double y, double& u, double& v) const noexcept {
    double t[kMaxTerms];
    Monomials((x - cx_) * sx_, (y - cy_) * sy_, t);
    const int k = Terms();
    double su = 0, sv = 0;
    for (int i = 0; i < k; ++i) {
        su += cu_[static_cast<size_t>(i)] * t[i];
        sv += cv_[static_cast<size_t>(i)] * t[i];
    }
    u = su;
    v = sv;
}

void PolyFit::Eval(const double* xs, const double* ys, double* us, double* vs, size_t n) const noexcept {
    // Horner-free explicit form: the loop body is branch-free and vectorises.
    const auto& a = cu_;
    const auto& b = cv_;
    const double cx = cx_, cy = cy_, sx = sx_, sy = sy_;
    switch (degree_) {
        case 1:
            for (size_t i = 0; i < n; ++i) {
                const double X = (xs[i] - cx) * sx, Y = (ys[i] - cy) * sy;
                const double u = a[0] + a[1] * X + a[2] * Y;
                const double v = b[0] + b[1] * X + b[2] * Y;
                us[i] = u;
                vs[i] = v;
            }
            break;
        case 2:
            for (size_t i = 0; i < n; ++i) {
                const double X = (xs[i] - cx) * sx, Y = (ys[i] - cy) * sy;
                const double XX = X * X, XY = X * Y, YY = Y * Y;
                const double u = a[0] + a[1] * X + a[2] * Y + a[3] * XX + a[4] * XY + a[5] * YY;
                const double v = b[0] + b[1] * X + b[2] * Y + b[3] * XX + b[4] * XY + b[5] * YY;
                us[i] = u;
                vs[i] = v;
            }
            break;
        default:
            for (size_t i = 0; i < n; ++i) {
                const double X = (xs[i] - cx) * sx, Y = (ys[i] - cy) * sy;
                const double XX = X * X, XY = X * Y, YY = Y * Y;
                const double u = a[0] + a[1] * X + a[2] * Y + a[3] * XX + a[4] * XY + a[5] * YY + a[6] * XX * X +
                                 a[7] * XX * Y + a[8] * X * YY + a[9] * YY * Y;
                const double v = b[0] + b[1] * X + b[2] * Y + b[3] * XX + b[4] * XY + b[5] * YY + b[6] * XX * X +
                                 b[7] * XX * Y + b[8] * X * YY + b[9] * YY * Y;
                us[i] = u;
                vs[i] = v;
            }
            break;
    }
}

std::optional<PolyFit> FitPolynomial(const double* xs, const double* ys, const double* us, const double* vs, size_t n,
                                     int degree) {
    if (degree < 1 || degree > 3) return std::nullopt;
    const int k = PolyFit::TermsFor(degree);
    if (n < static_cast<size_t>(k)) return std::nullopt;

    PolyFit f;
    f.degree_ = degree;
    double minX = xs[0], maxX = xs[0], minY = ys[0], maxY = ys[0];
    for (size_t i = 1; i < n; ++i) {
        minX = std::min(minX, xs[i]);
        maxX = std::max(maxX, xs[i]);
        minY = std::min(minY, ys[i]);
        maxY = std::max(maxY, ys[i]);
    }
    f.cx_ = 0.5 * (minX + maxX);
    f.cy_ = 0.5 * (minY + maxY);
    f.sx_ = maxX > minX ? 2.0 / (maxX - minX) : 1.0;
    f.sy_ = maxY > minY ? 2.0 / (maxY - minY) : 1.0;

    std::vector<double> A(n * static_cast<size_t>(k)), bu(us, us + n), bv(vs, vs + n);
    double t[PolyFit::kMaxTerms];
    for (size_t i = 0; i < n; ++i) {
        Monomials((xs[i] - f.cx_) * f.sx_, (ys[i] - f.cy_) * f.sy_, t);
        for (int j = 0; j < k; ++j) A[static_cast<size_t>(j) * n + i] = t[j];
    }
    double cu[PolyFit::kMaxTerms] = {}, cv[PolyFit::kMaxTerms] = {};
    if (!LeastSquares(A, n, k, bu, bv, cu, cv)) return std::nullopt;
    for (int j = 0; j < k; ++j) {
        f.cu_[static_cast<size_t>(j)] = cu[j];
        f.cv_[static_cast<size_t>(j)] = cv[j];
    }
    double maxR = 0, sumR2 = 0;
    for (size_t i = 0; i < n; ++i) {
        double u = 0, v = 0;
        f.Eval(xs[i], ys[i], u, v);
        const double r2 = (u - us[i]) * (u - us[i]) + (v - vs[i]) * (v - vs[i]);
        sumR2 += r2;
        maxR = std::max(maxR, std::sqrt(r2));
    }
    f.maxResidual_ = maxR;
    f.rmsResidual_ = std::sqrt(sumR2 / static_cast<double>(n));
    return f;
}

std::optional<PolyFit> FitPolynomialAdaptive(const Transform& transform, double minX, double minY, double maxX,
                                             double maxY, double maxResidual, int samples, Direction direction) {
    samples = std::max(samples, 4);
    const size_t m = static_cast<size_t>(samples) * static_cast<size_t>(samples);
    std::vector<double> xs(m), ys(m), us(m), vs(m);
    for (int j = 0; j < samples; ++j)
        for (int i = 0; i < samples; ++i) {
            const size_t k = static_cast<size_t>(j * samples + i);
            xs[k] = minX + (maxX - minX) * i / (samples - 1);
            ys[k] = minY + (maxY - minY) * j / (samples - 1);
        }
    us = xs;
    vs = ys;
    if (transform.TransformPoints(us.data(), vs.data(), nullptr, m, direction) != 0) return std::nullopt;
    for (int degree = 1; degree <= 3; ++degree) {
        auto f = FitPolynomial(xs.data(), ys.data(), us.data(), vs.data(), m, degree);
        if (f && f->MaxResidual() <= maxResidual) return f;
    }
    return std::nullopt;
}

}  // namespace ddbt::crs
