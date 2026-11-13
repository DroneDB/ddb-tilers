// SPDX-License-Identifier: MPL-2.0
#pragma once

// Least-squares 2-D polynomial fits of a mapping (x, y) -> (u, v): affine (degree 1, 3 terms per
// output), quadratic (6) and cubic (10). For transforms over small areas (a COPC tile): a cubic
// fit of a projection over a few hundred metres is good to well under a millimetre, and costs
// 20 multiply-adds per point (vectorisable) instead of a PROJ call.
//
// The inputs are centred and scaled to [-1, 1] before the fit (conditioning), the system is
// solved by Householder QR. FitPolynomialAdaptive() samples a Transform over a rectangle and
// returns the lowest degree whose max residual is within a bound.

#include <array>
#include <cstddef>
#include <memory>
#include <optional>

#include "core/crs/transform.h"

namespace ddbt::crs {

class PolyFit {
public:
    static constexpr int kMaxTerms = 10;

    int Degree() const noexcept { return degree_; }
    int Terms() const noexcept { return TermsFor(degree_); }
    static constexpr int TermsFor(int degree) noexcept { return (degree + 1) * (degree + 2) / 2; }

    /// Max and RMS distance between the fit and the samples it was fitted to (output units).
    double MaxResidual() const noexcept { return maxResidual_; }
    double RmsResidual() const noexcept { return rmsResidual_; }

    /// Evaluates the fit at one point.
    void Eval(double x, double y, double& u, double& v) const noexcept;
    /// Evaluates n points (structure of arrays; out may alias in).
    void Eval(const double* xs, const double* ys, double* us, double* vs, size_t n) const noexcept;

    /// Coefficients of u and v in normalised coordinates (X = (x - cx) * sx, Y = (y - cy) * sy),
    /// term order 1, X, Y, X^2, XY, Y^2, X^3, X^2Y, XY^2, Y^3.
    const std::array<double, kMaxTerms>& CoeffsU() const noexcept { return cu_; }
    const std::array<double, kMaxTerms>& CoeffsV() const noexcept { return cv_; }

    friend std::optional<PolyFit> FitPolynomial(const double* xs, const double* ys, const double* us,
                                                const double* vs, size_t n, int degree);

private:
    int degree_ = 1;
    double cx_ = 0, cy_ = 0, sx_ = 1, sy_ = 1;
    std::array<double, kMaxTerms> cu_{}, cv_{};
    double maxResidual_ = 0, rmsResidual_ = 0;
};

/// Least-squares fit of degree 1, 2 or 3 through n samples (x, y) -> (u, v). nullopt when there
/// are fewer samples than terms, the samples are degenerate (rank deficient), or the degree is
/// out of range.
std::optional<PolyFit> FitPolynomial(const double* xs, const double* ys, const double* us, const double* vs,
                                     size_t n, int degree);

/// Samples `transform` on a samples x samples grid over the rectangle and fits degree 1, then 2,
/// then 3, returning the first whose max residual <= maxResidual (or nullopt if none does, or
/// if a sample fails to transform).
std::optional<PolyFit> FitPolynomialAdaptive(const Transform& transform, double minX, double minY, double maxX,
                                             double maxY, double maxResidual, int samples = 8,
                                             Direction direction = Direction::Forward);

}  // namespace ddbt::crs
