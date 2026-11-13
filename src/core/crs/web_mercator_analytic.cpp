// SPDX-License-Identifier: MPL-2.0
//
// Bit-exact ports of PROJ 9.9.0 (src/conversions/unitconvert.cpp, src/fwd.cpp, src/inv.cpp,
// src/projections/merc.cpp webmerc, src/adjlon.cpp) and GDAL 3.13.3 (ogr/ogrct.cpp,
// OGRProjCT::TransformWithErrorCodes, "Optimized transform from WebMercator to WGS84").
// Compiled with -ffp-contract=off -fno-lto (sources.cmake): PROJ is built without FMA.

#include "core/crs/web_mercator_analytic.h"

#include <cmath>

namespace ddbt::crs::analytic {

namespace {

constexpr double kDegToRad = .017453292519943296;  // PROJ DEG_TO_RAD, the "deg" unit factor
constexpr double kPi = 3.14159265358979323846;     // M_PI
constexpr double kHalfPi = 1.57079632679489661923;  // M_PI_2
constexpr double kTwoPi = 6.28318530717958647693;   // PROJ M_TWOPI
constexpr double kEpsLat = 1e-12;                   // PJ_EPS_LAT
constexpr double kA = kWebMercatorRadius;
const double kRa = 1. / kA;  // ell_set.cpp: P->ra = 1. / P->a

double AdjLon(double longitude) noexcept {
    if (std::fabs(longitude) < kPi + 1e-12) return longitude;
    longitude += kPi;
    longitude -= kTwoPi * std::floor(longitude / kTwoPi);
    longitude -= kPi;
    return longitude;
}

}  // namespace

size_t LonLatToWebMercator(const double* lon, const double* lat, double* x, double* y, size_t n) noexcept {
    size_t failed = 0;
    for (size_t i = 0; i < n; ++i) {
        const double lo = lon[i], la = lat[i];
        if (lo == HUGE_VAL || la == HUGE_VAL || std::isnan(lo) || std::isnan(la)) {
            x[i] = y[i] = HUGE_VAL;
            ++failed;
            continue;
        }
        // unitconvert xy_in=deg xy_out=rad: point *= xy_factor
        double lam = lo * kDegToRad;
        double phi = la * kDegToRad;
        // fwd_prepare (radians input)
        if (std::fabs(phi) > kHalfPi) {
            if (phi > 0 ? phi - kHalfPi > kEpsLat : phi - kHalfPi < -kEpsLat) {
                // PROJ 9.9 compares phi - M_HALFPI for negative phi too: any phi < -pi/2 fails
                x[i] = y[i] = HUGE_VAL;
                ++failed;
                continue;
            }
            phi = phi > 0 ? kHalfPi : -kHalfPi;
        }
        if (std::fabs(lam) > kPi) {
            if (std::fabs(lam) > 10) {
                x[i] = y[i] = HUGE_VAL;
                ++failed;
                continue;
            }
            lam = AdjLon(lam);
        }
        lam = (lam - 0.0) - 0.0;  // from_greenwich, lam0
        lam = AdjLon(lam);
        // merc_s_forward (k0 = 1), fwd_finalize (CLASSIC: *= a, then fr_meter * (v + x0))
        const double xs = 1.0 * lam;
        const double ys = 1.0 * std::asinh(std::tan(phi));
        x[i] = 1.0 * (xs * kA + 0.0);
        y[i] = 1.0 * (ys * kA + 0.0);
    }
    return failed;
}

size_t WebMercatorToLonLatProj(const double* x, const double* y, double* lon, double* lat, size_t n) noexcept {
    size_t failed = 0;
    for (size_t i = 0; i < n; ++i) {
        const double xi = x[i], yi = y[i];
        if (xi == HUGE_VAL || yi == HUGE_VAL || std::isnan(xi) || std::isnan(yi)) {
            lon[i] = lat[i] = HUGE_VAL;
            ++failed;
            continue;
        }
        // inv_prepare (CLASSIC): v = to_meter * v - x0, then *= ra
        const double xs = (1.0 * xi - 0.0) * kRa;
        const double ys = (1.0 * yi - 0.0) * kRa;
        // merc_s_inverse (k0 = 1)
        const double phi = std::atan(std::sinh(ys / 1.0));
        double lam = xs / 1.0;
        // inv_finalize: lam + from_greenwich + lam0, adjlon
        lam = lam + 0.0 + 0.0;
        lam = AdjLon(lam);
        // unitconvert inverse: point /= xy_factor
        lon[i] = lam / kDegToRad;
        lat[i] = phi / kDegToRad;
    }
    return failed;
}

size_t WebMercatorToLonLatGdal(const double* x, const double* y, double* lon, double* lat, size_t n) noexcept {
    constexpr double kReverseSphereRadius = 1.0 / 6378137.0;
    constexpr double kRadToDeg = 180. / kPi;
    size_t failed = 0;
    for (size_t i = 0; i < n; ++i) {
        double xi = x[i];
        const double yi = y[i];
        if (xi == HUGE_VAL) {
            lon[i] = lat[i] = HUGE_VAL;
            ++failed;
            continue;
        }
        xi = xi * kReverseSphereRadius;
        if (xi > kPi) {
            if (xi < kPi + 1e-14) {
                xi = kPi;
            } else {
                do {
                    xi -= 2 * kPi;
                } while (xi > kPi);
            }
        } else if (xi < -kPi) {
            if (xi > -kPi - 1e-14) {
                xi = -kPi;
            } else {
                do {
                    xi += 2 * kPi;
                } while (xi < -kPi);
            }
        }
        lon[i] = xi * kRadToDeg;
        lat[i] = (kPi / 2.0 - 2.0 * std::atan(std::exp(-yi * kReverseSphereRadius))) * kRadToDeg;
    }
    return failed;
}

}  // namespace ddbt::crs::analytic
