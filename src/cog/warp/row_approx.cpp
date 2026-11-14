// SPDX-License-Identifier: MPL-2.0
//
// Ported from GDAL 3.13.3, MIT licence: GDALApproxTransform and GDALApproxTransformInternal of
// alg/gdaltransformer.cpp. The copyright notices and the permission notice of the ported code:
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
// Compiled without floating-point contraction (sources.cmake): the interpolation must round as
// GDAL's.
#include "cog/warp/row_approx.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ddbt::cog::warp {

namespace {

// The x of destination pixel i of the row: GDAL's iDstX + 0.5 + nDstXOff, the same value for any
// chunk offset nDstXOff (whole numbers and halves are exact).
double X(size_t i) { return static_cast<double>(i) + 0.5; }

}  // namespace

struct RowApproximator::Span {
    double y = 0;
    size_t first = 0, last = 0;  // requested points [first, last)
    double* xs = nullptr;
    double* ys = nullptr;
    char* ok = nullptr;
    bool Overlaps(size_t from, size_t to) const { return from < last && to >= first; }  // [from, to]
};

void RowApproximator::Set(const Span& s, size_t i, const Pt& p) const {
    if (i < s.first || i >= s.last) return;
    s.xs[i - s.first] = p.x;
    s.ys[i - s.first] = p.y;
    s.ok[i - s.first] = p.ok;
}

void RowApproximator::Exactly(const Span& s, size_t from, size_t to) const {
    if (from > to || !s.Overlaps(from, to)) return;
    const size_t a = std::max(from, s.first), b = std::min(to + 1, s.last);
    for (size_t i = a; i < b; ++i) {
        s.xs[i - s.first] = X(i);
        s.ys[i - s.first] = s.y;
    }
    exact_(s.xs + (a - s.first), s.ys + (a - s.first), b - a, s.ok + (a - s.first));
}

void RowApproximator::Segment(const Span& s, size_t start, size_t count, const Pt sme[3]) const {
    if (!s.Overlaps(start, start + count - 1)) return;
    const size_t mid = (count - 1) / 2;
    const double x0 = X(start), xm = X(start + mid), xe = X(start + count - 1);
    const double dX = (sme[2].x - sme[0].x) / (xe - x0);
    const double dY = (sme[2].y - sme[0].y) / (xe - x0);
    const double error = std::fabs((sme[0].x + dX * (xm - x0)) - sme[1].x) + std::fabs((sme[0].y + dY * (xm - x0)) - sme[1].y);
    if (error > maxError_) {
        // Halves [start, start + mid - 1] and [start + mid, start + count - 1], each with its
        // middle point; a half of at most 5 points is transformed exactly.
        const size_t ia = start + (mid - 1) / 2, ib = start + mid - 1, ic = start + mid + (count - mid - 1) / 2;
        const bool exact1 = mid <= 5, exact2 = count - mid <= 5;
        Pt m[3];
        bool success = false;
        if (!exact1 || !exact2) {
            double mx[3], my[3];
            char mok[3] = {1, 1, 1};
            size_t k = 0;
            const size_t from = exact1 ? 2 : 0, to = exact2 ? 2 : 3;
            for (size_t j = from; j < to; ++j, ++k) {
                mx[k] = X(j == 0 ? ia : j == 1 ? ib : ic);
                my[k] = s.y;
            }
            exact_(mx, my, k, mok);
            success = true;
            for (size_t j = from, q = 0; j < to; ++j, ++q) {
                m[j] = {mx[q], my[q], mok[q]};
                success = success && mok[q];
            }
        }
        if (!success) {
            Exactly(s, start + 1, start + mid - 1);
            Exactly(s, start + mid + 1, start + count - 2);
            Set(s, start, sme[0]);
            Set(s, start + mid, sme[1]);
            Set(s, start + count - 1, sme[2]);
            return;
        }
        if (!exact1) {
            const Pt half[3] = {sme[0], m[0], m[1]};
            Segment(s, start, mid, half);
        } else {
            Exactly(s, start + 1, start + mid - 1);
            Set(s, start, sme[0]);
        }
        if (!exact2) {
            const Pt half[3] = {sme[1], m[2], sme[2]};
            Segment(s, start + mid, count - mid, half);
        } else {
            Exactly(s, start + mid + 1, start + count - 2);
            Set(s, start + mid, sme[1]);
            Set(s, start + count - 1, sme[2]);
        }
        return;
    }
    const size_t a = std::max(start, s.first), b = std::min(start + count, s.last);
    for (size_t i = a; i < b; ++i) {
        const double dist = X(i) - x0;
        s.xs[i - s.first] = sme[0].x + dX * dist;
        s.ys[i - s.first] = sme[0].y + dY * dist;
        s.ok[i - s.first] = 1;
    }
}

void RowApproximator::Points(size_t start, size_t n, double y, size_t first, size_t last, double* xs, double* ys,
                             char* ok) const {
    first = std::max(first, start);
    last = std::min(last, start + n);
    if (first >= last) return;
    const Span s{y, first, last, xs, ys, ok};
    if (n <= 5 || maxError_ == 0.0) {
        Exactly(s, first, last - 1);
        return;
    }
    const size_t mid = (n - 1) / 2;
    double sx[3] = {X(start), X(start + mid), X(start + n - 1)}, sy[3] = {y, y, y};
    char sok[3] = {0, 0, 0};
    exact_(sx, sy, 3, sok);
    if (!sok[0] || !sok[1] || !sok[2]) {
        Exactly(s, first, last - 1);
        return;
    }
    const Pt sme[3] = {{sx[0], sy[0], 1}, {sx[1], sy[1], 1}, {sx[2], sy[2], 1}};
    Segment(s, start, n, sme);
}

}  // namespace ddbt::cog::warp
