// SPDX-License-Identifier: MPL-2.0
//
// GDAL's approximate transformer (GDALApproxTransform, alg/gdaltransformer.cpp) as its warp kernel
// calls it: once per destination row of a warp chunk (WarpChunk), on the centres x_i = i + 0.5
// (i in [start, start + n)) of the n pixels of the row in the chunk, all at the same y. The
// transform of the first, middle and last point is exact; when the middle is within maxError
// (|dx| + |dy|) of the straight line between the ends, every point is interpolated on that line,
// otherwise each half is handled the same way (a half of at most 5 points, or a segment whose
// points fail, is transformed exactly). gdalwarp's warper uses it from destination pixels to
// source pixels with an error of 0.125 source pixels (-et 0.125).
//
// Points(..., first, last) gives the points [first, last) of the row with GDAL's values to the bit
// (the same exact points, the same interpolation): the recursion goes only into the halves that
// hold them, so one tile of a row costs about the same as GDAL's transform of that part of it.
//
// Compiled without floating-point contraction (sources.cmake).
#pragma once

#include <cstddef>
#include <functional>

namespace ddbt::cog::warp {

class RowApproximator {
public:
    /// The exact transform: n points in place, ok[i] = 0 for a point that fails (x, y HUGE_VAL).
    using Exact = std::function<void(double* xs, double* ys, size_t n, char* ok)>;

    RowApproximator(Exact exact, double maxError) : exact_(std::move(exact)), maxError_(maxError) {}

    /// Points [first, last) of the row of destination pixels [start, start + n) at y (a pixel
    /// centre): xs, ys and ok receive last - first values (a failed point: HUGE_VAL, ok 0).
    void Points(size_t start, size_t n, double y, size_t first, size_t last, double* xs, double* ys, char* ok) const;

private:
    struct Pt {
        double x = 0, y = 0;
        char ok = 0;
    };
    struct Span;  // the requested points and the row's y
    void Exactly(const Span& s, size_t from, size_t to) const;  // points [from, to] exactly
    void Set(const Span& s, size_t i, const Pt& p) const;
    void Segment(const Span& s, size_t start, size_t count, const Pt sme[3]) const;  // GDALApproxTransformInternal

    Exact exact_;
    double maxError_;
};

}  // namespace ddbt::cog::warp
