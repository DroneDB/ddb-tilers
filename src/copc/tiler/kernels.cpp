// SPDX-License-Identifier: MPL-2.0
//
// Highway kernels of the COPC tiler. Compiled once per enabled target (foreach_target.h
// re-includes this file); HWY_DYNAMIC_DISPATCH picks the best target at runtime. The file is
// compiled with -ffp-contract=off -fno-lto (sources.cmake): x * scale + offset must round twice,
// as PDAL's and DroneDB's builds (generic x86-64, no FMA) do, and the ramp's division and floor
// must see the same doubles as the scalar reference.

#include "copc/tiler/kernels.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "copc/copc/hierarchy.h"

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "copc/tiler/kernels.cpp"
#include "hwy/foreach_target.h"  // IWYU pragma: keep, must come before highway.h
#include "hwy/highway.h"

#ifndef DDBT_COPC_TILER_KERNELS_ONCE
#define DDBT_COPC_TILER_KERNELS_ONCE

namespace ddbt::copc::tiler::kernels {

namespace {

constexpr uint8_t kPestelShades[256][3] = {
#include "copc/tiler/pestel_shades.inc"
};

struct RampTableData {
    uint32_t rgba[kRampOutside + 1];
    RampTableData() {
        for (size_t i = 0; i < 256; ++i)
            rgba[i] = uint32_t{kPestelShades[i][0]} | (uint32_t{kPestelShades[i][1]} << 8) |
                      (uint32_t{kPestelShades[i][2]} << 16) | 0xFF000000u;
        rgba[kRampOutside] = 0xFF000000u;
    }
};

const RampTableData& Table() {
    static const RampTableData t;
    return t;
}

inline int32_t AxisValue(const NodeView& n, int a, size_t i) noexcept {
    return n.off16[a] != nullptr ? static_cast<int32_t>(n.base[a] + n.off16[a][i]) : n.val32[a][i];
}

}  // namespace

const uint32_t* RampTable() noexcept { return Table().rgba; }

uint32_t RampIndexScalar(double z, double min, double max) noexcept {
    // ColorinterpFilter::processOne (PDAL 2.10.1), operations in the same order.
    if (z < min || z > max) return kRampOutside;
    const double factor = (z - min) / (max - min);
    const size_t width = 256;
    size_t position = static_cast<size_t>(std::floor(factor * static_cast<double>(width)));
    position = std::min(position, width - 1);
    return static_cast<uint32_t>(position);
}

void RampRgbaScalar(const double* zs, size_t n, double min, double max, uint32_t* rgba) {
    const uint32_t* t = RampTable();
    for (size_t i = 0; i < n; ++i) rgba[i] = t[RampIndexScalar(zs[i], min, max)];
}

size_t DecodeCullScalar(const NodeView& node, size_t begin, size_t end, const Scaling& s, const Window& w,
                        double* xs, double* ys, double* zs, uint32_t* idx) {
    size_t out = 0;
    for (size_t i = begin; i < end; ++i) {
        const double x = MulAddUnfused(AxisValue(node, 0, i), s.scale[0], s.offset[0]);
        const double y = MulAddUnfused(AxisValue(node, 1, i), s.scale[1], s.offset[1]);
        if (!(x >= w.minX && x <= w.maxX && y >= w.minY && y <= w.maxY)) continue;
        xs[out] = x;
        ys[out] = y;
        zs[out] = MulAddUnfused(AxisValue(node, 2, i), s.scale[2], s.offset[2]);
        idx[out] = static_cast<uint32_t>(i);
        ++out;
    }
    return out;
}

}  // namespace ddbt::copc::tiler::kernels

#endif  // DDBT_COPC_TILER_KERNELS_ONCE

HWY_BEFORE_NAMESPACE();
namespace ddbt::copc::tiler::kernels {
namespace HWY_NAMESPACE {

namespace hn = hwy::HWY_NAMESPACE;
using DF = hn::ScalableTag<double>;
using VF = hn::Vec<DF>;
using DI32 = hn::Rebind<int32_t, DF>;
using DU32 = hn::Rebind<uint32_t, DF>;
using DU16 = hn::Rebind<uint16_t, DF>;

HWY_INLINE VF LoadAxis(const NodeView& n, int a, size_t i) {
    const DF df;
    const DI32 di;
    if (n.off16[a] != nullptr) {
        const DU16 d16;
        const DU32 du;
        const auto off = hn::PromoteTo(du, hn::LoadU(d16, n.off16[a] + i));
        return hn::PromoteTo(df, hn::Add(hn::BitCast(di, off), hn::Set(di, n.base[a])));
    }
    return hn::PromoteTo(df, hn::LoadU(di, n.val32[a] + i));
}

size_t DecodeCullImpl(const NodeView& node, size_t begin, size_t end, const Scaling& s, const Window& w, double* xs,
                      double* ys, double* zs, uint32_t* idx) {
    const DF df;
    const DU32 du;
    const size_t N = hn::Lanes(df);
    const VF sx = hn::Set(df, s.scale[0]), ox = hn::Set(df, s.offset[0]);
    const VF sy = hn::Set(df, s.scale[1]), oy = hn::Set(df, s.offset[1]);
    const VF sz = hn::Set(df, s.scale[2]), oz = hn::Set(df, s.offset[2]);
    const VF minX = hn::Set(df, w.minX), maxX = hn::Set(df, w.maxX);
    const VF minY = hn::Set(df, w.minY), maxY = hn::Set(df, w.maxY);
    size_t out = 0;
    size_t i = begin;
    for (; i + N <= end; i += N) {
        // Two roundings: the file is compiled without contraction.
        const VF x = hn::Add(hn::Mul(LoadAxis(node, 0, i), sx), ox);
        const VF y = hn::Add(hn::Mul(LoadAxis(node, 1, i), sy), oy);
        const auto m = hn::And(hn::And(hn::Ge(x, minX), hn::Le(x, maxX)), hn::And(hn::Ge(y, minY), hn::Le(y, maxY)));
        if (hn::AllFalse(df, m)) continue;
        const VF z = hn::Add(hn::Mul(LoadAxis(node, 2, i), sz), oz);
        // CompressStore may write up to N lanes; out <= i - begin keeps it within the outputs.
        hn::CompressStore(x, m, df, xs + out);
        hn::CompressStore(y, m, df, ys + out);
        hn::CompressStore(z, m, df, zs + out);
        const auto mi = hn::DemoteMaskTo(du, df, m);
        hn::CompressStore(hn::Iota(du, static_cast<uint32_t>(i)), mi, du, idx + out);
        out += hn::CountTrue(df, m);
    }
    if (i < end) out += DecodeCullScalar(node, i, end, s, w, xs + out, ys + out, zs + out, idx + out);
    return out;
}

void RampRgbaImpl(const double* zs, size_t n, double min, double max, uint32_t* rgba) {
    const DF df;
    const DI32 di;
    const DU32 du;
    const size_t N = hn::Lanes(df);
    const uint32_t* table = RampTable();
    const VF vmin = hn::Set(df, min), vmax = hn::Set(df, max);
    const VF range = hn::Set(df, max - min);
    const VF width = hn::Set(df, 256.0);
    const auto limit = hn::Set(di, 255);
    const auto outside = hn::Set(di, static_cast<int32_t>(kRampOutside));
    size_t i = 0;
    for (; i + N <= n; i += N) {
        const VF v = hn::LoadU(df, zs + i);
        const auto out = hn::Or(hn::Lt(v, vmin), hn::Gt(v, vmax));
        // Out-of-range lanes are replaced below; zero them first so the conversion is defined.
        const VF f = hn::IfThenZeroElse(out, hn::Div(hn::Sub(v, vmin), range));
        const VF p = hn::Floor(hn::Mul(f, width));
        auto index = hn::Min(hn::DemoteTo(di, p), limit);
        index = hn::IfThenElse(hn::RebindMask(di, hn::DemoteMaskTo(du, df, out)), outside, index);
        hn::StoreU(hn::GatherIndex(du, table, index), du, rgba + i);
    }
    if (i < n) RampRgbaScalar(zs + i, n - i, min, max, rgba + i);
}

}  // namespace HWY_NAMESPACE
}  // namespace ddbt::copc::tiler::kernels
HWY_AFTER_NAMESPACE();

#if HWY_ONCE

namespace ddbt::copc::tiler::kernels {

HWY_EXPORT(DecodeCullImpl);
HWY_EXPORT(RampRgbaImpl);

size_t DecodeCull(const NodeView& node, size_t begin, size_t end, const Scaling& s, const Window& w, double* xs,
                  double* ys, double* zs, uint32_t* idx) {
    return HWY_DYNAMIC_DISPATCH(DecodeCullImpl)(node, begin, end, s, w, xs, ys, zs, idx);
}

void RampRgba(const double* zs, size_t n, double min, double max, uint32_t* rgba) {
    HWY_DYNAMIC_DISPATCH(RampRgbaImpl)(zs, n, min, max, rgba);
}

}  // namespace ddbt::copc::tiler::kernels

#endif  // HWY_ONCE
