// SPDX-License-Identifier: MPL-2.0
//
// The warped COG's full-resolution tiles with the semantics of GDAL's warper as DroneDB runs it
// (gdalwarp -r bilinear, alg/gdalwarpkernel.cpp, alg/gdalwarpoperation.cpp, alg/gdalwarper.cpp):
//
//   - destination pixel centre -> EPSG:3857 -> source CRS (a GridApprox per output tile with a
//     0.125 source pixel bound, as gdalwarp's approximate transformer, -et 0.125) -> source pixel
//     through the inverse geotransform; a centre outside the source (1e-10 slack), or within one
//     pixel outside and still outside after an exact transform, is not written;
//   - GDAL's bilinear kernel (bilinear.h) on the source window of the tile;
//   - UNIFIED_SRC_NODATA=YES with a finite nodata (cog.cpp): a pixel is nodata when every band is;
//     without it (a NaN / infinite / huge nodata becoming an alpha band) both the unified mask
//     and each band's own nodata apply.
//
// Source blocks (tiles or strips) are decoded once: a planning pass computes, for every output
// tile, the source window it reads (its edges transformed exactly, plus a margin), and counts
// the tiles that use each block; a block is decoded by the first tile that needs it (single
// flight) and freed when its last tile is done. A tile whose pixels reach outside its planned
// window (not expected for smooth transforms) reads the missing blocks without the cache.
#include "cog/warp/warp_producer.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <vector>

#include "cog/pyramid/overview.h"
#include "cog/warp/bilinear.h"
#include "cog/warp/target_grid.h"
#include "cog/warp/warp_kernels.h"
#include "core/crs/grid_approx.h"
#include "core/exec/parallel_for.h"

namespace ddbt::cog::warp {

using build::Plan;
using build::SourceRaster;
using build::Strip;
using build::Transparency;

namespace {

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

// The source as the warper reads it.
struct SourceSpec {
    int bands = 0;                 // source bands
    std::vector<int> warped;       // source bands warped into the output (alpha excluded)
    int alpha = -1;                // source alpha band (density)
    bool mask = false;             // the internal mask as the unified validity
    std::optional<double> nodata;  // source nodata (per band and unified validity)
    bool unifiedOnly = false;      // UNIFIED_SRC_NODATA=YES: no per-band validity
    bool dstAlpha = false;
    std::optional<double> dstNodata;
    bool NoMasks() const { return alpha < 0 && !mask && !nodata; }
};

SourceSpec Analyse(const SourceRaster& src) {
    SourceSpec s;
    s.bands = src.Bands();
    // gdalwarp takes the last band as the source alpha when it is an alpha band.
    if (src.AlphaBand() == s.bands - 1 && s.bands > 1) s.alpha = s.bands - 1;
    for (int b = 0; b < s.bands; ++b)
        if (b != s.alpha) s.warped.push_back(b);
    s.nodata = src.NoData();
    if (s.nodata) {
        const double v = *s.nodata;
        if (std::isfinite(v) && std::fabs(v) < 1e30) {
            s.dstNodata = v;
            s.unifiedOnly = true;
        } else {
            s.dstAlpha = true;
        }
    } else if (src.MaskIfd() && s.alpha < 0) {
        s.mask = true;
        s.dstAlpha = true;
    }
    if (s.alpha >= 0) s.dstAlpha = true;
    return s;
}

bool JpegEligible(const SourceRaster& src) {
    const int n = src.Bands();
    if ((n != 3 && n != 4) || src.NoData()) return false;
    if (src.Type() != tiff::SampleType::kU8) return false;
    return n == 3 || src.AlphaBand() == 3;
}

class BlockCache {
public:
    struct Entry {
        std::once_flag once;
        build::LargeBytes data;
        build::LargeBytes mask;
        uint32_t rows = 0;
        std::atomic<int> remaining{0};
        std::string error;
    };

    void Init(size_t blocks) {
        entries_.clear();
        entries_.resize(blocks);
        for (auto& e : entries_) e = std::make_unique<Entry>();
    }
    Entry& At(size_t b) { return *entries_[b]; }
    std::atomic<uint64_t> decoded{0}, bytes{0}, peak{0};

private:
    std::vector<std::unique_ptr<Entry>> entries_;
};

struct Rect {
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;  // [x0, x1) x [y0, y1)
    bool Empty() const { return x1 <= x0 || y1 <= y0; }
};

class WarpProducer final : public build::TileProducer {
public:
    WarpProducer(const SourceRaster& src, const Plan& plan, SourceSpec spec, std::shared_ptr<const crs::Transform> t)
        : src_(src), plan_(plan), spec_(std::move(spec)), transform_(std::move(t)) {}

    Status Init() {
        if (!InvertGeoTransform(*src_.Georef().geoTransform, srcInv_))
            return Status(StatusCode::kInvalidArgument, "Cannot invert the geotransform of " + src_.Path());
        const auto& sgt = *src_.Georef().geoTransform;
        srcPixel_ = std::min(std::hypot(sgt[1], sgt[4]), std::hypot(sgt[2], sgt[5]));
        W_ = static_cast<int>(src_.Width());
        H_ = static_cast<int>(src_.Height());
        bw_ = static_cast<int>(src_.BlockWidth());
        bh_ = static_cast<int>(src_.BlockHeight());
        across_ = static_cast<int>(src_.BlocksAcross());
        down_ = static_cast<int>(src_.BlocksDown());
        const uint32_t B = plan_.blockSize;
        tilesAcross_ = (plan_.width + B - 1) / B;
        tilesDown_ = (plan_.height + B - 1) / B;
        windows_.resize(static_cast<size_t>(tilesAcross_) * tilesDown_);
        mapping_.inverse = srcInv_;
        mapping_.width = W_;
        mapping_.height = H_;
        InitKernelParams();
        cache_.Init(static_cast<size_t>(across_) * static_cast<size_t>(down_));
        // Planning pass: the source window of every output tile, and the consumers of each block.
        exec::ParallelFor(0, windows_.size(), 16, [&](size_t i) {
            windows_[i] = PlanWindow(static_cast<uint32_t>(i % tilesAcross_), static_cast<uint32_t>(i / tilesAcross_));
        });
        for (const Rect& r : windows_) {
            if (r.Empty()) continue;
            for (int by = r.y0 / bh_; by <= (r.y1 - 1) / bh_; ++by)
                for (int bx = r.x0 / bw_; bx <= (r.x1 - 1) / bw_; ++bx)
                    cache_.At(static_cast<size_t>(by) * static_cast<size_t>(across_) + static_cast<size_t>(bx))
                        .remaining.fetch_add(1, std::memory_order_relaxed);
        }
        return Status::Ok();
    }

    uint64_t BlocksDecoded() const override { return cache_.decoded.load(); }
    uint64_t PeakCacheBytes() const override { return cache_.peak.load(); }

    void Produce(uint32_t col, Strip& strip) override {
        const uint32_t B = plan_.blockSize;
        const uint32_t x0 = col * B;
        const uint32_t tw = std::min(B, strip.width - x0);
        const uint32_t th = strip.rows;
        InitTile(strip, x0, tw, th);
        const Rect planned = windows_[static_cast<size_t>(strip.row) * tilesAcross_ + col];
        // Source pixel of every destination pixel centre, row by row (a row stays in the cache
        // through the approximate transform, the inverse geotransform and its tap bounds): a
        // point the approximation put just outside the source is transformed again exactly.
        thread_local std::vector<double> tlsX, tlsY, tlsColumns;
        thread_local std::vector<uint32_t> tlsOutside;
        thread_local std::vector<kernels::TapOrigins> tlsTaps;
        const size_t n = static_cast<size_t>(tw) * th;
        tlsX.resize(n);
        tlsY.resize(n);
        tlsColumns.resize(tw);
        tlsOutside.resize(tw);
        tlsTaps.resize(th);
        double* const sx = tlsX.data();
        double* const sy = tlsY.data();
        const auto& gt = plan_.geoTransform;
        for (uint32_t x = 0; x < tw; ++x) tlsColumns[x] = gt[0] + (x0 + x + 0.5) * gt[1];
        crs::GridApproxOptions approxOptions;
        approxOptions.gridSize = 9;
        approxOptions.maxError = 0.125 * srcPixel_;
        approxOptions.direction = crs::Direction::Inverse;
        const double padX = std::abs(gt[1]), padY = std::abs(gt[5]);
        const double bx0 = gt[0] + x0 * gt[1] - padX, bx1 = gt[0] + (x0 + tw) * gt[1] + padX;
        const double by1 = gt[3] + strip.y0 * gt[5] + padY, by0 = gt[3] + (strip.y0 + th) * gt[5] - padY;
        const crs::GridApprox approx = crs::GridApprox::Build(transform_, bx0, by0, bx1, by1, approxOptions);
        for (uint32_t y = 0; y < th; ++y) {
            double* const rx = sx + static_cast<size_t>(y) * tw;
            double* const ry = sy + static_cast<size_t>(y) * tw;
            std::copy(tlsColumns.begin(), tlsColumns.end(), rx);
            std::fill(ry, ry + tw, gt[3] + (strip.y0 + y + 0.5) * gt[5]);
            approx.TransformPoints(rx, ry, tw);
            const size_t outside = kernels::ToSourcePixels(mapping_, rx, ry, tw, tlsOutside.data());
            for (size_t k = 0; k < outside; ++k) {
                const uint32_t x = tlsOutside[k];
                if (!InsideOrRetry(rx[x], ry[x], x0 + x, strip.y0 + y))
                    rx[x] = ry[x] = std::numeric_limits<double>::quiet_NaN();
            }
            tlsTaps[y] = kernels::TapOriginBounds(rx, ry, tw);
        }
        // Rows of the tile in chunks whose source window stays near 16 MiB (wide pixels, or an
        // output coarser than the source).
        auto needOf = [&](uint32_t r0, uint32_t r1) {
            Rect need{INT32_MAX, INT32_MAX, INT32_MIN, INT32_MIN};
            for (uint32_t y = r0; y < r1; ++y) {
                const kernels::TapOrigins& o = tlsTaps[y];
                if (!o.any) continue;
                need.x0 = std::min(need.x0, static_cast<int>(o.minX));
                need.y0 = std::min(need.y0, static_cast<int>(o.minY));
                need.x1 = std::max(need.x1, static_cast<int>(o.maxX) + 2);
                need.y1 = std::max(need.y1, static_cast<int>(o.maxY) + 2);
            }
            if (need.x1 > need.x0) {
                need.x0 = std::max(need.x0, 0);
                need.y0 = std::max(need.y0, 0);
                need.x1 = std::min(need.x1, W_);
                need.y1 = std::min(need.y1, H_);
            }
            return need;
        };
        const Rect whole = needOf(0, th);
        uint32_t chunkRows = th;
        if (whole.x1 > whole.x0) {
            const uint64_t bytes = static_cast<uint64_t>(whole.x1 - whole.x0) * static_cast<uint64_t>(whole.y1 - whole.y0) *
                                   src_.PixelBytes();
            const uint64_t parts = (bytes + (uint64_t{16} << 20) - 1) / (uint64_t{16} << 20);
            chunkRows = static_cast<uint32_t>(std::max<uint64_t>(8, (th + parts - 1) / std::max<uint64_t>(parts, 1)));
        }
        for (uint32_t r0 = 0; r0 < th; r0 += chunkRows) {
            const uint32_t r1 = std::min(th, r0 + chunkRows);
            const Rect need = chunkRows == th ? whole : needOf(r0, r1);
            if (need.x1 <= need.x0 || need.y1 <= need.y0) continue;
            thread_local Window tlsWin;
            Window& win = tlsWin;
            Gather(planned, need, win);
            for (uint32_t y = r0; y < r1; ++y) {
                const size_t row = static_cast<size_t>(y) * tw;
                WarpRow(params_, win, sx + row, sy + row, tw, strip.Pixel(x0, y),
                        params_.alphaOut == -1 ? strip.MaskAt(x0, y) : nullptr);
            }
            if (win.px.capacity() > (size_t{32} << 20)) win = Window();  // do not keep a huge buffer per thread
        }
        Release(planned);
    }

private:
    // GWKCheckAndComputeSrcOffsets: inside the source (with GDAL's 1e-10 slack); a point within
    // one pixel outside is transformed again exactly (the approximation may have moved it out).
    bool InsideOrRetry(double& fx, double& fy, uint32_t dx, uint32_t dy) const {
        auto inside = [&](double x, double y) { return x >= 0 && y >= 0 && x + 1e-10 <= W_ && y + 1e-10 <= H_; };
        if (inside(fx, fy)) return true;
        if (fx > -1 && fy > -1 && fx < W_ + 1 && fy < H_ + 1) {
            const auto& gt = plan_.geoTransform;
            double X = gt[0] + (dx + 0.5) * gt[1], Y = gt[3] + (dy + 0.5) * gt[5];
            if (!transform_->TransformPoint(X, Y, nullptr, crs::Direction::Inverse)) return false;
            fx = srcInv_[0] + X * srcInv_[1] + Y * srcInv_[2];
            fy = srcInv_[3] + X * srcInv_[4] + Y * srcInv_[5];
            return inside(fx, fy);
        }
        return false;
    }

    // The source window of output tile (col, row): its edges transformed exactly, 2 px margin.
    Rect PlanWindow(uint32_t col, uint32_t row) const {
        const uint32_t B = plan_.blockSize;
        const uint32_t x0 = col * B, y0 = row * B;
        const uint32_t x1 = std::min(x0 + B, plan_.width), y1 = std::min(y0 + B, plan_.height);
        constexpr int kSteps = 16;
        std::vector<double> xs, ys;
        const auto& gt = plan_.geoTransform;
        for (int i = 0; i <= kSteps; ++i) {
            const double fx = x0 + (x1 - x0) * static_cast<double>(i) / kSteps;
            const double fy = y0 + (y1 - y0) * static_cast<double>(i) / kSteps;
            const double pts[4][2] = {{fx, static_cast<double>(y0)}, {fx, static_cast<double>(y1)},
                                      {static_cast<double>(x0), fy}, {static_cast<double>(x1), fy}};
            for (const auto& p : pts) {
                xs.push_back(gt[0] + p[0] * gt[1]);
                ys.push_back(gt[3] + p[1] * gt[5]);
            }
        }
        transform_->TransformPoints(xs.data(), ys.data(), nullptr, xs.size(), crs::Direction::Inverse);
        double mnx = 1e300, mny = 1e300, mxx = -1e300, mxy = -1e300;
        bool any = false;
        for (size_t i = 0; i < xs.size(); ++i) {
            if (!std::isfinite(xs[i]) || xs[i] == HUGE_VAL || !std::isfinite(ys[i])) continue;
            const double fx = srcInv_[0] + xs[i] * srcInv_[1] + ys[i] * srcInv_[2];
            const double fy = srcInv_[3] + xs[i] * srcInv_[4] + ys[i] * srcInv_[5];
            mnx = std::min(mnx, fx);
            mxx = std::max(mxx, fx);
            mny = std::min(mny, fy);
            mxy = std::max(mxy, fy);
            any = true;
        }
        Rect r;
        if (!any) return r;
        constexpr double kMargin = 2.0;
        r.x0 = static_cast<int>(std::max(0.0, std::floor(mnx - kMargin)));
        r.y0 = static_cast<int>(std::max(0.0, std::floor(mny - kMargin)));
        r.x1 = static_cast<int>(std::min(static_cast<double>(W_), std::ceil(mxx + kMargin)));
        r.y1 = static_cast<int>(std::min(static_cast<double>(H_), std::ceil(mxy + kMargin)));
        if (r.Empty()) return Rect{};
        return r;
    }

    BlockCache::Entry& Acquire(size_t b) {
        BlockCache::Entry& e = cache_.At(b);
        std::call_once(e.once, [&] {
            try {
                Decode(b, e.data, e.mask, e.rows);
                cache_.decoded.fetch_add(1, std::memory_order_relaxed);
                const uint64_t sz = e.data.size() + e.mask.size();
                const uint64_t now = cache_.bytes.fetch_add(sz) + sz;
                uint64_t peak = cache_.peak.load();
                while (now > peak && !cache_.peak.compare_exchange_weak(peak, now)) {
                }
            } catch (const std::exception& ex) {
                e.error = ex.what();
            }
        });
        if (!e.error.empty()) throw std::runtime_error(e.error);
        return e;
    }

    void Decode(size_t b, build::LargeBytes& data, build::LargeBytes& mask, uint32_t& rows) const {
        const uint32_t bx = static_cast<uint32_t>(b % static_cast<size_t>(across_));
        const uint32_t by = static_cast<uint32_t>(b / static_cast<size_t>(across_));
        if (data.size() != src_.BlockBytes()) data = build::LargeBytes(src_.BlockBytes());
        auto r = src_.ReadBlock(bx, by, data.get());
        if (!r.ok()) throw std::runtime_error(std::string(r.status().message()));
        rows = *r;
        if (spec_.mask) {
            const size_t mb = static_cast<size_t>(bw_) * static_cast<size_t>(bh_);
            if (mask.size() != mb) mask = build::LargeBytes(mb);
            const Status s = src_.ReadMaskBlock(bx, by, mask.get());
            if (!s.ok()) throw std::runtime_error(std::string(s.message()));
        }
    }

    void Release(const Rect& r) {
        if (r.Empty()) return;
        for (int by = r.y0 / bh_; by <= (r.y1 - 1) / bh_; ++by)
            for (int bx = r.x0 / bw_; bx <= (r.x1 - 1) / bw_; ++bx) {
                BlockCache::Entry& e =
                    cache_.At(static_cast<size_t>(by) * static_cast<size_t>(across_) + static_cast<size_t>(bx));
                if (e.remaining.fetch_sub(1, std::memory_order_acq_rel) == 1) {
                    cache_.bytes.fetch_sub(e.data.size() + e.mask.size());
                    e.data.Reset();
                    e.mask.Reset();
                }
            }
    }

    // Copies the source pixels of `need` into the window (blocks of the planned window through
    // the cache, others decoded here), and computes the unified validity and the density.
    void Gather(const Rect& planned, const Rect& need, Window& win) {
        win.x0 = need.x0;
        win.y0 = need.y0;
        win.w = need.x1 - need.x0;
        win.h = need.y1 - need.y0;
        win.pb = src_.PixelBytes();
        const size_t np = static_cast<size_t>(win.w) * static_cast<size_t>(win.h);
        win.px.resize(np * win.pb + kWindowPadding);
        if (spec_.mask) win.mask.resize(np);
        thread_local build::LargeBytes tmpData, tmpMask;
        for (int by = need.y0 / bh_; by <= (need.y1 - 1) / bh_; ++by) {
            for (int bx = need.x0 / bw_; bx <= (need.x1 - 1) / bw_; ++bx) {
                const size_t b = static_cast<size_t>(by) * static_cast<size_t>(across_) + static_cast<size_t>(bx);
                const bool inPlan = !planned.Empty() && bx >= planned.x0 / bw_ && bx <= (planned.x1 - 1) / bw_ &&
                                    by >= planned.y0 / bh_ && by <= (planned.y1 - 1) / bh_;
                const uint8_t* data;
                const uint8_t* mask = nullptr;
                if (inPlan) {
                    BlockCache::Entry& e = Acquire(b);
                    data = e.data.get();
                    mask = e.mask.get();
                } else {
                    uint32_t rows = 0;
                    Decode(b, tmpData, tmpMask, rows);
                    unplannedReads_.fetch_add(1, std::memory_order_relaxed);
                    data = tmpData.get();
                    mask = tmpMask.get();
                }
                const int ox = bx * bw_, oy = by * bh_;
                const int cx0 = std::max(ox, need.x0), cx1 = std::min(ox + bw_, need.x1);
                const int cy0 = std::max(oy, need.y0), cy1 = std::min(oy + bh_, need.y1);
                for (int y = cy0; y < cy1; ++y) {
                    const size_t so = static_cast<size_t>(y - oy) * static_cast<size_t>(bw_) + static_cast<size_t>(cx0 - ox);
                    std::memcpy(win.px.data() + win.Index(cx0, y) * win.pb, data + so * win.pb,
                                static_cast<size_t>(cx1 - cx0) * win.pb);
                    if (spec_.mask) std::memcpy(win.mask.data() + win.Index(cx0, y), mask + so, static_cast<size_t>(cx1 - cx0));
                }
            }
        }
        // Unified validity: the mask, or "not every band at nodata" (the merged per-band masks).
        const size_t ss = src_.SampleSize();
        if (spec_.mask) {
            win.valid.resize(np + kWindowPadding);
            for (size_t i = 0; i < np; ++i) win.valid[i] = win.mask[i] != 0;
        } else if (spec_.nodata) {
            win.valid.resize(np + kWindowPadding);
            const double nodata = *spec_.nodata;
            VisitSampleType(src_.Type(), [&]<typename T>() {
                for (size_t i = 0; i < np; ++i) {
                    const uint8_t* const pixel = win.px.data() + i * win.pb;
                    bool any = false;
                    for (int b : spec_.warped) {
                        if (!IsNodata<T>(pixel + static_cast<size_t>(b) * ss, nodata)) {
                            any = true;
                            break;
                        }
                    }
                    win.valid[i] = any;
                }
            });
        } else {
            win.valid.clear();
        }
        if (spec_.alpha >= 0) {
            win.density.resize(np);
            const uint8_t* const alpha = win.px.data() + static_cast<size_t>(spec_.alpha) * ss;
            VisitSampleType(src_.Type(), [&]<typename T>() {
                const float inv = 1.0f / 255.0f;
                for (size_t i = 0; i < np; ++i) {
                    // GDALWarpSrcAlphaMasker: value * (1 / SRC_ALPHA_MAX), at most 1.
                    win.density[i] = std::min(1.0f, static_cast<float>(LoadSample<T>(alpha + i * win.pb)) * inv);
                }
            });
        } else {
            win.density.clear();
        }
        IndexValidity(win);
    }

    // The destination before the warp: nodata, or 0 (alpha 0).
    void InitTile(Strip& s, uint32_t x0, uint32_t tw, uint32_t th) const {
        const size_t pb = s.pixelBytes;
        std::vector<uint8_t> fill(pb, 0);
        if (spec_.dstNodata) {
            tiff::FillSamples({fill.data(), fill.size()}, plan_.type, spec_.dstNodata);
            if (plan_.alphaBand >= 0)
                std::memset(fill.data() + static_cast<size_t>(plan_.alphaBand) * plan_.SampleSize(), 0, plan_.SampleSize());
        }
        for (uint32_t y = 0; y < th; ++y) {
            uint8_t* row = s.Pixel(x0, y);
            if (!spec_.dstNodata) {
                std::memset(row, 0, static_cast<size_t>(tw) * pb);
            } else {
                for (uint32_t x = 0; x < tw; ++x) std::memcpy(row + static_cast<size_t>(x) * pb, fill.data(), pb);
            }
            if (s.mask) std::memset(s.MaskAt(x0, y), 0, tw);
        }
    }

    // GDAL's warp options for this output, resolved once.
    void InitKernelParams() {
        BilinearParams& p = params_;
        const int nWarped = static_cast<int>(spec_.warped.size());
        const int nOut = plan_.MaskOut() ? 3 : nWarped;  // a JPEG output keeps the colour bands
        p.type = src_.Type();
        p.sourceWidth = W_;
        p.sourceHeight = H_;
        p.bandOffsets.clear();
        for (int bi = 0; bi < nOut; ++bi)
            p.bandOffsets.push_back(static_cast<size_t>(spec_.warped[static_cast<size_t>(bi)]) * src_.SampleSize());
        p.outPixelBytes = plan_.PixelBytes();
        p.noMasks = spec_.NoMasks();
        if (spec_.nodata && !spec_.unifiedOnly) p.perBandNodata = spec_.nodata;
        p.dstNodata = spec_.dstNodata;
        const bool hasDstNd = spec_.dstNodata.has_value();
        p.avoidSingle = hasDstNd && (nWarped == 1 || !spec_.unifiedOnly);
        p.avoidMulti = hasDstNd && !p.avoidSingle;
        // Destination density when a value is blended (GWKSetPixelValue): a destination alpha or
        // nodata mask starts at 0; without either the destination counts as fully valid.
        p.dstDensityInit = (spec_.dstAlpha || hasDstNd) ? 0.0 : 1.0;
        p.alphaOut = spec_.dstAlpha ? (plan_.MaskOut() ? -1 : plan_.alphaBand) : -2;
    }

    const SourceRaster& src_;
    const Plan& plan_;
    SourceSpec spec_;
    std::shared_ptr<const crs::Transform> transform_;
    std::array<double, 6> srcInv_{};
    kernels::SourcePixelMapping mapping_;
    BilinearParams params_;
    double srcPixel_ = 1;
    int W_ = 0, H_ = 0, bw_ = 0, bh_ = 0, across_ = 0, down_ = 0;
    uint32_t tilesAcross_ = 0, tilesDown_ = 0;
    std::vector<Rect> windows_;
    BlockCache cache_;
    std::atomic<uint64_t> unplannedReads_{0};
};

}  // namespace

Expected<build::Plan> WarpPlan(const build::SourceRaster& src, const WarpSettings& settings) {
    if (!src.Georef().crs)
        return Status(StatusCode::kInvalidArgument,
                      "Cannot build COG: input file has no coordinate reference system: " + src.Path());
    auto grid = ComputeTargetGrid(*src.Georef().crs, *src.Georef().geoTransform, src.Width(), src.Height(),
                                  settings.evenTileOrigin);
    if (!grid.ok()) return grid.status();
    const SourceSpec spec = Analyse(src);
    Plan p;
    p.ownGrid = false;
    p.width = grid->width;
    p.height = grid->height;
    p.blockSize = 512;
    p.geoTransform = grid->geoTransform;
    p.zoom = grid->zoom;
    p.overviews = pyramid::OverviewSizes(p.width, p.height, p.blockSize, true);
    p.type = src.Type();
    const bool jpeg = !settings.noJpeg && JpegEligible(src);
    std::vector<std::string> interps;
    for (int b : spec.warped) interps.push_back(src.ColorInterp(b));
    if (jpeg) {
        p.bands = 3;
        p.codec = encode::Codec::kJpeg;
        p.jpegQuality = settings.jpegQuality;
        p.photometric = tiff::photometric::kYCbCr;
        p.transparency = spec.dstAlpha ? Transparency::kMask : Transparency::kNone;
    } else {
        p.bands = static_cast<int>(spec.warped.size()) + (spec.dstAlpha ? 1 : 0);
        if (spec.dstAlpha) {
            interps.emplace_back("Alpha");
            p.alphaBand = p.bands - 1;
        }
        build::SetPhotometric(p, interps);
        // gdalwarp keeps the colour table of a 1-band source (its indices are warped as values)
        if (const auto table = src.ColorTable(); table && p.bands == 1) build::SetPalette(p, *table);
        p.codec = settings.losslessCodec;
        p.predictor = build::IsFloatType(p.type) ? 3 : 2;
        p.level = settings.level;
        if (spec.dstNodata) {
            p.transparency = Transparency::kNoData;
            p.nodata = spec.dstNodata;
            p.nodataText = src.NoDataText();
        } else if (spec.dstAlpha) {
            p.transparency = Transparency::kAlpha;
            p.alphaMasksOverviews = build::AlphaMaskRule(p.bands, p.alphaBand, p.type);
        }
    }
    return p;
}

Expected<std::unique_ptr<build::TileProducer>> MakeWarpProducer(const build::SourceRaster& src, const build::Plan& plan,
                                                                uint64_t memoryBudget) {
    (void)memoryBudget;
    auto grid = ComputeTargetGrid(*src.Georef().crs, *src.Georef().geoTransform, src.Width(), src.Height(), false);
    if (!grid.ok()) return grid.status();
    auto p = std::make_unique<WarpProducer>(src, plan, Analyse(src), grid->transform);
    DDBT_TRY(p->Init());
    return std::unique_ptr<build::TileProducer>(std::move(p));
}

}  // namespace ddbt::cog::warp
