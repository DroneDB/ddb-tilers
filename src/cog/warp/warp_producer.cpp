// SPDX-License-Identifier: MPL-2.0
//
// The warped COG's full-resolution tiles with the semantics of GDAL's warper as DroneDB runs it
// (gdalwarp -r bilinear, alg/gdalwarpkernel.cpp, alg/gdalwarpoperation.cpp, alg/gdalwarper.cpp):
//
//   - destination pixel centre -> EPSG:3857 -> source CRS (a GridApprox per output tile with a
//     0.125 source pixel bound, as gdalwarp's approximate transformer, -et 0.125) -> source pixel
//     through the inverse geotransform; a centre outside the source (1e-10 slack), or within one
//     pixel outside and still outside after an exact transform, is not written;
//   - a source georeferenced by GCPs (no geotransform) follows GDAL's warper itself, whose
//     approximation of the whole transform moves its pixels by up to 0.125 source pixel: GDAL's
//     chunks of the output (WarpChunk), its approximate transformer over each row of a chunk
//     (RowApproximator) on the exact transform (output pixel -> EPSG:3857 -> GCP CRS -> inverse
//     GCP transform, SourceGeoreference), the source window of the chunk;
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
#include <climits>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <vector>

#include "cog/pyramid/overview.h"
#include "cog/warp/bilinear.h"
#include "cog/warp/row_approx.h"
#include "cog/warp/warp_chunks.h"
#include "cog/warp/target_grid.h"
#include "cog/warp/warp_kernels.h"
#include "core/crs/grid_approx.h"
#include "core/exec/parallel_for.h"

namespace ddbt::cog::warp {

using build::Plan;
using build::RasterSource;
using build::Strip;
using build::Transparency;

namespace {

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

SourceSpec Analyse(const RasterSource& src) {
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
    } else if (src.HasMask() && s.alpha < 0) {
        s.mask = true;
        s.dstAlpha = true;
    }
    if (s.alpha >= 0) s.dstAlpha = true;
    return s;
}

bool JpegEligible(const RasterSource& src) {
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
    WarpProducer(const RasterSource& src, const Plan& plan, SourceSpec spec, SourceGeoreference geo,
                 std::shared_ptr<const crs::Transform> t)
        : src_(src), plan_(plan), spec_(std::move(spec)), geo_(std::move(geo)), transform_(std::move(t)) {}

    Status Init() {
        W_ = static_cast<int>(src_.Width());
        H_ = static_cast<int>(src_.Height());
        if (const auto& sgt = geo_.GeoTransform()) {
            const auto& g = *sgt;
            srcPixel_ = std::min(std::hypot(g[1], g[4]), std::hypot(g[2], g[5]));
        } else {
            InitGdalChunks();
        }
        bw_ = static_cast<int>(src_.BlockWidth());
        bh_ = static_cast<int>(src_.BlockHeight());
        across_ = static_cast<int>(src_.BlocksAcross());
        down_ = static_cast<int>(src_.BlocksDown());
        const uint32_t B = plan_.blockSize;
        tilesAcross_ = (plan_.width + B - 1) / B;
        tilesDown_ = (plan_.height + B - 1) / B;
        windows_.resize(static_cast<size_t>(tilesAcross_) * tilesDown_);
        mapping_.inverse = geo_.PixelAffine();
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

    const std::vector<WarpChunk>& Chunks() const noexcept { return chunks_; }

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
        // through the transform, the source pixel step and its tap bounds): a point the
        // approximation put just outside the source is transformed again exactly.
        thread_local std::vector<double> tlsX, tlsY;
        thread_local std::vector<kernels::TapOrigins> tlsTaps;
        const size_t n = static_cast<size_t>(tw) * th;
        tlsX.resize(n);
        tlsY.resize(n);
        tlsTaps.resize(th);
        double* const sx = tlsX.data();
        double* const sy = tlsY.data();
        if (rows_) {
            GcpSourcePixels(x0, tw, strip.y0, th, sx, sy);
        } else {
            GeoTransformSourcePixels(x0, tw, strip.y0, th, sx, sy);
        }
        for (uint32_t y = 0; y < th; ++y) {
            const size_t row = static_cast<size_t>(y) * tw;
            tlsTaps[y] = kernels::TapOriginBounds(sx + row, sy + row, tw);
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
    // Source pixels of the destination pixels (x0 .. x0 + tw - 1, y0 .. y0 + th - 1) of a source
    // with a geotransform: a GridApprox of the reprojection over the tile with a 0.125 source
    // pixel bound, then the inverse geotransform; NaN where there is no source pixel.
    void GeoTransformSourcePixels(uint32_t x0, uint32_t tw, uint32_t y0, uint32_t th, double* sx, double* sy) const {
        thread_local std::vector<double> tlsColumns;
        thread_local std::vector<uint32_t> tlsOutside;
        tlsColumns.resize(tw);
        tlsOutside.resize(tw);
        const auto& gt = plan_.geoTransform;
        for (uint32_t x = 0; x < tw; ++x) tlsColumns[x] = gt[0] + (x0 + x + 0.5) * gt[1];
        crs::GridApproxOptions approxOptions;
        approxOptions.gridSize = 9;
        approxOptions.maxError = 0.125 * srcPixel_;
        approxOptions.direction = crs::Direction::Inverse;
        const double padX = std::abs(gt[1]), padY = std::abs(gt[5]);
        const double bx0 = gt[0] + x0 * gt[1] - padX, bx1 = gt[0] + (x0 + tw) * gt[1] + padX;
        const double by1 = gt[3] + y0 * gt[5] + padY, by0 = gt[3] + (y0 + th) * gt[5] - padY;
        const crs::GridApprox approx = crs::GridApprox::Build(transform_, bx0, by0, bx1, by1, approxOptions);
        const Rect raster{0, 0, W_, H_};
        for (uint32_t y = 0; y < th; ++y) {
            double* const rx = sx + static_cast<size_t>(y) * tw;
            double* const ry = sy + static_cast<size_t>(y) * tw;
            std::copy(tlsColumns.begin(), tlsColumns.end(), rx);
            std::fill(ry, ry + tw, gt[3] + (y0 + y + 0.5) * gt[5]);
            approx.TransformPoints(rx, ry, tw);
            const size_t outside = kernels::ToSourcePixels(mapping_, rx, ry, tw, tlsOutside.data());
            for (size_t k = 0; k < outside; ++k) {
                const uint32_t x = tlsOutside[k];
                if (!InsideOrRetry(rx[x], ry[x], x0 + x, y0 + y, raster))
                    rx[x] = ry[x] = std::numeric_limits<double>::quiet_NaN();
            }
        }
    }

    // Source pixels of a source georeferenced by GCPs, as GDAL's warper computes them: in each of
    // its chunks (WarpChunk), each row through its approximate transformer (RowApproximator, 0.125
    // source pixels), a position outside the chunk's source window not written (after an exact
    // try when it is within a pixel of it), nor a pixel of no chunk; NaN where there is none.
    void GcpSourcePixels(uint32_t x0, uint32_t tw, uint32_t y0, uint32_t th, double* sx, double* sy) const {
        thread_local std::vector<char> tlsOk;
        thread_local std::vector<const WarpChunk*> tlsChunks;
        tlsOk.resize(tw);
        std::fill(sx, sx + static_cast<size_t>(tw) * th, std::numeric_limits<double>::quiet_NaN());
        std::fill(sy, sy + static_cast<size_t>(tw) * th, std::numeric_limits<double>::quiet_NaN());
        const int tx0 = static_cast<int>(x0), tx1 = static_cast<int>(x0 + tw);
        const int ty0 = static_cast<int>(y0), ty1 = static_cast<int>(y0 + th);
        tlsChunks.clear();
        for (const WarpChunk& c : chunks_)
            if (c.dx < tx1 && c.dx + c.dw > tx0 && c.dy < ty1 && c.dy + c.dh > ty0) tlsChunks.push_back(&c);
        for (const WarpChunk* c : tlsChunks) {
            const int cx0 = std::max(tx0, c->dx), cx1 = std::min(tx1, c->dx + c->dw);
            const int cy0 = std::max(ty0, c->dy), cy1 = std::min(ty1, c->dy + c->dh);
            const Rect window{c->sx, c->sy, c->sx + c->sw, c->sy + c->sh};
            for (int y = cy0; y < cy1; ++y) {
                const size_t row = static_cast<size_t>(y - ty0) * tw;
                double* const rx = sx + row + static_cast<size_t>(cx0 - tx0);
                double* const ry = sy + row + static_cast<size_t>(cx0 - tx0);
                rows_->Points(static_cast<size_t>(c->dx), static_cast<size_t>(c->dw), y + 0.5, static_cast<size_t>(cx0),
                              static_cast<size_t>(cx1), rx, ry, tlsOk.data());
                for (int x = 0; x < cx1 - cx0; ++x) {
                    if (!tlsOk[static_cast<size_t>(x)] || std::isnan(rx[x]) || std::isnan(ry[x]) ||
                        !InsideOrRetry(rx[x], ry[x], static_cast<uint32_t>(cx0 + x), static_cast<uint32_t>(y), window))
                        rx[x] = ry[x] = std::numeric_limits<double>::quiet_NaN();
                }
            }
        }
    }

    // GDAL's chunks of the whole output for a GCP source, and its approximate transformer.
    void InitGdalChunks() {
        rows_.emplace([this](double* xs, double* ys, size_t n, char* ok) { ExactSourcePixels(xs, ys, n, ok); },
                      0.125);
        dstInverse_ = GdalInvGeoTransform(plan_.geoTransform);
        // GetWorkingMemoryForWindow's costs: the working type (the band type) of each warped band,
        // then the masks GDAL keeps for them.
        const int gdalBands = static_cast<int>(spec_.warped.size()) / (src_.Complex() ? 2 : 1);
        const int bits = static_cast<int>(src_.SampleSize() * 8 * spec_.warped.size());
        WarpChunkOptions o;
        o.srcPixelBits = bits + (spec_.alpha >= 0 ? 32 : spec_.mask ? 1 : 0) + (spec_.nodata ? gdalBands : 0);
        o.dstPixelBits = bits + (spec_.dstNodata ? gdalBands : 0) + (spec_.dstAlpha ? 32 : 0);
        o.specialPointInOutput = PoleInOutput();
        // A source with RPC metadata (GDAL's TIFF driver reads the 92 coefficients of the tag): even
        // when its GCPs georeference it, gdalwarp reads 5 more source pixels around each window
        // and warps only within the suggested extent of the source, 5 pixels around it.
        int wx = 0, wy = 0, ww = static_cast<int>(plan_.width), wh = static_cast<int>(plan_.height);
        if (src_.HasRpc()) {
            o.sourceExtra = 5;
            if (!RestrictToRpcExtent(wx, wy, ww, wh)) return;
        }
        chunks_ = CollectWarpChunks(wx, wy, ww, wh, W_, H_,
                                    [this](bool dstToSrc, double* xs, double* ys, size_t n, char* ok) {
                                        if (dstToSrc) ExactSourcePixels(xs, ys, n, ok);
                                        else ExactDestinationPixels(xs, ys, n, ok);
                                    },
                                    o);
    }

    // GDALWarpOperation's special points, the poles (longitude 0, latitude +/-89.9999) in output
    // pixels as GDALTransformLonLatToDestGenImgProjTransformer computes them: for a source CRS
    // that is geographic in degrees it applies the inverse output geotransform to the longitude
    // and latitude themselves (no reprojection); for a projected one the poles land outside any
    // EPSG:3857 output. Without a reprojection (the source is in EPSG:3857) there is none.
    bool PoleInOutput() const {
        if (transform_->GetKind() == crs::Transform::Kind::Identity) return false;
        const crs::Crs& c = transform_->Source();
        if (!c.IsHorizontallyGeographic() || !(std::fabs(c.HorizontalUnitFactor() - 0.0174532925199433) < 1e-9))
            return false;
        for (const double lat : {-89.9999, 89.9999}) {
            const double x = dstInverse_[0] + 0 * dstInverse_[1] + lat * dstInverse_[2];
            const double y = dstInverse_[3] + 0 * dstInverse_[4] + lat * dstInverse_[5];
            if (0 <= x && plan_.width >= x && 0 <= y && plan_.height >= y) return true;  // GDAL's test
        }
        return false;
    }

    // AdjustOutputExtentForRPC: the output window within 5 pixels of the source's suggested
    // extent in output pixels; false when they do not meet (nothing is warped).
    bool RestrictToRpcExtent(int& x, int& y, int& w, int& h) const {
        const auto e = SuggestedPixelExtent(*transform_, geo_, src_.Width(), src_.Height(), plan_.geoTransform);
        if (!e.ok()) return true;
        const double threshold = static_cast<double>(INT_MAX) / 2;
        for (const double v : *e)
            if (!(std::fabs(v) < threshold)) return true;
        constexpr int kPadding = 5;
        x = std::max(x, static_cast<int>(std::floor((*e)[0])) - kPadding);
        y = std::max(y, static_cast<int>(std::floor((*e)[1])) - kPadding);
        w = std::min(w - x, static_cast<int>(std::ceil((*e)[2])) + kPadding - x);
        h = std::min(h - y, static_cast<int>(std::ceil((*e)[3])) + kPadding - y);
        return w > 0 && h > 0;
    }

    // GDAL's exact transform from destination pixels to source pixels (GDALGenImgProjTransform):
    // the output geotransform, the reprojection to the source CRS, the source pixel step.
    void ExactSourcePixels(double* xs, double* ys, size_t n, char* ok) const {
        const auto& gt = plan_.geoTransform;
        for (size_t i = 0; i < n; ++i) {
            const double X = gt[0] + xs[i] * gt[1] + ys[i] * gt[2];
            const double Y = gt[3] + xs[i] * gt[4] + ys[i] * gt[5];
            xs[i] = X;
            ys[i] = Y;
        }
        transform_->TransformPoints(xs, ys, nullptr, n, crs::Direction::Inverse);
        geo_.CrsToPixel(xs, ys, n);
        for (size_t i = 0; i < n; ++i) ok[i] = xs[i] != HUGE_VAL && ys[i] != HUGE_VAL;
    }

    // And back: source pixel step, reprojection, GDAL's inverse of the output geotransform.
    void ExactDestinationPixels(double* xs, double* ys, size_t n, char* ok) const {
        geo_.PixelToCrs(xs, ys, n);
        transform_->TransformPoints(xs, ys, nullptr, n, crs::Direction::Forward);
        const auto& inv = dstInverse_;
        for (size_t i = 0; i < n; ++i) {
            ok[i] = xs[i] != HUGE_VAL && ys[i] != HUGE_VAL;
            if (!ok[i]) continue;
            const double x = inv[0] + xs[i] * inv[1] + ys[i] * inv[2];
            const double y = inv[3] + xs[i] * inv[4] + ys[i] * inv[5];
            xs[i] = x;
            ys[i] = y;
        }
    }

    // GWKCheckAndComputeSrcOffsets: inside the source window (with GDAL's 1e-10 slack); a point
    // within one pixel outside is transformed again exactly (the approximation may have moved it
    // out).
    bool InsideOrRetry(double& fx, double& fy, uint32_t dx, uint32_t dy, const Rect& w) const {
        auto inside = [&](double x, double y) {
            return x >= w.x0 && y >= w.y0 && x + 1e-10 <= w.x1 && y + 1e-10 <= w.y1;
        };
        if (inside(fx, fy)) return true;
        if (fx > w.x0 - 1 && fy > w.y0 - 1 && fx < w.x1 + 1 && fy < w.y1 + 1) {
            double X = dx + 0.5, Y = dy + 0.5;
            char ok = 0;
            ExactSourcePixels(&X, &Y, 1, &ok);
            if (!ok) return false;
            fx = X;
            fy = Y;
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
        geo_.CrsToPixel(xs.data(), ys.data(), xs.size());
        double mnx = 1e300, mny = 1e300, mxx = -1e300, mxy = -1e300;
        bool any = false;
        for (size_t i = 0; i < xs.size(); ++i) {
            const double fx = xs[i], fy = ys[i];
            if (!std::isfinite(fx) || !std::isfinite(fy)) continue;
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

    const RasterSource& src_;
    const Plan& plan_;
    SourceSpec spec_;
    SourceGeoreference geo_;
    std::shared_ptr<const crs::Transform> transform_;
    // Sources without a geotransform: GDAL's approximate transformer, its chunks, and the
    // inverse of the output geotransform as GDAL computes it.
    std::optional<RowApproximator> rows_;
    std::vector<WarpChunk> chunks_;
    std::array<double, 6> dstInverse_{};
    kernels::SourcePixelMapping mapping_;
    BilinearParams params_;
    double srcPixel_ = 1;
    int W_ = 0, H_ = 0, bw_ = 0, bh_ = 0, across_ = 0, down_ = 0;
    uint32_t tilesAcross_ = 0, tilesDown_ = 0;
    std::vector<Rect> windows_;
    BlockCache cache_;
    std::atomic<uint64_t> unplannedReads_{0};
};

// The source's pixel <-> CRS mapping and its EPSG:3857 grid.
struct SourceGrid {
    SourceGeoreference geo;
    TargetGrid grid;
};

Expected<SourceGrid> GridOf(const RasterSource& src, bool evenTileOrigin) {
    if (!src.Georef().crs)
        return Status(StatusCode::kInvalidArgument,
                      "Cannot build COG: input file has no coordinate reference system: " + src.Path());
    auto geo = SourceGeoreference::Of(src.Georef());
    if (!geo.ok())
        return Status(StatusCode::kInvalidArgument,
                      "Cannot build COG: " + std::string(geo.status().message()) + ": " + src.Path());
    auto grid = ComputeTargetGrid(*src.Georef().crs, *geo, src.Width(), src.Height(), evenTileOrigin);
    if (!grid.ok()) return grid.status();
    return SourceGrid{std::move(*geo), std::move(*grid)};
}

}  // namespace

Expected<build::Plan> WarpPlan(const build::RasterSource& src, const WarpSettings& settings) {
    auto sg = GridOf(src, settings.evenTileOrigin);
    if (!sg.ok()) return sg.status();
    const TargetGrid& grid = sg->grid;
    const SourceSpec spec = Analyse(src);
    Plan p;
    p.ownGrid = false;
    p.width = grid.width;
    p.height = grid.height;
    p.blockSize = 512;
    p.geoTransform = grid.geoTransform;
    p.zoom = grid.zoom;
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
        if (src.Complex()) {
            std::vector<std::string> complexInterps;
            for (int b = 0; b < src.Bands() / 2; ++b) complexInterps.push_back(src.ColorInterp(b));
            build::SetComplex(p, complexInterps);
        }
        // gdalwarp keeps the colour table of a 1-band source (its indices are warped as values)
        if (const auto table = src.ColorTable(); table && p.bands == 1) build::SetPalette(p, *table);
        p.codec = settings.losslessCodec;
        if (!p.complex) p.predictor = build::IsFloatType(p.type) ? 3 : 2;
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

namespace {

Expected<std::unique_ptr<WarpProducer>> MakeProducer(const build::RasterSource& src, const build::Plan& plan) {
    auto sg = GridOf(src, false);
    if (!sg.ok()) return sg.status();
    auto p = std::make_unique<WarpProducer>(src, plan, Analyse(src), std::move(sg->geo), sg->grid.transform);
    DDBT_TRY(p->Init());
    return p;
}

}  // namespace

Expected<std::unique_ptr<build::TileProducer>> MakeWarpProducer(const build::RasterSource& src, const build::Plan& plan,
                                                                uint64_t memoryBudget) {
    (void)memoryBudget;
    auto p = MakeProducer(src, plan);
    if (!p.ok()) return p.status();
    return std::unique_ptr<build::TileProducer>(std::move(*p));
}

Expected<std::vector<WarpChunk>> GdalWarpChunks(const build::RasterSource& src, const build::Plan& plan) {
    auto p = MakeProducer(src, plan);
    if (!p.ok()) return p.status();
    return (*p)->Chunks();
}

}  // namespace ddbt::cog::warp
