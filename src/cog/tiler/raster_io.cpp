// SPDX-License-Identifier: MPL-2.0
#include "cog/tiler/raster_io.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "cog/tiler/kernels.h"
#include "cog/tiler/tile_error.h"
#include "cog/tiler/warp.h"

namespace ddbt::cog::tiler {

namespace {

template <typename T>
T Load(const uint8_t* p) noexcept {
    T v;
    std::memcpy(&v, p, sizeof(T));
    return v;
}

using kernels::AreRealEqual;

// A small per-call memo of the last blocks used, so a gather looks a block up once per run.
class BlockMemo {
public:
    BlockMemo(const CogSource& s, uint32_t ifd) : source_(s), ifd_(ifd) {}
    const CachedBlock& Get(uint32_t index) {
        if (!entries_.empty() && entries_.back().first == index) return *entries_.back().second;
        for (auto& e : entries_)
            if (e.first == index) return *e.second;
        BlockPtr b = GetBlock(source_, ifd_, index);
        if (entries_.size() >= 8) entries_.erase(entries_.begin());
        entries_.emplace_back(index, std::move(b));
        return *entries_.back().second;
    }

private:
    const CogSource& source_;
    uint32_t ifd_;
    std::vector<std::pair<uint32_t, BlockPtr>> entries_;
};

// Gathers samples of `bands` (0-based sample indices of the file) from level `lvl` of the
// source. `at(i)` gives (sx, sy) of buffer pixel i, or sx < 0 for none.
template <typename At>
void GatherNative(const CogSource& src, int lvl, size_t pixels, std::span<const int> bands, uint8_t* out,
                  size_t planeBytes, At&& at) {
    const SourceLevel& level = src.Levels()[static_cast<size_t>(lvl)];
    const tiff::IfdInfo& info = src.Reader().Ifd(level.ifd);
    const size_t ss = PixelTypeSize(src.Type());
    const bool contig = info.IsPixelInterleaved();
    if (contig) {
        BlockMemo memo(src, level.ifd);
        for (size_t i = 0; i < pixels; ++i) {
            const auto [sx, sy] = at(i);
            if (sx < 0) {
                for (size_t b = 0; b < bands.size(); ++b) std::memset(out + b * planeBytes + i * ss, 0, ss);
                continue;
            }
            const uint32_t bc = static_cast<uint32_t>(sx) / info.blockWidth;
            const uint32_t br = static_cast<uint32_t>(sy) / info.blockHeight;
            const CachedBlock& cb = memo.Get(info.BlockIndex(bc, br));
            const uint8_t* px = cb.block.data() + static_cast<size_t>(static_cast<uint32_t>(sy) - br * info.blockHeight) * cb.block.rowStride +
                                static_cast<size_t>(static_cast<uint32_t>(sx) - bc * info.blockWidth) * cb.pixelBytes;
            for (size_t b = 0; b < bands.size(); ++b)
                std::memcpy(out + b * planeBytes + i * ss, px + static_cast<size_t>(bands[b]) * ss, ss);
        }
    } else {
        for (size_t b = 0; b < bands.size(); ++b) {
            BlockMemo memo(src, level.ifd);
            uint8_t* plane = out + b * planeBytes;
            for (size_t i = 0; i < pixels; ++i) {
                const auto [sx, sy] = at(i);
                if (sx < 0) {
                    std::memset(plane + i * ss, 0, ss);
                    continue;
                }
                const uint32_t bc = static_cast<uint32_t>(sx) / info.blockWidth;
                const uint32_t br = static_cast<uint32_t>(sy) / info.blockHeight;
                const CachedBlock& cb = memo.Get(info.BlockIndex(bc, br, static_cast<uint32_t>(bands[b])));
                const uint8_t* px = cb.block.data() +
                                    static_cast<size_t>(static_cast<uint32_t>(sy) - br * info.blockHeight) * cb.block.rowStride +
                                    static_cast<size_t>(static_cast<uint32_t>(sx) - bc * info.blockWidth) * ss;
                std::memcpy(plane + i * ss, px, ss);
            }
        }
    }
}

// Mask bytes (0 / 255) of the internal mask of level `lvl`; `fill` where there is no sample.
template <typename At>
void GatherMask(const CogSource& src, int lvl, size_t pixels, uint8_t* out, At&& at) {
    const SourceLevel& level = src.Levels()[static_cast<size_t>(lvl)];
    if (!level.maskIfd) {
        std::memset(out, 255, pixels);
        return;
    }
    const tiff::IfdInfo& info = src.Reader().Ifd(*level.maskIfd);
    BlockMemo memo(src, *level.maskIfd);
    for (size_t i = 0; i < pixels; ++i) {
        const auto [sx, sy] = at(i);
        if (sx < 0) {
            out[i] = 0;
            continue;
        }
        const uint32_t bc = static_cast<uint32_t>(sx) / info.blockWidth;
        const uint32_t br = static_cast<uint32_t>(sy) / info.blockHeight;
        const CachedBlock& cb = memo.Get(info.BlockIndex(bc, br));
        if (cb.summary != MaskSummary::kMixed) {
            out[i] = cb.summary == MaskSummary::kAllOne ? 255 : 0;
            continue;
        }
        out[i] = cb.block.data()[static_cast<size_t>(static_cast<uint32_t>(sy) - br * info.blockHeight) * cb.block.rowStride +
                                 static_cast<size_t>(static_cast<uint32_t>(sx) - bc * info.blockWidth)];
    }
}

// Separable plans: the buffer columns grouped into runs that read one block column, each run
// fetched once per block row (instead of a block lookup per pixel). A run whose columns are
// consecutive (a read at the level's own resolution, the common case of tiles aligned with the
// COG grid) is copied as one span of the block row.
struct ColumnRuns {
    struct Run {
        int32_t begin = 0, end = 0;  // buffer columns [begin, end)
        int32_t blockCol = -1;       // -1: no sample
        bool contiguous = true;      // xIn steps by 1 over the run
        size_t Size() const noexcept { return static_cast<size_t>(end - begin); }
    };
    std::vector<Run> runs;
    std::vector<uint32_t> xIn;  // column inside its block, per buffer column
};

ColumnRuns RunsOf(const std::vector<int32_t>& cols, uint32_t blockWidth) {
    ColumnRuns r;
    r.xIn.resize(cols.size());
    for (size_t i = 0; i < cols.size(); ++i) {
        const int32_t c = cols[i];
        const int32_t bc = c < 0 ? -1 : static_cast<int32_t>(static_cast<uint32_t>(c) / blockWidth);
        r.xIn[i] = c < 0 ? 0 : static_cast<uint32_t>(c) - static_cast<uint32_t>(bc) * blockWidth;
        if (r.runs.empty() || r.runs.back().blockCol != bc) {
            r.runs.push_back({static_cast<int32_t>(i), static_cast<int32_t>(i) + 1, bc, true});
        } else {
            ColumnRuns::Run& run = r.runs.back();
            run.end = static_cast<int32_t>(i) + 1;
            run.contiguous = run.contiguous && r.xIn[i] == r.xIn[i - 1] + 1;
        }
    }
    return r;
}

// The blocks of one block row, fetched lazily (separable gathers walk rows in order).
class RowBlocks {
public:
    RowBlocks(const CogSource& s, const tiff::IfdInfo& info, uint32_t plane) : s_(s), info_(info), plane_(plane) {}
    const CachedBlock& Get(uint32_t blockRow, uint32_t blockCol) {
        if (blockRow != row_) {
            row_ = blockRow;
            blocks_.assign(info_.blocksAcross, nullptr);
        }
        BlockPtr& b = blocks_[blockCol];
        if (!b) b = GetBlock(s_, info_.index, info_.BlockIndex(blockCol, blockRow, plane_));
        return *b;
    }

private:
    const CogSource& s_;
    const tiff::IfdInfo& info_;
    uint32_t plane_;
    uint32_t row_ = UINT32_MAX;
    std::vector<BlockPtr> blocks_;
};

template <size_t SS>
void CopyRun(const uint8_t* row, const uint32_t* xIn, int32_t b, int32_t e, size_t pixelBytes, size_t bandOff,
             uint8_t* dst) {
    for (int32_t i = b; i < e; ++i)
        std::memcpy(dst + static_cast<size_t>(i) * SS, row + xIn[i] * pixelBytes + bandOff, SS);
}

void CopyRunAny(size_t ss, const uint8_t* row, const uint32_t* xIn, int32_t b, int32_t e, size_t pixelBytes,
                size_t bandOff, uint8_t* dst) {
    switch (ss) {
        case 1:
            for (int32_t i = b; i < e; ++i) dst[i] = row[xIn[i] * pixelBytes + bandOff];
            break;
        case 2: CopyRun<2>(row, xIn, b, e, pixelBytes, bandOff, dst); break;
        case 4: CopyRun<4>(row, xIn, b, e, pixelBytes, bandOff, dst); break;
        default: CopyRun<8>(row, xIn, b, e, pixelBytes, bandOff, dst); break;
    }
}

void GatherSeparable(const CogSource& src, const SamplePlan& plan, const std::vector<int>& idx, uint8_t* out,
                     size_t planeBytes) {
    const SourceLevel& level = src.Levels()[static_cast<size_t>(plan.viewLevel)];
    const tiff::IfdInfo& info = src.Reader().Ifd(level.ifd);
    const size_t ss = PixelTypeSize(src.Type());
    const size_t w = static_cast<size_t>(plan.bufW);
    const ColumnRuns runs = RunsOf(plan.cols, info.blockWidth);
    const bool contig = info.IsPixelInterleaved();
    const size_t planes = contig ? 1 : idx.size();
    std::vector<uint8_t*> dsts(idx.size());
    for (size_t pl = 0; pl < planes; ++pl) {
        RowBlocks rb(src, info, contig ? 0u : static_cast<uint32_t>(idx[pl]));
        for (int j = 0; j < plan.bufH; ++j) {
            const int32_t r = plan.rows[static_cast<size_t>(j)];
            const size_t o = static_cast<size_t>(j) * w;
            for (const auto& run : runs.runs) {
                const size_t first = o + static_cast<size_t>(run.begin);
                if (r < 0 || run.blockCol < 0) {
                    for (size_t b = 0; b < (contig ? idx.size() : 1); ++b)
                        std::memset(out + (contig ? b : pl) * planeBytes + first * ss, 0, run.Size() * ss);
                    continue;
                }
                const uint32_t br = static_cast<uint32_t>(r) / info.blockHeight;
                const CachedBlock& cb = rb.Get(br, static_cast<uint32_t>(run.blockCol));
                const uint8_t* row = cb.block.data() + static_cast<size_t>(static_cast<uint32_t>(r) - br * info.blockHeight) *
                                                           cb.block.rowStride;
                const size_t x0 = runs.xIn[static_cast<size_t>(run.begin)];
                if (!contig) {
                    if (run.contiguous) std::memcpy(out + pl * planeBytes + first * ss, row + x0 * ss, run.Size() * ss);
                    else if (ss == 4)
                        kernels::PickSamples32(row, cb.block.width, runs.xIn.data() + run.begin, run.Size(),
                                               out + pl * planeBytes + first * 4);
                    else CopyRunAny(ss, row, runs.xIn.data(), run.begin, run.end, ss, 0, out + pl * planeBytes + o * ss);
                } else if (run.contiguous) {
                    // Consecutive pixels: the asked samples split into their planes at once.
                    for (size_t b = 0; b < idx.size(); ++b) dsts[b] = out + b * planeBytes + first * ss;
                    kernels::ExtractSamples(row + x0 * cb.pixelBytes, ss, cb.pixelBytes / ss, run.Size(), idx.data(),
                                            idx.size(), dsts.data());
                } else if (ss == 4 && cb.pixelBytes == 4) {
                    // One 32-bit sample per pixel (every index is 0), resampled.
                    for (size_t b = 0; b < idx.size(); ++b)
                        kernels::PickSamples32(row, cb.block.width, runs.xIn.data() + run.begin, run.Size(),
                                               out + b * planeBytes + first * 4);
                } else {
                    for (size_t b = 0; b < idx.size(); ++b)
                        CopyRunAny(ss, row, runs.xIn.data(), run.begin, run.end, cb.pixelBytes,
                                   static_cast<size_t>(idx[b]) * ss, out + b * planeBytes + o * ss);
                }
            }
        }
    }
}

void GatherMaskSeparable(const CogSource& src, const SamplePlan& plan, uint8_t* out) {
    const SourceLevel& level = src.Levels()[static_cast<size_t>(plan.viewLevel)];
    const size_t w = static_cast<size_t>(plan.bufW);
    if (!level.maskIfd) {
        std::memset(out, 255, w * static_cast<size_t>(plan.bufH));
        return;
    }
    const tiff::IfdInfo& info = src.Reader().Ifd(*level.maskIfd);
    const ColumnRuns runs = RunsOf(plan.cols, info.blockWidth);
    RowBlocks rb(src, info, 0);
    for (int j = 0; j < plan.bufH; ++j) {
        const int32_t r = plan.rows[static_cast<size_t>(j)];
        uint8_t* dst = out + static_cast<size_t>(j) * w;
        for (const auto& run : runs.runs) {
            const size_t n = static_cast<size_t>(run.end - run.begin);
            if (r < 0 || run.blockCol < 0) {
                std::memset(dst + run.begin, 0, n);
                continue;
            }
            const uint32_t br = static_cast<uint32_t>(r) / info.blockHeight;
            const CachedBlock& cb = rb.Get(br, static_cast<uint32_t>(run.blockCol));
            if (cb.summary != MaskSummary::kMixed) {
                std::memset(dst + run.begin, cb.summary == MaskSummary::kAllOne ? 255 : 0, n);
                continue;
            }
            const uint8_t* row =
                cb.block.data() + static_cast<size_t>(static_cast<uint32_t>(r) - br * info.blockHeight) * cb.block.rowStride;
            if (run.contiguous) std::memcpy(dst + run.begin, row + runs.xIn[static_cast<size_t>(run.begin)], n);
            else CopyRunAny(1, row, runs.xIn.data(), run.begin, run.end, 1, 0, dst);
        }
    }
}

// Runs fn with the (sx, sy) accessor of a plan and the source level it reads.
template <typename Fn>
void WithAccessor(const RasterView& view, const SamplePlan& plan, Fn&& fn) {
    if (plan.pointwise) {
        const int32_t* sx = plan.sx.data();
        const int32_t* sy = plan.sy.data();
        fn(plan.srcLevel, [sx, sy](size_t i) { return std::pair<int32_t, int32_t>(sx[i] < 0 ? -1 : sx[i], sy[i]); });
    } else {
        const int32_t* cols = plan.cols.data();
        const int32_t* rows = plan.rows.data();
        const size_t w = static_cast<size_t>(plan.bufW);
        fn(plan.viewLevel, [cols, rows, w](size_t i) {
            const int32_t c = cols[i % w], r = rows[i / w];
            return std::pair<int32_t, int32_t>(c < 0 || r < 0 ? -1 : c, r);
        });
    }
    (void)view;
}

// GDALCopyWords to Byte: integers clamp, floats round half away (GDAL: + 0.5 then truncate,
// clamped; NaN gives 0).
template <typename T>
uint8_t ToByte(T v) noexcept {
    if constexpr (std::is_floating_point_v<T>) {
        if (std::isnan(v)) return 0;
        if (v <= 0) return 0;
        if (v >= 255) return 255;
        return static_cast<uint8_t>(static_cast<double>(v) + 0.5);
    } else {
        if (v <= 0) return 0;
        if (v >= 255) return 255;
        return static_cast<uint8_t>(v);
    }
}

}  // namespace

// RasterView ------------------------------------------------------------------------------------

RasterView::RasterView(const CogSource& source, std::optional<double> nodataOverride)
    : source_(&source), override_(nodataOverride) {
    if (const WarpModel* w = source.Warp()) {
        warped_ = true;
        gt_ = w->GeoTransform();
        levels_ = w->Levels();
    } else {
        gt_ = source.GeoTransform();
        for (const auto& l : source.Levels())
            levels_.emplace_back(static_cast<int>(l.width), static_cast<int>(l.height));
    }
    bandCount_ = source.BandCount();
    alphaBand_ = source.AlphaBand();
}

ColorInterp RasterView::Interp(int band1) const { return source_->Band(band1).interp; }

std::optional<double> RasterView::Nodata(int band1) const {
    if (override_) return override_;
    return source_->Band(band1).nodata;
}

bool RasterView::HasDatasetMask() const noexcept {
    if (alphaBand_ > 0) return false;
    return source_->Mask() == MaskKind::kPerDataset;
}

int RasterView::DataBandCount() const noexcept { return source_->DataBandCount(); }

bool SamplePlan::AllInside() const noexcept {
    if (pointwise) {
        for (const int32_t v : sx)
            if (v < 0) return false;
        return true;
    }
    for (const int32_t v : cols)
        if (v < 0) return false;
    for (const int32_t v : rows)
        if (v < 0) return false;
    return true;
}

int BestLevel(const std::vector<std::pair<int, int>>& levels, double desired, double threshold) {
    // GDALBandGetBestOverviewLevel: index -1 is the full resolution (returned as 0 here).
    int best = -1;
    double bestRatio = 0;
    constexpr double kEps = 1e-1;
    const double w0 = levels[0].first, h0 = levels[0].second;
    for (int i = -1; i < static_cast<int>(levels.size()) - 1; ++i) {
        double ratio = 1.0;
        if (i >= 0) {
            const auto& l = levels[static_cast<size_t>(i + 1)];
            if (l.first > levels[0].first || l.second > levels[0].second) continue;
            ratio = std::min(w0 / l.first, h0 / l.second);
        }
        if (ratio >= desired * threshold + kEps || ratio <= bestRatio) continue;
        best = i;
        bestRatio = ratio;
        if (std::abs(desired - ratio) < kEps) break;
    }
    return best + 1;
}

namespace {

// The view-level plan of GDAL's RasterIO (compat).
void PlanSeparable(const std::vector<std::pair<int, int>>& levels, int x, int y, int w, int h, int bufW, int bufH,
                   SamplePlan& plan) {
    plan.bufW = bufW;
    plan.bufH = bufH;
    int level = 0;
    if ((bufW < w || bufH < h) && levels.size() > 1) {
        const double desired = (w / static_cast<double>(bufW) < h / static_cast<double>(bufH) || bufH == 1)
                                   ? w / static_cast<double>(bufW)
                                   : h / static_cast<double>(bufH);
        const int best = BestLevel(levels, desired, 1.2);
        if (best > 0) {
            const auto& l = levels[static_cast<size_t>(best)];
            const double fx = levels[0].first / static_cast<double>(l.first);
            const double fy = levels[0].second / static_cast<double>(l.second);
            const int ox = std::min(l.first - 1, static_cast<int>(x / fx + 0.5));
            const int oy = std::min(l.second - 1, static_cast<int>(y / fy + 0.5));
            int ow = std::max(1, static_cast<int>(w / fx + 0.5));
            int oh = std::max(1, static_cast<int>(h / fy + 0.5));
            if (ox + ow > l.first) ow = l.first - ox;
            if (oy + oh > l.second) oh = l.second - oy;
            x = ox;
            y = oy;
            w = ow;
            h = oh;
            level = best;
        }
    }
    plan.viewLevel = level;
    const int lw = levels[static_cast<size_t>(level)].first;
    const int lh = levels[static_cast<size_t>(level)].second;
    plan.cols.resize(static_cast<size_t>(bufW));
    plan.rows.resize(static_cast<size_t>(bufH));
    if (w == bufW && h == bufH) {
        for (int i = 0; i < bufW; ++i) plan.cols[static_cast<size_t>(i)] = x + i;
        for (int j = 0; j < bufH; ++j) plan.rows[static_cast<size_t>(j)] = y + j;
        return;
    }
    constexpr double kEps = 1e-10;
    const double incX = w / static_cast<double>(bufW);
    const double incY = h / static_cast<double>(bufH);
    for (int j = 0; j < bufH; ++j) {
        const double sy = (j + 0.5) * incY + y + kEps;
        plan.rows[static_cast<size_t>(j)] = static_cast<int>(std::min(std::max(0.0, sy), static_cast<double>(lh - 1)));
    }
    const double startX = 0.5 * incX + x + kEps;
    if (static_cast<int>(incX) == incX) {
        // DownsamplingIntegerXFactor: integer steps, only the last column clamped.
        const int inc = static_cast<int>(incX);
        int sx = static_cast<int>(startX);
        for (int i = 0; i < bufW; ++i) {
            if (i == bufW - 1 && bufW > 1) {
                plan.cols[static_cast<size_t>(i)] = static_cast<int>(
                    std::min(static_cast<int64_t>(sx), static_cast<int64_t>(lw - 1)));
            } else {
                plan.cols[static_cast<size_t>(i)] = sx;
            }
            sx += inc;
        }
    } else {
        double sx = startX;
        for (int i = 0; i < bufW; ++i, sx += incX)
            plan.cols[static_cast<size_t>(i)] = static_cast<int>(std::min(std::max(0.0, sx), static_cast<double>(lw - 1)));
    }
}

}  // namespace

SamplePlan PlanRead(const RasterView& view, int x, int y, int w, int h, int bufW, int bufH) {
    SamplePlan plan;
    PlanSeparable(view.Levels(), x, y, w, h, bufW, bufH, plan);
    if (view.Warped()) {
        plan.pointwise = true;
        view.Source().Warp()->Map(view.Source(), plan.viewLevel, plan.cols, plan.rows, plan.srcLevel, plan.sx,
                                  plan.sy);
    }
    return plan;
}

SamplePlan PlanTile(const RasterView& view, double minX, double minY, double maxX, double maxY, int size) {
    (void)minY;
    SamplePlan plan;
    plan.bufW = plan.bufH = size;
    const auto& gt = view.GeoTransform();
    const auto& levels = view.Levels();
    const double tileRes = (maxX - minX) / size;
    const int level = BestLevel(levels, tileRes / std::abs(gt[1]), 1.2);
    plan.viewLevel = level;
    const int lw = levels[static_cast<size_t>(level)].first;
    const int lh = levels[static_cast<size_t>(level)].second;
    const double resX = gt[1] * levels[0].first / lw;
    const double resY = gt[5] * levels[0].second / lh;
    plan.cols.resize(static_cast<size_t>(size));
    plan.rows.resize(static_cast<size_t>(size));
    for (int i = 0; i < size; ++i) {
        const double cx = minX + (i + 0.5) * tileRes;
        const double c = std::floor((cx - gt[0]) / resX);
        plan.cols[static_cast<size_t>(i)] = (c >= 0 && c < lw) ? static_cast<int32_t>(c) : -1;
    }
    for (int j = 0; j < size; ++j) {
        const double cy = maxY - (j + 0.5) * tileRes;
        const double r = std::floor((cy - gt[3]) / resY);
        plan.rows[static_cast<size_t>(j)] = (r >= 0 && r < lh) ? static_cast<int32_t>(r) : -1;
    }
    if (view.Warped()) {
        plan.pointwise = true;
        view.Source().Warp()->Map(view.Source(), plan.viewLevel, plan.cols, plan.rows, plan.srcLevel, plan.sx,
                                  plan.sy);
    }
    return plan;
}

void GatherBands(const RasterView& view, const SamplePlan& plan, std::span<const int> bands, uint8_t* out,
                 size_t planeBytes) {
    const CogSource& src = view.Source();
    std::vector<int> idx(bands.size());
    for (size_t i = 0; i < bands.size(); ++i) {
        if (bands[i] < 1 || bands[i] > src.BandCount()) Fail("Cannot read band " + std::to_string(bands[i]));
        idx[i] = bands[i] - 1;
    }
    const size_t pixels = static_cast<size_t>(plan.bufW) * static_cast<size_t>(plan.bufH);
    if (!plan.pointwise) {
        GatherSeparable(src, plan, idx, out, planeBytes);
        return;
    }
    WithAccessor(view, plan, [&](int lvl, auto at) { GatherNative(src, lvl, pixels, idx, out, planeBytes, at); });
    if (plan.pointwise) {
        // Inside the warped grid but outside the source: the value the built COG has there (its
        // dstnodata, else 0).
        const size_t ss = PixelTypeSize(src.Type());
        const std::optional<double> nd = src.Band(1).nodata;
        if (nd && std::isfinite(*nd) && std::abs(*nd) < 1e30) {
            std::vector<uint8_t> fill(ss);
            const double v = *nd;
            switch (src.Type()) {
                case PixelType::kU8: fill[0] = static_cast<uint8_t>(std::clamp(v, 0.0, 255.0)); break;
                case PixelType::kI8: fill[0] = static_cast<uint8_t>(static_cast<int8_t>(std::clamp(v, -128.0, 127.0))); break;
                case PixelType::kU16: { const uint16_t t = static_cast<uint16_t>(std::clamp(v, 0.0, 65535.0)); std::memcpy(fill.data(), &t, 2); break; }
                case PixelType::kI16: { const int16_t t = static_cast<int16_t>(std::clamp(v, -32768.0, 32767.0)); std::memcpy(fill.data(), &t, 2); break; }
                case PixelType::kU32: { const uint32_t t = static_cast<uint32_t>(std::clamp(v, 0.0, 4294967295.0)); std::memcpy(fill.data(), &t, 4); break; }
                case PixelType::kI32: { const int32_t t = static_cast<int32_t>(std::clamp(v, -2147483648.0, 2147483647.0)); std::memcpy(fill.data(), &t, 4); break; }
                case PixelType::kF32: { const float t = static_cast<float>(v); std::memcpy(fill.data(), &t, 4); break; }
                case PixelType::kF64: std::memcpy(fill.data(), &v, 8); break;
            }
            for (size_t i = 0; i < pixels; ++i) {
                if (plan.sx[i] != -2) continue;
                for (size_t b = 0; b < idx.size(); ++b) std::memcpy(out + b * planeBytes + i * ss, fill.data(), ss);
            }
        }
    }
}

bool IsNodataSample(PixelType type, const uint8_t* p, double nd) noexcept {
    switch (type) {
        case PixelType::kU8: return Load<uint8_t>(p) == static_cast<uint8_t>(static_cast<int64_t>(nd));
        case PixelType::kI8: return static_cast<int32_t>(Load<int8_t>(p)) == static_cast<int32_t>(nd);
        case PixelType::kU16: return Load<uint16_t>(p) == static_cast<uint16_t>(static_cast<int64_t>(nd));
        case PixelType::kI16: return Load<int16_t>(p) == static_cast<int16_t>(static_cast<int64_t>(nd));
        case PixelType::kU32: return Load<uint32_t>(p) == static_cast<uint32_t>(static_cast<int64_t>(nd));
        case PixelType::kI32: return Load<int32_t>(p) == static_cast<int32_t>(static_cast<int64_t>(nd));
        case PixelType::kF32: {
            const float v = Load<float>(p);
            if (std::isnan(nd)) return std::isnan(v);
            return AreRealEqual(v, static_cast<float>(nd));
        }
        case PixelType::kF64: {
            const double v = Load<double>(p);
            if (std::isnan(nd)) return std::isnan(v);
            return AreRealEqual(v, nd);
        }
    }
    return false;
}

double SampleToDouble(PixelType type, const uint8_t* p) noexcept {
    switch (type) {
        case PixelType::kU8: return Load<uint8_t>(p);
        case PixelType::kI8: return Load<int8_t>(p);
        case PixelType::kU16: return Load<uint16_t>(p);
        case PixelType::kI16: return Load<int16_t>(p);
        case PixelType::kU32: return Load<uint32_t>(p);
        case PixelType::kI32: return Load<int32_t>(p);
        case PixelType::kF32: return Load<float>(p);
        case PixelType::kF64: return Load<double>(p);
    }
    return 0;
}

uint8_t SampleToByte(PixelType type, const uint8_t* p) noexcept {
    switch (type) {
        case PixelType::kU8: return Load<uint8_t>(p);
        case PixelType::kI8: return ToByte(Load<int8_t>(p));
        case PixelType::kU16: return ToByte(Load<uint16_t>(p));
        case PixelType::kI16: return ToByte(Load<int16_t>(p));
        case PixelType::kU32: return ToByte(Load<uint32_t>(p));
        case PixelType::kI32: return ToByte(Load<int32_t>(p));
        case PixelType::kF32: return ToByte(Load<float>(p));
        case PixelType::kF64: return ToByte(Load<double>(p));
    }
    return 0;
}

void GatherDatasetMask(const RasterView& view, const SamplePlan& plan, uint8_t* out) {
    if (!plan.pointwise) {
        GatherMaskSeparable(view.Source(), plan, out);
        return;
    }
    const size_t pixels = static_cast<size_t>(plan.bufW) * static_cast<size_t>(plan.bufH);
    WithAccessor(view, plan, [&](int lvl, auto at) { GatherMask(view.Source(), lvl, pixels, out, at); });
}

MaskSummary PlanMaskSummary(const RasterView& view, const SamplePlan& plan) {
    const CogSource& src = view.Source();
    if (view.AlphaBand() > 0 || src.Mask() != MaskKind::kPerDataset || plan.pointwise) return MaskSummary::kMixed;
    const SourceLevel& level = src.Levels()[static_cast<size_t>(plan.viewLevel)];
    if (!level.maskIfd) return MaskSummary::kMixed;
    const tiff::IfdInfo& info = src.Reader().Ifd(*level.maskIfd);
    // Distinct block columns / rows the plan touches.
    std::vector<uint32_t> bcs, brs;
    for (const int32_t c : plan.cols) {
        if (c < 0) return MaskSummary::kMixed;  // pixels without a sample are transparent anyway
        const uint32_t b = static_cast<uint32_t>(c) / info.blockWidth;
        if (bcs.empty() || bcs.back() != b) bcs.push_back(b);
    }
    for (const int32_t r : plan.rows) {
        if (r < 0) return MaskSummary::kMixed;
        const uint32_t b = static_cast<uint32_t>(r) / info.blockHeight;
        if (brs.empty() || brs.back() != b) brs.push_back(b);
    }
    std::optional<MaskSummary> all;
    for (const uint32_t br : brs) {
        for (const uint32_t bc : bcs) {
            const BlockPtr b = GetBlock(src, *level.maskIfd, info.BlockIndex(bc, br));
            if (b->summary == MaskSummary::kMixed) return MaskSummary::kMixed;
            if (all && *all != b->summary) return MaskSummary::kMixed;
            all = b->summary;
        }
    }
    return all.value_or(MaskSummary::kMixed);
}

void GatherAlpha(const RasterView& view, const SamplePlan& plan, uint8_t* out, const uint8_t* band1) {
    const CogSource& src = view.Source();
    const size_t pixels = static_cast<size_t>(plan.bufW) * static_cast<size_t>(plan.bufH);
    const PixelType type = src.Type();
    const size_t ss = PixelTypeSize(type);

    // The transparency of what the view adds outside the source (warped: the built COG's alpha /
    // nodata, or none at all: opaque black).
    const bool warpTransparent = !view.Warped() || src.AlphaBand() > 0 || src.Mask() != MaskKind::kAllValid ||
                                 view.NodataOverride().has_value();

    if (view.AlphaBand() > 0) {
        const int b = view.AlphaBand();
        if (type == PixelType::kU8) {
            GatherBands(view, plan, std::span<const int>(&b, 1), out, pixels);  // its bytes as they are
        } else {
            std::vector<uint8_t> a(pixels * ss);
            GatherBands(view, plan, std::span<const int>(&b, 1), a.data(), pixels * ss);
            if (type == PixelType::kF32) kernels::FloatToByte(a.data(), pixels, out);
            else
                for (size_t i = 0; i < pixels; ++i) out[i] = SampleToByte(type, a.data() + i * ss);
        }
    } else {
        MaskKind kind = src.Mask();
        if (view.NodataOverride() && kind != MaskKind::kPerDataset) kind = MaskKind::kNodata;
        switch (kind) {
            case MaskKind::kPerDataset: GatherDatasetMask(view, plan, out); break;
            case MaskKind::kNodata: {
                const std::optional<double> nd = view.Nodata(1);
                std::vector<uint8_t> own;
                if (band1 == nullptr) {
                    own.resize(pixels * ss);
                    const int b = 1;
                    GatherBands(view, plan, std::span<const int>(&b, 1), own.data(), pixels * ss);
                    band1 = own.data();
                }
                if (!nd) {
                    std::memset(out, 255, pixels);
                } else if (type == PixelType::kF32 && !std::isnan(*nd)) {
                    // The common DEM case (same test as IsNodataSample).
                    kernels::NodataMaskF32(band1, pixels, static_cast<float>(*nd), out);
                } else {
                    for (size_t i = 0; i < pixels; ++i) out[i] = IsNodataSample(type, band1 + i * ss, *nd) ? 0 : 255;
                }
                break;
            }
            case MaskKind::kAlpha:
            case MaskKind::kAllValid: std::memset(out, 255, pixels); break;
        }
    }
    // No sample: transparent (outside the window), or for a warped view inside its grid but off
    // the source, what the built COG has there.
    if (plan.pointwise) {
        for (size_t i = 0; i < pixels; ++i) {
            if (plan.sx[i] == -1) out[i] = 0;
            else if (plan.sx[i] == -2) out[i] = warpTransparent ? 0 : 255;
        }
    } else {
        // Row by row: a row without a sample is cleared whole, else only its columns without one.
        const size_t w = static_cast<size_t>(plan.bufW);
        std::vector<size_t> outsideCols;
        for (size_t i = 0; i < w; ++i)
            if (plan.cols[i] < 0) outsideCols.push_back(i);
        for (size_t j = 0; j < static_cast<size_t>(plan.bufH); ++j) {
            uint8_t* row = out + j * w;
            if (plan.rows[j] < 0) std::memset(row, 0, w);
            else
                for (const size_t i : outsideCols) row[i] = 0;
        }
    }
}

}  // namespace ddbt::cog::tiler
