// SPDX-License-Identifier: MPL-2.0
#include "cog/warp/region_warp.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "cog/build/pipeline.h"
#include "cog/build/tiff_source.h"
#include "cog/tiler/raster_io.h"
#include "cog/warp/source_georef.h"
#include "core/crs/transform.h"
#include "core/crs/transform_bounds.h"
#include "core/exec/parallel_for.h"

namespace ddbt::cog::warp {

namespace {

// gdalwarp -ovr AUTO with an explicit grid: the source pixels of a 10 x 10 grid of the output (its
// corners included), their extent over the output size, the smaller ratio; above 1, the overview
// GDALBandGetBestOverviewLevel picks for it with an oversampling threshold of 1
int OverviewLevel(const build::TiffSource& src, const crs::Crs& target, const std::array<double, 6>& gt,
                  uint32_t width, uint32_t height) {
    const auto sizes = src.LevelSizes();
    if (sizes.size() < 2 || !src.Georef().crs) return 0;
    auto geo = SourceGeoreference::Of(src.Georef());
    if (!geo.ok() || !geo->GeoTransform()) return 0;
    const auto& sgt = *geo->GeoTransform();
    const double W = src.Width(), H = src.Height();
    const double ex[4] = {sgt[0], sgt[0] + W * sgt[1] + H * sgt[2], sgt[3], sgt[3] + W * sgt[4] + H * sgt[5]};
    const auto t = crs::TransformCache::Global().GetForExtent(*src.Georef().crs, target, std::min(ex[0], ex[1]),
                                                              std::min(ex[2], ex[3]), std::max(ex[0], ex[1]),
                                                              std::max(ex[2], ex[3]));
    constexpr int kSide = 10;
    std::vector<double> xs, ys;
    for (int ix = 0; ix < kSide; ++ix)
        for (int iy = 0; iy < kSide; ++iy) {
            const double px = width * static_cast<double>(ix) / (kSide - 1);
            const double py = height * static_cast<double>(iy) / (kSide - 1);
            xs.push_back(gt[0] + px * gt[1] + py * gt[2]);
            ys.push_back(gt[3] + px * gt[4] + py * gt[5]);
        }
    t->TransformPoints(xs.data(), ys.data(), nullptr, xs.size(), crs::Direction::Inverse);
    geo->CrsToPixel(xs.data(), ys.data(), xs.size());
    double minX = std::numeric_limits<double>::infinity(), maxX = -minX, minY = minX, maxY = -minX;
    for (size_t i = 0; i < xs.size(); ++i) {
        if (xs[i] == HUGE_VAL || ys[i] == HUGE_VAL || !std::isfinite(xs[i]) || !std::isfinite(ys[i])) continue;
        minX = std::min(minX, xs[i]);
        maxX = std::max(maxX, xs[i]);
        minY = std::min(minY, ys[i]);
        maxY = std::max(maxY, ys[i]);
    }
    double rx = 0, ry = 0;
    if (maxX > minX) rx = (maxX - minX) / width;
    if (maxY > minY) ry = (maxY - minY) / height;
    const double ratio = std::min(rx, ry);
    if (!(ratio > 1.0)) return 0;
    return cog::tiler::BestLevel(sizes, ratio, 1.0);
}

}  // namespace

Expected<WarpedRaster> WarpRegion(const std::string& path, const RegionRequest& request) {
    if (request.width == 0 || request.height == 0) return Status(StatusCode::kInvalidArgument, "Invalid size");
    crs::Crs target;
    std::array<double, 4> e = request.extent;
    try {
        target = crs::Crs::FromUserInput(request.targetCrs);
        if (!request.extentCrs.empty()) {
            // -te_srs: the bounds of the extent in the output's CRS, as gdalwarp computes them
            const crs::Crs from = crs::Crs::FromUserInput(request.extentCrs);
            const auto b = crs::TransformBounds(from, target, {e[0], e[1], e[2], e[3]});
            if (!b) return Status(StatusCode::kInvalidArgument, "The extent does not transform to the output's CRS");
            e = {b->minX, b->minY, b->maxX, b->maxY};
        }
    } catch (const std::exception& ex) {
        return Status(StatusCode::kInvalidArgument, std::string("Invalid CRS: ") + ex.what());
    }
    if (!(e[2] > e[0] && e[3] > e[1])) return Status(StatusCode::kInvalidArgument, "Invalid extent");
    // gdalwarp's output geotransform for -te and -ts
    const std::array<double, 6> gt = {e[0], (e[2] - e[0]) / request.width, 0.0,
                                      e[3], 0.0, -(e[3] - e[1]) / request.height};

    auto full = build::TiffSource::Open(path);
    if (!full.ok()) return full.status();
    int level = 0;
    try {
        level = OverviewLevel(**full, target, gt, request.width, request.height);
    } catch (const std::exception& ex) {
        return Status(StatusCode::kInvalidArgument, std::string("Cannot transform to the CRS of the grid: ") + ex.what());
    }
    std::unique_ptr<build::TiffSource> src = std::move(*full);
    if (level > 0) {
        auto ovr = build::TiffSource::OpenLevel(path, level);
        if (!ovr.ok()) return ovr.status();
        src = std::move(*ovr);
    }

    WarpSettings settings;
    settings.grid = ExplicitGrid{target, gt, request.width, request.height};
    // gdalwarp's default warp memory (GDALWarpOperation: 64 MB without -wm)
    settings.warpMemory = 64.0 * 1024 * 1024;
    settings.resampling = request.resampling;
    settings.dstAlpha = request.dstAlpha;
    settings.dstNodata = request.dstNodata;
    for (const int b : request.bands) {
        if (b < 1 || b > src->Bands()) return Status(StatusCode::kInvalidArgument, "Invalid band " + std::to_string(b));
        settings.bands.push_back(b - 1);
    }
    auto plan = WarpPlan(*src, settings);
    if (!plan.ok()) return plan.status();
    // The unit of work of the producer: blocks of 64 pixels share a 256 x 256 render among 16
    // threads (the samples do not depend on it: GDAL's chunks place every row)
    plan->blockSize = 64;
    auto producer = MakeWarpProducer(*src, *plan, settings, uint64_t{1} << 30);
    if (!producer.ok()) return producer.status();

    WarpedRaster out;
    out.width = plan->width;
    out.height = plan->height;
    out.bands = plan->bands;
    out.type = plan->type;
    out.geoTransform = gt;
    out.wkt = target.ToWkt(crs::WktFlavor::GdalDefault);
    out.nodata = plan->nodata;
    out.alphaBand = plan->alphaBand;
    out.sourceLevel = level;
    // gdalwarp's MEM output: the warped bands undefined, the alpha band alpha
    for (int b = 0; b < out.bands; ++b) out.colorInterp.emplace_back(b == out.alphaBand ? "Alpha" : "Undefined");
    const size_t ss = plan->SampleSize(), pb = plan->PixelBytes();
    const size_t plane = static_cast<size_t>(out.width) * out.height * ss;
    out.pixels.resize(plane * static_cast<size_t>(out.bands));

    const uint32_t B = plan->blockSize;
    const uint32_t rows = (out.height + B - 1) / B, cols = (out.width + B - 1) / B;
    try {
        for (uint32_t r = 0; r < rows; ++r) {
            build::Strip strip;
            strip.level = 0;
            strip.row = r;
            strip.y0 = r * B;
            strip.rows = std::min(B, out.height - strip.y0);
            strip.width = out.width;
            strip.pixelBytes = pb;
            strip.pixels = build::LargeBytes(static_cast<size_t>(strip.rows) * out.width * pb);
            if (plan->StripMask()) strip.mask = build::LargeBytes(static_cast<size_t>(strip.rows) * out.width);
            (*producer)->BeginRow(r);
            exec::ParallelFor(0, cols, 1, [&](size_t c) { (*producer)->Produce(static_cast<uint32_t>(c), strip); });
            (*producer)->EndRow(r);
            // Pixel-interleaved strip -> band planes
            for (uint32_t y = 0; y < strip.rows; ++y) {
                const uint8_t* row = strip.Pixel(0, y);
                for (int b = 0; b < out.bands; ++b) {
                    uint8_t* dst = out.pixels.data() + static_cast<size_t>(b) * plane +
                                   (static_cast<size_t>(strip.y0 + y) * out.width) * ss;
                    for (uint32_t x = 0; x < out.width; ++x)
                        std::memcpy(dst + x * ss, row + x * pb + static_cast<size_t>(b) * ss, ss);
                }
            }
        }
    } catch (const std::exception& ex) {
        return Status(StatusCode::kInternal, ex.what());
    }
    return out;
}

}  // namespace ddbt::cog::warp
