// SPDX-License-Identifier: MPL-2.0
#include "cog/build/derived_source.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <utility>
#include <vector>

namespace ddbt::cog::build {

namespace {

class GcpSource final : public ForwardingSource {
public:
    GcpSource(std::unique_ptr<RasterSource> inner, tiff::Georeferencing georef)
        : ForwardingSource(std::move(inner)), georef_(std::move(georef)) {}

    const tiff::Georeferencing& Georef() const noexcept override { return georef_; }
    // The GCPs replace whatever georeferenced the source.
    bool IsOptimizedCog() const override { return false; }
    bool HasRpc() const noexcept override { return false; }

private:
    tiff::Georeferencing georef_;
};

class ScaledSource final : public ForwardingSource {
public:
    ScaledSource(std::unique_ptr<RasterSource> inner, uint32_t width, uint32_t height)
        : ForwardingSource(std::move(inner)), width_(width), height_(height) {
        const RasterSource& in = Inner();
        // GDALRasterBand::IRasterIO's nearest neighbour over the whole raster: the columns by an
        // accumulated increment, the rows by a product, both with a 1e-10 epsilon, clamped.
        constexpr double kEps = 1e-10;
        const double dx = static_cast<double>(in.Width()) / width_;
        dy_ = static_cast<double>(in.Height()) / height_;
        columns_.resize(width_);
        double x = 0.5 * dx + kEps;
        for (uint32_t i = 0; i < width_; ++i, x += dx)
            columns_[i] = static_cast<uint32_t>(std::min(std::max(0.0, x), static_cast<double>(in.Width() - 1)));
        const size_t rowBytes = static_cast<size_t>(width_) * in.PixelBytes();
        blockHeight_ =
            static_cast<uint32_t>(std::clamp<size_t>((size_t{4} << 20) / std::max<size_t>(rowBytes, 1), 1, 256));
        blockHeight_ = std::min(blockHeight_, height_);
    }

    uint32_t Width() const noexcept override { return width_; }
    uint32_t Height() const noexcept override { return height_; }
    uint32_t BlockWidth() const noexcept override { return width_; }
    uint32_t BlockHeight() const noexcept override { return blockHeight_; }
    bool IsOptimizedCog() const override { return false; }
    // Its blocks are made here, not kept.
    const uint8_t* ResidentBlock(uint32_t, uint32_t) const override { return nullptr; }
    const uint8_t* ResidentMaskBlock(uint32_t, uint32_t) const override { return nullptr; }

    Expected<uint32_t> ReadBlock(uint32_t col, uint32_t row, uint8_t* out) const override {
        if (col != 0 || row >= BlocksDown()) return Status(StatusCode::kOutOfRange, "block out of range");
        const RasterSource& in = Inner();
        const size_t pb = in.PixelBytes();
        return Resample(row, pb, out, [&](uint32_t bx, uint32_t by, uint8_t* block) -> Expected<uint32_t> {
            return in.ReadBlock(bx, by, block);
        });
    }

    Status ReadMaskBlock(uint32_t col, uint32_t row, uint8_t* out) const override {
        if (col != 0 || row >= BlocksDown()) return Status(StatusCode::kOutOfRange, "block out of range");
        const RasterSource& in = Inner();
        auto r = Resample(row, 1, out, [&](uint32_t bx, uint32_t by, uint8_t* block) -> Expected<uint32_t> {
            DDBT_TRY(in.ReadMaskBlock(bx, by, block));
            return std::min(in.BlockHeight(), in.Height() - by * in.BlockHeight());
        });
        return r.ok() ? Status::Ok() : r.status();
    }

private:
    // The rows of output block `row` (pb bytes per pixel) from the source rows they take, read
    // a row of source blocks at a time through `read`.
    template <typename Read>
    Expected<uint32_t> Resample(uint32_t row, size_t pb, uint8_t* out, Read read) const {
        const RasterSource& in = Inner();
        const uint32_t bw = in.BlockWidth(), bh = in.BlockHeight(), across = in.BlocksAcross();
        const uint32_t y0 = row * blockHeight_, rows = std::min(blockHeight_, height_ - y0);
        std::vector<uint8_t> blocks(static_cast<size_t>(across) * bw * bh * pb);
        std::vector<uint8_t> line(static_cast<size_t>(in.Width()) * pb);
        int64_t loaded = -1;
        int64_t lineOf = -1;
        constexpr double kEps = 1e-10;
        for (uint32_t y = 0; y < rows; ++y) {
            const double sy = (static_cast<double>(y0 + y) + 0.5) * dy_ + kEps;
            const auto srcY = static_cast<uint32_t>(std::min(std::max(0.0, sy), static_cast<double>(in.Height() - 1)));
            const uint32_t by = srcY / bh;
            if (static_cast<int64_t>(by) != loaded) {
                for (uint32_t bx = 0; bx < across; ++bx) {
                    auto r = read(bx, by, blocks.data() + static_cast<size_t>(bx) * bw * bh * pb);
                    if (!r.ok()) return r.status();
                }
                loaded = by;
                lineOf = -1;
            }
            if (static_cast<int64_t>(srcY) != lineOf) {
                // The source row, contiguous.
                for (uint32_t bx = 0; bx < across; ++bx) {
                    const uint32_t x0 = bx * bw, n = std::min(bw, in.Width() - x0);
                    std::memcpy(line.data() + static_cast<size_t>(x0) * pb,
                                blocks.data() + (static_cast<size_t>(bx) * bh + (srcY - by * bh)) * bw * pb,
                                static_cast<size_t>(n) * pb);
                }
                lineOf = srcY;
            }
            uint8_t* dst = out + static_cast<size_t>(y) * width_ * pb;
            for (uint32_t x = 0; x < width_; ++x)
                std::memcpy(dst + static_cast<size_t>(x) * pb, line.data() + static_cast<size_t>(columns_[x]) * pb, pb);
        }
        return rows;
    }

    uint32_t width_, height_;
    uint32_t blockHeight_ = 1;
    double dy_ = 1;
    std::vector<uint32_t> columns_;
};

}  // namespace

Expected<std::unique_ptr<RasterSource>> WithGcps(std::unique_ptr<RasterSource> source, std::vector<crs::Gcp> gcps,
                                                 crs::Crs gcpCrs, std::string crsDefinition) {
    const tiff::Georeferencing& own = source->Georef();
    if (own.crs && (own.geoTransform || !own.gcps.empty()))
        return Status(StatusCode::kInvalidArgument,
                      "Cannot georeference " + source->Path() + " by GCPs: it is georeferenced already");
    tiff::Georeferencing g;
    for (const crs::Gcp& p : gcps) g.gcps.push_back({p.pixel, p.line, p.x, p.y, p.z});
    g.crs = std::move(gcpCrs);
    g.crsDefinition = std::move(crsDefinition);
    return std::unique_ptr<RasterSource>(new GcpSource(std::move(source), std::move(g)));
}

Expected<std::unique_ptr<RasterSource>> ScaledNearest(std::unique_ptr<RasterSource> source, uint32_t width,
                                                      uint32_t height) {
    if (width == 0 || height == 0) return Status(StatusCode::kInvalidArgument, "Invalid output size");
    if (width == source->Width() && height == source->Height()) return source;
    return std::unique_ptr<RasterSource>(new ScaledSource(std::move(source), width, height));
}

}  // namespace ddbt::cog::build
