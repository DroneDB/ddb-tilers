// SPDX-License-Identifier: MPL-2.0
// The raster exports of the C API for a host that reads rasters itself (a GeoTIFF or COG, georeferenced
// or not, in any CRS): ddbtRasterOpen / ddbtRasterClose, ddbtRasterInfo (what GDAL reports of the file),
// ddbtRasterMetadata (its metadata domains)
// and ddbtRasterRead (a window at a resolution, as GDAL's RasterIO reads it with nearest resampling).
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "api/api_common.h"
#include "cog/tiler/cog_source.h"
#include "cog/tiler/json_lite.h"
#include "cog/tiler/raster_io.h"
#include "cog/tiler/tile_error.h"
#include "core/util/half.h"
#include "ddbt/ddbt.h"

struct ddbtRaster {
    std::shared_ptr<const ddbt::cog::tiler::CogSource> source;
    ddbt::cog::tiler::RasterView view;
    explicit ddbtRaster(std::shared_ptr<const ddbt::cog::tiler::CogSource> s)
        : source(std::move(s)), view(*source) {}
};

namespace {

namespace json = ddbt::cog::tiler::json;
using ddbt::api::Fail;
using ddbt::cog::tiler::FailInvalid;

const char* MaskKindName(ddbt::cog::tiler::MaskKind kind) noexcept {
    using ddbt::cog::tiler::MaskKind;
    switch (kind) {
        case MaskKind::kPerDataset: return "perDataset";
        case MaskKind::kNodata: return "nodata";
        case MaskKind::kAlpha: return "alpha";
        default: return "allValid";
    }
}

json::Value Strings(const std::vector<std::string>& items) {
    json::Value v = json::Array();
    for (const auto& s : items) v.arr.push_back(json::String(s));
    return v;
}

// The samples of `type` converted as GDALCopyWords converts them: integers exactly, a double to the
// nearest float (beyond the float range, an infinity of its sign)
template <typename T, typename Out>
void ConvertAs(const uint8_t* in, size_t count, Out* out) {
    for (size_t i = 0; i < count; ++i) {
        T v;
        std::memcpy(&v, in + i * sizeof(T), sizeof(T));
        if constexpr (std::is_same_v<T, double> && std::is_same_v<Out, float>) {
            constexpr double kMax = std::numeric_limits<float>::max();
            out[i] = v > kMax    ? std::numeric_limits<float>::infinity()
                     : v < -kMax ? -std::numeric_limits<float>::infinity()
                                 : static_cast<float>(v);
        } else {
            out[i] = static_cast<Out>(v);
        }
    }
}

template <typename Out>
void ConvertSamples(ddbt::cog::tiler::PixelType type, const uint8_t* in, size_t count, Out* out) {
    using ddbt::cog::tiler::PixelType;
    switch (type) {
        case PixelType::kU8: ConvertAs<uint8_t>(in, count, out); break;
        case PixelType::kI8: ConvertAs<int8_t>(in, count, out); break;
        case PixelType::kU16: ConvertAs<uint16_t>(in, count, out); break;
        case PixelType::kI16: ConvertAs<int16_t>(in, count, out); break;
        case PixelType::kU32: ConvertAs<uint32_t>(in, count, out); break;
        case PixelType::kI32: ConvertAs<int32_t>(in, count, out); break;
        case PixelType::kF32: ConvertAs<float>(in, count, out); break;
        case PixelType::kF64: ConvertAs<double>(in, count, out); break;
        case PixelType::kI64: ConvertAs<int64_t>(in, count, out); break;
        case PixelType::kU64: ConvertAs<uint64_t>(in, count, out); break;
        case PixelType::kF16:
            for (size_t i = 0; i < count; ++i) {
                uint16_t h;
                std::memcpy(&h, in + i * 2, 2);
                out[i] = static_cast<Out>(ddbt::util::HalfToFloat(h));
            }
            break;
    }
}

}  // namespace

ddbtErr ddbtRasterOpen(const char* path, ddbtRaster** out) { return ddbtRasterOpenEx(path, 0, out); }

ddbtErr ddbtRasterOpenEx(const char* path, unsigned int flags, ddbtRaster** out) {
    if (out == nullptr) return Fail(DDBT_ERR_INVALIDARGS, "Invalid out: NULL");
    *out = nullptr;
    return ddbt::api::GuardedCall([&]() -> ddbtErr {
        if (ddbt::api::IsEmpty(path)) return Fail(DDBT_ERR_INVALIDARGS, "Invalid path: empty");
        if ((flags & ~DDBT_RASTER_OPEN_PRIVATE) != 0) return Fail(DDBT_ERR_INVALIDARGS, "Invalid flags");
        using ddbt::cog::tiler::CogSource;
        using ddbt::cog::tiler::SourceUse;
        // A private source is this handle's alone: its file closes with the last reference to it
        auto source = (flags & DDBT_RASTER_OPEN_PRIVATE) != 0 ? CogSource::Open(path, SourceUse::kRead)
                                                              : CogSource::Get(path, SourceUse::kRead);
        *out = new ddbtRaster(std::move(source));
        return DDBT_ERR_NONE;
    });
}

ddbtErr ddbtRasterClose(ddbtRaster* raster) {
    delete raster;
    return DDBT_ERR_NONE;
}

ddbtErr ddbtRasterInfo(const ddbtRaster* raster, char** outJson) {
    if (raster == nullptr || outJson == nullptr)
        return Fail(DDBT_ERR_INVALIDARGS, "Invalid raster or outJson: NULL");
    *outJson = nullptr;
    return ddbt::api::GuardedCall([&]() -> ddbtErr {
        namespace tl = ddbt::cog::tiler;
        const tl::CogSource& s = *raster->source;
        const auto& main = s.Reader().Ifd(s.Levels()[0].ifd);
        json::Value o = json::Object();
        o.obj["width"] = json::Unsigned(static_cast<uint64_t>(s.Width()));
        o.obj["height"] = json::Unsigned(static_cast<uint64_t>(s.Height()));
        o.obj["type"] = json::String(tl::PixelTypeName(s.Type()));
        o.obj["pixelInterleaved"] = json::Bool(s.PixelInterleaved());
        o.obj["tiled"] = json::Bool(main.tiled);
        o.obj["blockWidth"] = json::Unsigned(main.blockWidth);
        o.obj["blockHeight"] = json::Unsigned(main.blockHeight);
        json::Value overviews = json::Array();
        for (size_t i = 1; i < s.Levels().size(); ++i) {
            json::Value wh = json::Array();
            wh.arr.push_back(json::Unsigned(s.Levels()[i].width));
            wh.arr.push_back(json::Unsigned(s.Levels()[i].height));
            overviews.arr.push_back(std::move(wh));
        }
        o.obj["overviews"] = std::move(overviews);
        o.obj["mask"] = json::String(MaskKindName(s.Mask()));
        o.obj["alphaBand"] = json::Unsigned(static_cast<uint64_t>(s.AlphaBand()));
        if (s.HasGeoTransform()) {
            json::Value gt = json::Array();
            for (const double v : s.GeoTransform()) gt.arr.push_back(json::Number(v));
            o.obj["geoTransform"] = std::move(gt);
        }
        if (s.Georef().crs)
            o.obj["wkt"] = json::String(s.Georef().crs->ToWkt(ddbt::crs::WktFlavor::GdalDefault));
        json::Value bands = json::Array();
        for (int b = 1; b <= s.BandCount(); ++b) {
            const tl::BandMeta& m = s.Band(b);
            json::Value band = json::Object();
            band.obj["colorInterp"] = json::String(tl::ColorInterpName(m.interp));
            if (m.nodata) band.obj["nodata"] = json::Number(*m.nodata);
            // The colour table of band 1, as GDAL gives it
            if (b == 1)
                if (const auto table = ddbt::cog::tiff::GdalColorTable(main)) band.obj["colorTable"] = ddbt::api::ColorTableJson(*table);
            if (m.hasScale) band.obj["scale"] = json::Number(m.scale);
            if (m.hasOffset) band.obj["offset"] = json::Number(m.offset);
            if (!m.unitType.empty()) band.obj["unit"] = json::String(m.unitType);
            if (!m.description.empty()) band.obj["description"] = json::String(m.description);
            band.obj["metadata"] = Strings(m.metadata);
            bands.arr.push_back(std::move(band));
        }
        o.obj["bands"] = std::move(bands);
        return ddbt::api::CopyOut(json::Dump(o, 0), outJson);
    });
}

namespace {

// The domains that hold items, in the order GDAL lists them, as [{"name", "items"}]
json::Value DomainsJson(const ddbt::cog::tiff::MetadataDomains& domains) {
    json::Value out = json::Array();
    for (const auto& d : domains.All()) {
        if (d.items.empty()) continue;
        json::Value domain = json::Object();
        domain.obj["name"] = json::String(d.name);
        domain.obj["items"] = Strings(d.items);
        out.arr.push_back(std::move(domain));
    }
    return out;
}

}  // namespace

ddbtErr ddbtRasterMetadata(const ddbtRaster* raster, char** outJson) {
    if (raster == nullptr || outJson == nullptr)
        return Fail(DDBT_ERR_INVALIDARGS, "Invalid raster or outJson: NULL");
    *outJson = nullptr;
    return ddbt::api::GuardedCall([&]() -> ddbtErr {
        const ddbt::cog::tiff::GdalMetadata& md = raster->source->Metadata();
        json::Value o = json::Object();
        o.obj["domains"] = DomainsJson(md.dataset);
        json::Value bands = json::Array();
        for (const auto& band : md.bands) {
            json::Value b = json::Object();
            b.obj["domains"] = DomainsJson(band.domains);
            bands.arr.push_back(std::move(b));
        }
        o.obj["bands"] = std::move(bands);
        return ddbt::api::CopyOut(json::Dump(o, 0), outJson);
    });
}

namespace {

namespace tl = ddbt::cog::tiler;

// Checks the bands and the sample type of a read
void CheckRead(const ddbtRaster* raster, const int* bands, int bandCount, ddbtSampleType type) {
    if (bands == nullptr || bandCount <= 0) FailInvalid("Invalid bands");
    for (int i = 0; i < bandCount; ++i)
        if (bands[i] < 1 || bands[i] > raster->source->BandCount())
            FailInvalid("Invalid band " + std::to_string(bands[i]));
    if (type != DDBT_SAMPLE_NATIVE && type != DDBT_SAMPLE_FLOAT32 && type != DDBT_SAMPLE_FLOAT64)
        FailInvalid("Invalid sample type");
}

// The samples of `bands` at the plan, converted to `type`, and the dataset mask
void ReadPlanned(const ddbtRaster* raster, const tl::SamplePlan& plan, const int* bands, int bandCount,
                 ddbtSampleType type, void* out, uint8_t* mask) {
    const tl::RasterView& view = raster->view;
    const size_t pixels = static_cast<size_t>(plan.bufW) * static_cast<size_t>(plan.bufH);
    const size_t native = tl::PixelTypeSize(view.Type());
    const std::span<const int> bandList(bands, static_cast<size_t>(bandCount));
    const tl::PixelType nativeType = view.Type();
    const bool asStored = type == DDBT_SAMPLE_NATIVE ||
                          (type == DDBT_SAMPLE_FLOAT32 && nativeType == tl::PixelType::kF32) ||
                          (type == DDBT_SAMPLE_FLOAT64 && nativeType == tl::PixelType::kF64);
    if (asStored) {
        // The samples go where they belong, with no copy
        tl::GatherBands(view, plan, bandList, static_cast<uint8_t*>(out), pixels * native);
    } else {
        std::vector<uint8_t> samples(pixels * native * static_cast<size_t>(bandCount));
        tl::GatherBands(view, plan, bandList, samples.data(), pixels * native);
        const size_t count = pixels * static_cast<size_t>(bandCount);
        if (type == DDBT_SAMPLE_FLOAT32)
            ConvertSamples(nativeType, samples.data(), count, static_cast<float*>(out));
        else
            ConvertSamples(nativeType, samples.data(), count, static_cast<double*>(out));
    }
    if (mask != nullptr) {
        // GDAL's mask band of band 1 when it is the dataset's own (GMF_PER_DATASET): an internal mask wins over an
        // alpha band (RasterView::HasDatasetMask, the tiles' rule, leaves it to the alpha band)
        if (view.Source().Mask() == tl::MaskKind::kPerDataset) tl::GatherDatasetMask(view, plan, mask);
        else std::memset(mask, 255, pixels);
    }
}

}  // namespace

ddbtErr ddbtRasterRead(const ddbtRaster* raster, int x, int y, int width, int height, int bufWidth,
                       int bufHeight, const int* bands, int bandCount, ddbtSampleType type, void* out,
                       uint8_t* mask) {
    if (raster == nullptr || out == nullptr) return Fail(DDBT_ERR_INVALIDARGS, "Invalid raster or out: NULL");
    return ddbt::api::GuardedCall([&]() -> ddbtErr {
        const tl::RasterView& view = raster->view;
        if (width <= 0 || height <= 0 || bufWidth <= 0 || bufHeight <= 0)
            FailInvalid("Invalid window or buffer size");
        // GDAL refuses a window that is not inside the raster (Access window out of range)
        if (x < 0 || y < 0 || static_cast<int64_t>(x) + width > view.Width() ||
            static_cast<int64_t>(y) + height > view.Height())
            FailInvalid("Access window out of range in RasterIO()");
        CheckRead(raster, bands, bandCount, type);
        ReadPlanned(raster, tl::PlanRead(view, x, y, width, height, bufWidth, bufHeight), bands, bandCount, type,
                    out, mask);
        return DDBT_ERR_NONE;
    });
}

ddbtErr ddbtRasterReadLevel(const ddbtRaster* raster, int level, int x, int y, int width, int height,
                            const int* bands, int bandCount, ddbtSampleType type, void* out, uint8_t* mask) {
    if (raster == nullptr || out == nullptr) return Fail(DDBT_ERR_INVALIDARGS, "Invalid raster or out: NULL");
    return ddbt::api::GuardedCall([&]() -> ddbtErr {
        const tl::RasterView& view = raster->view;
        const auto& levels = view.Levels();
        if (level < 0 || static_cast<size_t>(level) >= levels.size())
            FailInvalid("Invalid level " + std::to_string(level));
        const auto [levelWidth, levelHeight] = levels[static_cast<size_t>(level)];
        if (width <= 0 || height <= 0) FailInvalid("Invalid window size");
        if (x < 0 || y < 0 || static_cast<int64_t>(x) + width > levelWidth ||
            static_cast<int64_t>(y) + height > levelHeight)
            FailInvalid("Access window out of range in RasterIO()");
        CheckRead(raster, bands, bandCount, type);
        ReadPlanned(raster, tl::PlanLevelRead(view, level, x, y, width, height), bands, bandCount, type, out, mask);
        return DDBT_ERR_NONE;
    });
}
