// SPDX-License-Identifier: MPL-2.0
// The raster exports of the C API for a host that reads rasters itself (a GeoTIFF or COG, georeferenced
// or not, in any CRS): ddbtRasterOpen / ddbtRasterClose, ddbtRasterInfo (what GDAL reports of the file)
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

json::Value Array() {
    json::Value v;
    v.kind = json::Value::Kind::kArray;
    return v;
}

json::Value Strings(const std::vector<std::string>& items) {
    json::Value v = Array();
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
    }
}

}  // namespace

ddbtErr ddbtRasterOpen(const char* path, ddbtRaster** out) {
    if (out == nullptr) return Fail(DDBT_ERR_INVALIDARGS, "Invalid out: NULL");
    *out = nullptr;
    return ddbt::api::GuardedCall([&]() -> ddbtErr {
        if (ddbt::api::IsEmpty(path)) return Fail(DDBT_ERR_INVALIDARGS, "Invalid path: empty");
        auto source = ddbt::cog::tiler::CogSource::Get(path, ddbt::cog::tiler::SourceUse::kRead);
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
        json::Value overviews = Array();
        for (size_t i = 1; i < s.Levels().size(); ++i) {
            json::Value wh = Array();
            wh.arr.push_back(json::Unsigned(s.Levels()[i].width));
            wh.arr.push_back(json::Unsigned(s.Levels()[i].height));
            overviews.arr.push_back(std::move(wh));
        }
        o.obj["overviews"] = std::move(overviews);
        o.obj["mask"] = json::String(MaskKindName(s.Mask()));
        o.obj["alphaBand"] = json::Unsigned(static_cast<uint64_t>(s.AlphaBand()));
        if (s.HasGeoTransform()) {
            json::Value gt = Array();
            for (const double v : s.GeoTransform()) gt.arr.push_back(json::Number(v));
            o.obj["geoTransform"] = std::move(gt);
        }
        if (s.Georef().crs)
            o.obj["wkt"] = json::String(s.Georef().crs->ToWkt(ddbt::crs::WktFlavor::GdalDefault));
        json::Value bands = Array();
        for (int b = 1; b <= s.BandCount(); ++b) {
            const tl::BandMeta& m = s.Band(b);
            json::Value band = json::Object();
            band.obj["colorInterp"] = json::String(tl::ColorInterpName(m.interp));
            if (m.nodata) band.obj["nodata"] = json::Number(*m.nodata);
            if (m.hasScale) band.obj["scale"] = json::Number(m.scale);
            if (m.hasOffset) band.obj["offset"] = json::Number(m.offset);
            if (!m.unitType.empty()) band.obj["unit"] = json::String(m.unitType);
            if (!m.description.empty()) band.obj["description"] = json::String(m.description);
            band.obj["metadata"] = Strings(m.metadata);
            bands.arr.push_back(std::move(band));
        }
        o.obj["bands"] = std::move(bands);
        o.obj["metadata"] = Strings(s.DatasetMetadata());
        return ddbt::api::CopyOut(json::Dump(o, 0), outJson);
    });
}

ddbtErr ddbtRasterRead(const ddbtRaster* raster, int x, int y, int width, int height, int bufWidth,
                       int bufHeight, const int* bands, int bandCount, ddbtSampleType type, void* out,
                       uint8_t* mask) {
    if (raster == nullptr || out == nullptr) return Fail(DDBT_ERR_INVALIDARGS, "Invalid raster or out: NULL");
    return ddbt::api::GuardedCall([&]() -> ddbtErr {
        namespace tl = ddbt::cog::tiler;
        const tl::RasterView& view = raster->view;
        if (width <= 0 || height <= 0 || bufWidth <= 0 || bufHeight <= 0)
            FailInvalid("Invalid window or buffer size");
        // GDAL refuses a window that is not inside the raster (Access window out of range)
        if (x < 0 || y < 0 || static_cast<int64_t>(x) + width > view.Width() ||
            static_cast<int64_t>(y) + height > view.Height())
            FailInvalid("Access window out of range in RasterIO()");
        if (bands == nullptr || bandCount <= 0) FailInvalid("Invalid bands");
        for (int i = 0; i < bandCount; ++i)
            if (bands[i] < 1 || bands[i] > raster->source->BandCount())
                FailInvalid("Invalid band " + std::to_string(bands[i]));
        if (type != DDBT_SAMPLE_NATIVE && type != DDBT_SAMPLE_FLOAT32 && type != DDBT_SAMPLE_FLOAT64)
            FailInvalid("Invalid sample type");

        const tl::SamplePlan plan = tl::PlanRead(view, x, y, width, height, bufWidth, bufHeight);
        const size_t pixels = static_cast<size_t>(bufWidth) * static_cast<size_t>(bufHeight);
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
            if (view.HasDatasetMask()) tl::GatherDatasetMask(view, plan, mask);
            else std::memset(mask, 255, pixels);
        }
        return DDBT_ERR_NONE;
    });
}
