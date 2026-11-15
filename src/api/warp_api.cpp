// SPDX-License-Identifier: MPL-2.0
// ddbtWarpRaster: a raster warped into memory on the host's grid (cog/warp/region_warp.h).
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <new>
#include <string>

#include "api/api_common.h"
#include "cog/tiler/json_lite.h"
#include "cog/warp/region_warp.h"
#include "core/util/last_error.h"
#include "ddbt/ddbt.h"

namespace {

namespace json = ddbt::cog::tiler::json;
using ddbt::api::Fail;
using ddbt::cog::warp::RegionRequest;

bool Size(const json::Value& v, uint32_t& out) {
    if (!v.IsNumber() || v.num < 1 || v.num > 1e6 || v.num != std::floor(v.num)) return false;
    out = static_cast<uint32_t>(v.num);
    return true;
}

std::string ParseRequest(const char* optionsJson, RegionRequest& r) {
    if (ddbt::api::IsEmpty(optionsJson)) return "Invalid optionsJson: empty";
    const auto parsed = json::Parse(optionsJson);
    if (!parsed || parsed->kind != json::Value::Kind::kObject) return "Invalid optionsJson: not a JSON object";
    bool hasExtent = false;
    for (const auto& [key, val] : parsed->obj) {
        if (key == "targetCrs" || key == "extentCrs") {
            if (val.kind != json::Value::Kind::kString) return "Invalid optionsJson: " + key + " must be a CRS";
            (key == "targetCrs" ? r.targetCrs : r.extentCrs) = val.str;
        } else if (key == "extent") {
            if (val.kind != json::Value::Kind::kArray || val.arr.size() != 4) return "Invalid optionsJson: extent";
            for (size_t i = 0; i < 4; ++i) {
                if (!val.arr[i].IsNumber()) return "Invalid optionsJson: extent";
                r.extent[i] = val.arr[i].num;
            }
            hasExtent = true;
        } else if (key == "width") {
            if (!Size(val, r.width)) return "Invalid optionsJson: width";
        } else if (key == "height") {
            if (!Size(val, r.height)) return "Invalid optionsJson: height";
        } else if (key == "resampling") {
            if (val.kind != json::Value::Kind::kString) return "Invalid optionsJson: resampling";
            if (val.str == "bilinear") r.resampling = ddbt::cog::warp::Resampling::kBilinear;
            else if (val.str == "nearest") r.resampling = ddbt::cog::warp::Resampling::kNearest;
            else return "Invalid optionsJson: resampling must be bilinear or nearest";
        } else if (key == "bands") {
            if (val.kind != json::Value::Kind::kArray) return "Invalid optionsJson: bands";
            for (const json::Value& b : val.arr) {
                if (!b.IsNumber() || b.num < 1 || b.num != std::floor(b.num)) return "Invalid optionsJson: bands";
                r.bands.push_back(static_cast<int>(b.num));
            }
        } else if (key == "dstAlpha") {
            if (val.kind != json::Value::Kind::kBool) return "Invalid optionsJson: dstAlpha";
            r.dstAlpha = val.b;
        } else if (key == "dstNodata") {
            if (!val.IsNumber()) return "Invalid optionsJson: dstNodata";
            r.dstNodata = val.num;
        } else {
            return "Invalid optionsJson: unknown option " + key;
        }
    }
    if (r.targetCrs.empty() || !hasExtent || r.width == 0 || r.height == 0)
        return "Invalid optionsJson: targetCrs, extent, width and height are required";
    return std::string();
}

}  // namespace

extern "C" ddbtErr ddbtWarpRaster(const char* inputPath, const char* optionsJson, char** outJson, uint8_t** outPixels,
                                  size_t* outSize) {
    try {
        ddbt::ClearLastError();
        if (ddbt::api::IsEmpty(inputPath)) return Fail(DDBT_ERR_INVALIDARGS, "Invalid inputPath: empty");
        if (outJson == nullptr || outPixels == nullptr || outSize == nullptr)
            return Fail(DDBT_ERR_INVALIDARGS, "Invalid output: NULL");
        RegionRequest request;
        if (const std::string err = ParseRequest(optionsJson, request); !err.empty())
            return Fail(DDBT_ERR_INVALIDARGS, err);
        if (!ddbt::api::PathExists(inputPath)) return Fail(DDBT_ERR_NOTFOUND, std::string(inputPath) + " does not exist");
        auto warped = ddbt::cog::warp::WarpRegion(inputPath, request);
        if (!warped.ok()) return Fail(ddbt::api::CodeOf(warped.status()), std::string(warped.status().message()));
        const auto& w = *warped;
        json::Value o = json::Object();
        o.obj["width"] = json::Unsigned(w.width);
        o.obj["height"] = json::Unsigned(w.height);
        o.obj["bands"] = json::Unsigned(static_cast<uint64_t>(w.bands));
        o.obj["type"] = json::String(ddbt::api::GdalTypeName(w.type));
        json::Value gt = json::Value{};
        gt.kind = json::Value::Kind::kArray;
        for (const double v : w.geoTransform) gt.arr.push_back(json::Number(v));
        o.obj["geoTransform"] = std::move(gt);
        o.obj["wkt"] = json::String(w.wkt);
        json::Value interps = json::Value{};
        interps.kind = json::Value::Kind::kArray;
        for (const auto& i : w.colorInterp) interps.arr.push_back(json::String(i));
        o.obj["colorInterp"] = std::move(interps);
        if (w.nodata) o.obj["nodata"] = json::Number(*w.nodata);
        o.obj["alphaBand"] = json::Unsigned(static_cast<uint64_t>(w.alphaBand + 1));
        o.obj["sourceLevel"] = json::Unsigned(static_cast<uint64_t>(w.sourceLevel));
        auto* pixels = static_cast<uint8_t*>(std::malloc(std::max<size_t>(1, w.pixels.size())));
        if (pixels == nullptr) return Fail(DDBT_ERR_EXCEPTION, "Out of memory");
        std::memcpy(pixels, w.pixels.data(), w.pixels.size());
        if (const ddbtErr rc = ddbt::api::CopyOut(json::Dump(o, 0), outJson); rc != DDBT_ERR_NONE) {
            std::free(pixels);
            return rc;
        }
        *outPixels = pixels;
        *outSize = w.pixels.size();
        return DDBT_ERR_NONE;
    } catch (const std::bad_alloc&) {
        return Fail(DDBT_ERR_EXCEPTION, "Out of memory");
    } catch (const std::exception& e) {
        return Fail(DDBT_ERR_EXCEPTION, e.what());
    } catch (...) {
        return Fail(DDBT_ERR_EXCEPTION, "unknown exception");
    }
}
