// SPDX-License-Identifier: MPL-2.0
// ddbtWriteRaster: a GeoTIFF of 8-bit bands a host computes (cog_builder.h, WriteHostRaster).
#include <cmath>
#include <exception>
#include <new>
#include <stdexcept>
#include <string>

#include "api/api_common.h"
#include "api/build_progress.h"
#include "cog/build/cog_builder.h"
#include "cog/tiler/json_lite.h"
#include "core/util/last_error.h"
#include "ddbt/ddbt.h"

namespace {

namespace json = ddbt::cog::tiler::json;
using ddbt::api::Fail;
using ddbt::cog::build::HostRaster;
using ddbt::cog::encode::Codec;

bool Unsigned(const json::Value& v, uint32_t& out) {
    if (!v.IsNumber() || v.num < 0 || v.num > 4294967295.0 || v.num != static_cast<double>(static_cast<uint32_t>(v.num)))
        return false;
    out = static_cast<uint32_t>(v.num);
    return true;
}

// The raster of the options; returns an error message, "" when fine
std::string ParseRaster(const char* optionsJson, HostRaster& r, uint64_t& memoryBudget) {
    if (ddbt::api::IsEmpty(optionsJson)) return "Invalid optionsJson: empty";
    const auto parsed = json::Parse(optionsJson);
    if (!parsed || parsed->kind != json::Value::Kind::kObject) return "Invalid optionsJson: not a JSON object";
    bool hasWidth = false, hasHeight = false;
    for (const auto& [key, val] : parsed->obj) {
        if (key == "width") {
            if (!Unsigned(val, r.width) || r.width == 0) return "Invalid optionsJson: width";
            hasWidth = true;
        } else if (key == "height") {
            if (!Unsigned(val, r.height) || r.height == 0) return "Invalid optionsJson: height";
            hasHeight = true;
        } else if (key == "colorInterp") {
            if (val.kind != json::Value::Kind::kArray || val.arr.empty())
                return "Invalid optionsJson: colorInterp must name every band";
            for (const json::Value& v : val.arr) {
                if (v.kind != json::Value::Kind::kString) return "Invalid optionsJson: colorInterp must be names";
                r.colorInterp.push_back(v.str);
            }
        } else if (key == "blockSize") {
            if (!Unsigned(val, r.blockSize)) return "Invalid optionsJson: blockSize";
        } else if (key == "codec") {
            if (val.kind != json::Value::Kind::kString) return "Invalid optionsJson: codec";
            if (val.str == "deflate") r.codec = Codec::kDeflate;
            else if (val.str == "lzw") r.codec = Codec::kLzw;
            else if (val.str == "zstd") r.codec = Codec::kZstd;
            else return "Invalid optionsJson: codec must be deflate, lzw or zstd";
        } else if (key == "predictor") {
            uint32_t p = 0;
            if (!Unsigned(val, p) || (p != 1 && p != 2)) return "Invalid optionsJson: predictor must be 1 or 2";
            r.predictor = static_cast<int>(p);
        } else if (key == "level") {
            uint32_t l = 0;
            if (!Unsigned(val, l) || l > 22) return "Invalid optionsJson: level";
            r.level = static_cast<int>(l);
        } else if (key == "nodata") {
            if (val.IsNumber()) r.nodata = val.num;
            else if (val.kind == json::Value::Kind::kString && val.str == "nan") r.nodata = std::nan("");
            else return "Invalid optionsJson: nodata must be a number or \"nan\"";
        } else if (key == "mask") {
            if (val.kind != json::Value::Kind::kBool) return "Invalid optionsJson: mask must be a boolean";
            r.mask = val.b;
        } else if (key == "overviewFactors") {
            if (val.kind != json::Value::Kind::kArray) return "Invalid optionsJson: overviewFactors";
            for (const json::Value& v : val.arr) {
                uint32_t f = 0;
                if (!Unsigned(v, f) || f < 2) return "Invalid optionsJson: overviewFactors must be factors from 2";
                r.overviewFactors.push_back(f);
            }
        } else if (key == "geoTransform") {
            if (val.kind != json::Value::Kind::kArray || val.arr.size() != 6) return "Invalid optionsJson: geoTransform";
            for (size_t i = 0; i < 6; ++i) {
                if (!val.arr[i].IsNumber()) return "Invalid optionsJson: geoTransform";
                r.geoTransform[i] = val.arr[i].num;
            }
            r.georeferenced = true;
        } else if (key == "geoKeysFrom") {
            if (val.kind != json::Value::Kind::kString || val.str.empty()) return "Invalid optionsJson: geoKeysFrom";
            auto keys = ddbt::cog::writer::GeoKeysOfFile(val.str);
            if (!keys.ok()) return "Cannot read the GeoKeys of " + val.str + ": " + std::string(keys.status().message());
            r.geoKeys = std::move(*keys);
        } else if (key == "memoryMb") {
            uint32_t mb = 0;
            if (!Unsigned(val, mb) || mb == 0) return "Invalid optionsJson: memoryMb";
            memoryBudget = uint64_t{mb} << 20;
        } else {
            return "Invalid optionsJson: unknown option " + key;
        }
    }
    if (!hasWidth || !hasHeight || r.colorInterp.empty())
        return "Invalid optionsJson: width, height and colorInterp are required";
    return std::string();
}

}  // namespace

extern "C" ddbtErr ddbtWriteRaster(const char* outputPath, const char* optionsJson, ddbtFillCallback fill,
                                   void* fillUserData, ddbtProgressCallback progress, void* progressUserData) {
    try {
        ddbt::ClearLastError();
        if (ddbt::api::IsEmpty(outputPath)) return Fail(DDBT_ERR_INVALIDARGS, "Invalid outputPath: empty");
        if (fill == nullptr) return Fail(DDBT_ERR_INVALIDARGS, "Invalid fill: NULL");
        HostRaster raster;
        ddbt::cog::build::BuildOptions options;
        if (const std::string err = ParseRaster(optionsJson, raster, options.memoryBudget); !err.empty())
            return Fail(DDBT_ERR_INVALIDARGS, err);
        raster.fill = [fill, fillUserData](uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint8_t* out, size_t stride) {
            if (fill(fillUserData, static_cast<int>(x), static_cast<int>(y), static_cast<int>(w), static_cast<int>(h),
                     out, stride) != 0)
                throw std::runtime_error("The host stopped the write");
        };
        const ddbt::api::BuildProgress forward(progress, progressUserData);
        options.progress = forward.Fn();
        const ddbt::Status s = ddbt::cog::build::WriteHostRaster(raster, outputPath, options);
        if (!s.ok()) return Fail(ddbt::api::CodeOf(s), std::string(s.message()));
        return DDBT_ERR_NONE;
    } catch (const std::bad_alloc&) {
        return Fail(DDBT_ERR_EXCEPTION, "Out of memory");
    } catch (const std::exception& e) {
        return Fail(DDBT_ERR_EXCEPTION, e.what());
    } catch (...) {
        return Fail(DDBT_ERR_EXCEPTION, "unknown exception");
    }
}
