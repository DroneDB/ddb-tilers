// SPDX-License-Identifier: MPL-2.0
// The point cloud exports for a host that reads point clouds itself: ddbtPointCloudInfo (the summary of
// copc/readers/point_info.h as JSON) and ddbtReadPoints (the points in file order,
// copc/readers/point_stream.h).
#include <string>

#include "api/api_common.h"
#include "cog/tiler/json_lite.h"
#include "copc/readers/point_info.h"
#include "copc/readers/point_stream.h"
#include "core/exec/thread_pool.h"
#include "ddbt/ddbt.h"

ddbtErr ddbtPointCloudInfo(const char* inputPath, bool readBounds, char** outJson) {
    namespace json = ddbt::cog::tiler::json;
    using ddbt::api::Fail;
    if (outJson == nullptr) return Fail(DDBT_ERR_INVALIDARGS, "Invalid outJson: NULL");
    return ddbt::api::GuardedCall([&]() -> ddbtErr {
        *outJson = nullptr;
        if (ddbt::api::IsEmpty(inputPath)) return Fail(DDBT_ERR_INVALIDARGS, "Invalid inputPath: empty");
        const std::string in(inputPath);
        if (!ddbt::api::PathExists(in)) return Fail(DDBT_ERR_NOTFOUND, in + " does not exist");
        auto summary = ddbt::copc::readers::SummarizePointCloud(in, ddbt::exec::DefaultPool(), readBounds);
        if (!summary.ok())
            return Fail(ddbt::api::CodeOf(summary.status()), std::string(summary.status().message()));

        json::Value out = json::Object();
        out.obj["kind"] = json::String(summary->kind);
        out.obj["pointCount"] = json::Unsigned(summary->pointCount);
        out.obj["wkt"] = json::String(summary->wkt);
        json::Value dimensions;
        dimensions.kind = json::Value::Kind::kArray;
        for (const std::string& d : summary->dimensions) dimensions.arr.push_back(json::String(d));
        out.obj["dimensions"] = std::move(dimensions);
        if (summary->hasBounds) {
            json::Value bounds;
            bounds.kind = json::Value::Kind::kArray;
            for (const double v : summary->min) bounds.arr.push_back(json::Number(v));
            for (const double v : summary->max) bounds.arr.push_back(json::Number(v));
            out.obj["bounds"] = std::move(bounds);
        }
        return ddbt::api::CopyOut(json::Dump(out, 0), outJson);
    });
}

ddbtErr ddbtReadPoints(const char* inputPath, ddbtPointBatchCallback batch, void* userData) {
    using ddbt::api::Fail;
    if (batch == nullptr) return Fail(DDBT_ERR_INVALIDARGS, "Invalid batch callback: NULL");
    return ddbt::api::GuardedCall([&]() -> ddbtErr {
        if (ddbt::api::IsEmpty(inputPath)) return Fail(DDBT_ERR_INVALIDARGS, "Invalid inputPath: empty");
        const std::string in(inputPath);
        if (!ddbt::api::PathExists(in)) return Fail(DDBT_ERR_NOTFOUND, in + " does not exist");
        const ddbt::Status s = ddbt::copc::readers::ForEachPoint(
            in, ddbt::exec::DefaultPool(), [&](size_t count, const double* xyz, const uint16_t* rgb) {
                return batch(static_cast<uint64_t>(count), xyz, rgb, userData) == 0;
            });
        if (!s.ok()) return Fail(ddbt::api::CodeOf(s), std::string(s.message()));
        return DDBT_ERR_NONE;
    });
}
