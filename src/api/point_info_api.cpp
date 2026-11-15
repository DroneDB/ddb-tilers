// SPDX-License-Identifier: MPL-2.0
// ddbtPointCloudInfo: the summary of a point cloud (copc/readers/point_info.h) as JSON.
#include <string>

#include "api/api_common.h"
#include "cog/tiler/json_lite.h"
#include "copc/readers/point_info.h"
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
