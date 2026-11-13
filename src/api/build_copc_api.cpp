// SPDX-License-Identifier: MPL-2.0
// The COPC build exports of the C API: ddbtBuildCopc and ddbtBuildCopc2 (with progress), DroneDB's
// buildCopc (one or more inputs).
//
// optionsJson (all optional): {"inputs": ["more.laz", ...] (merged with inputPath),
// "threads": n (0: the default pool), "memoryMb": n (memory cap; default DDBT_COPC_BUILD_MEM_MB
// or 4096, at most half the physical memory), "tempDir": "<dir>" (spill files; default: the
// output's directory), "forceSpill": bool, "tuneAllocator": bool (glibc mmap threshold, true),
// "maxPartitionPoints": n, "metadata": bool (write the sidecar, true), "metadataPath": "<path>"
// (default "<output>.meta.json"), "report": "<path>" (a JSON report of the build)}.
#include <exception>
#include <filesystem>
#include <fstream>
#include <new>
#include <string>

#include "cog/tiler/json_lite.h"
#include "copc/build/copc_builder.h"
#include "api/api_common.h"
#include "api/build_progress.h"
#include "core/util/last_error.h"
#include "ddbt/ddbt.h"

namespace {

namespace json = ddbt::cog::tiler::json;
using ddbt::copc::build::CopcBuildOptions;
using ddbt::copc::build::CopcBuildReport;

using ddbt::api::Fail;
using ddbt::api::IsEmpty;

std::string ParseOptions(const char* text, CopcBuildOptions& o, std::string& reportPath) {
    if (IsEmpty(text)) return "";
    const auto v = json::Parse(text);
    if (!v || !v->IsObject()) return "Invalid optionsJson: not a JSON object";
    for (const auto& [key, val] : v->obj) {
        if (key == "inputs") {
            if (val.kind != json::Value::Kind::kArray) return "Invalid optionsJson: inputs must be an array of strings";
            for (const auto& s : val.arr) {
                if (s.kind != json::Value::Kind::kString || s.str.empty())
                    return "Invalid optionsJson: inputs must be an array of strings";
                o.extraInputs.push_back(s.str);
            }
        } else if (key == "threads" || key == "memoryMb" || key == "maxPartitionPoints") {
            if (!val.isUint) return "Invalid optionsJson: " + key + " must be a non-negative integer";
            if (key == "threads") o.threads = static_cast<size_t>(std::min<uint64_t>(val.u, 4096));
            if (key == "memoryMb") o.memoryBudget = val.u << 20;
            if (key == "maxPartitionPoints") o.maxPartitionPoints = val.u;
        } else if (key == "forceSpill" || key == "metadata" || key == "tuneAllocator") {
            if (val.kind != json::Value::Kind::kBool) return "Invalid optionsJson: " + key + " must be a boolean";
            if (key == "forceSpill") o.forceSpill = val.b;
            if (key == "tuneAllocator") o.tuneAllocator = val.b;
            if (key == "metadata") o.metadata = val.b;
        } else if (key == "tempDir" || key == "metadataPath" || key == "report") {
            if (val.kind != json::Value::Kind::kString) return "Invalid optionsJson: " + key + " must be a string";
            if (key == "tempDir") o.tempDir = val.str;
            if (key == "metadataPath") o.metadataPath = val.str;
            if (key == "report") reportPath = val.str;
        } else {
            return "Invalid optionsJson: unknown option " + key;
        }
    }
    return "";
}

void WriteReport(const std::string& path, const CopcBuildReport& r) {
    json::Value j = json::Object();
    j.obj["points"] = json::Unsigned(r.points);
    j.obj["nodes"] = json::Unsigned(r.nodes);
    j.obj["bins"] = json::Unsigned(r.bins);
    j.obj["partitions"] = json::Unsigned(r.partitions);
    j.obj["topNodes"] = json::Unsigned(r.topNodes);
    j.obj["maxDepth"] = json::Unsigned(static_cast<uint64_t>(r.maxDepth));
    j.obj["leafLevel"] = json::Unsigned(static_cast<uint64_t>(r.leafLevel));
    j.obj["pointFormat"] = json::Unsigned(static_cast<uint64_t>(r.format));
    j.obj["extraBytes"] = json::Unsigned(static_cast<uint64_t>(r.extraBytes));
    json::Value sc, of;
    sc.kind = of.kind = json::Value::Kind::kArray;
    for (size_t a = 0; a < 3; ++a) {
        sc.arr.push_back(json::Number(r.scale[a]));
        of.arr.push_back(json::Number(r.offset[a]));
    }
    j.obj["scale"] = sc;
    j.obj["offset"] = of;
    j.obj["spacing"] = json::Number(r.spacing);
    j.obj["halfSize"] = json::Number(r.cube.halfSize);
    j.obj["spilled"] = json::Bool(r.spilled);
    j.obj["spillBytes"] = json::Unsigned(r.spillBytes);
    j.obj["memoryBudget"] = json::Unsigned(r.memoryBudget);
    j.obj["peakStoreBytes"] = json::Unsigned(r.peakStoreBytes);
    j.obj["peakProcessBytes"] = json::Unsigned(r.peakProcessBytes);
    j.obj["maxPartitionPoints"] = json::Unsigned(r.maxPartitionPoints);
    j.obj["largestPartition"] = json::Unsigned(r.largestPartition);
    j.obj["restarted"] = json::Bool(r.restarted);
    j.obj["color16"] = json::Bool(r.color16);
    j.obj["fileSize"] = json::Unsigned(r.fileSize);
    j.obj["secondsPlan"] = json::Number(r.secondsPlan);
    j.obj["secondsScatter"] = json::Number(r.secondsScatter);
    j.obj["secondsProcess"] = json::Number(r.secondsProcess);
    j.obj["secondsCommitWait"] = json::Number(r.secondsCommitWait);
    j.obj["secondsTopNodes"] = json::Number(r.secondsTopNodes);
    j.obj["seconds"] = json::Number(r.secondsTotal);
    std::ofstream out(path, std::ios::trunc | std::ios::binary);
    out << json::Dump(j, 2);
}

}  // namespace

extern "C" ddbtErr ddbtBuildCopc2(const char* inputPath, const char* outputPath, const char* optionsJson,
                                  ddbtProgressCallback progress, void* userData) {
    try {
        ddbt::ClearLastError();
        if (IsEmpty(inputPath)) return Fail(DDBT_ERR_INVALIDARGS, "Invalid inputPath: empty");
        if (IsEmpty(outputPath)) return Fail(DDBT_ERR_INVALIDARGS, "Invalid outputPath: empty");
        CopcBuildOptions options;
        std::string reportPath;
        if (const std::string err = ParseOptions(optionsJson, options, reportPath); !err.empty())
            return Fail(DDBT_ERR_INVALIDARGS, err);
        std::vector<std::string> all{inputPath};
        all.insert(all.end(), options.extraInputs.begin(), options.extraInputs.end());
        for (const std::string& in : all)
            if (!ddbt::api::PathExists(in)) return Fail(DDBT_ERR_NOTFOUND, in + " does not exist");
        const ddbt::api::BuildProgress forward(progress, userData);
        options.progress = forward.Fn();
        CopcBuildReport report;
        const ddbt::Status s = ddbt::copc::build::BuildCopc(inputPath, outputPath, options, &report);
        if (!s.ok()) return Fail(ddbt::api::CodeOf(s), std::string(s.message()));
        if (!reportPath.empty()) WriteReport(reportPath, report);
        return DDBT_ERR_NONE;
    } catch (const std::bad_alloc&) {
        return Fail(DDBT_ERR_EXCEPTION, "Out of memory");
    } catch (const std::exception& e) {
        return Fail(DDBT_ERR_EXCEPTION, e.what());
    } catch (...) {
        return Fail(DDBT_ERR_EXCEPTION, "unknown exception");
    }
}

extern "C" ddbtErr ddbtBuildCopc(const char* inputPath, const char* outputPath, const char* optionsJson) {
    return ddbtBuildCopc2(inputPath, outputPath, optionsJson, nullptr, nullptr);
}
