// SPDX-License-Identifier: MPL-2.0
// The COG build exports of the C API: ddbtBuildCog and ddbtBuildCog2 (with progress), DroneDB's
// buildCog contract (cog.cpp).
//
// optionsJson (all optional): {"codec": "lzw" | "deflate" | "zstd" (non-JPEG outputs),
// "level": n (DEFLATE / ZSTD level), "jpegQuality": 1..100 (warped JPEG outputs; default
// DDB_COG_JPEG_QUALITY or 75), "noJpeg": bool, "memoryMb": n (pipeline budget, 2048),
// "statistics": bool (write "<output>.stats.json", true), "evenTileOrigin": bool (false),
// "insertRange": bool (true), "report": "<path>" (a JSON report of the build is written there)}.
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <new>
#include <string>

#include "cog/build/cog_builder.h"
#include "cog/tiler/json_lite.h"
#include "api/api_common.h"
#include "api/build_progress.h"
#include "core/util/last_error.h"
#include "ddbt/ddbt.h"

namespace {

namespace json = ddbt::cog::tiler::json;
using ddbt::cog::build::BuildOptions;
using ddbt::cog::build::BuildReport;

using ddbt::api::Fail;
using ddbt::api::IsEmpty;

// Parses the options; returns an error message, "" when fine.
std::string ParseOptions(const char* text, BuildOptions& o, std::string& reportPath) {
    if (IsEmpty(text)) return "";
    const auto v = json::Parse(text);
    if (!v || !v->IsObject()) return "Invalid optionsJson: not a JSON object";
    for (const auto& [key, val] : v->obj) {
        if (key == "codec") {
            if (val.kind != json::Value::Kind::kString) return "Invalid optionsJson: codec must be a string";
            if (val.str == "lzw" || val.str == "LZW") o.losslessCodec = ddbt::cog::encode::Codec::kLzw;
            else if (val.str == "deflate" || val.str == "DEFLATE") o.losslessCodec = ddbt::cog::encode::Codec::kDeflate;
            else if (val.str == "zstd" || val.str == "ZSTD") o.losslessCodec = ddbt::cog::encode::Codec::kZstd;
            else return "Invalid optionsJson: unknown codec " + val.str;
        } else if (key == "level" || key == "jpegQuality" || key == "memoryMb") {
            if (!val.isUint) return "Invalid optionsJson: " + key + " must be a non-negative integer";
            if (key == "level") o.level = static_cast<int>(val.u);
            if (key == "jpegQuality") {
                if (val.u < 1 || val.u > 100) return "Invalid optionsJson: jpegQuality must be 1 to 100";
                o.jpegQuality = static_cast<int>(val.u);
            }
            if (key == "memoryMb") o.memoryBudget = val.u << 20;
        } else if (key == "noJpeg" || key == "statistics" || key == "evenTileOrigin" || key == "insertRange") {
            if (val.kind != json::Value::Kind::kBool) return "Invalid optionsJson: " + key + " must be a boolean";
            if (key == "noJpeg") o.noJpeg = val.b;
            if (key == "statistics") o.statistics = val.b;
            if (key == "evenTileOrigin") o.evenTileOrigin = val.b;
            if (key == "insertRange") o.writer.insertRange = val.b;
        } else if (key == "report") {
            if (val.kind != json::Value::Kind::kString) return "Invalid optionsJson: report must be a string";
            reportPath = val.str;
        } else {
            return "Invalid optionsJson: unknown option " + key;
        }
    }
    return "";
}

void WriteReport(const std::string& path, const BuildReport& r) {
    json::Value j = json::Object();
    j.obj["ownGrid"] = json::Bool(r.ownGrid);
    j.obj["width"] = json::Unsigned(r.width);
    j.obj["height"] = json::Unsigned(r.height);
    j.obj["levels"] = json::Unsigned(r.levels);
    j.obj["bands"] = json::Unsigned(static_cast<uint64_t>(r.bands));
    j.obj["codec"] = json::String(r.codec);
    j.obj["jpegQuality"] = json::Unsigned(static_cast<uint64_t>(r.jpegQuality));
    j.obj["transparency"] = json::String(r.transparency);
    j.obj["fileSize"] = json::Unsigned(r.file.fileSize);
    j.obj["prefixBytes"] = json::Unsigned(r.file.prefixBytes);
    j.obj["fullResBytes"] = json::Unsigned(r.file.fullResBytes);
    j.obj["overviewBytes"] = json::Unsigned(r.file.overviewBytes);
    j.obj["spilledBytes"] = json::Unsigned(r.file.spilledBytes);
    j.obj["sparseBlocks"] = json::Unsigned(r.file.sparseBlocks);
    j.obj["bigTiff"] = json::Bool(r.file.bigTiff);
    j.obj["insertedRange"] = json::Bool(r.file.insertedRange);
    j.obj["seconds"] = json::Number(r.secondsTotal);
    j.obj["peakStripBytes"] = json::Unsigned(r.peakStripBytes);
    j.obj["sourceBlocksDecoded"] = json::Unsigned(r.sourceBlocksDecoded);
    j.obj["peakSourceCacheBytes"] = json::Unsigned(r.peakSourceCacheBytes);
    std::ofstream out(path, std::ios::trunc | std::ios::binary);
    out << json::Dump(j, 2);
}

}  // namespace

extern "C" ddbtErr ddbtBuildCog2(const char* inputPath, const char* outputPath, const char* optionsJson,
                                 ddbtProgressCallback progress, void* userData) {
    try {
        ddbt::ClearLastError();
        if (IsEmpty(inputPath)) return Fail(DDBT_ERR_INVALIDARGS, "Invalid inputPath: empty");
        if (IsEmpty(outputPath)) return Fail(DDBT_ERR_INVALIDARGS, "Invalid outputPath: empty");
        BuildOptions options;
        std::string reportPath;
        if (const std::string err = ParseOptions(optionsJson, options, reportPath); !err.empty())
            return Fail(DDBT_ERR_INVALIDARGS, err);
        const std::string in(inputPath);
        if (!ddbt::api::PathExists(in)) return Fail(DDBT_ERR_NOTFOUND, in + " does not exist");
        const ddbt::api::BuildProgress forward(progress, userData);
        options.progress = forward.Fn();
        BuildReport report;
        const ddbt::Status s = ddbt::cog::build::BuildCog(inputPath, outputPath, options, &report);
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

extern "C" ddbtErr ddbtBuildCog(const char* inputPath, const char* outputPath, const char* optionsJson) {
    return ddbtBuildCog2(inputPath, outputPath, optionsJson, nullptr, nullptr);
}
