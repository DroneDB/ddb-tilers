// SPDX-License-Identifier: MPL-2.0
// The exports a host library uses to run ddbt inside itself: ddbtConfigure (process-wide
// settings the host owns: where proj.db is, threads, cache budgets, the COPC tile mode and build
// memory), ddbtTileInfo (the bounds and zoom range of an input, which the host's tilers expose),
// ddbtReleaseSources and ddbtSetRemoteReader (the host's range reads of remote inputs).
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>

#include "api/api_common.h"
#include "api/copc_tile.h"
#include "cog/tiler/block_cache.h"
#include "cog/tiler/cog_source.h"
#include "cog/tiler/cog_tiler.h"
#include "cog/tiler/json_lite.h"
#include "copc/build/copc_builder.h"
#include "copc/cache/node_cache.h"
#include "copc/tiler/cloud.h"
#include "copc/tiler/copc_tiler.h"
#include "core/crs/crs.h"
#include "core/exec/thread_pool.h"
#include "core/io/remote_source.h"
#include "core/util/status.h"
#include "ddbt/ddbt.h"

namespace {

namespace json = ddbt::cog::tiler::json;
using ddbt::api::Fail;
using ddbt::api::GuardedCall;

// Sizes are given in MiB; this bound keeps the byte count far from overflowing.
constexpr uint64_t kMaxMiB = uint64_t{1} << 24;

struct Config {
    std::optional<std::string> projDataDir;
    std::optional<uint64_t> threads;
    std::optional<uint64_t> blockCacheMb, tileCacheMb, copcNodeCacheMb, copcTileCacheMb, copcBuildMemMb;
    std::optional<ddbt::copc::tiler::Mode> copcTilerMode;
};

// Parses and checks every setting; returns an error message, "" when fine. Nothing is applied.
std::string Parse(const char* text, Config& c) {
    const auto v = json::Parse(text);
    if (!v || !v->IsObject()) return "Invalid configJson: not a JSON object";
    for (const auto& [key, val] : v->obj) {
        if (key == "projDataDir") {
            if (val.kind != json::Value::Kind::kString) return "Invalid configJson: projDataDir must be a string";
            c.projDataDir = val.str;
        } else if (key == "copcTilerMode") {
            const auto mode = val.kind == json::Value::Kind::kString ? ddbt::copc::tiler::ParseMode(val.str)
                                                                     : std::nullopt;
            if (!mode) return "Invalid configJson: copcTilerMode must be \"compat\", \"hybrid\" or \"modern\"";
            c.copcTilerMode = *mode;
        } else if (key == "threads") {
            if (!val.isUint || val.u > 4096) return "Invalid configJson: threads must be an integer from 0 to 4096";
            c.threads = val.u;
        } else if (key == "blockCacheMb" || key == "tileCacheMb" || key == "copcNodeCacheMb" ||
                   key == "copcTileCacheMb" || key == "copcBuildMemMb") {
            if (!val.isUint || val.u > kMaxMiB)
                return "Invalid configJson: " + key + " must be an integer from 0 to " + std::to_string(kMaxMiB);
            if (key == "blockCacheMb") c.blockCacheMb = val.u;
            if (key == "tileCacheMb") c.tileCacheMb = val.u;
            if (key == "copcNodeCacheMb") c.copcNodeCacheMb = val.u;
            if (key == "copcTileCacheMb") c.copcTileCacheMb = val.u;
            if (key == "copcBuildMemMb") c.copcBuildMemMb = val.u;
        } else {
            return "Invalid configJson: unknown setting " + key;
        }
    }
    return "";
}

size_t Bytes(uint64_t mib) { return static_cast<size_t>(mib << 20); }

// A host callback's code and reason as a Status of the remote reader.
ddbt::Status CallbackStatus(ddbtErr code, const char* error, const std::string& what) {
    if (code == DDBT_ERR_NONE) return ddbt::Status::Ok();
    std::string message = what + ": " + (error[0] != '\0' ? error : "the host's remote reader failed");
    switch (code) {
        case DDBT_ERR_NOTFOUND: return ddbt::NotFound(std::move(message));
        case DDBT_ERR_CANCELED: return ddbt::Cancelled(std::move(message));
        default: return ddbt::IoError(std::move(message));
    }
}

// The host's reader as the callbacks of io::RemoteReader.
std::shared_ptr<const ddbt::io::RemoteReader> AdaptReader(const ddbtRemoteReader& host) {
    constexpr size_t kErrorSize = 512;
    constexpr size_t kVersionSize = 256;
    auto reader = std::make_shared<ddbt::io::RemoteReader>();
    reader->stat = [host](const std::string& url, ddbt::io::RemoteStat& out) {
        char error[kErrorSize] = {};
        char version[kVersionSize] = {};
        uint64_t size = 0;
        const ddbtErr code = host.stat(url.c_str(), &size, version, kVersionSize, error, kErrorSize, host.userData);
        error[kErrorSize - 1] = '\0';
        version[kVersionSize - 1] = '\0';
        if (code != DDBT_ERR_NONE) return CallbackStatus(code, error, "Cannot stat " + url);
        out.size = size;
        out.version = version;
        return ddbt::Status::Ok();
    };
    reader->read = [host](const std::string& url, uint64_t offset, std::span<uint8_t> out) {
        char error[kErrorSize] = {};
        const ddbtErr code = host.read(url.c_str(), offset, out.data(), out.size(), error, kErrorSize, host.userData);
        error[kErrorSize - 1] = '\0';
        return CallbackStatus(code, error, "Cannot read " + url);
    };
    return reader;
}

}  // namespace

extern "C" {

ddbtErr ddbtConfigure(const char* configJson) {
    return GuardedCall([&]() -> ddbtErr {
        if (ddbt::api::IsEmpty(configJson)) return Fail(DDBT_ERR_INVALIDARGS, "Invalid configJson: empty");
        Config c;
        if (const std::string err = Parse(configJson, c); !err.empty()) return Fail(DDBT_ERR_INVALIDARGS, err);
        // The only setting that can be refused now comes first, so a refusal changes nothing.
        if (c.threads && !ddbt::exec::SetDefaultPoolThreads(static_cast<size_t>(*c.threads)))
            return Fail(DDBT_ERR_EXCEPTION, "Cannot set threads: the thread pool is already running");
        if (c.projDataDir) ddbt::crs::SetProjDataDir(*c.projDataDir);
        if (c.blockCacheMb) ddbt::cog::tiler::SetBlockCacheBudget(Bytes(*c.blockCacheMb));
        if (c.tileCacheMb) ddbt::cog::tiler::SetTileCacheBudget(Bytes(*c.tileCacheMb));
        if (c.copcNodeCacheMb) ddbt::copc::cache::NodeCache::Global().SetBudget(Bytes(*c.copcNodeCacheMb));
        if (c.copcTileCacheMb) ddbt::copc::tiler::SetTileCacheBudget(Bytes(*c.copcTileCacheMb));
        if (c.copcTilerMode) ddbt::copc::tiler::SetMode(*c.copcTilerMode);
        if (c.copcBuildMemMb) ddbt::copc::build::SetDefaultCopcMemoryBudget(*c.copcBuildMemMb << 20);
        return DDBT_ERR_NONE;
    });
}

ddbtErr ddbtReleaseSources(void) {
    return GuardedCall([&]() -> ddbtErr {
        ddbt::cog::tiler::CogSource::ClearRegistry();
        ddbt::copc::tiler::Cloud::ClearRegistry();
        return DDBT_ERR_NONE;
    });
}

ddbtErr ddbtSetRemoteReader(const ddbtRemoteReader* reader) {
    return GuardedCall([&]() -> ddbtErr {
        if (reader == nullptr) {
            ddbt::io::SetRemoteReader(nullptr);
            return DDBT_ERR_NONE;
        }
        if (reader->structSize < sizeof(ddbtRemoteReader))
            return Fail(DDBT_ERR_INVALIDARGS, "Invalid remote reader: structSize is too small");
        if (reader->stat == nullptr || reader->read == nullptr)
            return Fail(DDBT_ERR_INVALIDARGS, "Invalid remote reader: a callback is NULL");
        ddbt::io::SetRemoteReader(AdaptReader(*reader));
        return DDBT_ERR_NONE;
    });
}

ddbtErr ddbtTileInfo(const char* inputPath, int tileSize, char** outJson) {
    return GuardedCall([&]() -> ddbtErr {
        if (ddbt::api::NullOrBlank(inputPath)) return Fail(DDBT_ERR_INVALIDARGS, "No input path provided");
        if (outJson == nullptr) return Fail(DDBT_ERR_INVALIDARGS, "Invalid outJson: NULL");
        const std::string path(inputPath);
        const bool pointCloud = ddbt::api::IsCopcPath(inputPath);
        if (pointCloud)
            ddbt::api::RequireLocalOrUrl(path);
        else
            ddbt::api::RequireLocal(path);
        const ddbt::tile::compat::TileExtent e =
            pointCloud ? ddbt::copc::tiler::Extent(path, tileSize) : ddbt::cog::tiler::Extent(path, tileSize);
        json::Value j = json::Object();
        j.obj["type"] = json::String(pointCloud ? "pointcloud" : "raster");
        j.obj["minX"] = json::Number(e.minX);
        j.obj["minY"] = json::Number(e.minY);
        j.obj["maxX"] = json::Number(e.maxX);
        j.obj["maxY"] = json::Number(e.maxY);
        j.obj["minZoom"] = json::Number(e.zooms.min);
        j.obj["maxZoom"] = json::Number(e.zooms.max);
        return ddbt::api::CopyOut(json::Dump(j, 0), outJson);
    });
}

}  // extern "C"
