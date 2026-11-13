// SPDX-License-Identifier: MPL-2.0
#include "ddbt/ddbt.h"

#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

#include "core/simd/simd.h"
#include "core/util/last_error.h"
#include "core/version.h"

namespace {

using ddbt::ClearLastError;
using ddbt::SetLastError;

ddbtErr Fail(ddbtErr code, std::string message) {
    SetLastError(std::move(message));
    return code;
}

template <typename F>
ddbtErr Guarded(F&& f) noexcept {
    try {
        return f();
    } catch (const std::exception& e) {
        return Fail(DDBT_ERR_EXCEPTION, e.what());
    } catch (...) {
        return Fail(DDBT_ERR_EXCEPTION, "unknown exception");
    }
}

void AppendJsonString(std::string& out, const std::string& s) {
    out += '"';
    for (const char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    out += "\\u00";
                    out += "0123456789abcdef"[(static_cast<unsigned char>(c) >> 4) & 0xF];
                    out += "0123456789abcdef"[static_cast<unsigned char>(c) & 0xF];
                } else {
                    out += c;
                }
        }
    }
    out += '"';
}

void AppendJsonStringArray(std::string& out, const std::vector<std::string>& v) {
    out += '[';
    for (size_t i = 0; i < v.size(); ++i) {
        if (i != 0) out += ',';
        AppendJsonString(out, v[i]);
    }
    out += ']';
}

}  // namespace

extern "C" {

const char* ddbtVersion(void) { return ddbt::Version(); }

const char* ddbtLastError(void) { return ddbt::LastError(); }

// ddbtMemoryTileFmt, ddbtMemoryTileEx2 and ddbtMemoryTileEx3: src/api/tile_api.cpp; ddbtBuildCog: src/api/build_api.cpp;
// ddbtBuildCopc: src/api/build_copc_api.cpp.

ddbtErr ddbtFree(void* ptr) {
    std::free(ptr);
    return DDBT_ERR_NONE;
}

ddbtErr ddbtCpuInfo(char** outJson) {
    return Guarded([&]() -> ddbtErr {
        ClearLastError();
        if (outJson == nullptr) return Fail(DDBT_ERR_INVALIDARGS, "Invalid outJson: NULL");

        std::string j = "{\"version\":";
        AppendJsonString(j, ddbt::Version());
        j += ",\"gitSha\":";
        AppendJsonString(j, ddbt::GitSha());
        j += ",\"compiler\":";
        AppendJsonString(j, ddbt::CompilerId());
        j += ",\"buildType\":";
        AppendJsonString(j, ddbt::BuildType());
        j += ",\"isaBaseline\":";
        AppendJsonString(j, ddbt::IsaBaseline());
        j += ",\"highway\":{\"staticTarget\":";
        AppendJsonString(j, ddbt::simd::StaticTargetName());
        j += ",\"dispatchTarget\":";
        AppendJsonString(j, ddbt::simd::ActiveTargetName());
        j += ",\"compiledTargets\":";
        AppendJsonStringArray(j, ddbt::simd::CompiledTargetNames());
        j += ",\"supportedTargets\":";
        AppendJsonStringArray(j, ddbt::simd::SupportedTargetNames());
        j += "}}";

        char* buf = static_cast<char*>(std::malloc(j.size() + 1));
        if (buf == nullptr) return Fail(DDBT_ERR_EXCEPTION, "out of memory");
        std::memcpy(buf, j.c_str(), j.size() + 1);
        *outJson = buf;
        return DDBT_ERR_NONE;
    });
}

}  // extern "C"
