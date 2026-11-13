// SPDX-License-Identifier: MPL-2.0
#include "api/api_common.h"

#include <cctype>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <system_error>

#include "core/util/last_error.h"

namespace ddbt::api {

ddbtErr Fail(ddbtErr code, const std::string& message) {
    SetLastError(message);
    return code;
}

bool IsEmpty(const char* s) noexcept { return s == nullptr || *s == '\0'; }

bool NullOrBlank(const char* s) noexcept {
    if (s == nullptr) return true;
    for (; *s; ++s)
        if (!std::isspace(static_cast<unsigned char>(*s))) return false;
    return true;
}

ddbtErr CodeOf(cog::tiler::TileErrorCode code) noexcept {
    using cog::tiler::TileErrorCode;
    switch (code) {
        case TileErrorCode::kInvalidArgs:
        case TileErrorCode::kRefused: return DDBT_ERR_INVALIDARGS;
        case TileErrorCode::kNotFound: return DDBT_ERR_NOTFOUND;
        case TileErrorCode::kNotImplemented: return DDBT_ERR_NOT_IMPLEMENTED;
        case TileErrorCode::kException: break;
    }
    return DDBT_ERR_EXCEPTION;
}

ddbtErr CodeOf(const Status& status) noexcept {
    switch (status.code()) {
        case StatusCode::kInvalidArgument: return DDBT_ERR_INVALIDARGS;
        case StatusCode::kNotFound: return DDBT_ERR_NOTFOUND;
        case StatusCode::kUnsupported: return DDBT_ERR_NOT_IMPLEMENTED;
        case StatusCode::kCancelled: return DDBT_ERR_CANCELED;
        default: return DDBT_ERR_EXCEPTION;
    }
}

bool PathExists(const std::string& utf8Path) {
    std::error_code ec;
    return std::filesystem::exists(std::filesystem::path(std::u8string(utf8Path.begin(), utf8Path.end())), ec);
}

ddbtErr CopyOut(const std::string& text, char** out) {
    char* buf = static_cast<char*>(std::malloc(text.size() + 1));
    if (buf == nullptr) return Fail(DDBT_ERR_EXCEPTION, "Out of memory");
    std::memcpy(buf, text.c_str(), text.size() + 1);
    *out = buf;
    return DDBT_ERR_NONE;
}

encode::TileFormat ParseFormat(const char* outputFormat) {
    const std::string f = outputFormat != nullptr ? outputFormat : "";
    const auto parsed = encode::ParseTileFormat(f);
    if (!parsed) cog::tiler::FailInvalid("Unsupported image format: " + f);
    return *parsed;
}

void RequireLocal(const std::string& path) {
    const auto starts = [&](const char* p) {
        const size_t n = std::strlen(p);
        if (path.size() < n) return false;
        for (size_t i = 0; i < n; ++i)
            if (std::tolower(static_cast<unsigned char>(path[i])) != p[i]) return false;
        return true;
    };
    if (starts("http://") || starts("https://") || starts("s3://") || starts("/vsi"))
        throw cog::tiler::TileError(cog::tiler::TileErrorCode::kNotImplemented,
                                    "Remote inputs are not supported yet: " + path);
}

}  // namespace ddbt::api
