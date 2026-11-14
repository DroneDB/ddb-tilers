// SPDX-License-Identifier: MPL-2.0
#include "core/io/remote_source.h"

#include <atomic>
#include <cctype>
#include <cstring>
#include <utility>

namespace ddbt::io {

namespace {

std::atomic<std::shared_ptr<const RemoteReader>>& Current() {
    static std::atomic<std::shared_ptr<const RemoteReader>> reader;
    return reader;
}

bool StartsWithNoCase(std::string_view text, std::string_view prefix) noexcept {
    if (text.size() < prefix.size()) return false;
    for (size_t i = 0; i < prefix.size(); ++i)
        if (std::tolower(static_cast<unsigned char>(text[i])) != prefix[i]) return false;
    return true;
}

Expected<std::shared_ptr<const RemoteReader>> RequireReader(const std::string& url) {
    auto reader = Current().load();
    if (!reader) return Unsupported("Remote inputs need a remote reader (ddbtSetRemoteReader): " + url);
    return reader;
}

}  // namespace

bool IsRemotePath(std::string_view path) noexcept {
    return StartsWithNoCase(path, "http://") || StartsWithNoCase(path, "https://");
}

void SetRemoteReader(std::shared_ptr<const RemoteReader> reader) { Current().store(std::move(reader)); }

bool HasRemoteReader() noexcept { return Current().load() != nullptr; }

Expected<RemoteStat> StatRemote(const std::string& url) {
    DDBT_ASSIGN_OR_RETURN(std::shared_ptr<const RemoteReader> reader, RequireReader(url));
    RemoteStat st;
    Status s = reader->stat(url, st);
    if (!s.ok()) return s;
    return st;
}

Expected<std::unique_ptr<ByteSource>> OpenRemoteSource(const std::string& url, uint64_t size) {
    DDBT_ASSIGN_OR_RETURN(std::shared_ptr<const RemoteReader> reader, RequireReader(url));
    // The source holds the reader it was opened with, so a reader removed later stays alive for it
    auto read = [reader, url](uint64_t offset, std::span<uint8_t> out) { return reader->read(url, offset, out); };
    return std::unique_ptr<ByteSource>(std::make_unique<RangeCallbackSource>(size, std::move(read), url));
}

}  // namespace ddbt::io
