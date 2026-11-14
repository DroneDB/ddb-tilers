// SPDX-License-Identifier: MPL-2.0
#pragma once

// Remote inputs (http:// and https:// URLs) read through range callbacks that the host
// registers once for the process (ddbtSetRemoteReader). ddbt has no HTTP client of its own: the
// host already has one, with its proxy, certificate and authentication settings.

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <string_view>

#include "core/io/byte_source.h"
#include "core/util/status.h"

namespace ddbt::io {

/// The identity of a remote object at the time of the stat: a changed object has another size or
/// another version (an ETag, a Last-Modified date; "" when the server gives none).
struct RemoteStat {
    uint64_t size = 0;
    std::string version;
};

/// The callbacks of the host. Both are called concurrently from several threads.
struct RemoteReader {
    /// Size and version of url; NotFound when the object does not exist.
    std::function<Status(const std::string& url, RemoteStat& out)> stat;
    /// Fills out with the bytes of url at offset, entirely, or returns an error.
    std::function<Status(const std::string& url, uint64_t offset, std::span<uint8_t> out)> read;
};

/// Whether path is a URL that the remote reader reads (http:// or https://, any case).
bool IsRemotePath(std::string_view path) noexcept;

/// Sets the reader of the process (nullptr removes it). Calls that run keep the reader they started
/// with.
void SetRemoteReader(std::shared_ptr<const RemoteReader> reader);

/// Whether a reader is set.
bool HasRemoteReader() noexcept;

/// The size and version of url. Unsupported when no reader is set.
Expected<RemoteStat> StatRemote(const std::string& url);

/// url as a ByteSource of the size given by its stat, read through the reader set now.
/// Unsupported when no reader is set.
Expected<std::unique_ptr<ByteSource>> OpenRemoteSource(const std::string& url, uint64_t size);

}  // namespace ddbt::io
