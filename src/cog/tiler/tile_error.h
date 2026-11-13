// SPDX-License-Identifier: MPL-2.0
//
// TileError: the failures of a tile request, with the C API code they map to. The tiler ports
// DroneDB's GDALTiler / vis_render logic, which reports with exceptions; inside src/cog/tiler a
// failure is a TileError (a reader Status is converted with ThrowIfError), and src/api maps it to
// a ddbtErr. Exceptions also carry errors out of the cache loaders (core/cache), as the core's
// error policy prescribes for loaders; each waiter of a load gets its own copy of the error
// (cache::WaiterCopyable: same class, code and message), so no exception object is shared
// across threads.
#pragma once

#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

#include "core/cache/sharded_lru.h"
#include "core/util/status.h"

namespace ddbt::cog::tiler {

enum class TileErrorCode {
    kException,       // generic failure (DroneDB GDALException / AppException)
    kInvalidArgs,     // InvalidArgsException: a malformed argument
    kRefused,         // RefusedArgumentException: an argument the raster cannot honour
    kNotFound,        // the input does not exist
    kNotImplemented,  // valid input ddbt cannot tile yet (non-TIFF raster that needs a conversion)
};

class TileError : public std::runtime_error, public cache::WaiterCopyable {
public:
    TileError(TileErrorCode code, std::string message) : std::runtime_error(std::move(message)), code_(code) {}
    TileErrorCode code() const noexcept { return code_; }
    /// A new TileError with this code and message at every call (subclasses override it).
    std::function<void()> MakeThrower() const override {
        return [code = code_, m = std::string(what())] { throw TileError(code, m); };
    }

private:
    TileErrorCode code_;
};

[[noreturn]] inline void Fail(std::string message) { throw TileError(TileErrorCode::kException, std::move(message)); }
[[noreturn]] inline void FailInvalid(std::string message) {
    throw TileError(TileErrorCode::kInvalidArgs, std::move(message));
}
[[noreturn]] inline void FailRefused(std::string message) {
    throw TileError(TileErrorCode::kRefused, std::move(message));
}

/// Throws a TileError (kException, or kNotFound for a NotFound status) with `context: message`.
inline void ThrowIfError(const Status& s, const std::string& context) {
    if (s.ok()) return;
    const TileErrorCode code = s.code() == StatusCode::kNotFound ? TileErrorCode::kNotFound : TileErrorCode::kException;
    throw TileError(code, context.empty() ? std::string(s.message()) : context + ": " + std::string(s.message()));
}

template <typename T>
T ValueOrThrow(Expected<T>&& e, const std::string& context) {
    if (!e.ok()) ThrowIfError(e.status(), context);
    return std::move(e).value();
}

}  // namespace ddbt::cog::tiler
