// SPDX-License-Identifier: MPL-2.0
//
// libddbt_copc_harness.so: the C API entry points ddbt-bench's "ddbt" backend loads
// (ddbtMemoryTileFmt, ddbtFree, ddbtLastError, ddbtVersion), with ddbtMemoryTileFmt serving COPC
// inputs through ddbt::api::CopcMemoryTile. Built only with -DDDBT_COPC_HARNESS_SHIM=ON
// (src/copc/tiler/sources.cmake), for measurements while src/api's tile dispatch does not route
// point clouds yet; once it does, libddbt.so serves them and this library can go.
#include <cstdlib>

#include "api/copc_tile.h"
#include "core/util/last_error.h"
#include "core/version.h"
#include "ddbt/ddbt.h"

#define DDBT_SHIM_EXPORT __attribute__((visibility("default")))

extern "C" {

DDBT_SHIM_EXPORT const char* ddbtVersion(void) { return ddbt::Version(); }

DDBT_SHIM_EXPORT const char* ddbtLastError(void) { return ddbt::LastError(); }

DDBT_SHIM_EXPORT ddbtErr ddbtFree(void* ptr) {
    std::free(ptr);
    return DDBT_ERR_NONE;
}

DDBT_SHIM_EXPORT ddbtErr ddbtMemoryTileFmt(const char* inputPath, int tz, int tx, int ty, uint8_t** outBuffer,
                                           int* outBufferSize, int tileSize, bool tms, bool forceRecreate,
                                           const char* inputPathHash, const char* outputFormat) {
    return ddbt::api::CopcMemoryTile(inputPath, tz, tx, ty, outBuffer, outBufferSize, tileSize, tms, forceRecreate,
                                     inputPathHash, outputFormat);
}

}  // extern "C"
