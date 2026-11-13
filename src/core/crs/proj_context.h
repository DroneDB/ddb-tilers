// SPDX-License-Identifier: MPL-2.0
#pragma once

// PROJ contexts: one PJ_CONTEXT per thread (a context and the objects created in it must not be
// used by two threads at once), all configured the same way: network off, proj.db from the
// directory below, PROJ's messages captured instead of printed.
//
// proj.db: the first of
//   1. SetProjDataDir() (before the first transform; contexts created later use it),
//   2. the environment variable DDBT_PROJ_DATA,
//   3. the share/proj of the PROJ this library was built with (the vcpkg_installed tree of the
//      build: a development default, baked in at compile time),
//   4. PROJ's own search (PROJ_DATA, then its install prefix).
// A system PROJ_DATA is NOT used before 3: a proj.db of another PROJ version may not open.

#include <proj.h>

#include <cstdint>
#include <memory>
#include <string>

#include "core/crs/crs.h"  // CrsError, ProjDataDir

namespace ddbt::crs {

namespace detail {

/// A context configured as above, owned by the caller (proj_context_destroy).
PJ_CONTEXT* NewContext();

/// The calling thread's context (created on first use, destroyed at thread exit).
PJ_CONTEXT* ThreadContext();

/// Last error PROJ reported on this thread (the text of the last logged message, else the
/// errno string of ctx), and clears it.
std::string LastError(PJ_CONTEXT* ctx);

/// Forgets the last logged message of this thread (before an operation that may fail).
void ClearLastError() noexcept;

/// RAII for PJ objects.
struct PjDeleter {
    void operator()(PJ* p) const noexcept { proj_destroy(p); }
};
using PjPtr = std::unique_ptr<PJ, PjDeleter>;

/// The calling thread's copy of a shared object. `id` is unique per owner (never reused);
/// `alive` expires when the owner is destroyed, which lets the thread drop its copies of dead
/// owners. `make` builds the copy in the thread's context (called once per thread and id).
/// Returns nullptr if make fails.
using MakeFn = PJ* (*)(const void* owner, PJ_CONTEXT* ctx);
PJ* ThreadCopy(uint64_t id, const std::weak_ptr<const void>& alive, const void* owner, MakeFn make);

/// A process-unique id for ThreadCopy.
uint64_t NextId() noexcept;

}  // namespace detail

}  // namespace ddbt::crs
