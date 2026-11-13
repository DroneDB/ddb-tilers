// SPDX-License-Identifier: MPL-2.0
#pragma once

// DDBT_TARGET("features"): compiles one function for instructions above the library's baseline
// (GCC / Clang target attribute), for hand-written paths outside Highway that the caller picks at
// run time with simd::HasSse4() / simd::HasAvx2(). MSVC needs no attribute: its intrinsics are
// available whatever /arch says.
#if defined(__GNUC__) || defined(__clang__)
#define DDBT_TARGET(features) __attribute__((target(features)))
#else
#define DDBT_TARGET(features)
#endif

#if defined(__x86_64__) || defined(_M_X64)
#define DDBT_X86_64 1
#else
#define DDBT_X86_64 0
#endif
