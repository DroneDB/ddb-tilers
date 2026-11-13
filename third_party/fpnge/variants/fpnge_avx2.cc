// SPDX-License-Identifier: Apache-2.0
// ddb-tilers: fpnge.cc (unmodified) compiled as ddbt_fpnge_encode_avx2 for AVX2, BMI2, SSE 4.1
// and CLMUL (every CPU that runs the library's AVX2 target).
// The standard headers and fpnge.h come first, outside the target region, so their inline
// functions and templates stay compiled for the library's baseline: only fpnge's own code uses
// the instructions.
#include <algorithm>
#include <array>
#include <cstddef>
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <utility>
#include <vector>

#include "variants/fpnge_variants.h"

// Zen 1 / Zen 2 run PEXT in microcode (hundreds of cycles): fpnge's PEXT path is not used, the
// variant runs on any CPU with these instructions.
#define FPNGE_USE_PEXT 0
#define FPNGEEncode ddbt_fpnge_encode_avx2
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC target("avx2,bmi,bmi2,sse4.1,pclmul")
#elif defined(__clang__)
#error "fpnge variants: GCC or MSVC only"
#endif
// fpnge.cc picks its code by the feature macros: MSVC does not define them for the instructions
// its intrinsics use, and GCC before 14 does not define those of a target pragma in C++.
#ifndef __AVX2__
#define __AVX2__ 1
#endif
#ifndef __SSE4_1__
#define __SSE4_1__ 1
#endif
#ifndef __PCLMUL__
#define __PCLMUL__ 1
#endif
#include "../fpnge.cc"
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
