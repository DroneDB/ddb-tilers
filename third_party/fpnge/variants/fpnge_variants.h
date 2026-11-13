// SPDX-License-Identifier: Apache-2.0
// ddb-tilers: fpnge compiled twice, for CPUs with SSE 4.1 + CLMUL and for CPUs with AVX2 (+ BMI2),
// each under its own name, so the library's baseline stays plain x86-64 and the caller picks the
// variant the running CPU supports. Same arguments and result as FPNGEEncode (fpnge.h).
#ifndef DDBT_FPNGE_VARIANTS_H
#define DDBT_FPNGE_VARIANTS_H

#include <stddef.h>

#include "fpnge.h"

#ifdef __cplusplus
extern "C" {
#endif

size_t ddbt_fpnge_encode_sse4(size_t bytes_per_channel, size_t num_channels, const void *data,
                              size_t width, size_t row_stride, size_t height, void *output,
                              const struct FPNGEOptions *options);
size_t ddbt_fpnge_encode_avx2(size_t bytes_per_channel, size_t num_channels, const void *data,
                              size_t width, size_t row_stride, size_t height, void *output,
                              const struct FPNGEOptions *options);

#ifdef __cplusplus
}
#endif

#endif  // DDBT_FPNGE_VARIANTS_H
