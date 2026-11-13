# fpnge in ddb-tilers

Upstream: https://github.com/veluca93/fpnge, commit `097ebccf3d3db443d9a8f238b1deed010a82f961` (2025-10-09).
Files: `fpnge.cc`, `fpnge.h`, `LICENSE` (Apache-2.0, Copyright 2021 Google LLC), copied unmodified.

Used by `src/core/encode/png_encoder.cpp` for grey, grey + alpha, RGB and RGBA tiles when the PNG engine is fpnge
(`PngOptions::engine` or `DDBT_PNG_ENGINE=fpnge`; libdeflate is the default: fpnge encodes raster tiles 4.6-5.1x faster
but 20-26 % larger, which is slower end to end below about 300 Mbit/s). Palette tiles and the blank-tile cache always
keep the libdeflate path.

`variants/` compiles `fpnge.cc` twice under other names: `ddbt_fpnge_encode_sse4` (SSE 4.1 + CLMUL)
and `ddbt_fpnge_encode_avx2` (AVX2 + BMI2), each through a GCC target pragma (MSVC: the macros fpnge
reads), with the standard headers included first so that only fpnge's own code uses those
instructions. The library's baseline stays plain x86-64; the PNG encoder calls the variant the CPU
runs, and libdeflate on a CPU without SSE 4.1 + CLMUL. fpnge's PEXT path is off in both (Zen 1 / 2
run PEXT in microcode).

