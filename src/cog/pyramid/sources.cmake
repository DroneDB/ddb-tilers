# SPDX-License-Identifier: MPL-2.0
# COG builder: overview resampling (GDAL's cubic convolution, mask-aware). Paths relative to src/.
list(APPEND DDBT_SOURCES
  cog/pyramid/overview.cpp
  cog/pyramid/overview_kernels.cpp)
# Sums must round as GDAL's (generic x86-64, no FMA): no contraction, no LTO. The kernels too:
# GCC implements the vector intrinsics as plain vector arithmetic, which it would otherwise fuse.
if(NOT MSVC)
  set_source_files_properties(cog/pyramid/overview.cpp cog/pyramid/overview_kernels.cpp
    PROPERTIES COMPILE_OPTIONS "-ffp-contract=off;-fno-lto")
endif()
