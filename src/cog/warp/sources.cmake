# SPDX-License-Identifier: MPL-2.0
# COG builder: the source pixel <-> CRS mapping (geotransform or GCPs), the warped grid
# (GDALSuggestedWarpOutput2 + COG GoogleMapsCompatible snapping), GDAL's warp chunks and
# approximate transformer (GCP sources), and GDAL's bilinear warp kernel (scalar reference +
# Highway SIMD) with a consumer-counted source block cache. Paths relative to src/.
list(APPEND DDBT_SOURCES
  cog/warp/source_georef.cpp
  cog/warp/target_grid.cpp
  cog/warp/row_approx.cpp
  cog/warp/warp_chunks.cpp
  cog/warp/bilinear.cpp
  cog/warp/warp_kernels.cpp
  cog/warp/warp_producer.cpp)
# The grid and the kernels must round as GDAL's (generic x86-64, no FMA): no contraction (GCC
# contracts by default in C++, the x86 intrinsics included), no LTO (with LTO the per-file flag is
# lost when the code is inlined at link time).
if(NOT MSVC)
  set_source_files_properties(cog/warp/source_georef.cpp cog/warp/target_grid.cpp cog/warp/row_approx.cpp
    cog/warp/warp_chunks.cpp cog/warp/bilinear.cpp cog/warp/warp_kernels.cpp cog/warp/warp_producer.cpp
    PROPERTIES COMPILE_OPTIONS "-ffp-contract=off;-fno-lto")
endif()
