# SPDX-License-Identifier: MPL-2.0
# COG tiler: shared sources (registry), decoded-block and encoded-tile caches, GDAL RasterIO
# emulation, the warped grid of non-EPSG:3857 sources, exact band statistics, visualisation
# parameters (formulas, colormaps, sensor profiles) and GDALTiler's tile logic. Paths relative to
# src/.
set(DDBT_COG_TILER_SOURCES
  cog/tiler/cog_source.cpp
  cog/tiler/block_cache.cpp
  cog/tiler/kernels.cpp
  cog/tiler/raster_io.cpp
  cog/tiler/warp.cpp
  cog/tiler/stats.cpp
  cog/tiler/stats_kernel.cpp
  cog/tiler/vis.cpp
  cog/tiler/json_lite.cpp
  cog/tiler/cog_tiler.cpp)
list(APPEND DDBT_SOURCES ${DDBT_COG_TILER_SOURCES})

# The stretch (scalar and Highway kernels), the statistics, the formulas, the colormap ramps and
# the warped grid must round as DroneDB / GDAL do (built for generic x86-64: no fused
# multiply-add). No contraction, and no LTO (with LTO the per-file flag is lost when the code is
# inlined at link time).
if(NOT MSVC)
  set_source_files_properties(
    cog/tiler/kernels.cpp cog/tiler/raster_io.cpp cog/tiler/warp.cpp cog/tiler/stats.cpp cog/tiler/stats_kernel.cpp
    cog/tiler/vis.cpp cog/tiler/cog_tiler.cpp
    PROPERTIES COMPILE_OPTIONS "-ffp-contract=off;-fno-lto")
endif()
