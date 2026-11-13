# SPDX-License-Identifier: MPL-2.0
# COPC tiler: shared clouds (registry), Highway kernels (decode + cull, ramp), the compat and
# default rasterizers, the tile logic; plus the point-cloud entry of the C API
# (api/copc_tile.cpp). Paths relative to src/.
list(APPEND DDBT_SOURCES
  copc/tiler/kernels.cpp
  copc/tiler/rasterizer.cpp
  copc/tiler/cloud.cpp
  copc/tiler/copc_tiler.cpp
  api/copc_tile.cpp)

# The kernels and the rasterizer must round as DroneDB / PDAL do (built for generic x86-64: no
# fused multiply-add): x * scale + offset, the ramp and the pixel mapping. No contraction, and no
# LTO (with LTO the per-file flag is lost when the code is inlined at link time).
if(NOT MSVC)
  set_source_files_properties(copc/tiler/kernels.cpp copc/tiler/rasterizer.cpp
    PROPERTIES COMPILE_OPTIONS "-ffp-contract=off;-fno-lto")
endif()

# libddbt_copc_harness.so: ddbtMemoryTileFmt for COPC inputs only, for ddbt-bench while src/api's
# tile dispatch does not route point clouds yet (harness_shim.cpp). Off by default.
option(DDBT_COPC_HARNESS_SHIM "Build libddbt_copc_harness.so (COPC tiles behind ddbtMemoryTileFmt for ddbt-bench)" OFF)
if(DDBT_COPC_HARNESS_SHIM AND NOT MSVC)
  function(ddbt_copc_harness_shim)
    add_library(ddbt_copc_harness SHARED copc/tiler/harness_shim.cpp)
    target_include_directories(ddbt_copc_harness PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}/generated")
    # The static library: its tile_api / ddbt_api objects are not pulled in (nothing refers to them).
    target_link_libraries(ddbt_copc_harness PRIVATE ddbt ddbt::deps)
    ddbt_apply_build_settings(ddbt_copc_harness)
    target_link_options(ddbt_copc_harness PRIVATE -Wl,--exclude-libs,ALL)
  endfunction()
  # After src/CMakeLists.txt has defined the ddbt target.
  cmake_language(DEFER CALL ddbt_copc_harness_shim)
endif()
