# SPDX-License-Identifier: MPL-2.0
# Tile maths (WebMercatorQuad, DroneDB compat). Paths relative to src/.
list(APPEND DDBT_SOURCES
  core/tile/web_mercator.cpp
  core/tile/compat.cpp)

# These files reproduce the baseline's (and gdal2tiles') floating-point results bit for bit: no
# fused multiply-add may replace the two roundings of a*b+c (x86-64-v3 has FMA and GCC contracts
# by default in C++). -fno-lto too: with LTO the per-file -ffp-contract=off is lost when the
# functions are inlined at link time (measured: 364 of 503 TileBounds cases off by 1 ulp).
# MSVC (VS 2022) does not contract under its default /fp:precise.
if(NOT MSVC)
  set_source_files_properties(core/tile/web_mercator.cpp core/tile/compat.cpp
    PROPERTIES COMPILE_OPTIONS "-ffp-contract=off;-fno-lto")
endif()
