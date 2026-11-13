# SPDX-License-Identifier: MPL-2.0
# Google Benchmark microbenchmarks. Every bench/micro/<area>/*_bench.cpp joins one binary,
# ddbt_micro, so parallel work on different areas never edits this file.
find_package(benchmark CONFIG REQUIRED)
file(GLOB_RECURSE DDBT_MICRO_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*/*_bench.cpp")
list(SORT DDBT_MICRO_SOURCES)
if(DDBT_MICRO_SOURCES)
  add_executable(ddbt_micro ${DDBT_MICRO_SOURCES})
  target_include_directories(ddbt_micro PRIVATE "${PROJECT_SOURCE_DIR}/src")
  target_link_libraries(ddbt_micro PRIVATE ddbt::ddbt ddbt::deps benchmark::benchmark benchmark::benchmark_main)
  ddbt_apply_build_settings(ddbt_micro)
endif()
