# SPDX-License-Identifier: Apache-2.0
#
# fpnge (https://github.com/veluca93/fpnge, commit 097ebccf3d3d, unmodified; see README.ddbt.md):
# a fast lossless PNG encoder with SSE4.1 / AVX2 code. fpnge.cc is compiled twice (variants/), once
# for SSE 4.1 + CLMUL and once for AVX2, as ddbt_fpnge_encode_sse4 / _avx2 (variants/fpnge_variants.h):
# the library's baseline stays plain x86-64 and the PNG encoder picks the variant the CPU runs.
# Static library, hidden visibility, built with the project's optimisation / LTO / sanitizer /
# PGO settings but not with its warnings (third-party code). The include directory is SYSTEM.
add_library(ddbt_fpnge STATIC variants/fpnge_sse4.cc variants/fpnge_avx2.cc)
add_library(ddbt::fpnge ALIAS ddbt_fpnge)
target_include_directories(ddbt_fpnge SYSTEM PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_compile_features(ddbt_fpnge PRIVATE cxx_std_17)
set_target_properties(ddbt_fpnge PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
  POSITION_INDEPENDENT_CODE ON)
target_link_libraries(ddbt_fpnge PRIVATE ddbt::flags ddbt::sanitizers ddbt::pgo)
if(MSVC)
  target_compile_options(ddbt_fpnge PRIVATE /W1)
else()
  target_compile_options(ddbt_fpnge PRIVATE -w)
endif()
