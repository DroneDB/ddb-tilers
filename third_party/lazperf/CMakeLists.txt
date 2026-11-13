# SPDX-License-Identifier: Apache-2.0
#
# In-tree fork of laz-perf 3.4.0 (see FORK.md). Static library, hidden visibility, built with
# the project's ISA / optimisation / LTO / sanitizer / PGO settings but not with its warnings
# (third-party code: warnings are reported, never fatal). Consumers include <lazperf/...>; the
# include directory is SYSTEM so the headers do not warn in our code.
file(GLOB DDBT_LAZPERF_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/lazperf/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/lazperf/detail/*.cpp")
list(SORT DDBT_LAZPERF_SOURCES)

add_library(ddbt_lazperf STATIC ${DDBT_LAZPERF_SOURCES})
add_library(ddbt::lazperf ALIAS ddbt_lazperf)

target_include_directories(ddbt_lazperf SYSTEM PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_include_directories(ddbt_lazperf PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lazperf")
# LAZPERF_VENDORED: LAZPERF_EXPORT expands to nothing (no default-visibility symbols).
target_compile_definitions(ddbt_lazperf PUBLIC LAZPERF_VENDORED)
target_compile_features(ddbt_lazperf PUBLIC cxx_std_17)
set_target_properties(ddbt_lazperf PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
  POSITION_INDEPENDENT_CODE ON)
target_link_libraries(ddbt_lazperf PRIVATE ddbt::flags ddbt::sanitizers ddbt::pgo)

if(MSVC)
  target_compile_options(ddbt_lazperf PRIVATE /W1)
else()
  # Upstream's own list (cmake/unix_compiler_options.cmake) of warnings it does not fix.
  target_compile_options(ddbt_lazperf PRIVATE
    -Wno-implicit-fallthrough -Wno-int-in-bool-context -Wno-dangling-else
    -Wno-unused-parameter -Wno-unused-variable -Wno-deprecated-declarations)
endif()
