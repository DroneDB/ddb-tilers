# SPDX-License-Identifier: MPL-2.0

set(DDBT_SOURCES
  api/ddbt_api.cpp
  core/version.cpp
  core/util/last_error.cpp
  core/simd/dispatch.cpp)

# Each area lists its own sources in a sources.cmake (paths relative to src/), appended to
# DDBT_SOURCES, so parallel work on different areas never edits this file.
file(GLOB_RECURSE DDBT_AREA_SOURCE_LISTS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*/sources.cmake")
list(SORT DDBT_AREA_SOURCE_LISTS)
foreach(_list IN LISTS DDBT_AREA_SOURCE_LISTS)
  include("${_list}")
endforeach()

# Targets
#   ddbt         static library: for the CLI, the tests and the benchmarks
#   ddbt_shared  libddbt.so / ddbt.dll: the C API only (visibility hidden), for dlopen() by the
#                bench driver; third-party code is linked in statically and not re-exported
#
# GCC / Clang compile the sources once (object library) and both targets reuse the objects.
# MSVC needs a different DDBT_API (dllexport) for the DLL, so each target compiles them.

function(ddbt_configure_core tgt)
  target_include_directories(${tgt}
    PUBLIC  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}/generated")
  target_compile_features(${tgt} PUBLIC cxx_std_20)
  target_link_libraries(${tgt} PRIVATE ddbt::deps)
  ddbt_apply_build_settings(${tgt})
  set_target_properties(${tgt} PROPERTIES POSITION_INDEPENDENT_CODE ON)
endfunction()

if(MSVC)
  add_library(ddbt STATIC ${DDBT_SOURCES})
  ddbt_configure_core(ddbt)
  target_compile_definitions(ddbt PUBLIC DDBT_STATIC)
  if(DDBT_BUILD_SHARED)
    add_library(ddbt_shared SHARED ${DDBT_SOURCES})
    ddbt_configure_core(ddbt_shared)
    target_compile_definitions(ddbt_shared PRIVATE DDBT_BUILDING_SHARED PUBLIC DDBT_SHARED)
  endif()
else()
  add_library(ddbt_obj OBJECT ${DDBT_SOURCES})
  ddbt_configure_core(ddbt_obj)

  add_library(ddbt STATIC $<TARGET_OBJECTS:ddbt_obj>)
  ddbt_configure_core(ddbt)

  if(DDBT_BUILD_SHARED)
    add_library(ddbt_shared SHARED $<TARGET_OBJECTS:ddbt_obj>)
    ddbt_configure_core(ddbt_shared)
    target_link_options(ddbt_shared PRIVATE -Wl,--exclude-libs,ALL)
    target_compile_definitions(ddbt_shared PUBLIC DDBT_SHARED)
  endif()
endif()

add_library(ddbt::ddbt ALIAS ddbt)
if(TARGET ddbt_shared)
  set_target_properties(ddbt_shared PROPERTIES OUTPUT_NAME ddbt)
  add_library(ddbt::shared ALIAS ddbt_shared)
endif()
