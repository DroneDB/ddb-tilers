# SPDX-License-Identifier: MPL-2.0
#
# ddbt_jxl: libjxl (the JPEG-XL blocks of TIFF files), its CMS and its dependencies. The vcpkg port
# installs pkg-config files only, so the libraries are found by name.
find_path(DDBT_JXL_INCLUDE_DIR jxl/decode.h REQUIRED)
find_library(DDBT_JXL_LIBRARY NAMES jxl REQUIRED)
find_library(DDBT_JXL_CMS_LIBRARY NAMES jxl_cms REQUIRED)
find_package(unofficial-brotli CONFIG REQUIRED)
find_package(lcms2 CONFIG QUIET)
if(TARGET lcms2::lcms2)
  set(_ddbt_lcms lcms2::lcms2)
else()
  find_library(_ddbt_lcms NAMES lcms2 REQUIRED)
endif()

add_library(ddbt_jxl INTERFACE)
target_include_directories(ddbt_jxl SYSTEM INTERFACE "${DDBT_JXL_INCLUDE_DIR}")
target_link_libraries(ddbt_jxl INTERFACE
  "${DDBT_JXL_LIBRARY}" "${DDBT_JXL_CMS_LIBRARY}" ${_ddbt_lcms}
  unofficial::brotli::brotlienc unofficial::brotli::brotlidec unofficial::brotli::brotlicommon hwy::hwy)
# A static libjxl (no jxl.dll next to it) exports nothing: its headers must not ask for dllimport
get_filename_component(_ddbt_jxl_libdir "${DDBT_JXL_LIBRARY}" DIRECTORY)
if(NOT EXISTS "${_ddbt_jxl_libdir}/../bin/jxl.dll")
  target_compile_definitions(ddbt_jxl INTERFACE JXL_STATIC_DEFINE JXL_CMS_STATIC_DEFINE)
endif()
