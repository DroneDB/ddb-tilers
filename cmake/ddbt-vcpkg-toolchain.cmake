# SPDX-License-Identifier: MPL-2.0
#
# Toolchain wrapper used by CMakePresets.json: locates the vcpkg checkout and forwards to its
# toolchain file. Resolution order:
#   1. -DDDBT_VCPKG_ROOT=<dir>
#   2. the VCPKG_ROOT environment variable
#   3. the shared checkout of this workstation (DroneDB's submodule):
#        Linux   /mnt/data/repos/DroneDB/vcpkg
#        Windows C:/repos/DroneDB/vcpkg
# The checkout is only read (never modified) and its binary cache is shared.
if(NOT DDBT_VCPKG_ROOT AND DEFINED ENV{DDBT_VCPKG_ROOT})
  set(DDBT_VCPKG_ROOT "$ENV{DDBT_VCPKG_ROOT}")
endif()
if(NOT DDBT_VCPKG_ROOT AND DEFINED ENV{VCPKG_ROOT} AND NOT "$ENV{VCPKG_ROOT}" STREQUAL "")
  set(DDBT_VCPKG_ROOT "$ENV{VCPKG_ROOT}")
endif()
if(NOT DDBT_VCPKG_ROOT)
  foreach(_cand "/mnt/data/repos/DroneDB/vcpkg" "C:/repos/DroneDB/vcpkg")
    if(EXISTS "${_cand}/scripts/buildsystems/vcpkg.cmake")
      set(DDBT_VCPKG_ROOT "${_cand}")
      break()
    endif()
  endforeach()
endif()
if(NOT DDBT_VCPKG_ROOT OR NOT EXISTS "${DDBT_VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
  message(FATAL_ERROR "vcpkg not found: set VCPKG_ROOT (or DDBT_VCPKG_ROOT) to a vcpkg checkout")
endif()
set(DDBT_VCPKG_ROOT "${DDBT_VCPKG_ROOT}" CACHE PATH "vcpkg checkout" FORCE)
set(ENV{VCPKG_ROOT} "${DDBT_VCPKG_ROOT}")
include("${DDBT_VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
