# SPDX-License-Identifier: MPL-2.0
#
# ddbt_pgo: DDBT_PGO = OFF | GEN | USE, profile directory DDBT_PGO_DIR.
#
# GCC: -fprofile-generate / -fprofile-use. GCC mangles the object path into the .gcda name, so
# -fprofile-prefix-path strips the build directory: the GEN and USE presets may use different
# binary directories. -fprofile-update=prefer-atomic because the profiled code is multi-threaded.
# MSVC: /GENPROFILE and /USEPROFILE (need /LTCG, which DDBT_LTO provides).
add_library(ddbt_pgo INTERFACE)
add_library(ddbt::pgo ALIAS ddbt_pgo)

if(NOT DDBT_PGO MATCHES "^(OFF|GEN|USE)$")
  message(FATAL_ERROR "DDBT_PGO must be OFF, GEN or USE (got '${DDBT_PGO}')")
endif()

if(NOT DDBT_PGO STREQUAL "OFF")
  if(MSVC)
    if(NOT DDBT_LTO_ACTIVE)
      message(FATAL_ERROR "DDBT_PGO with MSVC needs DDBT_LTO=ON in a Release-like configuration")
    endif()
    if(DDBT_PGO STREQUAL "GEN")
      target_link_options(ddbt_pgo INTERFACE /GENPROFILE:PGD=${DDBT_PGO_DIR}/ddbt.pgd)
    else()
      target_link_options(ddbt_pgo INTERFACE /USEPROFILE:PGD=${DDBT_PGO_DIR}/ddbt.pgd)
    endif()
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    file(MAKE_DIRECTORY "${DDBT_PGO_DIR}")
    set(_pgo_common -fprofile-prefix-path=${CMAKE_BINARY_DIR})
    if(DDBT_PGO STREQUAL "GEN")
      target_compile_options(ddbt_pgo INTERFACE ${_pgo_common}
        -fprofile-generate=${DDBT_PGO_DIR} -fprofile-update=prefer-atomic)
      target_link_options(ddbt_pgo INTERFACE -fprofile-generate=${DDBT_PGO_DIR} -fprofile-update=prefer-atomic)
    else()
      target_compile_options(ddbt_pgo INTERFACE ${_pgo_common}
        -fprofile-use=${DDBT_PGO_DIR} -fprofile-correction -Wno-missing-profile)
      target_link_options(ddbt_pgo INTERFACE -fprofile-use=${DDBT_PGO_DIR})
    endif()
  else()
    message(FATAL_ERROR "DDBT_PGO is implemented for GCC and MSVC (compiler: ${CMAKE_CXX_COMPILER_ID})")
  endif()
endif()
