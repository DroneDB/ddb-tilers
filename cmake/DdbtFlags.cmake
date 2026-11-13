# SPDX-License-Identifier: MPL-2.0
#
# ddbt_flags: ISA baseline, optimization, LTO and linker pruning for OUR code only (the vcpkg
# libraries are built by vcpkg with the triplet's flags). Sanitizer builds skip LTO / -O3.
include(CheckIPOSupported)

add_library(ddbt_flags INTERFACE)
add_library(ddbt::flags ALIAS ddbt_flags)

set(_sanitize ON)
if("${DDBT_SANITIZE}" STREQUAL "")
  set(_sanitize OFF)
endif()

# ---- ISA baseline ----------------------------------------------------------------------
if(NOT DDBT_ISA_BASELINE MATCHES "^(x86-64|x86-64-v2|x86-64-v3|native)$")
  message(FATAL_ERROR "DDBT_ISA_BASELINE must be x86-64, x86-64-v2, x86-64-v3 or native (got '${DDBT_ISA_BASELINE}')")
endif()
set(DDBT_ISA_BASELINE_NAME "${DDBT_ISA_BASELINE}")
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _proc)
set(_is_x86 OFF)
if(_proc MATCHES "^(x86_64|amd64|x64)$")
  set(_is_x86 ON)
endif()

# ---- LTO ---------------------------------------------------------------------------------
set(DDBT_LTO_ACTIVE OFF)
if(DDBT_LTO AND NOT _sanitize AND CMAKE_BUILD_TYPE MATCHES "^(Release|RelWithDebInfo|MinSizeRel)$")
  check_ipo_supported(RESULT _ipo_ok OUTPUT _ipo_out LANGUAGES C CXX)
  if(_ipo_ok)
    set(DDBT_LTO_ACTIVE ON)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
  else()
    message(WARNING "DDBT_LTO requested but not supported: ${_ipo_out}")
  endif()
endif()

if(MSVC)
  # ------------------------------------------------------------------------------------
  # MSVC
  # ------------------------------------------------------------------------------------
  target_compile_options(ddbt_flags INTERFACE
    /permissive- /Zc:__cplusplus /utf-8 /EHsc /Zc:inline
    $<$<COMPILE_LANGUAGE:CXX>:/Zc:preprocessor>)
  target_compile_definitions(ddbt_flags INTERFACE
    NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS _SILENCE_ALL_CXX20_DEPRECATION_WARNINGS)
  if(NOT _sanitize)
    foreach(_opt /O2 /Ob3 /Gw /Gy)
      target_compile_options(ddbt_flags INTERFACE "$<$<CONFIG:Release,RelWithDebInfo,MinSizeRel>:${_opt}>")
    endforeach()
    if(DDBT_ISA_BASELINE STREQUAL "x86-64-v3" OR DDBT_ISA_BASELINE STREQUAL "native")
      target_compile_options(ddbt_flags INTERFACE /arch:AVX2)
    endif()
    if(DDBT_LTO_ACTIVE)
      # CMake adds /GL and /LTCG for INTERPROCEDURAL_OPTIMIZATION
      target_link_options(ddbt_flags INTERFACE
        "$<$<CONFIG:Release,RelWithDebInfo,MinSizeRel>:/OPT:REF,ICF>")
    endif()
  else()
    target_compile_options(ddbt_flags INTERFACE /Od /Zi)
    target_link_options(ddbt_flags INTERFACE /DEBUG /INCREMENTAL:NO)
  endif()
else()
  # ------------------------------------------------------------------------------------
  # GCC / Clang
  # ------------------------------------------------------------------------------------
  target_compile_options(ddbt_flags INTERFACE
    -fvisibility=hidden $<$<COMPILE_LANGUAGE:CXX>:-fvisibility-inlines-hidden>
    -fno-math-errno)           # no -ffast-math: results must stay reproducible
  if(_is_x86)
    if(DDBT_ISA_BASELINE STREQUAL "x86-64")
      target_compile_options(ddbt_flags INTERFACE -march=x86-64 -mtune=generic)
    else()
      # x86-64-v2/v3 do not include PCLMUL and AES, which Highway requires before it treats
      # SSE4 / AVX2 as the static baseline (every CPU with SSE4.2 or AVX2 has them).
      target_compile_options(ddbt_flags INTERFACE -march=${DDBT_ISA_BASELINE} -mpclmul -maes -mtune=generic)
    endif()
  endif()
  if(NOT _sanitize)
    target_compile_options(ddbt_flags INTERFACE
      $<$<CONFIG:Release,RelWithDebInfo,MinSizeRel>:-O3>
      -fno-semantic-interposition -ffunction-sections -fdata-sections)
    if(NOT CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      target_compile_options(ddbt_flags INTERFACE -fno-plt)
      target_link_options(ddbt_flags INTERFACE -Wl,--gc-sections)
    else()
      target_link_options(ddbt_flags INTERFACE -Wl,-dead_strip)
    endif()
  endif()
  target_compile_options(ddbt_flags INTERFACE
    $<$<CONFIG:RelWithDebInfo>:-fno-omit-frame-pointer>)
endif()

# Highway: dynamic dispatch compiles the kernels for these targets above the baseline. The
# newest ones (AVX3_SPR, AVX10_2) are disabled to save compile time: AVX3_ZEN4 / AVX3_DL / AVX3
# cover the machines we measure on.
target_compile_definitions(ddbt_flags INTERFACE
  "HWY_DISABLED_TARGETS=(HWY_AVX3_SPR|HWY_AVX10_2)")

# Build information baked into the library (version.cpp).
execute_process(COMMAND git rev-parse --short HEAD
                WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
                OUTPUT_VARIABLE DDBT_GIT_SHA OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_QUIET RESULT_VARIABLE _git_rc)
if(NOT _git_rc EQUAL 0 OR DDBT_GIT_SHA STREQUAL "")
  set(DDBT_GIT_SHA "unknown")
endif()
set(DDBT_VERSION_STRING "${PROJECT_VERSION}")
set(DDBT_COMPILER_ID "${CMAKE_CXX_COMPILER_ID}")
set(DDBT_COMPILER_VERSION "${CMAKE_CXX_COMPILER_VERSION}")
if(CMAKE_CONFIGURATION_TYPES)
  set(DDBT_BUILD_TYPE "multi-config")
else()
  set(DDBT_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
endif()
configure_file("${PROJECT_SOURCE_DIR}/cmake/ddbt_config.h.in"
               "${PROJECT_BINARY_DIR}/generated/ddbt_config.h" @ONLY)

# Every target we build (library, CLI, tests, bench) takes the same settings through this.
function(ddbt_apply_build_settings target)
  target_link_libraries(${target} PRIVATE ddbt::flags ddbt::warnings ddbt::sanitizers ddbt::pgo)
endfunction()
