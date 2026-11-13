# SPDX-License-Identifier: MPL-2.0
cmake_minimum_required(VERSION 3.25)

# Options are declared before project(): the vcpkg toolchain runs inside project() and reads
# VCPKG_MANIFEST_FEATURES, so the dev-only dependencies follow the options.
# Embedded in another project (add_subdirectory), only the static library is built by default:
# no tool, tests, benchmarks or shared library, no LTO of our objects (the host links them), and
# the host's output directories are left alone.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(_ddbt_top ON)
else()
  set(_ddbt_top OFF)
endif()
option(DDBT_BUILD_CLI "Build the ddbt command line tool" ${_ddbt_top})
option(DDBT_BUILD_TESTS "Build the unit tests (vcpkg feature: tests)" ${_ddbt_top})
option(DDBT_BUILD_BENCH "Build bench/ (vcpkg feature: bench)" OFF)
option(DDBT_BUILD_SHARED "Also build libddbt.so / ddbt.dll (ddbt_shared)" ${_ddbt_top})
option(DDBT_LTO "Link time optimization for our code (Release-like configs, no sanitizer)" ${_ddbt_top})
option(DDBT_WERROR "Treat warnings in our code as errors" OFF)
# The baseline is the least CPU the library runs on. Its SIMD kernels (Highway) and the other
# hand-written fast paths (CRC-32C, fpnge) are compiled for SSE4, AVX2, AVX-512 and Zen 4 as well
# and chosen at run time for the CPU, so plain x86-64 loses only the compiler's own use of newer
# instructions in the rest of the code.
set(DDBT_ISA_BASELINE "x86-64" CACHE STRING "Baseline ISA of our code: x86-64, x86-64-v2, x86-64-v3 or native (Highway dispatches above it)")
set_property(CACHE DDBT_ISA_BASELINE PROPERTY STRINGS x86-64 x86-64-v2 x86-64-v3 native)
set(DDBT_PGO "OFF" CACHE STRING "Profile guided optimization: OFF, GEN (instrumented) or USE")
set_property(CACHE DDBT_PGO PROPERTY STRINGS OFF GEN USE)
set(DDBT_PGO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build/pgo-profiles" CACHE PATH "Where PGO profiles are written (GEN) and read (USE)")
set(DDBT_SANITIZE "" CACHE STRING "Sanitizers: empty, asan-ubsan or tsan")
set_property(CACHE DDBT_SANITIZE PROPERTY STRINGS "" asan-ubsan tsan)

if(DDBT_BUILD_TESTS)
  list(APPEND VCPKG_MANIFEST_FEATURES tests)
endif()
if(DDBT_BUILD_BENCH)
  list(APPEND VCPKG_MANIFEST_FEATURES bench)
endif()

project(ddbt VERSION 0.2.2 LANGUAGES C CXX
        DESCRIPTION "Fast, light COG and COPC builders and tilers"
        HOMEPAGE_URL "https://github.com/DroneDB/ddb-tilers")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
if(PROJECT_IS_TOP_LEVEL)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
  endif()
endif()

include(DdbtFlags)       # ddbt_flags: ISA, optimization, LTO, linker gc-sections
include(DdbtWarnings)    # ddbt_warnings: warnings for our code only
include(DdbtSanitizers)  # ddbt_sanitizers
include(DdbtPgo)         # ddbt_pgo

# Dependencies (vcpkg manifest, static). Each one is linked into the library so that the
# later lanes can use them; tests/unit/deps_check_test.cpp proves that each one links.
find_package(PROJ CONFIG REQUIRED)
find_package(libdeflate CONFIG REQUIRED)
find_package(libjpeg-turbo CONFIG REQUIRED)
find_package(WebP CONFIG REQUIRED)
find_package(zstd CONFIG REQUIRED)
find_package(OpenJPEG CONFIG REQUIRED)
find_package(unofficial-lerc CONFIG REQUIRED)
find_package(hwy CONFIG REQUIRED)
add_subdirectory(third_party/lazperf)  # ddbt::lazperf: in-tree laz-perf fork (third_party/lazperf/FORK.md)
add_subdirectory(third_party/fpnge)    # ddbt::fpnge: fast PNG encoder (third_party/fpnge/README.ddbt.md)
include(DdbtDeps)       # ddbt::deps interface target aliasing the imported targets above

add_subdirectory(src)

if(DDBT_BUILD_CLI)
  add_subdirectory(apps/cli)
endif()

if(DDBT_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(DDBT_BUILD_BENCH)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/bench/CMakeLists.txt")
    add_subdirectory(bench)
  endif()
  # bench/CMakeLists.txt adds bench/micro (per-area *_bench.cpp microbenchmarks)
endif()

message(STATUS "ddbt ${PROJECT_VERSION}: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}, "
               "${CMAKE_BUILD_TYPE}, ISA baseline ${DDBT_ISA_BASELINE}, LTO ${DDBT_LTO_ACTIVE}, "
               "PGO ${DDBT_PGO}, sanitize '${DDBT_SANITIZE}'")
