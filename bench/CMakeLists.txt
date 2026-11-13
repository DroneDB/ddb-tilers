# SPDX-License-Identifier: MPL-2.0
# Benchmark tooling of ddb-tilers. Self-contained: configurable on its own
#   cmake -S bench -B /mnt/data/ddb-tilers-work/build-bench && cmake --build /mnt/data/ddb-tilers-work/build-bench
# or through the top-level project (DDBT_BUILD_BENCH=ON -> add_subdirectory(bench)).
# The driver uses only the C++ standard library, dl and pthread (no vcpkg dependency).
cmake_minimum_required(VERSION 3.20)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(ddbt_bench LANGUAGES CXX)
  set(DDBT_BENCH_STANDALONE ON)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
  endif()
  include(CTest)
else()
  set(DDBT_BENCH_STANDALONE OFF)
endif()

find_package(Threads REQUIRED)

add_executable(ddbt-bench
  driver/main.cpp
  driver/cmd_tile.cpp
  driver/cmd_misc.cpp
  driver/json.cpp
  driver/os_linux.cpp
  driver/os_win.cpp)
target_compile_features(ddbt-bench PRIVATE cxx_std_20)
set_target_properties(ddbt-bench PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(ddbt-bench PRIVATE Threads::Threads ${CMAKE_DL_LIBS})
if(MSVC)
  target_compile_options(ddbt-bench PRIVATE /W4 /permissive-)
  target_link_libraries(ddbt-bench PRIVATE psapi)
else()
  target_compile_options(ddbt-bench PRIVATE -Wall -Wextra)
endif()

# Microbenchmarks (Google Benchmark) only exist inside the top-level build.
if(NOT DDBT_BENCH_STANDALONE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/micro/CMakeLists.txt")
  add_subdirectory(micro)
endif()

# Harness tests (Python unittest). The driver smoke test needs the frozen baseline; it skips
# itself when the baseline library is not found.
find_package(Python3 COMPONENTS Interpreter)
if(BUILD_TESTING AND Python3_Interpreter_FOUND)
  add_test(NAME bench_harness_unit
           COMMAND ${Python3_EXECUTABLE} -m unittest discover -s tests -t . -v
           WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/harness)
  set_tests_properties(bench_harness_unit PROPERTIES
                       ENVIRONMENT "DDBT_BENCH_BIN=$<TARGET_FILE:ddbt-bench>"
                       LABELS "bench")
endif()
