# SPDX-License-Identifier: MPL-2.0
# C API exports beyond src/api/ddbt_api.cpp (listed in src/CMakeLists.txt). Paths relative to src/.
list(APPEND DDBT_SOURCES
  api/api_common.cpp
  api/host_api.cpp
  api/tile_api.cpp
  api/build_api.cpp)
