# SPDX-License-Identifier: MPL-2.0
# COPC builder: plan, scatter (memory bins or a spill file), deterministic bottom-up LOD,
# node encoding, COPC writer, metadata sidecar. Paths relative to src/.
list(APPEND DDBT_SOURCES
  copc/build/bin_store.cpp
  copc/build/lod.cpp
  copc/build/node_encoder.cpp
  copc/build/copc_writer.cpp
  copc/build/sidecar.cpp
  copc/build/copc_check.cpp
  copc/build/copc_builder.cpp)
# The C API entry point of the builder (ddbtBuildCopc), listed here so src/api/sources.cmake
# stays untouched by this area.
list(APPEND DDBT_SOURCES api/build_copc_api.cpp)
