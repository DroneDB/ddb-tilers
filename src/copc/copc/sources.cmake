# SPDX-License-Identifier: MPL-2.0
# COPC reader: CopcFile, HierarchyIndex (flattened octree, PDAL-identical node bounds, pruned
# selection), NodeDecoder (layered LAZ decode into arrays or compact nodes). Paths relative to
# src/.
list(APPEND DDBT_SOURCES
  copc/copc/copc_file.cpp
  copc/copc/hierarchy.cpp
  copc/copc/node_decoder.cpp)
