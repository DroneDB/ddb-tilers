# SPDX-License-Identifier: MPL-2.0
# Point cloud readers of the COPC builder: LAS / LAZ, PLY, E57, XYZ / PTS. Paths relative to src/.
list(APPEND DDBT_SOURCES
  copc/readers/las_reader.cpp
  copc/readers/e57_reader.cpp
  copc/readers/ply_reader.cpp
  copc/readers/text_reader.cpp
  copc/readers/open_reader.cpp)
