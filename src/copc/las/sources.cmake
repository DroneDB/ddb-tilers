# SPDX-License-Identifier: MPL-2.0
# LAS / LAZ / COPC structures: header 1.0-1.4, VLRs and EVLRs, extra-bytes descriptors, LAZ and
# COPC info VLRs, point formats 0-10, CRS from WKT / GeoTIFF keys. Paths relative to src/.
list(APPEND DDBT_SOURCES
  copc/las/las_header.cpp
  copc/las/las_crs.cpp)
