# SPDX-License-Identifier: MPL-2.0
# TIFF / BigTIFF / COG reader: IFDs, ghost header, block index, block decoding, GeoTIFF keys,
# geotransform, CRS and sidecar files. Paths relative to src/.
list(APPEND DDBT_SOURCES
  cog/tiff/tiff_reader.cpp
  cog/tiff/block_reader.cpp)
list(APPEND DDBT_SOURCES
  cog/tiff/geotiff.cpp
  cog/tiff/geotiff_crs.cpp)
