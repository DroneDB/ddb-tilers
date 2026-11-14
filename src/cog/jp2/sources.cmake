# SPDX-License-Identifier: MPL-2.0
# JPEG 2000 reader: the boxes of a JP2 file (codestream, colour boxes, GeoJP2 / GMLJP2 / MSIG
# georeferencing, metadata) and the codestream decoded with OpenJPEG. Paths relative to src/.
list(APPEND DDBT_SOURCES
  cog/jp2/jp2_file.cpp
  cog/jp2/codestream.cpp)
