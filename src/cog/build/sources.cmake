# SPDX-License-Identifier: MPL-2.0
# COG builder: source reader, exact statistics and sidecar, the build pipeline (own grid and
# warped). Paths relative to src/.
list(APPEND DDBT_SOURCES
  cog/build/source.cpp
  cog/build/tiff_source.cpp
  cog/build/jp2_source.cpp
  cog/build/image_source.cpp
  cog/build/jpeg_source.cpp
  cog/build/png_source.cpp
  cog/build/gif_source.cpp
  cog/build/webp_source.cpp
  cog/build/stats.cpp
  cog/build/cog_builder.cpp)
# The statistics arithmetic is cog/tiler/stats_kernel.cpp (compiled without contraction or LTO).
