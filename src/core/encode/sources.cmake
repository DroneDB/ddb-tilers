# SPDX-License-Identifier: MPL-2.0
# Tile encoders: own PNG (Highway filters + libdeflate), JPEG (TurboJPEG 3), WebP (libwebp).
list(APPEND DDBT_SOURCES
  core/encode/kernels.cpp
  core/encode/tile_image.cpp
  core/encode/png_encoder.cpp
  core/encode/jpeg_encoder.cpp
  core/encode/webp_encoder.cpp
  core/encode/tile_encoder.cpp)
