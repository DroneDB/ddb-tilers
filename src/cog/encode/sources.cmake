# SPDX-License-Identifier: MPL-2.0
# COG builder: block encoders (predictors, LZW / DEFLATE / ZSTD / JPEG tiles, 1-bit masks, JPEG
# tables and quality). Paths relative to src/.
list(APPEND DDBT_SOURCES
  cog/encode/block_encoder.cpp)
