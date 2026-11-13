# SPDX-License-Identifier: MPL-2.0
# TIFF / COG block codecs. Decode side: LZW (own), PackBits, DEFLATE (libdeflate), ZSTD, LERC,
# WebP, JPEG (libjpeg-turbo), predictors 2 / 3 and byte swapping (Highway). Encode side: LZW (own,
# lzw_encoder.cpp, for the COG builder). Paths relative to src/.
list(APPEND DDBT_SOURCES
  cog/codecs/lossless.cpp
  cog/codecs/lzw.cpp
  cog/codecs/lerc_webp.cpp
  cog/codecs/jpeg.cpp
  cog/codecs/predictor.cpp
  cog/codecs/lzw_encoder.cpp)
