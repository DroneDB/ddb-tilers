# SPDX-License-Identifier: MPL-2.0
# LAZ glue over the laz-perf fork (third_party/lazperf): layered chunk decoder with per-thread
# pools, reference decode and chunk compression. Paths relative to src/.
list(APPEND DDBT_SOURCES
  copc/laz/chunk_decoder.cpp)
