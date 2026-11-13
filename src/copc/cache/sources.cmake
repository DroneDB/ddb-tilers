# SPDX-License-Identifier: MPL-2.0
# COPC node cache (process-wide ShardedLru of compact nodes, single flight, parallel misses).
# Paths relative to src/.
list(APPEND DDBT_SOURCES
  copc/cache/node_cache.cpp)
