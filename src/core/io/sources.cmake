# SPDX-License-Identifier: MPL-2.0
# io: positional files, mappings, ByteSource, read coalescing, atomic OutFile, I/O counters.
list(APPEND DDBT_SOURCES
  core/io/byte_source.cpp
  core/io/coalesce.cpp
  core/io/file.cpp
  core/io/io_stats.cpp
  core/io/mapped_file.cpp
  core/io/out_file.cpp
  core/io/platform_posix.cpp
  core/io/platform_win.cpp)
