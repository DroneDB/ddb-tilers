# SPDX-License-Identifier: MPL-2.0
# util: Status / Expected, counters, a small XML reader (checked math, endian and timer are
# header-only).
list(APPEND DDBT_SOURCES
  core/util/status.cpp
  core/util/counters.cpp
  core/util/xml_lite.cpp)
