# SPDX-License-Identifier: MPL-2.0
#
# ddbt::deps: every third-party library of the manifest behind one interface target.
add_library(ddbt_deps INTERFACE)
add_library(ddbt::deps ALIAS ddbt_deps)
target_link_libraries(ddbt_deps INTERFACE
  PROJ::proj
  libdeflate::libdeflate_static
  libjpeg-turbo::turbojpeg-static
  WebP::webp
  WebP::sharpyuv
  zstd::libzstd
  openjp2
  unofficial::Lerc::Lerc
  hwy::hwy
  ddbt::lazperf
  ddbt::fpnge)
