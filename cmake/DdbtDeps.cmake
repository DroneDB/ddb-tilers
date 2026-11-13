# SPDX-License-Identifier: MPL-2.0
#
# ddbt::deps: every third-party library of the manifest behind one interface target.
#
# Some vcpkg ports name their CMake target after the linkage of the triplet (static in our
# triplets, often shared in a host's): the first target that exists is used.
function(ddbt_pick_target out)
  foreach(_candidate IN LISTS ARGN)
    if(TARGET ${_candidate})
      set(${out} ${_candidate} PARENT_SCOPE)
      return()
    endif()
  endforeach()
  list(JOIN ARGN ", " _names)
  message(FATAL_ERROR "ddbt: none of these targets exists: ${_names}")
endfunction()

ddbt_pick_target(_ddbt_libdeflate libdeflate::libdeflate_static libdeflate::libdeflate_shared libdeflate::libdeflate)
ddbt_pick_target(_ddbt_turbojpeg libjpeg-turbo::turbojpeg-static libjpeg-turbo::turbojpeg)
# The static TurboJPEG archive holds the libjpeg API (jpeg_*) the codecs call; the shared
# TurboJPEG library does not export it, so a shared build links libjpeg too.
set(_ddbt_libjpeg "")
if(_ddbt_turbojpeg STREQUAL "libjpeg-turbo::turbojpeg")
  ddbt_pick_target(_ddbt_libjpeg libjpeg-turbo::jpeg)
endif()
ddbt_pick_target(_ddbt_zstd zstd::libzstd zstd::libzstd_static zstd::libzstd_shared)

add_library(ddbt_deps INTERFACE)
add_library(ddbt::deps ALIAS ddbt_deps)
target_link_libraries(ddbt_deps INTERFACE
  PROJ::proj
  ${_ddbt_libdeflate}
  ${_ddbt_turbojpeg}
  ${_ddbt_libjpeg}
  WebP::webp
  WebP::sharpyuv
  ${_ddbt_zstd}
  openjp2
  unofficial::Lerc::Lerc
  hwy::hwy
  ddbt::lazperf
  ddbt::fpnge)
