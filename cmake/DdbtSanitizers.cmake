# SPDX-License-Identifier: MPL-2.0
#
# ddbt_sanitizers: DDBT_SANITIZE = "" | asan-ubsan | tsan. Applied to our targets AND to the
# tests/CLI that link them (it is an INTERFACE target linked by everything we build). The vcpkg
# libraries are not instrumented.
add_library(ddbt_sanitizers INTERFACE)
add_library(ddbt::sanitizers ALIAS ddbt_sanitizers)

if(NOT DDBT_SANITIZE STREQUAL "")
  if(NOT DDBT_SANITIZE MATCHES "^(asan-ubsan|tsan)$")
    message(FATAL_ERROR "DDBT_SANITIZE must be empty, asan-ubsan or tsan (got '${DDBT_SANITIZE}')")
  endif()
  if(MSVC)
    if(DDBT_SANITIZE STREQUAL "asan-ubsan")
      # MSVC has AddressSanitizer only (no UBSan, no TSan)
      target_compile_options(ddbt_sanitizers INTERFACE /fsanitize=address)
      target_link_options(ddbt_sanitizers INTERFACE /INCREMENTAL:NO)
    else()
      message(FATAL_ERROR "MSVC does not support DDBT_SANITIZE=tsan")
    endif()
  else()
    if(DDBT_SANITIZE STREQUAL "asan-ubsan")
      set(_san "-fsanitize=address,undefined" "-fno-sanitize-recover=undefined")
      target_compile_options(ddbt_sanitizers INTERFACE ${_san} -fno-omit-frame-pointer)
      target_link_options(ddbt_sanitizers INTERFACE ${_san})
    else()
      set(_san "-fsanitize=thread")
      target_compile_options(ddbt_sanitizers INTERFACE ${_san} -fno-omit-frame-pointer)
      target_link_options(ddbt_sanitizers INTERFACE ${_san})
    endif()
    # sanitizer builds: readable stacks, light optimization
    target_compile_options(ddbt_sanitizers INTERFACE -O1 -g)
  endif()
endif()
