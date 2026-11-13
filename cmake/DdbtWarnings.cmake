# SPDX-License-Identifier: MPL-2.0
#
# ddbt_warnings: warnings for our code only. Third-party headers come in through imported
# targets (SYSTEM), so they do not warn.
add_library(ddbt_warnings INTERFACE)
add_library(ddbt::warnings ALIAS ddbt_warnings)

if(MSVC)
  target_compile_options(ddbt_warnings INTERFACE /W4 /w14242 /w14254 /w14263 /w14265 /w14287
    /we4289 /w14296 /w14311 /w14545 /w14546 /w14547 /w14549 /w14555 /w14619 /w14640 /w14826
    /w14905 /w14906 /w14928)
  if(DDBT_WERROR)
    target_compile_options(ddbt_warnings INTERFACE /WX)
  endif()
else()
  target_compile_options(ddbt_warnings INTERFACE
    -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Wformat=2
    -Wcast-align -Wnull-dereference -Wdouble-promotion -Wimplicit-fallthrough -Wundef
    -Wnon-virtual-dtor -Woverloaded-virtual
    $<$<COMPILE_LANGUAGE:CXX>:-Wold-style-cast>)
  if(DDBT_WERROR)
    target_compile_options(ddbt_warnings INTERFACE -Werror)
  endif()
endif()
