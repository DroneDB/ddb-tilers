# SPDX-License-Identifier: MPL-2.0
find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

add_subdirectory(unit)

# Integration tests (synthetic fixtures), when present.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/integration/CMakeLists.txt")
  add_subdirectory(integration)
endif()
