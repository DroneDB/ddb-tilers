# SPDX-License-Identifier: MPL-2.0

# Unit tests of the library (internal headers from src/ are visible).
# Every *_test.cpp under tests/unit/<area>/ joins the unit test binary (plus the two top-level ones).
file(GLOB_RECURSE DDBT_AREA_TESTS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*/*_test.cpp")
list(SORT DDBT_AREA_TESTS)
add_executable(ddbt_unit_tests
  api_test.cpp
  dispatch_test.cpp
  ${DDBT_AREA_TESTS})
target_include_directories(ddbt_unit_tests PRIVATE "${PROJECT_SOURCE_DIR}/src")
target_link_libraries(ddbt_unit_tests PRIVATE ddbt::ddbt GTest::gtest_main)
ddbt_apply_build_settings(ddbt_unit_tests)
gtest_discover_tests(ddbt_unit_tests DISCOVERY_TIMEOUT 60)

# One test per third-party library: proves that it is found, headers match the linked code and
# the static archive links (a tiny call into each).
add_executable(ddbt_deps_check deps_check_test.cpp)
target_link_libraries(ddbt_deps_check PRIVATE ddbt::deps GTest::gtest_main)
ddbt_apply_build_settings(ddbt_deps_check)
gtest_discover_tests(ddbt_deps_check DISCOVERY_TIMEOUT 60)

# The shared library must expose the C API (it is dlopen()ed by the bench driver).
if(TARGET ddbt_shared)
  add_executable(ddbt_shared_check shared_check_test.cpp)
  target_link_libraries(ddbt_shared_check PRIVATE ddbt::shared GTest::gtest_main)
  ddbt_apply_build_settings(ddbt_shared_check)
  gtest_discover_tests(ddbt_shared_check DISCOVERY_TIMEOUT 60)
endif()
