# SPDX-License-Identifier: MPL-2.0
add_executable(ddbt_cli main.cpp)
set_target_properties(ddbt_cli PROPERTIES OUTPUT_NAME ddbt)
target_link_libraries(ddbt_cli PRIVATE ddbt::ddbt)
ddbt_apply_build_settings(ddbt_cli)
