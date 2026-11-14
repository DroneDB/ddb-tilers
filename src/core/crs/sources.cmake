# SPDX-License-Identifier: MPL-2.0
# CRS: PROJ wrapper (per-thread contexts, cached transforms), GridApprox, polynomial fits,
# analytic EPSG:4326 <-> EPSG:3857, GCP transforms. Paths relative to src/.
list(APPEND DDBT_SOURCES
  core/crs/proj_context.cpp
  core/crs/crs.cpp
  core/crs/transform.cpp
  core/crs/web_mercator_analytic.cpp
  core/crs/grid_approx.cpp
  core/crs/grid_approx_kernels.cpp
  core/crs/polyfit.cpp
  core/crs/gcp_transform.cpp)

# Development default for proj.db: the share/proj of the PROJ we link (vcpkg_installed of this
# build; PROJ_DIR is where find_package(PROJ) found proj-config.cmake). Overridden at run time by
# DDBT_PROJ_DATA or ddbt::crs::SetProjDataDir(). Embedded, there is no default: the build tree is
# not where the host is deployed, so the host passes its proj.db folder (ddbtConfigure), else
# PROJ's own search (PROJ_DATA) applies.
if(NOT PROJECT_IS_TOP_LEVEL)
  # no compiled default
elseif(DEFINED PROJ_DIR AND EXISTS "${PROJ_DIR}/proj.db")
  set_property(SOURCE core/crs/proj_context.cpp APPEND PROPERTY
    COMPILE_DEFINITIONS "DDBT_PROJ_DATA_DEFAULT=\"${PROJ_DIR}\"")
else()
  message(WARNING "crs: proj.db not found next to PROJ's CMake config (${PROJ_DIR}); set DDBT_PROJ_DATA at run time")
endif()

# The analytic Web Mercator forms and the GCP transforms reproduce PROJ's / GDAL's bits, and the
# GridApprox kernel the scalar code's explicit FMAs: no other contraction, and no LTO (with LTO
# the per-file flag is lost when the code is inlined at link time).
if(NOT MSVC)
  set_source_files_properties(core/crs/web_mercator_analytic.cpp core/crs/grid_approx_kernels.cpp
    core/crs/gcp_transform.cpp
    PROPERTIES COMPILE_OPTIONS "-ffp-contract=off;-fno-lto")
endif()
