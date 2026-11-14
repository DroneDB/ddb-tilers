# Third-party notices

ddb-tilers is licensed under the Mozilla Public License 2.0 ([LICENSE](LICENSE)). It contains
or links the third-party code below. Every licence here is permissive and compatible with
MPL-2.0; no GPL or LGPL library is linked.

A binary distribution of ddb-tilers (the static or shared library, or a program that embeds
it) must ship the licence texts of the components it contains. The vcpkg dependencies install
theirs in `<build>/vcpkg_installed/<triplet>/share/<port>/copyright`.

## Code in this repository

| Component | Where | Licence | Notes |
|---|---|---|---|
| laz-perf 3.4.0, modified | `third_party/lazperf` | Apache-2.0 | `LICENSE` and `NOTICE` in the folder; every change is listed in `FORK.md` and marked with a `ddbt fork` comment |
| fpnge (commit 097ebccf) | `third_party/fpnge` | Apache-2.0 | Unmodified (compiled twice for SSE 4.1 and AVX2 by the wrappers in `variants/`); `LICENSE` in the folder |
| GDAL 3.13.3, ported code | `src/core/crs/gcp_transform.cpp` | MIT | The GCP transforms of `alg/gdal_crs.cpp` (with the GRASS CRS.C routines, Copyright (c) 1992 Michigan State University), `alg/gdal_homography.cpp`, `alg/gdallinearsystem.cpp` and `alg/gdaltransformer.cpp`, ported to C++20; the copyright notices and the MIT permission notice are at the top of the file |
| GDAL 3.13.3, ported code | `src/cog/warp/row_approx.cpp`, `src/cog/warp/warp_chunks.cpp`, `src/cog/warp/source_georef.cpp` | MIT | The approximate transformer of `alg/gdaltransformer.cpp` (GDALApproxTransform) and its GDALInvGeoTransform, and the warp chunk list and source windows of `alg/gdalwarpoperation.cpp` (GDALWarpOperation), ported to C++20; the copyright notices and the MIT permission notice are at the top of each file |
| vcpkg port of libjpeg-turbo, modified | `ports/libjpeg-turbo` | MIT | Build script only, not linked; see `ports/NOTICE.libjpeg-turbo` |

## Libraries linked through vcpkg (static)

| Library | Licence | Pulled in by |
|---|---|---|
| PROJ | MIT (the EPSG data in `proj.db` follows the EPSG terms of use) | ddbt |
| libdeflate | MIT | ddbt |
| libjpeg-turbo | IJG, BSD-3-Clause, zlib | ddbt |
| libwebp | BSD-3-Clause (with Google's patent grant) | ddbt |
| zstd | BSD-3-Clause (dual licensed with GPL-2.0; used under BSD) | ddbt |
| OpenJPEG | BSD-2-Clause | ddbt |
| LERC | Apache-2.0 | ddbt |
| Highway | Apache-2.0 or BSD-3-Clause | ddbt |
| libjxl | BSD-3-Clause (with Google's patent grant) | ddbt (JPEG-XL TIFF blocks) |
| Brotli | MIT | libjxl |
| Little CMS (lcms2) | MIT | libjxl |
| libtiff | libtiff (BSD-style) | ddbt (the TIFF compressions without a codec of its own), PROJ |
| SQLite | Public domain | PROJ |
| nlohmann/json | MIT (header-only) | PROJ |
| zlib | zlib | libtiff, libspng |
| liblzma (XZ Utils) | 0BSD | libtiff |
| libspng | BSD-2-Clause | libjpeg-turbo (TurboJPEG image I/O) |

The IJG licence asks binary distributions to state: "This software is based in part on the work
of the Independent JPEG Group."

Development only (not part of the library): GoogleTest (BSD-3-Clause) and Google Benchmark
(Apache-2.0).

The test samples in `tests/fixtures/gdal/` come from GDAL's test suite (MIT/X style licence,
`tests/fixtures/gdal/GDAL-LICENSE.TXT`); they are not part of the library. The COGs
`tests/fixtures/gcp/gdal/gdal_*.tif` are GDAL's warps of three of them (same licence).
