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
| libtiff | libtiff (BSD-style) | PROJ |
| SQLite | Public domain | PROJ |
| nlohmann/json | MIT (header-only) | PROJ |
| zlib | zlib | libtiff, libspng |
| liblzma (XZ Utils) | 0BSD | libtiff |
| libspng | BSD-2-Clause | libjpeg-turbo (TurboJPEG image I/O) |

The IJG licence asks binary distributions to state: "This software is based in part on the work
of the Independent JPEG Group."

Development only (not part of the library): GoogleTest (BSD-3-Clause) and Google Benchmark
(Apache-2.0).
