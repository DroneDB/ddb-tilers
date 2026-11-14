# ddb-tilers

[![CI](https://github.com/DroneDB/ddb-tilers/actions/workflows/ci.yml/badge.svg)](https://github.com/DroneDB/ddb-tilers/actions/workflows/ci.yml)

Fast, light builders and tilers for Cloud Optimized GeoTIFF (COG) rasters and Cloud Optimized
Point Cloud (COPC) files, written in C++20 for Linux and Windows.

ddb-tilers is part of [DroneDB](https://github.com/DroneDB/DroneDB). Its goal is to improve the
COG and COPC build process and the tiling of both formats: faster builds, faster web map tiles,
less memory, and output that does not depend on the number of threads. The output contract is
DroneDB's: the same COG grid, codecs and statistics sidecar, the same tile coordinates and
pixels, so a tile server can switch between the two and compare them.

Status: version 0.3.0, under active development. The C API is not stable yet.

## What it does

| Component | Input | Output |
|---|---|---|
| COG builder | GeoTIFF (any sample type and band count) | EPSG:3857 COG on the GoogleMapsCompatible grid, overviews, `<output>.stats.json` |
| COG tiler | GeoTIFF / COG | 256 or 512 px XYZ / TMS tiles in PNG, JPEG or WebP |
| COPC builder | LAS / LAZ (point formats 0-10), PLY, E57, XYZ, PTS, one or several | COPC (LAS 1.4, point format 6, 7 or 8), `<output>.meta.json` |
| COPC tiler | COPC | 256 or 512 px XYZ / TMS tiles in PNG, JPEG or WebP |

**COG builder.** One streamed pass: full-resolution blocks are warped row by row of blocks,
compressed in parallel and written in order, and each overview row is computed as soon as the
rows it reads exist. Every source block is decoded once. Details:

- Grid and resampling as DroneDB's COG build: GDAL's suggested warp output, `UPPER` zoom
  strategy, bilinear warp, cubic overviews (mask-aware).
- JPEG quality 75 YCbCr for 8-bit RGB(A) without nodata (a 1-bit mask for the alpha), else LZW
  (default), DEFLATE or ZSTD with a predictor.
- Fully transparent tiles are not written (sparse COG); classic TIFF unless the file is larger
  than 4 GiB.
- Exact statistics are accumulated during the pass (no second read).
- The output appears atomically (a temporary file, then a rename).

**COG tiler.** One shared, immutable reader per file and a decoded-block cache with one byte
budget for the process. A tile of an EPSG:3857 COG is a crop of at most four blocks of the
right overview; other CRSs go through a warp with a bounded-error approximate transform.
Rendering is fused into the encoder (no intermediate image). Tiles with visualisation
parameters are supported: band presets and selections, vegetation and thermal formulas (NDVI,
VARI, EVI, ...), colormaps, rescale and a NoData override.

**COPC builder.** A plan from the headers and a sample, one parallel pass that decodes and
distributes every point into octree bins (in memory or in one spill file), then partitions
built in parallel with a deterministic bottom-up level of detail. The output is byte-identical
whatever the number of threads and the memory budget. The memory cap is 4 GiB by default
(`memoryMb`, `DDBT_COPC_BUILD_MEM_MB`), at most half of the physical memory. A colourless cloud
stays colourless (point format 6, coloured by elevation in the tiles).

**COPC tiler.** A flattened octree index, a node cache of 6-16 bytes per point, layer-selective
LAZ decoding (only X, Y, Z and colour), a per-pixel z-test that does not depend on the order of
the points. Three modes (`DDBT_COPC_TILER_MODE`):

- `compat`: DroneDB 2.4.1's tiles, bit for bit.
- `hybrid` (default): the same look without the known defects (points drawn at their true
  position, no seams at zoom 21 and above, one 8/16-bit colour decision per cloud).
- `modern`: also a symmetric point disc and a level of detail from the projected point spacing.

## Performance

Measured against DroneDB 2.4.1 on the same machine (AMD Ryzen 9 9950X, Linux, NVMe), with the
same inputs: medians of 3 to 5 runs, interleaved. Ratios below 1 mean that ddb-tilers is faster
or uses less memory.

| Builds (8 threads) | Wall time | Peak memory |
|---|---|---|
| COG, 8 rasters (1 to 53 bands, 8 to 32 bits) | 0.08x to 0.29x | 0.25x to 0.65x, 1.15x on 53 Float32 bands |
| COPC, 4 point clouds (0.9 to 88.8 million points) | 0.18x to 0.71x | 0.68x to 3.8x (the 4 GiB cap is the next target) |

| Tiles (1 to 16 threads, cold and warm caches, PNG / JPEG / WebP) | Rasters | Point clouds |
|---|---|---|
| Median latency | 0.11x to 0.29x | 0.06x to 0.39x |
| Throughput at 16 threads, warm, PNG 256 px | 4.8x to 20x | 5x to 200x |
| Peak memory | 0.30x to 0.56x | 0.35x to 0.55x (1.2x to 1.4x on one cloud at 1 thread) |

Raster tiles are pixel-identical to DroneDB 2.4.1's on the test rasters (DroneDB re-encodes JPEG
COGs, so those differ within JPEG noise). The COGs pass GDAL's COG validator and have DroneDB's
grid. WebP tiles use libwebp method 1 (DroneDB uses 2): 12-41 % more
tiles per second, a slightly higher PSNR, at most 8 % more bytes.

## Build

Requirements:

- CMake 3.25 or later and Ninja.
- Linux: GCC 13 or later. Windows: Visual Studio 2022 (MSVC, x64).
- A [vcpkg](https://github.com/microsoft/vcpkg) checkout in `VCPKG_ROOT`. The manifest pins the
  baseline in `vcpkg-configuration.json`. All dependencies are static and release-only.
- NASM on Linux (`apt install nasm`): the libjpeg-turbo overlay port in `ports/` refuses to build
  without its SIMD code. On Windows vcpkg downloads NASM.
- Any x86-64 CPU. The SIMD code (Highway kernels, the fpnge PNG encoder, CRC-32C) and the hot
  scalar loops (band statistics, overview input rows, warped pixel mapping, grid interpolation,
  the Morton keys of the COPC builder with BMI2 PDEP / PEXT) are compiled for SSE2, SSSE3, SSE4,
  AVX2, AVX-512 and Zen 4 and the best variant the CPU runs is chosen at start-up, with the same
  output on every variant; the rest of the code is compiled for plain x86-64
  (`DDBT_ISA_BASELINE`: `x86-64-v2`, `x86-64-v3` or `native` raise it). `scripts/check-baseline-isa.py` checks a build for
  instructions above the baseline outside that dispatched code.

```sh
export VCPKG_ROOT=/path/to/vcpkg
cmake --preset linux-gcc-release
cmake --build build/linux-gcc-release
ctest --preset linux-gcc-release
```

On Windows, from a "x64 Native Tools" prompt: `cmake --preset windows-msvc-release`, then the
same build and test commands with `windows-msvc-release`.

Presets (`CMakePresets.json`):

| Preset | Use |
|---|---|
| `linux-gcc-release`, `windows-msvc-release` | Release, LTO, x86-64 baseline with run-time SIMD dispatch, benchmarks included |
| `linux-gcc-relwithdebinfo` | Release with frame pointers and debug information (profiling) |
| `linux-gcc13-release` | Release with GCC 13 |
| `linux-asan-ubsan`, `linux-tsan`, `windows-msvc-asan` | Sanitizers |
| `linux-pgo-gen`, `linux-pgo-use` | Profile-guided optimisation |

Embedded in another CMake project (`add_subdirectory`), only the static library `ddbt::ddbt` is
built by default (no tool, tests, benchmarks or shared library) and the host's output
directories are left alone; the laz-perf fork is available as `ddbt::lazperf`. The host passes
its `proj.db` folder with `ddbtConfigure`.

The build produces the static library `libddbt.a` / `ddbt_static.lib`, the shared library
`libddbt.so` / `ddbt.dll` (C API only), the `ddbt` command line tool and the test binaries.

## Use

Command line:

```sh
ddbt cog build input.tif output.tif '{"codec": "zstd"}'
ddbt copc build input.laz output.copc.laz '{"memoryMb": 2048}'
ddbt info --cpu        # version, build and the SIMD targets selected at run time (JSON)
```

C API (`include/ddbt/ddbt.h`; the functions follow DroneDB's libddb signatures):

| Function | |
|---|---|
| `ddbtBuildCog(input, output, optionsJson)` | Build a COG. Options: `codec` (`lzw`, `deflate`, `zstd`), `level`, `jpegQuality`, `noJpeg`, `memoryMb`, `statistics`, `report` (path of a JSON build report) |
| `ddbtBuildCopc(input, output, optionsJson)` | Build a COPC. Options: `inputs` (more files to merge), `threads`, `memoryMb`, `tempDir`, `metadata`, `metadataPath`, `report` |
| `ddbtBuildCog2(...)`, `ddbtBuildCopc2(...)` | The same builds with a progress callback (fraction and phase) |
| `ddbtMemoryTileFmt(...)` | A tile of a COG or COPC file in PNG, JPEG or WebP |
| `ddbtMemoryTileEx2(...)`, `ddbtMemoryTileEx3(...)` | A raster tile with visualisation parameters, in PNG (Ex2) or a chosen format (Ex3) |
| `ddbtTileInfo(input, tileSize, outJson)` | Bounds (EPSG:3857) and zoom range of a COG or COPC file |
| `ddbtReleaseSources()` | Closes the files the tilers keep open between calls |
| `ddbtConfigure(configJson)` | Process-wide settings of a host that embeds the library: `projDataDir`, `threads`, cache budgets (`blockCacheMb`, `tileCacheMb`, `copcNodeCacheMb`, `copcTileCacheMb`), `copcTilerMode`, `copcBuildMemMb`; they win over the environment variables |
| `ddbtFree`, `ddbtLastError`, `ddbtVersion`, `ddbtCpuInfo` | Memory release, error message of the last call on the thread, version, CPU dispatch |

Every function returns a `ddbtErr`; no exception crosses the C API.

Environment variables (all optional):

| Variable | Default | Effect |
|---|---|---|
| `DDBT_THREADS` | every CPU | Threads of the default pool |
| `DDBT_BLOCK_CACHE_MB` | 3 % of the memory, 64 MiB to 1 GiB | Decoded-block cache of the COG tiler (0 disables) |
| `DDBT_TILE_CACHE_MB` | 256 | Encoded-tile cache of the COG tiler |
| `DDBT_TILE_REQUEST_MB` | 1024 | Working memory one raster tile request may use |
| `DDBT_COPC_NODE_CACHE_MB` | 512 | Decoded-node cache of the COPC tiler |
| `DDBT_COPC_TILE_CACHE_MB` | 64 | Encoded-tile cache of the COPC tiler |
| `DDBT_COPC_TILER_MODE` | `hybrid` | `compat`, `hybrid` or `modern` |
| `DDBT_COPC_BUILD_MEM_MB` | 4096 | Memory cap of the COPC builder |
| `DDBT_WEBP_METHOD` | 1 | libwebp method (0-6) of WebP tiles |
| `DDBT_PNG_ENGINE` | `deflate` | `fpnge`: faster PNG encoding, larger files |
| `DDBT_TILER_IO` | `pread` | `mmap` maps the COG files instead of positional reads |
| `DDBT_PROJ_DATA` | the build's PROJ data | Directory of `proj.db` |

## Tests and benchmarks

`ctest` runs the unit tests (GoogleTest). The tests use synthetic data. Tests that need the
DroneDB test data or the local benchmark corpus skip when these are absent.

`bench/` holds the benchmark tools: `ddbt-bench` (a driver that loads either backend and
measures tile requests and builds: latency percentiles, tiles per second, CPU time, peak
memory, I/O), Google Benchmark microbenchmarks (`ddbt_micro`) and a Python harness
(`bench/harness`) that runs suites (`bench/suites/*.toml`) with interleaved A/B repetitions,
quiet-host gates and bootstrap confidence intervals. `scripts/bench-ab.sh` compares two commits.
The harness needs the benchmark corpus and a DroneDB build to compare against; `DDBT_WORK`
selects its work directory.

## Third-party code

- `third_party/lazperf`: a fork of [laz-perf](https://github.com/hobuinc/laz-perf) 3.4.0
  (Apache-2.0) with layer-selective decoding, decoder reuse and point formats 4, 5, 9 and 10.
  `third_party/lazperf/FORK.md` lists every change.
- `third_party/fpnge`: [fpnge](https://github.com/veluca93/fpnge) (Apache-2.0), unmodified.
- Through vcpkg: PROJ, libdeflate, libjpeg-turbo, libwebp, zstd, OpenJPEG, LERC, Google Highway,
  GoogleTest and Google Benchmark.

## License

[MPL-2.0](LICENSE). The licences of the third-party code and libraries are listed in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
