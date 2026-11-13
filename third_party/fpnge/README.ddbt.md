# fpnge in ddb-tilers

Upstream: https://github.com/veluca93/fpnge, commit `097ebccf3d3db443d9a8f238b1deed010a82f961` (2025-10-09).
Files: `fpnge.cc`, `fpnge.h`, `LICENSE` (Apache-2.0, Copyright 2021 Google LLC), copied unmodified.

Used by `src/core/encode/png_encoder.cpp` for grey, grey + alpha, RGB and RGBA tiles when the PNG engine is fpnge
(`PngOptions::engine` or `DDBT_PNG_ENGINE=fpnge`; libdeflate is the default: fpnge encodes raster tiles 4.6-5.1x faster
but 20-26 % larger, which is slower end to end below about 300 Mbit/s). Palette tiles and the blank-tile cache always
keep the libdeflate path.
