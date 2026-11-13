# laz-perf fork

| | |
|---|---|
| Upstream | https://github.com/hobu/laz-perf |
| Version | 3.4.0 + 2 (`git describe`: 3.4.0-2-g2b68700) |
| Commit | `2b68700d33bf981d257d6101d96714620c64019c` (local clone `/mnt/data/repos/laz-perf`) |
| License | Apache-2.0 (`LICENSE`, `NOTICE`) |
| Imported | `cpp/lazperf/**` as `lazperf/**` (taken with `git archive`, LF line endings as committed upstream), `COPYING` as `LICENSE`, `README.md` as `README.upstream.md` |
| Not imported | `js/`, `cpp/emscripten`, `cpp/benchmarks`, `cpp/examples`, `cpp/test`, `cpp/tools`, `cmake/` (replaced by `CMakeLists.txt`) |

## Why a fork

ddb-tilers reads COPC nodes (one LAZ chunk each) on the tile path and writes them in the COPC
builder. The fork adds what upstream does not offer:

1. Layer-selective decoding of point formats 6-8 (decode XY + Z + RGB(+NIR) only, skip the bytes
   of every other layer). Exact: each layer has its own arithmetic decoder over its own bytes,
   and only the XY layer (changed-values symbol with the GPS-time-changed bit, scanner channel,
   return numbers) feeds the contexts of the others; tested on synthetic chunks of every field
   pattern and on every node of the corpus COPC files against upstream's `chunk_decompressor`.
2. A structure-of-arrays sink (int32 x/y/z, uint16 r/g/b/nir) without packing and unpacking
   LAS records.
3. Decoder reuse across chunks (`reset()`) instead of reallocating every model per chunk.

For the COPC builder: point formats 4 / 5 (WAVEPACKET13) and 9 / 10 (the
WAVEPACKET14 layer skipped) in the readers, the chunk-table offset -1.

## Build

`CMakeLists.txt` builds `ddbt::lazperf`, a static library with hidden visibility
(`LAZPERF_VENDORED`, so `LAZPERF_EXPORT` is empty), with the project's ISA / optimisation /
LTO / sanitizer / PGO settings but without the project's warning set (upstream's own
`-Wno-*` list applies; warnings are never errors here). It is linked into `ddbt::deps`.

## Patch list

Every change in the sources is marked with a `ddbt fork` comment. Keep this list up to date.

| # | Files | Change | Why |
|---|---|---|---|
| 1 | `streams.hpp` | `InCbStream` direct mode: reads a borrowed buffer of known size (bounds-checked, throws `lazperf::error("LAZ chunk is truncated")`), `skip()`, `reset()`, `position()`. New `InViewStream`: decoder input that borrows a layer's bytes from a direct stream (copies them from a callback stream); reads past its end return 0. | Skipped layers are never read; decoded layers are not copied; a truncated or lying chunk cannot read out of bounds (upstream `MemoryStream::getByte` indexes past the vector). |
| 2 | `decoder.hpp`, `model.hpp`, `decompressor.hpp` | `reset()` on `decoders::arithmetic`, `models::arithmetic`, `models::arithmetic_bit`, `decompressors::integer`: back to the constructed state without reallocating. | Decoder reuse across chunks. |
| 3 | `layers.hpp` (new), `las.hpp` | Layer bits (`lazperf::layer::xy`, `z`, ..., `rgb`, `nir`, `extra_bytes`); included by `las.hpp`. | Layer mask. |
| 4 | `detail/field_point14.{hpp,cpp}` | Channel contexts created on first use and only with the integer coders of their side (`Point14Base(bool decoder)`, `ChannelCtxs`); `Point14Decompressor` holds the stream by reference, decodes into `InViewStream` decoders, has `setLayers()` (masked layers: bytes skipped in `readData()`; scan angle, point source and GPS time additionally gated by the mask since they are driven by changed bits of the XY layer), `decompressNext()` (decode into the context, return the point; `decompress()` packs it), `reset()` (resets only the models of layers decoded since the last reset). | Layer-selective decode, SoA output, reuse. A new decompressor costs 85-125 us and ~0.9-1.2 MB of heap instead of 470-510 us and ~5 MB (upstream builds four channel contexts with both sides' coders). |
| 5 | `detail/field_rgb14.*`, `field_nir14.*`, `field_byte14.*` | `InViewStream` decoders, `readData(bool decode)` (skip the layer bytes), `decompressNext()` (RGB, NIR), `reset()`. | Same. |
| 6 | `layered.{hpp,cpp}` (new) | `lazperf::layered_decompressor(format 6-8, ebCount, layers)`: `begin(data, size)` per chunk (models reset, no allocation), `decompress(char*)` (LAS record) and `decompress(soa_sink, n)` (int32 x/y/z, uint16 r/g/b/nir arrays), `chunkPointCount()`, `consumed()`. | The COPC node decode entry point. |
| 7 | `utils.hpp`, `compressor.hpp`, `decompressor.hpp`, `detail/field_point14.cpp` | Two's complement wrapping arithmetic where upstream overflows `int` (corrector folding and `(1<<k) - 1` for k = 31, `-INT_MIN`, `pred + corrector`, X/Y deltas, GPS time `multi * last_diff` predictions): `utils::wrapAdd` / `wrapMul` and unsigned casts. | Undefined behaviour found by UBSan on GPS time jumps and large deltas; the values are what upstream's binaries compute (they wrap), so streams and decoded points are unchanged. |

| 8 | `detail/field_wavepacket13.{hpp,cpp}` (new), `las.hpp`, `lazperf.{hpp,cpp}` | `Wavepacket13Compressor` / `Wavepacket13Decompressor` (LASzip's WAVEPACKET13 item, version 1: descriptor index with a 256-symbol model; packet offset as "same", "previous offset + previous size", a 32-bit difference through an integer coder, or a raw 64-bit value, chosen by one of four 4-symbol models selected by the previous choice; packet size, return point location and x / y / z (float bits as int32) predicted from the previous packet; the first packet of a chunk raw), `point_compressor_4/5`, `point_decompressor_4/5` and their cases in `build_las_compressor` / `build_las_decompressor`. Byte loads of its own: upstream's `utils::unpack<uint64_t>` sign-extends bytes above 0x7F. | Point formats 4 and 5 in point-wise chunked LAZ. **Written from the LASzip algorithm and verified only by round trips with the fork's own compressor** (no PDRF 4 / 5 LAZ file written by LASzip was available): a real file decoding to garbage would point at this item. |
| 9 | `layered.{hpp,cpp}` | `layered_decompressor` takes formats 9 and 10: format 6 / 8 plus a WAVEPACKET14 layer after the colour (raw first packet, one size word after the colour sizes, its bytes after the colour bytes, LASzip's item order), skipped; records get zeros in the wave-packet bytes. | COPC has no waveforms; skipping a layer is exact (each layer has its own decoder over its own bytes). Verified on chunks made by splicing a layer into laz-perf's format 6 / 8 chunks (no LASzip-written PDRF 9 / 10 file was available). |
| 10 | `readers.cpp` | `basic_file::Private::parseChunkTable`: a chunk-table offset of -1 (LASzip writing to a stream it cannot seek back on) is replaced by the offset stored in the last 8 bytes of the file, as LASzip reads it. | Such files were refused ("Chunk table offset == -1 is not supported"). ddbt's own LAZ reader (`src/copc/readers/las_reader.cpp`) does the same. |

Unchanged behaviour: for every layer the fork decodes, the values are those of upstream. With
all layers, `chunk_decompressor` (upstream path, patched internals) and
`layered_decompressor::decompress(char*)` give records byte-identical to upstream 3.4.0 on every
node of the corpus COPC files (2 618 nodes, 57.2 M points, hashed per node against a build of
the original upstream sources).

Upstream behaviour kept as is: `point_decompressor_6/7/8` hand the scanner channel to the RGB /
NIR / extra-byte fields only on the first point of a chunk and when it changes (0 otherwise).
The compressor does the same on purpose (`Point14Compressor::compress`: "There is a bug in
laszip where the context does not get set unless there is a *change* in context. [...] This
test maintains that behavior."), so files stay LASzip-compatible; the layered path hands the
channel exactly as upstream does.
