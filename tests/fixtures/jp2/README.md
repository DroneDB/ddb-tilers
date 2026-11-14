# JPEG 2000 samples

For `Jp2GdalSamples.*` (tests/unit/jp2/jp2_samples_test.cpp): every file here must read as GDAL's
JP2OpenJPEG driver reads it.

- Small files from GDAL 3.13.3's test suite (`autotest/gdrivers/data/jpeg2000`; GDAL's licence:
  `GDAL-LICENSE.TXT`), with their `.j2w` world files: GeoJP2 and GMLJP2 georeferencing (axis order,
  explicit axis names, the alternative offset vector order, NUL characters in the GML, both boxes
  disagreeing), GCPs, PixelIsPoint, world files, a raw codestream, an image origin that is not 0,
  TLM / PLT markers, a tile larger than the image, tiles of 16 pixels, 13-bit samples with a 1-bit
  alpha, a 1-bit alpha promoted to 8 bits, the resolution box, 20-bit and 1 / 23-bit samples, a
  single 32769 x 16385 tile (checked on windows), and two files GDAL refuses (no codestream, a tile
  it cannot decode).
- `gen_*.jp2`, written by `gen.py` with GDAL's JP2OpenJPEG driver: a colour table, 4:2:0 YCbCr,
  lossy RGB tiles with the YCC transform, RGBA and 16-bit RGBA with an alpha channel definition,
  GCPs and RPC metadata in the GeoJP2 box, a GMLJP2 box only, a rotated geotransform, 24-bit signed
  samples, the GDAL metadata box, and a single lossy tile larger than 1024 x 1024.

- `gtiff/`: GDAL's GeoTIFF copies (gdal.Translate) of 12 of them, for `Jp2Cog.IsTheCogOfItsGeoTiffCopy`
  (tests/unit/jp2/jp2_cog_test.cpp: the COG of the JPEG 2000 file is byte for byte the COG of its
  copy), and `uint16_half_alpha.tif` (16-bit RGBA, alpha 32768) for the alpha scaling of the warp.

`expected.tsv` holds what GDAL reads in each (size, band types, CRC-32 of every band, colour
interpretation, NBITS, colour table, geotransform, GCPs, CRS, RPC presence, or the refusal);
`expected.py` writes it, with the samples decoded by `gdal_bands.cpp` built against the GDAL libddb
ships (vcpkg's GDAL 3.13.3 and OpenJPEG 2.5.4): the system's OpenJPEG decodes the green band of the
lossy single tile differently. The same test runs on any folder with `DDBT_JP2_PARITY_DIR` and
`DDBT_JP2_PARITY_TSV` (expected.py's output for that folder).
