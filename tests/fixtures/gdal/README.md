# GDAL samples

Small TIFF files from GDAL 3.13.3's test suite (`autotest/gcore/data`, `autotest/gcore/data/gtiff`;
GDAL's licence: `GDAL-LICENSE.TXT`), for `TiffGdalSamples.*` (tests/unit/tiff/gdal_samples_test.cpp):
complex samples, 24-bit floats, 10 / 24-bit integers, LZMA, CCITT Group 4, old JPEG, ThunderScan,
NeXT, SGILog, YCbCr without JPEG (subsampled), JPEG-XL (both compression codes, strips and tiles,
PIXEL and BAND interleave), GCP-only georeferencing. `ref_*.tif` are the rasters GDAL's tests
compare the JPEG-XL files with.

`expected.tsv` holds what GDAL reads in each (one CRC-32 per band); `expected.py` writes it with
the system GDAL.
