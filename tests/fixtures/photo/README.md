# Photos

For `PhotoGdalSamples.*` (tests/unit/photo/photo_samples_test.cpp): every file here must read as
GDAL's drivers read it (JPEG, PNG, GIF, WEBP, GTiff), and for the geoprojection tests.

- Written by `gen.py` with Pillow (and the system's `gdal_translate` for the TIFFs), from a seeded
  random image of 65 x 41 pixels: JPEG with 4:2:0, 4:2:2 and 4:4:4 sampling, progressive,
  optimized Huffman tables, grey, CMYK (Adobe), an EXIF orientation of 6 (not applied by GDAL),
  and a truncated file (GDAL's "Premature end of JPEG file" error); PNG RGB, RGBA, grey, grey +
  alpha, 16-bit grey, grey and RGB with a tRNS colour (nodata), a palette with one transparent
  entry (nodata) and with several (no nodata), a 4-bit palette, a 1-bit image; GIF plain,
  interlaced and with a transparent index; WebP lossy, lossless, and both with alpha; TIFF photos
  without georeferencing (JPEG YCbCr, DEFLATE); `geo_photo.jpg` and `geo_photo_trns.png` (192 x 144,
  soft shapes, a road and a roof; the PNG's road colour is its tRNS colour), the photos of the
  geoprojection tests.
- `dng_fc220_preview.dng`: the first directory of raw.pixls.us' "DJI - FC220 - 16bit (4:3).DNG"
  (DJI Mavic Pro, Creative Commons 0, public domain: https://raw.pixls.us/), cut by `gen.py --dng`:
  its 160 x 120 YCbCr JPEG preview, which is what GDAL's GTiff driver reads from a DNG, with the
  DNG tags of that directory; the raw image (SubIFDs), the EXIF and GPS directories and the
  private data are left out.
- `gdal_*`: samples of GDAL 3.13.3's test suite (`autotest/gdrivers/data`; GDAL's licence:
  `GDAL-LICENSE.TXT`): a 12-bit JPEG, the validity mask GDAL appends to its JPEGs, lossless JPEG
  (8-bit; 16-bit, which GDAL refuses), CMYK, corrupted JPEGs (one read with warnings, one GDAL
  refuses), 16-bit RGBA and RGB with a tRNS colour, an interlaced 16-bit PNG, a broken IDAT GDAL
  refuses, a GIF and a WebP.
- `refused/`: files GDAL reads with more than one nodata value or image, which the readers
  refuse (`PhotoSource.*`, tests/unit/photo/photo_source_test.cpp): a GIF of two images (two
  bands in GDAL) and an RGB PNG whose tRNS colour has three different values (one nodata value
  per band in GDAL).

`expected.tsv` holds what GDAL reads in each (size, band types, CRC-32 of every band, colour
interpretation, NBITS, nodata, the per-dataset mask, the colour table, or the refusal), written by
`tests/fixtures/tools/gdal_expected.py`:

    python3 -I tests/fixtures/tools/gdal_expected.py --tool gdal_expected --ext .jpg --ext .png \
        --ext .gif --ext .webp --ext .tif --ext .dng tests/fixtures/photo tests/fixtures/photo/expected.tsv

with `gdal_expected` (`tests/fixtures/tools/gdal_expected.cpp`) built against the GDAL libddb ships
(vcpkg's GDAL 3.13.3, libjpeg-turbo, libpng, giflib, libwebp, libtiff). The same test runs on any
folder with `DDBT_PHOTO_PARITY_DIR` and `DDBT_PHOTO_PARITY_TSV` (gdal_expected.py's output for that
folder).

## Geoprojection

For `PhotoGeoproject*` (tests/unit/photo/photo_geoproject_test.cpp), `geoproject.py` writes
`geoproject.tsv` with GDAL 3.13.3 of vcpkg (`tests/fixtures/tools/gdal_geoproject.cpp`: libddb's
gdal_translate with the four GCPs of the footprint, then gdalwarp): for each case the grid, the
GCPs, the CRC-32 of every band of GDAL's output written losslessly, and, in the GCPs' CRS, the
chunks of GDAL's warper for libddb's JPEG GeoTIFF; and `gdal/geo_photo_2steps.tif`, libddb's COG
of `geo_photo.jpg` (its JPEG GeoTIFF in EPSG:4326, then the COG of that file with libddb's
arguments, `tests/unit/cogbuild/data/gdal_cog_warp.cpp`). The geoprojection of a palette photo
fails in GDAL (libtiff writes no JPEG of a palette image), so the cases use RGB photos.
