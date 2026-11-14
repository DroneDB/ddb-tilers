#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Writes expected.tsv: what GDAL reads in each sample of this folder, for TiffGdalSamples.*.

One line per band: file, band (1-based), width, height, data type, CRC-32 of the band as
band.ReadRaster() returns it (host order, complex: real then imaginary per sample).

Samples this GDAL cannot decode (JPEG-XL without libjxl, 12-bit JPEG without a 12-bit libjpeg)
take the expectation of the raster they are a lossless copy of (REFERENCES, from GDAL's own tests);
the others are skipped and not tested.

    python3 -I tests/fixtures/gdal/expected.py   (system GDAL with Python bindings)
"""
import os
import zlib

os.environ["GDAL_PAM_ENABLED"] = "NO"
from osgeo import gdal  # noqa: E402

gdal.UseExceptions()
HERE = os.path.dirname(os.path.abspath(__file__))

# GDAL's autotest: these JPEG-XL files are lossless copies of byte.tif, rgbsmall.tif, rgba.tif
REFERENCES = {
    "byte_JXL.tif": "ref_byte.tif",
    "byte_JXL_tiled.tif": "ref_byte.tif",
    "byte_jxl_deprecated_50002.tif": "ref_byte.tif",
    "byte_jxl_dng_1_7_52546.tif": "ref_byte.tif",
    "jxl-rgbi.tif": "ref_rgba.tif",
    "rgbsmall_JXL.tif": "ref_rgbsmall.tif",
    "rgbsmall_JXL_separate.tif": "ref_rgbsmall.tif",
    "rgbsmall_JXL_tiled.tif": "ref_rgbsmall.tif",
    "rgbsmall_JXL_tiled_separate.tif": "ref_rgbsmall.tif",
}


def bands(path):
    ds = gdal.Open(path)
    out = []
    for b in range(1, ds.RasterCount + 1):
        band = ds.GetRasterBand(b)
        data = band.ReadRaster()
        out.append((b, ds.RasterXSize, ds.RasterYSize, gdal.GetDataTypeName(band.DataType), zlib.crc32(data)))
    return out


def main():
    lines = []
    for name in sorted(os.listdir(HERE)):
        if not name.endswith(".tif") or name.startswith("ref_"):
            continue
        source = os.path.join(HERE, REFERENCES.get(name, name))
        try:
            rows = bands(source)
        except RuntimeError as e:
            print(f"skip {name}: {str(e).splitlines()[0]}")
            continue
        for b, w, h, t, crc in rows:
            lines.append(f"{name}\t{b}\t{w}\t{h}\t{t}\t{crc:08x}\n")
    with open(os.path.join(HERE, "expected.tsv"), "w") as f:
        f.writelines(lines)
    print(f"wrote {len(lines)} bands")


if __name__ == "__main__":
    main()
