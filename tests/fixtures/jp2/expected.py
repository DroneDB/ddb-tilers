#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
r"""Writes expected.tsv: what GDAL's JP2OpenJPEG driver reads in each JPEG 2000 file of a folder,
for Jp2GdalSamples.* (tests/unit/jp2/jp2_samples_test.cpp).

    python3 -I tests/fixtures/jp2/expected.py --bands-tool gdal_bands [DIR OUT.tsv]

The system GDAL's Python bindings give the structure, colour tables and georeferencing; the
samples come from gdal_bands (gdal_bands.cpp, built against the GDAL libddb ships: vcpkg's GDAL
3.13.3 and OpenJPEG 2.5.4), since the system's OpenJPEG can decode lossy files differently
(without --bands-tool the bindings read them too). Without DIR: this folder and its expected.tsv;
with DIR OUT.tsv: any folder, for a local run of the test with DDBT_JP2_PARITY_DIR=DIR and
DDBT_JP2_PARITY_TSV=OUT.tsv.

PAM (.aux.xml) is disabled, as libddb disables it. Lines (tab-separated):

    size     FILE WIDTH HEIGHT BANDS TYPE
    band     FILE BAND INTERP NBITS CRC        CRC-32 of band.ReadRaster() (host order); NBITS "-" when none
    window   FILE BAND X Y W H CRC             the same for a window, for rasters too large to read whole
    grid     FILE BAND STEP HEX                a lossy file (the 9/7 wavelet): the band's samples at every
                                               STEP-th column and row, in hex (host order). Its decoding is
                                               floating-point, and OpenJPEG builds round it differently
                                               (1 level in a few pixels): the CRC is exact where the
                                               expectations were made (Linux), the grid within 1 elsewhere
    ct       FILE ENTRIES CRC                  the colour table of band 1 (CRC-32 of its R, G, B bytes)
    gt       FILE GT0 .. GT5                   the geotransform (%.17g), when there is one
    gcp      FILE PIXEL LINE X Y Z             each ground control point (%.17g)
    crs      FILE WKT                          the CRS (of the geotransform or of the GCPs), WKT2, with
                                               backslashes, new lines and tabs escaped (\\, \n, \t)
    rpc      FILE                              the file has RPC metadata
    refused  FILE                              GDAL cannot open or read the file
"""
import argparse
import math
import os
import subprocess
import zlib

os.environ["GDAL_PAM_ENABLED"] = "NO"
from osgeo import gdal  # noqa: E402

gdal.UseExceptions()
HERE = os.path.dirname(os.path.abspath(__file__))
WHOLE_LIMIT = 64 << 20  # bytes of a band read whole; larger rasters are checked on windows
GRID_SAMPLES = 1024  # samples of a band on the grid of a lossy file, at most


def g(v):
    return "%.17g" % v


def tool_crcs(tool, path, window=None):
    """The CRC-32 of every band (or of a window) as gdal_bands reads them, by band number."""
    args = [tool, path] + ([str(v) for v in window] if window else [])
    out = subprocess.run(args, check=True, capture_output=True, text=True).stdout.split()
    if out == ["fail"]:
        raise RuntimeError("gdal_bands cannot open " + path)
    crcs = dict(zip(out[0::2], out[1::2]))
    if "fail" in crcs.values():
        raise RuntimeError("gdal_bands cannot read " + path)
    return {int(k): v for k, v in crcs.items()}


def describe(path, name, tool):
    ds = gdal.OpenEx(path, gdal.OF_RASTER, allowed_drivers=["JP2OpenJPEG"])
    lines = []
    b1 = ds.GetRasterBand(1)
    dt = gdal.GetDataTypeName(b1.DataType)
    lines.append(["size", name, ds.RasterXSize, ds.RasterYSize, ds.RasterCount, dt])
    size = ds.RasterXSize * ds.RasterYSize * gdal.GetDataTypeSize(b1.DataType) // 8
    W, H = ds.RasterXSize, ds.RasterYSize
    windows = [] if size <= WHOLE_LIMIT else [(0, 0, 1500, 1100), (W - 1300, H - 700, 1300, 700),
                                              (W // 2 - 600, H // 2 - 600, 1200, 1200)]

    def crcs(window=None):
        if tool:
            return tool_crcs(tool, path, window)
        read = (lambda band: band.ReadRaster(*window)) if window else (lambda band: band.ReadRaster())
        return {b: "%08x" % zlib.crc32(read(ds.GetRasterBand(b))) for b in range(1, ds.RasterCount + 1)}

    whole = crcs() if not windows else {}
    per_window = [(w, crcs(w)) for w in windows]
    for b in range(1, ds.RasterCount + 1):
        band = ds.GetRasterBand(b)
        interp = gdal.GetColorInterpretationName(band.GetColorInterpretation())
        nbits = band.GetMetadataItem("NBITS", "IMAGE_STRUCTURE") or "-"
        lines.append(["band", name, b, interp, nbits, whole.get(b, "-")])
        for (x, y, w, h), c in per_window:
            lines.append(["window", name, b, x, y, w, h, c[b]])
        reversibility = ds.GetMetadataItem("COMPRESSION_REVERSIBILITY", "IMAGE_STRUCTURE") or ""
        if reversibility.startswith("LOSSY") and not windows:
            step = max(1, math.ceil(math.sqrt(W * H / GRID_SAMPLES)))
            grid = band.ReadAsArray()[::step, ::step]
            lines.append(["grid", name, b, step, grid.tobytes().hex()])
    ct = b1.GetColorTable()
    if ct is not None:
        rgb = bytearray()
        for i in range(ct.GetCount()):
            e = ct.GetColorEntry(i)
            rgb += bytes(e[:3])
        lines.append(["ct", name, ct.GetCount(), "%08x" % zlib.crc32(bytes(rgb))])
    gt = ds.GetGeoTransform(can_return_null=True)
    if gt is not None:
        lines.append(["gt", name] + [g(v) for v in gt])
    for p in ds.GetGCPs() or []:
        lines.append(["gcp", name, g(p.GCPPixel), g(p.GCPLine), g(p.GCPX), g(p.GCPY), g(p.GCPZ)])
    srs = ds.GetSpatialRef() or ds.GetGCPSpatialRef()
    if srs is not None:
        wkt = srs.ExportToWkt(["FORMAT=WKT2_2019", "MULTILINE=NO"])
        lines.append(["crs", name, wkt.replace("\\", "\\\\").replace("\n", "\\n").replace("\t", "\\t")])
    if ds.GetMetadata("RPC"):
        lines.append(["rpc", name])
    return lines


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bands-tool", help="gdal_bands built against the GDAL libddb ships")
    parser.add_argument("folder", nargs="?", default=HERE)
    parser.add_argument("out", nargs="?", default=os.path.join(HERE, "expected.tsv"))
    args = parser.parse_args()
    lines = []
    for name in sorted(os.listdir(args.folder)):
        if not name.lower().endswith((".jp2", ".j2k", ".jpc", ".jpx")):
            continue
        try:
            lines += describe(os.path.join(args.folder, name), name, args.bands_tool)
        except (RuntimeError, subprocess.CalledProcessError) as e:
            print(f"refused {name}: {str(e).splitlines()[0]}")
            lines.append(["refused", name])
    with open(args.out, "w") as f:
        for line in lines:
            f.write("\t".join(str(v) for v in line) + "\n")
    print(f"wrote {len(lines)} lines to {args.out}")


if __name__ == "__main__":
    main()
