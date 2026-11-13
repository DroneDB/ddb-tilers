#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Real map tiles for the encoder microbenchmarks (bench/micro/encode/encode_bench.cpp).

Renders raster tiles of a COG and point cloud tiles of a COPC through the frozen baseline
(libddb 2.4.1, DDBMemoryTileFmt, PNG), decodes them with PIL and writes, per tile,
  <name>.rgba        raw RGBA, width * height * 4 bytes (row-major, no padding)
  <name>.ddb.png     the PNG bytes libddb returned (GDAL PNG driver, ZLEVEL=1)
plus manifest.tsv (name, width, height, source, z, x, y, opaque, transparent, ddb_png_bytes).

Source baseline.env first. The fixtures are not part of the repository.

Usage: make_fixtures.py --snapshot DIR --cog COG.tif --copc cloud.copc.laz --out DIR [--home DIR]
"""
import argparse
import ctypes
import io
import os
import struct
import sys

os.environ.setdefault("GDAL_PAM_ENABLED", "NO")
from osgeo import gdal, osr  # noqa: E402  (system GDAL, probing only)
from PIL import Image  # noqa: E402

gdal.UseExceptions()
ORIGIN = 20037508.342789244


def to_3857(srs_wkt, x, y):
    srs = osr.SpatialReference(wkt=srs_wkt)
    srs.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    dst = osr.SpatialReference()
    dst.ImportFromEPSG(3857)
    dst.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    px, py, _ = osr.CoordinateTransformation(srs, dst).TransformPoint(x, y)
    return px, py


def cog_bounds(path):
    ds = gdal.Open(path)
    gt = ds.GetGeoTransform()
    wkt = ds.GetProjection()
    xs, ys = [], []
    for cx, cy in ((0, 0), (ds.RasterXSize, 0), (0, ds.RasterYSize), (ds.RasterXSize, ds.RasterYSize)):
        x, y = to_3857(wkt, gt[0] + gt[1] * cx + gt[2] * cy, gt[3] + gt[4] * cx + gt[5] * cy)
        xs.append(x)
        ys.append(y)
    return min(xs), min(ys), max(xs), max(ys)


def copc_bounds(path):
    with open(path, "rb") as f:
        h = f.read(375)
        hsize, = struct.unpack_from("<H", h, 94)
        _, nvlr = struct.unpack_from("<II", h, 96)
        maxx, minx, maxy, miny, _, _ = struct.unpack_from("<6d", h, 179)
        f.seek(hsize)
        wkt = None
        for _ in range(nvlr):
            vh = f.read(54)
            uid = vh[2:18].split(b"\0")[0]
            rid, ln = struct.unpack_from("<HH", vh, 18)
            body = f.read(ln)
            if uid == b"LASF_Projection" and rid == 2112:
                wkt = body.split(b"\0")[0].decode()
    x0, y0 = to_3857(wkt, minx, miny)
    x1, y1 = to_3857(wkt, maxx, maxy)
    return min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)


def tiles_in(bounds, z):
    n = 1 << z
    res = 2 * ORIGIN / n
    x0 = int((bounds[0] + ORIGIN) // res)
    x1 = int((bounds[2] + ORIGIN) // res)
    y0 = int((ORIGIN - bounds[3]) // res)
    y1 = int((ORIGIN - bounds[1]) // res)
    return [(x, y) for y in range(y0, y1 + 1) for x in range(x0, x1 + 1)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", required=True)
    ap.add_argument("--cog", required=True)
    ap.add_argument("--copc", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--home", default=None, help="DDB_HOME for the tile caches (default: <out>/ddb-home)")
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    os.environ["DDB_HOME"] = a.home or os.path.join(a.out, "ddb-home")
    os.makedirs(os.environ["DDB_HOME"], exist_ok=True)

    lib = ctypes.CDLL(os.path.join(a.snapshot, "usr/lib/libddb.so"))
    lib.DDBGetLastError.restype = ctypes.c_char_p
    lib.DDBRegisterProcess.argtypes = [ctypes.c_bool]
    lib.DDBShutdown.argtypes = [ctypes.c_int]
    lib.DDBVSIFree.argtypes = [ctypes.c_void_p]
    lib.DDBMemoryTileFmt.argtypes = [
        ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_int,
        ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_int),
        ctypes.c_int, ctypes.c_bool, ctypes.c_bool, ctypes.c_char_p, ctypes.c_char_p]
    if lib.DDBRegisterProcess(False) != 0:
        sys.exit("DDBRegisterProcess failed: %s" % lib.DDBGetLastError())

    def render(path, z, x, y, size):
        buf = ctypes.c_void_p()
        n = ctypes.c_int()
        # libddb 2.4.1: tms=True takes the XYZ (web) row
        e = lib.DDBMemoryTileFmt(path.encode(), z, x, y, ctypes.byref(buf), ctypes.byref(n),
                                 size, True, False, None, b"png")
        if e != 0:
            return None
        data = ctypes.string_at(buf, n.value)
        lib.DDBVSIFree(buf)
        return data

    jobs = []
    cb = cog_bounds(a.cog)
    for z in (19, 20, 21):
        jobs += [("cog", a.cog, z, x, y, 256) for x, y in tiles_in(cb, z)]
    # z22 interior tiles (full resolution, opaque)
    jobs += [("cog", a.cog, 22, x, y, 256) for x, y in tiles_in(cb, 22)[18:26]]
    for z in (19, 20):
        jobs += [("cog", a.cog, z, x, y, 512) for x, y in tiles_in(cb, z)]
    pb = copc_bounds(a.copc)
    for z in (17, 18, 19):
        jobs += [("copc", a.copc, z, x, y, 256) for x, y in tiles_in(pb, z)][:8]
    for z in (17, 18):
        jobs += [("copc", a.copc, z, x, y, 512) for x, y in tiles_in(pb, z)][:4]

    rows = []
    for kind, path, z, x, y, size in jobs:
        data = render(path, z, x, y, size)
        if data is None:
            print("skip %s z%d/%d/%d@%d: %s" % (kind, z, x, y, size, lib.DDBGetLastError()))
            continue
        img = Image.open(io.BytesIO(data))
        img.load()
        rgba = img.convert("RGBA")
        raw = rgba.tobytes()
        alpha = raw[3::4]
        opaque = alpha.count(255) == len(alpha)
        transparent = alpha.count(0) == len(alpha)
        if transparent:
            print("skip blank %s z%d/%d/%d@%d" % (kind, z, x, y, size))
            continue
        name = "%s-%d-%d-%d-%d" % (kind, size, z, x, y)
        with open(os.path.join(a.out, name + ".rgba"), "wb") as f:
            f.write(raw)
        with open(os.path.join(a.out, name + ".ddb.png"), "wb") as f:
            f.write(data)
        rows.append((name, size, size, kind, z, x, y, int(opaque), int(transparent), len(data), img.mode))
        print("%s %s %d bytes opaque=%d" % (name, img.mode, len(data), opaque))

    with open(os.path.join(a.out, "manifest.tsv"), "w") as f:
        f.write("name\twidth\theight\tsource\tz\tx\ty\topaque\ttransparent\tddb_png_bytes\tddb_png_mode\n")
        for r in rows:
            f.write("\t".join(str(v) for v in r) + "\n")
    print("%d tiles -> %s" % (len(rows), a.out))
    lib.DDBShutdown(5000)


if __name__ == "__main__":
    main()
