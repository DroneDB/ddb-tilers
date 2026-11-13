#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Smoke test of the frozen baseline (DroneDB 2.4.1) through the libddb C API.

Loads libddb.so from the snapshot, calls DDBRegisterProcess, renders one raster tile and one COPC
tile with DDBMemoryTileFmt, writes them to --out and shuts the library down. Source baseline.env
first (it unsets DDB_* variables so nothing resolves into /mnt/data/repos/DroneDB/build).

Usage: baseline_smoke.py --snapshot DIR --cog COG.tif --copc cloud.copc.laz --out DIR
"""
import argparse
import ctypes
import math
import os
import sys

os.environ.setdefault("GDAL_PAM_ENABLED", "NO")
from osgeo import gdal, osr  # noqa: E402  (system GDAL, probing only)

gdal.UseExceptions()
ORIGIN = 20037508.342789244


def tile_of(x, y, z):
    n = 1 << z
    res = 2 * ORIGIN / n
    return int((x + ORIGIN) // res), int((ORIGIN - y) // res)


def footprint_center_3857(path):
    ds = gdal.Open(path)
    gt = ds.GetGeoTransform()
    srs = osr.SpatialReference(wkt=ds.GetProjection())
    srs.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    dst = osr.SpatialReference()
    dst.ImportFromEPSG(3857)
    dst.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    ct = osr.CoordinateTransformation(srs, dst)
    cx = gt[0] + gt[1] * ds.RasterXSize / 2
    cy = gt[3] + gt[5] * ds.RasterYSize / 2
    x, y, _ = ct.TransformPoint(cx, cy)
    px = abs(gt[1])
    if srs.IsGeographic():
        px = px * 111319.49
    return x, y, px


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", required=True)
    ap.add_argument("--cog", required=True)
    ap.add_argument("--copc", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--zoom", type=int, default=None)
    a = ap.parse_args()

    lib_path = os.path.join(a.snapshot, "usr/lib/libddb.so")
    lib = ctypes.CDLL(lib_path)
    maps = open("/proc/self/maps").read()
    bad = [l.split()[-1] for l in maps.splitlines()
           if "libddb" in l and not l.split()[-1].startswith(a.snapshot)]
    if bad:
        sys.exit("libddb resolved outside the snapshot: %s" % bad[0])

    lib.DDBGetVersion.restype = ctypes.c_char_p
    lib.DDBGetAbiVersion.restype = ctypes.c_uint32
    lib.DDBGetLastError.restype = ctypes.c_char_p
    lib.DDBRegisterProcess.argtypes = [ctypes.c_bool]
    lib.DDBShutdown.argtypes = [ctypes.c_int]
    lib.DDBVSIFree.argtypes = [ctypes.c_void_p]
    lib.DDBMemoryTileFmt.argtypes = [
        ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_int,
        ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_int),
        ctypes.c_int, ctypes.c_bool, ctypes.c_bool, ctypes.c_char_p, ctypes.c_char_p]

    print("libddb %s ABI 0x%08x (%s)" % (lib.DDBGetVersion().decode(), lib.DDBGetAbiVersion(), lib_path))
    err = lib.DDBRegisterProcess(False)
    if err != 0:
        sys.exit("DDBRegisterProcess failed: %d %s" % (err, lib.DDBGetLastError()))

    os.makedirs(a.out, exist_ok=True)

    def tile(path, z, x, y, fmt, size=256, name=None):
        buf = ctypes.c_void_p()
        n = ctypes.c_int()
        # forceRecreate stays false: never rebuild caches in a measured run.
        # Quirk of libddb 2.4.1: the tms flag is inverted. tms=True takes the web (XYZ, y down)
        # row and converts it internally; tms=False expects the row already counted from the south,
        # and a row outside the cloud/raster range gives a blank tile (raster) or "Out of bounds" (COPC).
        e = lib.DDBMemoryTileFmt(path.encode(), z, x, y, ctypes.byref(buf), ctypes.byref(n),
                                 size, True, False, None, fmt.encode())
        if e != 0:
            sys.exit("DDBMemoryTileFmt(%s z%d/%d/%d) failed: %d %s" % (path, z, x, y, e, lib.DDBGetLastError()))
        data = ctypes.string_at(buf, n.value)
        lib.DDBVSIFree(buf)
        out = os.path.join(a.out, name or "tile-%d-%d-%d.%s" % (z, x, y, "jpg" if fmt == "jpeg" else fmt))
        open(out, "wb").write(data)
        print("tile %s z%d/%d/%d %s %d bytes -> %s" % (os.path.basename(path), z, x, y, fmt, len(data), out))
        return data

    cx, cy, px = footprint_center_3857(a.cog)
    z = a.zoom if a.zoom is not None else max(0, int(round(math.log2(2 * ORIGIN / 256 / px))) - 1)
    x, y = tile_of(cx, cy, z)
    d = tile(a.cog, z, x, y, "png", name="cog-tile.png")
    assert d[:8] == b"\x89PNG\r\n\x1a\n"
    d = tile(a.cog, z, x, y, "webp", name="cog-tile.webp")
    assert d[:4] == b"RIFF"

    cx, cy, _ = copc_center(a.copc)
    zc = 18
    x, y = tile_of(cx, cy, zc)
    d = tile(a.copc, zc, x, y, "png", name="copc-tile.png")
    assert d[:8] == b"\x89PNG\r\n\x1a\n"

    print("shutdown:", lib.DDBShutdown(5000))
    print("SMOKE OK")


def copc_center(path):
    """Centre of a LAS/COPC file from its header bounds, in EPSG:3857 (EPSG from the WKT VLR)."""
    import struct
    with open(path, "rb") as f:
        h = f.read(375)
        hsize, = struct.unpack_from("<H", h, 94)
        off, nvlr = struct.unpack_from("<II", h, 96)
        maxx, minx, maxy, miny, maxz, minz = struct.unpack_from("<6d", h, 179)
        f.seek(hsize)
        wkt = None
        for _ in range(nvlr):
            vh = f.read(54)
            uid = vh[2:18].split(b"\0")[0]
            rid, ln = struct.unpack_from("<HH", vh, 18)
            body = f.read(ln)
            if uid == b"LASF_Projection" and rid == 2112:
                wkt = body.split(b"\0")[0].decode()
    srs = osr.SpatialReference()
    srs.ImportFromWkt(wkt)
    srs.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    dst = osr.SpatialReference()
    dst.ImportFromEPSG(3857)
    dst.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    x, y, _ = osr.CoordinateTransformation(srs, dst).TransformPoint((minx + maxx) / 2, (miny + maxy) / 2)
    return x, y, 0


if __name__ == "__main__":
    main()
