#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""What DroneDB 2.4.1 pays to encode the encoder fixtures (see make_fixtures.py).

Loads the GDAL that the frozen baseline links (statically, inside libpdalcpp.so) and encodes
each <name>.rgba the way DroneDB's TileEncoder does: a MEM dataset of band planes (an opaque
alpha plane dropped; JPEG: no alpha, pixels under alpha 0 written white; WebP: RGB(A)), then
GDALCreateCopy to /vsimem/ with DroneDB's creation options. Prints one TSV row per tile and
format: name, format, bytes, median ms (of --reps, single thread, MEM build + CreateCopy + steal).

Source baseline.env first. Usage: gdal_reference.py --snapshot DIR --fixtures DIR [--reps N]
"""
import argparse
import ctypes
import os
import statistics
import sys
import time

import numpy as np

GDT_Byte = 1
GF_Write = 1
GCI_AlphaBand = 6


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", required=True)
    ap.add_argument("--fixtures", required=True)
    ap.add_argument("--reps", type=int, default=15)
    ap.add_argument("--formats", default="png,jpeg,webp-m0,webp-m1,webp-m2")
    a = ap.parse_args()

    g = ctypes.CDLL(os.path.join(a.snapshot, "usr/lib/libpdalcpp.so.20"))
    vp, cp, ci = ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int
    g.GDALAllRegister.argtypes = []
    g.GDALGetDriverByName.restype = vp
    g.GDALGetDriverByName.argtypes = [cp]
    g.GDALCreate.restype = vp
    g.GDALCreate.argtypes = [vp, cp, ci, ci, ci, ci, vp]
    g.GDALGetRasterBand.restype = vp
    g.GDALGetRasterBand.argtypes = [vp, ci]
    g.GDALRasterIO.argtypes = [vp, ci, ci, ci, ci, ci, vp, ci, ci, ci, ci, ci]
    g.GDALSetRasterColorInterpretation.argtypes = [vp, ci]
    g.GDALCreateCopy.restype = vp
    g.GDALCreateCopy.argtypes = [vp, cp, vp, ci, ctypes.POINTER(cp), vp, vp]
    g.GDALClose.argtypes = [vp]
    g.VSIGetMemFileBuffer.restype = vp
    g.VSIGetMemFileBuffer.argtypes = [cp, ctypes.POINTER(ctypes.c_uint64), ci]
    g.VSIFree.argtypes = [vp]
    g.GDALVersionInfo.restype = cp
    g.GDALVersionInfo.argtypes = [cp]
    g.GDALAllRegister()
    print("# GDAL %s (%s)" % (g.GDALVersionInfo(b"RELEASE_NAME").decode(), a.snapshot), file=sys.stderr)

    mem = g.GDALGetDriverByName(b"MEM")
    fmts = {
        "png": (b"PNG", [b"ZLEVEL=1"]),
        "jpeg": (b"JPEG", [b"QUALITY=75"]),
        "webp-m0": (b"WEBP", [b"QUALITY=80", b"LOSSLESS=FALSE", b"METHOD=0"]),
        "webp-m1": (b"WEBP", [b"QUALITY=80", b"LOSSLESS=FALSE", b"METHOD=1"]),
        "webp-m2": (b"WEBP", [b"QUALITY=80", b"LOSSLESS=FALSE", b"METHOD=2"]),
    }

    def encode(planes, alpha, fmt):
        drv_name, opts = fmts[fmt]
        drv = g.GDALGetDriverByName(drv_name)
        h, w = planes[0].shape
        bands = len(planes) + (1 if alpha is not None else 0)
        ds = g.GDALCreate(mem, b"", w, h, bands, GDT_Byte, None)
        for i, p in enumerate(planes):
            g.GDALRasterIO(g.GDALGetRasterBand(ds, i + 1), GF_Write, 0, 0, w, h,
                           p.ctypes.data, w, h, GDT_Byte, 0, 0)
        if alpha is not None:
            b = g.GDALGetRasterBand(ds, bands)
            g.GDALRasterIO(b, GF_Write, 0, 0, w, h, alpha.ctypes.data, w, h, GDT_Byte, 0, 0)
            g.GDALSetRasterColorInterpretation(b, GCI_AlphaBand)
        arr = (cp * (len(opts) + 1))(*opts, None)
        path = b"/vsimem/ddbt-ref." + fmt.encode()
        out = g.GDALCreateCopy(drv, path, ds, 0, arr, None, None)
        g.GDALClose(out)
        n = ctypes.c_uint64()
        buf = g.VSIGetMemFileBuffer(path, ctypes.byref(n), 1)
        data = ctypes.string_at(buf, n.value)
        g.VSIFree(buf)
        g.GDALClose(ds)
        return data

    names = [l.split("\t")[0] for l in open(os.path.join(a.fixtures, "manifest.tsv")).read().splitlines()[1:]]
    print("name\tformat\tbytes\tms")
    for name in names:
        rows = [l.split("\t") for l in open(os.path.join(a.fixtures, "manifest.tsv")).read().splitlines()]
        size = int(next(r for r in rows if r[0] == name)[1])
        raw = np.fromfile(os.path.join(a.fixtures, name + ".rgba"), dtype=np.uint8).reshape(size, size, 4)
        rgb = [np.ascontiguousarray(raw[:, :, c]) for c in range(3)]
        al = np.ascontiguousarray(raw[:, :, 3])
        keep_alpha = not bool((al == 255).all())
        for fmt in a.formats.split(","):
            if fmt == "jpeg":
                planes = [np.where(al == 0, 255, p).astype(np.uint8) if keep_alpha else p for p in rgb]
                alpha = None
            else:
                planes = rgb
                alpha = al if keep_alpha else None
            times = []
            data = b""
            for _ in range(a.reps):
                t0 = time.perf_counter_ns()
                data = encode(planes, alpha, fmt)
                times.append((time.perf_counter_ns() - t0) / 1e6)
            with open(os.path.join(a.fixtures, "%s.gdal.%s" % (name, fmt)), "wb") as f:
                f.write(data)
            print("%s\t%s\t%d\t%.4f" % (name, fmt, len(data), statistics.median(times)))
            sys.stdout.flush()


if __name__ == "__main__":
    main()
