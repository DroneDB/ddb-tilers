#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Cross-tiler check: the frozen libddb 2.4.1 tiles a COG built by ddbt and the COG it built itself
for the same input; the decoded PNG tiles are compared (PSNR over RGB, alpha agreement).

    . baseline.env
    crosstile.py --snapshot "$BASELINE_SNAPSHOT" --ours OURS.tif --base BASE.tif --work DIR [--per-zoom 64]

Both files are hard-linked (or copied) to DIR/{ours,base}/.ddb/build/<64 hex>/cog/cog.tif, where
libddb treats them as its own build outputs (it tiles them in place, no conversion). Tiles: every
zoom from the raster's minimum to its maximum, at most --per-zoom tiles per zoom (evenly spread
over the raster's tiles), 256 px, tms=True (XYZ rows, libddb's inverted flag).
"""
import argparse
import ctypes
import io
import json
import math
import os
import shutil
import sys

import numpy as np
from PIL import Image

os.environ.setdefault("GDAL_PAM_ENABLED", "NO")
from osgeo import gdal  # noqa: E402

R = 6378137 * math.pi
KEY = "0123456789abcdef" * 4


def place(src, root):
    d = os.path.join(root, ".ddb", "build", KEY, "cog")
    os.makedirs(d, exist_ok=True)
    dst = os.path.join(d, "cog.tif")
    if os.path.exists(dst):
        os.unlink(dst)
    try:
        os.link(src, dst)
    except OSError:
        shutil.copyfile(src, dst)
    return dst


def tiles_for(path, per_zoom):
    ds = gdal.Open(path)
    gt = ds.GetGeoTransform()
    w, h = ds.RasterXSize, ds.RasterYSize
    minx, maxy = gt[0], gt[3]
    maxx, miny = gt[0] + w * gt[1], gt[3] + h * gt[5]
    zmax = int(math.floor(math.log2(2 * R / 256 / gt[1])))
    zmin = max(0, zmax - int(math.ceil(math.log2(max(w, h) / 256))))
    jobs = []
    for z in range(zmin, zmax + 1):
        n = 2 ** z
        span = 2 * R / n
        x0, x1 = int((minx + R) // span), int((maxx + R - 1e-6) // span)
        y0, y1 = int((R - maxy) // span), int((R - miny - 1e-6) // span)
        all_tiles = [(x, y) for y in range(y0, y1 + 1) for x in range(x0, x1 + 1)]
        step = max(1, len(all_tiles) // per_zoom)
        jobs += [(z, x, y) for (x, y) in all_tiles[::step][:per_zoom]]
    return jobs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", required=True)
    ap.add_argument("--ours", required=True)
    ap.add_argument("--base", required=True)
    ap.add_argument("--work", required=True)
    ap.add_argument("--per-zoom", type=int, default=64)
    ap.add_argument("--json")
    a = ap.parse_args()
    os.makedirs(a.work, exist_ok=True)
    os.environ["DDB_HOME"] = os.path.join(a.work, "ddb-home")
    os.makedirs(os.environ["DDB_HOME"], exist_ok=True)
    ours = place(os.path.abspath(a.ours), os.path.join(a.work, "ours"))
    base = place(os.path.abspath(a.base), os.path.join(a.work, "base"))
    # Statistics sidecars next to them (libddb reads them for non-8-bit stretches).
    for s, d in ((a.ours, ours), (a.base, base)):
        if os.path.exists(s + ".stats.json"):
            shutil.copyfile(s + ".stats.json", d + ".stats.json")

    lib = ctypes.CDLL(os.path.join(a.snapshot, "usr/lib/libddb.so"))
    lib.DDBGetLastError.restype = ctypes.c_char_p
    lib.DDBRegisterProcess.argtypes = [ctypes.c_bool]
    lib.DDBVSIFree.argtypes = [ctypes.c_void_p]
    lib.DDBMemoryTileFmt.argtypes = [
        ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_int,
        ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_int),
        ctypes.c_int, ctypes.c_bool, ctypes.c_bool, ctypes.c_char_p, ctypes.c_char_p]
    if lib.DDBRegisterProcess(False) != 0:
        sys.exit("DDBRegisterProcess failed: %s" % lib.DDBGetLastError())

    def render(path, z, x, y):
        buf = ctypes.c_void_p()
        n = ctypes.c_int()
        e = lib.DDBMemoryTileFmt(path.encode(), z, x, y, ctypes.byref(buf), ctypes.byref(n), 256, True, False, None,
                                 b"png")
        if e != 0:
            return None, lib.DDBGetLastError().decode()
        data = ctypes.string_at(buf, n.value)
        lib.DDBVSIFree(buf)
        return np.asarray(Image.open(io.BytesIO(data)).convert("RGBA")).astype(np.float64), None

    jobs = tiles_for(base, a.per_zoom)
    rows = []
    for z, x, y in jobs:
        ta, ea = render(ours, z, x, y)
        tb, eb = render(base, z, x, y)
        if ta is None or tb is None:
            rows.append({"z": z, "x": x, "y": y, "error": ea or eb, "bothFailed": ta is None and tb is None})
            continue
        va, vb = ta[..., 3] > 0, tb[..., 3] > 0
        union = np.count_nonzero(va | vb)
        iou = 1.0 if union == 0 else np.count_nonzero(va & vb) / union
        both = va & vb
        if np.count_nonzero(both):
            d = ta[..., :3][both] - tb[..., :3][both]
            mse = float(np.mean(d * d))
            psnr = float("inf") if mse == 0 else 10 * math.log10(255 * 255 / mse)
        else:
            psnr = float("inf")
        rows.append({"z": z, "x": x, "y": y, "psnr": psnr, "iou": iou})
    ok = [r for r in rows if "psnr" in r]
    failed = [r for r in rows if "error" in r and not r.get("bothFailed")]
    by_zoom = {}
    for r in ok:
        by_zoom.setdefault(r["z"], []).append(r)
    summary = {
        "tiles": len(rows),
        "compared": len(ok),
        "oneSideFailed": len(failed),
        "psnrMin": min((r["psnr"] for r in ok), default=None),
        "psnrMean": float(np.mean([min(r["psnr"], 99) for r in ok])) if ok else None,
        "iouMin": min((r["iou"] for r in ok), default=None),
        "pass40": sum(1 for r in ok if r["psnr"] >= 40 and r["iou"] >= 0.999),
        "byZoom": {z: {"n": len(v), "psnrMin": min(r["psnr"] for r in v), "iouMin": min(r["iou"] for r in v)}
                   for z, v in sorted(by_zoom.items())},
    }
    print(json.dumps(summary, indent=1))
    if a.json:
        with open(a.json, "w") as f:
            json.dump({"summary": summary, "tiles": rows}, f, indent=1)


if __name__ == "__main__":
    main()
