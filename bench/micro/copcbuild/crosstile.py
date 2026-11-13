#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Cross-tiler check of a COPC: the frozen libddb 2.4.1 (PointCloudTiler) tiles the COPC built by
ddbt and the COPC its own Untwine build made for the same input; decoded PNG tiles are compared.

    . baseline.env
    crosstile.py --snapshot "$BASELINE_SNAPSHOT" --ours OURS.copc.laz --base BASE.copc.laz \
        --meta OURS.copc.laz.meta.json --zooms 15-22 --work DIR [--per-zoom 32]

The two builds thin the levels differently (Untwine samples at random, ddbt keeps the point
closest to each cell centre), so coarse zooms differ point by point; at the deepest zooms every
point is drawn and the tiles should agree. Reported per zoom: tiles rendered by both, failures
on one side, alpha IoU (covered pixels) and PSNR over pixels covered on both sides.
Tiles: every zoom of --zooms, at most --per-zoom tiles per zoom spread over the footprint
(EPSG:4326 polygon of the sidecar), 256 px, tms=True (XYZ rows, libddb's inverted flag).
"""
import argparse
import ctypes
import io
import json
import math
import os
import sys

import numpy as np
from PIL import Image

R = 6378137 * math.pi


def merc(lon, lat):
    x = lon * R / 180
    y = math.log(math.tan((90 + lat) * math.pi / 360)) * R / math.pi
    return x, y


def tiles_for(poly, zooms, per_zoom):
    ring = poly["coordinates"][0]
    xs, ys = zip(*[merc(p[0], p[1]) for p in ring])
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    jobs = []
    for z in zooms:
        n = 2 ** z
        span = 2 * R / n
        x0, x1 = int((minx + R) // span), int((maxx + R - 1e-6) // span)
        y0, y1 = int((R - maxy) // span), int((R - miny - 1e-6) // span)
        allt = [(x, y) for y in range(y0, y1 + 1) for x in range(x0, x1 + 1)]
        step = max(1, len(allt) // per_zoom)
        jobs += [(z, x, y) for (x, y) in allt[::step][:per_zoom]]
    return jobs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", required=True)
    ap.add_argument("--ours", required=True)
    ap.add_argument("--base", required=True)
    ap.add_argument("--meta", required=True, help="ddbt's sidecar of --ours (footprint)")
    ap.add_argument("--zooms", default="15-22")
    ap.add_argument("--work", required=True)
    ap.add_argument("--per-zoom", type=int, default=32)
    ap.add_argument("--json")
    a = ap.parse_args()
    os.makedirs(a.work, exist_ok=True)
    os.environ["DDB_HOME"] = os.path.join(a.work, "ddb-home")
    os.makedirs(os.environ["DDB_HOME"], exist_ok=True)
    z0, z1 = (int(v) for v in a.zooms.split("-"))
    meta = json.load(open(a.meta))
    if "polyBounds" not in meta:
        sys.exit("the sidecar has no footprint: " + meta.get("footprintWarning", ""))

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
        e = lib.DDBMemoryTileFmt(os.path.abspath(path).encode(), z, x, y, ctypes.byref(buf), ctypes.byref(n), 256,
                                 True, False, None, b"png")
        if e != 0:
            return None, lib.DDBGetLastError().decode()
        data = ctypes.string_at(buf, n.value)
        lib.DDBVSIFree(buf)
        return np.asarray(Image.open(io.BytesIO(data)).convert("RGBA")).astype(np.float64), None

    rows = []
    for z, x, y in tiles_for(meta["polyBounds"], range(z0, z1 + 1), a.per_zoom):
        ta, ea = render(a.ours, z, x, y)
        tb, eb = render(a.base, z, x, y)
        if ta is None or tb is None:
            rows.append({"z": z, "x": x, "y": y, "errorOurs": ea, "errorBase": eb})
            continue
        va, vb = ta[..., 3] > 0, tb[..., 3] > 0
        union = np.count_nonzero(va | vb)
        iou = 1.0 if union == 0 else np.count_nonzero(va & vb) / union
        both = va & vb
        if np.count_nonzero(both):
            d = ta[..., :3][both] - tb[..., :3][both]
            mse = float(np.mean(d * d))
            psnr = 99.0 if mse == 0 else min(99.0, 10 * math.log10(255 * 255 / mse))
        else:
            psnr = 99.0
        rows.append({"z": z, "x": x, "y": y, "psnr": psnr, "iou": iou, "pixelsOurs": int(np.count_nonzero(va)),
                     "pixelsBase": int(np.count_nonzero(vb))})
    by_zoom = {}
    for r in rows:
        by_zoom.setdefault(r["z"], []).append(r)
    summary = {"tiles": len(rows), "byZoom": {}}
    for z, v in sorted(by_zoom.items()):
        ok = [r for r in v if "psnr" in r]
        summary["byZoom"][z] = {
            "n": len(v),
            "failedOurs": sum(1 for r in v if r.get("errorOurs")),
            "failedBase": sum(1 for r in v if r.get("errorBase")),
            "iouMean": float(np.mean([r["iou"] for r in ok])) if ok else None,
            "iouMin": min((r["iou"] for r in ok), default=None),
            "psnrMean": float(np.mean([r["psnr"] for r in ok])) if ok else None,
            "pixelsOurs/base": (sum(r["pixelsOurs"] for r in ok) / max(1, sum(r["pixelsBase"] for r in ok))) if ok else None,
        }
    print(json.dumps(summary, indent=1))
    if a.json:
        with open(a.json, "w") as f:
            json.dump({"summary": summary, "tiles": rows}, f, indent=1)
    failed_ours = sum(1 for r in rows if r.get("errorOurs") and not r.get("errorBase"))
    return 1 if failed_ours else 0


if __name__ == "__main__":
    sys.exit(main())
