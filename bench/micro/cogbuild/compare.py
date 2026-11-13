#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Compares a COG built by ddbt with the baseline (libddb 2.4.1) COG of the same input.

    compare.py OURS.tif BASELINE.tif [--lib libddbt.so] [--json out.json]

Checks (system GDAL Python, GDAL_PAM_ENABLED=NO, read-only):
  - GDAL's validate_cloud_optimized_geotiff --full-check=yes on ours;
  - the gdalinfo -json contract: size, geotransform (1e-9 relative), CRS, per band type, colour
    interpretation, nodata, block size, overview sizes, mask flags and mask overview sizes,
    description, metadata; dataset metadata; IMAGE_STRUCTURE COMPRESSION / INTERLEAVE / LAYOUT;
  - per level and band: PSNR (8-bit) or max |diff| (lossless data), over the pixels both masks
    mark valid; mask agreement (pixels whose validity differs);
  - the statistics sidecars (relative delta of min / max / mean / std, valid counts);
  - with --lib: our own TiffReader + COG tiler read the file (ddbtMemoryTileFmt of a few tiles).
"""
import argparse
import ctypes
import json
import math
import os
import subprocess
import sys

os.environ["GDAL_PAM_ENABLED"] = "NO"
from osgeo import gdal  # noqa: E402
import numpy as np  # noqa: E402

gdal.UseExceptions()
VALIDATOR = "/usr/lib/python3/dist-packages/osgeo_utils/samples/validate_cloud_optimized_geotiff.py"


def validate(path):
    p = subprocess.run([sys.executable, VALIDATOR, "--full-check=yes", path], capture_output=True, text=True,
                       env=dict(os.environ, GDAL_PAM_ENABLED="NO"))
    out = (p.stdout + p.stderr).strip()
    return p.returncode == 0 and "is a valid cloud optimized GeoTIFF" in out, out


def contract(path):
    info = gdal.Info(path, format="json")
    c = {
        "size": info["size"],
        "geoTransform": info.get("geoTransform"),
        "crs": info.get("coordinateSystem", {}).get("wkt"),
        "metadata": {k: v for k, v in info.get("metadata", {}).items() if k not in ("DERIVED_SUBDATASETS",)},
        "bands": [],
    }
    ims = c["metadata"].get("IMAGE_STRUCTURE", {})
    c["imageStructure"] = {k: ims.get(k) for k in ("COMPRESSION", "INTERLEAVE", "LAYOUT", "OVERVIEW_RESAMPLING")}
    c["metadata"].pop("IMAGE_STRUCTURE", None)
    for b in info["bands"]:
        c["bands"].append({
            "type": b["type"],
            "colorInterpretation": b.get("colorInterpretation"),
            "noDataValue": b.get("noDataValue"),
            "block": b.get("block"),
            "overviews": [o["size"] for o in b.get("overviews", [])],
            "mask": b.get("mask", {}).get("flags"),
            "maskOverviews": [o["size"] for o in b.get("mask", {}).get("overviews", [])],
            "description": b.get("description", ""),
            "metadata": b.get("metadata", {}),
        })
    return c


def contract_diff(a, b):
    diffs = []
    for k in ("size", "crs", "metadata", "imageStructure"):
        if a[k] != b[k]:
            diffs.append(f"{k}: ours {a[k]!r} baseline {b[k]!r}")
    ga, gb = a["geoTransform"], b["geoTransform"]
    if (ga is None) != (gb is None) or (ga and any(abs(x - y) > 1e-9 * max(1.0, abs(y)) for x, y in zip(ga, gb))):
        diffs.append(f"geoTransform: ours {ga} baseline {gb}")
    if len(a["bands"]) != len(b["bands"]):
        diffs.append(f"band count: ours {len(a['bands'])} baseline {len(b['bands'])}")
    for i, (x, y) in enumerate(zip(a["bands"], b["bands"])):
        for k in x:
            if x[k] != y[k]:
                diffs.append(f"band {i + 1} {k}: ours {x[k]!r} baseline {y[k]!r}")
    return diffs


def level_datasets(ds):
    yield 0, ds
    b1 = ds.GetRasterBand(1)
    for i in range(b1.GetOverviewCount()):
        yield i + 1, None


def read_level(ds, level, windows=None):
    """Bands and band 1's mask of a level, whole or as the concatenation of windows."""
    n = ds.RasterCount
    bands = []
    masks = []
    for k in range(1, n + 1):
        b = ds.GetRasterBand(k)
        if level > 0:
            b = b.GetOverview(level - 1)
        if windows is None:
            bands.append(b.ReadAsArray())
        else:
            bands.append(np.concatenate([b.ReadAsArray(*w).ravel() for w in windows])[None, :])
        if k == 1:
            m = b.GetMaskBand()
            if windows is None:
                masks.append(m.ReadAsArray())
            else:
                masks.append(np.concatenate([m.ReadAsArray(*w).ravel() for w in windows])[None, :])
    return bands, masks[0]


def sample_windows(ds, level, count, seed=7):
    b = ds.GetRasterBand(1)
    if level > 0:
        b = b.GetOverview(level - 1)
    w, h = b.XSize, b.YSize
    rng = np.random.default_rng(seed + level)
    size = 512
    if w * h <= count * size * size:
        return None
    wins = []
    for _ in range(count):
        x = int(rng.integers(0, max(1, w - size)))
        y = int(rng.integers(0, max(1, h - size)))
        wins.append((x, y, min(size, w - x), min(size, h - y)))
    return wins


def compare_pixels(ours, base, sample=0):
    da, db = gdal.Open(ours), gdal.Open(base)
    out = []
    nlev = min(da.GetRasterBand(1).GetOverviewCount(), db.GetRasterBand(1).GetOverviewCount()) + 1
    for level in range(nlev):
        wins = sample_windows(da, level, sample) if sample else None
        ba, ma = read_level(da, level, wins)
        bb, mb = read_level(db, level, wins)
        if ba[0].shape != bb[0].shape:
            out.append({"level": level, "error": f"shape {ba[0].shape} vs {bb[0].shape}"})
            continue
        valid = (ma > 0) & (mb > 0)
        mask_diff = int(np.count_nonzero((ma > 0) != (mb > 0)))
        row = {"level": level, "shape": list(ba[0].shape), "maskDiff": mask_diff,
               "maskDiffPct": 100.0 * mask_diff / ma.size, "bands": []}
        for k, (x, y) in enumerate(zip(ba, bb)):
            xv = x[valid].astype(np.float64)
            yv = y[valid].astype(np.float64)
            fin = np.isfinite(xv) & np.isfinite(yv)
            d = np.abs(xv[fin] - yv[fin])
            entry = {"band": k + 1, "valid": int(valid.sum())}
            if d.size:
                entry["maxAbs"] = float(d.max())
                entry["meanAbs"] = float(d.mean())
                entry["pctOver1"] = 100.0 * float(np.count_nonzero(d > 1)) / d.size
                if x.dtype == np.uint8:
                    mse = float(np.mean(d * d))
                    entry["psnr"] = float("inf") if mse == 0 else 10 * math.log10(255 * 255 / mse)
            entry["nanMismatch"] = int(np.count_nonzero(np.isnan(xv) != np.isnan(yv))) if xv.dtype.kind == "f" else 0
            row["bands"].append(entry)
        out.append(row)
    return out


def compare_stats(ours, base):
    try:
        a = json.load(open(ours + ".stats.json"))
        b = json.load(open(base + ".stats.json"))
    except OSError as e:
        return {"error": str(e)}
    res = {"schemaKeys": sorted(a.keys()) == sorted(b.keys()), "bands": {}}
    for k in sorted(set(a.get("bands", {})) | set(b.get("bands", {})), key=int):
        x, y = a["bands"].get(k), b["bands"].get(k)
        if not x or not y:
            res["bands"][k] = {"missing": "ours" if not x else "baseline"}
            continue
        e = {}
        for f in ("min", "max", "mean", "std"):
            e[f] = (x[f], y[f])
        e["valid"] = (x["valid"], y["valid"])
        e["total"] = (x["total"], y["total"])
        res["bands"][k] = e
    return res


def tiler_read(lib, path):
    """Renders the tiles of the top and the bottom zoom levels through our TiffReader + tiler."""
    L = ctypes.CDLL(lib)
    L.ddbtMemoryTileFmt.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                                    ctypes.POINTER(ctypes.POINTER(ctypes.c_uint8)), ctypes.POINTER(ctypes.c_int),
                                    ctypes.c_int, ctypes.c_bool, ctypes.c_bool, ctypes.c_char_p, ctypes.c_char_p]
    L.ddbtLastError.restype = ctypes.c_char_p
    ds = gdal.Open(path)
    gt = ds.GetGeoTransform()
    R = 6378137 * math.pi
    res = gt[1]
    z = max(0, min(24, int(round(math.log2(2 * R / 256 / res)))))
    cx = gt[0] + gt[1] * ds.RasterXSize / 2
    cy = gt[3] + gt[5] * ds.RasterYSize / 2
    ok = 0
    errors = []
    for zz in (z, max(0, z - 3)):
        n = 2 ** zz
        tx = int((cx + R) / (2 * R) * n)
        ty = int((R - cy) / (2 * R) * n)
        buf = ctypes.POINTER(ctypes.c_uint8)()
        size = ctypes.c_int()
        rc = L.ddbtMemoryTileFmt(path.encode(), zz, tx, ty, ctypes.byref(buf), ctypes.byref(size), 256, True, True,
                                 None, b"png")
        if rc == 0 and size.value > 0:
            ok += 1
            L.ddbtFree(buf)
        else:
            errors.append(f"z{zz}/{tx}/{ty}: {rc} {L.ddbtLastError().decode()}")
    return {"tiles": ok, "errors": errors}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ours")
    ap.add_argument("baseline")
    ap.add_argument("--lib")
    ap.add_argument("--json")
    ap.add_argument("--no-pixels", action="store_true")
    ap.add_argument("--sample", type=int, default=0, help="compare N random 512 px windows per level (large rasters)")
    a = ap.parse_args()
    report = {}
    report["valid"], report["validator"] = validate(a.ours)
    ca, cb = contract(a.ours), contract(a.baseline)
    report["contractDiffs"] = contract_diff(ca, cb)
    if not a.no_pixels:
        report["levels"] = compare_pixels(a.ours, a.baseline, a.sample)
    report["stats"] = compare_stats(a.ours, a.baseline)
    if a.lib:
        report["tiler"] = tiler_read(a.lib, a.ours)
    report["sizeOurs"] = os.path.getsize(a.ours)
    report["sizeBaseline"] = os.path.getsize(a.baseline)
    text = json.dumps(report, indent=1)
    if a.json:
        with open(a.json, "w") as f:
            f.write(text)
    print(text)


if __name__ == "__main__":
    main()
