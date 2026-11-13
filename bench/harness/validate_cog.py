#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Validate a Cloud Optimized GeoTIFF with the system GDAL and extract its contract.

* GDAL's validate_cloud_optimized_geotiff.py (--full-check=yes)
* gdalinfo -json contract: size, bands (type, nodata, mask flags, block size, overview sizes),
  CRS (WKT and EPSG when known), geotransform, IMAGE_STRUCTURE metadata (COMPRESSION,
  INTERLEAVE, LAYOUT), file size
* --reference other.tif: contract comparison (size, band count and types, overview count and
  sizes, CRS, compression and geotransform within --gt-tol, relative)

GDAL_PAM_ENABLED=NO is forced: no .aux.xml is ever written next to an input.
Usage: validate_cog.py FILE [--reference REF] [--out result.json] [--no-full-check]
Exit code 0 when valid (and, with --reference, when the contracts match).
"""
import argparse
import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common  # noqa: E402

VALIDATOR = "/usr/lib/python3/dist-packages/osgeo_utils/samples/validate_cloud_optimized_geotiff.py"


def gdal_env():
    env = dict(os.environ)
    # The baseline env (LD_LIBRARY_PATH with libddb's GDAL) must not leak into the system GDAL.
    for k in ("LD_LIBRARY_PATH", "GDAL_DATA", "PROJ_DATA", "PROJ_LIB", "GDAL_DRIVER_PATH"):
        env.pop(k, None)
    env["GDAL_PAM_ENABLED"] = "NO"
    return env


def run_validator(path, full_check=True):
    if not os.path.isfile(VALIDATOR):
        return {"ran": False, "error": "validator not found: " + VALIDATOR}
    cmd = [sys.executable if sys.executable else "python3", VALIDATOR]
    if full_check:
        cmd.append("--full-check=yes")
    cmd.append(path)
    p = subprocess.run(cmd, capture_output=True, text=True, env=gdal_env())
    out = (p.stdout + p.stderr).strip()
    return {"ran": True, "valid": p.returncode == 0, "exit_code": p.returncode, "output": out[-4000:]}


def gdalinfo(path):
    p = subprocess.run(["gdalinfo", "-json", path], capture_output=True, text=True, env=gdal_env())
    if p.returncode != 0:
        raise RuntimeError("gdalinfo failed: " + p.stderr.strip())
    return json.loads(p.stdout)


def contract(info, path=None):
    md = (info.get("metadata") or {}).get("IMAGE_STRUCTURE") or {}
    cs = info.get("coordinateSystem") or {}
    stac = info.get("stac") or {}
    bands = []
    for b in info.get("bands", []):
        bands.append({
            "type": b.get("type"),
            "block": b.get("block"),
            "nodata": b.get("noDataValue"),
            "mask_flags": (b.get("mask") or {}).get("flags"),
            "color": b.get("colorInterpretation"),
            "overviews": [o.get("size") for o in b.get("overviews", [])],
        })
    c = {
        "size": info.get("size"),
        "band_count": len(bands),
        "bands": bands,
        "wkt": cs.get("wkt"),
        "epsg": stac.get("proj:epsg"),
        "geotransform": info.get("geoTransform"),
        "compression": md.get("COMPRESSION"),
        "interleave": md.get("INTERLEAVE"),
        "layout": md.get("LAYOUT"),
        "overview_count": len(bands[0]["overviews"]) if bands else 0,
    }
    if path and os.path.exists(path):
        c["file_size"] = os.path.getsize(path)
    return c


def compare_contracts(a, b, gt_tol=1e-9):
    diffs = []
    for k in ("size", "band_count", "overview_count", "compression", "interleave", "epsg"):
        if a.get(k) != b.get(k):
            diffs.append("%s: %r vs %r" % (k, a.get(k), b.get(k)))
    ta = [x["type"] for x in a.get("bands", [])]
    tb = [x["type"] for x in b.get("bands", [])]
    if ta != tb:
        diffs.append("band types: %r vs %r" % (ta, tb))
    oa = [x["overviews"] for x in a.get("bands", [])[:1]]
    ob = [x["overviews"] for x in b.get("bands", [])[:1]]
    if oa != ob:
        diffs.append("overview sizes: %r vs %r" % (oa, ob))
    ga, gb = a.get("geotransform"), b.get("geotransform")
    if (ga is None) != (gb is None):
        diffs.append("geotransform present in one only")
    elif ga is not None:
        for i, (x, y) in enumerate(zip(ga, gb)):
            scale = max(abs(x), abs(y), 1e-12)
            if abs(x - y) > gt_tol * scale and abs(x - y) > 1e-12:
                diffs.append("geotransform[%d]: %r vs %r" % (i, x, y))
    if (a.get("wkt") or "") != (b.get("wkt") or "") and a.get("epsg") is None:
        diffs.append("CRS WKT differs")
    return diffs


def validate(path, reference=None, full_check=True, gt_tol=1e-9):
    res = {"schema": "ddbt-validate-cog/1", "file": path}
    res["validator"] = run_validator(path, full_check)
    try:
        res["contract"] = contract(gdalinfo(path), path)
    except Exception as e:  # noqa: BLE001 - reported in the result
        res["contract_error"] = str(e)
    ok = res["validator"].get("valid", False) and "contract" in res
    if reference:
        try:
            ref = contract(gdalinfo(reference), reference)
            res["reference"] = {"file": reference, "contract": ref}
            if "contract" in res:
                res["contract_diffs"] = compare_contracts(res["contract"], ref, gt_tol)
                ok = ok and not res["contract_diffs"]
        except Exception as e:  # noqa: BLE001
            res["reference_error"] = str(e)
            ok = False
    res["ok"] = bool(ok)
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("--reference")
    ap.add_argument("--out")
    ap.add_argument("--no-full-check", action="store_true")
    ap.add_argument("--gt-tol", type=float, default=1e-9)
    a = ap.parse_args(argv)
    res = validate(a.file, a.reference, not a.no_full_check, a.gt_tol)
    if a.out:
        common.write_json(a.out, res)
    print(json.dumps({k: res[k] for k in res if k not in ("contract", "reference")}, indent=1))
    return 0 if res["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
