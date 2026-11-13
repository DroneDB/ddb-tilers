#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Validate a COPC file: point count, bounds and SRS, optionally against a reference.

With a `pdal` executable (--pdal, $DDBT_PDAL, the baseline bin dir or PATH): `pdal info
--metadata --summary` (and --stats with --stats). Without one (the DroneDB 2.4.1 .deb ships
libpdalcpp but no pdal executable), the LAS 1.4 header and VLRs are read directly: version,
point format, count, scale/offset, bounds, WKT, and the COPC info VLR (must be the first VLR,
at offset 375, as the COPC specification requires).

Usage: validate_copc.py FILE [--reference INPUT_OR_COPC] [--stats] [--out result.json]
Exit code 0 when the file is a COPC (and, with --reference, counts/bounds/SRS match).
"""
import argparse
import json
import os
import shutil
import struct
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common  # noqa: E402


def find_pdal(explicit=None):
    for c in (explicit, os.environ.get("DDBT_PDAL"), os.path.join(common.BASELINE_DIR, "usr", "bin", "pdal")):
        if c and os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    return shutil.which("pdal")


def read_las_header(path):
    """LAS/LAZ/COPC header summary (pure Python, reads the header and the VLRs only)."""
    with open(path, "rb") as f:
        h = f.read(375)
        if len(h) < 227 or h[:4] != b"LASF":
            raise ValueError("not a LAS file")
        vmaj, vmin = h[24], h[25]
        header_size, = struct.unpack_from("<H", h, 94)
        offset_points, nvlr = struct.unpack_from("<II", h, 96)
        pdrf_raw, rec_len = struct.unpack_from("<BH", h, 104)
        legacy_count, = struct.unpack_from("<I", h, 107)
        sx, sy, sz, ox, oy, oz = struct.unpack_from("<6d", h, 131)
        maxx, minx, maxy, miny, maxz, minz = struct.unpack_from("<6d", h, 179)
        count = legacy_count
        n_evlr = 0
        if (vmaj, vmin) >= (1, 4) and len(h) >= 255:
            evlr_start, n_evlr = struct.unpack_from("<QI", h, 235)
            count, = struct.unpack_from("<Q", h, 247)
        out = {
            "version": "%d.%d" % (vmaj, vmin), "point_format": pdrf_raw & 0x3F,
            "compressed": bool(pdrf_raw & 0x80), "point_record_length": rec_len,
            "count": count, "scale": [sx, sy, sz], "offset": [ox, oy, oz],
            "bounds": {"min": [minx, miny, minz], "max": [maxx, maxy, maxz]},
            "header_size": header_size, "vlr_count": nvlr, "evlr_count": n_evlr,
        }
        vlrs = []
        pos = header_size
        wkt = None
        geokeys = False
        copc = None
        for _ in range(nvlr):
            f.seek(pos)
            vh = f.read(54)
            if len(vh) < 54:
                break
            user = vh[2:18].split(b"\0", 1)[0].decode("ascii", "replace")
            rid, length = struct.unpack_from("<HH", vh, 18)
            vlrs.append({"user_id": user, "record_id": rid, "length": length, "offset": pos})
            body_pos = pos + 54
            if user == "LASF_Projection" and rid == 2112:
                f.seek(body_pos)
                wkt = f.read(length).split(b"\0", 1)[0].decode("utf-8", "replace")
            if user == "LASF_Projection" and rid == 34735:
                geokeys = True
            if user == "copc" and rid == 1 and length >= 160:
                f.seek(body_pos)
                b = f.read(160)
                cx, cy, cz, half, spacing = struct.unpack_from("<5d", b, 0)
                root_off, root_size = struct.unpack_from("<QQ", b, 40)
                gmin, gmax = struct.unpack_from("<2d", b, 56)
                copc = {"vlr_offset": pos, "center": [cx, cy, cz], "halfsize": half, "spacing": spacing,
                        "root_hier_offset": root_off, "root_hier_size": root_size, "gpstime": [gmin, gmax]}
            pos = body_pos + length
        out["vlrs"] = vlrs
        out["wkt"] = wkt
        out["geokeys"] = geokeys
        out["copc_info"] = copc
        out["is_copc"] = bool(copc and copc["vlr_offset"] == 375 and out["compressed"]
                              and out["point_format"] in (6, 7, 8))
        return out


def pdal_info(pdal, path, stats=False):
    cmd = [pdal, "info", "--metadata", "--summary"] + (["--stats"] if stats else []) + [path]
    env = dict(os.environ, GDAL_PAM_ENABLED="NO")
    p = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if p.returncode != 0:
        raise RuntimeError("pdal info failed: " + p.stderr.strip()[-2000:])
    return json.loads(p.stdout)


def summary_from_pdal(info):
    s = info.get("summary") or {}
    md = info.get("metadata") or {}
    b = s.get("bounds") or {}
    return {"count": s.get("num_points", md.get("count")),
            "bounds": {"min": [b.get("minx"), b.get("miny"), b.get("minz")],
                       "max": [b.get("maxx"), b.get("maxy"), b.get("maxz")]},
            "srs_wkt": ((s.get("srs") or {}).get("wkt")) or md.get("comp_spatialreference")}


def compare(a, b, scale, warnings):
    diffs = []
    if a.get("count") != b.get("count"):
        diffs.append("count: %r vs %r" % (a.get("count"), b.get("count")))
    tol = [max(abs(s), 1e-9) * 1.0001 for s in scale] if scale else [1e-6] * 3
    for k in ("min", "max"):
        va, vb = a["bounds"][k], b["bounds"][k]
        for i in range(3):
            if va[i] is None or vb[i] is None or abs(va[i] - vb[i]) > tol[i]:
                diffs.append("bounds %s[%d]: %r vs %r" % (k, i, va[i], vb[i]))
    has_a = bool(a.get("srs_wkt") or a.get("srs_geokeys"))
    has_b = bool(b.get("srs_wkt") or b.get("srs_geokeys"))
    if has_a != has_b:
        diffs.append("SRS present in one only")
    elif bool(a.get("srs_wkt")) != bool(b.get("srs_wkt")):
        warnings.append("SRS forms differ (WKT vs GeoTIFF keys): not compared")
    elif a.get("srs_wkt") and a.get("srs_wkt") != b.get("srs_wkt"):
        warnings.append("SRS WKT strings differ (may be equivalent)")
    return diffs


def describe(path, pdal=None, stats=False):
    d = {"file": path}
    try:
        d["header"] = read_las_header(path)
    except (OSError, ValueError, struct.error) as e:
        d["header_error"] = str(e)
    if pdal:
        try:
            info = pdal_info(pdal, path, stats)
            d["pdal"] = {"summary": summary_from_pdal(info)}
            if stats:
                d["pdal"]["stats"] = info.get("stats")
        except (RuntimeError, ValueError) as e:
            d["pdal_error"] = str(e)
    if "pdal" in d:
        d["summary"] = d["pdal"]["summary"]
    elif "header" in d:
        h = d["header"]
        d["summary"] = {"count": h["count"], "bounds": h["bounds"], "srs_wkt": h["wkt"],
                        "srs_geokeys": h["geokeys"]}
    return d


def validate(path, reference=None, pdal=None, stats=False):
    res = {"schema": "ddbt-validate-copc/1", "pdal": pdal}
    res["target"] = describe(path, pdal, stats)
    h = res["target"].get("header") or {}
    ok = bool(h.get("is_copc")) and "summary" in res["target"]
    if not h.get("is_copc"):
        res["problems"] = ["not a COPC file (COPC info VLR at offset 375, LAZ, PDRF 6-8 expected)"]
    if reference:
        res["reference"] = describe(reference, pdal, stats)
        if "summary" in res["reference"] and "summary" in res["target"]:
            res["warnings"] = []
            res["diffs"] = compare(res["target"]["summary"], res["reference"]["summary"], h.get("scale"),
                                   res["warnings"])
            ok = ok and not res["diffs"]
        else:
            ok = False
    res["ok"] = ok
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("--reference")
    ap.add_argument("--stats", action="store_true")
    ap.add_argument("--pdal")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    res = validate(a.file, a.reference, find_pdal(a.pdal), a.stats)
    if a.out:
        common.write_json(a.out, res)
    brief = {"ok": res["ok"], "pdal": res["pdal"], "summary": res["target"].get("summary"),
             "diffs": res.get("diffs"), "warnings": res.get("warnings"), "problems": res.get("problems")}
    if brief["summary"] and brief["summary"].get("srs_wkt"):
        brief["summary"] = dict(brief["summary"], srs_wkt=brief["summary"]["srs_wkt"][:80] + "...")
    print(json.dumps(brief, indent=1))
    return 0 if res["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
