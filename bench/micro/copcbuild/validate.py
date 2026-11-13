#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Validates a COPC built by ddbt against its input with PDAL (independent of ddbt's readers).

  python3 -I validate.py --pdal PDAL INPUT OUTPUT.copc.laz [--work DIR] [--colour-scale K] [--pts-intensity]

Checks:
  summary   pdal info --summary of the output: readers.copc, count = input count, the bounds of
            the output = the min / max of the input's points (pdal info --stats).
  metadata  copc, LAS 1.4, format 6/7/8 as expected, WKT bit, scale / offset.
  stats     pdal info --stats: per dimension count, minimum and maximum equal, sums equal to
            1e-9 relative (PDAL accumulates in point order).
  multiset  both files rewritten by `pdal translate` as LAS 1.4 records of the output's format,
            scale and offset (extra dims kept), then an order-independent hash of every record
            (sum and xor of a 64-bit hash per record) must be equal. --assign adds
            --colour-scale / --pts-intensity apply the documented conversions of non-LAS inputs
            to the input's records (8-bit colour x 257, PTS intensity (i + 2048) * 16).
Exit status 0 when every check passes. A JSON summary is printed last.
"""
import argparse
import json
import os
import struct
import subprocess
import sys

import numpy as np


def run(cmd):
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)} failed: {r.stderr[-2000:]}")
    return r.stdout


def pdal_json(pdal, args):
    return json.loads(run([pdal, "info"] + args))


def las_header(path):
    with open(path, "rb") as f:
        h = f.read(375)
    if h[:4] != b"LASF":
        raise RuntimeError(path + ": not LAS")
    minor = h[25]
    fmt = h[104] & 0x3F
    rec_len = struct.unpack_from("<H", h, 105)[0]
    offset = struct.unpack_from("<I", h, 96)[0]
    count = struct.unpack_from("<I", h, 107)[0]
    if minor >= 4:
        count = struct.unpack_from("<Q", h, 247)[0] or count
    scale = struct.unpack_from("<3d", h, 131)
    off = struct.unpack_from("<3d", h, 155)
    return dict(minor=minor, format=fmt, rec_len=rec_len, offset=offset, count=count, scale=scale, off=off)


M1 = np.uint64(0x9E3779B97F4A7C15)
M2 = np.uint64(0xBF58476D1CE4E5B9)
M3 = np.uint64(0x94D049BB133111EB)


def mix(x):
    x = x ^ (x >> np.uint64(30))
    x = x * M2
    x = x ^ (x >> np.uint64(27))
    x = x * M3
    return x ^ (x >> np.uint64(31))


def convert_input(rec, fmt, colour_scale, pts_intensity):
    """Documented conversions of non-LAS inputs, applied to PDAL's records of the input."""
    if colour_scale != 1 and fmt >= 7:
        rgb = rec[:, 30:36].copy().view("<u2").astype(np.float64)
        rgb = np.minimum(np.floor(rgb * colour_scale + 0.5), 65535).astype("<u2")
        rec[:, 30:36] = rgb.view(np.uint8).reshape(-1, 6)
    if pts_intensity:
        i = rec[:, 12:14].copy().view("<u2").astype(np.float64)
        rec[:, 12:14] = np.clip(np.floor((i + 2048) * 16 + 0.5), 0, 65535).astype("<u2").view(np.uint8).reshape(-1, 2)


def multiset_hash(path, chunk=4_000_000, colour_scale=1, pts_intensity=False, zero_intensity=False):
    h = las_header(path)
    n, rs = h["count"], h["rec_len"]
    words = (rs + 7) // 8
    total = np.uint64(0)
    xor = np.uint64(0)
    with open(path, "rb") as f:
        f.seek(h["offset"])
        done = 0
        with np.errstate(over="ignore"):
            while done < n:
                m = min(chunk, n - done)
                buf = np.frombuffer(f.read(m * rs), dtype=np.uint8)
                if buf.size != m * rs:
                    raise RuntimeError(path + ": short read")
                rec = np.zeros((m, words * 8), dtype=np.uint8)
                rec[:, :rs] = buf.reshape(m, rs)
                convert_input(rec, h["format"], colour_scale, pts_intensity)
                if zero_intensity:
                    rec[:, 12:14] = 0
                w = rec.view("<u8")
                acc = np.full(m, np.uint64(rs), dtype=np.uint64)
                for k in range(words):
                    acc = mix(acc ^ (w[:, k] + M1 * np.uint64(k + 1)))
                total = np.uint64(total + acc.sum(dtype=np.uint64))
                xor = np.uint64(xor ^ np.bitwise_xor.reduce(acc))
                done += m
    return dict(count=n, sum=f"{int(total):016x}", xor=f"{int(xor):016x}", rec_len=rs, format=h["format"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pdal", required=True)
    ap.add_argument("--work", default=None)
    ap.add_argument("--keep", action="store_true")
    ap.add_argument("--colour-scale", type=float, default=1.0,
                    help="multiply the input's colour (8-bit inputs: 257) before comparing")
    ap.add_argument("--pts-intensity", action="store_true", help="input intensity i becomes (i + 2048) * 16")
    ap.add_argument("--ignore-intensity", action="store_true",
                    help="leave intensity out (E57 with degenerate limits: PDAL writes 0, ddbt the value)")
    ap.add_argument("--reader", default=None, help="PDAL reader of the input (default: inferred)")
    ap.add_argument("--input-arg", action="append", default=[],
                    help="extra pdal argument for reading the input (e.g. --readers.text.header=...)")
    ap.add_argument("--no-multiset", action="store_true")
    ap.add_argument("input")
    ap.add_argument("output")
    a = ap.parse_args()
    work = a.work or os.path.dirname(os.path.abspath(a.output))
    os.makedirs(work, exist_ok=True)
    res = {"input": a.input, "output": a.output, "checks": {}}
    ok = True

    def check(name, passed, detail):
        nonlocal ok
        res["checks"][name] = {"ok": bool(passed), "detail": detail}
        ok = ok and bool(passed)
        print(f"[{'OK' if passed else 'FAIL'}] {name}: {detail}")

    reader_args = (["--driver", a.reader] if a.reader else []) + a.input_arg
    is_las = a.input.lower().endswith((".las", ".laz"))
    st_in = pdal_json(a.pdal, ["--stats", a.input] + reader_args)["stats"]["statistic"]
    st_out = pdal_json(a.pdal, ["--stats", a.output])["stats"]["statistic"]
    din = {d["name"]: d for d in st_in}
    dout = {d["name"]: d for d in st_out}
    summ = pdal_json(a.pdal, ["--summary", a.output])["summary"]
    meta = pdal_json(a.pdal, ["--metadata", a.output])["metadata"]
    hdr = las_header(a.output)

    n_in = din["X"]["count"]
    b = summ["bounds"]
    exp_b = {"minx": din["X"]["minimum"], "maxx": din["X"]["maximum"], "miny": din["Y"]["minimum"],
             "maxy": din["Y"]["maximum"], "minz": din["Z"]["minimum"], "maxz": din["Z"]["maximum"]}
    bounds_ok = all(abs(b[k] - exp_b[k]) <= 0.5 * hdr["scale"]["xyz".index(k[3])] + 1e-9 for k in exp_b)
    check("summary", summ["num_points"] == n_in and bounds_ok,
          f"count {summ['num_points']} vs {n_in}; bounds {b} vs input points {exp_b}")
    check("metadata", meta.get("copc") is True and meta.get("minor_version") == 4 and
          meta.get("dataformat_id") in (6, 7, 8) and (meta.get("global_encoding", 0) & 16) != 0,
          f"copc={meta.get('copc')} 1.{meta.get('minor_version')} format {meta.get('dataformat_id')} "
          f"global_encoding {meta.get('global_encoding')} scale {hdr['scale']} offset {hdr['off']}")

    bad = []
    dropped = []
    for name, d in din.items():
        o = dout.get(name)
        if o is None:
            if is_las:
                bad.append(f"{name} missing")
            else:
                dropped.append(name)
            continue
        sx = d["average"] * d["count"]
        sy = o["average"] * o["count"]
        tol = 1e-9 * max(1.0, abs(sx))
        if (name in ("Red", "Green", "Blue") and a.colour_scale != 1) or (name == "Intensity" and (a.pts_intensity or a.ignore_intensity)):
            continue
        if name == "ScanAngleRank":  # formats 6-8 store it in 0.006 degree steps (PDRF 0-5: whole degrees)
            if d["count"] != o["count"] or abs(d["minimum"] - o["minimum"]) > 0.0031 or abs(d["maximum"] - o["maximum"]) > 0.0031:
                bad.append(f"{name}: in {d['minimum']}/{d['maximum']} out {o['minimum']}/{o['maximum']}")
            continue
        if not is_las and name in ("X", "Y", "Z"):
            q = 0.5 * hdr["scale"]["XYZ".index(name)] + 1e-9
            if (d["count"] != o["count"] or abs(d["minimum"] - o["minimum"]) > q or abs(d["maximum"] - o["maximum"]) > q
                    or abs(sx - sy) > q * d["count"]):
                bad.append(f"{name}: in {d['minimum']}/{d['maximum']}/{sx} out {o['minimum']}/{o['maximum']}/{sy}")
            continue
        if d["count"] != o["count"] or d["minimum"] != o["minimum"] or d["maximum"] != o["maximum"] or abs(sx - sy) > tol:
            bad.append(f"{name}: in {d['count']}/{d['minimum']}/{d['maximum']}/{sx} out {o['count']}/{o['minimum']}/{o['maximum']}/{sy}")
    check("stats", not bad, ("; ".join(bad) if bad else f"{len(din) - len(dropped)} dimensions equal") +
          (f" (not kept: {', '.join(dropped)})" if dropped else ""))

    if not a.no_multiset:
        fmt = meta["dataformat_id"]
        sc = [meta["scale_x"], meta["scale_y"], meta["scale_z"]]
        of = [meta["offset_x"], meta["offset_y"], meta["offset_z"]]
        common = [f"--writers.las.minor_version=4", f"--writers.las.dataformat_id={fmt}",
                  "--writers.las.compression=false"]
        if is_las:
            common.append("--writers.las.extra_dims=all")
        for i, ax in enumerate("xyz"):
            common += [f"--writers.las.scale_{ax}={sc[i]!r}", f"--writers.las.offset_{ax}={of[i]!r}"]
        tin = os.path.join(work, "validate.in.las")
        tout = os.path.join(work, "validate.out.las")
        translate_args = (["--reader", a.reader] if a.reader else []) + a.input_arg
        run([a.pdal, "translate", a.input, tin] + common + translate_args)
        run([a.pdal, "translate", a.output, tout] + common)
        try:
            hi = multiset_hash(tin, colour_scale=a.colour_scale, pts_intensity=a.pts_intensity, zero_intensity=a.ignore_intensity)
            ho = multiset_hash(tout, zero_intensity=a.ignore_intensity)
        finally:
            if not a.keep:
                for p in (tin, tout):
                    if os.path.exists(p):
                        os.remove(p)
        check("multiset", hi == ho, f"input {hi} output {ho}")

    print(json.dumps(res))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
