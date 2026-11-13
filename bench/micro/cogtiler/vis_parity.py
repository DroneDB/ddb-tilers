#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Parity of tiles with visualisation parameters: libddb 2.4.1's DDBMemoryTileEx2 against
ddbtMemoryTileEx2, through ctypes, one worker process per backend (the legacy one in the
baseline environment and its own DDB_HOME).

A case file (JSON list) gives the requests:
  {"legacy": PATH, "ddbt": PATH | "artefact", "z": Z, "x": X, "y": Y, "size": 256, "tms": true,
   "preset": .., "bands": .., "formula": .., "bandFilter": .., "colormap": .., "rescale": ..,
   "nodata": N, "rescaleUnit": U, "label": "..."}
("artefact": the COG libddb built for the legacy path in its DDB_HOME, so both read one file.)
Compares return codes, error messages and the decoded tiles (bench/harness/tilecmp.py).

  python3 bench/micro/cogtiler/vis_parity.py --cases cases.json --out DIR \
      --ddbt-lib /mnt/data/ddb-tilers-work/build-cogtile/lib/libddbt.so
"""
import argparse
import ctypes
import glob
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HARNESS = os.path.normpath(os.path.join(HERE, "..", "..", "harness"))
sys.path.insert(0, HARNESS)
import common  # noqa: E402
import tilecmp  # noqa: E402


class RasterOptions(ctypes.Structure):
    _fields_ = [("structSize", ctypes.c_uint32), ("flags", ctypes.c_uint32), ("nodata", ctypes.c_double),
                ("bbox", ctypes.c_double * 4), ("formula", ctypes.c_char_p), ("rescaleUnit", ctypes.c_char_p)]


def enc(s):
    return None if s is None else s.encode()


def worker(kind, lib, cases_file, out):
    L = ctypes.CDLL(lib)
    if kind == "legacy":
        L.DDBRegisterProcess(False)
        tile, free, last = L.DDBMemoryTileEx2, L.DDBVSIFree, L.DDBGetLastError
    else:
        tile, free, last = L.ddbtMemoryTileEx2, L.ddbtFree, L.ddbtLastError
    last.restype = ctypes.c_char_p
    tile.restype = ctypes.c_int
    tile.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_bool,
                     ctypes.c_bool, ctypes.c_char_p] + [ctypes.c_char_p] * 6 + [
                         ctypes.POINTER(RasterOptions), ctypes.POINTER(ctypes.POINTER(ctypes.c_uint8)),
                         ctypes.POINTER(ctypes.c_int)]
    with open(cases_file) as f:
        cases = json.load(f)
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "index.jsonl"), "w") as idx:
        for i, c in enumerate(cases):
            path = c["path_" + kind]
            opts = None
            if "nodata" in c or "rescaleUnit" in c:
                o = RasterOptions()
                o.structSize = ctypes.sizeof(RasterOptions)
                if "nodata" in c:
                    o.flags |= 1
                    o.nodata = c["nodata"]
                o.rescaleUnit = enc(c.get("rescaleUnit"))
                opts = ctypes.pointer(o)
            buf = ctypes.POINTER(ctypes.c_uint8)()
            size = ctypes.c_int(0)
            code = tile(path.encode(), c["z"], c["x"], c["y"], c.get("size", 256), c.get("tms", True), False, None,
                        enc(c.get("preset")), enc(c.get("bands")), enc(c.get("formula")), enc(c.get("bandFilter")),
                        enc(c.get("colormap")), enc(c.get("rescale")), opts, ctypes.byref(buf), ctypes.byref(size))
            e = {"i": i, "z": c["z"], "x": c["x"], "y": c["y"], "code": code, "label": c.get("label", "")}
            if code == 0:
                name = "%06d_%d_%d_%d.png" % (i, c["z"], c["x"], c["y"])
                with open(os.path.join(out, name), "wb") as g:
                    g.write(ctypes.string_at(buf, size.value))
                free(buf)
                e.update({"ok": True, "file": name})
            else:
                e.update({"ok": False, "error": (last() or b"").decode(errors="replace")})
            idx.write(json.dumps(e) + "\n")


def find_artefact(home):
    arts = sorted(glob.glob(os.path.join(home, ".ddb", "artefacts", "*.tif")), key=os.path.getmtime)
    return arts[-1] if arts else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cases", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--ddbt-lib", default=common.default_ddbt_lib())
    ap.add_argument("--home", default=os.path.join(common.HOME_ROOT, "cogtiler-vis"))
    ap.add_argument("--pin", default="8-15,24-31")
    ap.add_argument("--psnr-min", type=float, default=45.0)
    ap.add_argument("--worker", choices=["legacy", "ddbt"])
    ap.add_argument("--lib")
    a = ap.parse_args()
    if a.worker:
        worker(a.worker, a.lib, a.cases, a.out)
        return
    with open(a.cases) as f:
        cases = json.load(f)
    os.makedirs(a.out, exist_ok=True)
    # Legacy first: it builds its artefacts, which "artefact" cases then name for ddbt.
    homes = {}
    for c in cases:
        c["path_legacy"] = c["legacy"]
        key = c.get("home", os.path.basename(c["legacy"]))
        homes.setdefault(key, os.path.join(a.home, key))
        c["_home"] = homes[key]
    by_home = {}
    for c in cases:
        by_home.setdefault(c["_home"], []).append(c)
    leg_dirs = []
    for home, cs in by_home.items():
        os.makedirs(home, exist_ok=True)
        cf = os.path.join(a.out, "cases-%s.json" % os.path.basename(home))
        with open(cf, "w") as f:
            json.dump(cs, f)
        d = os.path.join(a.out, "legacy-" + os.path.basename(home))
        env = common.baseline_env()
        env["GDAL_PAM_ENABLED"] = "NO"
        env["DDB_HOME"] = home
        p = subprocess.run(["taskset", "-c", a.pin, sys.executable, os.path.abspath(__file__), "--worker", "legacy",
                            "--lib", common.baseline_lib(), "--cases", cf, "--out", d],
                           env=env, capture_output=True, text=True)
        if p.returncode != 0:
            raise SystemExit("legacy worker failed: " + p.stderr[-3000:])
        art = find_artefact(home)
        for c in cs:
            c["path_ddbt"] = (art or c["legacy"]) if c.get("ddbt", "artefact") == "artefact" else c["ddbt"]
        leg_dirs.append((home, cs, d))
    # One ddbt run over every case (in the original order) and one merged legacy dump.
    allcases = [c for _, cs, _ in leg_dirs for c in cs]
    cf = os.path.join(a.out, "cases-all.json")
    with open(cf, "w") as f:
        json.dump(allcases, f)
    env = {k: v for k, v in os.environ.items() if not k.startswith("DDB_")}
    dn = os.path.join(a.out, "ddbt")
    p = subprocess.run(["taskset", "-c", a.pin, sys.executable, os.path.abspath(__file__), "--worker", "ddbt", "--lib",
                        a.ddbt_lib, "--cases", cf, "--out", dn], env=env, capture_output=True, text=True)
    if p.returncode != 0:
        raise SystemExit("ddbt worker failed: " + p.stderr[-3000:])
    dl = os.path.join(a.out, "legacy")
    os.makedirs(dl, exist_ok=True)
    n = 0
    with open(os.path.join(dl, "index.jsonl"), "w") as idx:
        for _, cs, d in leg_dirs:
            with open(os.path.join(d, "index.jsonl")) as f:
                for line in f:
                    e = json.loads(line)
                    if e.get("file"):
                        src = os.path.join(d, e["file"])
                        e["file"] = "%06d_%d_%d_%d.png" % (n, e["z"], e["x"], e["y"])
                        os.replace(src, os.path.join(dl, e["file"]))
                    e["i"] = n
                    idx.write(json.dumps(e) + "\n")
                    n += 1
    leg = [json.loads(l) for l in open(os.path.join(dl, "index.jsonl"))]
    new = [json.loads(l) for l in open(os.path.join(dn, "index.jsonl"))]
    mismatched = []
    for c, el, en in zip(allcases, leg, new):
        if el["ok"] != en["ok"] or (not el["ok"] and (el["error"] != en["error"])):
            mismatched.append({"label": c.get("label"), "legacy": [el["code"], el.get("error")],
                               "ddbt": [en["code"], en.get("error")]})
    r = tilecmp.compare_dirs(dl, dn, {"psnr_min": a.psnr_min}, diff_dir=os.path.join(a.out, "diff"))
    per = []
    for c, t in zip(allcases, r["tiles"]):
        per.append({"label": c.get("label"), "exact": t.get("exact"), "psnr": t.get("psnr"),
                    "iou": t.get("alpha_iou"), "pass": t.get("pass")})
    res = {"cases": len(allcases), "status_mismatches": mismatched, "summary": r["summary"], "tiles": per}
    with open(os.path.join(a.out, "vis_parity.json"), "w") as f:
        json.dump(tilecmp._json_safe(res), f, indent=1)
    print(json.dumps(tilecmp._json_safe({k: res[k] for k in ("cases", "status_mismatches", "summary")}), indent=1))


if __name__ == "__main__":
    main()
