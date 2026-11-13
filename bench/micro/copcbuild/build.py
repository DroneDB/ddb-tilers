#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Times COPC builds of the ddbt CLI (`ddbt copc build`): wall, CPU and peak RSS per case.

    build.py --ddbt build/linux-gcc-release/bin/ddbt --cpus 8-15,24-31 --reps 3 \
        --case P4=/path/model.laz --case P1=/path/b.laz --out /work/dir [--options '{"threads":8}']

Each run: taskset to --cpus, /usr/bin/time -v (wall, user + sys CPU, peak RSS, file-system
output bytes), the build report (phases, partitions, spill bytes), output size and sha256 (the
outputs of the repetitions must be identical). Prints one JSON line per case (medians) and a
Markdown table. Outputs are deleted unless --keep.
"""
import argparse
import hashlib
import json
import os
import re
import statistics
import subprocess
import sys


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 24), b""):
            h.update(b)
    return h.hexdigest()


def run_once(a, name, inp, rep):
    out = os.path.join(a.out, f"{name}.{rep}.copc.laz")
    report = os.path.join(a.out, f"{name}.{rep}.report.json")
    opts = json.loads(a.options)
    opts["report"] = report
    if "tempDir" not in opts:
        opts["tempDir"] = a.out
    cmd = ["/usr/bin/time", "-v", "taskset", "-c", a.cpus, a.ddbt, "copc", "build", inp, out, json.dumps(opts)]
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"{name}: {r.stderr[-1500:]}")
    t = r.stderr

    def field(label, conv=float):
        m = re.search(re.escape(label) + r":\s*([0-9.:]+)", t)
        return conv(m.group(1)) if m else None

    wall = field("Elapsed (wall clock) time (h:mm:ss or m:ss)", str)
    parts = [float(x) for x in wall.split(":")]
    wall_s = parts[-1] + 60 * parts[-2] + (3600 * parts[-3] if len(parts) > 2 else 0)
    rep_json = json.load(open(report))
    res = {
        "wall": wall_s,
        "cpu": field("User time (seconds)") + field("System time (seconds)"),
        "rssMB": field("Maximum resident set size (kbytes)") / 1024,
        "fsOutMB": field("File system outputs") * 512 / 1e6,
        "outMB": os.path.getsize(out) / 1e6,
        "sha256": sha256(out),
        "report": rep_json,
    }
    if not a.keep:
        os.remove(out)
        meta = out + ".meta.json"
        if os.path.exists(meta):
            os.remove(meta)
    os.remove(report)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ddbt", required=True)
    ap.add_argument("--cpus", default="8-15,24-31")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--case", action="append", required=True, help="NAME=PATH")
    ap.add_argument("--out", required=True)
    ap.add_argument("--options", default="{}")
    ap.add_argument("--keep", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rows = []
    for c in a.case:
        name, inp = c.split("=", 1)
        runs = [run_once(a, name, inp, i) for i in range(a.reps)]
        shas = {r["sha256"] for r in runs}
        rep = runs[0]["report"]
        row = {
            "case": name,
            "points": rep["points"],
            "wall": statistics.median(r["wall"] for r in runs),
            "wallMin": min(r["wall"] for r in runs),
            "cpu": statistics.median(r["cpu"] for r in runs),
            "rssMB": statistics.median(r["rssMB"] for r in runs),
            "fsOutMB": statistics.median(r["fsOutMB"] for r in runs),
            "spillMB": rep["spillBytes"] / 1e6,
            "outMB": runs[0]["outMB"],
            "identical": len(shas) == 1,
            "sha256": runs[0]["sha256"],
            "partitions": rep["partitions"],
            "spilled": rep["spilled"],
            "phases": [statistics.median(r["report"][k] for r in runs) for k in ("secondsPlan", "secondsScatter", "secondsProcess")],
            "options": a.options,
        }
        rows.append(row)
        print(json.dumps(row), flush=True)
    print("| case | points | wall s (min) | CPU s | peak RSS MB | spill MB | written MB | output MB | plan / scatter / process s | identical |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        ph = " / ".join(f"{x:.2f}" for x in r["phases"])
        print(f"| {r['case']} | {r['points']:,} | {r['wall']:.2f} ({r['wallMin']:.2f}) | {r['cpu']:.1f} | {r['rssMB']:.0f} | "
              f"{r['spillMB']:.0f} | {r['fsOutMB']:.0f} | {r['outMB']:.1f} | {ph} | {'yes' if r['identical'] else 'NO'} |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
