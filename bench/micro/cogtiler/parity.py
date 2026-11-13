#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Tile parity of the COG tiler against the frozen baseline (libddb 2.4.1).

Generates a request set (bench/harness/requests.py) for a corpus item, renders it with both
backends through ddbt-bench (--dump-tiles, one thread pinned to --pin), and compares the dumps
with bench/harness/tilecmp.py. The two backends may read different files with the same tiles:

  --legacy-path   what libddb tiles (default: the item's path; a raster that is not a libddb
                  artefact is converted by libddb into DDB_HOME first, as in production)
  --ddbt-path     what ddbt tiles (default: the same path; "artefact" = the COG libddb built in
                  its DDB_HOME for --legacy-path, the "exactness" comparison: both read one file)

Legacy runs get baseline.env and a DDB_HOME kept under work/home/cogtiler-parity/<label> (the
artefacts survive between runs); ddbt runs get the shell environment without DDB_*.

  python3 bench/micro/cogtiler/parity.py --dataset R1 --scenario sweep --cap 64 \
      --ddbt-lib /mnt/data/ddb-tilers-work/build-cogtile/lib/libddbt.so --out /tmp/x
"""
import argparse
import glob
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HARNESS = os.path.normpath(os.path.join(HERE, "..", "..", "harness"))
sys.path.insert(0, HARNESS)
import common  # noqa: E402
import requests as reqgen  # noqa: E402
import tilecmp  # noqa: E402

PARITY_HOME = os.path.join(common.HOME_ROOT, "cogtiler-parity")


def legacy_env(home):
    env = common.baseline_env()
    env["GDAL_PAM_ENABLED"] = "NO"
    env["DDB_HOME"] = home
    return env


def ddbt_env(extra):
    env = {k: v for k, v in os.environ.items() if not k.startswith("DDB_")}
    for kv in extra or []:
        k, _, v = kv.partition("=")
        env[k] = v
    return env


def run_driver(driver, backend, lib, reqfile, dump, out_json, env, pin, size, fmt, threads):
    if os.path.isdir(dump):
        shutil.rmtree(dump)
    cmd = [driver, "tile", "--backend", backend, "--lib", lib, "--requests", reqfile, "--threads", str(threads),
           "--pin", pin, "--warmup-rounds", "0", "--rounds", "1", "--format", fmt, "--size", str(size), "--hashes",
           "--dump-tiles", dump, "--out", out_json]
    cmd = ["taskset", "-c", pin] + cmd
    p = subprocess.run(cmd, env=env, capture_output=True, text=True)
    if p.returncode != 0:
        raise SystemExit("%s run failed: %s" % (backend, p.stderr[-2000:]))
    return common.read_json(out_json)


def rewrite_requests(src, dst, path):
    with open(src) as f, open(dst, "w") as g:
        for line in f:
            if not line.strip():
                continue
            r = json.loads(line)
            r["path"] = path
            g.write(json.dumps(r) + "\n")


def find_artefact(home):
    arts = sorted(glob.glob(os.path.join(home, ".ddb", "artefacts", "*.tif")), key=os.path.getmtime)
    return arts[-1] if arts else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--label", help="DDB_HOME / output label (default: dataset)")
    ap.add_argument("--scenario", default="sweep", choices=["sweep", "pan", "retina", "edge"])
    ap.add_argument("--cap", type=int, default=64, help="sweep: tiles per zoom")
    ap.add_argument("--min-requests", type=int, default=300)
    ap.add_argument("--steps", type=int, default=60)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--minz", type=int)
    ap.add_argument("--maxz", type=int)
    ap.add_argument("--format", default="png")
    ap.add_argument("--legacy-path")
    ap.add_argument("--ddbt-path", help="a path, or 'artefact'")
    ap.add_argument("--ddbt-lib", default=common.default_ddbt_lib())
    ap.add_argument("--ddbt-env", action="append")
    ap.add_argument("--driver")
    ap.add_argument("--pin", default="8")
    ap.add_argument("--threads", type=int, default=1)
    ap.add_argument("--prepare-cpus", default="8-15,24-31", help="CPUs of libddb's conversion of a raw input")
    ap.add_argument("--psnr-min", type=float, default=45.0)
    ap.add_argument("--iou-min", type=float, default=0.999)
    ap.add_argument("--ssim-min", type=float, default=0.98)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    driver = common.find_driver(a.driver)
    items = {i["id"]: i for i in common.read_json(common.MANIFEST)["items"]}
    item = items[a.dataset]
    label = a.label or a.dataset
    os.makedirs(a.out, exist_ok=True)
    fp, minz, maxz = common.dataset_geometry(item)
    minz = a.minz if a.minz is not None else minz
    maxz = a.maxz if a.maxz is not None else maxz
    legacy_path = a.legacy_path or item["path"]
    size = 512 if a.scenario == "retina" else 256
    base_req = os.path.join(a.out, "requests.jsonl")
    reqs = reqgen.generate(a.scenario, legacy_path, fp, minz, maxz, a.format, a.seed, a.cap, a.steps, a.min_requests)
    reqgen.write_jsonl(reqs, base_req)

    home = os.path.join(PARITY_HOME, label)
    os.makedirs(home, exist_ok=True)
    if "/.ddb/build/" not in legacy_path and find_artefact(home) is None:
        # libddb converts the input on its first tile: let that build use --prepare-cpus.
        first = os.path.join(a.out, "requests-first.jsonl")
        with open(base_req) as f, open(first, "w") as g:
            g.write(f.readline())
        cmd = ["taskset", "-c", a.prepare_cpus, driver, "tile", "--backend", "legacy", "--lib",
               common.baseline_lib(), "--requests", first, "--threads", "1", "--rounds", "1", "--warmup-rounds", "0",
               "--out", os.path.join(a.out, "legacy-prepare.json")]
        p = subprocess.run(cmd, env=legacy_env(home), capture_output=True, text=True)
        if p.returncode != 0:
            raise SystemExit("legacy prepare failed: %s" % p.stderr[-2000:])
    leg = run_driver(driver, "legacy", common.baseline_lib(), base_req, os.path.join(a.out, "tiles-legacy"),
                     os.path.join(a.out, "legacy.json"), legacy_env(home), a.pin, size, a.format, a.threads)

    ddbt_path = a.ddbt_path or legacy_path
    if ddbt_path == "artefact":
        ddbt_path = find_artefact(home) or legacy_path
    ddbt_req = os.path.join(a.out, "requests-ddbt.jsonl")
    rewrite_requests(base_req, ddbt_req, ddbt_path)
    new = run_driver(driver, "ddbt", a.ddbt_lib, ddbt_req, os.path.join(a.out, "tiles-ddbt"),
                     os.path.join(a.out, "ddbt.json"), ddbt_env(a.ddbt_env), a.pin, size, a.format, a.threads)

    th = {"psnr_min": a.psnr_min, "ssim_min": a.ssim_min, "iou_min": a.iou_min}
    r = tilecmp.compare_dirs(os.path.join(a.out, "tiles-legacy"), os.path.join(a.out, "tiles-ddbt"), th,
                             diff_dir=os.path.join(a.out, "diff"))
    with open(os.path.join(a.out, "tilecmp.json"), "w") as f:
        json.dump(tilecmp._json_safe(r), f, indent=1)
    s = r["summary"]
    rl = leg["round_results"][0]
    rn = new["round_results"][0]
    print(json.dumps({"dataset": a.dataset, "scenario": a.scenario, "legacy_path": legacy_path, "ddbt_path": ddbt_path,
                      "requests": len(reqs), "legacy_failures": rl["failures"], "ddbt_failures": rn["failures"],
                      "legacy_errors": rl.get("errors"), "ddbt_errors": rn.get("errors"),
                      "legacy_set_hash": leg.get("set_hash"), "ddbt_set_hash": new.get("set_hash"),
                      "summary": tilecmp._json_safe(s)}, indent=1))


if __name__ == "__main__":
    main()
