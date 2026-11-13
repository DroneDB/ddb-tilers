#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Quick regression guard (driven by scripts/bench-guard.sh).

Modes
  timed       run.py on the smoke suite (one backend, --reps 3), then each case of
              bench/guard/baseline.json is checked: p50/p99 and build wall within
              thresholds.time_ratio_max, tiles/s within 1/time_ratio_max, peak RSS within
              thresholds.rss_ratio_max, no more failures, and the tile set hash unchanged
              (thresholds.hash_must_match).
  cachegrind  deterministic: `valgrind --tool=cachegrind --cache-sim=no` runs ddbt-bench tile
              (1 thread, 1 round) on small fixed request sets; the total instruction count
              (Ir) must stay within thresholds.instr_ratio_max, and the set hash must match.
  --update    write the measured values as the new reference into baseline.json.

Usage: guard.py [--mode timed|cachegrind|both] [--backend SPEC] [--update] [--baseline FILE]
Exit code 0: pass, 1: regression, 2: no reference for this backend (run with --update).
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common  # noqa: E402
import requests as reqgen  # noqa: E402
import run as runner  # noqa: E402

BASELINE = os.path.join(common.GUARD_DIR, "baseline.json")
DEFAULT_THRESHOLDS = {"time_ratio_max": 1.10, "rss_ratio_max": 1.10, "instr_ratio_max": 1.01,
                      "hash_must_match": True}
# cachegrind sets: item, scenario, sweep cap per zoom
CG_SETS = [("R1", "sweep", 8), ("R3", "sweep", 4), ("P2", "sweep", 4)]


def load_baseline(path):
    try:
        return common.read_json(path)
    except (OSError, ValueError):
        return {"schema": "ddbt-guard/1", "thresholds": dict(DEFAULT_THRESHOLDS), "references": {}}


def timed(args, backend):
    run_id = "guard-%s-%s" % (common.utc_stamp(), common.git_sha(short=True))
    argv = ["--suite", "smoke", "--backend", backend.spec, "--reps", str(args.reps), "--run-id", run_id,
            "--no-summary-copy", "--gate-policy", args.gate_policy]
    if args.driver:
        argv += ["--driver", args.driver]
    rc = runner.main(argv)
    if rc not in (0,):
        raise SystemExit("guard: run.py failed (%d)" % rc)
    run = common.read_json(os.path.join(common.RESULTS_ROOT, run_id, "run.json"))
    values = {}
    for c in run["cases"]:
        s = c["summary"]

        def med(k):
            return (s.get(k) or {}).get("median")

        if c["kind"] == "tile":
            values[c["id"]] = {"p50_ms": med("p50_ms"), "p99_ms": med("p99_ms"), "tiles_per_s": med("tiles_per_s"),
                               "rss_peak_kb": med("rss_peak_kb"), "failures": med("failures"),
                               "set_hashes": s.get("set_hashes")}
        else:
            values[c["id"]] = {"wall_s": med("wall_s"), "tree_rss_peak_kb": med("tree_rss_peak_kb")}
    return values, run_id


def cachegrind(args, backend):
    items, _ = common.load_manifest()
    driver = common.find_driver(args.driver)
    values = {}
    work = tempfile.mkdtemp(prefix="guard-cg-", dir=os.path.join(common.WORK, "tmp"))
    try:
        for iid, scen, cap in CG_SETS:
            item = items.get(iid)
            if not item:
                continue
            fp, minz, maxz = common.dataset_geometry(item)
            path = item["path"]
            if item.get("kind") == "pointcloud" and not (item.get("probe") or {}).get("copc"):
                continue
            reqs = reqgen.sweep(path, fp, minz, maxz, 256, "png", cap)
            rf = os.path.join(work, "%s.jsonl" % iid)
            reqgen.write_jsonl(reqs, rf)
            home = os.path.join(work, "home-" + iid)
            os.makedirs(home)
            env = common.baseline_env() if backend.kind == "legacy" else dict(os.environ)
            env["DDB_HOME"] = home
            env["GDAL_PAM_ENABLED"] = "NO"
            base_cmd = [driver, "tile", "--backend", backend.kind, "--lib", backend.lib, "--requests", rf,
                        "--threads", "1", "--rounds", "1", "--hashes"]
            # warm DDB_HOME (and convert the input if needed) outside valgrind
            subprocess.run(base_cmd + ["--out", os.path.join(work, "warm.json")], env=env, check=True,
                           capture_output=True)
            def ir_of(extra, tag):
                out_cg = os.path.join(work, "%s-%s.cg" % (iid, tag))
                out_json = os.path.join(work, "%s-%s.json" % (iid, tag))
                p = subprocess.run(["valgrind", "--tool=cachegrind", "--cache-sim=no",
                                    "--cachegrind-out-file=" + out_cg] + base_cmd + extra + ["--out", out_json],
                                   env=env, capture_output=True, text=True)
                if p.returncode != 0:
                    raise SystemExit("guard: valgrind failed on %s: %s" % (iid, p.stderr[-1500:]))
                ir = None
                with open(out_cg) as f:
                    for line in f:
                        m = re.match(r"summary:\s+(\d+)", line)
                        if m:
                            ir = int(m.group(1))
                return ir, common.read_json(out_json)

            # Net instructions of the tiles: the full set minus a run of its first request only
            # (process start, library load and init, first-request setup cancel out).
            ir_full, res = ir_of([], "full")
            ir_one, _ = ir_of(["--limit", "1"], "one")
            ir = ir_full - ir_one if ir_full is not None and ir_one is not None else None
            values["cachegrind/%s/%s" % (iid, scen)] = {"instructions": ir, "instructions_total": ir_full,
                                                        "set_hashes": [res.get("set_hash")],
                                                        "failures": res["round_results"][-1]["failures"],
                                                        "n_requests": res["n_requests"]}
            common.log("cachegrind %s: %d requests, net Ir %s (total %s)" % (iid, res["n_requests"], ir, ir_full))
    finally:
        shutil.rmtree(work, ignore_errors=True)
    return values


def check(ref, cur, th):
    problems = []
    for cid, r in ref.items():
        c = cur.get(cid)
        if c is None:
            problems.append("%s: missing in this run" % cid)
            continue

        def ratio(k):
            if r.get(k) in (None, 0) or c.get(k) is None:
                return None
            return c[k] / r[k]

        for k in ("p50_ms", "p99_ms", "wall_s"):
            q = ratio(k)
            if q is not None and q > th["time_ratio_max"]:
                problems.append("%s: %s x%.3f > %.2f" % (cid, k, q, th["time_ratio_max"]))
        q = ratio("tiles_per_s")
        if q is not None and q < 1 / th["time_ratio_max"]:
            problems.append("%s: tiles_per_s x%.3f" % (cid, q))
        for k in ("rss_peak_kb", "tree_rss_peak_kb"):
            q = ratio(k)
            if q is not None and q > th["rss_ratio_max"]:
                problems.append("%s: %s x%.3f > %.2f" % (cid, k, q, th["rss_ratio_max"]))
        q = ratio("instructions")
        if q is not None and q > th["instr_ratio_max"]:
            problems.append("%s: instructions x%.4f > %.3f" % (cid, q, th["instr_ratio_max"]))
        if (c.get("failures") or 0) > (r.get("failures") or 0):
            problems.append("%s: failures %s > %s" % (cid, c.get("failures"), r.get("failures")))
        if th.get("hash_must_match") and r.get("set_hashes") and c.get("set_hashes") != r.get("set_hashes"):
            problems.append("%s: tile set hash %s != %s" % (cid, c.get("set_hashes"), r.get("set_hashes")))
    return problems


class GuardBackend:
    def __init__(self, spec):
        self.spec = spec
        b = runner.parse_backend(spec)
        self.kind, self.lib, self.name = b.kind, b.lib, b.name
        if not self.lib or not os.path.isfile(self.lib):
            raise SystemExit("guard: backend library not found: %s" % self.lib)
        self.key = "%s:%s" % (self.kind, os.path.basename(self.lib))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=("timed", "cachegrind", "both"), default="timed")
    ap.add_argument("--backend", default="ddbt", help="backend spec as in run.py (default: ddbt)")
    ap.add_argument("--baseline", default=BASELINE)
    ap.add_argument("--update", action="store_true")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--driver")
    ap.add_argument("--gate-policy", default="warn", choices=("strict", "warn", "off"))
    a = ap.parse_args(argv)
    backend = GuardBackend(a.backend)
    base = load_baseline(a.baseline)
    th = dict(DEFAULT_THRESHOLDS, **base.get("thresholds", {}))
    cur, meta = {}, {}
    if a.mode in ("timed", "both"):
        v, run_id = timed(a, backend)
        cur.update(v)
        meta["timed_run"] = run_id
    if a.mode in ("cachegrind", "both"):
        cur.update(cachegrind(a, backend))
    refs = base.setdefault("references", {})
    if a.update:
        entry = refs.setdefault(backend.key, {"cases": {}})
        entry["cases"].update(cur)
        entry.update({"sha": common.git_sha(), "updated": common.utc_iso(), **meta})
        base["thresholds"] = th
        common.write_json(a.baseline, base)
        print("guard: reference updated for %s (%d cases) in %s" % (backend.key, len(cur), a.baseline))
        return 0
    ref = (refs.get(backend.key) or {}).get("cases")
    if not ref:
        print("guard: no reference for %s in %s: run with --update first" % (backend.key, a.baseline))
        return 2
    ref = {k: v for k, v in ref.items()
           if (a.mode == "both") or (a.mode == "cachegrind") == k.startswith("cachegrind/")}
    problems = check(ref, cur, th)
    if problems:
        print("guard: FAIL (%d)" % len(problems))
        for p in problems:
            print("  " + p)
        return 1
    print("guard: pass (%d cases, backend %s)" % (len(ref), backend.key))
    return 0


if __name__ == "__main__":
    sys.exit(main())
