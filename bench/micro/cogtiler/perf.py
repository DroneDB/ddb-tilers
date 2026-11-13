#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Preliminary tile performance of the COG tiler against libddb 2.4.1, pinned to chosen CPUs
(CCD1 while baseline benchmarks use CCD0): ddbt-bench tile with both backends, interleaved
ABBA per repetition, warm (inputs in the page cache, warmup round, measured rounds, one process)
and cold (inputs and libddb artefacts evicted, fresh process, one round).

Not the official A/B (bench/harness/run.py on a quiet host): no quiet / thermal gates. Prints a
table of medians over the repetitions (p50 / p99 of successful requests, tiles/s, peak RSS).

  python3 bench/micro/cogtiler/perf.py --dataset R6 --legacy-path P --scenario pan --threads 1,8 \
      --ddbt-lib /mnt/data/ddb-tilers-work/build-cogtile/lib/libddbt.so --out DIR
"""
import argparse
import glob
import json
import os
import statistics
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HARNESS = os.path.normpath(os.path.join(HERE, "..", "..", "harness"))
sys.path.insert(0, HARNESS)
import common  # noqa: E402
import requests as reqgen  # noqa: E402

PARITY_HOME = os.path.join(common.HOME_ROOT, "cogtiler-parity")
PINS = {1: "9", 8: "8-15"}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--label")
    ap.add_argument("--legacy-path")
    ap.add_argument("--ddbt-path")
    ap.add_argument("--scenario", default="pan")
    ap.add_argument("--min-requests", type=int, default=1000)
    ap.add_argument("--cap", type=int, default=128)
    ap.add_argument("--threads", default="1,8")
    ap.add_argument("--caches", default="warm,cold")
    ap.add_argument("--format", default="png")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--rounds", type=int, default=3)
    ap.add_argument("--ddbt-lib", default=common.default_ddbt_lib())
    ap.add_argument("--ddbt-env", action="append")
    ap.add_argument("--pin-1", default=PINS[1])
    ap.add_argument("--pin-8", default=PINS[8])
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    driver = common.find_driver(None)
    items = {i["id"]: i for i in common.read_json(common.MANIFEST)["items"]}
    item = items[a.dataset]
    label = a.label or a.dataset
    fp, minz, maxz = common.dataset_geometry(item)
    legacy_path = a.legacy_path or item["path"]
    ddbt_path = a.ddbt_path or legacy_path
    os.makedirs(a.out, exist_ok=True)
    size = 512 if a.scenario == "retina" else 256
    reqs = reqgen.generate(a.scenario, legacy_path, fp, minz, maxz, a.format, 1, a.cap, 200, a.min_requests)
    rl = os.path.join(a.out, "req-legacy.jsonl")
    rd = os.path.join(a.out, "req-ddbt.jsonl")
    reqgen.write_jsonl(reqs, rl)
    with open(rl) as f, open(rd, "w") as g:
        for line in f:
            r = json.loads(line)
            r["path"] = ddbt_path
            g.write(json.dumps(r) + "\n")
    home = os.path.join(PARITY_HOME, label)
    os.makedirs(home, exist_ok=True)
    lenv = common.baseline_env()
    lenv["GDAL_PAM_ENABLED"] = "NO"
    lenv["DDB_HOME"] = home
    denv = {k: v for k, v in os.environ.items() if not k.startswith("DDB_")}
    for kv in a.ddbt_env or []:
        k, _, v = kv.partition("=")
        denv[k] = v
    backends = [("legacy", common.baseline_lib(), rl, lenv), ("ddbt", a.ddbt_lib, rd, denv)]
    arts = glob.glob(os.path.join(home, ".ddb", "artefacts", "*"))
    rows = []
    for th in [int(t) for t in a.threads.split(",")]:
        pin = a.pin_1 if th == 1 else (a.pin_8 if th == 8 else a.pin_8)
        for cache in a.caches.split(","):
            res = {b[0]: [] for b in backends}
            for rep in range(a.reps):
                order = backends if rep % 2 == 0 else list(reversed(backends))
                for name, lib, req, env in order:
                    paths = [legacy_path, ddbt_path] + arts
                    action = "evict" if cache == "cold" else "warm"
                    subprocess.run([driver, "pagecache", action] + [p for p in paths if os.path.exists(p)],
                                   capture_output=True)
                    out = os.path.join(a.out, "%s-%dT-%s-%d.json" % (name, th, cache, rep))
                    cmd = ["taskset", "-c", pin, driver, "tile", "--backend", name, "--lib", lib, "--requests", req,
                           "--threads", str(th), "--pin", pin, "--warmup-rounds", "0" if cache == "cold" else "1",
                           "--rounds", "1" if cache == "cold" else str(a.rounds), "--format", a.format, "--size",
                           str(size), "--out", out]
                    p = subprocess.run(cmd, env=env, capture_output=True, text=True)
                    if p.returncode != 0:
                        raise SystemExit("%s failed: %s" % (name, p.stderr[-2000:]))
                    r = common.read_json(out)
                    meas = [x for x in r["round_results"] if not x.get("warmup")]
                    res[name].append({
                        "p50": statistics.median([x["latency_ok_ms"]["p50"] for x in meas]),
                        "p99": statistics.median([x["latency_ok_ms"]["p99"] for x in meas]),
                        "mean": statistics.median([x["latency_ok_ms"]["mean"] for x in meas]),
                        "tps": statistics.median([x["tiles_per_s"] for x in meas]),
                        "rss": max(x["rss_kb"]["peak"] for x in meas),
                        "fail": sum(x["failures"] for x in meas)})
            for name in res:
                v = res[name]
                rows.append({"dataset": a.dataset, "scenario": a.scenario, "requests": len(reqs), "threads": th,
                             "cache": cache, "backend": name,
                             **{k: statistics.median([x[k] for x in v]) for k in ("p50", "p99", "mean", "tps", "rss")},
                             "failures": sum(x["fail"] for x in v)})
    with open(os.path.join(a.out, "perf.json"), "w") as f:
        json.dump(rows, f, indent=1)
    print("| dataset | scenario | T | cache | backend | p50 ms | p99 ms | mean ms | tiles/s | peak RSS MB |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        print("| %s | %s (%d) | %d | %s | %s | %.2f | %.2f | %.2f | %.0f | %.0f |" % (
            r["dataset"], r["scenario"], r["requests"], r["threads"], r["cache"], r["backend"], r["p50"], r["p99"],
            r["mean"], r["tps"], r["rss"] / 1024.0))


if __name__ == "__main__":
    main()
