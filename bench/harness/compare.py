#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""A/B comparison tables, and the A/A noise file.

  compare.py RUN_JSON                          table of the comparisons inside one run
                                               (interleaved backends: the preferred A/B)
  compare.py --base RUN_A --head RUN_B         cross-run comparison of the cases with the same id
                                               (not interleaved: drift between runs is included)
     [--base-backend NAME] [--head-backend NAME]
  compare.py RUN_JSON --write-noise FILE       A/A run (run.py --aa): per-metric spread -> noise.json
Options: --json OUT (machine-readable result), --fail-on-worse (exit 1 when a verdict is "worse").
"""
import argparse
import datetime
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common  # noqa: E402
import report  # noqa: E402
import stats  # noqa: E402


def rep_values(case, metric):
    return [r["metrics"].get(metric) for r in case.get("reps", []) if r.get("valid") and
            r["metrics"].get(metric) is not None]


def cross_run(run_a, run_b, base_backend=None, head_backend=None, noise=None):
    def index(run, backend):
        out = {}
        for c in run["cases"]:
            if backend and c["backend"] != backend:
                continue
            out.setdefault(c["id"], c)
        return out

    ia, ib = index(run_a, base_backend), index(run_b, head_backend)
    cmps = []
    for cid in sorted(set(ia) & set(ib)):
        a, b = ia[cid], ib[cid]
        metrics = ["p50_ms", "p95_ms", "p99_ms", "tiles_per_s", "rss_peak_kb"] if a["kind"] == "tile" else \
            ["wall_s", "cpu_s", "tree_rss_peak_kb", "write_bytes"]
        if any("instructions" in r["metrics"] for r in a.get("reps", [])):
            metrics.append("instructions")
        rows = []
        for m in metrics:
            bv, hv = rep_values(a, m), rep_values(b, m)
            if bv and hv:
                rows.append(stats.compare(bv, hv, m, noise))
        c = {"id": cid, "kind": a["kind"], "base": "%s@%s" % (a["backend"], run_a["run_id"]),
             "head": "%s@%s" % (b["backend"], run_b["run_id"]), "metrics": rows}
        if a["kind"] == "tile":
            c["same_tiles"] = a["summary"].get("set_hashes") == b["summary"].get("set_hashes")
        cmps.append(c)
    return cmps


def table(cmps):
    lines = ["| case | metric | base median | head median | ratio [95% CI] | theta | verdict |",
             "|---|---|---|---|---|---|---|"]
    for c in cmps:
        for m in c["metrics"]:
            lines.append("| %s | %s | %s | %s | %s | %s | %s |" % (
                c["id"], m["metric"], report.f(m["base"].get("median"), 3), report.f(m["head"].get("median"), 3),
                report.ci_text(m), report.f(m["theta"] * 100, 1) + "%", m["label"]))
        if "same_tiles" in c:
            lines.append("| %s | identical tiles | | | %s | | |" % (c["id"], "yes" if c["same_tiles"] else "no"))
    return "\n".join(lines) + "\n"


def write_noise(run, path, extra_runs=()):
    """Per-metric A/A spread from the comparisons of an A/A run (both sides the same build); extra_runs are
    further A/A runs whose cases are pooled."""
    by_id = {}
    for r_ in (run, *extra_runs):
        for c in r_["cases"]:
            by_id.setdefault(c["id"], []).append(c)
    spreads = {}
    for cid, cs in by_id.items():
        if len(cs) < 2:
            continue
        a, b = cs[0], cs[1]
        metrics = {k for r in a.get("reps", []) for k, v in r["metrics"].items() if isinstance(v, (int, float))}
        for m in metrics:
            av, bv = rep_values(a, m), rep_values(b, m)
            if len(av) >= 2 and len(bv) >= 2:
                s = stats.aa_spread(av, bv)
                if s == s:  # not NaN
                    spreads.setdefault(m, []).append(s)
    out = {"schema": "ddbt-noise/1", "theta_min": stats.THETA_MIN,
           "generated": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
           "source_run": ", ".join(r_.get("run_id") for r_ in (run, *extra_runs)),
           "rule": "theta = max(theta_min, 2 x aa_spread); aa_spread = 90th percentile over the A/A cases of "
                   "max(CI upper - 1, 1 - CI lower)",
           "metrics": {}}
    allv = []
    for m, v in sorted(spreads.items()):
        v = sorted(v)
        p90 = v[min(len(v) - 1, int(round(0.9 * (len(v) - 1))))]
        out["metrics"][m] = {"aa_spread": round(p90, 5), "max": round(v[-1], 5), "cases": len(v)}
        allv.append(p90)
    if allv:
        out["metrics"]["default"] = {"aa_spread": round(sorted(allv)[len(allv) // 2], 5),
                                     "note": "median of the metric spreads, for metrics not listed"}
    common.write_json(path, out)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_json", nargs="?")
    ap.add_argument("--base")
    ap.add_argument("--head")
    ap.add_argument("--base-backend")
    ap.add_argument("--head-backend")
    ap.add_argument("--write-noise")
    ap.add_argument("--also", action="append", default=[], help="more A/A run.json files pooled into --write-noise")
    ap.add_argument("--json")
    ap.add_argument("--fail-on-worse", action="store_true")
    a = ap.parse_args(argv)
    noise = stats.load_noise()
    if a.base and a.head:
        cmps = cross_run(common.read_json(a.base), common.read_json(a.head), a.base_backend, a.head_backend, noise)
    elif a.run_json:
        run = common.read_json(a.run_json)
        if a.write_noise:
            out = write_noise(run, a.write_noise, [common.read_json(x) for x in a.also])
            print(json.dumps(out, indent=1))
            return 0
        cmps = run.get("comparisons", [])
    else:
        ap.error("RUN_JSON or --base/--head is required")
    sys.stdout.write(table(cmps))
    if a.json:
        common.write_json(a.json, {"schema": "ddbt-compare/1", "comparisons": cmps})
    worse = [(c["id"], m["metric"]) for c in cmps for m in c["metrics"] if m["verdict"] == "worse"]
    if worse:
        print("\nworse: " + ", ".join("%s %s" % w for w in worse), file=sys.stderr)
    return 1 if (a.fail_on_worse and worse) else 0


if __name__ == "__main__":
    sys.exit(main())
