#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Markdown tables of a baseline run (run.json with reps): tiles per scenario, builds, prepare times.

Usage: baseline_table.py RUN_JSON [--backend legacy] [--out tables.md]
Tile rows: p50/p95/p99 ms (median over valid repetitions of the per-repetition medians over rounds), tiles/s, peak RSS,
instructions per tile (whole invocation incl. init and warmup rounds), failures, CV of p50 over the repetitions.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common  # noqa: E402
from report import f, med  # noqa: E402


def mb(kb):
    return None if kb is None else kb / 1024


def tile_rows(run, backend):
    out = {}
    for c in run["cases"]:
        if c["kind"] != "tile" or c["backend"] != backend:
            continue
        sm = c["summary"]
        p = c["params"]
        ipt = med(sm, "instr_per_tile")
        out.setdefault(c["scenario"], []).append([
            c["dataset"], "%s/%d" % (p["format"], p["size"]), "%dT" % p["threads"], p["cache"], str(p.get("n_requests")),
            f(med(sm, "p50_ms"), 2), f(med(sm, "p95_ms"), 2), f(med(sm, "p99_ms"), 2), f(med(sm, "tiles_per_s"), 0),
            f(mb(med(sm, "rss_peak_kb")), 0), f(ipt / 1e6 if ipt else None, 1), f(med(sm, "failures"), 0),
            ("%.1f%%" % (100 * sm["p50_ms"]["cv"])) if sm.get("p50_ms", {}).get("cv") is not None else "-",
            "%s/%s" % (sm.get("valid_reps"), sm.get("reps"))])
    return out


TILE_HDR = ["item", "fmt/size", "thr", "cache", "reqs", "p50 ms", "p95 ms", "p99 ms", "tiles/s", "RSS MB", "Minstr/tile",
            "fail", "CV p50", "reps"]


def md(hdr, rows):
    return "\n".join(["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)] + ["| " + " | ".join(r) + " |" for r in rows])


def build_rows(run, backend):
    rows = []
    for c in run["cases"]:
        if c["kind"] != "build" or c["backend"] != backend:
            continue
        sm = c["summary"]
        v = [r.get("validation", {}).get("ok") for r in c["reps"] if "validation" in r]
        ins = med(sm, "instructions")
        rows.append([c["dataset"], c["scenario"], f(med(sm, "wall_s"), 2),
                     ("%.1f%%" % (100 * sm["wall_s"]["cv"])) if sm.get("wall_s", {}).get("cv") is not None else "-",
                     f(med(sm, "cpu_s"), 1), f(mb(med(sm, "tree_rss_peak_kb")), 0),
                     f((med(sm, "read_bytes") or 0) / 2 ** 20, 0), f((med(sm, "write_bytes") or 0) / 2 ** 20, 0),
                     f((med(sm, "output_bytes") or 0) / 2 ** 20, 1), f(ins / 1e9 if ins else None, 0),
                     "yes" if v and all(v) else ("NO" if v else "-"), "%s/%s" % (sm.get("valid_reps"), sm.get("reps"))])
    return rows


BUILD_HDR = ["item", "scenario", "wall s", "CV", "CPU s", "tree RSS MB", "read MB", "write MB", "output MB", "Ginstr",
             "valid", "reps"]


def prepare_rows(run, backend):
    rows = []
    seen = set()
    for c in run["cases"]:
        if c["kind"] != "tile" or c["backend"] != backend:
            continue
        pr = c.get("prepare") or {}
        if pr.get("converts_input") and not pr.get("seeded") and c["dataset"] not in seen:
            seen.add(c["dataset"])
            rows.append([c["dataset"], f(pr.get("wall_s"), 2), f(pr.get("first_tile_ms") / 1000 if pr.get("first_tile_ms") else None, 2),
                         c["id"]])
    return rows


def render(run, backend):
    out = []
    for scen, rows in tile_rows(run, backend).items():
        out += ["### Tiles: %s" % scen, "", md(TILE_HDR, rows), ""]
    b = build_rows(run, backend)
    if b:
        out += ["### Builds", "", md(BUILD_HDR, b), ""]
    p = prepare_rows(run, backend)
    if p:
        out += ["### Prepare (first tile on a raw raster = COG build, 8 CCD0 cores)", "",
                md(["item", "invocation wall s", "first tile s", "measured in case"], p), ""]
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_json")
    ap.add_argument("--backend", default="legacy")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    text = render(common.read_json(a.run_json), a.backend)
    if a.out:
        with open(a.out, "w") as fh:
            fh.write(text)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
