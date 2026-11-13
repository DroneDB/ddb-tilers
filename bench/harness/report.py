#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Markdown report of a run.json (schema ddbt-bench/1).

Usage: report.py RUN_JSON [--out report.md]
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common  # noqa: E402


def f(v, nd=2):
    if v is None:
        return "-"
    if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
        return "-"
    if isinstance(v, int):
        return str(v)
    return ("%%.%df" % nd) % v


def pct(v):
    return "-" if v is None or (isinstance(v, float) and math.isnan(v)) else "%.1f%%" % (v * 100)


def med(summary, k):
    s = summary.get(k) or {}
    return s.get("median")


def ci_text(c):
    lo, hi = c["ci95"]
    return "%s [%s, %s]" % (f(c["cost_ratio"], 3), f(lo, 3), f(hi, 3))


def machine_lines(run):
    m = run.get("machine", {})
    cpu = m.get("cpu", {})
    power = m.get("power", {})
    temps = m.get("temperatures", {})
    repos = m.get("repos", {})
    lines = [
        "- Machine: %s, %s (%s, %s logical CPUs), kernel %s" % (
            m.get("hostname", "?"), cpu.get("model", "?"), cpu.get("isa_level", "?"), cpu.get("logical_cpus", "?"),
            (m.get("kernel") or {}).get("release", "?")),
        "- Power: governor %s, EPP %s, boost %s, SMT %s; THP %s" % (
            ",".join(power.get("governors", [])), ",".join(power.get("energy_performance_preference", [])),
            power.get("boost"), power.get("smt_active"), (m.get("thp") or {}).get("enabled")),
        "- Temperatures at start: %s" % ", ".join("%s %.1f C" % (k, v) for k, v in temps.items()),
    ]
    for name, r in repos.items():
        lines.append("- %s: `%s`%s" % (name, (r.get("sha") or "?")[:12],
                                       " (%d dirty files)" % r["dirty_files"] if r.get("dirty_files") else ""))
    b = run.get("baseline", {})
    lines.append("- Baseline: `%s` (MANIFEST sha256 `%s`)" % (b.get("dir"), (b.get("manifest_sha256") or "?")[:16]))
    for name, be in (run.get("build", {}).get("backends") or {}).items():
        lines.append("- Backend `%s`: %s `%s`%s" % (name, be.get("kind"), be.get("lib"),
                                                     " sha256 `%s`" % be["lib_sha256"][:16] if be.get("lib_sha256")
                                                     else ""))
    return lines


def render(run):
    out = ["# Benchmark run %s" % run.get("run_id"), ""]
    s = run.get("suite", {})
    out.append("Suite **%s**, status **%s**, created %s." % (s.get("name"), run.get("status"), run.get("created")))
    out.append("")
    out += machine_lines(run)
    g = (run.get("guards") or {}).get("readonly", {})
    out.append("- Read-only guard: %s (%s paths)" % ("ok" if g.get("ok") else "**VIOLATION**", g.get("checked_paths")))
    iso = run.get("isolation")
    if iso:
        inv = iso.get("other_cpus_util_during_invocations", {})
        out.append("- Isolation: **%s**, gate CPUs %s; load of the other CPUs during invocations mean %s%% (max %s%%, %s samples); "
                   "loadavg %s -> %s; Tctl %s" % (
                       iso.get("mode"), iso.get("gate_cpus"), inv.get("mean_pct"), inv.get("max_pct"), inv.get("samples"),
                       iso.get("loadavg_start"), iso.get("loadavg_end"), iso.get("tctl_c")))
    noise = run.get("noise")
    out.append("- Noise file: %s" % ("present (theta from A/A)" if noise else "absent (theta = 3%)"))
    out.append("")

    warnings = []
    for c in run.get("cases", []):
        qg = c.get("quiet_gate") or {}
        if qg.get("ok") is False:
            warnings.append("%s / %s: quiet gate failed (load1 %s, busy %s)" % (
                c["id"], c["backend"], qg.get("load1"), ", ".join("%s %s%%" % (b["comm"], b["cpu_pct"])
                                                                  for b in qg.get("busy", []))))
        for r in c.get("reps", []):
            if not r.get("valid"):
                warnings.append("%s / %s rep %s: invalid (%s)" % (c["id"], c["backend"], r["rep"],
                                                                   r.get("invalid_reason")))
            th = r.get("thermal") or {}
            if th.get("ok") is False:
                warnings.append("%s / %s rep %s: thermal gate %s C" % (c["id"], c["backend"], r["rep"],
                                                                      th.get("temp_c")))
    for e in run.get("events", []):
        warnings.append("%s: %s" % (e.get("case"), e.get("error")))
    if warnings:
        out += ["## Warnings", ""] + ["- " + w for w in warnings[:60]]
        if len(warnings) > 60:
            out.append("- ... %d more" % (len(warnings) - 60))
        out.append("")

    tiles = [c for c in run.get("cases", []) if c["kind"] == "tile"]
    if tiles:
        out += ["## Tiles", "",
                "| case | backend | reqs | p50 ms | p95 ms | p99 ms | p50 ok ms | tiles/s | RSS MB | fail | CV p50 | reps | prepare s |",
                "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for c in tiles:
            sm = c["summary"]
            rss = med(sm, "rss_peak_kb")
            cvp = (sm.get("p50_ms") or {}).get("cv")
            pr = c.get("prepare") or {}
            prep = pr.get("wall_s")
            if pr.get("seeded"):  # the conversion was measured in the first case of this item (seed_prepare)
                prep = "seeded (%s)" % f((pr.get("seed_prepare") or {}).get("wall_s"))
            out.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s/%s | %s |" % (
                c["id"], c["backend"], c["params"].get("n_requests"), f(med(sm, "p50_ms")), f(med(sm, "p95_ms")),
                f(med(sm, "p99_ms")), f(med(sm, "p50_ok_ms")), f(med(sm, "tiles_per_s"), 0), f(rss / 1024 if rss else None, 0),
                f(med(sm, "failures"), 0), pct(cvp),
                sm.get("valid_reps"), sm.get("reps"), prep if isinstance(prep, str) else f(prep)))
        out.append("")
    builds = [c for c in run.get("cases", []) if c["kind"] == "build"]
    if builds:
        out += ["## Builds", "",
                "| case | backend | wall s | CPU s | tree RSS MB | read MB | write MB | output MB | CV wall | reps | valid output |",
                "|---|---|---|---|---|---|---|---|---|---|---|"]
        for c in builds:
            sm = c["summary"]
            v = [r.get("validation", {}).get("ok") for r in c["reps"] if "validation" in r]
            cvw = (sm.get("wall_s") or {}).get("cv")

            def mb(k):
                x = med(sm, k)
                return f(x / (1024 if k.endswith("_kb") else 1024 * 1024) if x is not None else None, 1)

            out.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s/%s | %s |" % (
                c["id"], c["backend"], f(med(sm, "wall_s")), f(med(sm, "cpu_s")), mb("tree_rss_peak_kb"),
                mb("read_bytes"), mb("write_bytes"), mb("output_bytes"),
                pct(cvw), sm.get("valid_reps"),
                sm.get("reps"), "yes" if v and all(v) else ("no" if v else "-")))
        out.append("")

    cmps = run.get("comparisons", [])
    if cmps:
        out += ["## Comparisons (cost ratio head/base, < 1 = head better; 95% bootstrap CI)", "",
                "| case | head vs base | metric | ratio [95% CI] | theta | verdict |", "|---|---|---|---|---|---|"]
        for c in cmps:
            for m in c["metrics"]:
                out.append("| %s | %s vs %s | %s | %s | %s | %s |" % (
                    c["id"], c["head"], c["base"], m["metric"], ci_text(m), f(m["theta"] * 100, 1) + "%",
                    m["label"]))
            if c.get("kind") == "tile":
                out.append("| %s | %s vs %s | identical tiles | %s | | |" % (
                    c["id"], c["head"], c["base"], "yes" if c.get("same_tiles") else "no"))
        out.append("")
    tc = [c for c in run.get("cases", []) if c.get("tilecmp_vs_base")]
    if tc:
        out += ["## Tile comparison vs base", "", "| case | backend | tiles | pass | fail | exact | min PSNR | min SSIM | min IoU |",
                "|---|---|---|---|---|---|---|---|---|"]
        for c in tc:
            t = c["tilecmp_vs_base"]
            out.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
                c["id"], c["backend"], t["tiles"], t["pass"], t["fail"], t["exact"], t.get("psnr_min"),
                f(t.get("ssim_min"), 4), f(t.get("alpha_iou_min"), 4)))
        out.append("")
    return "\n".join(out) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_json")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    md = render(common.read_json(a.run_json))
    if a.out:
        with open(a.out, "w") as fh:
            fh.write(md)
    else:
        sys.stdout.write(md)
    return 0


if __name__ == "__main__":
    sys.exit(main())
