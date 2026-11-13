#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Append one row to the measurement log (default $DDBT_WORK/BENCHMARKS.md; append-only: existing
rows are never touched).

  log.py --run RUN_JSON --change "what changed" [--metric p50_ms] [--cases REGEX]
         [--verdict kept|rejected|neutral|baseline] [--commit SHA] [--file BENCHMARKS.md] [--dry-run]
  log.py --change "baseline" --result "text" --suite std --verdict baseline   (free-form row)

Result column: for --metric, the geometric mean of the head/base cost ratios over the compared
cases, the range of their 95% CIs and the verdict counts. Verdict (unless given): "kept" when
the metric is better in every compared case and no metric is worse anywhere, "rejected" when
a metric is worse anywhere, else "neutral".
"""
import argparse
import datetime
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common  # noqa: E402

HEADER_RE = re.compile(r"^\|\s*#\s*\|")


def summarize(run, metric, case_re=None):
    cmps = [c for c in run.get("comparisons", []) if not case_re or re.search(case_re, c["id"])]
    target = [m for c in cmps for m in c["metrics"] if m["metric"] == metric and m["cost_ratio"] == m["cost_ratio"]]
    worse_any = [(c["id"], m["metric"]) for c in cmps for m in c["metrics"] if m["verdict"] == "worse"]
    if not target:
        return "no comparison for %s" % metric, ("rejected" if worse_any else "neutral")
    gm = math.exp(sum(math.log(m["cost_ratio"]) for m in target) / len(target))
    lo = min(m["ci95"][0] for m in target)
    hi = max(m["ci95"][1] for m in target)
    counts = {v: sum(1 for m in target if m["verdict"] == v) for v in ("better", "neutral", "worse")}
    text = "%s x%.3f (geomean of %d cases, CIs %.3f-%.3f; better %d, neutral %d, worse %d)" % (
        metric, gm, len(target), lo, hi, counts["better"], counts["neutral"], counts["worse"])
    if worse_any:
        text += "; worse elsewhere: " + ", ".join("%s %s" % w for w in worse_any[:3])
        verdict = "rejected"
    elif counts["better"] == len(target):
        verdict = "kept"
    else:
        verdict = "neutral"
    return text, verdict


def next_number(lines):
    nums = []
    in_table = False
    for line in lines:
        if HEADER_RE.match(line):
            in_table = True
            continue
        if in_table and line.startswith("|"):
            cell = line.strip("|").split("|")[0].strip()
            if cell.isdigit():
                nums.append(int(cell))
    return (max(nums) + 1) if nums else 0


def cell(s):
    return str(s).replace("|", "\\|").replace("\n", " ")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run")
    ap.add_argument("--change", required=True)
    ap.add_argument("--metric", default="p50_ms")
    ap.add_argument("--cases", help="regex on case ids")
    ap.add_argument("--result")
    ap.add_argument("--verdict")
    ap.add_argument("--suite")
    ap.add_argument("--commit")
    ap.add_argument("--file", default=os.path.join(common.WORK, "BENCHMARKS.md"))
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)

    with open(a.file) as f:
        text = f.read()
    lines = text.splitlines()
    if not any(HEADER_RE.match(l) for l in lines):
        raise SystemExit("%s has no benchmark table" % a.file)
    run = common.read_json(a.run) if a.run else None
    result, verdict = (a.result, None)
    if run and not a.result:
        result, verdict = summarize(run, a.metric, a.cases)
    verdict = a.verdict or verdict or "-"
    suite = a.suite or (run or {}).get("suite", {}).get("name", "-")
    commit = a.commit or common.git_sha(short=True)
    runs = (run or {}).get("run_id", "-")
    date = datetime.date.today().isoformat()
    row = "| %d | %s | %s | %s | %s | %s | %s | %s |" % (
        next_number(lines), date, cell(commit), cell(a.change), cell(suite), cell(result or "-"), cell(verdict),
        cell(runs))
    if a.dry_run:
        print(row)
        return 0
    with open(a.file, "a") as f:
        if not text.endswith("\n"):
            f.write("\n")
        f.write(row + "\n")
    print(row)
    return 0


if __name__ == "__main__":
    sys.exit(main())
