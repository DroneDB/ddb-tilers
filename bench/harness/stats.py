#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Statistics of the benchmark harness.

* median, MAD x 1.4826 (consistent with the standard deviation for normal data), CV
* bootstrap 95% CI of the ratio of medians head/base (independent resampling of both sides,
  10 000 resamples, seeded: the same data always gives the same interval)
* adoption rule, on the "cost" ratio (head/base for lower-is-better metrics such as time and
  RSS, base/head for higher-is-better metrics such as tiles/s; < 1 means head is better):
      better (faster)  if CI upper <= 1 - theta
      worse  (slower)  if CI lower >= 1 + theta
      neutral          otherwise
  theta = max(3%, 2 x A/A spread of the metric), the spread read from bench/guard/noise.json.
"""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common  # noqa: E402

THETA_MIN = 0.03
MAD_SCALE = 1.4826

# metric -> higher is better
HIGHER_IS_BETTER = {"tiles_per_s": True, "throughput": True}


def higher_is_better(metric):
    return HIGHER_IS_BETTER.get(metric, False) or metric.endswith("_per_s")


def median(xs):
    xs = [x for x in xs if x is not None]
    return float(np.median(xs)) if xs else float("nan")


def mad(xs, scale=MAD_SCALE):
    xs = np.asarray([x for x in xs if x is not None], dtype=float)
    if xs.size == 0:
        return float("nan")
    return float(scale * np.median(np.abs(xs - np.median(xs))))


def cv(xs):
    """Coefficient of variation (sample standard deviation / mean)."""
    xs = np.asarray([x for x in xs if x is not None], dtype=float)
    if xs.size < 2 or xs.mean() == 0:
        return float("nan") if xs.size < 2 else float("inf")
    return float(xs.std(ddof=1) / abs(xs.mean()))


def summarize(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return {"n": 0}
    return {"n": len(xs), "median": median(xs), "mad": mad(xs), "cv": cv(xs),
            "min": float(min(xs)), "max": float(max(xs)), "mean": float(np.mean(xs))}


def bootstrap_ratio_ci(base, head, n_resamples=10000, seed=20261111, alpha=0.05):
    """(ratio, lo, hi): ratio of medians head/base and its percentile bootstrap CI."""
    b = np.asarray([x for x in base if x is not None], dtype=float)
    h = np.asarray([x for x in head if x is not None], dtype=float)
    if b.size == 0 or h.size == 0 or np.median(b) == 0:
        return float("nan"), float("nan"), float("nan")
    ratio = float(np.median(h) / np.median(b))
    rng = np.random.default_rng(seed)
    bm = np.median(b[rng.integers(0, b.size, size=(n_resamples, b.size))], axis=1)
    hm = np.median(h[rng.integers(0, h.size, size=(n_resamples, h.size))], axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        r = hm / bm
    r = r[np.isfinite(r)]
    if r.size == 0:
        return ratio, float("nan"), float("nan")
    lo, hi = np.quantile(r, [alpha / 2, 1 - alpha / 2])
    return ratio, float(lo), float(hi)


def load_noise(path=common.NOISE_FILE):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def theta_for(metric, noise=None):
    """max(theta_min, 2 x A/A spread). The spread is looked up for the metric, else "default"."""
    tmin = THETA_MIN
    spread = 0.0
    if noise:
        tmin = float(noise.get("theta_min", THETA_MIN))
        m = noise.get("metrics", {})
        ent = m.get(metric) or m.get("default") or {}
        spread = float(ent.get("aa_spread", 0.0) or 0.0)
    return max(tmin, 2.0 * spread)


def verdict(lo, hi, theta):
    if math.isnan(lo) or math.isnan(hi):
        return "n/a"
    if hi <= 1.0 - theta:
        return "better"
    if lo >= 1.0 + theta:
        return "worse"
    return "neutral"


def compare(base, head, metric, noise=None, n_resamples=10000, seed=20261111):
    """Compare per-repetition values of one metric. Returns ratio/CI on the cost scale."""
    hib = higher_is_better(metric)
    ratio, lo, hi = bootstrap_ratio_ci(head if hib else base, base if hib else head, n_resamples, seed)
    th = theta_for(metric, noise)
    v = verdict(lo, hi, th)
    return {"metric": metric, "higher_is_better": hib, "cost_ratio": ratio, "ci95": [lo, hi], "theta": th,
            "verdict": v, "label": {"better": "faster" if not metric.startswith("rss") else "lower",
                                     "worse": "slower" if not metric.startswith("rss") else "higher"}.get(v, v),
            "base": summarize(base), "head": summarize(head)}


def aa_spread(a, b, n_resamples=10000, seed=20261111):
    """A/A spread of a metric: how far from 1 the 95% CI of the ratio of two identical setups
    reaches (max(hi - 1, 1 - lo))."""
    _, lo, hi = bootstrap_ratio_ci(a, b, n_resamples, seed)
    if math.isnan(lo):
        return float("nan")
    return max(hi - 1.0, 1.0 - lo, 0.0)


if __name__ == "__main__":
    # tiny CLI: stats.py base.txt head.txt metric  (one value per line)
    if len(sys.argv) != 4:
        print(__doc__)
        sys.exit(2)
    with open(sys.argv[1]) as f:
        base = [float(x) for x in f.read().split()]
    with open(sys.argv[2]) as f:
        head = [float(x) for x in f.read().split()]
    print(json.dumps(compare(base, head, sys.argv[3], load_noise()), indent=1))
