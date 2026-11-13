# SPDX-License-Identifier: MPL-2.0
import math
import unittest

import numpy as np

from tests import _path  # noqa: F401
import stats


class Basic(unittest.TestCase):
    def test_median_mad_cv(self):
        self.assertEqual(stats.median([3, 1, 2]), 2.0)
        self.assertEqual(stats.median([4, 1, 2, 3]), 2.5)
        # |x - 3| = 2,1,0,1,97 -> median 1 -> MAD 1.4826
        self.assertAlmostEqual(stats.mad([1, 2, 3, 4, 100]), 1.4826)
        self.assertAlmostEqual(stats.cv([10, 10, 10]), 0.0)
        self.assertAlmostEqual(stats.cv([9, 11]), math.sqrt(2) / 10)
        # MAD x 1.4826 estimates sigma for normal data
        x = np.random.default_rng(1).normal(100, 5, 20000)
        self.assertAlmostEqual(stats.mad(x), 5.0, delta=0.15)


class Bootstrap(unittest.TestCase):
    def test_identical(self):
        r, lo, hi = stats.bootstrap_ratio_ci([5.0] * 7, [5.0] * 7)
        self.assertEqual((r, lo, hi), (1.0, 1.0, 1.0))

    def test_deterministic(self):
        a, b = [10, 11, 9, 10.5, 9.5], [8, 8.5, 7.9, 8.2, 8.1]
        self.assertEqual(stats.bootstrap_ratio_ci(a, b), stats.bootstrap_ratio_ci(a, b))

    def test_ci_covers_true_ratio(self):
        rng = np.random.default_rng(5)
        base = rng.normal(100, 2, 15)
        head = rng.normal(80, 2, 15)
        r, lo, hi = stats.bootstrap_ratio_ci(base, head)
        self.assertAlmostEqual(r, np.median(head) / np.median(base))
        self.assertLess(lo, 0.8 + 0.03)
        self.assertGreater(hi, 0.8 - 0.03)
        self.assertLess(hi, 0.9)


class Rule(unittest.TestCase):
    def test_verdict_boundaries(self):
        self.assertEqual(stats.verdict(0.80, 0.97, 0.03), "better")
        self.assertEqual(stats.verdict(0.80, 0.971, 0.03), "neutral")
        self.assertEqual(stats.verdict(1.03, 1.2, 0.03), "worse")
        self.assertEqual(stats.verdict(1.029, 1.2, 0.03), "neutral")
        self.assertEqual(stats.verdict(float("nan"), 1.0, 0.03), "n/a")

    def test_theta(self):
        self.assertEqual(stats.theta_for("p50_ms", None), 0.03)
        noise = {"theta_min": 0.03, "metrics": {"p50_ms": {"aa_spread": 0.025}, "default": {"aa_spread": 0.01}}}
        self.assertAlmostEqual(stats.theta_for("p50_ms", noise), 0.05)
        self.assertAlmostEqual(stats.theta_for("wall_s", noise), 0.03)  # 2 x 0.01 < 3%

    def test_compare_directions(self):
        base = [10.0, 10.2, 9.9, 10.1, 10.0, 9.8, 10.3]
        faster = [v * 0.7 for v in base]
        c = stats.compare(base, faster, "p50_ms")
        self.assertEqual(c["verdict"], "better")
        self.assertEqual(c["label"], "faster")
        slower = [v * 1.3 for v in base]
        self.assertEqual(stats.compare(base, slower, "wall_s")["verdict"], "worse")
        # throughput: higher is better, the cost ratio is base/head
        tput = stats.compare(base, [v * 2 for v in base], "tiles_per_s")
        self.assertEqual(tput["verdict"], "better")
        self.assertAlmostEqual(tput["cost_ratio"], 0.5)
        same = stats.compare(base, list(reversed(base)), "p50_ms")
        self.assertEqual(same["verdict"], "neutral")
        self.assertEqual(stats.compare(base, [v * 0.98 for v in base], "p50_ms")["verdict"], "neutral")

    def test_aa_spread(self):
        a = [10.0, 10.1, 9.9, 10.05, 9.95]
        same = stats.aa_spread(a, a)  # independent resampling of both sides: small, not 0
        self.assertGreaterEqual(same, 0.0)
        self.assertLess(same, 0.02)
        self.assertGreater(stats.aa_spread(a, [10.0, 10.3, 9.7, 10.2, 9.8]), same)

if __name__ == "__main__":
    unittest.main()
