# SPDX-License-Identifier: MPL-2.0
import unittest

from tests import _path  # noqa: F401
import guard

TH = dict(guard.DEFAULT_THRESHOLDS)


class Check(unittest.TestCase):
    REF = {"tile/R1/sweep": {"p50_ms": 2.0, "p99_ms": 4.0, "tiles_per_s": 500.0, "rss_peak_kb": 1000,
                             "failures": 0, "set_hashes": ["aa"]},
           "build/R1/cog/8T": {"wall_s": 1.0, "tree_rss_peak_kb": 2000},
           "cachegrind/R1/sweep": {"instructions": 1000000, "set_hashes": ["bb"], "failures": 0}}

    def cur(self, **over):
        c = {k: dict(v) for k, v in self.REF.items()}
        for path, v in over.items():
            case, key = path.split(":")
            c[case][key] = v
        return c

    def test_pass(self):
        self.assertEqual(guard.check(self.REF, self.cur(), TH), [])
        self.assertEqual(guard.check(self.REF, self.cur(**{"tile/R1/sweep:p50_ms": 2.19}), TH), [])

    def test_regressions(self):
        cases = {
            "tile/R1/sweep:p50_ms": 2.3,
            "tile/R1/sweep:tiles_per_s": 400.0,
            "tile/R1/sweep:rss_peak_kb": 1200,
            "tile/R1/sweep:failures": 1,
            "tile/R1/sweep:set_hashes": ["cc"],
            "build/R1/cog/8T:wall_s": 1.2,
            "cachegrind/R1/sweep:instructions": 1011000,
        }
        for k, v in cases.items():
            probs = guard.check(self.REF, self.cur(**{k: v}), TH)
            self.assertEqual(len(probs), 1, (k, probs))

    def test_missing_case(self):
        cur = self.cur()
        del cur["build/R1/cog/8T"]
        self.assertIn("missing", guard.check(self.REF, cur, TH)[0])


if __name__ == "__main__":
    unittest.main()
