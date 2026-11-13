# SPDX-License-Identifier: MPL-2.0
import unittest

from tests import _path  # noqa: F401
import gates


class GateCpus(unittest.TestCase):
    def test_parse_cpulist(self):
        self.assertEqual(gates.parse_cpulist("0-7,16-23"), list(range(8)) + list(range(16, 24)))
        self.assertEqual(gates.parse_cpulist("3"), [3])
        self.assertEqual(gates.parse_cpulist("5,1,1,2-3"), [1, 2, 3, 5])

    def test_cpu_util(self):
        before = {0: (100, 1000), 1: (0, 1000), 9: (50, 500)}
        after = {0: (150, 1100), 1: (0, 1100), 9: (100, 600)}
        u = gates.cpu_util(before, after, {0, 1})
        self.assertEqual(sorted(u), [0, 1])
        self.assertAlmostEqual(u[0], 50.0)
        self.assertAlmostEqual(u[1], 0.0)
        self.assertEqual(gates.util_summary(u), {"mean": 25.0, "max": 50.0})

    def test_quiet_sample_reads_proc_stat(self):
        s = gates.quiet_sample(0.2, 100.0, gate_cpus=[0, 1])
        self.assertIn("other_cpus_util_pct", s)
        self.assertEqual(s["busy"], [])  # nothing can exceed 100%
        self.assertTrue(gates.is_quiet(s, load1_max=0.0, gated_load=False))


if __name__ == "__main__":
    unittest.main()
