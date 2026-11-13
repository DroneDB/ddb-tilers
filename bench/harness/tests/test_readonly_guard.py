# SPDX-License-Identifier: MPL-2.0
import os
import tempfile
import unittest

from tests import _path  # noqa: F401
import run


class Guard(unittest.TestCase):
    def test_new_sidecar_in_build_tree_is_caught(self):
        with tempfile.TemporaryDirectory() as d:
            cog = os.path.join(d, "ds", ".ddb", "build", "h" * 64, "cog")
            os.makedirs(cog)
            f = os.path.join(cog, "cog.tif")
            open(f, "wb").write(b"x")
            before = run.snapshot_dirs([f])
            self.assertEqual(run.snapshot_diff(before, run.snapshot_dirs([f])), [])
            open(f + ".stats.json", "w").write("{}")           # next to the COG
            diff = run.snapshot_diff(before, run.snapshot_dirs([f]))
            self.assertIn(f + ".stats.json", [x["path"] for x in diff])
            os.makedirs(os.path.join(d, "ds", ".ddb", "other"))  # elsewhere in the dataset tree
            diff = run.snapshot_diff(before, run.snapshot_dirs([f]))
            self.assertIn(os.path.join(d, "ds", ".ddb", "other"), [x["path"] for x in diff])


if __name__ == "__main__":
    unittest.main()
