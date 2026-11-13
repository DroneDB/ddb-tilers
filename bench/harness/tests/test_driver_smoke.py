# SPDX-License-Identifier: MPL-2.0
"""Driver smoke test: builds bench/ standalone (unless $DDBT_BENCH_BIN is set), then runs
`ddbt-bench tile --backend legacy` on 3 tiles of R1 against the frozen baseline libddb,
`run -- sleep 0.2` and `sysinfo`. Skipped when the baseline or the R1 file is missing."""
import json
import os
import shutil
import subprocess
import tempfile
import unittest

from tests import _path  # noqa: F401
import common
import requests as rq

R1 = "/mnt/data/repos/test_data/ortho/brighton-beach-cog.tif"
R1_FP = [-10240777.97523657, 5916350.113617214, -10240638.168072835, 5916486.047398996]


def driver():
    b = os.environ.get("DDBT_BENCH_BIN")
    if b and os.access(b, os.X_OK):
        return b
    build = os.path.join(common.WORK, "build-bench")
    src = os.path.join(common.REPO, "bench")
    subprocess.run(["cmake", "-S", src, "-B", build, "-DCMAKE_BUILD_TYPE=Release"], check=True,
                   capture_output=True)
    subprocess.run(["cmake", "--build", build, "--target", "ddbt-bench"], check=True, capture_output=True)
    return os.path.join(build, "ddbt-bench")


class DriverSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bin = driver()

    def run_json(self, args, env=None):
        p = subprocess.run([self.bin, args[0], "--out", "-"] + args[1:], capture_output=True, text=True, env=env)
        self.assertEqual(p.returncode, 0, p.stderr)
        return json.loads(p.stdout)

    def test_sysinfo(self):
        d = self.run_json(["sysinfo"])
        self.assertEqual(d["schema"], "ddbt-bench-sysinfo/1")
        self.assertGreater(d["cpu"]["logical_cpus"], 0)
        self.assertTrue(d["cpu"]["model"])
        self.assertIn("ddb-tilers", d["repos"])
        sets = common.pin_sets(d)
        self.assertEqual(len(sets["1T"]), 1)
        self.assertNotEqual(sets["1T"][0], 0)

    def test_run_sleep(self):
        d = self.run_json(["run", "--", "sleep", "0.2"])
        self.assertEqual(d["exit_code"], 0)
        self.assertGreaterEqual(d["wall_s"], 0.19)
        self.assertLess(d["wall_s"], 2.0)
        self.assertGreater(d["tree"]["rss_peak_kb"], 0)
        self.assertGreater(d["tree"]["samples"], 0)
        p = subprocess.run([self.bin, "run", "--out", "-", "--", "sh", "-c", "exit 7"], capture_output=True, text=True)
        self.assertEqual(p.returncode, 7)
        self.assertEqual(json.loads(p.stdout)["exit_code"], 7)

    def test_hash(self):
        with tempfile.NamedTemporaryFile("wb") as f:
            f.write(b"abc")
            f.flush()
            out = subprocess.run([self.bin, "hash", f.name], capture_output=True, text=True, check=True).stdout
        self.assertTrue(out.startswith("44bc2cf5ad770999"))  # XXH64("abc", seed 0)

    @unittest.skipUnless(common.baseline_lib() and os.path.isfile(R1), "baseline libddb or R1 missing")
    def test_tile_legacy(self):
        x0, x1, y0, y1 = rq.tile_range(R1_FP, 20)
        reqs = [rq._req(R1, 18, *rq.tile_range(R1_FP, 18)[::2], 256, "png"),
                rq._req(R1, 20, (x0 + x1) // 2, (y0 + y1) // 2, 256, "png"),
                rq._req(R1, 20, (x0 + x1) // 2, (y0 + y1) // 2, 256, "webp")]
        home = tempfile.mkdtemp(prefix="smoke-", dir=os.path.join(common.WORK, "tmp"))
        try:
            with tempfile.TemporaryDirectory() as t:
                rf = os.path.join(t, "r.jsonl")
                rq.write_jsonl(reqs, rf)
                env = common.baseline_env()
                env["DDB_HOME"] = home
                dump = os.path.join(t, "tiles")
                d = self.run_json(["tile", "--backend", "legacy", "--lib", common.baseline_lib(), "--requests", rf,
                                   "--threads", "2", "--warmup-rounds", "1", "--rounds", "2", "--dump-tiles", dump],
                                  env)
                self.assertEqual(d["n_requests"], 3)
                self.assertTrue(d["lib_version"].startswith("2.4.1"))
                self.assertEqual(d["hash_mismatches"], 0)
                meas = [r for r in d["round_results"] if not r["warmup"]]
                self.assertEqual(len(meas), 2)
                for r in meas:
                    self.assertEqual(r["failures"], 0, r["errors"])
                    self.assertGreater(r["latency_ms"]["p50"], 0)
                    self.assertGreater(r["bytes"], 0)
                self.assertEqual(len(d["requests"]), 3)
                files = sorted(f for f in os.listdir(dump) if f != "index.jsonl")
                self.assertEqual(len(files), 3)
                self.assertTrue(files[2].endswith(".webp"))
                with open(os.path.join(dump, files[0]), "rb") as f:
                    self.assertEqual(f.read(8), b"\x89PNG\r\n\x1a\n")
                print("\n  legacy R1 tile p50 %.2f ms (warm, 2 threads), set hash %s" % (
                    meas[0]["latency_ms"]["p50"], d["set_hash"]))
        finally:
            shutil.rmtree(home, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
