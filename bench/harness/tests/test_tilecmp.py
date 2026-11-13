# SPDX-License-Identifier: MPL-2.0
import contextlib
import io
import json
import math
import os
import tempfile
import unittest

import numpy as np
from PIL import Image

from tests import _path  # noqa: F401
import tilecmp


def gradient(h=256, w=256, alpha=255):
    yy, xx = np.mgrid[0:h, 0:w]
    img = np.zeros((h, w, 4), np.uint8)
    img[..., 0] = 60 + (xx * 120) // w
    img[..., 1] = 60 + (yy * 120) // h
    img[..., 2] = 100
    img[..., 3] = alpha
    return img


class Metrics(unittest.TestCase):
    def test_identical(self):
        a = gradient()
        r = tilecmp.compare_images(a, a.copy())
        self.assertTrue(r["exact"])
        self.assertEqual(r["psnr"], math.inf)
        self.assertAlmostEqual(r["ssim"], 1.0, places=12)
        self.assertEqual(r["alpha_iou"], 1.0)
        self.assertEqual(r["max_abs_diff"], 0)
        self.assertTrue(r["pass"])

    def test_constant_offset_psnr(self):
        a = gradient()
        b = a.copy()
        b[..., :3] += 5  # no clipping: values stay below 255
        expected = 20 * math.log10(255 / 5)  # MSE = 25
        r = tilecmp.compare_images(a, b)
        self.assertAlmostEqual(r["psnr"], expected, places=9)
        self.assertEqual(r["max_abs_diff"], 5)
        self.assertFalse(r["exact"])

    def test_known_noise_psnr(self):
        a = gradient()
        b = a.copy().astype(np.int16)
        noise = np.where((np.indices(a.shape[:2]).sum(0) % 2) == 0, 3, -3)  # +-3 checkerboard
        b[..., :3] += noise[..., None]
        b = b.astype(np.uint8)
        r = tilecmp.compare_images(a, b)
        self.assertAlmostEqual(r["psnr"], 10 * math.log10(255 ** 2 / 9), places=9)
        self.assertLess(r["ssim"], 1.0)
        self.assertGreater(r["ssim"], 0.5)

    def test_psnr_only_where_both_opaque(self):
        a = gradient()
        b = a.copy()
        b[:, 128:, 3] = 0
        b[:, 128:, :3] = 0  # garbage under transparency must not count
        self.assertEqual(tilecmp.psnr(a, b), math.inf)
        self.assertAlmostEqual(tilecmp.alpha_iou(a, b), 0.5)
        r = tilecmp.compare_images(a, b)
        self.assertFalse(r["pass"])
        self.assertIn("alpha IoU", r["reason"])
        empty = gradient(alpha=0)
        self.assertIsNone(tilecmp.psnr(empty, empty))
        self.assertEqual(tilecmp.alpha_iou(empty, empty), 1.0)

    def test_ssim_matches_direct_computation(self):
        rng = np.random.default_rng(3)
        a = gradient(32, 32)
        b = a.copy()
        b[..., :3] = np.clip(b[..., :3].astype(int) + rng.integers(-20, 21, (32, 32, 3)), 0, 255)
        x, y = tilecmp.luma(a), tilecmp.luma(b)
        vals = []
        for i in range(32 - 7):
            for j in range(32 - 7):
                wx, wy = x[i:i + 8, j:j + 8], y[i:i + 8, j:j + 8]
                mx, my = wx.mean(), wy.mean()
                vx, vy = wx.var(), wy.var()
                cxy = ((wx - mx) * (wy - my)).mean()
                vals.append(((2 * mx * my + tilecmp.C1) * (2 * cxy + tilecmp.C2)) /
                            ((mx * mx + my * my + tilecmp.C1) * (vx + vy + tilecmp.C2)))
        self.assertAlmostEqual(tilecmp.ssim(a, b), float(np.mean(vals)), places=9)

    def test_size_mismatch(self):
        r = tilecmp.compare_images(gradient(256, 256), gradient(512, 512))
        self.assertFalse(r["pass"])


class Dirs(unittest.TestCase):
    def test_compare_dirs(self):
        with tempfile.TemporaryDirectory() as t:
            da, db, dd = (os.path.join(t, n) for n in ("a", "b", "diff"))
            os.makedirs(da)
            os.makedirs(db)
            g = gradient()
            bad = g.copy()
            bad[..., 0] = 255 - bad[..., 0]
            rows = [(0, g, g), (1, g, bad)]
            for d, idx in ((da, 1), (db, 2)):
                with open(os.path.join(d, "index.jsonl"), "w") as f:
                    for i, *imgs in rows:
                        name = "%06d_18_1_2.png" % i
                        Image.fromarray(imgs[idx - 1], "RGBA").save(os.path.join(d, name))
                        f.write(json.dumps({"i": i, "z": 18, "x": 1, "y": 2, "file": name, "ok": True}) + "\n")
                    # a request that failed on both sides
                    f.write(json.dumps({"i": 2, "z": 30, "x": 0, "y": 0, "file": "", "ok": False}) + "\n")
            r = tilecmp.compare_dirs(da, db, diff_dir=dd)
            s = r["summary"]
            self.assertEqual((s["tiles"], s["pass"], s["fail"], s["exact"], s["both_failed"]), (3, 2, 1, 1, 1))
            self.assertTrue(os.path.isfile(os.path.join(dd, "000001_18_1_2_mosaic.png")))
            with Image.open(os.path.join(dd, "000001_18_1_2_mosaic.png")) as m:
                self.assertEqual(m.size, (768, 256))
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(tilecmp.main(["--a", da, "--b", da]), 0)


if __name__ == "__main__":
    unittest.main()
