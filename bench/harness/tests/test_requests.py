# SPDX-License-Identifier: MPL-2.0
import math
import random
import unittest

from tests import _path  # noqa: F401
import requests as rq

O = 20037508.342789244
# R1 (brighton-beach-cog.tif) footprint, EPSG:3857, from the corpus manifest probe
R1_FP = [-10240777.97523657, 5916350.113617214, -10240638.168072835, 5916486.047398996]


class TileMaths(unittest.TestCase):
    def test_mercantile_tile(self):
        # mercantile docs: mercantile.tile(-105.0, 40.0, 1) -> Tile(x=0, y=0, z=1)
        self.assertEqual(rq.lonlat_to_tile(-105.0, 40.0, 1), (0, 0))
        self.assertEqual(rq.lonlat_to_tile(0.0, 0.0, 0), (0, 0))
        self.assertEqual(rq.lonlat_to_tile(179.9, -84.9, 2), (3, 3))

    def test_mercantile_ul(self):
        # mercantile docs: mercantile.ul(486, 332, 10) -> LngLat(lng=-9.140625, lat=53.33087298301705)
        lon, lat = rq.tile_ul_lonlat(486, 332, 10)
        self.assertAlmostEqual(lon, -9.140625, places=12)
        self.assertAlmostEqual(lat, 53.33087298301705, places=10)
        # a point just inside that corner belongs to the tile
        self.assertEqual(rq.lonlat_to_tile(lon + 1e-6, lat - 1e-6, 10), (486, 332))

    def test_world_bounds(self):
        # mercantile.xy_bounds(0, 0, 0)
        b = rq.tile_bounds(0, 0, 0)
        for v, e in zip(b, (-O, -O, O, O)):
            self.assertAlmostEqual(v, e, places=6)
        self.assertAlmostEqual(rq.lonlat_to_merc(180.0, 0.0)[0], O, places=6)
        self.assertEqual(rq.lonlat_to_merc(0.0, 0.0), (0.0, 0.0))
        lat_max = rq.merc_to_lonlat(0.0, O)[1]
        self.assertAlmostEqual(lat_max, rq.MAX_LAT, places=9)

    def test_xyz_tms(self):
        # gdal2tiles: TMS row counted from the south
        self.assertEqual(rq.xyz_to_tms(0, 1), 1)
        self.assertEqual(rq.xyz_to_tms(92370, 18), (1 << 18) - 1 - 92370)
        self.assertEqual(rq.xyz_to_tms(rq.xyz_to_tms(5, 4), 4), 5)

    def test_two_routes_agree(self):
        rng = random.Random(7)
        for _ in range(2000):
            lon, lat = rng.uniform(-179.9, 179.9), rng.uniform(-85.0, 85.0)
            z = rng.randint(0, 22)
            mx, my = rq.lonlat_to_merc(lon, lat)
            t1 = rq.lonlat_to_tile(lon, lat, z)
            t2 = rq.merc_to_tile(mx, my, z)
            # equal except within 1e-9 of a tile edge (floating point)
            if t1 != t2:
                span = rq.tile_span(z)
                fx = ((mx + O) / span) % 1.0
                fy = ((O - my) / span) % 1.0
                self.assertTrue(min(fx, 1 - fx, fy, 1 - fy) < 1e-6, (lon, lat, z, t1, t2))

    def test_bounds_roundtrip(self):
        for z, x, y in ((5, 3, 7), (18, 64083, 92370), (22, 1234567, 2345678)):
            b = rq.tile_bounds(x, y, z)
            self.assertEqual(rq.merc_to_tile((b[0] + b[2]) / 2, (b[1] + b[3]) / 2, z), (x, y))

    def test_r1_range(self):
        # Checked against libddb 2.4.1: tile 18/64083/92370 holds the north-west corner of R1
        self.assertEqual(rq.tile_range(R1_FP, 18), (64083, 64084, 92370, 92371))
        self.assertEqual(rq.tile_range(R1_FP, 14), (4005, 4005, 5773, 5773))
        # exact tile edges are exclusive at the max side
        b = rq.tile_bounds(10, 20, 6)
        self.assertEqual(rq.tile_range(list(b), 6), (10, 10, 20, 20))


class Scenarios(unittest.TestCase):
    def test_sweep_cap_and_stride(self):
        reqs = rq.sweep("f.tif", R1_FP, 18, 21, cap=5)
        per_z = {}
        for r in reqs:
            per_z.setdefault(r["z"], []).append((r["x"], r["y"]))
            self.assertTrue(r["tms"])
        self.assertEqual(len(per_z[18]), 4)  # 2x2 < cap: every tile
        self.assertEqual(len(per_z[21]), 5)
        self.assertEqual(len(set(per_z[21])), 5)
        self.assertEqual(rq.stride_pick(10, 4), [0, 2, 5, 7])
        self.assertEqual(rq.stride_pick(3, 4), [0, 1, 2])

    def test_pan_deterministic_and_large(self):
        a = rq.pan("f.tif", R1_FP, 18, 22, seed=3)
        b = rq.pan("f.tif", R1_FP, 18, 22, seed=3)
        c = rq.pan("f.tif", R1_FP, 18, 22, seed=4)
        self.assertEqual(a, b)
        self.assertNotEqual(a, c)
        self.assertGreaterEqual(len(a), 2000)
        self.assertTrue(all(18 <= r["z"] <= 22 for r in a))
        # no tile twice within a walk (client cache), first view = 8x5 tiles
        seen = {}
        for r in a:
            k = (r["walk"], r["z"], r["x"], r["y"])
            self.assertNotIn(k, seen)
            seen[k] = True
        first = [r for r in a if r["walk"] == 0 and r["step"] == 0]
        self.assertLessEqual(len(first), 40)
        self.assertEqual(rq.viewport_grid(), (8, 5))

    def test_pan_centre_out(self):
        reqs = rq.pan("f.tif", R1_FP, 18, 22, seed=1, steps=20, min_requests=1)
        step0 = [r for r in reqs if r["walk"] == 0 and r["step"] == 0]
        # tiles nearer to the viewport centre come first: the 4 central tiles of an 8x5 grid
        xs = sorted({r["x"] for r in step0})
        ys = sorted({r["y"] for r in step0})
        cx, cy = (xs[0] + xs[-1] + 1) / 2, (ys[0] + ys[-1] + 1) / 2
        d = [((r["x"] + .5 - cx) ** 2 + (r["y"] + .5 - cy) ** 2) for r in step0]
        self.assertLessEqual(d[0], min(d) + 1.0)
        self.assertLessEqual(d[0], d[-1])

    def test_retina(self):
        reqs = rq.generate("retina", "f.tif", R1_FP, 18, 22, min_requests=100)
        self.assertTrue(all(r["size"] == 512 for r in reqs))

    def test_edge(self):
        reqs = rq.edge("f.tif", R1_FP, 18, 21)
        kinds = {r["kind"] for r in reqs}
        self.assertTrue({"border", "outside", "far", "overzoom", "underzoom"} <= kinds)
        for r in reqs:
            n = 1 << r["z"]
            self.assertTrue(0 <= r["x"] < n and 0 <= r["y"] < n)
        keys = [(r["z"], r["x"], r["y"]) for r in reqs]
        self.assertEqual(len(keys), len(set(keys)))
        x0, x1, y0, y1 = rq.tile_range(R1_FP, 18)
        self.assertIn((18, x0 - 1, (y0 + y1) // 2), keys)


if __name__ == "__main__":
    unittest.main()
