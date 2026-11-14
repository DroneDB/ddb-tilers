#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Generates the GCP-only GeoTIFFs of tests/fixtures/gcp, GDAL's COGs of them and gcp_cog_reference.inc.

    python3 tests/unit/cogbuild/data/gen_gcp_cog_reference.py --tool gdal_cog_warp --proj-data DIR

Not run by the build: the outputs are committed. The sources are small rasters georeferenced only by
ground control points (GCPs in the TIFF, the CRS in the GeoKeys, no geotransform), written with the
python bindings of the development host's GDAL (seeded, so the same files every time), plus three of
GDAL's own samples (tests/fixtures/gdal). The reference is gdal_cog_warp.cpp, a small tool built
against the static GDAL 3.13.3 of vcpkg (the GDAL libddb ships), which runs GDALWarp with libddb's
COG arguments (-of COG -t_srs EPSG:3857 -r bilinear, GoogleMapsCompatible, ZOOM_LEVEL_STRATEGY=UPPER,
its transparency and compression rules); --proj-data is the proj.db folder of that GDAL's PROJ.
For each case the .inc records GDAL's grid (size and geotransform, as hexadecimal floats) and the
chunks its warper ran (source and destination windows, from GDAL's debug messages), or GDAL's
failure, and tests/fixtures/gcp/gdal holds GDAL's COG. A raster without a CRS is warped by GDAL as if
its GCPs were in EPSG:3857, but libddb refuses it before (no projection): no COG is kept for it.

--capella FILE adds a real-world case that is not committed: a Capella SLC GeoTIFF (CInt16, 225 GCPs
in EPSG:4326 and RPC metadata), for which the .inc records the CRC-32 of every band of every level of
GDAL's COG instead (the test runs where the file is).
"""

import argparse
import math
import os
import subprocess
import zlib

import numpy as np
from osgeo import gdal, osr

gdal.UseExceptions()
osr.UseExceptions()

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", "..", "..", ".."))
FIXTURES = os.path.join(ROOT, "tests", "fixtures", "gcp")
SAMPLES = os.path.join(ROOT, "tests", "fixtures", "gdal")

NP_TYPES = {gdal.GDT_Byte: np.uint8, gdal.GDT_UInt16: np.uint16, gdal.GDT_Int16: np.int16,
            gdal.GDT_Float32: np.float32}


def content(rng, w, h, b, scale, noise):
    """A smooth pattern with noise, so that a shift of a fraction of a pixel shows."""
    y, x = np.mgrid[0:h, 0:w].astype(np.float64)
    v = 120 + 80 * np.sin(x / (7.0 + 3 * b)) * np.cos(y / (5.0 + 2 * b)) + noise * rng.standard_normal((h, w))
    return v * scale


def write(name, w, h, gcps, crs, dtype=gdal.GDT_Byte, bands=1, nodata=None, scale=1.0, noise=8.0, alpha=False):
    rng = np.random.default_rng(sum(map(ord, name)))
    path = os.path.join(FIXTURES, name + ".tif")
    opts = ["COMPRESS=DEFLATE"] + (["ALPHA=YES"] if alpha else [])
    d = gdal.GetDriverByName("GTiff").Create(path, w, h, bands, dtype, opts)
    srs = ""
    if crs:
        srs = osr.SpatialReference()
        srs.SetFromUserInput(crs)
    d.SetGCPs([gdal.GCP(X, Y, 0, p, l) for (p, l, X, Y) in gcps], srs)
    t = NP_TYPES[dtype]
    for b in range(bands):
        if alpha and b == bands - 1:
            yy, xx = np.mgrid[0:h, 0:w]
            a = np.where((xx - w * 0.3) ** 2 + (yy - h * 0.6) ** 2 < (min(w, h) * 0.2) ** 2, 0, 255).astype(np.uint8)
            d.GetRasterBand(b + 1).WriteArray(a)
            continue
        a = content(rng, w, h, b, scale, noise)
        if t != np.float32:
            info = np.iinfo(t)
            a = np.clip(np.round(a), info.min, info.max)
        a = a.astype(t)
        if nodata is not None:
            yy, xx = np.mgrid[0:h, 0:w]
            a[(xx - w * 0.7) ** 2 + (yy - h * 0.3) ** 2 < (min(w, h) * 0.18) ** 2] = nodata
            d.GetRasterBand(b + 1).SetNoDataValue(nodata)
        d.GetRasterBand(b + 1).WriteArray(a)
    d = None
    return path


def utm(p, l, angle=25, size=0.5, curve=0.0):
    a = math.radians(angle)
    return (500000 + size * (p * math.cos(a) + l * math.sin(a)) + curve * p * p,
            5000000 + size * (p * math.sin(a) - l * math.cos(a)))


def make_sources():
    os.makedirs(FIXTURES, exist_ok=True)
    W, H = 64, 48
    rng = np.random.default_rng(11)
    cases = []
    # 4 GCPs of a projective mapping: GDAL's homography.
    quad = [(0, 0, 500000, 5000030), (W, 0, 500026, 5000028), (W, H, 500024, 5000005), (0, H, 499999, 5000001)]
    cases.append(("homography4_utm_byte", write("homography4_utm_byte", W, H, quad, "EPSG:32632")))
    # 5 GCPs, least-squares homography, geographic, Int16.
    q5 = [(0, 0, 7.0, 45.0005), (W, 0, 7.0008, 45.0006), (W, H, 7.00085, 44.99995), (0, H, 7.00002, 44.99992),
          (W / 2, H / 2, 7.00042, 45.000255)]
    cases.append(("homography5_4326_int16",
                  write("homography5_4326_int16", W, H, q5, "EPSG:4326", gdal.GDT_Int16, scale=10)))
    # 6 GCPs: a polynomial of order 2 (an exact fit), rotated and curved, UInt16 with nodata.
    g6 = [(0, 0), (W, 0), (0, H), (W, H), (W / 2, H / 3), (W / 3, H * 0.8)]
    cases.append(("poly6_utm_uint16_nodata",
                  write("poly6_utm_uint16_nodata", W, H, [(p, l, *utm(p, l, curve=2e-3)) for p, l in g6],
                        "EPSG:32633", gdal.GDT_UInt16, nodata=0, scale=100)))
    # 10 GCPs at random places, geographic and sheared, Float32 with nodata.
    pts = [(rng.uniform(0, W), rng.uniform(0, H)) for _ in range(10)]
    cases.append(("poly10_4326_float32_nodata",
                  write("poly10_4326_float32_nodata", W, H,
                        [(p, l, 11.0 + 6e-6 * p + 1.2e-6 * l, 46.0 - 4.5e-6 * l + 9e-7 * p) for p, l in pts],
                        "EPSG:4326", gdal.GDT_Float32, nodata=-9999.0)))
    # 25 noisy GCPs (a least-squares polynomial of order 2), gray and alpha.
    g25 = []
    for j in range(5):
        for i in range(5):
            p, l = W * (i + 0.5) / 5, H * (j + 0.5) / 5
            X, Y = utm(p, l, angle=-12, size=0.4)
            g25.append((p, l, X + 0.05 * rng.standard_normal(), Y + 0.05 * rng.standard_normal()))
    cases.append(("poly25_noisy_utm_gray_alpha",
                  write("poly25_noisy_utm_gray_alpha", W, H, g25, "EPSG:32633", bands=2, alpha=True)))
    # 3 GCPs already in EPSG:3857 (order 1, no reprojection), rotated.
    g3 = [(0, 0, 1000000.0, 5000000.0), (W, 0, 1000060.0, 5000020.0), (0, H, 1000015.0, 4999955.0)]
    cases.append(("poly3_3857_byte", write("poly3_3857_byte", W, H, g3, "EPSG:3857")))
    # 6 GCPs across the antimeridian (longitudes unwrapped by GDAL before the fit).
    am = []
    for p, l in g6:
        lon = 179.9995 + 1.5e-5 * p
        am.append((p, l, lon - 360 if lon > 180 else lon, -16.5 - 1.2e-5 * l))
    cases.append(("antimeridian6_4326_byte", write("antimeridian6_4326_byte", W, H, am, "EPSG:4326")))
    # Failures.
    small = 8
    cases.append(("fail_one_gcp", write("fail_one_gcp", small, small, [(0, 0, 7.0, 45.0)], "EPSG:4326")))
    cases.append(("fail_collinear", write("fail_collinear", small, small,
                                          [(0, 0, 7.0, 45.0), (4, 4, 7.001, 44.999), (8, 8, 7.002, 44.998)],
                                          "EPSG:4326")))
    cases.append(("no_crs", write("no_crs", small, small, quad, None)))
    for s in ("byte_gcp", "byte_gcp_pixelispoint", "cint_sar"):
        cases.append(("gdal_" + s, os.path.join(SAMPLES, s + ".tif")))
    return cases


def hexf(v):
    return float(v).hex()


def level_crcs(path):
    """CRC-32 of every band of every level (full resolution first) as GDAL reads it."""
    ds = gdal.Open(path)
    out = []
    for level in range(1 + ds.GetRasterBand(1).GetOverviewCount()):
        for b in range(ds.RasterCount):
            band = ds.GetRasterBand(b + 1)
            if level:
                band = band.GetOverview(level - 1)
            out.append(zlib.crc32(band.ReadRaster()))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tool", required=True, help="gdal_cog_warp built against vcpkg's GDAL 3.13.3")
    ap.add_argument("--proj-data", required=True, help="the proj.db folder of that GDAL's PROJ")
    ap.add_argument("--capella", default="/mnt/data/ddb-tilers-work/fixtures/realworld/capella_slc_24mb.tif")
    ap.add_argument("--scratch", default="/tmp", help="where the COG of the --capella case is written")
    args = ap.parse_args()
    cases = make_sources()
    if os.path.exists(args.capella):
        cases.append(("capella_slc", args.capella))
    out_dir = os.path.join(FIXTURES, "gdal")
    os.makedirs(out_dir, exist_ok=True)
    env = dict(os.environ, PROJ_DATA=args.proj_data, GDAL_PAM_ENABLED="NO")
    lines = ["// Generated by gen_gcp_cog_reference.py: do not edit.",
             "// GDALWarp of GDAL 3.13.3 (vcpkg) with libddb's COG arguments on rasters georeferenced by GCPs only:",
             "// {name, source (relative to tests/fixtures, or absolute), ok, width, height, geotransform,",
             "//  GDAL's message, chunks {sx, sy, sw, sh, dx, dy, dw, dh}, CRC-32 per level and band (when no COG is kept)}"]
    version = ""
    for name, src in cases:
        external = not src.startswith(os.path.join(ROOT, "tests", "fixtures"))
        out = os.path.join(args.scratch if external else out_dir, name + ".tif")
        if os.path.exists(out):
            os.unlink(out)
        p = subprocess.run([args.tool, src, out], capture_output=True, text=True, env=env)
        res = p.stdout.strip().splitlines()
        version = res[0]
        last = res[-1]
        chunks = ["{%s}" % ", ".join(l.split()[1:]) for l in res if l.startswith("chunk ")]
        rel = src if external else os.path.relpath(src, os.path.join(ROOT, "tests", "fixtures"))
        if last.startswith("ok "):
            f = last.split()
            w, h = int(f[1]), int(f[2])
            gt = [float(v) for v in f[3:9]]
            crcs = level_crcs(out) if external else []
            if external or name == "no_crs":
                os.unlink(out)
            lines.append('{"%s", "%s", true, %d, %d, {%s}, "",\n {%s},\n {%s}},' %
                         (name, rel, w, h, ", ".join(hexf(v) for v in gt), ", ".join(chunks),
                          ", ".join("0x%08x" % c for c in crcs)))
        else:
            if os.path.exists(out):
                os.unlink(out)
            msg = last[5:].replace("\\", "\\\\").replace('"', '\\"')
            lines.append('{"%s", "%s", false, 0, 0, {}, "%s", {}, {}},' % (name, rel, msg))
        print(name, last, len(chunks), "chunks")
    lines.insert(1, "// %s" % version)
    with open(os.path.join(HERE, "gcp_cog_reference.inc"), "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
