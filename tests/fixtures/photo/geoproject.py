#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Writes geoproject.tsv and gdal/geo_photo_2steps.tif: GDAL's geoprojections of the photos of
this folder, for PhotoGeoproject.* (tests/unit/photo/photo_geoproject_test.cpp).

    python3 -I tests/fixtures/photo/geoproject.py --tools DIR

DIR holds gdal_geoproject (tests/fixtures/tools/gdal_geoproject.cpp), gdal_expected
(tests/fixtures/tools/gdal_expected.cpp) and gdal_cog_warp (tests/unit/cogbuild/data/
gdal_cog_warp.cpp), built against the GDAL libddb ships (vcpkg's GDAL 3.13.3); PROJ_DATA must
point at its share/proj. The system GDAL's Python bindings read the block size of an output.

Each case georeferences a photo by the four corners of its footprint (libddb's GCPs) and warps it:

    source  libddb's geoprojection (gdalwarp -dstalpha into EPSG:4326, nearest neighbour), written
            losslessly (DEFLATE, the strips of libddb's JPEG GeoTIFF) to check the pixels before
            JPEG; the chunks of GDAL's warper are those of libddb's JPEG GeoTIFF;
    3857    the grid of libddb's COG of the photo (the COG of its JPEG GeoTIFF), and one gdalwarp of
            the photo onto that grid (-r bilinear -dstalpha, -te / -tr of the grid, the 256 px tiles
            of the COG driver's temporary GeoTIFF, -wm 512), LZW.

Lines (tab-separated):

    case    NAME PHOTO TARGET OUTSCALE WIDTH HEIGHT GT0 .. GT5
    gcp     NAME PIXEL LINE X Y
    band    NAME BAND CRC          CRC-32 of the band of GDAL's output (full resolution)
    chunk   NAME SX SY SW SH DX DY DW DH

gdal/geo_photo_2steps.tif is libddb's COG of geo_photo.jpg before ddb-tilers: the JPEG GeoTIFF of
the geoprojection, then its COG (gdal_cog_warp: libddb's COG arguments).
"""
import argparse
import math
import os
import shutil
import subprocess
import tempfile

from osgeo import gdal

gdal.UseExceptions()
HERE = os.path.dirname(os.path.abspath(__file__))

def footprint(width, height, gsd):
    """The footprint of a photo of width x height pixels of gsd metres (upper left, lower left,
    lower right, upper right; longitude, latitude, rounded to 13 decimals as libddb gives them),
    rotated by 30 degrees, the far edge 8 % wider (a little perspective)."""
    lon0, lat0 = 12.4964, 41.9028
    w, h, th = width * gsd, height * gsd, math.radians(30)

    def pt(dx, dy):
        x, y = dx * math.cos(th) - dy * math.sin(th), dx * math.sin(th) + dy * math.cos(th)
        return (float("%.13f" % (lon0 + x / (111320 * math.cos(math.radians(lat0))))),
                float("%.13f" % (lat0 + y / 111320)))

    return [pt(-w / 2 * 1.08, h / 2), pt(-w / 2, -h / 2), pt(w / 2, -h / 2), pt(w / 2 * 1.08, h / 2)]


CASES = [
    # name, photo, target, outsize ("-" or "N%"), ground sample distance (m)
    ("jpg_source", "geo_photo.jpg", "source", "-", 0.25),
    ("jpg_3857", "geo_photo.jpg", "3857", "-", 0.25),
    # At 0.21 m one warp of the photo would take the next zoom level; libddb's two warps do not.
    ("jpg_3857_zoom", "geo_photo.jpg", "3857", "-", 0.21),
    ("png_source", "geo_photo_trns.png", "source", "-", 0.25),
    ("png_3857", "geo_photo_trns.png", "3857", "-", 0.25),
    ("png_source_half", "geo_photo_trns.png", "source", "50%", 0.25),
]

# One warp of the photo onto the grid of libddb's COG (the tiles of the COG driver's temporary
# GeoTIFF, its warp memory).
WARP_ARGS = ["-of", "GTiff", "-t_srs", "EPSG:3857", "-r", "bilinear", "-dstalpha", "-co", "TILED=YES", "-co",
             "COMPRESS=LZW", "-co", "PREDICTOR=2", "-wm", "512"]


def run(tool, args, chunks=False):
    env = dict(os.environ)
    if chunks:
        env["CHUNKS"] = "1"
    out = subprocess.run([tool] + args, check=True, capture_output=True, text=True, env=env).stdout.split("\n")
    ok = [line for line in out if line.startswith("ok ")]
    if not ok:
        raise RuntimeError(" ".join(out))
    return ok[0].split()[1:], [line.split()[1:] for line in out if line.startswith("chunk ")]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tools", required=True)
    args = parser.parse_args()
    geoproject = os.path.join(args.tools, "gdal_geoproject")
    expected = os.path.join(args.tools, "gdal_expected")
    cog_warp = os.path.join(args.tools, "gdal_cog_warp")
    lines = []
    with tempfile.TemporaryDirectory() as tmp:
        for name, photo, target, outsize, gsd in CASES:
            path = os.path.join(HERE, photo)
            ds = gdal.Open(path)
            w, h = ds.RasterXSize, ds.RasterYSize
            ds = None
            fp = footprint(w, h, gsd)
            fp_args = ["%.13f" % v for corner in fp for v in corner]
            out = os.path.join(tmp, name + ".tif")
            chunks = []
            jpeg = os.path.join(tmp, name + "_jpeg.tif")
            # libddb's JPEG GeoTIFF: its strips and GDAL's chunks.
            _, chunks = run(geoproject, [path, jpeg, str(w), str(h), outsize] + fp_args, chunks=True)
            if target == "source":
                # The same warp, lossless.
                jds = gdal.Open(jpeg)
                rows = jds.GetRasterBand(1).GetBlockSize()[1]
                jds = None
                grid, lossless_chunks = run(geoproject, [path, out, str(w), str(h), outsize] + fp_args +
                                            ["--replace", "-of", "GTiff", "-co", "COMPRESS=DEFLATE", "-co",
                                             "BLOCKYSIZE=%d" % rows, "-dstalpha"], chunks=True)
                assert lossless_chunks == chunks, name
            else:
                # libddb's COG of the JPEG GeoTIFF gives the grid; one warp of the photo onto it.
                cog = os.path.join(tmp, name + "_cog.tif")
                ok = [line.split()[1:] for line in subprocess.run([cog_warp, jpeg, cog], check=True,
                                                                     capture_output=True, text=True).stdout.splitlines()
                      if line.startswith("ok ")][0]
                cw, ch, gt = ok[0], ok[1], [float(v) for v in ok[2:]]
                maxx, miny = gt[0] + int(cw) * gt[1], gt[3] + int(ch) * gt[5]
                te = ["-te", "%.17g" % gt[0], "%.17g" % miny, "%.17g" % maxx, "%.17g" % gt[3],
                      "-tr", "%.17g" % gt[1], "%.17g" % -gt[5]]
                grid, _ = run(geoproject, [path, out, str(w), str(h), outsize] + fp_args + ["--replace"] +
                              WARP_ARGS + te)
                assert grid == [cw, ch] + ["%.17g" % v for v in gt], (name, grid, cw, ch, gt)
                chunks = []
                if name == "jpg_3857":
                    final = os.path.join(HERE, "gdal", "geo_photo_2steps.tif")
                    if os.path.exists(final):
                        os.remove(final)
                    shutil.copyfile(cog, final)
            scale = "1" if outsize == "-" else "%.17g" % (float(outsize[:-1]) / 100)
            lines.append(["case", name, photo, target, scale] + grid)
            sw = w if outsize == "-" else int(w * float(outsize[:-1]) / 100)
            sh = h if outsize == "-" else int(h * float(outsize[:-1]) / 100)
            for (px, py), (x, y) in zip([(0, 0), (0, sh), (sw, sh), (sw, 0)], fp):
                lines.append(["gcp", name, px, py, "%.13f" % x, "%.13f" % y])
            res = subprocess.run([expected, out, name], check=True, capture_output=True, text=True).stdout
            for line in res.splitlines():
                f = line.split("\t")
                if f[0] == "band":
                    lines.append(["band", name, f[2], f[5]])
            lines += [["chunk", name] + c for c in chunks]
    with open(os.path.join(HERE, "geoproject.tsv"), "w") as f:
        for line in lines:
            f.write("\t".join(str(v) for v in line) + "\n")
    print(f"wrote {len(lines)} lines")


if __name__ == "__main__":
    main()
