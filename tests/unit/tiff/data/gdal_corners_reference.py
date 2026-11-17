#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""GDAL reference corners for the GeoKeys CRS check (tests/unit/tiff/geotiff_corners_test.cpp).

For every GeoTIFF under the given directories that GDAL opens with a geotransform and a CRS,
writes one line:

  path  width  height  lon0 lat0  lon1 lat1  lon2 lat2  lon3 lat3  wkt2

The four corners of the raster (pixel (0, 0), (width, 0), (width, height), (0, height)) are
mapped by GDAL from its CRS to EPSG:4326 (longitude, latitude; "nan" when GDAL cannot map one);
wkt2 is GDAL's CRS as WKT2_2019 with newlines escaped as \\n. Files are opened read-only with
GDAL_PAM_ENABLED=NO, so nothing is written next to them.

Run:  python3 -I tests/unit/tiff/data/gdal_corners_reference.py --out FILE DIR [DIR ...]
"""

import argparse
import math
import os
import sys

os.environ["GDAL_PAM_ENABLED"] = "NO"

from osgeo import gdal, osr  # noqa: E402

gdal.UseExceptions()
osr.UseExceptions()
gdal.SetConfigOption("GDAL_PAM_ENABLED", "NO")


def rasters(dirs):
    for root_dir in dirs:
        for root, _, files in os.walk(root_dir):
            for name in sorted(files):
                if name.lower().endswith((".tif", ".tiff")):
                    yield os.path.join(root, name)


def corners(path):
    """(width, height, [lon, lat] * 4, wkt2) or None when GDAL has no geotransform or CRS."""
    try:
        ds = gdal.OpenEx(path, gdal.OF_RASTER | gdal.OF_READONLY, allowed_drivers=["GTiff"])
    except RuntimeError:
        return None
    srs = ds.GetSpatialRef()
    gt = ds.GetGeoTransform(can_return_null=True)
    if srs is None or gt is None:
        return None
    wgs84 = osr.SpatialReference()
    wgs84.ImportFromEPSG(4326)
    wgs84.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    try:
        ct = osr.CoordinateTransformation(srs, wgs84)
    except RuntimeError:
        return None
    w, h = ds.RasterXSize, ds.RasterYSize
    out = []
    for px, py in ((0, 0), (w, 0), (w, h), (0, h)):
        x = gt[0] + px * gt[1] + py * gt[2]
        y = gt[3] + px * gt[4] + py * gt[5]
        try:
            lon, lat, _ = ct.TransformPoint(x, y)
        except RuntimeError:
            lon = lat = math.nan
        if not (math.isfinite(lon) and math.isfinite(lat)):
            lon = lat = math.nan
        out += [lon, lat]
    wkt = srs.ExportToWkt(["FORMAT=WKT2_2019"]).replace("\n", "\\n")
    return w, h, out, wkt


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("dirs", nargs="+")
    args = ap.parse_args()
    n = 0
    with open(args.out, "w", encoding="utf-8") as f:
        for path in rasters(args.dirs):
            r = corners(path)
            if r is None:
                continue
            w, h, c, wkt = r
            f.write("\t".join([path, str(w), str(h)] + ["%.17g" % v for v in c] + [wkt]) + "\n")
            n += 1
    print("%d rasters with a geotransform and a CRS" % n, file=sys.stderr)


if __name__ == "__main__":
    main()
