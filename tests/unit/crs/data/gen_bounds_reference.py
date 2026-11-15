#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Generates bounds_reference.inc: GDAL's bounds of boxes transformed between CRSs, the reference of
TransformBounds.

    python3 tests/unit/crs/data/gen_bounds_reference.py [--proj-data DIR]

The reference is what gdalwarp does with -te_srs: osr.CoordinateTransformation with the area of
interest GDALComputeAreaOfInterest gives for the box (traditional GIS axis order), then
TransformBounds with 21 points of densification. It uses the proj.db of this build (--proj-data,
default the vcpkg_installed of the linux-gcc-release preset).
"""

import argparse
import os

from osgeo import gdal, osr

gdal.UseExceptions()
osr.UseExceptions()

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
DEFAULT_PROJ_DATA = os.path.join(REPO, "build", "linux-gcc-release", "vcpkg_installed", "x64-linux-ddbt", "share",
                                 "proj")

# name, from, to, box (min x, min y, max x, max y in the units of from, longitude first)
CASES = [
    ("utm_to_wgs84", "EPSG:32633", "EPSG:4326", (500000, 4999000, 501000, 5000000)),
    ("wgs84_to_webmercator", "EPSG:4326", "EPSG:3857", (11.5, 44.9, 11.6, 45.0)),
    ("webmercator_to_utm", "EPSG:3857", "EPSG:32633", (1675000, 5620000, 1680000, 5625000)),
    ("lambert93_to_wgs84", "EPSG:2154", "EPSG:4326", (640000, 6850000, 665000, 6875000)),
    # a box of longitudes across the antimeridian, to a projection centred near it and to UTM 60
    ("antimeridian_to_pdc_mercator", "EPSG:4326", "EPSG:3832", (170, -10, -170, 10)),
    ("antimeridian_to_utm60", "EPSG:4326", "EPSG:32660", (175, -5, -175, 5)),
    # a projected box whose longitudes cross the antimeridian: west > east
    ("pdc_mercator_to_wgs84", "EPSG:3832", "EPSG:4326", (2000000, -1000000, 4000000, 1000000)),
    # the poles of a geographic target in the box
    ("arctic_polar_stereo_to_wgs84", "EPSG:3413", "EPSG:4326", (-1000000, -1000000, 1000000, 1000000)),
    ("antarctic_polar_stereo_to_wgs84", "EPSG:3031", "EPSG:4326", (-1000000, -1000000, 1000000, 1000000)),
    # the pole of a projected target in the box, and the Mercator pole
    ("arctic_to_polar_stereo", "EPSG:4326", "EPSG:3413", (-180, 60, 180, 90)),
    ("world_to_webmercator", "EPSG:4326", "EPSG:3857", (-180, -89, 180, 89)),
    # the pole of a projected target in the box, beyond the ring (the Mercator pole)
    ("pole_box_to_webmercator", "EPSG:3413", "EPSG:3857", (-1000000, -1000000, 1000000, 1000000)),
    # the meridian opposite the central meridian of a projected target in the box
    ("pacific_to_utm33", "EPSG:4326", "EPSG:32633", (-170, -10, -160, 10)),
    ("atlantic_to_pdc_mercator", "EPSG:4326", "EPSG:3832", (-41, -10, -21, 10)),
    # a projection in grads from Paris
    ("ntf_lambert2_to_wgs84", "EPSG:27572", "EPSG:4326", (600000, 2400000, 610000, 2410000)),
    ("wgs84_to_ntf_lambert2", "EPSG:4326", "EPSG:27572", (2.3, 48.8, 2.4, 48.9)),
    # no point transforms
    ("nothing_transforms", "EPSG:4326", "EPSG:32633", (10, 95, 20, 99)),
]


def crs(text):
    s = osr.SpatialReference()
    s.SetFromUserInput(text)
    s.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    return s


def area_of_interest(src, box):
    """GDALComputeAreaOfInterest of a box (its overload for an extent, which gdalwarp calls for
    -te_srs): the corners in the geographic CRS of the source, in its own angular unit."""
    horiz = src.Clone()
    if horiz.IsCompound():
        horiz.StripVertical()
    geog = horiz.CloneGeogCS()
    geog.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    try:
        ct = osr.CoordinateTransformation(horiz, geog)
    except Exception:
        return None
    x1, y1, x2, y2 = box
    corners = [(x1, y1), (x2, y1), (x1, y2), (x2, y2)]
    out = []
    for x, y in corners:
        try:
            p = ct.TransformPoint(x, y)
            ok = all(abs(v) != float("inf") and v == v for v in p[:2])
            out.append((p[0], p[1]) if ok else None)
        except Exception:
            out.append(None)
    valid = [p for p in out if p is not None]
    if not valid:
        return None
    w = min(p[0] for p in valid)
    s = min(p[1] for p in valid)
    e = max(p[0] for p in valid)
    n = max(p[1] for p in valid)
    if out[0] and out[1] and (x1 - x2) * (out[0][0] - out[1][0]) < 0:
        w, e = out[0][0], out[1][0]
    return w, s, e, n


def bounds(src_text, dst_text, box):
    src, dst = crs(src_text), crs(dst_text)
    options = osr.CoordinateTransformationOptions()
    aoi = area_of_interest(src, box)
    if aoi is not None:
        try:
            options.SetAreaOfInterest(*aoi)  # refused out of its ranges: no area of interest
        except Exception:
            pass
    ct = osr.CoordinateTransformation(src, dst, options)
    try:
        b = ct.TransformBounds(*box, 21)
    except Exception:
        return None
    return None if any(abs(v) == float("inf") for v in b) else b  # GDAL's failure: HUGE_VAL


def hexf(v):
    return float(v).hex() if v == v else "NAN"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--proj-data", default=DEFAULT_PROJ_DATA)
    ap.add_argument("--out", default=os.path.join(HERE, "bounds_reference.inc"))
    args = ap.parse_args()
    osr.SetPROJSearchPaths([args.proj_data])
    out = ["// SPDX-License-Identifier: MPL-2.0",
           "// Generated by tests/unit/crs/data/gen_bounds_reference.py: do not edit.",
           f"// GDAL {gdal.__version__} (osr, PROJ {osr.GetPROJVersionMajor()}.{osr.GetPROJVersionMinor()}."
           f"{osr.GetPROJVersionMicro()}); proj.db of this build.",
           "static const BoundsCase kBoundsCases[] = {"]
    for name, src, dst, box in CASES:
        b = bounds(src, dst, box)
        boxText = ", ".join(hexf(v) for v in box)
        if b is None:
            out.append(f'    {{"{name}", "{src}", "{dst}", {{{boxText}}}, false, {{0, 0, 0, 0}}}},')
        else:
            out.append(f'    {{"{name}", "{src}", "{dst}", {{{boxText}}}, true, {{{", ".join(hexf(v) for v in b)}}}}},')
        print(name, b)
    out.append("};")
    with open(args.out, "w") as f:
        f.write("\n".join(out) + "\n")


if __name__ == "__main__":
    main()
