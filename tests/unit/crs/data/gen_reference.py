#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Generates crs_reference.inc: reference transforms for core/crs, from GDAL and from cs2cs.

    python3 tests/unit/crs/data/gen_reference.py [--proj-data DIR] [--laz FILE]

References (the system tools of the development host; versions recorded in the output):
  - GDAL's osr (python bindings): OGRCoordinateTransformation with the area of interest that
    GDALComputeAreaOfInterest gives for the source extent, traditional GIS axis order: what GDAL's
    warper does for a raster in that CRS;
  - cs2cs (PROJ CLI) with --bbox = that area of interest (authority axis order handled here).
Both are pointed at the proj.db of this build (--proj-data, default the vcpkg_installed of the
linux-gcc-release preset), so that the PROJ *code* versions are compared, not two EPSG database
versions; the script also reports the difference the system proj.db makes.
The WKT-only CRS comes from the WKT VLR of a LAZ file of the corpus (read only: the header and
the VLRs are parsed; nothing is written there).
"""

import argparse
import os
import random
import struct
import subprocess
import sys

from osgeo import gdal, osr

gdal.UseExceptions()
osr.UseExceptions()

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
DEFAULT_PROJ_DATA = os.path.join(REPO, "build", "linux-gcc-release", "vcpkg_installed", "x64-linux-ddbt", "share",
                                 "proj")
DEFAULT_LAZ = "/mnt/data/datasets/orthos/toconvert/GH-P4_point_clouds_2025-06-06_02-03-15.laz"


def las_wkt(path):
    """The OGC WKT VLR (LASF_Projection 2112) of a LAS/LAZ file, reading only the header."""
    with open(path, "rb") as f:
        h = f.read(375)
        if h[:4] != b"LASF":
            raise SystemExit(f"{path}: not a LAS file")
        header_size = struct.unpack("<H", h[94:96])[0]
        nvlr = struct.unpack("<I", h[100:104])[0]
        f.seek(header_size)
        for _ in range(nvlr):
            vh = f.read(54)
            user = vh[2:18].rstrip(b"\0")
            rid = struct.unpack("<H", vh[18:20])[0]
            length = struct.unpack("<H", vh[20:22])[0]
            data = f.read(length)
            if user == b"LASF_Projection" and rid == 2112:
                return data.rstrip(b"\0").decode("ascii")
    raise SystemExit(f"{path}: no WKT VLR")


def srs(text):
    s = osr.SpatialReference()
    if s.SetFromUserInput(text) != 0:
        raise SystemExit(f"cannot parse {text[:60]}")
    s.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    return s


def compute_aoi(src, minx, miny, maxx, maxy):
    """GDALComputeAreaOfInterest (gdaltransformer.cpp) with a north-up extent."""
    horiz = src.Clone()
    if horiz.IsCompound():
        horiz.StripVertical()
    if horiz.IsGeocentric():
        return None
    geog = horiz.CloneGeogCS()
    geog.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    geog.SetAngularUnits(osr.SRS_UA_DEGREE, float(osr.SRS_UA_DEGREE_CONV))
    ct = osr.CoordinateTransformation(horiz, geog)
    corners = [(minx, maxy), (maxx, maxy), (minx, miny), (maxx, miny)]
    out = []
    for x, y in corners:
        try:
            r = ct.TransformPoint(x, y)
            out.append((r[0], r[1]) if all(map(lambda v: abs(v) < 1e300, r[:2])) else None)
        except RuntimeError:
            out.append(None)
    valid = [p for p in out if p]
    if not valid:
        return None
    w = min(p[0] for p in valid)
    s = min(p[1] for p in valid)
    e = max(p[0] for p in valid)
    n = max(p[1] for p in valid)
    if out[0] and out[1] and out[0][0] > out[1][0]:
        w, e = out[0][0], out[1][0]
    if abs(w) > 180 or abs(e) > 180 or abs(s) > 90 or abs(n) > 90:
        return None
    return (w, s, e, n)


def gdal_transform(src, dst, aoi, pts):
    opts = osr.CoordinateTransformationOptions()
    if aoi:
        opts.SetAreaOfInterest(*aoi)
    ct = osr.CreateCoordinateTransformation(src, dst, opts)
    return [tuple(r[:3]) for r in ct.TransformPoints([list(p) for p in pts])]


def authority_order_swaps(s):
    """True if the CRS's authority axis order is lat/long (or northing/easting)."""
    return bool(s.EPSGTreatsAsLatLong() or s.EPSGTreatsAsNorthingEasting())


def cs2cs(src_text, dst_text, src, dst, aoi, pts, env):
    args = ["cs2cs", "-f", "%.12f"]
    if aoi:
        args += ["--bbox", ",".join(repr(v) for v in aoi)]
    args += [src_text, dst_text]
    swap_in = authority_order_swaps(src)
    swap_out = authority_order_swaps(dst)
    lines = []
    for x, y, z in pts:
        a, b = (y, x) if swap_in else (x, y)
        lines.append(f"{a!r} {b!r} {z!r}")
    r = subprocess.run(args, input="\n".join(lines) + "\n", capture_output=True, text=True, env=env, check=True)
    out = []
    for line in r.stdout.strip().splitlines():
        v = line.split()
        if v[0] == "*":
            out.append((float("inf"), float("inf"), 0.0))
            continue
        a, b, c = float(v[0]), float(v[1]), float(v[2])
        out.append((b, a, c) if swap_out else (a, b, c))
    return out


def cstr(s):
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--proj-data", default=DEFAULT_PROJ_DATA)
    ap.add_argument("--laz", default=DEFAULT_LAZ)
    ap.add_argument("--out", default=os.path.join(HERE, "crs_reference.inc"))
    args = ap.parse_args()

    stereo_wkt = las_wkt(args.laz)
    rng = random.Random(4326)

    # name, src, dst, extent (source units), z range
    cases = [
        ("bd72_belgian_lambert72", "EPSG:31370", "EPSG:3857", (140000, 160000, 160000, 180000), (0, 0)),
        ("bd72_to_wgs84", "EPSG:31370", "EPSG:4326", (140000, 160000, 160000, 180000), (0, 0)),
        ("sweref99_13_30", "EPSG:3008", "EPSG:3857", (140000, 6390000, 160000, 6410000), (0, 0)),
        ("rgf93_lambert93", "EPSG:2154", "EPSG:3857", (640000, 6850000, 665000, 6875000), (0, 0)),
        ("utm17n", "EPSG:32617", "EPSG:3857", (310000, 4450000, 330000, 4470000), (0, 0)),
        ("wgs84_to_webmercator", "EPSG:4326", "EPSG:3857", (-179.9, -85.0, 179.9, 85.0), (0, 0)),
        ("webmercator_to_wgs84", "EPSG:3857", "EPSG:4326", (-2e7, -2e7, 2e7, 2e7), (0, 0)),
        ("nad83_2011_wa_south_ftus", "EPSG:6599", "EPSG:3857", (1500000, 300000, 1700000, 600000), (0, 0)),
        ("ecef_to_geographic3d", "EPSG:4978", "EPSG:4979", None, (0, 0)),
        ("rd_nap_compound_to_webmercator", "EPSG:7415", "EPSG:3857", (150000, 400000, 160000, 410000), (-5, 50)),
        ("rd_nap_compound_to_geographic3d", "EPSG:7415", "EPSG:4979", (150000, 400000, 160000, 410000), (-5, 50)),
        ("wkt_oblique_stereographic_las", stereo_wkt, "EPSG:3857", (565212.0969, 671863.705, 569779.712, 674729.3051),
         (462, 522)),
    ]

    env_same = dict(os.environ, PROJ_DATA=args.proj_data, PROJ_NETWORK="OFF")
    env_sys = {k: v for k, v in os.environ.items() if k not in ("PROJ_DATA", "PROJ_LIB")}
    env_sys["PROJ_NETWORK"] = "OFF"

    out = []
    w = out.append
    w("// SPDX-License-Identifier: MPL-2.0")
    w("// Generated by tests/unit/crs/data/gen_reference.py: do not edit.")
    cs2cs_version = subprocess.run(["cs2cs"], capture_output=True, text=True).stderr.splitlines()[0].strip()
    w(f"// GDAL {gdal.__version__} (osr, PROJ {osr.GetPROJVersionMajor()}.{osr.GetPROJVersionMinor()}."
      f"{osr.GetPROJVersionMicro()}); cs2cs {cs2cs_version}.")
    w(f"// proj.db: {os.path.relpath(args.proj_data, REPO)} (the one this build links).")
    w(f"// WKT-only CRS: WKT VLR of {os.path.basename(args.laz)}.")

    table = []
    report = []
    for idx, (name, src_text, dst_text, extent, zr) in enumerate(cases):
        osr.SetPROJSearchPath(args.proj_data)
        src = srs(src_text)
        dst = srs(dst_text)
        if extent is None:  # ECEF: points on the ellipsoid from lon/lat/h
            geo = srs("EPSG:4979")
            ct = osr.CoordinateTransformation(geo, src)
            lls = [(rng.uniform(-180, 180), rng.uniform(-80, 80), rng.uniform(-100, 3000)) for _ in range(24)]
            pts = [tuple(ct.TransformPoint(*p)[:3]) for p in lls]
            aoi = None
        else:
            minx, miny, maxx, maxy = extent
            pts = [(minx, miny), (maxx, maxy), (minx, maxy), (maxx, miny), ((minx + maxx) / 2, (miny + maxy) / 2)]
            pts += [(rng.uniform(minx, maxx), rng.uniform(miny, maxy)) for _ in range(19)]
            pts = [(x, y, rng.uniform(*zr) if zr[1] > zr[0] else 0.0) for x, y in pts]
            aoi = compute_aoi(src, minx, miny, maxx, maxy)
        ref_gdal = gdal_transform(src, dst, aoi, pts)
        ref_cs2cs = cs2cs(src_text, dst_text, src, dst, aoi, pts, env_same)
        # Same with the system proj.db (report only)
        osr.SetPROJSearchPath("/usr/share/proj")
        ref_sys = gdal_transform(srs(src_text), srs(dst_text), aoi, pts)
        d_sys = max(max(abs(a[i] - b[i]) for i in range(2)) for a, b in zip(ref_gdal, ref_sys))
        d_cs = max(max(abs(a[i] - b[i]) for i in range(3)) for a, b in zip(ref_gdal, ref_cs2cs))
        report.append(f"{name}: |gdal - cs2cs| max {d_cs:.3g}, |vcpkg proj.db - system proj.db| max {d_sys:.3g}")

        def arr(label, rows):
            w(f"static const double k{label}{idx}[] = {{")
            for r in rows:
                w("    " + ", ".join(float(v).hex() if abs(v) != float("inf") else "HUGE_VAL" for v in r) + ",")
            w("};")

        arr("In", pts)
        arr("Gdal", ref_gdal)
        arr("Cs2cs", ref_cs2cs)
        a = aoi if aoi else (0, 0, 0, 0)
        ext = extent if extent else (0, 0, 0, 0)
        table.append(f"    {{{cstr(name)}, {cstr(src_text)}, {cstr(dst_text)}, {1 if extent else 0}, "
                     f"{{{', '.join(float(v).hex() for v in ext)}}}, {1 if aoi else 0}, "
                     f"{{{', '.join(float(v).hex() for v in a)}}}, {len(pts)}, kIn{idx}, kGdal{idx}, kCs2cs{idx}}},")
    w("static const CrsCase kCrsCases[] = {")
    out.extend(table)
    w("};")
    for r in report:
        w("// " + r)
        print(r)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
