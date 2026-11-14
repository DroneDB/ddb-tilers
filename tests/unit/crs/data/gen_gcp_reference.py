#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Generates gcp_reference.inc: reference GCP transforms for core/crs/gcp_transform, from GDAL.

    python3 tests/unit/crs/data/gen_gcp_reference.py [--capella FILE] [--builtin-gdal TOOL]
        [--builtin-proj-data DIR]

Not run by the build: the output is committed. The reference is GDAL's general image transformer
(the python bindings of the development host, gdal.Transformer(dataset, None, options)) on an
in-memory raster that has GCPs and no geotransform, which is what gdalwarp builds for such a
raster (GDALCreateGenImgProjTransformer2 with no target: pixel / line <-> GCP CRS). Each case
records the GCPs, the options, the points and GDAL's results both ways, or GDAL's failure.

GDAL's homography solves a 9 x 9 linear system with Armadillo (LAPACK) when GDAL is built with
it, as the GDAL of most Linux distributions is, and with its own LU decomposition otherwise (as
vcpkg's GDAL is): the two agree to about 1e-12 only. GcpTransform follows GDAL's own solver, so
with --builtin-gdal (gdal_gcp_transform.cpp, built against a GDAL without Armadillo) every case
also records that GDAL's results, which the test expects to the last bit.

The real-world case takes the 225 GCPs (EPSG:4326) of a Capella SLC GeoTIFF (CInt16, no
geotransform) and a 9 x 9 grid over the raster, transformed by GDAL on the file itself.
"""

import argparse
import math
import os
import random
import subprocess

from osgeo import gdal, osr

gdal.UseExceptions()
osr.UseExceptions()

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CAPELLA = "/mnt/data/ddb-tilers-work/fixtures/realworld/capella_slc_24mb.tif"
W, H = 1000, 800

METHODS = {"auto": (0, None), "homography": (1, "GCP_HOMOGRAPHY"), "polynomial": (2, "GCP_POLYNOMIAL")}


def utm_like(p, l):
    """A smooth, mildly non-linear pixel/line -> projected mapping (metres)."""
    return (500000.0 + 0.5 * p + 0.02 * l + 2e-6 * p * l + 1e-7 * p * p,
            4500000.0 - 0.5 * l + 0.03 * p - 3e-6 * l * l + 1e-6 * p * l)


def perspective(p, l):
    """A projective (homography) pixel/line -> projected mapping, as an oblique photo has."""
    w = 1.0 + 1.5e-4 * p - 0.8e-4 * l
    return ((320000.0 + 0.8 * p + 0.1 * l) / w, (5100000.0 + 0.05 * p - 0.9 * l) / w)


def grid_gcps(nx, ny, f, rng=None, noise=0.0, jitter=0.0):
    gcps = []
    for j in range(ny):
        for i in range(nx):
            p = (W - 1) * i / max(1, nx - 1)
            l = (H - 1) * j / max(1, ny - 1)
            if rng is not None and jitter:
                p += rng.uniform(-jitter, jitter)
                l += rng.uniform(-jitter, jitter)
            x, y = f(p, l)
            if rng is not None and noise:
                x += rng.gauss(0.0, noise)
                y += rng.gauss(0.0, noise)
            gcps.append((p, l, x, y))
    return gcps


def scattered_gcps(n, f, rng, noise=0.0):
    gcps = []
    for _ in range(n):
        p = rng.uniform(0, W)
        l = rng.uniform(0, H)
        x, y = f(p, l)
        gcps.append((p, l, x + rng.gauss(0.0, noise) if noise else x, y + rng.gauss(0.0, noise) if noise else y))
    return gcps


def lonlat_antimeridian(p, l):
    """Longitudes 178 .. 182 (written in [-180, 180]), latitudes -16 .. -18: crosses 180."""
    lon = 178.0 + 4.0 * p / W + 0.05 * l / H
    lat = -16.0 - 2.0 * l / H + 0.01 * p / W + 0.02 * (p / W) * (l / H)
    return (lon - 360.0 if lon > 180.0 else lon, lat)


def grid_points(width, height, n):
    pts = []
    for j in range(n):
        for i in range(n):
            pts.append((-0.1 * width + 1.2 * width * i / (n - 1), -0.1 * height + 1.2 * height * j / (n - 1)))
    return pts


def run_case(name, gcps, srs, method, max_order, points, ds=None, expect_kind=None, expect_order=0):
    opts = []
    code, value = METHODS[method]
    if value:
        opts.append("METHOD=" + value)
    if max_order:
        opts.append("MAX_GCP_ORDER=%d" % max_order)
    if ds is None:
        ds = gdal.GetDriverByName("MEM").Create("", W, H, 1, gdal.GDT_Byte)
        gl = [gdal.GCP(x, y, 0.0, p, l) for (p, l, x, y) in gcps]
        s = None
        if srs:
            s = osr.SpatialReference()
            s.SetFromUserInput(srs)
            s.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
        ds.SetGCPs(gl, s)
    case = {"name": name, "crs": srs or "", "method": code, "max_order": max_order, "gcps": gcps}
    try:
        tr = gdal.Transformer(ds, None, opts)
    except RuntimeError as e:
        case.update(ok=False, error=str(e), kind="", order=0, points=[], fwd=[], fwd_ok=[], inv_in=[], inv=[],
                    inv_ok=[])
        return case
    fwd, fwd_ok = tr.TransformPoints(0, [(p, l, 0.0) for (p, l) in points])
    inv_in = [(x, y) for (x, y, _z) in fwd]
    inv, inv_ok = tr.TransformPoints(1, [(x, y, 0.0) for (x, y) in inv_in])
    case.update(ok=True, error="", kind=expect_kind, order=expect_order, points=points,
                fwd=[(x, y) for (x, y, _z) in fwd], fwd_ok=list(fwd_ok), inv_in=inv_in,
                inv=[(x, y) for (x, y, _z) in inv], inv_ok=list(inv_ok))
    return case


def run_builtin(tool, proj_data, cases):
    """GDAL version and results of the reference tool (a GDAL without Armadillo), by case."""
    lines = []
    for c in cases:
        n = len(c["points"])
        lines.append("case %s %s %d %d %d %d" % (c["name"], c["crs"] or "-", c["method"], c["max_order"],
                                               len(c["gcps"]), n))
        lines += [" ".join(float.hex(float(v)) for v in g) for g in c["gcps"]]
        lines += [" ".join(float.hex(float(v)) for v in p) for p in c["points"]]
        lines += [" ".join(float.hex(float(v)) for v in p) for p in c["inv_in"]]
    env = dict(os.environ)
    if proj_data:
        env["PROJ_DATA"] = proj_data
    out = subprocess.run([tool], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True,
                         env=env).stdout
    results, cur, version = {}, None, "?"
    for line in out.splitlines():
        parts = line.split()
        if parts[0] == "gdal":
            version = parts[1]
        elif parts[0] == "case":
            cur = {"ok": parts[2] == "ok", "fwd": [], "fwd_ok": [], "inv": [], "inv_ok": []}
            results[parts[1]] = cur
        else:
            key = "fwd" if parts[0] == "f" else "inv"
            cur[key].append((float.fromhex(parts[1]), float.fromhex(parts[2])))
            cur[key + "_ok"].append(int(parts[3]))
    return version, results


def hx(v):
    if not math.isfinite(v):
        return "0.0"
    return float.hex(float(v))


def emit_array(out, cname, ctype, rows, per_row):
    out.append("static const %s %s[] = {" % (ctype, cname))
    if not rows:
        out.append("    0,")
    for r in rows:
        if ctype == "double":
            out.append("    " + ", ".join(hx(v) for v in r[:per_row]) + ",")
        else:
            out.append("    " + ", ".join(str(int(v)) for v in r) + ",")
    out.append("};")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--capella", default=DEFAULT_CAPELLA)
    ap.add_argument("--builtin-gdal", help="gdal_gcp_transform built against a GDAL without Armadillo")
    ap.add_argument("--builtin-proj-data", help="the proj.db folder of that GDAL's PROJ")
    ap.add_argument("--out", default=os.path.join(HERE, "gcp_reference.inc"))
    args = ap.parse_args()

    rng = random.Random(20261114)
    pts = grid_points(W, H, 7)
    cases = []
    add = cases.append

    # Polynomial order selection and the 2-GCP rectangle.
    add(run_case("one_gcp", [(10, 20, 500010.0, 4499990.0)], "EPSG:32633", "auto", 0, pts))
    add(run_case("two_gcps_rectangle", [(0, 0, 500000.0, 4500000.0), (W, H, 500500.0, 4499600.0)], "EPSG:32633",
                 "auto", 0, pts, expect_kind="polynomial", expect_order=1))
    add(run_case("two_gcps_same_x", [(0, 0, 500000.0, 4500000.0), (W, H, 500000.0, 4499600.0)], "EPSG:32633",
                 "auto", 0, pts))
    add(run_case("two_gcps_order2", [(0, 0, 500000.0, 4500000.0), (W, H, 500500.0, 4499600.0)], "EPSG:32633",
                 "auto", 2, pts))
    g3 = [(0, 0) + utm_like(0, 0), (W, 0) + utm_like(W, 0), (300, H) + utm_like(300, H)]
    add(run_case("three_gcps_exact", g3, "EPSG:32633", "auto", 0, pts, expect_kind="polynomial", expect_order=1))
    add(run_case("three_gcps_collinear", [(0, 0, 0.0, 0.0), (100, 100, 10.0, 10.0), (300, 300, 30.0, 30.0)],
                 "EPSG:32633", "auto", 0, pts))

    # Homography: 4 and 5 GCPs with the default method.
    g4 = [(0, 0) + perspective(0, 0), (W, 0) + perspective(W, 0), (W, H) + perspective(W, H),
          (0, H) + perspective(0, H)]
    add(run_case("four_gcps_homography", g4, "EPSG:32632", "auto", 0, pts, expect_kind="homography"))
    add(run_case("four_gcps_homography_order1", g4, "EPSG:32632", "auto", 1, pts, expect_kind="homography"))
    add(run_case("four_gcps_polynomial", g4, "EPSG:32632", "polynomial", 0, pts, expect_kind="polynomial",
                 expect_order=1))
    g5 = scattered_gcps(5, perspective, rng, noise=0.4)
    add(run_case("five_gcps_homography_noisy", g5, "EPSG:32632", "auto", 0, pts, expect_kind="homography"))
    add(run_case("four_gcps_collinear", [(0, 0, 0.0, 0.0), (100, 100, 10.0, 10.0), (200, 200, 20.0, 20.0),
                                         (300, 300, 30.0, 30.0)], "EPSG:32632", "auto", 0, pts))
    add(run_case("four_gcps_bow_tie", [(0, 0, 0.0, 0.0), (W, 0, 100.0, 0.0), (W, H, 0.0, 100.0),
                                       (0, H, 100.0, 100.0)], "EPSG:32632", "auto", 0, pts))
    add(run_case("four_gcps_flat_line", [(0, 0, 5.0, 0.0), (W, 0, 5.0, 10.0), (W, H, 5.0, 20.0),
                                         (0, H, 5.0, 30.0)], "EPSG:32632", "auto", 0, pts))

    # Polynomials from 6 GCPs on.
    g6 = scattered_gcps(6, utm_like, rng)
    add(run_case("six_gcps_exact_order2", g6, "EPSG:32633", "auto", 0, pts, expect_kind="polynomial",
                 expect_order=2))
    add(run_case("six_gcps_homography", g6, "EPSG:32633", "homography", 0, pts, expect_kind="homography"))
    g9 = grid_gcps(3, 3, utm_like, rng, noise=0.3, jitter=20.0)
    add(run_case("nine_gcps_noisy_order2", g9, "EPSG:32633", "auto", 0, pts, expect_kind="polynomial",
                 expect_order=2))
    add(run_case("nine_gcps_order3", g9, "EPSG:32633", "auto", 3, pts))
    g10 = scattered_gcps(10, utm_like, rng, noise=0.2)
    add(run_case("ten_gcps_noisy_auto", g10, "EPSG:32633", "auto", 0, pts, expect_kind="polynomial",
                 expect_order=2))
    add(run_case("ten_gcps_exact_order3", g10, "EPSG:32633", "auto", 3, pts, expect_kind="polynomial",
                 expect_order=3))
    g25 = grid_gcps(5, 5, utm_like, rng, noise=0.5, jitter=15.0)
    add(run_case("twentyfive_gcps_noisy_auto", g25, "EPSG:32633", "auto", 0, pts, expect_kind="polynomial",
                 expect_order=2))
    for order in (1, 2, 3):
        add(run_case("twentyfive_gcps_noisy_order%d" % order, g25, "EPSG:32633", "auto", order, pts,
                     expect_kind="polynomial", expect_order=order))
    add(run_case("twentyfive_gcps_order4", g25, "EPSG:32633", "auto", 4, pts))
    add(run_case("seven_gcps_collinear", [(i * 100.0, i * 50.0, 1000.0 + i * 10.0, 2000.0 - i * 5.0)
                                          for i in range(7)], "EPSG:32633", "auto", 0, pts))

    # Antimeridian: geographic GCPs on both sides of 180 degrees.
    gam = grid_gcps(5, 5, lonlat_antimeridian, rng, noise=1e-5, jitter=10.0)
    add(run_case("antimeridian_unwrapped", gam, "EPSG:4326", "auto", 0, pts, expect_kind="polynomial",
                 expect_order=2))
    gam4 = [(p, l) + lonlat_antimeridian(p, l) for (p, l) in ((0, 0), (W, 0), (W, H), (0, H))]
    add(run_case("antimeridian_homography", gam4, "EPSG:4326", "auto", 0, pts, expect_kind="homography"))
    gfar = [(0, 0) + (165.0, -16.0)] + gam[1:]
    add(run_case("antimeridian_not_unwrapped", gfar, "EPSG:4326", "auto", 0, pts, expect_kind="polynomial",
                 expect_order=2))
    add(run_case("antimeridian_projected_crs", gam, "EPSG:32760", "auto", 0, pts, expect_kind="polynomial",
                 expect_order=2))

    # The real-world raster.
    if os.path.exists(args.capella):
        ds = gdal.Open(args.capella)
        gl = ds.GetGCPs()
        gc = [(g.GCPPixel, g.GCPLine, g.GCPX, g.GCPY) for g in gl]
        cpts = grid_points(ds.RasterXSize, ds.RasterYSize, 9)
        c = run_case("capella_slc_225_gcps", gc, "EPSG:4326", "auto", 0, cpts, ds=ds, expect_kind="polynomial",
                     expect_order=2)
        cases.append(c)
    else:
        raise SystemExit("missing " + args.capella)

    builtin_version, builtin = run_builtin(args.builtin_gdal, args.builtin_proj_data, cases) if args.builtin_gdal else ("", {})
    for c in cases:
        b = builtin.get(c["name"])
        if b is not None and (b["ok"] != c["ok"] or b["fwd_ok"] != c["fwd_ok"] or b["inv_ok"] != c["inv_ok"]):
            raise SystemExit("%s: the two GDALs disagree on success" % c["name"])

    out = ["// SPDX-License-Identifier: MPL-2.0",
           "// Generated by tests/unit/crs/data/gen_gcp_reference.py: do not edit.",
           "// GDAL %s (gdal.Transformer on a raster with GCPs and no geotransform)." % gdal.__version__,
           "// Capella case: the GCPs of capella_slc_24mb.tif (Capella Open Data, CInt16 SLC)."]
    if builtin:
        out.append("// Built-in solver results (kFwdB / kInvB): GDAL %s without Armadillo (gdal_gcp_transform)."
                   % builtin_version)
    for k, c in enumerate(cases):
        emit_array(out, "kGcps%d" % k, "double", c["gcps"], 4)
        emit_array(out, "kPts%d" % k, "double", c["points"], 2)
        emit_array(out, "kFwd%d" % k, "double", c["fwd"], 2)
        emit_array(out, "kFwdOk%d" % k, "int", [c["fwd_ok"]] if c["fwd_ok"] else [], 0)
        emit_array(out, "kInv%d" % k, "double", c["inv"], 2)
        emit_array(out, "kInvOk%d" % k, "int", [c["inv_ok"]] if c["inv_ok"] else [], 0)
        b = builtin.get(c["name"])
        if b is not None and c["ok"]:
            emit_array(out, "kFwdB%d" % k, "double", b["fwd"], 2)
            emit_array(out, "kInvB%d" % k, "double", b["inv"], 2)
    out.append("static const GcpReferenceCase kGcpReferenceCases[] = {")
    for k, c in enumerate(cases):
        has_b = c["name"] in builtin and c["ok"]
        out.append('    {"%s", "%s", %d, %d, %d, kGcps%d, %s, "%s", %d, %d, kPts%d, kFwd%d, kFwdOk%d, kInv%d, kInvOk%d, %s, %s},'
                   % (c["name"], c["crs"], c["method"], c["max_order"], len(c["gcps"]), k,
                      "true" if c["ok"] else "false", c["kind"] or "", c["order"], len(c["points"]), k, k, k, k, k,
                      "kFwdB%d" % k if has_b else "nullptr", "kInvB%d" % k if has_b else "nullptr"))
        if not c["ok"]:
            out.append("    // GDAL: %s" % c["error"].replace("\n", " "))
    out.append("};")
    with open(args.out, "w") as f:
        f.write("\n".join(out) + "\n")
    for c in cases:
        print("%-32s %s %s" % (c["name"], "ok" if c["ok"] else "FAIL", c["error"]))


if __name__ == "__main__":
    main()
