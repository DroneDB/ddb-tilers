#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Deterministic tile request sets for ddbt-bench (JSON lines).

Scenarios
  sweep   every tile of each zoom minZ..maxZ intersecting the EPSG:3857 footprint, capped at
          --cap tiles per zoom by a deterministic stride (row-major order)
  pan     random walks (seeded) of a 1920x1080 viewport (8x5 tiles): each step pans 1-3 tiles
          in one of 8 directions, or zooms +-1 with p=0.2; the newly visible tiles of a step are
          emitted centre-out (a client keeps the tiles it already has). Walks of --steps steps
          are added until at least --min-requests requests exist.
  retina  pan with 512 px tiles (same 8x5 tile grid: a 2x device-pixel-ratio screen)
  edge    half-covered border tiles, tiles just outside the footprint, far away tiles,
          over-zoom (maxZ+1, +2) and under-zoom (minZ-1, -2) tiles

Rows are XYZ (y = 0 at the north edge); requests carry "tms": true, the value DroneDB's
Registry passes with XYZ rows (libddb's tms flag inverts y internally).

Usage
  requests.py gen --dataset R1 --scenario sweep [--manifest M] [--seed 1] --out R1-sweep.jsonl
  requests.py gen --path FILE --footprint minx,miny,maxx,maxy --minz 14 --maxz 21 --scenario pan ...
  requests.py info --dataset R1
"""
import argparse
import json
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common  # noqa: E402

R = 6378137.0
O = common.ORIGIN_SHIFT  # pi * R
MAX_LAT = 85.0511287798066

VIEW_W, VIEW_H = 1920, 1080


# --------------------------------------------------------------------------- tile maths

def lonlat_to_merc(lon, lat):
    lat = max(-MAX_LAT, min(MAX_LAT, lat))
    x = R * math.radians(lon)
    y = R * math.atanh(math.sin(math.radians(lat)))  # = R ln tan(pi/4 + lat/2), exact at 0
    return x, y


def merc_to_lonlat(x, y):
    lon = math.degrees(x / R)
    lat = math.degrees(2 * math.atan(math.exp(y / R)) - math.pi / 2)
    return lon, lat


def tile_span(z):
    """Width of a tile at zoom z in EPSG:3857 metres."""
    return 2 * O / (1 << z)


def lonlat_to_tile(lon, lat, z):
    """XYZ tile containing lon/lat (mercantile.tile semantics, clamped to the world)."""
    n = 1 << z
    lat = max(-MAX_LAT, min(MAX_LAT, lat))
    x = (lon + 180.0) / 360.0 * n
    s = math.sin(math.radians(lat))
    y = (0.5 - 0.25 * math.log((1 + s) / (1 - s)) / math.pi) * n
    return min(n - 1, max(0, int(math.floor(x)))), min(n - 1, max(0, int(math.floor(y))))


def merc_to_tile(mx, my, z):
    """XYZ tile containing EPSG:3857 metres (clamped to the world)."""
    n = 1 << z
    span = tile_span(z)
    x = int(math.floor((mx + O) / span))
    y = int(math.floor((O - my) / span))
    return min(n - 1, max(0, x)), min(n - 1, max(0, y))


def tile_bounds(x, y, z):
    """EPSG:3857 bounds (minx, miny, maxx, maxy) of XYZ tile x/y/z (mercantile.xy_bounds)."""
    span = tile_span(z)
    minx = -O + x * span
    maxy = O - y * span
    return minx, maxy - span, minx + span, maxy


def tile_ul_lonlat(x, y, z):
    """Upper-left corner lon/lat of XYZ tile (mercantile.ul)."""
    n = 1 << z
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
    return lon, lat


def xyz_to_tms(y, z):
    return (1 << z) - 1 - y


def tile_range(fp, z):
    """Inclusive XYZ tile range (x0, x1, y0, y1) of the tiles that intersect footprint fp
    [minx, miny, maxx, maxy] with a positive area, clamped to the world."""
    n = 1 << z
    span = tile_span(z)
    x0 = int(math.floor((fp[0] + O) / span))
    x1 = int(math.ceil((fp[2] + O) / span)) - 1
    y0 = int(math.floor((O - fp[3]) / span))
    y1 = int(math.ceil((O - fp[1]) / span)) - 1
    x1, y1 = max(x0, x1), max(y0, y1)
    return max(0, x0), min(n - 1, x1), max(0, y0), min(n - 1, y1)


# --------------------------------------------------------------------------- scenarios

def _req(path, z, x, y, size, fmt, **extra):
    r = {"path": path, "z": z, "x": x, "y": y, "size": size, "format": fmt, "tms": True}
    r.update(extra)
    return r


def stride_pick(count, cap):
    """Deterministic, evenly spread selection of min(count, cap) indices of range(count)."""
    if cap <= 0 or count <= cap:
        return list(range(count))
    return [(i * count) // cap for i in range(cap)]


def sweep(path, fp, minz, maxz, size=256, fmt="png", cap=256):
    out = []
    for z in range(minz, maxz + 1):
        x0, x1, y0, y1 = tile_range(fp, z)
        w = x1 - x0 + 1
        count = w * (y1 - y0 + 1)
        for k in stride_pick(count, cap):
            out.append(_req(path, z, x0 + k % w, y0 + k // w, size, fmt, kind="sweep"))
    return out


def viewport_grid(view_w=VIEW_W, view_h=VIEW_H, tile=256):
    return int(math.ceil(view_w / tile)), int(math.ceil(view_h / tile))


def pan(path, fp, minz, maxz, size=256, fmt="png", seed=1, steps=200, min_requests=2000,
        p_zoom=0.2, client_cache=True, max_walks=1000):
    """Viewport random walks; see the module docstring. Tile grid: 8x5 at 256 and at 512."""
    rng = random.Random(seed)
    cols, rows = viewport_grid()
    dirs = [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]
    out = []
    walk = 0
    while len(out) < min_requests and walk < max_walks:
        z = rng.randint(minz, maxz)
        span = tile_span(z)
        cx = (rng.uniform(fp[0], fp[2]) + O) / span  # viewport centre in tile units
        cy = (O - rng.uniform(fp[1], fp[3])) / span
        seen = set()
        for step in range(steps + 1):
            action = "start"
            if step > 0:
                if rng.random() < p_zoom:
                    dz = rng.choice((-1, 1))
                    if not (minz <= z + dz <= maxz):
                        dz = -dz
                    if minz <= z + dz <= maxz:
                        f = 2.0 ** dz
                        cx, cy, z = cx * f, cy * f, z + dz
                        action = "zoom%+d" % dz
                else:
                    dx, dy = rng.choice(dirs)
                    m = rng.randint(1, 3)
                    cx, cy = cx + dx * m, cy + dy * m
                    action = "pan"
                # keep the centre over the data (a user pans around the dataset)
                x0, x1, y0, y1 = tile_range(fp, z)
                cx = min(max(cx, x0), x1 + 1)
                cy = min(max(cy, y0), y1 + 1)
            n = 1 << z
            tx0 = int(math.floor(cx - cols / 2))
            ty0 = int(math.floor(cy - rows / 2))
            new = []
            for ty in range(ty0, ty0 + rows):
                for tx in range(tx0, tx0 + cols):
                    if not (0 <= tx < n and 0 <= ty < n):
                        continue
                    key = (z, tx, ty)
                    if client_cache and key in seen:
                        continue
                    seen.add(key)
                    new.append((tx, ty))
            new.sort(key=lambda t: ((t[0] + 0.5 - cx) ** 2 + (t[1] + 0.5 - cy) ** 2, t[1], t[0]))
            for tx, ty in new:
                out.append(_req(path, z, tx, ty, size, fmt, kind="pan", walk=walk, step=step, action=action))
        walk += 1
    return out


def edge(path, fp, minz, maxz, size=256, fmt="png", max_zooms=6, include_invalid=False):
    out = []
    seen = set()

    def add(z, x, y, kind):
        n = 1 << z
        if not include_invalid and not (0 <= x < n and 0 <= y < n):
            return
        if (z, x, y) in seen:
            return
        seen.add((z, x, y))
        out.append(_req(path, z, x, y, size, fmt, kind=kind))

    zooms = list(range(minz, maxz + 1))
    zooms = [zooms[i] for i in stride_pick(len(zooms), max_zooms)]
    if maxz not in zooms:
        zooms.append(maxz)
    for z in zooms:
        x0, x1, y0, y1 = tile_range(fp, z)
        xm, ym = (x0 + x1) // 2, (y0 + y1) // 2
        for x, y in ((x0, y0), (x1, y0), (x0, y1), (x1, y1), (xm, y0), (xm, y1), (x0, ym), (x1, ym)):
            add(z, x, y, "border")
        for x, y in ((x0 - 1, ym), (x1 + 1, ym), (xm, y0 - 1), (xm, y1 + 1), (x0 - 1, y0 - 1), (x1 + 1, y1 + 1)):
            add(z, x, y, "outside")
        n = 1 << z
        add(z, (xm + n // 2) % n, ym, "far")
        add(z, xm, (ym + n // 2) % n, "far")
    cxm, cym = (fp[0] + fp[2]) / 2, (fp[1] + fp[3]) / 2
    for z in (maxz + 1, maxz + 2):
        if z <= 30:
            x, y = merc_to_tile(cxm, cym, z)
            add(z, x, y, "overzoom")
    for z in (minz - 1, minz - 2):
        if z >= 0:
            x, y = merc_to_tile(cxm, cym, z)
            add(z, x, y, "underzoom")
    if include_invalid:
        add(maxz, 1 << maxz, 0, "invalid")
        add(maxz, 0, -1, "invalid")
    return out


SCENARIOS = ("sweep", "pan", "retina", "edge")


def generate(scenario, path, fp, minz, maxz, fmt="png", seed=1, cap=256, steps=200, min_requests=2000,
             size=None, include_invalid=False):
    if scenario == "sweep":
        return sweep(path, fp, minz, maxz, size or 256, fmt, cap)
    if scenario == "pan":
        return pan(path, fp, minz, maxz, size or 256, fmt, seed, steps, min_requests)
    if scenario == "retina":
        return pan(path, fp, minz, maxz, size or 512, fmt, seed, steps, min_requests)
    if scenario == "edge":
        return edge(path, fp, minz, maxz, size or 256, fmt, include_invalid=include_invalid)
    raise ValueError("unknown scenario %r (expected one of %s)" % (scenario, ", ".join(SCENARIOS)))


def write_jsonl(reqs, out):
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    tmp = out + ".tmp"
    with open(tmp, "w") as f:
        for r in reqs:
            f.write(json.dumps(r, separators=(",", ":")) + "\n")
    os.replace(tmp, out)


def resolve(args):
    """(path, footprint, minz, maxz) from --dataset (manifest) or --path/--footprint."""
    if args.dataset:
        items, _ = common.load_manifest(args.manifest)
        if args.dataset not in items:
            raise SystemExit("dataset %s not in %s" % (args.dataset, args.manifest))
        item = items[args.dataset]
        fp, minz, maxz = common.dataset_geometry(item)
        if fp is None:
            raise SystemExit("dataset %s has no EPSG:3857 footprint in the manifest" % args.dataset)
        path = item["path"]
    else:
        if not (args.path and args.footprint):
            raise SystemExit("--dataset or --path with --footprint is required")
        path = args.path
        fp = [float(v) for v in args.footprint.split(",")]
        minz, maxz = None, None
    if args.minz is not None:
        minz = args.minz
    if args.maxz is not None:
        maxz = args.maxz
    if minz is None or maxz is None:
        raise SystemExit("zoom range unknown: pass --minz/--maxz")
    return path, fp, int(minz), int(maxz)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("gen", "info"):
        p = sub.add_parser(name)
        p.add_argument("--manifest", default=common.MANIFEST)
        p.add_argument("--dataset")
        p.add_argument("--path")
        p.add_argument("--footprint", help="minx,miny,maxx,maxy in EPSG:3857")
        p.add_argument("--minz", type=int)
        p.add_argument("--maxz", type=int)
        if name == "gen":
            p.add_argument("--scenario", required=True, choices=SCENARIOS)
            p.add_argument("--format", default="png")
            p.add_argument("--size", type=int)
            p.add_argument("--seed", type=int, default=1)
            p.add_argument("--cap", type=int, default=256, help="sweep: tiles per zoom")
            p.add_argument("--steps", type=int, default=200, help="pan: steps per walk")
            p.add_argument("--min-requests", type=int, default=2000, help="pan: lower bound of the set size")
            p.add_argument("--include-invalid", action="store_true", help="edge: add out-of-world tiles")
            p.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    path, fp, minz, maxz = resolve(a)
    if a.cmd == "info":
        print(json.dumps({"path": path, "footprint3857": fp, "minZ": minz, "maxZ": maxz,
                          "ranges": {z: tile_range(fp, z) for z in range(minz, maxz + 1)}}, indent=1))
        return 0
    reqs = generate(a.scenario, path, fp, minz, maxz, a.format, a.seed, a.cap, a.steps, a.min_requests,
                    a.size, a.include_invalid)
    write_jsonl(reqs, a.out)
    common.log("%s: %d requests (%s, z%d-%d)" % (a.out, len(reqs), a.scenario, minz, maxz))
    return 0


if __name__ == "__main__":
    sys.exit(main())
