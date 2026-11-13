#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Render the corpus page (Markdown) from the corpus manifest (written by corpus-manifest.py).

Usage: corpus_docs.py [--manifest FILE] [--out FILE]
"""
import argparse
import json
import os


def human(n):
    for unit, div in (("GB", 1e9), ("MB", 1e6), ("KB", 1e3)):
        if n >= div:
            return "%.1f %s" % (n / div, unit)
    return "%d B" % n


def count(n):
    if n >= 1e9:
        return "%.2f G" % (n / 1e9)
    if n >= 1e6:
        return "%.1f M" % (n / 1e6)
    return "{:,}".format(n)


def crs_text(c):
    if not c:
        return "none"
    code = c.get("code")
    name = c["name"]
    if code and code != "EPSG:None":
        return "%s (%s)" % (code, name)
    return name


def zoom_text(z):
    if not z or "maxZ" not in z:
        return "n/a"
    t = "%s..%d" % (z.get("minZ", "?"), z["maxZ"])
    if z.get("basis", "").startswith("estimated"):
        t += " *"
    return t


def raster_row(it):
    p = it["probe"]
    nd = [v for v in p["nodata"] if v is not None]
    flags = [f.lower() for f in p["mask_flags"] if f not in ("ALL_VALID",)]
    comp = "%s, %s %dx%d" % (p["compression"] or "none", p["layout"], p["block"][0], p["block"][1])
    extra = []
    extra.append("COG" if p["is_cog"] else "not COG")
    extra.append("%s" % p["tiff_flavour"])
    extra.append("%d overviews" % len(p["overviews"]))
    if flags:
        extra.append("mask: " + "+".join(flags))
    if nd:
        extra.append("nodata %s" % nd[0])
    return "| %s | %s | %s | %dx%d, %d x %s | %s | %s | %s | %s |" % (
        it["id"], it["role"], human(it["size"]), p["size"][0], p["size"][1], p["bands"],
        "/".join(p["types"]), crs_text(p["crs"]), comp + "; " + ", ".join(extra), zoom_text(p["zoom"]),
        ", ".join(it["tiers"]))


def cloud_row(it):
    p = it["probe"]
    bits = []
    bits.append("COPC" if p["copc"] else ("LAZ" if p["laz"] else "LAS"))
    bits.append("%s PDRF %d" % (p["format"], p["pdrf"]))
    attrs = []
    if p["has_rgb"]:
        attrs.append("RGB")
    if p["has_nir"]:
        attrs.append("NIR")
    if p["extra_bytes_per_point"]:
        attrs.append("%d extra bytes" % p["extra_bytes_per_point"])
    if attrs:
        bits.append("+".join(attrs))
    fp = p["footprint_3857"]
    ext = ""
    if p.get("zoom") and "extent_3857_m" in p["zoom"]:
        w, h = p["zoom"]["extent_3857_m"]
        ext = "%.0f x %.0f m" % (w, h)
    elif "error" in fp:
        ext = "n/a (%s)" % fp["error"].split("(")[0].strip()
    return "| %s | %s | %s | %s | %s | %s | %s | %s |" % (
        it["id"], it["role"], human(it["size"]), count(p["count"]), ", ".join(bits),
        crs_text(p["crs"]), ext, zoom_text(p["zoom"]) if p.get("zoom") else "n/a")


HEAD = """# Benchmark corpus

The corpus is read **in place**: nothing is copied, nothing is written next to the files (`GDAL_PAM_ENABLED=NO`,
read-only opens, and `scripts/corpus-manifest.py` compares `(path, size, mtime)` of every file and the listing of
its folder before and after the run). The manifest is `/mnt/data/ddb-tilers-work/corpus/manifest.json`
(schema `{{"version":1,"generated":...,"items":[{{"id","kind","role","path","size","sha256","tiers","probe"}}]}}`,
plus top-level `tiers` and `tool`). Regenerate it, and this page, with

```
python3 -I scripts/corpus-manifest.py                  # sha256 cached by (path, size, mtime) in corpus/sha-cache.json
python3 -I scripts/corpus_docs.py                      # rewrites $DDBT_WORK/corpus/CORPUS.md from the manifest
```

Generated {generated} with GDAL {gdal} (system python bindings; the baseline snapshot ships GDAL 3.13.3, which is only
used by the baseline runs). Suites: `bench/suites/{{smoke,std,full}}.toml`.

Tiers: **smoke** = {smoke}; **std** = {std}; **full** = all {n} items.

"Zoom" is the tile zoom range libddb 2.4.1 computes (`GDALTiler`: `maxZ = zoomForPixelSize(pixel size in EPSG:3857)`,
`minZ = zoomForPixelSize(pixelSize * max(w, h) / 256)`; `PointCloudTiler`: `minZ = zoomForLength(min extent in 3857)`, and
`maxZ = 22` as the Hub requests). For a raster that is not already a 3857 COG, libddb tiles the COG it builds first, so
the range is an **estimate** (marked `*`) from the output grid of `GDALSuggestedWarpOutput` and the COG driver's
`UPPER` zoom strategy (the first level at least as fine as the source). Rasters in EPSG:3857 use their own pixel size.

## Rasters

| Id | Role | Size | Pixels, bands | CRS | Encoding | Zoom | Tiers |
|---|---|---|---|---|---|---|---|
"""

MID = """
## Point clouds

| Id | Role | Size | Points | Format | CRS | Footprint in 3857 | Zoom |
|---|---|---|---|---|---|---|---|
"""

NOTES = """
## Notes found while probing (measured on libddb 2.4.1 unless said otherwise)

- **R4 is not georeferenced** (no CRS, no geotransform, `LAYOUT=COG`, 5 x UInt16). `DDBMemoryTileFmt` fails with
  "Cannot geoproject ..., not a GeoImage" and `ddb cog` with "input file has no coordinate reference system". It is in the
  smoke and std tiers, but the harness skips tile and build runs for it (`[item.R4]` in the suites): use it for
  reader, band and stretch work.
- **R8 replaced.** The former R8 (`orthos/err-mask/Ortofoto 22dec2025_masked.tif`) has all TileOffsets and TileByteCounts
  zero: its 4.2 GB of data is unreachable and GDAL reads every tile as blank. R8 is now Hancock Plaza (2.3 GB DEFLATE, no overviews).
- **R7 and R9 are already COGs, but in UTM, not 3857**: libddb warps them to a 3857 COG (R9 at z24, 0.0093 m pixels, 124k x 80k).
- **R10 has 1-row strips** (block 23942 x 1), no overviews: the worst case for a block-oriented reader.
- **E1** (eBird, 52 x Float32, custom sinusoidal CRS without an EPSG code) has a footprint far outside the EPSG:3857 range
  once transformed (the sinusoidal grid wraps the poles); its zoom range is the output grid of the warp, only informative.
- **E2** is a global 4326 raster: its footprint is clipped to the Web Mercator latitude limits.
- **P6 has no CRS** in its header (TerraScan, LAS 1.4, PDRF 1, 6 extra bytes): no EPSG:3857 footprint, so `PointCloudTiler`
  refuses it ("footprint unavailable", read from `pctiler.cpp`, not measured: a baseline build of 1.33 G points comes first).
  Build only.
- **E3's header says EPSG:4978** (geocentric) **but the coordinates are local** (x -88..212 m, y -106..270 m, z -5..32 m, scale 0.001, offset -1000).
  Measured: the baseline builds it to COPC (3.8 s wall, 20 s CPU, 95 MB) and tiles it at rows far beyond 2^z. Build only.
- **P2 and P3 are COPC already** (Untwine output); building them again is a COPC -> COPC rebuild. P2 is in feet (the
  baseline's "LOD in feet vs metres" quirk shows here); P2 and E5 are compound CRSs (horizontal + vertical).
- **E4 is in degrees** (scale 1e-7), **E5** is PDRF 8 (RGB + NIR) with 8 extra bytes per point.
- Raw LAS/LAZ items (P1, P4, P5, P6, E3, E4, E5) cannot be tiled directly: `DDBMemoryTileFmt` on a raw `.laz` fails with
  "not a GeoImage" (measured on P1). A tile run uses the COPC that `ddb copc` / `DDBBuild` made first (Untwine by default, PDAL with
  `DDB_COPC_BACKEND=pdal`), so the harness builds it once per backend. Raw rasters are different: the first tile call builds a COG
  into `DDB_HOME/.ddb/artefacts/` (measured on a copy of brighton-beach.tif), which is why a fresh `DDB_HOME` makes that tile slow.
- sha256 values are in the manifest. The big files (R9 7.3 GB, P6 9.9 GB) are hashed once and cached.

## Probing method

- Rasters: `osgeo.gdal` (read-only `OpenEx`, no statistics, no pixel reads), isCOG from the `LAYOUT=COG` image-structure item, mask
  flags of band 1, overview counts, block size, first 4 bytes for classic vs BigTIFF. Footprint: the four corners with 32
  points per edge through `osr` to EPSG:3857.
- Point clouds: the LAS 1.x header, VLRs and EVLR headers are parsed directly (PDRF, point count including the 1.4 extended count,
  scale, offset, bounds, WKT or GeoTIFF keys, COPC info). The baseline `.deb` ships `libpdalcpp` but **no `pdal` executable**, and
  `pdal info --summary` reads exactly this header for LAS/LAZ/COPC, so the manifest does not depend on one. The point data is never
  read (no `--stats`). Footprint: the XY bounds box with 32 points per edge through `osr` to EPSG:3857.
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", default="/mnt/data/ddb-tilers-work/corpus/manifest.json")
    ap.add_argument("--out", default=os.path.join(os.environ.get("DDBT_WORK", "/mnt/data/ddb-tilers-work"), "corpus", "CORPUS.md"))
    a = ap.parse_args()
    m = json.load(open(a.manifest))
    items = m["items"]
    out = HEAD.format(generated=m["generated"], gdal=m["tool"]["gdal"], smoke=", ".join(m["tiers"]["smoke"]),
                      std=", ".join(m["tiers"]["std"]), n=len(items))
    out += "\n".join(raster_row(i) for i in items if i["kind"] == "raster" and "error" not in i["probe"])
    out += "\n" + MID.rstrip(" \n") + "\n"
    out += "\n".join(cloud_row(i) for i in items if i["kind"] == "pointcloud" and "error" not in i["probe"])
    out += "\n" + NOTES
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w") as f:
        f.write(out)
    print("wrote", os.path.abspath(a.out))


if __name__ == "__main__":
    main()
