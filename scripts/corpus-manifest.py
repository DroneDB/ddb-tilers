#!/usr/bin/env python3
"""Write the benchmark corpus manifest (read in place, never copied or modified).

Output (default /mnt/data/ddb-tilers-work/corpus/manifest.json):
  {"version":1, "generated":..., "tiers":{...},
   "items":[{"id","kind":"raster"|"pointcloud","role","path","size","sha256","tiers":[...],"probe":{...}}]}

* sha256 is computed once per (path, size, mtime) and cached in sha-cache.json next to the manifest
  (some files are 10 GB).
* Rasters are probed read-only with the system GDAL python bindings (GDAL_PAM_ENABLED=NO; no statistics,
  no pixel reads). Point clouds are probed by reading the LAS/LAZ/COPC header and VLRs directly (the
  baseline .deb ships libpdalcpp but no `pdal` executable; a header read is what `pdal info --summary`
  does for these formats, and never touches the points).
* The "zoom" entries reproduce the tile zoom range libddb 2.4.1 computes (GDALTiler: tMaxZ =
  zoomForPixelSize(pixel size in EPSG:3857), tMinZ = zoomForPixelSize(pixelSize * max(w, h) / 256);
  PointCloudTiler: tMinZ = zoomForLength(min extent in 3857), maxZ 22 as the Hub requests).
* Guard: (path, size, mtime) of every corpus file and the listing of its folder are compared before and
  after; the script fails if anything changed.

Usage: scripts/corpus-manifest.py [--out FILE] [--cache FILE] [--only R1,P2] [--no-sha] [--jobs N]
"""
import argparse
import concurrent.futures
import hashlib
import json
import math
import os
import struct
import sys
import time

os.environ["GDAL_PAM_ENABLED"] = "NO"
os.environ.setdefault("GDAL_NUM_THREADS", "1")
from osgeo import gdal, osr  # noqa: E402

gdal.UseExceptions()
osr.UseExceptions()
gdal.PushErrorHandler("CPLQuietErrorHandler")  # PROJ warnings on out-of-range points are handled per point

WORK = "/mnt/data/ddb-tilers-work"
TD = "/mnt/data/repos/test_data"
DS = "/mnt/data/datasets"

SMOKE = ["R1", "R3", "R4", "P1", "P2"]
STD = ["R1", "R2", "R3", "R4", "R5", "R7", "R10", "E1", "E2", "P1", "P2", "P3", "P4", "E3", "E5"]

# id, kind, role, path
ITEMS = [
    ("R1", "raster", "Small EPSG:3857 JPEG COG with mask (brighton-beach): direct block-to-tile path, smoke",
     f"{TD}/ortho/brighton-beach-cog.tif"),
    ("R2", "raster", "Medium orthophoto GeoTIFF (aukerman): build + tile",
     f"{TD}/ortho/aukerman.tif"),
    ("R3", "raster", "Belgian Lambert 72 (EPSG:31370) orthophoto (vo): non-3857 reprojection path",
     f"{TD}/ortho/vo.tif"),
    ("R4", "raster", "5-band UInt16 multispectral (merged_0180): stretch / band selection",
     f"{TD}/multispectral/merged_0180.tif"),
    ("R5", "raster", "DroneDB-built Float32 DEM COG (test-tif): single band elevation, colormap tiles",
     f"{DS}/orthos/test-tif/.ddb/build/159b8f07cdc8e83b8098466a497cfe4d4e9d4d79983259f25e705b5a30d55285/cog/cog.tif"),
    ("R6", "raster", "DroneDB 3.3 Gpx JPEG COG (err-mask): large JPEG COG with mask, tile throughput",
     f"{DS}/orthos/err-mask/.ddb/build/29b077eede0e6ece29a9e52f136bcc30e9f359d588947b50841996a11b2ce0ce/cog/cog.tif"),
    ("R7", "raster", "Lamb-Hill farmland orthophoto, 2.8 GB GeoTIFF: large build",
     f"{DS}/orthos/Lamb-Hill-Farmland.Ortho.tif"),
    # The former R8 (orthos/err-mask/Ortofoto 22dec2025_masked.tif) has all TileOffsets and
    # TileByteCounts zero: its data is unreachable and every tile reads blank, so it is replaced.
    ("R8", "raster", "Hancock Plaza: RGBA DEFLATE predictor 2, EPSG:6428 (US ft), no overviews, 2.3 GB",
     f"{DS}/orthos/Hancock Plaza 8-15-25.tif"),
    ("R9", "raster", "ortho.tif, 7.3 GB: build within a declared memory cap",
     f"{DS}/orthos/ortho.tif"),
    ("R10", "raster", "Moulin 2 cm orthophoto, 1-row strips, 1.7 GB: strip-organised source",
     f"{DS}/orthos/moulin-hub-2cm/2cm/25.10.02_moulin_cote_2cm.tif"),
    ("E1", "raster", "Edge: 52-band Float32 (eBird abundance): band-heavy build and tile",
     f"{TD}/ortho/edge/abetow-ERD2018-EBIRD_SCIENCE-20191109-a5cf4cb2_hr_2018_abundance_median.tiff"),
    ("E2", "raster", "Edge: global EPSG:4326 raster (SPAM wheat harvested area): world-extent warp",
     f"{TD}/ortho/edge/spam2005v3r2_harvested-area_wheat_total.tiff"),
    ("P1", "pointcloud", "Small LAZ (brighton-beach): COPC build + tile, smoke",
     f"{TD}/point-clouds/brighton-beach.laz"),
    ("P2", "pointcloud", "autzen-classified COPC (81 MB): COPC tile reference, classified",
     f"{DS}/point-clouds/test-crash/autzen-classified.copc.laz"),
    ("P3", "pointcloud", "LHD LAMB93 / IGN69 COPC (205 MB): projected CRS with compound vertical datum",
     f"{DS}/point-clouds/test-crash/LHD_FXX_0658_6859_PTS_O_LAMB93_IGN69.copc.laz"),
    ("P4", "pointcloud", "ODM odm_data_zoo georeferenced model (88.8 M points): COPC build",
     f"{DS}/odm-outputs/odm_data_zoo-master/odm_georeferencing/odm_georeferenced_model.laz"),
    ("P5", "pointcloud", "ODM ziegeleipark georeferenced model (669 M points): large COPC build",
     f"{DS}/odm-outputs/ziegeleipark/odm_georeferencing/odm_georeferenced_model.laz"),
    ("P6", "pointcloud", "C_19GZ2 (1.33 G points, 9.9 GB LAZ): COPC build within <= 4 GB RAM",
     f"{DS}/point-clouds/clouds/C_19GZ2 (1).LAZ"),
    ("E3", "pointcloud", "Edge: ODM toledo cloud.las, header says ECEF (EPSG:4978) with local coordinates",
     f"{DS}/odm-outputs/odm_data_toledo/cloud.las"),
    ("E4", "pointcloud", "Edge: ODM meadows-road point_cloud.laz in EPSG:4326 (degrees)",
     f"{DS}/odm-outputs/meadows-road/point_cloud.laz"),
    ("E5", "pointcloud", "Edge: point-cloud-complex.laz, PDRF 8 (RGB + NIR) with extra bytes",
     f"{TD}/point-clouds/point-cloud-complex.laz"),
]

R_EARTH = 6378137.0
INITIAL_RES = 2.0 * math.pi * R_EARTH / 256.0  # GlobalMercator.initialResolution (tileSize 256)
MERC_MAX_LAT = 85.0511287798066


# ----------------------------------------------------------------------------------------------
# libddb tile maths (src/library/tiler.cpp)

def resolution(z):
    return INITIAL_RES / (2.0 ** z)


def zoom_for_pixel_size(px):
    """GlobalMercator::zoomForPixelSize: first zoom whose resolution is strictly below px, minus one."""
    for i in range(99):
        if px > resolution(i):
            return i - 1
    return 0


def zoom_for_length(meters):
    """GlobalMercator::zoomForLength (tileSize 256)."""
    return int(round(math.log(INITIAL_RES / meters * 256.0) / math.log(2.0)))


# ----------------------------------------------------------------------------------------------
# sha256 cache

class ShaCache:
    def __init__(self, path):
        self.path = path
        try:
            with open(path) as f:
                self.data = json.load(f)
        except (OSError, ValueError):
            self.data = {}

    def get(self, path, st):
        e = self.data.get(path)
        if e and e["size"] == st.st_size and e["mtime_ns"] == st.st_mtime_ns:
            return e["sha256"]
        return None

    def put(self, path, st, digest):
        self.data[path] = {"size": st.st_size, "mtime_ns": st.st_mtime_ns, "sha256": digest}

    def save(self):
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.data, f, indent=1, sort_keys=True)
            f.write("\n")
        os.replace(tmp, self.path)


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb", buffering=0) as f:
        buf = bytearray(8 << 20)
        mv = memoryview(buf)
        while True:
            n = f.readinto(buf)
            if not n:
                break
            h.update(mv[:n])
    return h.hexdigest()


# ----------------------------------------------------------------------------------------------
# geometry helpers

def srs_3857():
    s = osr.SpatialReference()
    s.ImportFromEPSG(3857)
    s.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    return s


def traditional(srs):
    srs = srs.Clone()
    srs.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
    return srs


def densify_ring(x0, y0, x1, y1, n=32):
    """Closed ring of the rectangle (x0,y0)-(x1,y1) with n points per edge."""
    pts = []
    for i in range(n):
        pts.append((x0 + (x1 - x0) * i / n, y0))
    for i in range(n):
        pts.append((x1, y0 + (y1 - y0) * i / n))
    for i in range(n):
        pts.append((x1 - (x1 - x0) * i / n, y1))
    for i in range(n):
        pts.append((x0, y1 - (y1 - y0) * i / n))
    return pts


def footprint(srs, ring, z=0.0):
    """Transform a ring of (x, y) to EPSG:3857; returns dict with bbox, or {"error": ...}."""
    try:
        ct = osr.CoordinateTransformation(traditional(srs), srs_3857())
    except Exception as e:  # no operation available
        return {"error": "no transformation to EPSG:3857: %s" % str(e).strip()}
    xs, ys, bad = [], [], 0
    for x, y in ring:
        try:
            tx, ty, _ = ct.TransformPoint(x, y, z)
        except Exception:
            bad += 1
            continue
        if not (math.isfinite(tx) and math.isfinite(ty)):
            bad += 1
            continue
        xs.append(tx)
        ys.append(ty)
    if not xs:
        return {"error": "no point of the footprint transforms to EPSG:3857"}
    out = {"bbox": [min(xs), min(ys), max(xs), max(ys)]}
    if bad:
        out["dropped_points"] = bad
    lim = 20037508.343 * 1.0001
    if min(xs) < -lim or max(xs) > lim or min(ys) < -lim or max(ys) > lim:
        # libddb refuses such a cloud ("footprint unavailable"): no zoom range can be derived
        out["error"] = "footprint outside the EPSG:3857 range (bbox is the raw transformed extent)"
    return out


def srs_summary(srs):
    if srs is None:
        return None
    out = {"name": srs.GetName()}
    code = None
    try:
        name, c = srs.GetAuthorityName(None), srs.GetAuthorityCode(None)
        if name and c:
            code = "%s:%s" % (name, c)
        else:
            srs2 = srs.Clone()
            if srs2.AutoIdentifyEPSG() == 0:
                code = "EPSG:%s" % srs2.GetAuthorityCode(None)
    except Exception:
        pass
    out["code"] = code
    out["kind"] = ("geocentric" if srs.IsGeocentric() else "geographic" if srs.IsGeographic()
                   else "projected" if srs.IsProjected() else "other")
    if srs.IsCompound():
        out["compound"] = True
    try:
        out["linear_unit"] = srs.GetLinearUnitsName()
    except Exception:
        pass
    return out


def is_3857(srs):
    try:
        return bool(srs.IsSame(srs_3857(), ["IGNORE_DATA_AXIS_TO_SRS_AXIS_MAPPING=YES", "CRITERION=EQUIVALENT_EXCEPT_AXIS_ORDER_GEOGCRS"]))
    except Exception:
        return False


# ----------------------------------------------------------------------------------------------
# rasters

def mask_flag_names(flags):
    names = []
    for bit, n in ((gdal.GMF_ALL_VALID, "ALL_VALID"), (gdal.GMF_PER_DATASET, "PER_DATASET"),
                   (gdal.GMF_ALPHA, "ALPHA"), (gdal.GMF_NODATA, "NODATA")):
        if flags & bit:
            names.append(n)
    return names


def cog_zoom_estimate(res):
    """Pixel size and zoom range of the COG DroneDB builds from a raster of native 3857 resolution `res`.

    The COG driver (GoogleMapsCompatible, 512 blocks, ZOOM_LEVEL_STRATEGY=UPPER) picks the first level
    whose resolution is <= the native one (1e-8 tolerance), i.e. resolution(K) for the smallest K >= 1.
    """
    k = 1
    while k < 40 and not (resolution(k) <= res * (1 + 1e-8)):
        k += 1
    return resolution(k), k


def probe_raster(path):
    ds = gdal.OpenEx(path, gdal.OF_RASTER | gdal.OF_READONLY)
    w, h, nb = ds.RasterXSize, ds.RasterYSize, ds.RasterCount
    b1 = ds.GetRasterBand(1)
    gt = ds.GetGeoTransform()
    srs = ds.GetSpatialRef()
    bx, by = b1.GetBlockSize()
    types = sorted({gdal.GetDataTypeName(ds.GetRasterBand(i + 1).DataType) for i in range(nb)})
    nodatas = []
    for i in range(1, min(nb, 8) + 1):
        nd = ds.GetRasterBand(i).GetNoDataValue()
        if nd not in nodatas:
            nodatas.append(nd)
    imd = ds.GetMetadata("IMAGE_STRUCTURE") or {}
    with open(path, "rb") as f:
        magic = f.read(4)
    tiff = {b"II*\x00": "classic", b"MM\x00*": "classic", b"II+\x00": "bigtiff", b"MM\x00+": "bigtiff"}.get(magic)
    flags = b1.GetMaskFlags()
    try:
        mask_ovr = b1.GetMaskBand().GetOverviewCount()
    except Exception:
        mask_ovr = None
    ovr = []
    for i in range(b1.GetOverviewCount()):
        o = b1.GetOverview(i)
        ovr.append([o.XSize, o.YSize])
    bytes_pp = max(gdal.GetDataTypeSize(ds.GetRasterBand(i + 1).DataType) for i in range(nb)) // 8

    p = {
        "driver": ds.GetDriver().ShortName,
        "size": [w, h],
        "bands": nb,
        "types": types,
        "color_interp": [gdal.GetColorInterpretationName(ds.GetRasterBand(i + 1).GetColorInterpretation())
                         for i in range(min(nb, 8))],
        "compression": imd.get("COMPRESSION"),
        "interleave": imd.get("INTERLEAVE"),
        "tiff_flavour": tiff,
        "block": [bx, by],
        "layout": "strips" if (bx == w and by != bx) else "tiles",
        "overviews": ovr,
        "nodata": nodatas,
        "mask_flags": mask_flag_names(flags),
        "mask_overviews": mask_ovr,
        "has_palette": b1.GetColorTable() is not None,
        "is_cog": imd.get("LAYOUT") == "COG",
        "pixels": w * h,
        "uncompressed_bytes": w * h * nb * bytes_pp,
        "crs": srs_summary(srs),
        "geotransform": list(gt) if gt else None,
    }
    if nb > 8:
        p["color_interp"].append("...")
    q = imd.get("JPEG_QUALITY")
    if q:
        p["jpeg_quality"] = q

    if srs is None or gt is None:
        p["footprint_3857"] = {"error": "no CRS or geotransform"}
        p["zoom"] = None
        return p

    # footprint: the four corners with densified edges (rotated geotransforms included)
    def px(c, r):
        return gt[0] + c * gt[1] + r * gt[2], gt[3] + c * gt[4] + r * gt[5]

    geographic = srs.IsGeographic()
    corners = [px(0, 0), px(w, 0), px(w, h), px(0, h)]
    n = 32
    ring = []
    for a, b in zip(corners, corners[1:] + corners[:1]):
        for i in range(n):
            ring.append((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n))
    clipped = False
    if geographic:
        cl = []
        for x, y in ring:
            ny = min(max(y, -MERC_MAX_LAT), MERC_MAX_LAT)
            nx = min(max(x, -180.0), 180.0)
            clipped = clipped or (ny != y or nx != x)
            cl.append((nx, ny))
        ring = cl
    fp = footprint(srs, ring)
    if clipped:
        fp["clipped_to_mercator_limits"] = True
    p["footprint_3857"] = fp

    # tile zoom range of libddb (GDALTiler)
    zoom = {}
    if is_3857(srs) and abs(gt[1]) > 0 and gt[2] == 0 and gt[4] == 0:
        ps, ww, hh = abs(gt[1]), w, h
        zoom["basis"] = "native EPSG:3857 pixel size"
    else:
        ps = ww = hh = None
        try:
            vrt = gdal.AutoCreateWarpedVRT(ds, None, srs_3857().ExportToWkt(), gdal.GRA_Bilinear)
            wgt = vrt.GetGeoTransform()
            native = abs(wgt[1])
            ps, k = cog_zoom_estimate(native)
            zoom["basis"] = "estimated COG grid (GDALSuggestedWarpOutput, UPPER)"
            zoom["suggested_pixel_size_3857"] = native
            zoom["cog_zoom_level"] = k
            ww = int(round(vrt.RasterXSize * native / ps))
            hh = int(round(vrt.RasterYSize * native / ps))
            vrt = None
        except Exception as e:
            zoom["error"] = "cannot compute the 3857 output grid: %s" % str(e).strip()[:200]
    if ps:
        zoom["pixel_size_3857"] = ps
        zoom["output_size"] = [ww, hh]
        zoom["maxZ"] = zoom_for_pixel_size(ps)
        zoom["minZ"] = zoom_for_pixel_size(ps * max(ww, hh) / 256.0)
    p["zoom"] = zoom
    return p


# ----------------------------------------------------------------------------------------------
# point clouds (LAS 1.0 - 1.4, LAZ, COPC): header and VLR read only

PDRF_BASE = {0: 20, 1: 28, 2: 26, 3: 34, 4: 57, 5: 63, 6: 30, 7: 36, 8: 38, 9: 59, 10: 67}
PDRF_RGB = {2, 3, 5, 7, 8, 10}
PDRF_GPS = {1, 3, 4, 5, 6, 7, 8, 9, 10}
PDRF_NIR = {8, 10}
PDRF_WAVE = {4, 5, 9, 10}


def cstr(b):
    return b.split(b"\0", 1)[0].decode("latin-1", "replace")


def read_las(path):
    with open(path, "rb") as f:
        hdr = f.read(375)
        if hdr[:4] != b"LASF":
            raise ValueError("not a LAS/LAZ file")
        vmaj, vmin = hdr[24], hdr[25]
        gen_sw = cstr(hdr[58:90])
        hsize, off_pts, nvlr = struct.unpack_from("<HII", hdr, 94)
        pdrf_raw, reclen = hdr[104], struct.unpack_from("<H", hdr, 105)[0]
        legacy_count = struct.unpack_from("<I", hdr, 107)[0]
        scale = struct.unpack_from("<3d", hdr, 131)
        offset = struct.unpack_from("<3d", hdr, 155)
        maxx, minx, maxy, miny, maxz, minz = struct.unpack_from("<6d", hdr, 179)
        gencoding = struct.unpack_from("<H", hdr, 6)[0]
        count = legacy_count
        evlr_start, n_evlr = 0, 0
        if (vmaj, vmin) >= (1, 4) and hsize >= 375:
            evlr_start, n_evlr = struct.unpack_from("<QI", hdr, 235)
            ext = struct.unpack_from("<Q", hdr, 247)[0]
            if ext or not legacy_count:
                count = ext
        vlrs = []
        f.seek(hsize)
        for _ in range(nvlr):
            h = f.read(54)
            if len(h) < 54:
                break
            uid, rid, ln = cstr(h[2:18]), struct.unpack_from("<H", h, 18)[0], struct.unpack_from("<H", h, 20)[0]
            body = f.read(ln) if ln <= (1 << 20) else (f.seek(ln, 1) and b"")
            vlrs.append({"user_id": uid, "record_id": rid, "length": ln, "evlr": False, "body": body})
        if n_evlr and evlr_start:
            f.seek(evlr_start)
            for _ in range(n_evlr):
                h = f.read(60)
                if len(h) < 60:
                    break
                uid, rid = cstr(h[2:18]), struct.unpack_from("<H", h, 18)[0]
                ln = struct.unpack_from("<Q", h, 20)[0]
                if (uid, rid) in (("LASF_Projection", 2112), ("LASF_Projection", 34735), ("LASF_Projection", 34737),
                                  ("LASF_Projection", 34736)) and ln <= (1 << 20):
                    body = f.read(ln)
                else:
                    body = b""
                    f.seek(ln, 1)
                vlrs.append({"user_id": uid, "record_id": rid, "length": ln, "evlr": True, "body": body})
    return {
        "version": (vmaj, vmin), "software": gen_sw, "pdrf": pdrf_raw & 0x3F, "compressed_bit": bool(pdrf_raw & 0x80),
        "record_length": reclen, "count": count, "scale": scale, "offset": offset,
        "min": (minx, miny, minz), "max": (maxx, maxy, maxz), "vlrs": vlrs, "global_encoding": gencoding,
        "point_data_offset": off_pts, "n_vlr": nvlr, "n_evlr": n_evlr,
    }


def las_srs(vlrs):
    """Return (osr.SpatialReference | None, source, details) from the WKT or GeoTIFF key VLRs."""
    for v in vlrs:
        if v["user_id"] == "LASF_Projection" and v["record_id"] == 2112 and v["body"]:
            wkt = cstr(v["body"])
            try:
                s = osr.SpatialReference()
                s.ImportFromWkt(wkt)
                return s, "wkt", {}
            except Exception as e:
                return None, "wkt (unparsable: %s)" % str(e).strip()[:80], {}
    for v in vlrs:
        if v["user_id"] == "LASF_Projection" and v["record_id"] == 34735 and v["body"]:
            b = v["body"]
            if len(b) < 8:
                continue
            _, _, _, nkeys = struct.unpack_from("<4H", b, 0)
            keys = {}
            for i in range(nkeys):
                if 8 + 8 * i + 8 > len(b):
                    break
                kid, loc, cnt, val = struct.unpack_from("<4H", b, 8 + 8 * i)
                keys[kid] = (loc, cnt, val)
            model = keys.get(1024, (0, 0, None))[2]
            proj = keys.get(3072, (0, 0, None))[2]
            geog = keys.get(2048, (0, 0, None))[2]
            vert = keys.get(4096, (0, 0, None))[2]
            det = {"geotiff_model_type": {1: "projected", 2: "geographic", 3: "geocentric"}.get(model, model),
                   "projected_cs": proj, "geographic_cs": geog, "vertical_cs": vert}
            code = proj if proj not in (None, 0, 32767) else geog if geog not in (None, 0, 32767) else None
            if code:
                s = osr.SpatialReference()
                try:
                    s.ImportFromEPSG(int(code))
                    return s, "geotiff keys", det
                except Exception:
                    pass
            if model == 3:
                s = osr.SpatialReference()
                s.ImportFromEPSG(4978)  # WGS 84 geocentric: the LAS 1.x key set has no code for it
                return s, "geotiff keys (geocentric model type; EPSG:4978 assumed)", det
            return None, "geotiff keys (user-defined or missing code)", det
    return None, "none", {}


def probe_pointcloud(path):
    info = read_las(path)
    vmaj, vmin = info["version"]
    pdrf = info["pdrf"]
    vlrs = info["vlrs"]
    names = [(v["user_id"], v["record_id"]) for v in vlrs]
    copc_vlr = next((v for v in vlrs if v["user_id"] == "copc" and v["record_id"] == 1), None)
    laz = info["compressed_bit"] or ("laszip encoded", 22204) in names or path.lower().endswith((".laz",))
    base = PDRF_BASE.get(pdrf)
    p = {
        "format": "LAS %d.%d" % (vmaj, vmin),
        "software": info["software"],
        "pdrf": pdrf,
        "count": info["count"],
        "laz": bool(laz),
        "copc": copc_vlr is not None,
        "record_length": info["record_length"],
        "extra_bytes_per_point": (info["record_length"] - base) if base is not None else None,
        "has_rgb": pdrf in PDRF_RGB,
        "has_nir": pdrf in PDRF_NIR,
        "has_gps_time": pdrf in PDRF_GPS,
        "has_waveform": pdrf in PDRF_WAVE,
        "scale": list(info["scale"]),
        "offset": list(info["offset"]),
        "bounds": {"min": list(info["min"]), "max": list(info["max"])},
        "vlrs": ["%s/%d%s" % (u, r, " (EVLR)" if v["evlr"] else "") for (u, r), v in zip(names, vlrs)],
    }
    if copc_vlr and len(copc_vlr["body"]) >= 80:
        cx, cy, cz, half, spacing = struct.unpack_from("<5d", copc_vlr["body"], 0)
        p["copc_info"] = {"center": [cx, cy, cz], "halfsize": half, "spacing": spacing}

    srs, source, det = las_srs(vlrs)
    p["crs_source"] = source
    if det:
        p["crs_geotiff"] = det
    p["crs"] = srs_summary(srs)
    if srs is None:
        p["footprint_3857"] = {"error": "no CRS in the file"}
        p["zoom"] = None
        return p

    (x0, y0, z0), (x1, y1, z1) = info["min"], info["max"]
    if srs.IsGeocentric():
        fp = footprint(srs, densify_ring(x0, y0, x1, y1, 8), (z0 + z1) / 2)
    elif srs.IsGeographic():
        fp = footprint(srs, densify_ring(x0, max(y0, -MERC_MAX_LAT), x1, min(y1, MERC_MAX_LAT), 32))
    else:
        fp = footprint(srs, densify_ring(x0, y0, x1, y1, 32), (z0 + z1) / 2)
    p["footprint_3857"] = fp
    if "error" in fp:
        p["zoom"] = None
        return p

    zoom = {"maxZ": 22, "maxZ_basis": "Hub request ceiling"}
    if "bbox" in fp:
        bx0, by0, bx1, by1 = fp["bbox"]
        w, h = bx1 - bx0, by1 - by0
        zoom["extent_3857_m"] = [w, h]
        if min(w, h) > 0:
            zoom["minZ"] = zoom_for_length(min(w, h))
            zoom["minZ_basis"] = "GlobalMercator::zoomForLength(min extent in 3857)"
    p["zoom"] = zoom
    return p


# ----------------------------------------------------------------------------------------------

def folder_listing(paths):
    out = {}
    for d in sorted({os.path.dirname(p) for p in paths}):
        entries = []
        for e in sorted(os.scandir(d), key=lambda e: e.name):
            try:
                st = e.stat(follow_symlinks=False)
                entries.append((e.name, st.st_size, st.st_mtime_ns))
            except OSError:
                entries.append((e.name, None, None))
        out[d] = entries
    return out


def file_guard(paths):
    return {p: (os.stat(p).st_size, os.stat(p).st_mtime_ns) for p in paths}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=f"{WORK}/corpus/manifest.json")
    ap.add_argument("--cache", default=None, help="sha cache (default: sha-cache.json next to --out)")
    ap.add_argument("--only", default="", help="comma separated ids to include")
    ap.add_argument("--no-sha", action="store_true", help="use cached sha256 only; never read a file for hashing")
    ap.add_argument("--jobs", type=int, default=4, help="parallel sha256 workers")
    a = ap.parse_args()

    out_dir = os.path.dirname(os.path.abspath(a.out))
    os.makedirs(out_dir, exist_ok=True)
    cache = ShaCache(a.cache or os.path.join(out_dir, "sha-cache.json"))
    only = {s.strip() for s in a.only.split(",") if s.strip()}
    items = [i for i in ITEMS if not only or i[0] in only]

    missing = [(i[0], i[3]) for i in items if not os.path.isfile(i[3])]
    for iid, p in missing:
        print("MISSING %s %s" % (iid, p), file=sys.stderr)
    items = [i for i in items if os.path.isfile(i[3])]

    paths = [i[3] for i in items]
    before_files = file_guard(paths)
    before_dirs = folder_listing(paths)

    # sha256 (cached by path + size + mtime), in parallel
    stats = {p: os.stat(p) for p in paths}
    shas = {p: cache.get(p, stats[p]) for p in paths}
    todo = [p for p in paths if shas[p] is None]
    if todo and a.no_sha:
        print("--no-sha: no cached sha256 for %d file(s)" % len(todo), file=sys.stderr)
    elif todo:
        t0 = time.time()
        total = sum(stats[p].st_size for p in todo)
        print("sha256 of %d file(s), %.1f GB ..." % (len(todo), total / 1e9), file=sys.stderr)
        with concurrent.futures.ThreadPoolExecutor(max_workers=a.jobs) as ex:
            futs = {ex.submit(sha256_file, p): p for p in todo}
            for fut in concurrent.futures.as_completed(futs):
                p = futs[fut]
                shas[p] = fut.result()
                cache.put(p, stats[p], shas[p])
                cache.save()
                print("  %s  %s (%.0fs)" % (shas[p][:12], os.path.basename(p), time.time() - t0), file=sys.stderr)

    result = []
    for iid, kind, role, path in items:
        tiers = [t for t, ids in (("smoke", SMOKE), ("std", STD), ("full", None)) if ids is None or iid in ids]
        entry = {"id": iid, "kind": kind, "role": role, "path": path, "size": stats[path].st_size,
                 "sha256": shas[path], "tiers": tiers}
        try:
            entry["probe"] = probe_raster(path) if kind == "raster" else probe_pointcloud(path)
        except Exception as e:
            entry["probe"] = {"error": "%s: %s" % (type(e).__name__, str(e).strip()[:300])}
            print("probe failed for %s: %s" % (iid, e), file=sys.stderr)
        result.append(entry)
        print("probed %s" % iid, file=sys.stderr)

    after_files = file_guard(paths)
    after_dirs = folder_listing(paths)
    if before_files != after_files or before_dirs != after_dirs:
        print("GUARD FAILURE: a corpus file or its folder changed during the run", file=sys.stderr)
        for p in paths:
            if before_files[p] != after_files[p]:
                print("  changed:", p, file=sys.stderr)
        for d in before_dirs:
            if before_dirs[d] != after_dirs[d]:
                print("  folder listing changed:", d, file=sys.stderr)
        sys.exit(3)

    manifest = {
        "version": 1,
        "generated": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "tool": {"gdal": gdal.__version__, "script": "scripts/corpus-manifest.py"},
        "tiers": {"smoke": [i for i in SMOKE], "std": [i for i in STD], "full": [i[0] for i in ITEMS]},
        "items": result,
    }
    tmp = a.out + ".tmp"
    with open(tmp, "w") as f:
        json.dump(manifest, f, indent=1)
        f.write("\n")
    os.replace(tmp, a.out)
    print("wrote %s: %d items%s" % (a.out, len(result), ", %d missing" % len(missing) if missing else ""))
    sys.exit(2 if missing else 0)


if __name__ == "__main__":
    main()
