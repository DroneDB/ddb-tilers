#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""GDAL reference data for the TIFF reader parity tests (tests/unit/tiff/gdal_parity_test.cpp).

Writes under the work directory (never into the repository, never next to the inputs):

  <out>/synthetic/*.tif     small GeoTIFFs written by GDAL with every codec, sample type,
                            interleave, predictor, byte order, strips / tiles, BigTIFF, masks,
                            overviews and sparse tiles the reader handles
  <out>/index.tsv           one line per raster: id, path, size, mtime, tier
  <out>/blocks/<id>.tsv     one line per (directory, band, GDAL block window):
                            key width height band x0 y0 w h crc32 adler32
                            key = main | ovr | mask | ovrmask; band 0 is the mask
  <out>/georef.tsv          id, has_gt, geotransform (6 x %.17g), WKT2_2019 of GDAL's CRS
                            (newlines escaped as \\n), "-" when there is none
  <out>/dumps/<id>/...      raw windows of JPEG directories (for max abs diff reports)

Inputs: the rasters of the corpus manifest, every GeoTIFF under test_data/ortho/edge and
test_data/testdata/raster, and the synthetic files. Opened read-only with GDAL_PAM_ENABLED=NO,
so no .aux.xml is ever written next to an input.

Large directories (more than 400 blocks) are sampled: 200 random blocks (seeded) plus the four
corner blocks. Everything else is read in full.

Run:  python3 -I tests/unit/tiff/data/gdal_reference.py [--out DIR] [--skip-big] [--only ID,...]
"""

import argparse
import json
import os
import random
import sys
import zlib

os.environ["GDAL_PAM_ENABLED"] = "NO"
os.environ.setdefault("GDAL_CACHEMAX", "512")

import numpy as np  # noqa: E402
from osgeo import gdal, osr  # noqa: E402

gdal.UseExceptions()
osr.UseExceptions()
gdal.SetConfigOption("GDAL_PAM_ENABLED", "NO")

WORK = "/mnt/data/ddb-tilers-work"
MANIFEST = WORK + "/corpus/manifest.json"
TEST_DATA = "/mnt/data/repos/test_data"
SAMPLE_LIMIT = 400
SAMPLE_COUNT = 200
JPEG_DUMPS_PER_DIR = 4


# ------------------------------------------------------------------------------------------
# Synthetic fixtures

def pattern(width, height, bands, dtype, seed):
    """Smooth gradients with noise and flat areas: compressible but not trivial."""
    rng = np.random.default_rng(seed)
    y, x = np.mgrid[0:height, 0:width].astype(np.float64)
    out = []
    for b in range(bands):
        v = (np.sin(x / (7.0 + 3 * b)) + np.cos(y / (11.0 + b)) + 2.0) / 4.0
        v += rng.normal(0, 0.03, size=v.shape)
        v[(x // 32 + y // 32) % 5 == 0] = 0.5  # flat areas
        v = np.clip(v, 0, 1)
        if np.issubdtype(dtype, np.integer):
            info = np.iinfo(dtype)
            lo, hi = float(info.min), float(info.max)
            if info.bits >= 32:
                lo, hi = max(lo, -2.0e9), min(hi, 4.0e9)
            out.append((lo + v * (hi - lo)).astype(dtype))
        else:
            out.append(((v - 0.3) * 2000.0).astype(dtype))
    return out


GDT = {
    "Byte": (gdal.GDT_Byte, np.uint8),
    "Int8": (gdal.GDT_Int8, np.int8),
    "UInt16": (gdal.GDT_UInt16, np.uint16),
    "Int16": (gdal.GDT_Int16, np.int16),
    "UInt32": (gdal.GDT_UInt32, np.uint32),
    "Int32": (gdal.GDT_Int32, np.int32),
    "UInt64": (gdal.GDT_UInt64, np.uint64),
    "Int64": (gdal.GDT_Int64, np.int64),
    "Float16": (gdal.GDT_Float16, np.float16),
    "Float32": (gdal.GDT_Float32, np.float32),
    "Float64": (gdal.GDT_Float64, np.float64),
}


def mem_dataset(width, height, bands, typename, seed, epsg=32633, nodata=None, alpha=False):
    gdt, dtype = GDT[typename]
    ds = gdal.GetDriverByName("MEM").Create("", width, height, bands, gdt)
    ds.SetGeoTransform([500000.0 + seed, 0.5, 0.0, 4500000.0, 0.0, -0.5])
    srs = osr.SpatialReference()
    srs.ImportFromEPSG(epsg)
    ds.SetSpatialRef(srs)
    arrays = pattern(width, height, bands, dtype, seed)
    for b in range(bands):
        band = ds.GetRasterBand(b + 1)
        a = arrays[b]
        if alpha and b == bands - 1:
            a = np.where((np.arange(width)[None, :] + np.arange(height)[:, None]) % 97 < 70, 255, 0).astype(dtype)
            band.SetColorInterpretation(gdal.GCI_AlphaBand)
        band.WriteArray(a)
        if nodata is not None:
            band.SetNoDataValue(nodata)
    return ds


SYNTHETIC = [
    # name, width, height, bands, type, creation options, extra
    ("none_u8_strip", 301, 203, 1, "Byte", ["COMPRESS=NONE"], {}),
    ("none_u8x3_tiled", 301, 203, 3, "Byte", ["COMPRESS=NONE", "TILED=YES", "BLOCKXSIZE=64", "BLOCKYSIZE=32"], {}),
    ("lzw_u8_strip7", 301, 203, 1, "Byte", ["COMPRESS=LZW", "BLOCKYSIZE=7"], {}),
    ("lzw_u8x4_pred2", 300, 200, 4, "Byte", ["COMPRESS=LZW", "PREDICTOR=2", "TILED=YES", "BLOCKXSIZE=128", "BLOCKYSIZE=64"], {}),
    ("lzw_u8x3_band_pred2", 300, 200, 3, "Byte", ["COMPRESS=LZW", "PREDICTOR=2", "INTERLEAVE=BAND", "TILED=YES"], {}),
    ("lzw_u16x5_pred2", 260, 190, 5, "UInt16", ["COMPRESS=LZW", "PREDICTOR=2", "TILED=YES", "BLOCKXSIZE=128", "BLOCKYSIZE=128"], {}),
    ("lzw_i16_pred2_be", 260, 190, 1, "Int16", ["COMPRESS=LZW", "PREDICTOR=2", "ENDIANNESS=BIG", "TILED=YES"], {}),
    ("lzw_u32x2_pred2", 200, 150, 2, "UInt32", ["COMPRESS=LZW", "PREDICTOR=2"], {}),
    ("lzw_i32_pred2_be_band", 200, 150, 3, "Int32", ["COMPRESS=LZW", "PREDICTOR=2", "ENDIANNESS=BIG", "INTERLEAVE=BAND"], {}),
    ("lzw_u64_pred2", 130, 70, 1, "UInt64", ["COMPRESS=LZW", "PREDICTOR=2", "TILED=YES", "BLOCKXSIZE=32", "BLOCKYSIZE=32"], {}),
    ("lzw_i64x2_pred2_be", 130, 70, 2, "Int64", ["COMPRESS=LZW", "PREDICTOR=2", "ENDIANNESS=BIG"], {}),
    ("lzw_i8", 130, 70, 1, "Int8", ["COMPRESS=LZW", "PREDICTOR=2"], {}),
    ("lzw_f32_pred3", 300, 200, 1, "Float32", ["COMPRESS=LZW", "PREDICTOR=3", "TILED=YES"], {"nodata": -9999.0}),
    ("lzw_f32x3_pred3_be", 300, 200, 3, "Float32", ["COMPRESS=LZW", "PREDICTOR=3", "ENDIANNESS=BIG", "TILED=YES", "BLOCKXSIZE=64", "BLOCKYSIZE=64"], {}),
    ("lzw_f64_pred3", 150, 100, 2, "Float64", ["COMPRESS=LZW", "PREDICTOR=3", "TILED=YES", "BLOCKXSIZE=32", "BLOCKYSIZE=32"], {}),
    ("lzw_f16", 150, 100, 1, "Float16", ["COMPRESS=LZW", "PREDICTOR=1"], {}),
    ("deflate_u8x3", 301, 203, 3, "Byte", ["COMPRESS=DEFLATE", "TILED=YES", "BLOCKXSIZE=64", "BLOCKYSIZE=64"], {}),
    ("deflate_u16_pred2_bigtiff", 301, 203, 1, "UInt16", ["COMPRESS=DEFLATE", "PREDICTOR=2", "BIGTIFF=YES", "TILED=YES"], {}),
    ("deflate_f32x52_pixel", 70, 50, 52, "Float32", ["COMPRESS=DEFLATE", "TILED=YES", "BLOCKXSIZE=32", "BLOCKYSIZE=32", "BIGTIFF=YES"], {"nodata": float("-inf")}),
    ("deflate_f64_pred3_be_bigtiff", 150, 100, 1, "Float64", ["COMPRESS=DEFLATE", "PREDICTOR=3", "ENDIANNESS=BIG", "BIGTIFF=YES"], {}),
    ("zstd_u8x4", 300, 200, 4, "Byte", ["COMPRESS=ZSTD", "TILED=YES"], {"alpha": True}),
    ("zstd_f32_pred3", 300, 200, 1, "Float32", ["COMPRESS=ZSTD", "PREDICTOR=3", "TILED=YES", "BLOCKXSIZE=128", "BLOCKYSIZE=128"], {}),
    ("zstd_u16x3_band_pred2_strips", 300, 200, 3, "UInt16", ["COMPRESS=ZSTD", "PREDICTOR=2", "INTERLEAVE=BAND", "BLOCKYSIZE=16"], {}),
    ("packbits_u8", 301, 203, 1, "Byte", ["COMPRESS=PACKBITS", "TILED=YES", "BLOCKXSIZE=32", "BLOCKYSIZE=32"], {}),
    ("packbits_u8x3_strips", 301, 203, 3, "Byte", ["COMPRESS=PACKBITS", "BLOCKYSIZE=5"], {}),
    ("lerc_f32", 300, 200, 1, "Float32", ["COMPRESS=LERC", "TILED=YES"], {"nodata": -9999.0}),
    ("lerc_deflate_f32_lossy", 300, 200, 1, "Float32", ["COMPRESS=LERC_DEFLATE", "MAX_Z_ERROR=0.5", "TILED=YES"], {}),
    ("lerc_zstd_u16x3", 300, 200, 3, "UInt16", ["COMPRESS=LERC_ZSTD", "TILED=YES", "BLOCKXSIZE=64", "BLOCKYSIZE=64"], {}),
    ("lerc_u8x4_alpha", 300, 200, 4, "Byte", ["COMPRESS=LERC", "TILED=YES"], {"alpha": True}),
    ("lerc_f64x2_band_strips", 150, 100, 2, "Float64", ["COMPRESS=LERC", "INTERLEAVE=BAND", "BLOCKYSIZE=8"], {}),
    ("lerc_i16_strips", 150, 100, 1, "Int16", ["COMPRESS=LERC_ZSTD", "BLOCKYSIZE=8"], {}),
    ("webp_u8x3", 300, 200, 3, "Byte", ["COMPRESS=WEBP", "TILED=YES", "WEBP_LEVEL=80"], {}),
    ("webp_u8x4_lossless", 300, 200, 4, "Byte", ["COMPRESS=WEBP", "TILED=YES", "WEBP_LOSSLESS=YES"], {"alpha": True}),
    ("jpeg_ycbcr", 300, 200, 3, "Byte", ["COMPRESS=JPEG", "PHOTOMETRIC=YCBCR", "TILED=YES"], {}),
    ("jpeg_ycbcr_strips", 300, 203, 3, "Byte", ["COMPRESS=JPEG", "PHOTOMETRIC=YCBCR", "BLOCKYSIZE=16"], {}),
    ("jpeg_rgb", 300, 200, 3, "Byte", ["COMPRESS=JPEG", "PHOTOMETRIC=RGB", "TILED=YES", "JPEG_QUALITY=90"], {}),
    ("jpeg_gray_band", 300, 200, 2, "Byte", ["COMPRESS=JPEG", "INTERLEAVE=BAND", "TILED=YES", "JPEGTABLESMODE=3"], {}),
    ("jpeg_rgba", 300, 200, 4, "Byte", ["COMPRESS=JPEG", "TILED=YES", "JPEGTABLESMODE=0"], {"alpha": True}),
    ("nbits1", 301, 203, 1, "Byte", ["NBITS=1", "COMPRESS=DEFLATE"], {"nbits1": True}),
    ("palette_lzw", 301, 203, 1, "Byte", ["COMPRESS=LZW", "TILED=YES"], {"palette": True}),
    ("sparse_u8_nodata", 300, 200, 1, "Byte", ["COMPRESS=DEFLATE", "TILED=YES", "BLOCKXSIZE=64", "BLOCKYSIZE=64", "SPARSE_OK=TRUE"], {"nodata": 7, "sparse": True}),
    ("sparse_f32_nodata", 300, 200, 1, "Float32", ["COMPRESS=LZW", "TILED=YES", "BLOCKXSIZE=64", "BLOCKYSIZE=64", "SPARSE_OK=TRUE"], {"nodata": -32767.0, "sparse": True}),
]

COGS = [
    # name, bands, type, COG creation options, extra
    ("cog_jpeg_mask", 3, "Byte", ["COMPRESS=JPEG", "BLOCKSIZE=128", "OVERVIEWS=AUTO"], {"mask": True}),
    ("cog_deflate_alpha_bigtiff", 4, "Byte", ["COMPRESS=DEFLATE", "BLOCKSIZE=128", "BIGTIFF=YES"], {"alpha": True}),
    ("cog_lzw_f32_nodata", 1, "Float32", ["COMPRESS=LZW", "PREDICTOR=YES", "BLOCKSIZE=128"], {"nodata": -32767.0, "sparse": True}),
    ("cog_webp_mask_be", 3, "Byte", ["COMPRESS=WEBP", "BLOCKSIZE=128"], {"mask": True}),
    ("cog_zstd_u16_band", 3, "UInt16", ["COMPRESS=ZSTD", "BLOCKSIZE=128", "INTERLEAVE=BAND"], {}),
]


# Georeferencing fixtures: tiny rasters whose GeoKeys GDAL writes from these definitions (every
# coordinate transformation libgeotiff knows, units, prime meridians, TOWGS84, vertical,
# geocentric and local CRSs, ESRI PE strings), plus PixelIsPoint, rotation, GCPs and world files.
GEOREF = [
    ("tm_intl", "+proj=tmerc +lat_0=10 +lon_0=-3 +k=0.9993 +x_0=200000 +y_0=-100 +ellps=intl +units=m +no_defs", {}),
    ("tm_usft", "+proj=tmerc +lat_0=31 +lon_0=-110.1666666666667 +k=0.9999 +x_0=213360 +y_0=0 +datum=NAD83 +units=us-ft +no_defs", {}),
    ("tmso", 'PROJCS["tmso",GEOGCS["WGS 84",DATUM["WGS_1984",SPHEROID["WGS 84",6378137,298.257223563]],PRIMEM["Greenwich",0],UNIT["degree",0.0174532925199433]],PROJECTION["Transverse_Mercator_South_Orientated"],PARAMETER["latitude_of_origin",0],PARAMETER["central_meridian",25],PARAMETER["scale_factor",1],PARAMETER["false_easting",0],PARAMETER["false_northing",0],UNIT["metre",1]]', {}),
    ("lcc2sp_ft", "+proj=lcc +lat_1=33 +lat_2=45 +lat_0=39 +lon_0=-96 +x_0=0 +y_0=0 +datum=NAD83 +units=ft +no_defs", {}),
    ("lcc1sp", "+proj=lcc +lat_1=46.8 +lat_0=46.8 +lon_0=2.337229166666667 +k_0=0.99987742 +x_0=600000 +y_0=2200000 +ellps=clrk80ign +pm=paris +towgs84=-168,-60,320,0,0,0,0 +units=m +no_defs", {}),
    ("aea", "+proj=aea +lat_1=29.5 +lat_2=45.5 +lat_0=23 +lon_0=-96 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs", {}),
    ("laea", "+proj=laea +lat_0=52 +lon_0=10 +x_0=4321000 +y_0=3210000 +ellps=GRS80 +units=m +no_defs", {}),
    ("merc1sp", "+proj=merc +lon_0=10 +k=0.99 +x_0=100 +y_0=200 +datum=WGS84 +units=m +no_defs", {}),
    ("merc2sp", "+proj=merc +lat_ts=30 +lon_0=-20 +x_0=0 +y_0=0 +ellps=bessel +units=m +no_defs", {}),
    ("stere_polar_n", "+proj=stere +lat_0=90 +lat_ts=70 +lon_0=-45 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs", {}),
    ("stere_polar_a", "+proj=stere +lat_0=-90 +lon_0=0 +k=0.994 +x_0=2000000 +y_0=2000000 +datum=WGS84 +units=m +no_defs", {}),
    ("sterea", "+proj=sterea +lat_0=52.15616055555555 +lon_0=5.38763888888889 +k=0.9999079 +x_0=155000 +y_0=463000 +ellps=bessel +units=m +no_defs", {}),
    ("stere", "+proj=stere +lat_0=40 +lon_0=-100 +k=1 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs", {}),
    ("omerc", "+proj=omerc +lat_0=4 +lonc=115 +alpha=53.31582047222222 +gamma=53.13010236111111 +k=0.99984 +x_0=590476.87 +y_0=442857.65 +ellps=evrstSS +units=m +no_defs", {}),
    ("omerc_ac", "+proj=omerc +lat_0=46.5 +lonc=7.5 +alpha=30 +k=1 +x_0=0 +y_0=0 +ellps=bessel +units=m +no_defs", {}),
    ("eqc", "+proj=eqc +lat_ts=30 +lat_0=0 +lon_0=10 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs", {}),
    ("cass", "+proj=cass +lat_0=31.73409694444445 +lon_0=35.21208055555556 +x_0=170251.555 +y_0=126867.909 +a=6378300.789 +b=6356566.435 +units=m +no_defs", {}),
    ("poly", "+proj=poly +lat_0=0 +lon_0=-54 +x_0=5000000 +y_0=10000000 +ellps=GRS80 +units=m +no_defs", {}),
    ("aeqd", "+proj=aeqd +lat_0=40 +lon_0=-100 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs", {}),
    ("mill", "+proj=mill +lat_0=0 +lon_0=0 +x_0=0 +y_0=0 +R_A +datum=WGS84 +units=m +no_defs", {}),
    ("ortho", "+proj=ortho +lat_0=45 +lon_0=10 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs", {}),
    ("gnom", "+proj=gnom +lat_0=45 +lon_0=10 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs", {}),
    ("robin", "+proj=robin +lon_0=0 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs", {}),
    ("sinu", "+proj=sinu +lon_0=0 +x_0=0 +y_0=0 +R=6371007.181 +units=m +no_defs", {}),
    ("vandg", "+proj=vandg +lon_0=0 +x_0=0 +y_0=0 +R_A +datum=WGS84 +units=m +no_defs", {}),
    ("cea", "+proj=cea +lon_0=0 +lat_ts=30 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs", {}),
    ("eqdc", "+proj=eqdc +lat_0=40 +lon_0=-96 +lat_1=20 +lat_2=60 +x_0=0 +y_0=0 +datum=NAD83 +units=m +no_defs", {}),
    ("nzmg", "+proj=nzmg +lat_0=-41 +lon_0=173 +x_0=2510000 +y_0=6023150 +ellps=intl +towgs84=59.47,-5.04,187.44,0.47,-0.1,1.024,-4.5993 +units=m +no_defs", {}),
    ("mollweide_esri_pe", "ESRI:54009", {}),
    ("geog_grad_paris", 'GEOGCS["NTF (Paris)",DATUM["Nouvelle_Triangulation_Francaise_Paris",SPHEROID["Clarke 1880 (IGN)",6378249.2,293.4660212936269],TOWGS84[-168,-60,320,0,0,0,0]],PRIMEM["Paris",2.5969213],UNIT["grad",0.01570796326794897]]', {}),
    ("geog_user_sphere", "+proj=longlat +R=6371000 +no_defs", {}),
    ("geog_epsg4269", "EPSG:4269", {}),
    ("epsg27700", "EPSG:27700", {}),
    ("epsg3035", "EPSG:3035", {}),
    ("epsg2263_ftus", "EPSG:2263", {}),
    ("compound_2154_5720", "EPSG:2154+5720", {}),
    ("compound_4326_5773", "EPSG:4326+5773", {}),
    ("geog3d_4979", "EPSG:4979", {}),
    ("geocentric_4978", "EPSG:4978", {}),
    ("local_ft", 'LOCAL_CS["arbitrary",UNIT["foot",0.3048]]', {}),
    ("pixelispoint", "EPSG:32633", {"point": True}),
    ("rotated", "EPSG:32633", {"rotated": True}),
    ("gcps", "EPSG:4326", {"gcps": True}),
    ("worldfile", None, {"worldfile": True}),
    ("user_tm_be", "+proj=tmerc +lat_0=0 +lon_0=9 +k=0.9996 +x_0=500000 +y_0=0 +ellps=WGS84 +units=m +no_defs", {"be": True}),
]


def build_georef(outdir):
    os.makedirs(outdir, exist_ok=True)
    paths = []
    gtiff = gdal.GetDriverByName("GTiff")
    for name, definition, extra in GEOREF:
        path = os.path.join(outdir, "georef_" + name + ".tif")
        paths.append(path)
        if os.path.exists(path):
            continue
        src = gdal.GetDriverByName("MEM").Create("", 8, 6, 1, gdal.GDT_Byte)
        src.GetRasterBand(1).WriteArray(np.arange(48, dtype=np.uint8).reshape(6, 8))
        if definition is not None:
            srs = osr.SpatialReference()
            srs.SetFromUserInput(definition)
        else:
            srs = None
        if extra.get("gcps"):
            gcps = [gdal.GCP(10.0 + i, 45.0 - j, 0.0, i * 7.0, j * 5.0) for i in range(2) for j in range(2)]
            src.SetGCPs(gcps, srs)
        else:
            if srs is not None:
                src.SetSpatialRef(srs)
            if extra.get("rotated"):
                src.SetGeoTransform([500000.0, 0.866, 0.5, 4500000.0, 0.5, -0.866])
            elif not extra.get("worldfile"):
                src.SetGeoTransform([123456.5, 2.0, 0.0, 654321.25, 0.0, -3.0])
        if extra.get("point"):
            src.SetMetadataItem("AREA_OR_POINT", "Point")
        co = ["ENDIANNESS=BIG"] if extra.get("be") else []
        gtiff.CreateCopy(path, src, options=co)
        if extra.get("worldfile"):
            with open(path[:-4] + ".tfw", "w") as f:
                f.write("2.5\n0.0\n0.0\n-2.5\n1000.25\n2000.75\n")
    return paths


def build_synthetic(outdir):
    os.makedirs(outdir, exist_ok=True)
    paths = []
    gtiff = gdal.GetDriverByName("GTiff")
    for i, (name, w, h, bands, typename, co, extra) in enumerate(SYNTHETIC):
        path = os.path.join(outdir, name + ".tif")
        paths.append(path)
        if os.path.exists(path):
            continue
        src = mem_dataset(w, h, bands, typename, seed=i, nodata=extra.get("nodata"), alpha=extra.get("alpha", False))
        if extra.get("sparse"):
            # A rectangle of whole nodata blocks: GDAL leaves them unwritten with SPARSE_OK.
            band = src.GetRasterBand(1)
            a = band.ReadAsArray()
            a[0:128, 64:192] = extra["nodata"]
            band.WriteArray(a)
        if extra.get("nbits1"):
            band = src.GetRasterBand(1)
            band.WriteArray((band.ReadAsArray() > 128).astype(np.uint8))
        if extra.get("palette"):
            band = src.GetRasterBand(1)
            ct = gdal.ColorTable()
            for k in range(256):
                ct.SetColorEntry(k, (k, 255 - k, (k * 7) % 256, 255))
            band.SetColorTable(ct)
        ds = gtiff.CreateCopy(path, src, options=co)
        ds = None
    for i, (name, bands, typename, co, extra) in enumerate(COGS):
        path = os.path.join(outdir, name + ".tif")
        paths.append(path)
        if os.path.exists(path):
            continue
        src = mem_dataset(700, 500, bands, typename, seed=100 + i, epsg=3857, nodata=extra.get("nodata"),
                          alpha=extra.get("alpha", False))
        if extra.get("mask"):
            src.CreateMaskBand(gdal.GMF_PER_DATASET)
            m = np.where((np.arange(700)[None, :] - np.arange(500)[:, None]) % 211 < 150, 255, 0).astype(np.uint8)
            m[0:256, 0:256] = 0  # whole blocks transparent
            src.GetRasterBand(1).GetMaskBand().WriteArray(m)
        if extra.get("sparse"):
            band = src.GetRasterBand(1)
            a = band.ReadAsArray()
            a[0:256, 0:384] = extra["nodata"]
            band.WriteArray(a)
        if name.endswith("_be"):
            co = co + ["ENDIANNESS=BIG"]
        try:
            gdal.GetDriverByName("COG").CreateCopy(path, src, options=co)
        except RuntimeError:
            # COG driver refuses some options (ENDIANNESS): fall back to plain options.
            gdal.GetDriverByName("COG").CreateCopy(path, src, options=[o for o in co if not o.startswith("ENDIANNESS")])
    return paths


def build_bench(outdir):
    """Real imagery re-encoded by GDAL for the codec benchmarks: a 8192 x 8192 window of R7
    (RGBA orthophoto) as LZW and ZSTD (level 9, GDAL's default) with predictor 2, 512 x 512
    tiles. No corpus raster is ZSTD, and R8 (the only LZW RGBA one) has no tile data."""
    os.makedirs(outdir, exist_ok=True)
    with open(MANIFEST) as f:
        man = json.load(f)
    src = next((it["path"] for it in man["items"] if it["id"] == "R7"), None)
    paths = []
    if src is None:
        return paths
    for name, co in (("r7_lzw_pred2", ["COMPRESS=LZW", "PREDICTOR=2"]),
                     ("r7_zstd_pred2", ["COMPRESS=ZSTD", "PREDICTOR=2", "ZSTD_LEVEL=9"])):
        path = os.path.join(outdir, name + ".tif")
        paths.append(path)
        if os.path.exists(path):
            continue
        ds = gdal.OpenEx(src, gdal.OF_RASTER | gdal.OF_READONLY)
        x0, y0 = (ds.RasterXSize - 8192) // 2, (ds.RasterYSize - 8192) // 2
        gdal.Translate(path, ds, srcWin=[x0, y0, 8192, 8192],
                       creationOptions=co + ["TILED=YES", "BLOCKXSIZE=512", "BLOCKYSIZE=512"])
    return paths


# ------------------------------------------------------------------------------------------
# Reference windows

def directories(ds):
    """(key, gdal object to read bands from, band list, mask band or None) for each IFD GDAL shows."""
    out = []
    b1 = ds.GetRasterBand(1)
    bands = [ds.GetRasterBand(i + 1) for i in range(ds.RasterCount)]
    out.append(("main", ds.RasterXSize, ds.RasterYSize, bands))
    for o in range(b1.GetOverviewCount()):
        ov = [b.GetOverview(o) for b in bands]
        out.append(("ovr", ov[0].XSize, ov[0].YSize, ov))
    if b1.GetMaskFlags() == gdal.GMF_PER_DATASET:
        m = b1.GetMaskBand()
        out.append(("mask", m.XSize, m.YSize, [m]))
        for o in range(m.GetOverviewCount()):
            mo = m.GetOverview(o)
            out.append(("ovrmask", mo.XSize, mo.YSize, [mo]))
    return out


def windows(band, rng):
    bw, bh = band.GetBlockSize()
    w, h = band.XSize, band.YSize
    nx, ny = (w + bw - 1) // bw, (h + bh - 1) // bh
    total = nx * ny
    if total <= SAMPLE_LIMIT:
        picks = [(x, y) for y in range(ny) for x in range(nx)]
    else:
        s = set(rng.sample(range(total), SAMPLE_COUNT))
        s.update([0, nx - 1, (ny - 1) * nx, total - 1])
        picks = [(k % nx, k // nx) for k in sorted(s)]
    for bx, by in picks:
        x0, y0 = bx * bw, by * bh
        yield x0, y0, min(bw, w - x0), min(bh, h - y0)


def reference(rid, path, outdir, compression):
    rng = random.Random(sum(map(ord, rid)) * 7919 + len(rid))
    ds = gdal.OpenEx(path, gdal.OF_RASTER | gdal.OF_READONLY)
    lines = []
    dumps = 0
    dumpdir = os.path.join(outdir, "dumps", rid)
    for key, w, h, bands in directories(ds):
        wins = list(windows(bands[0], rng))
        for bi, band in enumerate(bands):
            bandno = 0 if key in ("mask", "ovrmask") else bi + 1
            for k, (x0, y0, ww, hh) in enumerate(wins):
                data = band.ReadRaster(x0, y0, ww, hh)
                crc = zlib.crc32(data) & 0xFFFFFFFF
                adl = zlib.adler32(data) & 0xFFFFFFFF
                lines.append(f"{key}\t{w}\t{h}\t{bandno}\t{x0}\t{y0}\t{ww}\t{hh}\t{crc}\t{adl}\n")
                if "JPEG" in compression and key in ("main", "ovr") and k < JPEG_DUMPS_PER_DIR:
                    os.makedirs(dumpdir, exist_ok=True)
                    with open(os.path.join(dumpdir, f"{key}_{w}x{h}_b{bandno}_{x0}_{y0}_{ww}x{hh}.raw"), "wb") as f:
                        f.write(data)
                    dumps += 1
    with open(os.path.join(outdir, "blocks", rid + ".tsv"), "w") as f:
        f.writelines(lines)
    return len(lines)


def georef(rid, path):
    ds = gdal.OpenEx(path, gdal.OF_RASTER | gdal.OF_READONLY)
    gt = ds.GetGeoTransform(can_return_null=True)
    srs = ds.GetSpatialRef()
    if srs is None and gt is None and ds.GetGCPCount() > 0:
        srs = ds.GetGCPSpatialRef()
    wkt = srs.ExportToWkt(["FORMAT=WKT2_2019"]).replace("\n", "\\n") if srs is not None else "-"
    gts = "\t".join("%.17g" % v for v in gt) if gt is not None else "\t".join(["-"] * 6)
    return f"{rid}\t{1 if gt is not None else 0}\t{gts}\t{wkt}\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=WORK + "/fixtures/tiff")
    ap.add_argument("--skip-big", action="store_true", help="skip corpus rasters larger than 1 GB")
    ap.add_argument("--only", default="", help="comma-separated ids")
    args = ap.parse_args()
    out = args.out
    os.makedirs(os.path.join(out, "blocks"), exist_ok=True)

    items = []  # (id, path, tier)
    with open(MANIFEST) as f:
        man = json.load(f)
    for it in man["items"]:
        if it["kind"] == "raster":
            items.append((it["id"], it["path"], "corpus-big" if it["size"] > (1 << 30) else "corpus"))
    seen = {p for _, p, _ in items}
    for d in (TEST_DATA + "/ortho/edge", TEST_DATA + "/testdata/raster"):
        for name in sorted(os.listdir(d)):
            p = os.path.join(d, name)
            if p in seen or not name.lower().endswith((".tif", ".tiff", ".cog")):
                continue
            items.append(("td_" + os.path.basename(d) + "_" + os.path.splitext(name)[0], p, "testdata"))
            seen.add(p)
    for p in build_synthetic(os.path.join(out, "synthetic")):
        items.append(("syn_" + os.path.splitext(os.path.basename(p))[0], p, "synthetic"))
    for p in build_bench(os.path.join(out, "bench")):
        items.append(("bench_" + os.path.splitext(os.path.basename(p))[0], p, "bench"))
    for p in build_georef(os.path.join(out, "synthetic")):
        items.append(("syn_" + os.path.splitext(os.path.basename(p))[0], p, "synthetic"))

    only = set(filter(None, args.only.split(",")))
    index_lines = []
    georef_lines = []
    for rid, path, tier in items:
        st = os.stat(path)
        index_lines.append(f"{rid}\t{path}\t{st.st_size}\t{int(st.st_mtime)}\t{tier}\n")
        georef_lines.append(georef(rid, path))
        if only and rid not in only:
            continue
        if args.skip_big and tier == "corpus-big":
            continue
        ds = gdal.OpenEx(path, gdal.OF_RASTER | gdal.OF_READONLY)
        comp = ds.GetMetadata("IMAGE_STRUCTURE").get("COMPRESSION", "NONE")
        ds = None
        n = reference(rid, path, out, comp)
        print(f"{rid}: {n} windows ({comp})", flush=True)
    with open(os.path.join(out, "index.tsv"), "w") as f:
        f.writelines(index_lines)
    with open(os.path.join(out, "georef.tsv"), "w") as f:
        f.writelines(georef_lines)
    print(f"wrote {out}/index.tsv ({len(index_lines)} rasters)")


if __name__ == "__main__":
    sys.exit(main())
