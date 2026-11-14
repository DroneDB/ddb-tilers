#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Writes the gen_*.jp2 files of this folder with the system GDAL's JP2OpenJPEG driver (seeded, the
same files every time), for the cases GDAL's own samples do not cover: a colour table (pclr box),
4:2:0 YCbCr, lossy multi-tile RGB with the YCC transform, RGBA with an alpha channel definition,
16-bit RGBA, GCPs in the GeoJP2 box, RPC metadata, a GMLJP2 box only, a rotated geotransform,
signed 24-bit samples (Int32), dataset metadata (the GDAL metadata box), and a single-tile image larger
than 1024 (decoded by areas). Then GeoTIFF copies of some of them (gtiff/, GDAL's translation to
GTiff), which the COG builder must turn into the COG it builds from the JPEG 2000 file. Then run
expected.py.

    python3 -I tests/fixtures/jp2/gen.py
"""
import os

import numpy as np
from osgeo import gdal, osr

gdal.UseExceptions()
os.environ["GDAL_PAM_ENABLED"] = "NO"
gdal.SetConfigOption("GDAL_PAM_ENABLED", "NO")
HERE = os.path.dirname(os.path.abspath(__file__))
SAMPLES = os.path.join(HERE, "..", "gdal")
MEM = gdal.GetDriverByName("MEM")
JP2 = gdal.GetDriverByName("JP2OpenJPEG")


def pattern(rng, w, h, b, lo=0, hi=255, noise=6.0):
    y, x = np.mgrid[0:h, 0:w].astype(np.float64)
    v = 0.5 + 0.45 * np.sin(x / (9.0 + 4 * b)) * np.cos(y / (7.0 + 3 * b))
    v = lo + (hi - lo) * v + noise * rng.standard_normal((h, w))
    return np.clip(np.round(v), lo, hi)


def mem(w, h, bands, dtype, arrays, gt=(500000.0, 2.0, 0.0, 4100000.0, 0.0, -2.0), epsg=32633):
    ds = MEM.Create("", w, h, bands, dtype)
    if gt is not None:
        ds.SetGeoTransform(gt)
    if epsg is not None:
        srs = osr.SpatialReference()
        srs.ImportFromEPSG(epsg)
        ds.SetSpatialRef(srs)
    for i, a in enumerate(arrays):
        ds.GetRasterBand(i + 1).WriteArray(a)
    return ds


def write(name, src, options):
    path = os.path.join(HERE, name)
    JP2.CreateCopy(path, src, options=options)
    for side in (path + ".aux.xml",):
        if os.path.exists(side):
            os.remove(side)
    print(name, os.path.getsize(path))


def main():
    rng = np.random.default_rng(2026)
    # Colour table: a pclr box, lossless
    w, h = 61, 47
    idx = (pattern(rng, w, h, 0, 0, 15, 0.5)).astype(np.uint8)
    ds = mem(w, h, 1, gdal.GDT_Byte, [idx])
    ct = gdal.ColorTable()
    for i in range(16):
        ct.SetColorEntry(i, (i * 16, 255 - i * 13, (i * 37) % 256, 255))
    ds.GetRasterBand(1).SetColorTable(ct)
    write("gen_palette.jp2", ds, ["REVERSIBLE=YES", "QUALITY=100"])

    # 4:2:0 YCbCr, odd sizes
    w, h = 65, 49
    rgb = [pattern(rng, w, h, b).astype(np.uint8) for b in range(3)]
    write("gen_ycbcr420.jp2", mem(w, h, 3, gdal.GDT_Byte, rgb), ["YCBCR420=YES", "QUALITY=50"])

    # Lossy RGB in 128 x 128 tiles with the YCC transform (partial tiles at the edges)
    w, h = 300, 200
    rgb = [pattern(rng, w, h, b).astype(np.uint8) for b in range(3)]
    write("gen_rgb_tiled_lossy.jp2", mem(w, h, 3, gdal.GDT_Byte, rgb),
          ["YCC=YES", "QUALITY=20", "BLOCKXSIZE=128", "BLOCKYSIZE=128"])

    # RGBA with an alpha channel definition
    w, h = 150, 100
    a = np.full((h, w), 255, np.uint8)
    yy, xx = np.mgrid[0:h, 0:w]
    a[(xx - 40) ** 2 + (yy - 60) ** 2 < 900] = 0
    rgba = [pattern(rng, w, h, b).astype(np.uint8) for b in range(3)] + [a]
    ds = mem(w, h, 4, gdal.GDT_Byte, rgba)
    ds.GetRasterBand(4).SetColorInterpretation(gdal.GCI_AlphaBand)
    write("gen_rgba.jp2", ds, ["QUALITY=30", "ALPHA=YES", "BLOCKXSIZE=64", "BLOCKYSIZE=64"])

    # 16-bit RGBA, lossless
    w, h = 90, 70
    a = np.where((xx[:h, :w] - 30) ** 2 + (yy[:h, :w] - 35) ** 2 < 400, 0, 65535).astype(np.uint16)
    rgba = [pattern(rng, w, h, b, 0, 4000, 30).astype(np.uint16) for b in range(3)] + [a]
    ds = mem(w, h, 4, gdal.GDT_UInt16, rgba)
    ds.GetRasterBand(4).SetColorInterpretation(gdal.GCI_AlphaBand)
    write("gen_uint16_rgba.jp2", ds, ["REVERSIBLE=YES", "QUALITY=100", "ALPHA=YES"])

    # GCPs (GDAL's byte_gcp.tif) in the GeoJP2 box only
    src = gdal.Open(os.path.join(SAMPLES, "byte_gcp.tif"))
    write("gen_gcps.jp2", src, ["REVERSIBLE=YES", "QUALITY=100", "GMLJP2=NO"])

    # GCPs and RPC metadata
    w, h = 40, 30
    ds = mem(w, h, 1, gdal.GDT_Byte, [pattern(rng, w, h, 0).astype(np.uint8)], gt=None, epsg=None)
    srs = osr.SpatialReference()
    srs.ImportFromEPSG(4326)
    ds.SetGCPs([gdal.GCP(12.0, 45.0, 0, 0, 0), gdal.GCP(12.01, 45.0, 0, 40, 0),
                gdal.GCP(12.0, 44.99, 0, 0, 30), gdal.GCP(12.01, 44.99, 0, 40, 30)], srs)
    rpc = {"LINE_OFF": "15", "SAMP_OFF": "20", "LAT_OFF": "44.995", "LONG_OFF": "12.005", "HEIGHT_OFF": "0",
           "LINE_SCALE": "15", "SAMP_SCALE": "20", "LAT_SCALE": "0.005", "LONG_SCALE": "0.005", "HEIGHT_SCALE": "100",
           "LINE_NUM_COEFF": " ".join(["0", "0", "-1"] + ["0"] * 17),
           "LINE_DEN_COEFF": " ".join(["1"] + ["0"] * 19),
           "SAMP_NUM_COEFF": " ".join(["0", "1"] + ["0"] * 18),
           "SAMP_DEN_COEFF": " ".join(["1"] + ["0"] * 19)}
    ds.SetMetadata(rpc, "RPC")
    write("gen_rpc.jp2", ds, ["REVERSIBLE=YES", "QUALITY=100", "GMLJP2=NO"])

    # A GMLJP2 box only (EPSG:4326: GDAL writes it in latitude, longitude order)
    w, h = 50, 40
    ds = mem(w, h, 1, gdal.GDT_Int16, [pattern(rng, w, h, 0, -500, 1500, 20).astype(np.int16)],
             gt=(11.5, 0.001, 0.0, 46.2, 0.0, -0.001), epsg=4326)
    write("gen_gml_only.jp2", ds, ["REVERSIBLE=YES", "QUALITY=100", "GeoJP2=NO"])

    # A rotated geotransform
    w, h = 45, 35
    ds = mem(w, h, 1, gdal.GDT_UInt16, [pattern(rng, w, h, 0, 0, 3000, 10).astype(np.uint16)],
             gt=(600000.0, 1.8, 0.6, 5000000.0, 0.5, -1.9), epsg=32632)
    write("gen_rotated.jp2", ds, ["REVERSIBLE=YES", "QUALITY=100"])

    # Signed 24-bit samples (OpenJPEG decodes 31 bits at most): an Int32 band
    w, h = 33, 21
    ds = mem(w, h, 1, gdal.GDT_Int32, [pattern(rng, w, h, 0, -100000, 100000, 50).astype(np.int32)])
    write("gen_int32.jp2", ds, ["REVERSIBLE=YES", "QUALITY=100", "NBITS=24"])

    # Dataset metadata and TIFF tags in the GDAL metadata box
    w, h = 30, 20
    ds = mem(w, h, 1, gdal.GDT_Byte, [pattern(rng, w, h, 0).astype(np.uint8)])
    ds.SetMetadata({"ACQUISITION": "2026-01-02 <morning> & noon", "TIFFTAG_SOFTWARE": "gen.py",
                    "TIFFTAG_XRESOLUTION": "72", "TIFFTAG_YRESOLUTION": "72", "TIFFTAG_RESOLUTIONUNIT": "2"})
    write("gen_metadata.jp2", ds, ["REVERSIBLE=YES", "QUALITY=100", "WRITE_METADATA=YES", "MAIN_MD_DOMAIN_ONLY=YES"])

    # A single tile larger than 1024 x 1024, lossy: decoded by areas of 1024 x 1024
    w, h = 1500, 1100
    rgb = [pattern(rng, w, h, b, noise=2.0).astype(np.uint8) for b in range(3)]
    write("gen_single_tile.jp2", mem(w, h, 3, gdal.GDT_Byte, rgb),
          ["QUALITY=0.5", "BLOCKXSIZE=1500", "BLOCKYSIZE=1100", "RESOLUTIONS=4"])


# The files whose GeoTIFF copy (gdal.Translate) the end-to-end test builds a COG from: lossless or
# decoded alike by the system's OpenJPEG and the one libddb ships (expected.tsv checks the copies).
TIFF_COPIES = ["gen_palette.jp2", "gen_rgba.jp2", "gen_uint16_rgba.jp2", "gen_gcps.jp2", "gen_rotated.jp2",
               "gen_gml_only.jp2", "gen_ycbcr420.jp2", "gen_metadata.jp2", "gen_int32.jp2", "small_200ppcm.jp2",
               "tile_size_16.jp2", "513x513.jp2"]


def half_alpha_tiff():
    """A 16-bit RGBA GeoTIFF whose alpha is half of 65535: gdalwarp's SRC_ALPHA_MAX makes it a
    density of 32768 / 65535 (32768 again in the output), not an opaque pixel."""
    rng = np.random.default_rng(7)
    w, h = 64, 48
    bands = [pattern(rng, w, h, b, 0, 4000, 30).astype(np.uint16) for b in range(3)]
    bands.append(np.full((h, w), 32768, np.uint16))
    ds = mem(w, h, 4, gdal.GDT_UInt16, bands)
    ds.GetRasterBand(4).SetColorInterpretation(gdal.GCI_AlphaBand)
    out = os.path.join(HERE, "gtiff", "uint16_half_alpha.tif")
    gdal.GetDriverByName("GTiff").CreateCopy(out, ds, options=["COMPRESS=DEFLATE", "PREDICTOR=2", "ALPHA=YES"])
    print(os.path.basename(out), os.path.getsize(out))


def tiff_copies():
    os.makedirs(os.path.join(HERE, "gtiff"), exist_ok=True)
    for name in TIFF_COPIES:
        out = os.path.join(HERE, "gtiff", os.path.splitext(name)[0] + ".tif")
        gdal.Translate(out, os.path.join(HERE, name), format="GTiff",
                       creationOptions=["TILED=YES", "COMPRESS=DEFLATE", "PREDICTOR=2"])
        print(os.path.basename(out), os.path.getsize(out))


if __name__ == "__main__":
    main()
    tiff_copies()
    half_alpha_tiff()
