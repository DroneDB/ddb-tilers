# GeoTIFFs georeferenced by ground control points

Small rasters georeferenced only by GCPs (the GCPs in ModelTiepointTag, the CRS in the GeoKeys,
no geotransform), for `GcpCog.*` (tests/unit/cogbuild/gcp_cog_test.cpp). They are written,
seeded, by `tests/unit/cogbuild/data/gen_gcp_cog_reference.py` with the GDAL Python bindings:

| File | GCPs | CRS | Bands |
|---|---|---|---|
| `homography4_utm_byte.tif` | 4, a projective mapping (GDAL fits a homography) | EPSG:32632 | Byte |
| `homography5_4326_int16.tif` | 5 (least-squares homography) | EPSG:4326 | Int16 |
| `poly6_utm_uint16_nodata.tif` | 6, rotated and curved (polynomial of order 2) | EPSG:32633 | UInt16, nodata 0 |
| `poly10_4326_float32_nodata.tif` | 10 at random places, sheared | EPSG:4326 | Float32, nodata -9999 |
| `poly25_noisy_utm_gray_alpha.tif` | 25 with noise (least squares, order 2) | EPSG:32633 | Byte gray + alpha |
| `poly3_3857_byte.tif` | 3, rotated (order 1, no reprojection) | EPSG:3857 | Byte |
| `antimeridian6_4326_byte.tif` | 6 across the antimeridian | EPSG:4326 | Byte (GDAL refuses it) |
| `fail_one_gcp.tif`, `fail_collinear.tif` | 1; 3 on a line | EPSG:4326 | Byte (GDAL refuses them) |
| `no_crs.tif` | 4 | none | Byte (libddb refuses it) |

`gdal/` holds the COGs GDAL 3.13.3 (vcpkg) writes for them with libddb's arguments
(`tests/unit/cogbuild/data/gdal_cog_warp.cpp`): `-of COG -t_srs EPSG:3857 -r bilinear -co
TILING_SCHEME=GoogleMapsCompatible -co ZOOM_LEVEL_STRATEGY=UPPER -wm 512`, LZW with a predictor,
and for `gdal_*.tif` the GCP-only samples of GDAL's test suite in `../gdal` (`byte_gcp.tif`,
`byte_gcp_pixelispoint.tif`, `cint_sar.tif`; GDAL's licence, `../gdal/GDAL-LICENSE.TXT`).
`tests/unit/cogbuild/data/gcp_cog_reference.inc` has GDAL's grid, warp chunks and refusals.
