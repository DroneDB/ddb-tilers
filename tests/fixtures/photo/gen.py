#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Writes the generated photos of this folder (see README.md) with Pillow, and the TIFF photos
with the system's gdal_translate. Deterministic content: a seeded random image.

    python3 -I tests/fixtures/photo/gen.py [--dng ORIGINAL.DNG]

--dng: the CC0 sample of raw.pixls.us the small DNG is cut from (dng_fc220_preview.dng).
"""
import argparse
import io
import os
import struct
import subprocess
import tempfile

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))


def content(w, h, seed=7):
    """An RGB image with gradients, edges and noise (what JPEG and its upsampling must keep)."""
    rng = np.random.default_rng(seed)
    y, x = np.mgrid[0:h, 0:w]
    r = (x * 255 // max(w - 1, 1))
    g = (y * 255 // max(h - 1, 1))
    b = ((x // 8 + y // 8) % 2) * 200 + 20
    img = np.stack([r, g, b], axis=-1).astype(np.int32)
    img += rng.integers(-8, 9, size=img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def out(name):
    return os.path.join(HERE, name)


def jpegs(rgb):
    im = Image.fromarray(rgb)
    im.save(out("jpeg_baseline_420.jpg"), quality=85, subsampling=2)
    im.save(out("jpeg_422.jpg"), quality=85, subsampling=1)
    im.save(out("jpeg_444.jpg"), quality=92, subsampling=0)
    im.save(out("jpeg_progressive.jpg"), quality=80, progressive=True)
    im.save(out("jpeg_optimized.jpg"), quality=70, optimize=True)
    im.convert("L").save(out("jpeg_gray.jpg"), quality=85)
    im.convert("CMYK").save(out("jpeg_cmyk.jpg"), quality=90)
    exif = Image.Exif()
    exif[0x0112] = 6  # Orientation: rotate 90 degrees (GDAL does not apply it)
    im.save(out("jpeg_exif_orientation6.jpg"), quality=85, exif=exif.tobytes())
    # Truncated: "Premature end of JPEG file", which GDAL makes an error.
    buf = io.BytesIO()
    im.save(buf, format="JPEG", quality=85)
    data = buf.getvalue()
    with open(out("jpeg_truncated.jpg"), "wb") as f:
        f.write(data[: len(data) * 2 // 3])


def pngs(rgb):
    im = Image.fromarray(rgb)
    im.save(out("png_rgb.png"))
    rgba = np.dstack([rgb, (np.arange(rgb.shape[1])[None, :] * 4 % 256).repeat(rgb.shape[0], 0).astype(np.uint8)])
    Image.fromarray(rgba, "RGBA").save(out("png_rgba.png"))
    gray = im.convert("L")
    gray.save(out("png_gray.png"))
    Image.merge("LA", (gray, Image.fromarray(rgba[:, :, 3]))).save(out("png_gray_alpha.png"))
    gray.save(out("png_gray_trns.png"), transparency=int(np.asarray(gray)[3, 3]))
    g16 = (np.asarray(gray).astype(np.uint16) * 257 + np.arange(rgb.shape[1], dtype=np.uint16)[None, :])
    Image.fromarray(g16.astype(np.uint16)).save(out("png_gray16.png"))
    im.save(out("png_rgb_trns.png"), transparency=(40, 40, 40))
    # Red, green and blue transparency values that differ: one nodata value per band in GDAL.
    im.save(out("refused/png_rgb_trns_bands.png"), transparency=(40, 50, 60))
    pal = im.quantize(colors=64)
    pal.save(out("png_palette_trns.png"), transparency=5)
    alpha = bytes([255] * 3 + [0, 128, 0] + [255] * 10)
    pal.save(out("png_palette_alpha.png"), transparency=alpha)
    im.quantize(colors=16).save(out("png_palette_4bit.png"), bits=4)
    gray.point(lambda v: 255 if v > 127 else 0).convert("1").save(out("png_1bit.png"))


def gifs(rgb):
    pal = Image.fromarray(rgb).quantize(colors=128)
    pal.save(out("gif_plain.gif"), interlace=False)
    pal.save(out("gif_interlaced.gif"), interlace=True)
    pal.save(out("gif_transparent.gif"), transparency=9, interlace=False)
    # Two images of the same size: GDAL makes them two bands (refused here).
    frames = [pal, Image.fromarray(rgb[::-1].copy()).quantize(colors=128)]
    frames[0].save(out("refused/gif_animated.gif"), save_all=True, append_images=frames[1:], duration=100, loop=0)


def webps(rgb):
    im = Image.fromarray(rgb)
    im.save(out("webp_lossy.webp"), quality=80)
    im.save(out("webp_lossless.webp"), lossless=True)
    rgba = np.dstack([rgb, (np.arange(rgb.shape[0])[:, None] * 5 % 256).repeat(rgb.shape[1], 1).astype(np.uint8)])
    Image.fromarray(rgba, "RGBA").save(out("webp_alpha.webp"), quality=75)
    Image.fromarray(rgba, "RGBA").save(out("webp_lossless_alpha.webp"), lossless=True)


def tiffs(rgb):
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, "rgb.png")
        Image.fromarray(rgb).save(src)
        for name, co in (("tiff_photo_jpeg.tif", ["COMPRESS=JPEG", "PHOTOMETRIC=YCBCR"]),
                         ("tiff_photo_deflate.tif", ["COMPRESS=DEFLATE"])):
            args = ["gdal_translate", "-q", "-of", "GTiff"]
            for c in co:
                args += ["-co", c]
            subprocess.run(args + [src, out(name)], check=True)


def dng(original):
    """IFD0 of the DNG (its 8-bit YCbCr JPEG preview, which GDAL reads), without the raw image:
    the SubIFDs, EXIF and GPS directories and the private data are left out."""
    d = open(original, "rb").read()
    assert d[:4] == b"II*\0"
    off = struct.unpack("<I", d[4:8])[0]
    n = struct.unpack("<H", d[off:off + 2])[0]
    sizes = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 6: 1, 7: 1, 8: 2, 9: 4, 10: 8, 11: 4, 12: 8}
    entries = []
    for i in range(n):
        tag, typ, cnt = struct.unpack("<HHI", d[off + 2 + 12 * i: off + 10 + 12 * i])
        raw = d[off + 10 + 12 * i: off + 14 + 12 * i]
        size = sizes[typ] * cnt
        value = raw[:size] if size <= 4 else d[struct.unpack("<I", raw)[0]:][:size]
        if tag in (330, 34665, 34853, 50740):
            continue
        entries.append([tag, typ, cnt, value])
    tags = {e[0]: e for e in entries}
    strip_off = struct.unpack("<I", tags[273][3][:4])[0]
    strip_len = struct.unpack("<I", tags[279][3][:4])[0]
    strip = d[strip_off:strip_off + strip_len]
    # Layout: header, IFD, values, strip.
    ifd_at = 8
    values_at = ifd_at + 2 + 12 * len(entries) + 4
    blob = bytearray()
    fields = []
    for tag, typ, cnt, value in entries:
        if len(value) <= 4:
            fields.append((tag, typ, cnt, value.ljust(4, b"\0")))
        else:
            if (values_at + len(blob)) % 2:
                blob += b"\0"
            fields.append((tag, typ, cnt, struct.pack("<I", values_at + len(blob))))
            blob += value
    strip_at = values_at + len(blob)
    out_bytes = bytearray(b"II*\0" + struct.pack("<I", ifd_at) + struct.pack("<H", len(fields)))
    for tag, typ, cnt, v in fields:
        if tag == 273:
            v = struct.pack("<I", strip_at)
        out_bytes += struct.pack("<HHI", tag, typ, cnt) + v
    out_bytes += struct.pack("<I", 0) + blob + strip
    with open(out("dng_fc220_preview.dng"), "wb") as f:
        f.write(out_bytes)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dng", help="raw.pixls.us' DJI FC220 sample (CC0)")
    args = parser.parse_args()
    rgb = content(65, 41)
    jpegs(rgb)
    pngs(rgb)
    gifs(rgb)
    webps(rgb)
    tiffs(rgb)
    # The photo of the geoprojection tests: larger, 4:2:0 JPEG.
    Image.fromarray(content(320, 240, seed=11)).save(out("geo_photo.jpg"), quality=85)
    if args.dng:
        dng(args.dng)


if __name__ == "__main__":
    main()
