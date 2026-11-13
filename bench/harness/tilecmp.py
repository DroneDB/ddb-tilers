#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Compare two tile dumps (ddbt-bench tile --dump-tiles) decoded to RGBA.

Per tile: PSNR on RGB where both alphas > 0, SSIM on the luma of the alpha-premultiplied RGB
(8x8 windows, stride 1, computed with integral images), alpha IoU (alpha > 0), max absolute
difference (RGB where both alphas > 0, alpha everywhere) and an exact flag (identical RGBA).
A tile passes when it is exact, or PSNR >= --psnr-min, SSIM >= --ssim-min and alpha IoU >=
--iou-min. Failing tiles get a diff PNG (|A-B| x 4) and an A | B | diff mosaic in --diff-dir.

Usage: tilecmp.py --a DIR --b DIR [--out cmp.json] [--diff-dir DIR]
                  [--psnr-min 40] [--ssim-min 0.98] [--iou-min 0.999]
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common  # noqa: E402

DEFAULTS = {"psnr_min": 40.0, "ssim_min": 0.98, "iou_min": 0.999}
SSIM_WIN = 8
C1 = (0.01 * 255) ** 2
C2 = (0.03 * 255) ** 2


def load_rgba(path):
    with Image.open(path) as im:
        im.load()
        return np.asarray(im.convert("RGBA"), dtype=np.uint8)


def psnr(a, b, mask=None):
    """PSNR (dB) of RGB over mask (both alphas > 0 when None); inf when identical, None when
    the mask is empty."""
    if mask is None:
        mask = (a[..., 3] > 0) & (b[..., 3] > 0)
    if not mask.any():
        return None
    d = a[..., :3][mask].astype(np.float64) - b[..., :3][mask].astype(np.float64)
    mse = float(np.mean(d * d))
    if mse == 0.0:
        return math.inf
    return 10.0 * math.log10(255.0 * 255.0 / mse)


def _integral(x):
    s = np.zeros((x.shape[0] + 1, x.shape[1] + 1), dtype=np.float64)
    s[1:, 1:] = x.cumsum(0).cumsum(1)
    return s


def _window_sums(s, w):
    return s[w:, w:] - s[:-w, w:] - s[w:, :-w] + s[:-w, :-w]


def luma(img):
    """BT.601 luma of the alpha-premultiplied RGB (transparent pixels count as black)."""
    rgb = img[..., :3].astype(np.float64) * (img[..., 3:4].astype(np.float64) / 255.0)
    return rgb[..., 0] * 0.299 + rgb[..., 1] * 0.587 + rgb[..., 2] * 0.114


def ssim(a, b, win=SSIM_WIN):
    """Mean SSIM over all win x win windows (stride 1) of two luma images, via integral images."""
    x, y = luma(a), luma(b)
    if x.shape[0] < win or x.shape[1] < win:
        win = min(x.shape)
    n = float(win * win)
    sx = _window_sums(_integral(x), win)
    sy = _window_sums(_integral(y), win)
    sxx = _window_sums(_integral(x * x), win)
    syy = _window_sums(_integral(y * y), win)
    sxy = _window_sums(_integral(x * y), win)
    mx, my = sx / n, sy / n
    vx = np.maximum(sxx / n - mx * mx, 0.0)
    vy = np.maximum(syy / n - my * my, 0.0)
    cxy = sxy / n - mx * my
    m = ((2 * mx * my + C1) * (2 * cxy + C2)) / ((mx * mx + my * my + C1) * (vx + vy + C2))
    return float(m.mean())


def alpha_iou(a, b):
    ma, mb = a[..., 3] > 0, b[..., 3] > 0
    union = np.count_nonzero(ma | mb)
    if union == 0:
        return 1.0
    return np.count_nonzero(ma & mb) / union


def max_abs_diff(a, b):
    both = (a[..., 3] > 0) & (b[..., 3] > 0)
    d_alpha = int(np.max(np.abs(a[..., 3].astype(np.int16) - b[..., 3].astype(np.int16))))
    d_rgb = 0
    if both.any():
        d_rgb = int(np.max(np.abs(a[..., :3][both].astype(np.int16) - b[..., :3][both].astype(np.int16))))
    return max(d_rgb, d_alpha)


def compare_images(a, b, th=None):
    th = dict(DEFAULTS, **(th or {}))
    if a.shape != b.shape:
        return {"pass": False, "reason": "size %s vs %s" % (a.shape[:2], b.shape[:2]), "exact": False}
    exact = bool(np.array_equal(a, b))
    r = {"exact": exact, "psnr": psnr(a, b), "ssim": ssim(a, b), "alpha_iou": alpha_iou(a, b),
         "max_abs_diff": max_abs_diff(a, b)}
    if exact:
        r["pass"] = True
        return r
    reasons = []
    p = r["psnr"]
    if p is not None and p < th["psnr_min"]:
        reasons.append("psnr %.2f < %.2f" % (p, th["psnr_min"]))
    if r["ssim"] < th["ssim_min"]:
        reasons.append("ssim %.4f < %.4f" % (r["ssim"], th["ssim_min"]))
    if r["alpha_iou"] < th["iou_min"]:
        reasons.append("alpha IoU %.5f < %.5f" % (r["alpha_iou"], th["iou_min"]))
    r["pass"] = not reasons
    if reasons:
        r["reason"] = "; ".join(reasons)
    return r


def diff_image(a, b):
    d = np.abs(a.astype(np.int16) - b.astype(np.int16))
    rgb = np.clip(np.maximum(d[..., :3], d[..., 3:4]) * 4, 0, 255).astype(np.uint8)
    return np.dstack([rgb, np.full(rgb.shape[:2], 255, np.uint8)])


def checker(h, w, cell=8):
    yy, xx = np.mgrid[0:h, 0:w]
    c = (((yy // cell) + (xx // cell)) % 2 * 40 + 200).astype(np.uint8)
    return np.dstack([c, c, c, np.full((h, w), 255, np.uint8)])


def over_checker(img):
    bg = checker(*img.shape[:2]).astype(np.float64)
    al = img[..., 3:4].astype(np.float64) / 255.0
    rgb = img[..., :3] * al + bg[..., :3] * (1 - al)
    return np.dstack([rgb.astype(np.uint8), np.full(img.shape[:2], 255, np.uint8)])


def write_diffs(a, b, diff_dir, stem):
    os.makedirs(diff_dir, exist_ok=True)
    d = diff_image(a, b)
    Image.fromarray(d, "RGBA").save(os.path.join(diff_dir, stem + "_diff.png"))
    if a.shape == b.shape:
        mosaic = np.concatenate([over_checker(a), over_checker(b), d], axis=1)
        Image.fromarray(mosaic, "RGBA").save(os.path.join(diff_dir, stem + "_mosaic.png"))


def read_index(d):
    """index.jsonl of a dump: {(z, x, y, i): entry}; falls back to the file names."""
    out = {}
    idx = os.path.join(d, "index.jsonl")
    if os.path.isfile(idx):
        with open(idx) as f:
            for line in f:
                if line.strip():
                    e = json.loads(line)
                    out[(e["i"], e["z"], e["x"], e["y"])] = e
        return out
    for name in sorted(os.listdir(d)):
        stem, ext = os.path.splitext(name)
        parts = stem.split("_")
        if len(parts) == 4 and ext.lower() in (".png", ".jpg", ".jpeg", ".webp"):
            i, z, x, y = (int(p) for p in parts)
            out[(i, z, x, y)] = {"file": name, "ok": True}
    return out


def compare_dirs(dir_a, dir_b, th=None, diff_dir=None, max_diffs=200):
    th = dict(DEFAULTS, **(th or {}))
    ia, ib = read_index(dir_a), read_index(dir_b)
    keys = sorted(set(ia) | set(ib))
    tiles = []
    n_pass = n_exact = n_fail = n_both_failed = 0
    psnrs = []
    for k in keys:
        ea, eb = ia.get(k), ib.get(k)
        rec = {"i": k[0], "z": k[1], "x": k[2], "y": k[3]}
        oka = bool(ea and ea.get("ok", True) and ea.get("file"))
        okb = bool(eb and eb.get("ok", True) and eb.get("file"))
        if not oka and not okb:
            rec.update({"pass": True, "both_failed": True})
            n_both_failed += 1
            n_pass += 1
            tiles.append(rec)
            continue
        if oka != okb:
            rec.update({"pass": False, "reason": "missing in %s" % ("B" if oka else "A")})
            n_fail += 1
            tiles.append(rec)
            continue
        a = load_rgba(os.path.join(dir_a, ea["file"]))
        b = load_rgba(os.path.join(dir_b, eb["file"]))
        r = compare_images(a, b, th)
        rec.update(r)
        if r.get("exact"):
            n_exact += 1
        if r["pass"]:
            n_pass += 1
        else:
            n_fail += 1
            if diff_dir and n_fail <= max_diffs:
                write_diffs(a, b, diff_dir, "%06d_%d_%d_%d" % k)
        if r.get("psnr") is not None:
            psnrs.append(r["psnr"])
        tiles.append(rec)
    finite = [p for p in psnrs if math.isfinite(p)]
    summary = {"tiles": len(keys), "pass": n_pass, "fail": n_fail, "exact": n_exact, "both_failed": n_both_failed,
               "psnr_min": min(psnrs) if psnrs else None,
               "psnr_mean_finite": float(np.mean(finite)) if finite else None,
               "ssim_min": min((t["ssim"] for t in tiles if "ssim" in t), default=None),
               "alpha_iou_min": min((t["alpha_iou"] for t in tiles if "alpha_iou" in t), default=None),
               "thresholds": th}
    return {"schema": "ddbt-tilecmp/1", "a": dir_a, "b": dir_b, "summary": summary, "tiles": tiles}


def _json_safe(o):
    if isinstance(o, float) and not math.isfinite(o):
        return "inf" if o > 0 else ("-inf" if o < 0 else None)
    if isinstance(o, dict):
        return {k: _json_safe(v) for k, v in o.items()}
    if isinstance(o, list):
        return [_json_safe(v) for v in o]
    return o


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--a", required=True)
    ap.add_argument("--b", required=True)
    ap.add_argument("--out")
    ap.add_argument("--diff-dir")
    ap.add_argument("--psnr-min", type=float, default=DEFAULTS["psnr_min"])
    ap.add_argument("--ssim-min", type=float, default=DEFAULTS["ssim_min"])
    ap.add_argument("--iou-min", type=float, default=DEFAULTS["iou_min"])
    a = ap.parse_args(argv)
    res = compare_dirs(a.a, a.b, {"psnr_min": a.psnr_min, "ssim_min": a.ssim_min, "iou_min": a.iou_min},
                       a.diff_dir)
    res = _json_safe(res)
    if a.out:
        common.write_json(a.out, res)
    print(json.dumps(res["summary"], indent=1))
    return 0 if res["summary"]["fail"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
