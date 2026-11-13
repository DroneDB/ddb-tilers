#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""A visual comparison page of COPC tile dumps: baseline | compat | hybrid | modern, side by side.

For each cloud set and zoom, the tiles shown are taken from the baseline's dump (the same request
indices in every dump): the one with the most opaque pixels and the one with the median number of
opaque pixels among the non-empty ones. Each tile is shown whole (256 px) and as a 4x enlarged
64 x 64 crop of its centre (nearest neighbour, so the discs are visible). Images are embedded as
data URIs: the page is one self-contained file.

Under each tile: alpha IoU against the baseline (hybrid moved by (4, 1) px, modern by 4 px:
the baseline draws 4 px west and, for its stamp, 1 px north of the point) and the mean |dRGB| over
the pixels opaque in both.

Usage: modes_html.py --out page.html --legacy LEGACY_DIR --compat DIR --hybrid DIR --modern DIR
                     [--sets P2-sweep,P3-sweep,E5-sweep] [--zooms 17,19,21] [--title T] [--note TEXT]
(each DIR holds one sub-directory per set, in the dump layout of compare_dumps.py)
"""
import argparse
import base64
import html
import io
import json
import os

import numpy as np
from PIL import Image

SHIFTS = {"baseline": (0, 0), "compat": (0, 0), "hybrid": (4, 1), "modern": (4, 0)}


def load(d):
    rows = {}
    with open(os.path.join(d, "index.jsonl")) as f:
        for line in f:
            if line.strip():
                e = json.loads(line)
                rows[e["i"]] = e
    return rows


def rgba(d, e):
    with Image.open(os.path.join(d, e["file"])) as im:
        return np.asarray(im.convert("RGBA"), dtype=np.uint8)


def data_uri(a):
    buf = io.BytesIO()
    Image.fromarray(a, "RGBA").save(buf, format="PNG", optimize=True)
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode("ascii")


def shifted(a, dx, dy):
    s = np.zeros_like(a)
    h, w = a.shape[:2]
    s[max(dy, 0):h + min(dy, 0), max(dx, 0):w + min(dx, 0)] = a[max(-dy, 0):h - max(dy, 0), max(-dx, 0):w - max(dx, 0)]
    return s


def metrics(base, img, dx, dy, m=8):
    b = shifted(base, dx, dy)[m:-m, m:-m]
    i = img[m:-m, m:-m]
    ob, oi = b[..., 3] > 0, i[..., 3] > 0
    inter, union = int((ob & oi).sum()), int((ob | oi).sum())
    both = ob & oi
    d = np.abs(b[..., :3][both].astype(np.int32) - i[..., :3][both].astype(np.int32))
    return (inter / union if union else 1.0), (float(d.mean()) if d.size else 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--legacy", required=True)
    ap.add_argument("--compat", required=True)
    ap.add_argument("--hybrid", required=True)
    ap.add_argument("--modern", required=True)
    ap.add_argument("--sets", default="P2-sweep,P3-sweep,E5-sweep")
    ap.add_argument("--zooms", default="17,19,21")
    ap.add_argument("--title", default="COPC tiler modes: baseline | compat | hybrid | modern")
    ap.add_argument("--note", default="")
    args = ap.parse_args()
    dirs = {"baseline": args.legacy, "compat": args.compat, "hybrid": args.hybrid, "modern": args.modern}
    zooms = [int(z) for z in args.zooms.split(",")]

    out = [
        "<!DOCTYPE html><html><head><meta charset='utf-8'><title>%s</title><style>" % html.escape(args.title),
        "body{font-family:sans-serif;background:#222;color:#ddd;margin:16px}"
        "h1{font-size:20px}h2{font-size:17px;margin-top:28px;border-top:1px solid #555;padding-top:10px}"
        "table{border-collapse:collapse}td,th{padding:4px 6px;vertical-align:top;text-align:center;font-size:12px}"
        "img{display:block;background:repeating-conic-gradient(#3a3a3a 0% 25%,#2e2e2e 0% 50%) 50%/16px 16px}"
        "img.crop{width:256px;height:256px;image-rendering:pixelated}"
        ".m{color:#aaa;font-size:11px}p{max-width:1100px;font-size:13px;line-height:1.4}",
        "</style></head><body><h1>%s</h1>" % html.escape(args.title),
    ]
    if args.note:
        out.append("<p>%s</p>" % args.note)
    for s in args.sets.split(","):
        rows = {k: load(os.path.join(d, s)) for k, d in dirs.items()}
        base = rows["baseline"]
        for z in zooms:
            cands = []
            for i, e in base.items():
                if e["z"] != z or not e.get("ok"):
                    continue
                if not all(i in r and r[i].get("ok") for r in rows.values()):
                    continue
                n = int((rgba(dirs["baseline"] + "/" + s, e)[..., 3] > 0).sum())
                if n > 0:
                    cands.append((n, i))
            if not cands:
                continue
            cands.sort(key=lambda t: (-t[0], t[1]))
            picks = [cands[0][1]]
            med = sorted(cands)[len(cands) // 2][1]
            if med not in picks:
                picks.append(med)
            for i in picks:
                e = base[i]
                out.append("<h2>%s z%d tile %d/%d (XYZ row %d) &mdash; request %d</h2>" % (
                    html.escape(s), z, z, e["x"], e["y"], i))
                imgs = {k: rgba(os.path.join(dirs[k], s), rows[k][i]) for k in dirs}
                out.append("<table><tr>" + "".join("<th>%s</th>" % k for k in dirs) + "</tr><tr>")
                for k, a in imgs.items():
                    dx, dy = SHIFTS[k]
                    iou, dm = metrics(imgs["baseline"], a, dx, dy)
                    cap = "opaque %d" % int((a[..., 3] > 0).sum())
                    if k != "baseline":
                        cap += "<br>IoU vs baseline %.4f (moved %d,%d)<br>mean |dRGB| %.2f" % (iou, dx, dy, dm)
                    out.append("<td><img src='%s' width=256 height=256><div class=m>%s</div></td>" % (data_uri(a), cap))
                out.append("</tr><tr>")
                for k, a in imgs.items():
                    # The centre 64 x 64 crop; hybrid and modern cropped where the baseline's content
                    # lands in them (moved by their shift), so that the same points are shown.
                    dx, dy = SHIFTS[k]
                    c = a[96 + dy:160 + dy, 96 + dx:160 + dx]
                    out.append("<td><img class=crop src='%s'><div class=m>centre crop x4%s</div></td>" % (
                        data_uri(np.ascontiguousarray(c)), " (moved %d,%d)" % (dx, dy) if (dx or dy) else ""))
                out.append("</tr></table>")
    out.append("</body></html>")
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as f:
        f.write("\n".join(out))


if __name__ == "__main__":
    main()
