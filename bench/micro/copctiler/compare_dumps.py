#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Compares two COPC tile dumps (ddbt-bench tile --dump-tiles layout: index.jsonl + files).

Per tile pair (same request index): alpha IoU (alpha > 0), mean |dRGB| and PSNR over the
pixels opaque in both, exact flag; errors must agree (same message). Aggregates per zoom and
overall: pixel-weighted IoU (sum of intersections / sum of unions), the mean and 5th percentile
of the per-tile IoU, mean |dRGB| and PSNR over all common opaque pixels, exact tiles.

Error texts: ddbt-bench's index has none; --a-json DRIVER.json compares the multiset of the
failure messages of its first round with B's errors instead.

--shift-a DX moves A's pixels DX columns east and --shift-a-y DY rows south before comparing
(the baseline draws its stamp 4 px west and 1 px north of where hybrid draws it: compare a
baseline or compat dump with a hybrid one with --shift-a 4 --shift-a-y 1, with a modern one with
--shift-a 4); --margin M then ignores M columns on each side, --margin-y M rows.

Coverage: "cov B/A" is the number of opaque pixels of B over that of A (same window).

Usage: compare_dumps.py --a BASE_DIR --b HEAD_DIR [--a-json DRIVER.json] [--shift-a 4] [--shift-a-y 1]
                        [--margin 8] [--margin-y 8] [--json out.json]
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from PIL import Image


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


def error_text(e):
    t = e.get("error", "")
    # ddbt-bench prefixes libddb's code ("error 1: ...").
    if t.startswith("error ") and ": " in t:
        t = t.split(": ", 1)[1]
    return t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--a", required=True)
    ap.add_argument("--b", required=True)
    ap.add_argument("--shift-a", type=int, default=0)
    ap.add_argument("--shift-a-y", type=int, default=0)
    ap.add_argument("--margin", type=int, default=0)
    ap.add_argument("--margin-y", type=int, default=0)
    ap.add_argument("--a-json")
    ap.add_argument("--json")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    A, B = load(args.a), load(args.b)
    per_zoom = {}
    total = {"tiles": 0, "exact": 0, "inter": 0, "union": 0, "abs": 0.0, "sq": 0.0, "px": 0, "ious": [],
             "opaque_a": 0, "opaque_b": 0, "err_same": 0, "err_diff": 0, "only_a_ok": 0, "only_b_ok": 0}
    tiles = []
    b_errors = {}
    for i in sorted(A):
        if i not in B:
            continue
        a, b = A[i], B[i]
        z = a["z"]
        zs = per_zoom.setdefault(z, {"tiles": 0, "exact": 0, "inter": 0, "union": 0, "abs": 0.0, "sq": 0.0,
                                     "px": 0, "ious": [], "opaque_a": 0, "opaque_b": 0})
        if not a.get("ok") or not b.get("ok"):
            if not a.get("ok") and not b.get("ok"):
                if "error" not in a:
                    b_errors[error_text(b)] = b_errors.get(error_text(b), 0) + 1
                elif error_text(a) == error_text(b):
                    total["err_same"] += 1
                else:
                    total["err_diff"] += 1
                    if not args.quiet:
                        print("error differs", i, error_text(a), "|", error_text(b), file=sys.stderr)
            elif a.get("ok"):
                total["only_a_ok"] += 1
                if not args.quiet:
                    print("only A ok", i, a["z"], a["x"], a["y"], error_text(b), file=sys.stderr)
            else:
                total["only_b_ok"] += 1
                if not args.quiet:
                    print("only B ok", i, a["z"], a["x"], a["y"], error_text(a), file=sys.stderr)
            continue
        ia, ib = rgba(args.a, a), rgba(args.b, b)
        exact = ia.shape == ib.shape and np.array_equal(ia, ib)
        if args.shift_a:
            s = np.zeros_like(ia)
            if args.shift_a > 0:
                s[:, args.shift_a:] = ia[:, :-args.shift_a]
            else:
                s[:, :args.shift_a] = ia[:, -args.shift_a:]
            ia = s
        if args.shift_a_y:
            dy = args.shift_a_y
            s = np.zeros_like(ia)
            if dy > 0:
                s[dy:, :] = ia[:-dy, :]
            else:
                s[:dy, :] = ia[-dy:, :]
            ia = s
        if args.margin:
            m = args.margin
            ia, ib = ia[:, m:-m], ib[:, m:-m]
        if args.margin_y:
            m = args.margin_y
            ia, ib = ia[m:-m, :], ib[m:-m, :]
        oa, ob = ia[..., 3] > 0, ib[..., 3] > 0
        inter = int(np.count_nonzero(oa & ob))
        union = int(np.count_nonzero(oa | ob))
        both = oa & ob
        d = ia[..., :3][both].astype(np.int32) - ib[..., :3][both].astype(np.int32)
        absd = float(np.abs(d).sum())
        sq = float((d.astype(np.float64) ** 2).sum())
        npx = int(both.sum()) * 3
        iou = inter / union if union else 1.0
        tiles.append({"i": i, "z": z, "x": a["x"], "y": a["y"], "iou": iou, "exact": bool(exact),
                      "mean_abs": absd / npx if npx else 0.0})
        for acc in (zs, total):
            acc["tiles"] += 1
            acc["exact"] += int(exact)
            acc["inter"] += inter
            acc["union"] += union
            acc["abs"] += absd
            acc["sq"] += sq
            acc["px"] += npx
            acc["ious"].append(iou)
            acc["opaque_a"] += int(oa.sum())
            acc["opaque_b"] += int(ob.sum())

    def summary(acc):
        if not acc["tiles"]:
            return None
        mse = acc["sq"] / acc["px"] if acc["px"] else 0.0
        ious = sorted(acc["ious"])
        return {
            "tiles": acc["tiles"],
            "exact": acc["exact"],
            "iou_pixels": acc["inter"] / acc["union"] if acc["union"] else 1.0,
            "iou_mean": sum(ious) / len(ious),
            "iou_p5": ious[int(0.05 * (len(ious) - 1))],
            "iou_min": ious[0],
            "mean_abs_rgb": acc["abs"] / acc["px"] if acc["px"] else 0.0,
            "psnr": (10 * math.log10(255 * 255 / mse)) if mse > 0 else math.inf,
            "coverage_b_over_a": acc["opaque_b"] / acc["opaque_a"] if acc["opaque_a"] else math.inf,
        }

    if args.a_json:
        with open(args.a_json) as f:
            msgs = json.load(f)["round_results"][0].get("errors", {})
        a_errors = {}
        for k, v in msgs.items():
            t = error_text({"error": k})
            a_errors[t] = a_errors.get(t, 0) + v
        # B's errors on the requests both failed vs A's messages (A's top 20 only).
        total["err_same"] = sum(min(v, a_errors.get(k, 0)) for k, v in b_errors.items())
        total["err_diff"] = sum(b_errors.values()) - total["err_same"]
        if total["err_diff"] and not args.quiet:
            print("error multisets differ:", a_errors, b_errors, file=sys.stderr)
    out = {"a": args.a, "b": args.b, "shift_a": args.shift_a, "shift_a_y": args.shift_a_y, "margin": args.margin,
           "margin_y": args.margin_y,
           "overall": summary(total), "errors_same": total["err_same"], "errors_differ": total["err_diff"],
           "only_a_ok": total["only_a_ok"], "only_b_ok": total["only_b_ok"],
           "per_zoom": {str(z): summary(v) for z, v in sorted(per_zoom.items())}, "tiles": tiles}
    o = out["overall"]
    if o:
        print("%-40s tiles %4d exact %4d  IoU px %.4f mean %.4f p5 %.4f min %.4f  cov B/A %.3f  |dRGB| %.3f  "
              "PSNR %.2f  errors same %d differ %d onlyA %d onlyB %d" % (
                  os.path.basename(os.path.normpath(args.b)), o["tiles"], o["exact"], o["iou_pixels"], o["iou_mean"],
                  o["iou_p5"], o["iou_min"], o["coverage_b_over_a"], o["mean_abs_rgb"], o["psnr"], total["err_same"],
                  total["err_diff"], total["only_a_ok"], total["only_b_ok"]))
        if not args.quiet:
            for z, s in out["per_zoom"].items():
                if s:
                    print("   z%-3s tiles %4d exact %4d  IoU px %.4f mean %.4f min %.4f  cov B/A %.3f  |dRGB| %.3f  "
                          "PSNR %.2f" % (z, s["tiles"], s["exact"], s["iou_pixels"], s["iou_mean"], s["iou_min"],
                                         s["coverage_b_over_a"], s["mean_abs_rgb"], s["psnr"]))
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=1)


if __name__ == "__main__":
    main()
