#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Points and nodes per depth of COPC files, read from their hierarchy (no point is decoded).

    levels.py A.copc.laz [B.copc.laz ...] [--json OUT]

One column pair (nodes, points) per file, one row per depth, then the totals; with two or more
files the points of every file relative to the first one's (e.g. ddbt's build vs Untwine's).
"""
import argparse
import json
import struct
import sys


def read_levels(path):
    with open(path, "rb") as f:
        hdr = f.read(375)
        if hdr[:4] != b"LASF":
            raise ValueError(path + ": not a LAS file")
        header_size = struct.unpack_from("<H", hdr, 94)[0]
        f.seek(header_size)
        vlr = f.read(54 + 160)
        if vlr[2:18].rstrip(b"\0") != b"copc" or struct.unpack_from("<H", vlr, 18)[0] != 1:
            raise ValueError(path + ": no COPC info VLR first")
        info = vlr[54:]
        cx, cy, cz, half, spacing, root_off, root_size = struct.unpack_from("<5dQQ", info, 0)
        nodes, points = {}, {}
        pages = [(root_off, root_size)]
        while pages:
            off, size = pages.pop()
            f.seek(off)
            data = f.read(size)
            for i in range(0, size, 32):
                d, x, y, z, o, bs, cnt = struct.unpack_from("<4iQii", data, i)
                if cnt == -1:
                    pages.append((o, bs))
                    continue
                nodes[d] = nodes.get(d, 0) + 1
                points[d] = points.get(d, 0) + max(cnt, 0)
    return {"halfSize": half, "spacing": spacing, "nodes": nodes, "points": points}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--json")
    a = ap.parse_args()
    lv = [read_levels(p) for p in a.files]
    depths = sorted(set(d for l in lv for d in l["points"]))
    head = "| depth | " + " | ".join(f"{p} nodes | points" for p in a.files)
    if len(lv) > 1:
        head += " | " + " | ".join(f"points / first ({i})" for i in range(1, len(lv)))
    print(head + " |")
    print("|" + "---|" * (1 + 2 * len(lv) + max(0, len(lv) - 1)))
    rows = []
    for d in depths + ["total"]:
        if d == "total":
            vals = [(sum(l["nodes"].values()), sum(l["points"].values())) for l in lv]
        else:
            vals = [(l["nodes"].get(d, 0), l["points"].get(d, 0)) for l in lv]
        line = f"| {d} | " + " | ".join(f"{n:,} | {p:,}" for n, p in vals)
        if len(lv) > 1:
            base = vals[0][1]
            line += " | " + " | ".join(f"{v[1] / base:.3f}" if base else "-" for v in vals[1:])
        print(line + " |")
        rows.append({"depth": d, "nodes": [v[0] for v in vals], "points": [v[1] for v in vals]})
    if a.json:
        with open(a.json, "w") as f:
            json.dump({"files": a.files, "rows": rows}, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
