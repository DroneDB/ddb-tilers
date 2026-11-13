#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""End-to-end comparison of the tile encoder settings: server, transfer and client decode.

For every set (a request file of the benchmark results) and variant (PNG with libdeflate or fpnge,
WebP method 2 / 1 / 0), it measures with ddbt-bench:
  - server: warm tiles/s and p50 at the given thread counts (encoded-tile caches off, so every
    request renders and encodes), and the bytes of one round;
  - client: the tiles of one round dumped and decoded again (Pillow: libpng / libwebp, one core),
    as a stand-in for a browser or QGIS;
and writes a JSON file plus a Markdown report with a link model: per tile, the server p50 + the
transfer of its mean size at 10 / 50 / 100 / 1000 Mbit/s + its decode; per screenful (1920 x 1080
of 256 px tiles = 40 tiles, or 40 tiles of 512 px with @2x), the bytes, the transfer time and the
server CPU time.

  e2e_encoders.py --driver ddbt-bench --lib libddbt.so --out DIR --set NAME=REQUESTS[:SIZE] ...
                  [--threads 1,8,16] [--cpus 1T=1 8T=0-7 16T=0-7,16-23] [--reps 3]
"""
import argparse
import glob
import io
import json
import os
import statistics
import subprocess
import sys
import time

VARIANTS = {
    "png-deflate": ("png", {"DDBT_PNG_ENGINE": "deflate"}),
    "png-fpnge": ("png", {"DDBT_PNG_ENGINE": "fpnge"}),
    "webp-m2": ("webp", {"DDBT_WEBP_METHOD": "2"}),
    "webp-m1": ("webp", {"DDBT_WEBP_METHOD": "1"}),
    "webp-m0": ("webp", {"DDBT_WEBP_METHOD": "0"}),
}
LINKS_MBIT = [10, 50, 100, 1000]
SCREEN_TILES = 40


def cpulist(spec):
    out = []
    for part in spec.split(","):
        if "-" in part:
            a, b = part.split("-")
            out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


def base_env(home):
    env = {k: v for k, v in os.environ.items() if not k.startswith("DDB_")}
    env.update({"GDAL_PAM_ENABLED": "NO", "DDBT_TILE_CACHE_MB": "0", "DDBT_COPC_TILE_CACHE_MB": "0", "DDB_HOME": home})
    return env


def run_bench(a, req, size, fmt, extra_env, threads, out, dump=None, rounds=3, warmup=1):
    cpus = a.cpus["%dT" % threads]
    cmd = ["taskset", "-c", ",".join(map(str, a.allcpus)), a.driver, "tile", "--backend", "ddbt", "--lib", a.lib,
           "--requests", req, "--threads", str(threads), "--pin", ",".join(map(str, cpus[:threads])),
           "--warmup-rounds", str(warmup), "--rounds", str(rounds), "--format", fmt, "--size", str(size), "--out", out]
    if dump:
        cmd += ["--dump-tiles", dump]
    env = base_env(os.path.join(a.out, "home"))
    env.update(extra_env)
    subprocess.run(cmd, env=env, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    j = json.load(open(out))
    r = j["round_results"][-1]
    return {"p50_ms": r["latency_ms"]["p50"], "p99_ms": r["latency_ms"]["p99"], "tiles_per_s": r["tiles_per_s"],
            "bytes": r["bytes"], "tiles": r["tiles"], "ok": r["ok"],
            "cpu_s": (r.get("cpu") or {}).get("user_s", 0.0) + (r.get("cpu") or {}).get("sys_s", 0.0)}


def decode_ms(files):
    from PIL import Image
    data = [open(f, "rb").read() for f in files]
    best = None
    for _ in range(3):
        t0 = time.perf_counter()
        for d in data:
            im = Image.open(io.BytesIO(d))
            im.load()
        dt = time.perf_counter() - t0
        best = dt if best is None else min(best, dt)
    return 1000.0 * best / max(1, len(data))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--driver", required=True)
    ap.add_argument("--lib", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--set", action="append", required=True, help="NAME=REQUESTS.jsonl[:SIZE]")
    ap.add_argument("--variants", default=",".join(VARIANTS))
    ap.add_argument("--threads", default="1,8,16")
    ap.add_argument("--cpus", nargs="*", default=["1T=1", "8T=0-7", "16T=0-7,16-23"])
    ap.add_argument("--decode-cpu", default="1")
    ap.add_argument("--reps", type=int, default=3)
    a = ap.parse_args()
    a.cpus = {k: cpulist(v) for k, v in (c.split("=", 1) for c in a.cpus)}
    a.allcpus = sorted({c for v in a.cpus.values() for c in v})
    os.makedirs(os.path.join(a.out, "home"), exist_ok=True)
    os.sched_setaffinity(0, cpulist(a.decode_cpu))
    threads = [int(t) for t in a.threads.split(",")]
    results = []
    for s in a.set:
        name, spec = s.split("=", 1)
        req, _, size = spec.partition(":")
        size = int(size or 256)
        for v in a.variants.split(","):
            fmt, env = VARIANTS[v]
            row = {"set": name, "size": size, "variant": v, "server": {}}
            dump = os.path.join(a.out, "dump", name, v)
            subprocess.run(["rm", "-rf", dump], check=True)
            first = run_bench(a, req, size, fmt, env, 1, os.path.join(a.out, "tmp.json"), dump=dump, rounds=1, warmup=0)
            files = sorted(glob.glob(os.path.join(dump, "*." + fmt)))
            row["bytes_per_tile"] = first["bytes"] / max(1, first["tiles"])
            row["decode_ms"] = decode_ms(files)
            for t in threads:
                reps = [run_bench(a, req, size, fmt, env, t, os.path.join(a.out, "tmp.json")) for _ in range(a.reps)]
                row["server"]["%dT" % t] = {k: statistics.median(r[k] for r in reps)
                                            for k in ("p50_ms", "p99_ms", "tiles_per_s", "cpu_s", "bytes")}
            subprocess.run(["rm", "-rf", dump], check=True)
            results.append(row)
            print("%-5s %-12s %7.0f B/tile decode %.3f ms  1T p50 %.3f ms  %s" % (
                name, v, row["bytes_per_tile"], row["decode_ms"], row["server"]["1T"]["p50_ms"],
                "  ".join("%s %.0f t/s" % (k, x["tiles_per_s"]) for k, x in row["server"].items())), flush=True)
            json.dump(results, open(os.path.join(a.out, "e2e.json"), "w"), indent=1)
    write_report(results, threads, os.path.join(a.out, "e2e.md"))


def write_report(results, threads, path):
    lines = ["# Encoder end-to-end comparison", ""]
    lines.append("| set | variant | bytes/tile | decode ms | " + " | ".join("%dT tiles/s" % t for t in threads) +
                 " | 1T p50 ms | " + " | ".join("tile ms @%d Mbit/s" % l for l in LINKS_MBIT) +
                 " | screenful KB | screenful s @50 Mbit/s |")
    lines.append("|" + "---|" * (7 + len(threads) + len(LINKS_MBIT)))
    for r in results:
        b = r["bytes_per_tile"]
        p50 = r["server"]["1T"]["p50_ms"]
        per = [p50 + b * 8 / (l * 1e6) * 1000 + r["decode_ms"] for l in LINKS_MBIT]
        lines.append("| %s | %s | %.0f | %.3f | %s | %.3f | %s | %.0f | %.2f |" % (
            r["set"] + ("@2x" if r["size"] == 512 else ""), r["variant"], b, r["decode_ms"],
            " | ".join("%.0f" % r["server"]["%dT" % t]["tiles_per_s"] for t in threads), p50,
            " | ".join("%.2f" % x for x in per), SCREEN_TILES * b / 1024, SCREEN_TILES * b * 8 / 50e6))
    lines += ["", "Tile ms = 1T server p50 + transfer of the mean tile at the link speed + client decode (one core).",
              "Screenful = %d tiles (1920 x 1080 at 256 px, or at 512 px with @2x)." % SCREEN_TILES]
    open(path, "w").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    sys.exit(main())
