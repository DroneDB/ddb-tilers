#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Builds a COG with libddbt's ddbtBuildCog (ctypes), for validation and measurements.

    build.py --lib build/linux-gcc-release/lib/libddbt.so INPUT OUTPUT [--options '{"codec":"zstd"}']

Prints the build report (JSON) and the wall time. Run it under /usr/bin/time -v (and taskset) for
CPU time and peak RSS.
"""
import argparse
import ctypes
import json
import os
import sys
import tempfile
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lib", required=True)
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--options", default="{}")
    a = ap.parse_args()
    lib = ctypes.CDLL(a.lib)
    lib.ddbtBuildCog.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p]
    lib.ddbtBuildCog.restype = ctypes.c_int
    lib.ddbtLastError.restype = ctypes.c_char_p
    opts = json.loads(a.options)
    fd, report = tempfile.mkstemp(suffix=".json", dir=os.path.dirname(os.path.abspath(a.output)))
    os.close(fd)
    opts["report"] = report
    t0 = time.monotonic()
    rc = lib.ddbtBuildCog(a.input.encode(), a.output.encode(), json.dumps(opts).encode())
    wall = time.monotonic() - t0
    if rc != 0:
        os.unlink(report)
        print(f"ddbtBuildCog failed ({rc}): {lib.ddbtLastError().decode()}", file=sys.stderr)
        return 1
    with open(report) as f:
        r = json.load(f)
    os.unlink(report)
    r["wall"] = wall
    print(json.dumps(r, indent=1, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
