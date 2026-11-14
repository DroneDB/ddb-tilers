#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
r"""Writes the expectations of a parity test: what GDAL reads in each raster of a folder, as the
lines of gdal_expected (gdal_expected.cpp, built against the GDAL libddb ships), one file after
the other.

    python3 -I tests/fixtures/tools/gdal_expected.py --tool gdal_expected --ext .jp2 --ext .j2k \
        --driver JP2OpenJPEG DIR OUT.tsv

--ext (repeatable) selects the files by extension (case-insensitive); --driver (repeatable)
restricts the GDAL drivers that may open them (any driver when none is given); --grid-lossy adds
the sample grids of the files GDAL reports as lossy (gdal_expected.cpp), for formats decoded in
floating point.
"""
import argparse
import os
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tool", required=True, help="gdal_expected built against the GDAL libddb ships")
    parser.add_argument("--ext", action="append", required=True)
    parser.add_argument("--driver", action="append", default=[])
    parser.add_argument("--grid-lossy", action="store_true")
    parser.add_argument("folder")
    parser.add_argument("out")
    args = parser.parse_args()
    exts = tuple(e.lower() for e in args.ext)
    lines = []
    for name in sorted(os.listdir(args.folder)):
        if not name.lower().endswith(exts):
            continue
        # Bytes, not text: a CRS name may hold carriage returns, which are kept as they are.
        grid = ["--grid-lossy"] if args.grid_lossy else []
        out = subprocess.run([args.tool, os.path.join(args.folder, name), name] + grid + args.driver, check=True,
                             capture_output=True).stdout
        if out.startswith(b"refused"):
            print(f"refused {name}")
        lines += [line for line in out.split(b"\n") if line]
    with open(args.out, "wb") as f:
        for line in lines:
            f.write(line + b"\n")
    print(f"wrote {len(lines)} lines to {args.out}")


if __name__ == "__main__":
    main()
