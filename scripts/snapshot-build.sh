#!/usr/bin/env bash
# SPDX-License-Identifier: MPL-2.0
# Freeze a local DroneDB build for the benchmark harness, laid out like the release snapshot of
# scripts/snapshot-baseline.sh: <work>/snapshots/<name>/usr/{bin,lib,share/ddb} with libddb.so,
# the ddb CLI and its helper programs, the shared libraries libddb loads from the build's vcpkg
# tree (PDAL), and the data files; plus <work>/snapshots/<name>.env, the environment its runs
# source (nothing resolves into the build directory or a developer's DDB_* variables).
#
#   scripts/snapshot-build.sh <droneDB build dir> <name> [--work DIR]
#
# Then compare two builds with the harness, e.g.
#   run.py --suite std --backend A=legacy:<snap A>/usr/lib/libddb.so:<snap A>/usr/bin/ddb:<snap A>.env \
#                      --backend B=legacy:<snap B>/usr/lib/libddb.so:<snap B>/usr/bin/ddb:<snap B>.env
# An existing snapshot of that name is replaced.
set -euo pipefail

WORK="${DDBT_WORK:-/mnt/data/ddb-tilers-work}"
[ $# -ge 2 ] || { sed -n '3,14p' "$0"; exit 2; }
BUILD="$(cd "$1" && pwd)"; NAME="$2"; shift 2
while [ $# -gt 0 ]; do
  case "$1" in
    --work) WORK="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[ -f "$BUILD/libddb.so" ] && [ -x "$BUILD/ddbcmd" ] || { echo "not a DroneDB build directory: $BUILD" >&2; exit 1; }

SNAP="$WORK/snapshots/$NAME"
rm -rf "$SNAP"
mkdir -p "$SNAP/usr/bin" "$SNAP/usr/lib" "$SNAP/usr/share/ddb"

cp -a "$BUILD/libddb.so" "$SNAP/usr/lib/"
cp -a "$BUILD/ddbcmd" "$SNAP/usr/bin/ddb"
for helper in gdal-util metadata-util bm3d untwine; do
  [ -x "$BUILD/$helper" ] && cp -a "$BUILD/$helper" "$SNAP/usr/bin/"
done
# Shared libraries libddb resolves outside the system (PDAL and its plugins)
ldd "$BUILD/libddb.so" | awk '/=> \// {print $3}' | grep -v -E '^/(usr/)?lib' | while read -r lib; do
  cp -L "$lib" "$SNAP/usr/lib/"
done
# Data files the build copies next to its binaries
for f in proj.db sensor-profiles.json sensor_data.sqlite timezone21.bin curl-ca-bundle.crt gdal zoneinfo; do
  [ -e "$BUILD/$f" ] && cp -a "$BUILD/$f" "$SNAP/usr/share/ddb/"
done

cat > "$WORK/snapshots/$NAME.env" <<EOF
# Source this file to run the DroneDB build snapshot "$NAME" (from $BUILD, $(date -u +%FT%TZ)).
# Written by scripts/snapshot-build.sh.
for _v in \$(compgen -e | grep -E '^DDB_'); do unset "\$_v"; done
unset _v
export LD_LIBRARY_PATH="$SNAP/usr/lib"
export PATH="$SNAP/usr/bin:\$PATH"
export DDB_DATA="$SNAP/usr/share/ddb"
export PROJ_DATA="$SNAP/usr/share/ddb"
export PROJ_LIB="$SNAP/usr/share/ddb"
export GDAL_DATA="$SNAP/usr/share/ddb/gdal"
export GDAL_PAM_ENABLED=NO
EOF
[ -x "$SNAP/usr/bin/untwine" ] && echo "export DDB_UNTWINE_PATH=\"$SNAP/usr/bin/untwine\"" >> "$WORK/snapshots/$NAME.env"

# Smoke test: the snapshot loads its own libddb and nothing from the build directory
( . "$WORK/snapshots/$NAME.env"
  if LD_DEBUG=libs "$SNAP/usr/bin/ddb" --version 2>&1 | grep -q "$BUILD/"; then
    echo "the snapshot still loads files from $BUILD" >&2; exit 1
  fi
  "$SNAP/usr/bin/ddb" --version )
echo "snapshot $SNAP ($(du -sh "$SNAP" | cut -f1)), env $WORK/snapshots/$NAME.env"
