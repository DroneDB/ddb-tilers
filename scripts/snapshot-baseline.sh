#!/usr/bin/env bash
# SPDX-License-Identifier: MPL-2.0
# Freeze the benchmark baseline: the official DroneDB 2.4.1 release (git cbc4bffb, libddb ABI 2.4).
#
# Extracts the release .deb into <work>/baseline/ddb-2.4.1/, writes <work>/baseline/MANIFEST.json
# (sha256 + size of every file, versions, ABI, shared-object resolution) and <work>/baseline/baseline.env
# (the environment every baseline run sources), then smoke-tests the snapshot (CLI help, COG build,
# COPC build with Untwine and with PDAL, tiles through the libddb C API) into <work>/out/baseline/smoke/.
#
# Idempotent: when the snapshot already exists it is only verified against its manifest.
#
# Usage: scripts/snapshot-baseline.sh [--work DIR] [--deb FILE] [--no-smoke]
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK=/mnt/data/ddb-tilers-work
DEB="${DDB_BASELINE_DEB:-}"
SMOKE=1
DEB_NAME=ddb_2.4.1_amd64.deb
DEB_SHA=95484a26d92f79d5c74ab2172020c8b0c1b04691b9b82a4e645d2cf412505e03
TAG=v2.4.1

while [ $# -gt 0 ]; do
  case "$1" in
    --work) WORK="$2"; shift 2 ;;
    --deb) DEB="$2"; shift 2 ;;
    --no-smoke) SMOKE=0; shift ;;
    -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

BASE="$WORK/baseline"
SNAP="$BASE/ddb-2.4.1"
KEEP_DEB="$BASE/$DEB_NAME"
mkdir -p "$BASE"

check_sha() { [ "$(sha256sum "$1" | cut -d' ' -f1)" = "$DEB_SHA" ]; }

# 1. the .deb: kept next to the snapshot, sha256 pinned
if [ ! -f "$KEEP_DEB" ] || ! check_sha "$KEEP_DEB"; then
  if [ -n "$DEB" ] && [ -f "$DEB" ]; then
    cp -- "$DEB" "$KEEP_DEB.part"
  else
    echo "downloading $DEB_NAME from the DroneDB $TAG release"
    ( cd "$BASE" && gh release download "$TAG" -R DroneDB/DroneDB -p "$DEB_NAME" -O "$DEB_NAME.part" --clobber )
  fi
  check_sha "$KEEP_DEB.part" || { echo "sha256 mismatch for $DEB_NAME (want $DEB_SHA)" >&2; rm -f "$KEEP_DEB.part"; exit 1; }
  mv -- "$KEEP_DEB.part" "$KEEP_DEB"
fi
echo "deb ok: $KEEP_DEB ($DEB_SHA)"

# 2. extraction (never overwrites an existing snapshot)
if [ ! -d "$SNAP/usr" ]; then
  mkdir -p "$SNAP"
  dpkg-deb -x "$KEEP_DEB" "$SNAP"
  echo "extracted to $SNAP"
else
  echo "snapshot exists, not re-extracting: $SNAP"
fi

# 3. environment file: nothing may resolve into /mnt/data/repos/DroneDB/build
cat > "$BASE/baseline.env" <<'EOF'
# Source this file (". baseline.env") to run the frozen baseline: official DroneDB 2.4.1
# (git cbc4bffb, libddb ABI 2.4). Written by scripts/snapshot-baseline.sh.
_snap_root="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
export BASELINE_SNAPSHOT="$_snap_root/ddb-2.4.1"
# A developer shell may export DDB_*_PATH / DDB_DATA pointing into /mnt/data/repos/DroneDB/build
# (an unreachable DDB_UNTWINE_PATH silently falls back to PDAL): drop every DDB_* variable first.
for _v in $(compgen -e | grep -E '^DDB_'); do unset "$_v"; done
unset _v
export LD_LIBRARY_PATH="$BASELINE_SNAPSHOT/usr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PATH="$BASELINE_SNAPSHOT/usr/bin:$PATH"
export DDB_DATA="$BASELINE_SNAPSHOT/usr/share/ddb"
export DDB_UNTWINE_PATH="$BASELINE_SNAPSHOT/usr/bin/untwine"
export DDB_BUILDLOD_PATH="$BASELINE_SNAPSHOT/usr/bin/build-lod"
export DDB_OBJ2TILES_PATH="$BASELINE_SNAPSHOT/usr/bin/Obj2Tiles"
export DDB_DWG2DXF_PATH="$BASELINE_SNAPSHOT/usr/bin/dwg2dxf"
export PROJ_DATA="$BASELINE_SNAPSHOT/usr/share/ddb"
export PROJ_LIB="$BASELINE_SNAPSHOT/usr/share/ddb"
export GDAL_DATA="$BASELINE_SNAPSHOT/usr/share/ddb/gdal"
export GDAL_PAM_ENABLED=NO
# Per scenario the runner sets a fresh, empty DDB_HOME (never ~/.ddb), for example
#   export DDB_HOME=/mnt/data/ddb-tilers-work/home/<scenario>
# COPC backend: DDB_COPC_BACKEND=untwine|pdal (auto = Untwine when its binary is found).
unset _snap_root
EOF

# 4. manifest (resolution is checked under the baseline environment); verify when it exists
if [ -f "$BASE/MANIFEST.json" ]; then
  ( . "$BASE/baseline.env"; python3 -I "$HERE/baseline_manifest.py" --snapshot "$SNAP" --deb "$KEEP_DEB" --out "$BASE/MANIFEST.json" --verify )
else
  ( . "$BASE/baseline.env"; python3 -I "$HERE/baseline_manifest.py" --snapshot "$SNAP" --deb "$KEEP_DEB" --out "$BASE/MANIFEST.json" )
fi

# 5. freeze: read-only tree
chmod -R a-w "$SNAP"

[ "$SMOKE" = 1 ] || exit 0

# 6. smoke test, with a fresh DDB_HOME
IN_COG=/mnt/data/repos/test_data/ortho/brighton-beach.tif
IN_LAZ=/mnt/data/repos/test_data/point-clouds/brighton-beach.laz
OUT="$WORK/out/baseline/smoke"
mkdir -p "$WORK/home" "$OUT"
HOME_DIR="$(mktemp -d "$WORK/home/smoke-XXXXXX")"
GUARD="$(mktemp -d "$WORK/tmp/guard-XXXXXX" 2>/dev/null || mktemp -d)"
find /mnt/data/repos/test_data -printf '%p\t%s\t%T@\n' | sort > "$GUARD/before.txt"

(
  . "$BASE/baseline.env"
  export DDB_HOME="$HOME_DIR"
  [ "$(command -v ddb)" = "$SNAP/usr/bin/ddb" ] || { echo "ddb does not resolve into the snapshot" >&2; exit 1; }
  ddb --version
  ddb --help > "$GUARD/help.txt"
  grep -qE '^\s+cog - ' "$GUARD/help.txt" && grep -qE '^\s+copc - ' "$GUARD/help.txt" && grep -qE '^\s+tile - ' "$GUARD/help.txt" \
    || { echo "ddb --help does not list cog/copc/tile" >&2; exit 1; }

  rm -rf "$OUT/cog" "$OUT/copc-untwine" "$OUT/copc-pdal" "$OUT/tiles"
  mkdir -p "$OUT/cog"
  ddb cog --overwrite "$OUT/cog/brighton-beach-cog.tif" "$IN_COG" >/dev/null
  ls -l "$OUT/cog"

  ddb copc --debug "$OUT/copc-untwine" "$IN_LAZ" > "$GUARD/untwine.log" 2>&1
  grep -q "Spawning untwine" "$GUARD/untwine.log" || { echo "Untwine backend was not used" >&2; exit 1; }
  DDB_COPC_BACKEND=pdal ddb copc --debug "$OUT/copc-pdal" "$IN_LAZ" > "$GUARD/pdal.log" 2>&1
  grep -q "PDAL writers.copc" "$GUARD/pdal.log" || { echo "PDAL backend was not used" >&2; exit 1; }
  ls -l "$OUT/copc-untwine" "$OUT/copc-pdal"

  python3 -I "$HERE/baseline_smoke.py" --snapshot "$SNAP" --cog "$OUT/cog/brighton-beach-cog.tif" \
    --copc "$OUT/copc-untwine/cloud.copc.laz" --out "$OUT/tiles"
)

find /mnt/data/repos/test_data -printf '%p\t%s\t%T@\n' | sort > "$GUARD/after.txt"
if ! cmp -s "$GUARD/before.txt" "$GUARD/after.txt"; then
  echo "test_data changed during the smoke test:" >&2
  diff "$GUARD/before.txt" "$GUARD/after.txt" >&2 || true
  exit 1
fi
rm -rf "$GUARD" "$HOME_DIR"
echo "baseline snapshot OK: $SNAP"
