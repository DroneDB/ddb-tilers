#!/usr/bin/env bash
# SPDX-License-Identifier: MPL-2.0
# Fill refs/ (gitignored) with read-only reference sources, offline first.
#
#   refs/gdal-3.13.3/   GDAL 3.13.3     from a local tarball (DroneDB vcpkg downloads)
#   refs/pdal-2.10.1/   PDAL 2.10.1     from a local tarball
#   refs/proj-9.9.0/    PROJ 9.9.0      from a local tarball (the version libddb 2.4.1 ships); else /mnt/data/repos/PROJ
#   refs/laz-perf/      laz-perf        git clone --local of /mnt/data/repos/laz-perf at its HEAD (commit recorded)
#   refs/untwine/       Untwine (GPL)   git clone --local of /mnt/data/repos/untwine at the DroneDB submodule commit
#   refs/SOURCES.txt    what was fetched: source, version, sha256 / commit
#
# With --online it also makes treeless clones (git clone --filter=blob:none) of OSGeo/gdal at v3.13.3 and
# PDAL/PDAL at 2.10.1 into refs/gdal-git/ and refs/pdal-git/, for `git log` / `git blame` history.
#
# Idempotent: an existing, complete source is left alone. Nothing outside refs/ is written and the
# local source repositories are only read. Untwine is GPL: reference only, never copy code from it.
#
# Usage: scripts/fetch-refs.sh [--online] [--refs DIR]
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REFS="$HERE/../refs"
ONLINE=0
REPOS=/mnt/data/repos
DL="$REPOS/DroneDB/vcpkg/downloads"

while [ $# -gt 0 ]; do
  case "$1" in
    --online) ONLINE=1; shift ;;
    --refs) REFS="$2"; shift 2 ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
mkdir -p "$REFS"
REFS="$(cd "$REFS" && pwd)"
SOURCES="$REFS/SOURCES.txt"
TMP_SOURCES="$(mktemp)"
trap 'rm -f "$TMP_SOURCES"' EXIT

first_existing() { for f in "$@"; do [ -f "$f" ] && { echo "$f"; return 0; }; done; return 1; }

# extract_tarball <dir name> <expected top folder> <tarball candidates...>
extract_tarball() {
  local name="$1" top="$2"; shift 2
  local dest="$REFS/$name" tb
  if tb="$(first_existing "$@")"; then
    local sum; sum="$(sha256sum "$tb" | cut -d' ' -f1)"
    if [ -f "$dest/.fetched-from" ] && grep -q "$sum" "$dest/.fetched-from"; then
      echo "ok (present): $name"
    else
      [ -e "$dest" ] && { chmod -R u+w "$dest" 2>/dev/null || true; rm -rf "$dest"; }
      local stage; stage="$(mktemp -d "$REFS/.stage-XXXXXX")"
      tar -xzf "$tb" -C "$stage"
      [ -d "$stage/$top" ] || { echo "unexpected layout in $tb (no $top/)" >&2; rm -rf "$stage"; return 1; }
      mv "$stage/$top" "$dest"; rmdir "$stage"
      printf '%s\n%s\n' "$tb" "$sum" > "$dest/.fetched-from"
      chmod -R a-w "$dest"
      echo "extracted: $name <- $tb"
    fi
    printf '%s\t%s\ttarball %s\tsha256 %s\n' "$name" "$top" "$tb" "$sum" >> "$TMP_SOURCES"
    return 0
  fi
  return 1
}

# git_local <dir name> <source repo> [commit]: git clone --local, detached at the commit (default: source HEAD)
git_local() {
  local name="$1" src="$2" commit="${3:-}"
  local dest="$REFS/$name"
  [ -d "$src/.git" ] || [ -f "$src/.git" ] || { echo "missing source repo: $src" >&2; return 1; }
  [ -n "$commit" ] || commit="$(git -C "$src" rev-parse HEAD)"
  if [ ! -d "$dest/.git" ]; then
    git clone --quiet --local --no-hardlinks "$src" "$dest"
  fi
  if [ "$(git -C "$dest" rev-parse HEAD 2>/dev/null || true)" != "$commit" ]; then
    git -C "$dest" -c advice.detachedHead=false checkout --quiet --detach "$commit"
  fi
  local desc; desc="$(git -C "$dest" describe --tags --always 2>/dev/null || echo "$commit")"
  echo "ok: $name at $commit ($desc)"
  printf '%s\tgit %s\tcommit %s\tdescribe %s\n' "$name" "$src" "$commit" "$desc" >> "$TMP_SOURCES"
}

# --- GDAL 3.13.3, PDAL 2.10.1, PROJ 9.9.0 -----------------------------------------------------------
extract_tarball gdal-3.13.3 gdal-3.13.3 "$DL/OSGeo-gdal-v3.13.3.tar.gz" "$REPOS/OSGeo-gdal-v3.13.3.tar.gz" \
  || { echo "GDAL 3.13.3 tarball not found locally; run with --online or put OSGeo-gdal-v3.13.3.tar.gz in $DL" >&2; exit 1; }
extract_tarball pdal-2.10.1 PDAL-2.10.1 "$DL/PDAL-PDAL-2.10.1.tar.gz" "$REPOS/PDAL-PDAL-2.10.1.tar.gz" \
  || { echo "PDAL 2.10.1 tarball not found locally; run with --online or put PDAL-PDAL-2.10.1.tar.gz in $DL" >&2; exit 1; }
if ! extract_tarball proj-9.9.0 PROJ-9.9.0 "$DL/OSGeo-PROJ-9.9.0.tar.gz" "$REPOS/OSGeo-PROJ-9.9.0.tar.gz"; then
  echo "PROJ 9.9.0 tarball not found: using the PROJ git checkout"
  git_local proj-git "$REPOS/PROJ"
fi

# --- laz-perf (HEAD of the local clone) and Untwine (the commit DroneDB pins) ------------------------
git_local laz-perf "$REPOS/laz-perf"
UNTWINE_COMMIT="$(git -C "$REPOS/DroneDB" ls-tree HEAD vendor/untwine 2>/dev/null | awk '{print $3}')"
UNTWINE_SRC="$REPOS/untwine"
if [ -n "$UNTWINE_COMMIT" ] && ! git -C "$UNTWINE_SRC" cat-file -e "$UNTWINE_COMMIT^{commit}" 2>/dev/null; then
  UNTWINE_SRC="$REPOS/DroneDB/vendor/untwine"
fi
git_local untwine "$UNTWINE_SRC" "$UNTWINE_COMMIT"

# --- optional online history (treeless clones at the same tags) --------------------------------------
if [ "$ONLINE" = 1 ]; then
  online_clone() {
    local name="$1" url="$2" tag="$3" dest="$REFS/$1"
    if [ -d "$dest/.git" ]; then
      echo "ok (present): $name"
    else
      git clone --filter=blob:none --branch "$tag" "$url" "$dest"
    fi
    printf '%s\tgit %s\ttag %s\tcommit %s\n' "$name" "$url" "$tag" "$(git -C "$dest" rev-parse HEAD)" >> "$TMP_SOURCES"
  }
  online_clone gdal-git https://github.com/OSGeo/gdal.git v3.13.3
  online_clone pdal-git https://github.com/PDAL/PDAL.git 2.10.1
fi

{
  echo "# written by scripts/fetch-refs.sh ($(date -u +%Y-%m-%dT%H:%M:%SZ)); name, source, version / commit"
  cat "$TMP_SOURCES"
} > "$SOURCES"
echo "refs ready: $REFS (see $SOURCES)"
