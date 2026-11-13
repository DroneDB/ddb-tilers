#!/usr/bin/env bash
# SPDX-License-Identifier: MPL-2.0
# A/B benchmark of two commits of ddb-tilers.
#
#   scripts/bench-ab.sh <base-ref> <head-ref> --suite <smoke|std|full|file.toml> [run.py options...]
#
# Each ref is checked out (detached) in a git worktree under $DDBT_WORK/wt/<sha> and built with
# the CMake preset $DDBT_PRESET (default linux-gcc-release; targets ddbt_shared and ddbt_cli).
# Both libddbt.so run interleaved (ABBA) in ONE run.py invocation, with the same ddbt-bench
# driver (built from the current checkout), then compare.py prints the A/B table.
# Extra options go to run.py, e.g. --items R1,P2 --only tile --perf-stat --reps 7.
# --with-legacy adds the frozen baseline as a third backend. The worktrees are kept (rebuilds
# are incremental); remove one with: git worktree remove $DDBT_WORK/wt/<sha>
set -euo pipefail
usage() { sed -n '3,13p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[[ $# -ge 2 ]] || usage
base_ref="$1"; head_ref="$2"; shift 2
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/.." && pwd)"
work="${DDBT_WORK:-/mnt/data/ddb-tilers-work}"
preset="${DDBT_PRESET:-linux-gcc-release}"

suite=""
with_legacy=0
passthru=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --suite) suite="$2"; passthru+=("$1" "$2"); shift 2 ;;
    --with-legacy) with_legacy=1; shift ;;
    -h|--help) usage ;;
    *) passthru+=("$1"); shift ;;
  esac
done
[[ -n "$suite" ]] || { echo "bench-ab: --suite is required" >&2; exit 2; }

resolve() { git -C "$repo" rev-parse --verify "$1^{commit}"; }
base_sha="$(resolve "$base_ref")"
head_sha="$(resolve "$head_ref")"
[[ "$base_sha" != "$head_sha" ]] || echo "bench-ab: warning: base and head are the same commit (an A/A run)" >&2

build_ref() {  # sha -> prints "lib:cli"
  local sha="$1" dir="$work/wt/$1"
  if [[ ! -d "$dir" ]]; then
    mkdir -p "$work/wt"
    git -C "$repo" worktree add --detach "$dir" "$sha" >&2
  fi
  echo "bench-ab: building ${sha:0:7} ($preset) in $dir" >&2
  (cd "$dir" && cmake --preset "$preset" >&2 && cmake --build --preset "$preset" --target ddbt_shared ddbt_cli >&2)
  local lib cli
  lib="$(find "$dir/build/$preset" -name 'libddbt.so' -not -path '*/vcpkg_installed/*' -print -quit)"
  cli="$(find "$dir/build/$preset" -type f -name 'ddbt' -perm -u+x -not -path '*/vcpkg_installed/*' -print -quit)"
  [[ -n "$lib" ]] || { echo "bench-ab: libddbt.so not found for ${sha:0:7}" >&2; exit 1; }
  echo "$lib:$cli"
}

base_art="$(build_ref "$base_sha")"
head_art="$(build_ref "$head_sha")"

driver="${DDBT_BENCH_BIN:-$work/build-bench/ddbt-bench}"
if [[ -z "${DDBT_BENCH_BIN:-}" ]]; then
  cmake -S "$repo/bench" -B "$work/build-bench" -DCMAKE_BUILD_TYPE=Release >/dev/null
  cmake --build "$work/build-bench" --target ddbt-bench >/dev/null
fi

b7="${base_sha:0:7}"; h7="${head_sha:0:7}"
run_id="$(date -u +%Y%m%dT%H%M%SZ)-ab-$b7-$h7-$(basename "${suite%.toml}")"
backends=(--backend "base-$b7=ddbt:$base_art" --backend "head-$h7=ddbt:$head_art")
[[ $with_legacy -eq 0 ]] || backends+=(--backend legacy)

set +e
python3 "$repo/bench/harness/run.py" --driver "$driver" "${backends[@]}" --base "base-$b7" \
  --run-id "$run_id" "${passthru[@]}"
rc=$?
set -e
results="$work/results/$run_id"
if [[ -f "$results/run.json" ]]; then
  python3 "$repo/bench/harness/compare.py" "$results/run.json" --json "$results/compare.json" | tee "$results/compare.md"
  echo "bench-ab: $results (report.md, compare.md, run.json)"
fi
exit $rc
