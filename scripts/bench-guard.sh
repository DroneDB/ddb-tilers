#!/usr/bin/env bash
# SPDX-License-Identifier: MPL-2.0
# Quick regression guard on the smoke suite.
#
#   scripts/bench-guard.sh [--mode timed|cachegrind|both] [--backend SPEC] [--update] [--reps N]
#
#   timed       ~5 min: run.py --suite smoke --reps 3, checked against bench/guard/baseline.json
#               (p50/p99/wall <= x1.10, RSS <= x1.10, tile set hashes unchanged, no new failures)
#   cachegrind  deterministic instruction counts (valgrind --tool=cachegrind --cache-sim=no) of
#               ddbt-bench tile on small fixed request sets: Ir <= x1.01, set hashes unchanged
#   --update    store the current values as the reference
# Default backend: ddbt (build/linux-gcc-release/src/libddbt.so). Exit: 0 pass, 1 regression,
# 2 no reference yet.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/.." && pwd)"
work="${DDBT_WORK:-/mnt/data/ddb-tilers-work}"

# The driver: reuse the standalone build, (re)building it when its sources are newer.
driver="${DDBT_BENCH_BIN:-$work/build-bench/ddbt-bench}"
if [[ -z "${DDBT_BENCH_BIN:-}" ]]; then
  cmake -S "$repo/bench" -B "$work/build-bench" -DCMAKE_BUILD_TYPE=Release >/dev/null
  cmake --build "$work/build-bench" --target ddbt-bench >/dev/null
fi

exec python3 "$repo/bench/harness/guard.py" --driver "$driver" "$@"
