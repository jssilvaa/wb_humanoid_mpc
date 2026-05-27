#!/usr/bin/env bash
# B5 rung D: seed-averaged integration campaign -- observer feedforward + reactive stepping.
#
# Four arms on the sagittal (+x) push, fall rate vs magnitude:
#   off  : no ADR (baseline reference)
#   ff   : observer feedforward only (B2)
#   step : reactive stepping only (B3)
#   both : feedforward + stepping (B5 integration; FF kept on through swing)
# "Seeds" are the async MPC-thread / control-loop timing jitter (each repeat a fresh draw; sim + push
# otherwise deterministic), same methodology as B1/B2/B3. Fall = min base z < 0.5 m (report section 07).
# The grid extends down to 70 N (the B2 in-place floor) so the FF band and the stepping band both show.
#
# Runs from build_standalone (CppAD codegen cache is CWD-relative). Output (gitignored):
#   experiments/results/b5_step_sweep.csv  (arm,mag,seed,min_z,steps,first_step_t,fell)
# Env: M (seeds/cell, default 8), SIM (sim seconds, default 5).
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT/build_standalone"
OUT="$ROOT/experiments/results/b5_step_sweep.csv"
M=${M:-8}
SIM=${SIM:-5}

echo "mag,arm,seed,min_z,steps,first_step_t,fell" > "$OUT"

run() {  # mag arm(=mode)
  local mag=$1 arm=$2
  for ((s = 1; s <= M; s++)); do
    out=$(./bin/b5Probe "$SIM" "$mag" 0 2.0 0.1 "$arm" R "" 2>/dev/null | grep '^B5SUMMARY' || true)
    if [[ -z "$out" ]]; then
      minz=0; steps=0; fst=-1; fell=1
      echo "  WARN no summary: $mag $arm seed $s (treated as fall)" >&2
    else
      minz=$(echo "$out" | sed -E 's/.*min_z=([0-9.eE+-]+).*/\1/')
      steps=$(echo "$out" | sed -E 's/.*steps=([0-9]+).*/\1/')
      fst=$(echo "$out" | sed -E 's/.*first_step_t=([0-9.eE+-]+).*/\1/')
      fell=$(echo "$out" | sed -E 's/.*fell=([01]).*/\1/')
    fi
    echo "$mag,$arm,$s,$minz,$steps,$fst,$fell" >> "$OUT"
  done
  echo "  done: $mag $arm (M=$M)" >&2
}

for mag in 70 90 100 120 150 180 200; do
  for arm in off ff step both; do
    run "$mag" "$arm"
  done
done
echo "ALL DONE -> $OUT" >&2
