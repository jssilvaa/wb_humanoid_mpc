#!/usr/bin/env bash
# B3 rung 4: seed-averaged reactive-stepping campaign.
#
# auto (capture-point-triggered stepping) vs off (baseline, no step) -- fall rate vs push magnitude,
# sagittal (+x) and lateral (+y). "Seeds" are the async MPC-thread / control-loop timing jitter
# (each repeat is a fresh draw; the sim + push are otherwise deterministic), same methodology as the
# B1/B2 campaigns. Fall = min base z < 0.5 m (report section 07), computed by b3_stats.py from min_z.
#
# Runs from build_standalone (the CppAD codegen cache is CWD-relative). Output (gitignored):
#   experiments/results/b3_step_sweep.csv  (dir,mag,arm,seed,min_z,steps,fell)
# Env: M (seeds/cell, default 8), SIM (sim seconds, default 5).
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT/build_standalone"
OUT="$ROOT/experiments/results/b3_step_sweep.csv"
M=${M:-8}
SIM=${SIM:-5}

echo "dir,mag,arm,seed,min_z,steps,fell" > "$OUT"

run() {  # dir fx fy mag trigger(=arm)
  local dir=$1 fx=$2 fy=$3 mag=$4 trig=$5
  for ((s = 1; s <= M; s++)); do
    out=$(./bin/stepProbe "$SIM" "$fx" "$fy" 2.0 0.1 "$trig" R "" 2>/dev/null | grep '^B3SUMMARY' || true)
    if [[ -z "$out" ]]; then
      minz=0; steps=0; fell=1
      echo "  WARN no summary: $dir $mag $trig seed $s (treated as fall)" >&2
    else
      minz=$(echo "$out" | sed -E 's/.*min_z=([0-9.eE+-]+).*/\1/')
      steps=$(echo "$out" | sed -E 's/.*steps=([0-9]+).*/\1/')
      fell=$(echo "$out" | sed -E 's/.*fell=([01]).*/\1/')
    fi
    echo "$dir,$mag,$trig,$s,$minz,$steps,$fell" >> "$OUT"
  done
  echo "  done: $dir $mag $trig (M=$M)" >&2
}

for mag in 80 100 120 150 180 200; do
  run sag "$mag" 0 "$mag" off
  run sag "$mag" 0 "$mag" auto
done
for mag in 120 150 180 200 250; do
  run lat 0 "$mag" "$mag" off
  run lat 0 "$mag" "$mag" auto
done
echo "ALL DONE -> $OUT" >&2
