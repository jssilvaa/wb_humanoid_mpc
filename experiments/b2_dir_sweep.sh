#!/usr/bin/env bash
# B2 directional generalization: does observer-driven feedforward help for non-sagittal pushes?
# Seed-averaged fall rate + excursion vs push magnitude, baseline (ff=0) vs observer FF (ff=2,
# T=0.27), for lateral (+y) and diagonal (+x+y at 45 deg) pushes at torso_link. (Sagittal is in
# b2_edge_sweep.) Brackets report section 07 minor_lateral (+y 120 N / 0.1 s).
#
# Usage: experiments/b2_dir_sweep.sh [M=10]
set -uo pipefail
ROOT=/Users/josesilvaa/wb_humanoid_mpc
BIN="$ROOT/build_standalone/bin/pushRecovery"
OUT="$ROOT/experiments/results/b2_dir_sweep.csv"
M="${1:-10}"
# Per-direction |f| grids bracketing each baseline fall edge (probed: lateral ~150-200 N, diagonal
# weaker ~110-150 N since its sagittal component dominates). "name fx_frac fy_frac mags..."
DIRS=("lateral 0 1 140 170 200" "diagonal 0.70711 0.70711 80 110 140")
SPECS=("0 inf" "2 0.27")  # baseline, observer FF at the knee T

cd "$ROOT/build_standalone"   # codegen cache path is CWD-relative
echo "dir,ff_mode,T,mag_N,fx,fy,run,peak_com_dev_m,peak_dcm_dev_m,min_base_z_m,result" > "$OUT"
for d in "${DIRS[@]}"; do
  set -- $d; DNAME=$1; FXF=$2; FYF=$3; shift 3; DMAGS=("$@")
  for MAG in "${DMAGS[@]}"; do
    FX=$(echo "$MAG * $FXF" | bc -l)
    FY=$(echo "$MAG * $FYF" | bc -l)
    for spec in "${SPECS[@]}"; do
      set -- $spec; FF=$1; T=$2
      for r in $(seq 1 "$M"); do
        log="/tmp/dir_${DNAME}_${MAG}_ff${FF}_${r}.log"
        "$BIN" 6 "$FX" "$FY" 2.0 0.1 "" "" "$FF" "$T" > "$log" 2>&1 || true
        com=$(grep "peak CoM" "$log" | grep -oE "[0-9.]+" | head -1 || echo NA)
        dcm=$(grep "peak DCM" "$log" | grep -oE "[0-9.]+" | head -1 || echo NA)
        minz=$(grep "min base z" "$log" | grep -oE "min base z = [0-9.]+" | grep -oE "[0-9.]+" | head -1 || echo NA)
        res=$(grep -oE "\[pushRecovery\] (PASS|FAIL|NOTE)" "$log" | grep -oE "PASS|FAIL|NOTE" | head -1 || echo NA)
        echo "${DNAME},${FF},${T},${MAG},${FX},${FY},${r},${com},${dcm},${minz},${res}" >> "$OUT"
        echo "[dir] ${DNAME} ${MAG}N ff${FF} run ${r}/${M}: CoM=${com} minz=${minz} ${res}"
      done
    done
  done
done
echo "[dir] DONE -> $OUT"
