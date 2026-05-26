#!/usr/bin/env bash
# B2.0 oracle-FF edge sweep: does external-wrench feedforward raise the recovery (fall) threshold?
# Seed-averaged fall rate + peak CoM (over survivors) vs push magnitude, baseline (ff=0) vs oracle
# FF (ff=1, T=0.27 s -- the B2.0 knee). The balance-regime excursion benefit is real but small; the
# fall threshold is the robust, binary KPI where the feedforward's value should show. Runs that fall
# exit early (base z < 0.3), so fallen cells are quicker.
#
# Usage: experiments/b2_edge_sweep.sh [M=12]
set -uo pipefail
ROOT=/Users/josesilvaa/wb_humanoid_mpc
BIN="$ROOT/build_standalone/bin/pushRecovery"
OUT="$ROOT/experiments/results/b2_edge_sweep.csv"
M="${1:-12}"
MAGS=(70 80 90 100 110)
SPECS=("0 inf" "1 0.27" "2 0.27")   # baseline, oracle FF, observer FF (at the knee T=0.27)

cd "$ROOT/build_standalone"   # codegen cache path is CWD-relative
echo "cond,ff_mode,T,fx_N,run,peak_com_dev_m,peak_dcm_dev_m,min_base_z_m,result" > "$OUT"
for FX in "${MAGS[@]}"; do
  for spec in "${SPECS[@]}"; do
    set -- $spec; FF=$1; T=$2
    cond="ff${FF}_T${T}"
    for r in $(seq 1 "$M"); do
      log="/tmp/edge_${cond}_${FX}_${r}.log"
      "$BIN" 6 "$FX" 0 2.0 0.1 "" "" "$FF" "$T" > "$log" 2>&1 || true
      com=$(grep "peak CoM" "$log" | grep -oE "[0-9.]+" | head -1 || echo NA)
      dcm=$(grep "peak DCM" "$log" | grep -oE "[0-9.]+" | head -1 || echo NA)
      minz=$(grep "min base z" "$log" | grep -oE "min base z = [0-9.]+" | grep -oE "[0-9.]+" | head -1 || echo NA)
      res=$(grep -oE "\[pushRecovery\] (PASS|FAIL|NOTE)" "$log" | grep -oE "PASS|FAIL|NOTE" | head -1 || echo NA)
      echo "${cond},${FF},${T},${FX},${r},${com},${dcm},${minz},${res}" >> "$OUT"
      echo "[edge] ${FX}N ${cond} run ${r}/${M}: CoM=${com} minz=${minz} ${res}"
    done
  done
done
echo "[edge] DONE -> $OUT"
