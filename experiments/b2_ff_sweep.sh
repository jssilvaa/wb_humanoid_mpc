#!/usr/bin/env bash
# B2.0 oracle external-wrench feedforward sweep: seed-average peak CoM/DCM deviation, fall rate,
# and min base z vs the horizon decay time T, at a fixed impulse. Compares baseline (ff=0),
# ZOH (ff=1, T=inf) and finite-T decay (ff=1). Run-to-run variance is the async MPC-thread /
# control-loop timing jitter (see b1_seed_campaign.sh) -- single runs are noise-dominated here,
# so each (condition) cell is repeated M times and compared by median.
#
# Usage: experiments/b2_ff_sweep.sh [fx_N=50] [M=8]
set -uo pipefail
ROOT=/Users/josesilvaa/wb_humanoid_mpc
BIN="$ROOT/build_standalone/bin/pushRecovery"
OUT="$ROOT/experiments/results/b2_ff_sweep.csv"
FX="${1:-50}"
M="${2:-8}"
SPECS=("0 inf" "1 inf" "1 0.27" "1 0.05")   # "ff_mode T"; T=inf is ZOH, ff_mode 0 ignores T

cd "$ROOT/build_standalone"   # codegen cache path is CWD-relative
echo "cond,ff_mode,T,fx_N,run,peak_com_dev_m,peak_dcm_dev_m,min_base_z_m,result" > "$OUT"
for spec in "${SPECS[@]}"; do
  set -- $spec; FF=$1; T=$2
  cond="ff${FF}_T${T}"
  for r in $(seq 1 "$M"); do
    log="/tmp/b2_${cond}_${r}.log"
    "$BIN" 6 "$FX" 0 2.0 0.1 "" "" "$FF" "$T" > "$log" 2>&1 || true
    com=$(grep "peak CoM" "$log" | grep -oE "[0-9.]+" | head -1 || echo NA)
    dcm=$(grep "peak DCM" "$log" | grep -oE "[0-9.]+" | head -1 || echo NA)
    minz=$(grep "min base z" "$log" | grep -oE "min base z = [0-9.]+" | grep -oE "[0-9.]+" | head -1 || echo NA)
    res=$(grep -oE "\[pushRecovery\] (PASS|FAIL|NOTE)" "$log" | grep -oE "PASS|FAIL|NOTE" | head -1 || echo NA)
    echo "${cond},${FF},${T},${FX},${r},${com},${dcm},${minz},${res}" >> "$OUT"
    echo "[b2sweep] ${cond} run ${r}/${M}: CoM=${com} minz=${minz} ${res}"
  done
done
echo "[b2sweep] DONE -> $OUT"
