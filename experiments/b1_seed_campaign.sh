#!/usr/bin/env bash
# B1 seed-averaging campaign: repeat the Full-vs-SRBD push-recovery ablation M times
# per (model, push) cell to put mean +/- std error bars on the peak CoM/DCM deviation.
#
# Run-to-run variance is REAL but not injected: the sim + scripted push are deterministic,
# but the MPC solves in its own async thread while the control loop runs wall-clock-bound,
# so the policy each control step sees depends on thread-timing jitter. Verified
# non-deterministic by probe (Full 50N x3 -> peak CoM 0.0236/0.0237/0.0242). That jitter
# is the "seed"; each run is an independent sample of the closed loop's natural spread.
#
# Each run is one pushRecovery process (it _Exit()s past the MRT controller's
# non-terminating solver thread, so one process per run). Output: one row per run to
# experiments/results/b1_seeds.csv. No per-step CSV here (scalars only).
#
# Usage: experiments/b1_seed_campaign.sh [M=8]
set -uo pipefail
ROOT=/Users/josesilvaa/wb_humanoid_mpc
BIN="$ROOT/build_standalone/bin/pushRecovery"
TASK_SRBD="$ROOT/robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task_srbd.info"
OUT="$ROOT/experiments/results/b1_seeds.csv"
M="${1:-8}"
PUSHES=(30 50 70)

cd "$ROOT/build_standalone"   # codegen cache path is CWD-relative
echo "model,push_N,run,peak_base_disp_m,peak_com_dev_m,peak_dcm_dev_m,min_base_z_m,result" > "$OUT"
for N in "${PUSHES[@]}"; do
  for MODEL in full srbd; do
    TASK=""; [ "$MODEL" = srbd ] && TASK="$TASK_SRBD"
    for r in $(seq 1 "$M"); do
      log="/tmp/seed_${MODEL}_${N}_${r}.log"
      "$BIN" 6 "$N" 0 2.0 0.1 "$TASK" "" > "$log" 2>&1 || true
      base=$(grep "peak base" "$log" | grep -oE "[0-9.]+" | head -1 || echo NA)
      com=$(grep "peak CoM" "$log" | grep -oE "[0-9.]+" | head -1 || echo NA)
      dcm=$(grep "peak DCM" "$log" | grep -oE "[0-9.]+" | head -1 || echo NA)
      minz=$(grep "min base z" "$log" | grep -oE "min base z = [0-9.]+" | grep -oE "[0-9.]+" | head -1 || echo NA)
      res=$(grep -oE "\[pushRecovery\] (PASS|FAIL|NOTE)" "$log" | grep -oE "PASS|FAIL|NOTE" | head -1 || echo NA)
      echo "${MODEL},${N},${r},${base},${com},${dcm},${minz},${res}" >> "$OUT"
      echo "[campaign] ${MODEL} ${N}N run ${r}/${M}: CoM=${com} DCM=${dcm} ${res}"
    done
  done
done
echo "[campaign] DONE -> $OUT"
