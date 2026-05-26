#!/usr/bin/env python3
"""B2.0 oracle-FF edge sweep statistics.

Reads experiments/results/b2_edge_sweep.csv (one row per run, M runs per (push, arm)) and
reports, per (push magnitude, arm):
  - fall rate + Wilson 95% confidence interval (a run "fell" if it did not PASS or min base z < 0.5),
  - peak CoM / DCM excursion and min base z as mean +/- std over the RECOVERED runs.
Writes results/b2_edge_summary.csv. Fall rate is the headline KPI; with M ~ 12 the per-cell CI is
wide, so the robust signal is the arm-to-arm separation at a given magnitude, not the absolute rate.
"""
import csv
import math
import os
import statistics
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
src = os.path.join(HERE, "results", "b2_edge_sweep.csv")

cells = defaultdict(list)
with open(src) as f:
    for r in csv.DictReader(f):
        cells[(int(r["fx_N"]), r["ff_mode"])].append(r)


def fell(r):
    # Fall = base z dropped below 0.5 m. This catches the harness's z<0.3 "FAIL" trigger AND
    # near-collapses that dipped below 0.5 without hitting 0.3 (which "result==FAIL" would miss and
    # "result!=PASS" would conflate with genuine NOTEs). A NOTE that stayed up (min_z>0.5 but left
    # the 0.12 m in-regime bound -- common for lateral pushes) is a SURVIVOR, not a fall.
    return float(r["min_base_z_m"]) < 0.5


def wilson(k, n, z=1.96):
    if n == 0:
        return (float("nan"), float("nan"), float("nan"))
    p = k / n
    d = 1.0 + z * z / n
    center = (p + z * z / (2 * n)) / d
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (p, max(0.0, center - half), min(1.0, center + half))


def ms(xs):
    if not xs:
        return (float("nan"), float("nan"))
    return (statistics.mean(xs), statistics.stdev(xs) if len(xs) > 1 else 0.0)


arms = [("0", "baseline"), ("1", "oracle FF"), ("2", "observer FF")]
mags = sorted({k[0] for k in cells})

hdr = (f"{'push_N':>7} {'arm':<11}{'n':>3}{'fall':>5}{'fall_rate (95% CI)':>22}   "
       f"{'CoM mean+/-std':>18}{'DCM mean+/-std':>18}{'minz mean+/-std':>18}")
print(hdr)
print("-" * len(hdr))

out = []
reliable = {a: None for _, a in arms}  # highest magnitude with zero falls, per arm
for fx in mags:
    for ff, lab in arms:
        rs = cells.get((fx, ff), [])
        if not rs:
            continue
        n = len(rs)
        k = sum(fell(r) for r in rs)
        p, lo, hi = wilson(k, n)
        surv = [r for r in rs if not fell(r)]
        com_m, com_s = ms([float(r["peak_com_dev_m"]) for r in surv])
        dcm_m, dcm_s = ms([float(r["peak_dcm_dev_m"]) for r in surv])
        mz_m, mz_s = ms([float(r["min_base_z_m"]) for r in surv])
        if k == 0:
            reliable[lab] = fx
        print(f"{fx:>7} {lab:<11}{n:>3}{k:>5}{f'{p:.2f} [{lo:.2f},{hi:.2f}]':>22}   "
              f"{f'{com_m:.4f}+/-{com_s:.4f}':>18}{f'{dcm_m:.4f}+/-{dcm_s:.4f}':>18}"
              f"{f'{mz_m:.3f}+/-{mz_s:.3f}':>18}")
        out.append(dict(push_N=fx, arm=lab, n=n, n_fall=k, fall_rate=round(p, 3),
                        fall_lo=round(lo, 3), fall_hi=round(hi, 3), n_surv=len(surv),
                        com_mean=round(com_m, 5), com_std=round(com_s, 5),
                        dcm_mean=round(dcm_m, 5), dcm_std=round(dcm_s, 5),
                        minz_mean=round(mz_m, 4), minz_std=round(mz_s, 4)))
    print()

print("Reliable in-place threshold (highest magnitude with 0/N falls):")
for _, a in arms:
    print(f"  {a:<11}: {reliable[a]} N" if reliable[a] is not None else f"  {a:<11}: <lowest tested")

dst = os.path.join(HERE, "results", "b2_edge_summary.csv")
with open(dst, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(out[0].keys()))
    w.writeheader()
    w.writerows(out)
print("\nwrote", dst)
