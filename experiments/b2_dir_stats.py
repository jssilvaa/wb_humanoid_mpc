#!/usr/bin/env python3
"""B2 directional sweep statistics.

Per (direction, magnitude, arm): fall rate (base z < 0.5 m) + Wilson 95% CI, and peak CoM/DCM/min_z
mean +/- std over survivors. Reads experiments/results/b2_dir_sweep.csv, writes b2_dir_summary.csv.
"""
import csv
import math
import os
import statistics
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
src = os.path.join(HERE, "results", "b2_dir_sweep.csv")

cells = defaultdict(list)
with open(src) as f:
    for r in csv.DictReader(f):
        cells[(r["dir"], int(r["mag_N"]), r["ff_mode"])].append(r)


def fell(r):
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


arms = [("0", "baseline"), ("2", "observer FF")]
dirs = sorted({k[0] for k in cells})
mags = sorted({k[1] for k in cells})

out = []
for dname in dirs:
    print(f"\n=== {dname} ===")
    print(f"{'mag_N':>6} {'arm':<12}{'n':>3}{'fall':>5}{'fall_rate (95% CI)':>22}   "
          f"{'CoM mean+/-std':>18}{'minz mean+/-std':>16}")
    for mag in mags:
        for ff, lab in arms:
            rs = cells.get((dname, mag, ff), [])
            if not rs:
                continue
            n = len(rs)
            k = sum(fell(r) for r in rs)
            p, lo, hi = wilson(k, n)
            surv = [r for r in rs if not fell(r)]
            cm, cs = ms([float(r["peak_com_dev_m"]) for r in surv])
            mz, mzs = ms([float(r["min_base_z_m"]) for r in surv])
            print(f"{mag:>6} {lab:<12}{n:>3}{k:>5}{f'{p:.2f} [{lo:.2f},{hi:.2f}]':>22}   "
                  f"{f'{cm:.4f}+/-{cs:.4f}':>18}{f'{mz:.3f}+/-{mzs:.3f}':>16}")
            out.append(dict(dir=dname, arm=lab, mag_N=mag, n=n, n_fall=k, fall_rate=round(p, 3),
                            fall_lo=round(lo, 3), fall_hi=round(hi, 3), n_surv=len(surv),
                            com_mean=round(cm, 5), com_std=round(cs, 5),
                            minz_mean=round(mz, 4), minz_std=round(mzs, 4)))
        print()

dst = os.path.join(HERE, "results", "b2_dir_summary.csv")
with open(dst, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(out[0].keys()))
    w.writeheader()
    w.writerows(out)
print("wrote", dst)
