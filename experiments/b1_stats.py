#!/usr/bin/env python3
"""B1 seed-campaign statistics.

Reads experiments/results/b1_seeds.csv (one row per run, M runs per (model, push) cell)
and reports, per cell: the fall rate and the peak CoM/DCM deviation over the RECOVERED
runs. A run "fell" if it did not PASS or its min base z dropped below 0.5 m.

MEDIAN is the headline statistic: it is robust to (a) the cold-start artifact on the
campaign's very first run (cold OS/dylib cache -> the MPC loop ran slow, inflating that
one transient) and (b) the occasional high-side recovery near the envelope edge. Mean and
sample std are printed alongside; where they diverge from the median the distribution is
skewed by one of those outliers. Writes results/b1_seed_summary.csv.
"""
import csv
import os
import statistics

HERE = os.path.dirname(os.path.abspath(__file__))
src = os.path.join(HERE, "results", "b1_seeds.csv")

cells = {}
with open(src) as f:
    for row in csv.DictReader(f):
        cells.setdefault((row["model"], int(row["push_N"])), []).append(row)


def fell(r):
    return r["result"] != "PASS" or float(r["min_base_z_m"]) < 0.5


def stats(xs):
    xs = sorted(xs)
    return (statistics.median(xs),
            statistics.mean(xs),
            statistics.stdev(xs) if len(xs) > 1 else 0.0,
            min(xs), max(xs))


hdr = (f"{'cell':<10}{'n':>3}{'fall':>5}   "
       f"{'CoM med':>9}{'mean':>9}{'std':>9}{'max':>9}   "
       f"{'DCM med':>9}{'mean':>9}{'std':>9}")
print(hdr)
print("-" * len(hdr))

out = []
for N in (30, 50, 70):
    for model in ("full", "srbd"):
        rs = cells[(model, N)]
        falls = sum(fell(r) for r in rs)
        rec = [r for r in rs if not fell(r)]
        cm, cmean, cstd, _, cmax = stats([float(r["peak_com_dev_m"]) for r in rec])
        dm, dmean, dstd, _, _ = stats([float(r["peak_dcm_dev_m"]) for r in rec])
        print(f"{model + '-' + str(N):<10}{len(rs):>3}{falls:>5}   "
              f"{cm:>9.4f}{cmean:>9.4f}{cstd:>9.4f}{cmax:>9.4f}   "
              f"{dm:>9.4f}{dmean:>9.4f}{dstd:>9.4f}")
        out.append(dict(model=model, push_N=N, n=len(rs), n_fall=falls,
                        com_median=round(cm, 5), com_mean=round(cmean, 5), com_std=round(cstd, 5), com_max=round(cmax, 5),
                        dcm_median=round(dm, 5), dcm_mean=round(dmean, 5), dcm_std=round(dstd, 5)))

dst = os.path.join(HERE, "results", "b1_seed_summary.csv")
with open(dst, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(out[0].keys()))
    w.writeheader()
    w.writerows(out)
print("\nwrote", dst)
