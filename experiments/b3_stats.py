#!/usr/bin/env python3
"""B3 rung-4 stats: reactive stepping (auto) vs baseline (off) fall rate vs push magnitude.

Reads experiments/results/b3_step_sweep.csv and prints, per direction/magnitude, the fall rate
(min base z < 0.5 m -- already in the 'fell' column) with a Wilson 95% CI for each arm, plus the
median step count of the auto arm. Reports the reliable (0-fall) threshold per arm. Mirrors the
B2 stats methodology (seed-averaged, binomial proportion with Wilson interval).
"""
import csv
import math
import os
import statistics
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "results", "b3_step_sweep.csv")


def wilson(k, n):
    if n == 0:
        return float("nan"), 0.0, 0.0
    p = k / n
    z = 1.96
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return p, max(0.0, c - h), min(1.0, c + h)


def main():
    rows = list(csv.DictReader(open(CSV)))
    g = defaultdict(list)
    for r in rows:
        g[(r["dir"], int(r["mag"]), r["arm"])].append(r)

    for dname, label in [("sag", "SAGITTAL (+x)"), ("lat", "LATERAL (+y)")]:
        mags = sorted({int(r["mag"]) for r in rows if r["dir"] == dname})
        print(f"\n=== {label} === (fall = min base z < 0.5 m; M seeds/cell; Wilson 95% CI)")
        print(f"{'mag':>5} | {'baseline fall%':>22} | {'stepping fall%':>22} | {'auto steps (med)':>16}")
        print("-" * 76)
        thr = {"off": None, "auto": None}
        for mag in mags:
            cells = {}
            for arm in ("off", "auto"):
                rr = g.get((dname, mag, arm), [])
                k = sum(int(x["fell"]) for x in rr)
                n = len(rr)
                p, lo, hi = wilson(k, n)
                cells[arm] = (k, n, p, lo, hi)
                if n and k == 0:
                    thr[arm] = mag  # highest 0-fall magnitude
            steps = [int(x["steps"]) for x in g.get((dname, mag, "auto"), [])]
            smed = statistics.median(steps) if steps else 0
            ko, no, po, loo, hio = cells["off"]
            ka, na, pa, loa, hia = cells["auto"]
            print(f"{mag:>5} | {ko:>2}/{no:<2} {100*po:>4.0f}% [{100*loo:>3.0f},{100*hio:>3.0f}]    "
                  f"| {ka:>2}/{na:<2} {100*pa:>4.0f}% [{100*loa:>3.0f},{100*hia:>3.0f}]    | {smed:>10.0f}")
        print(f"  reliable (0-fall) threshold:  baseline {thr['off']} N   ->   stepping {thr['auto']} N")


if __name__ == "__main__":
    main()
