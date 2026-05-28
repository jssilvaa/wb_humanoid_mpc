#!/usr/bin/env python3
"""B5 stats: observer feedforward + reactive stepping integration, fall rate vs push magnitude.

Reads experiments/results/b5_step_sweep.csv and prints, per magnitude, the fall rate (min base z <
0.5 m -- the 'fell' column) with a Wilson 95% CI for each arm. Arms:
  off    baseline (no ADR)
  ff     observer feedforward only (B2)
  step   reactive stepping only (B3)
  both   feedforward + stepping, FF on throughout (keep-on)
  gate   feedforward + stepping, FF off whenever the stepper is active (full gate)
  hybrid feedforward + stepping, FF on through the first step's swing then gated off
Reports the reliable (0-fall) threshold per arm, checks whether each integration arm (both/gate/
hybrid) is no worse than min(ff, step) everywhere, and breaks out the ceiling (180/200 N) and
mid-band (90/120 N) fall rates -- the synergy vs collision trade-off. Same methodology as B1/B2/B3.
"""
import csv
import math
import os
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "results", "b5_step_sweep.csv")
ARMS = ("off", "ff", "step", "both", "gate", "hybrid", "magff")
INTEG = ("both", "gate", "hybrid", "magff")  # the integration arms compared against the individual ones


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
        g[(int(r["mag"]), r["arm"])].append(r)
    mags = sorted({int(r["mag"]) for r in rows})

    print("=== SAGITTAL (+x) === (fall = min base z < 0.5 m; M seeds/cell; Wilson 95% CI)")
    hdr = f"{'mag':>4} |" + "".join(f" {a+' fall%':>15} |" for a in ARMS)
    print(hdr)
    print("-" * len(hdr))
    thr = {a: None for a in ARMS}
    dom = {a: True for a in INTEG}      # arm <= min(ff, step) at every mag
    strict = {a: False for a in INTEG}  # arm < min(ff, step) somewhere
    rates = {a: {} for a in ARMS}
    for mag in mags:
        cell = {}
        for a in ARMS:
            rr = g.get((mag, a), [])
            k = sum(int(x["fell"]) for x in rr)
            n = len(rr)
            p, lo, hi = wilson(k, n)
            cell[a] = (k, n, p, lo, hi)
            rates[a][mag] = p
            if n and k == 0:
                thr[a] = mag
        line = f"{mag:>4} |"
        for a in ARMS:
            k, n, p, lo, hi = cell[a]
            line += f" {k:>2}/{n:<2}{100*p:>3.0f}[{100*lo:>3.0f},{100*hi:>3.0f}] |"
        print(line)
        best_indiv = min(cell["ff"][2], cell["step"][2])
        for a in INTEG:
            if cell[a][2] > best_indiv + 1e-9:
                dom[a] = False
            if cell[a][2] < best_indiv - 1e-9:
                strict[a] = True

    print("\n  reliable (0-fall) thresholds:  " + "   ".join(f"{a} {thr[a]} N" for a in ARMS))
    for a in INTEG:
        v = ("dominates min(ff,step) everywhere AND strictly better somewhere" if dom[a] and strict[a]
             else "dominates min(ff,step) (never strictly better)" if dom[a]
             else "WORSE than min(ff,step) at some magnitude")
        print(f"  arm '{a}' vs individual arms: {v}")

    def pct(a, m):
        return f"{100*rates[a].get(m, float('nan')):.0f}%"
    print("  ceiling fall% (FF should help the step survive past its own envelope):")
    for m in (180, 200):
        if m in mags:
            print(f"    {m} N:  step {pct('step',m):>4}  both {pct('both',m):>4}  gate {pct('gate',m):>4}  hybrid {pct('hybrid',m):>4}  magff {pct('magff',m):>4}")
    print("  mid-band fall% (FF should not re-trigger extra steps where step alone is clean):")
    for m in (90, 120):
        if m in mags:
            print(f"    {m} N:  step {pct('step',m):>4}  both {pct('both',m):>4}  gate {pct('gate',m):>4}  hybrid {pct('hybrid',m):>4}  magff {pct('magff',m):>4}")


if __name__ == "__main__":
    main()
