#!/usr/bin/env python3
"""B5 stats: observer feedforward + reactive stepping integration, fall rate vs push magnitude.

Reads experiments/results/b5_step_sweep.csv (arm in {off, ff, step, both}, sagittal +x) and prints,
per magnitude, the fall rate (min base z < 0.5 m -- the 'fell' column) with a Wilson 95% CI for each
arm, plus the median step count and median first-step time for the stepping arms (the FF-delay
diagnostic: does the feedforward push the step later in 'both' than in 'step'?). Reports the reliable
(0-fall) threshold per arm and checks the success criterion -- is 'both' no worse than min(ff, step)
at every magnitude (and strictly better somewhere)? Same methodology as the B1/B2/B3 campaigns.
"""
import csv
import math
import os
import statistics
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "results", "b5_step_sweep.csv")
ARMS = ("off", "ff", "step", "both")


def wilson(k, n):
    if n == 0:
        return float("nan"), 0.0, 0.0
    p = k / n
    z = 1.96
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return p, max(0.0, c - h), min(1.0, c + h)


def med_first_step(rows):
    ts = [float(x["first_step_t"]) for x in rows if float(x["first_step_t"]) >= 0.0]
    return statistics.median(ts) if ts else float("nan")


def main():
    rows = list(csv.DictReader(open(CSV)))
    g = defaultdict(list)
    for r in rows:
        g[(int(r["mag"]), r["arm"])].append(r)
    mags = sorted({int(r["mag"]) for r in rows})

    print("=== SAGITTAL (+x) === (fall = min base z < 0.5 m; M seeds/cell; Wilson 95% CI)")
    hdr = f"{'mag':>4} |" + "".join(f" {a+' fall%':>16} |" for a in ARMS) + f" {'step t':>7} | {'both t':>7}"
    print(hdr)
    print("-" * len(hdr))
    thr = {a: None for a in ARMS}
    dominates = True   # both <= min(ff, step) at every mag
    strictly = False   # both < min(ff, step) somewhere
    for mag in mags:
        cell = {}
        for a in ARMS:
            rr = g.get((mag, a), [])
            k = sum(int(x["fell"]) for x in rr)
            n = len(rr)
            p, lo, hi = wilson(k, n)
            cell[a] = (k, n, p, lo, hi)
            if n and k == 0:
                thr[a] = mag  # highest 0-fall magnitude (monotone-ish; last wins)
        line = f"{mag:>4} |"
        for a in ARMS:
            k, n, p, lo, hi = cell[a]
            line += f" {k:>2}/{n:<2}{100*p:>4.0f}[{100*lo:>3.0f},{100*hi:>3.0f}] |"
        st = med_first_step(g.get((mag, "step"), []))
        bt = med_first_step(g.get((mag, "both"), []))
        line += f" {st:>7.3f} | {bt:>7.3f}"
        print(line)
        # success-criterion bookkeeping (compare fall rates)
        pf, ps, pb = cell["ff"][2], cell["step"][2], cell["both"][2]
        best_indiv = min(pf, ps)
        if pb > best_indiv + 1e-9:
            dominates = False
        if pb < best_indiv - 1e-9:
            strictly = True

    print("\n  reliable (0-fall) thresholds:  " + "   ".join(f"{a} {thr[a]} N" for a in ARMS))
    print(f"  step-time (median): does 'both' fire later than 'step'? (FF-delay diagnostic above)")
    verdict = ("BOTH dominates (<= min(ff,step) everywhere) and is strictly better somewhere"
               if dominates and strictly else
               "BOTH dominates (<= min) but never strictly better" if dominates else
               "BOTH is WORSE than min(ff,step) at some magnitude -- inspect (anti-phase regime)")
    print(f"  success criterion: {verdict}")


if __name__ == "__main__":
    main()
