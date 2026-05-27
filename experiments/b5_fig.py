#!/usr/bin/env python3
"""B5 figure: observer feedforward + reactive stepping integrate without colliding.

Reads experiments/results/b5_step_sweep.csv (the seed campaign, gitignored) and writes
experiments/report/fig_b5_integration.pdf -- sagittal (+x) fall rate vs push magnitude for four arms
(baseline, feedforward only, stepping only, feedforward + stepping), with Wilson 95% CIs. Fall = min
base z < 0.5 m (the 'fell' column). Mirrors the B3 envelope figure style (no suptitle -- the LaTeX
caption is the description). Run after b5_step_sweep.sh.
"""
import csv
import math
import os
import sys
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "results", "b5_step_sweep.csv")
OUT = os.path.join(HERE, "report")
os.makedirs(OUT, exist_ok=True)


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

    # arm: (key, label, colour, marker, linestyle)
    arms = [
        ("off", "baseline (no ADR)", "#888888", "o", "--"),
        ("ff", "feedforward only", "#1f77b4", "s", "-"),
        ("step", "stepping only", "#d62728", "^", "-"),
        ("both", "feedforward + stepping", "#2ca02c", "D", "-"),
    ]
    fig, ax = plt.subplots(figsize=(6.2, 4.0))
    for arm, lab, col, mk, ls in arms:
        rate, lo, hi = [], [], []
        for m in mags:
            rr = g.get((m, arm), [])
            k = sum(int(x["fell"]) for x in rr)
            p, l, h = wilson(k, len(rr))
            rate.append(100 * p)
            lo.append(100 * (p - l))
            hi.append(100 * (h - p))
        ax.errorbar(mags, rate, yerr=[lo, hi], label=lab, color=col, marker=mk, ls=ls, capsize=3, lw=1.8, ms=6)
    ax.set_xlabel("push magnitude [N]")
    ax.set_ylabel("fall rate [%]")
    ax.set_ylim(-5, 105)
    ax.grid(alpha=0.3)
    ax.legend(frameon=False, fontsize=9)
    # No suptitle: the LaTeX caption is authoritative and must not carry internal block tags.
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_b5_integration.pdf"), bbox_inches="tight")
    print("wrote", os.path.join(OUT, "fig_b5_integration.pdf"))


if __name__ == "__main__":
    main()
