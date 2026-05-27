#!/usr/bin/env python3
"""B3 rung-4 figure: reactive stepping extends the push-recovery envelope.

Reads experiments/results/b3_step_sweep.csv (the seed campaign, gitignored) and writes
experiments/report/fig_b3_envelope.pdf -- fall rate vs push magnitude, baseline (off) vs reactive
stepping (auto), in two panels (sagittal +x, lateral +y), with Wilson 95% CIs. Fall = min base z <
0.5 m (the 'fell' column). Mirrors the B2 directional figure style. Run after b3_step_sweep.sh.
"""
import csv
import math
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "results", "b3_step_sweep.csv")
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
        g[(r["dir"], int(r["mag"]), r["arm"])].append(r)

    fig, axes = plt.subplots(1, 2, figsize=(9.5, 3.6), sharey=True)
    arms = [("off", "baseline (no step)", "#444444", "o"), ("auto", "reactive stepping", "#d62728", "^")]
    for ax, (dname, title) in zip(axes, [("sag", "sagittal (+x)"), ("lat", "lateral (+y)")]):
        mags = sorted({int(r["mag"]) for r in rows if r["dir"] == dname})
        for arm, lab, col, mk in arms:
            rate, lo, hi = [], [], []
            for m in mags:
                rr = g.get((dname, m, arm), [])
                k = sum(int(x["fell"]) for x in rr)
                p, l, h = wilson(k, len(rr))
                rate.append(100 * p)
                lo.append(100 * (p - l))
                hi.append(100 * (h - p))
            ax.errorbar(mags, rate, yerr=[lo, hi], label=lab, color=col, marker=mk, capsize=3, lw=1.8, ms=6)
        ax.set_xlabel("push magnitude [N]")
        ax.set_title(title)
        ax.set_ylim(-5, 105)
        ax.grid(alpha=0.3)
        ax.legend(frameon=False, fontsize=9)
    axes[0].set_ylabel("fall rate [%]")
    fig.suptitle("B3: reactive stepping vs baseline (seed-averaged, Wilson 95% CI)", y=1.02)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_b3_envelope.pdf"), bbox_inches="tight")
    print("wrote", os.path.join(OUT, "fig_b3_envelope.pdf"))


if __name__ == "__main__":
    main()
