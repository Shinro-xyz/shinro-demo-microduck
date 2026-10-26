#!/usr/bin/env python3
"""Every run, not the good one: the per-run numbers as a plot.

`policy-swap-velocity.png` shows one run each, which is the story but not the
evidence. This is the evidence: the same three groups, every run, so a reader can
see that the outcome does not depend on which run was picked.

Reads `docs/media/pollen/evidence/policy-swap-runs.csv` (written by hand from the
runs' own outputs — six mid-walk `robotctl policy load`s on Pollen's stack, five
Cake walk -> walk_velstand swaps, five Cake walk -> stand swaps) and writes
`docs/media/pollen/policy-swap-runs.png`.

    python scripts/plot_pollen_runs.py
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

POLLEN, CAKE, CAKE_REVERSE = "#C8871A", "#0A9D55", "#B4362B"
INK, MUTED, RULE = "#16211B", "#5A655E", "#E4E6E4"

HERE = Path(__file__).resolve().parent.parent
DEFAULT_CSV = HERE / "docs" / "media" / "pollen" / "evidence" / "policy-swap-runs.csv"
DEFAULT_OUT = HERE / "docs" / "media" / "pollen" / "policy-swap-runs.png"

GROUPS = [
    ("pollen", POLLEN, "Pollen\n(mid-walk load, n=6)"),
    ("cake_walk_to_walk_velstand", CAKE, "Cake\n(walk→walk, n=5)"),
    ("cake_walk_to_stand", CAKE_REVERSE, "Cake\n(walk→stand, n=5)"),
]


def _load(path: Path):
    rows = list(csv.DictReader(open(path)))
    return {key: [r for r in rows if r["group"] == key] for key, _, _ in GROUPS}


def plot(data, out: Path) -> Path:
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.4, 4.3),
                                   gridspec_kw={"width_ratios": [1.15, 1]})
    ticks, labels = [], []
    for i, (key, colour, label) in enumerate(GROUPS):
        xs = [i + j * 0.16 for j in range(len(data[key]))]
        tilts = [float(r["max_tilt_deg"]) for r in data[key]]
        lows = [float(r["lowest_trunk_cm"]) for r in data[key]]
        ax1.plot(xs, tilts, "o", ms=7, color=colour)
        ax2.plot(xs, lows, "o", ms=7, color=colour)
        ax1.plot([i - 0.1, xs[-1] + 0.1], [sum(tilts) / len(tilts)] * 2, color=colour,
                 lw=1.2, ls=(0, (3, 2)), alpha=0.8)
        ax2.plot([i - 0.1, xs[-1] + 0.1], [sum(lows) / len(lows)] * 2, color=colour,
                 lw=1.2, ls=(0, (3, 2)), alpha=0.8)
        ticks.append(i + 0.3)
        labels.append(label)

    ax1.axhline(90, color=MUTED, lw=0.8, ls=":")
    ax1.text(-0.32, 93, "horizontal", fontsize=8, color=MUTED)
    ax1.axhline(180, color=MUTED, lw=0.8, ls=":")
    ax1.text(-0.32, 168, "inverted", fontsize=8, color=MUTED)
    ax1.set_ylim(-5, 195)
    ax1.set_xticks(ticks)
    ax1.set_xticklabels(labels, fontsize=8.5, color=MUTED)
    ax1.set_ylabel("max trunk tilt in the run (deg)", fontsize=9, color=MUTED)
    ax1.set_title("every run, not the good one", loc="left", fontsize=11, color=INK, pad=10)

    ax2.axhline(0, color=MUTED, lw=0.8, ls=":")
    ax2.text(2.55, 1.5, "floor", fontsize=8, color=MUTED)
    ax2.set_ylim(-15, 15)
    ax2.set_xticks(ticks)
    ax2.set_xticklabels(labels, fontsize=8.5, color=MUTED)
    ax2.set_ylabel("lowest trunk height in the run (cm)", fontsize=9, color=MUTED)
    ax2.set_title("how far down it got", loc="left", fontsize=11, color=INK, pad=10)

    for ax in (ax1, ax2):
        ax.set_xlim(-0.35, 3.4)
        ax.grid(axis="y", color=RULE, lw=0.7)
        ax.set_axisbelow(True)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)

    fig.text(0.008, 0.012,
             "Same robot, same two networks (Pollen's alpha_walking and velstand), same 0.11 m/s / 0.11 rad/s command, same 50 Hz. Pollen: robotd 0.17.4 + duck-sim,\n"
             "`robotctl policy load` mid-walk. Cake: `make walk-swap`, the same pair as two capsules. Dashed lines are group means. All three groups report a COMPLETED or\n"
             "accepted transaction with the session identity unchanged — which is why the transaction is not the evidence; the posture is. Cake's walk→stand is the direction\n"
             "where the cross-fade cannot invent physical continuity, and it loses in 5 of 5. Below zero the free joint ends under the floor plane: that fall is violent, not\n"
             "a collapse. Numbers per run: evidence/policy-swap-runs.csv.",
             fontsize=7.7, color=MUTED, ha="left")
    fig.subplots_adjust(left=0.07, right=0.985, top=0.9, bottom=0.27, wspace=0.22)
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default=str(DEFAULT_CSV))
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    args = ap.parse_args(argv)
    out = plot(_load(Path(args.csv)), Path(args.out))
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
