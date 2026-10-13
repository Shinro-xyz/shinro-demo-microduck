"""Draw how the Cake resident handles a live ``replace-slot`` swap.

Reads the run artifacts written by ``cake/run-microduck.sh`` — ``swaps.json`` (the
schedule the runner drove and each swap's outcome) and ``trace.csv`` (the path the
walk actually traced) — and writes one figure with two panels:

* **left** — the trunk path, coloured by the trajectory module driving it, with a
  marker at each swap; and
* **right** — the update transaction Cake ran for each swap, phase by phase, with
  the module generations above it.

The right panel's phases are the resident's own event rows (``docs/event-code-
registry.md``): the affected closure is computed, the retiring generation drains,
the consumer's declared ``live_snapshot`` row is applied, the new binding snapshot
is published, and the retired generation is reclaimed. The point the figure makes
is the one the demo asserts: the policy module runs across the whole axis, the
resident's session identity does not change, and only the trajectory slot's
generation moves.

Usage::

    python scripts/plot_cake_swap.py [--run-dir build/cake/run] [--out PATH]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402  (use() must run before pyplot imports)
import numpy as np

#: The transaction phases, in Cake's own event-row order (docs/event-code-registry.md).
PHASES = (
    "closure\ncomputed",
    "drain retiring\ngeneration",
    "apply live_snapshot\nrow (rebind)",
    "publish binding\nsnapshot",
    "reclaim retired\ngeneration",
)

#: One colour per trajectory, shared by both panels.
COLORS = {
    "circle": "tab:orange",
    "eight": "tab:blue",
    "slalom": "tab:green",
    "forward": "tab:purple",
    "idle": "tab:gray",
}


def _load(run_dir: Path) -> tuple[dict, list[tuple[float, float, float]]]:
    """The swaps record and the trunk path (``t, x, y``)."""
    swaps_path = run_dir / "swaps.json"
    trace_path = run_dir / "trace.csv"
    try:
        record = json.loads(swaps_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"cannot read {swaps_path}: {exc}") from exc
    rows: list[tuple[float, float, float]] = []
    try:
        with open(trace_path) as fh:
            next(fh)  # header
            for line in fh:
                t, x, y, _z = (float(v) for v in line.split(","))
                rows.append((t, x, y))
    except (OSError, ValueError) as exc:
        raise SystemExit(f"cannot read {trace_path}: {exc}") from exc
    return record, rows


def plot(record: dict, rows: list[tuple[float, float, float]], out: Path) -> Path:
    schedule: list[list] = record["schedule"]
    swaps: list[dict] = record["swaps"]
    duration = float(record["duration_s"])

    fig = plt.figure(figsize=(12.4, 5.2), dpi=110)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.35], wspace=0.24)

    # --- left: the path, coloured by the module driving it ------------------
    ax = fig.add_subplot(gs[0, 0])
    times = np.array([r[0] for r in rows])
    xy = np.array([r[1:] for r in rows])
    starts = [float(s[1]) for s in schedule]
    bounds = starts + [duration + 1.0]
    for i, (name, _start) in enumerate(schedule):
        mask = (times >= bounds[i]) & (times < bounds[i + 1])
        if mask.any():
            ax.plot(xy[mask, 0], xy[mask, 1], "-", lw=2.4, color=COLORS.get(name, "tab:cyan"), label=name)
    for i, swap in enumerate(swaps, start=1):
        at = float(swap["at_s"])
        j = int(np.searchsorted(times, at))
        if 0 <= j < len(xy):
            ax.plot([xy[j, 0]], [xy[j, 1]], "D", color="black", ms=7, zorder=6)
            ax.annotate(
                f"swap {i}\n-> {swap['to']}",
                (xy[j, 0], xy[j, 1]),
                textcoords="offset points",
                xytext=(10, 8),
                fontsize=8,
            )
    ax.plot([xy[0, 0]], [xy[0, 1]], "ks", ms=7, label="start")
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("trunk path, coloured by trajectory module", fontsize=11)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8, framealpha=0.9)

    # --- right: how Cake ran each swap --------------------------------------
    ax = fig.add_subplot(gs[0, 1])
    traj_y = 2.5  # the swappable slot: one generation per segment
    policy_y = 1.35  # the consumer runs the whole time

    ax.broken_barh([(0.0, duration)], (policy_y, 0.5), facecolors="0.82", edgecolor="black", lw=0.8)
    ax.text(duration / 2.0, policy_y + 0.25, "policy module (microduck) — runs throughout", ha="center", va="center", fontsize=9)

    for i, (name, start) in enumerate(schedule):
        end = bounds[i + 1] if i + 1 < len(bounds) - 1 else duration
        end = min(end, duration)
        ax.broken_barh(
            [(start, end - start)], (traj_y, 0.5), facecolors=COLORS.get(name, "tab:cyan"), edgecolor="black", lw=0.8, alpha=0.85
        )
        ax.text((start + end) / 2.0, traj_y + 0.25, name, ha="center", va="center", fontsize=9, fontweight="bold")

    for i, swap in enumerate(swaps, start=1):
        at = float(swap["at_s"])
        ax.axvline(at, color="black", lw=1.3, ls="--", alpha=0.7)
        ax.text(
            at,
            3.2,
            f"swap {i}: replace-slot → {swap['to']}\ntxn {swap['phase']}\nsession unchanged",
            ha="center",
            va="bottom",
            fontsize=8,
            bbox={"boxstyle": "round", "fc": "white", "ec": "0.6", "alpha": 0.95},
        )

    # The transaction phase flow, drawn once: it is the same for every swap.
    xs = np.linspace(0.07, 0.93, len(PHASES))
    for k, (x, label) in enumerate(zip(xs, PHASES)):
        ax.text(
            x,
            0.5,
            label,
            ha="center",
            va="center",
            fontsize=7.5,
            bbox={"boxstyle": "round,pad=0.4", "fc": "0.93", "ec": "0.5"},
            transform=ax.transAxes,
        )
        if k < len(PHASES) - 1:
            ax.annotate(
                "",
                xy=(xs[k + 1] - 0.08, 0.5),
                xytext=(x + 0.08, 0.5),
                xycoords="axes fraction",
                textcoords="axes fraction",
                arrowprops={"arrowstyle": "->", "color": "0.4", "lw": 1.1},
            )
    ax.text(
        0.5,
        0.12,
        "one Cake update transaction per swap: EVT_TXN_CLOSURE_COMPUTED → … → EVT_TXN_GENERATIONS_RECLAIMED",
        ha="center",
        va="center",
        fontsize=7.5,
        transform=ax.transAxes,
        color="0.3",
    )

    ax.set_xlim(0.0, duration)
    ax.set_ylim(0.0, 4.15)
    ax.set_yticks([])
    ax.set_xlabel("t [s]")
    ax.set_title(f"resident session {record.get('session', '?')[:12]}… — never restarted", fontsize=10)
    ax.grid(True, axis="x", alpha=0.25)

    fig.suptitle("Microduck on the Cake plane: swapping the trajectory module while the walk continues", fontsize=12)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-dir", default="build/cake/run", help="where swaps.json and trace.csv live")
    parser.add_argument("--out", default=None, help="output image (default: <run-dir>/microduck_cake_swap.png)")
    args = parser.parse_args(argv)

    run_dir = Path(args.run_dir)
    record, rows = _load(run_dir)
    out = Path(args.out) if args.out else run_dir / "microduck_cake_swap.png"
    print(f"wrote {plot(record, rows, out)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
