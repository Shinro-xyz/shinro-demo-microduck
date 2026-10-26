"""Draw how the Cake resident handles a live policy ``replace-slot``.

The trajectory-swap counterpart is ``plot_cake_swap.py``. This one reads the
artifacts ``bin/run-policy-swap.sh`` writes — ``policy_swap.json`` (the swap the
runner drove and its outcome) and ``trace.csv`` (the path the walk traced) — and
writes one figure with two panels:

* **left** — the trunk path coloured by the **policy** driving it (the standing
  network before the swap, the walking one after), with a marker at the swap, and
  under it the trunk speed, which is the plainest reading of the change: near
  zero while the duck balances in place, ~0.2 m/s once it walks; and
* **right** — the running graph. The stable ``bridge`` (which owns the sim socket
  and the clock) runs across the whole axis; the ``policy`` **leaf** it consumes
  is the slot whose generation moves; the trajectory does not move. The update
  transaction Cake ran is drawn below it, the same phases as the trajectory swap,
  because it is the same transaction.

The point the figure makes is the one the demo asserts: swapping the policy moves
only the policy leaf's generation, the bridge keeps running, and the resident's
session identity does not change.

Usage::

    python scripts/plot_cake_policy_swap.py [--run-dir build/cake/run-policy-swap] [--out PATH]
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

#: One colour per policy. The keys are the demo's `--from`/`--to` names.
COLORS = {"stand": "tab:blue", "walk": "tab:orange", "walk_velstand": "tab:green"}
LABELS = {"stand": "standing network", "walk": "walking network",
          "walk_velstand": "walking network (velstand)"}


def _load(run_dir: Path) -> tuple[dict, list[tuple[float, float, float, float]]]:
    """The swap record and the trunk path with its cross-track error."""
    swaps_path = run_dir / "policy_swap.json"
    trace_path = run_dir / "trace.csv"
    try:
        record = json.loads(swaps_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"cannot read {swaps_path}: {exc}") from exc
    rows: list[tuple[float, float, float, float]] = []
    try:
        with open(trace_path) as fh:
            next(fh)  # header
            for line in fh:
                parts = [float(v) for v in line.split(",")]
                cte = parts[4] if len(parts) > 4 else float("nan")
                rows.append((parts[0], parts[1], parts[2], cte))
    except (OSError, ValueError) as exc:
        raise SystemExit(f"cannot read {trace_path}: {exc}") from exc
    return record, rows


def plot(record: dict, rows: list[tuple[float, float, float, float]], out: Path) -> Path:
    frm = record["from"]
    to = record["to"]
    at = float(record["at_s"])
    duration = float(record["duration_s"])

    times = np.array([r[0] for r in rows])
    xy = np.array([r[1:3] for r in rows])
    # Trunk speed from successive samples.
    dt = np.diff(times)
    dt[dt <= 0] = np.nan
    speed = np.concatenate([[np.nan], np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])) / dt])

    fig = plt.figure(figsize=(12.4, 6.0), dpi=110)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.35], wspace=0.24)
    left = gs[0, 0].subgridspec(2, 1, height_ratios=[3.0, 1.15], hspace=0.34)

    # --- left-top: the path, coloured by the policy driving it --------------
    ax = fig.add_subplot(left[0, 0])
    for name, mask in ((frm, times < at), (to, times >= at)):
        if mask.any():
            ax.plot(xy[mask, 0], xy[mask, 1], "-", lw=2.4, color=COLORS.get(name, "tab:cyan"), label=f"{LABELS.get(name, name)} ({name})")
    j = int(np.searchsorted(times, at))
    if 0 <= j < len(xy):
        ax.plot([xy[j, 0]], [xy[j, 1]], "D", color="black", ms=7, zorder=6)
        ax.annotate(
            f"replace-slot\n{frm} -> {to}", (xy[j, 0], xy[j, 1]), textcoords="offset points", xytext=(12, 10), ha="left", fontsize=8
        )
    ax.plot([xy[0, 0]], [xy[0, 1]], "ks", ms=7, label="start")
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("trunk path, coloured by policy", fontsize=11)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8, framealpha=0.9)

    # --- left-bottom: the speed the policy produced -------------------------
    axe = fig.add_subplot(left[1, 0])
    for name, mask in ((frm, times < at), (to, times >= at)):
        if mask.any():
            axe.plot(times[mask], speed[mask], "-", lw=1.8, color=COLORS.get(name, "tab:cyan"))
    axe.axvline(at, color="black", lw=1.2, ls="--", alpha=0.7)
    axe.set_xlim(0.0, duration)
    axe.set_ylim(0.0, max(0.05, float(np.nanmax(speed)) * 1.25 if np.any(np.isfinite(speed)) else 0.05))
    axe.set_xlabel("t [s]")
    axe.set_ylabel("trunk speed [m/s]")
    axe.set_title("speed, by policy (dashed = replace-slot)", fontsize=10)
    axe.grid(True, alpha=0.3)

    # --- right: how Cake ran the swap ---------------------------------------
    ax = fig.add_subplot(gs[0, 1])
    policy_y = 3.6  # the swappable slot: one generation per policy
    bridge_y = 2.8  # the stable clock + socket owner
    traj_y = 2.0  # unchanged

    ax.broken_barh([(0.0, duration)], (bridge_y, 0.5), facecolors="0.82", edgecolor="black", lw=0.8)
    ax.text(
        duration / 2.0,
        bridge_y + 0.25,
        "bridge (owns sim socket + 50 Hz clock) — runs throughout",
        ha="center",
        va="center",
        fontsize=9,
        bbox={"boxstyle": "round,pad=0.15", "fc": "white", "ec": "none", "alpha": 0.85},
    )

    ax.broken_barh([(0.0, at)], (policy_y, 0.5), facecolors=COLORS[frm], edgecolor="black", lw=0.8, alpha=0.85)
    ax.broken_barh([(at, duration - at)], (policy_y, 0.5), facecolors=COLORS[to], edgecolor="black", lw=0.8, alpha=0.85)
    ax.text(at / 2.0, policy_y + 0.25, frm, ha="center", va="center", fontsize=9, fontweight="bold")
    ax.text((at + duration) / 2.0, policy_y + 0.25, to, ha="center", va="center", fontsize=9, fontweight="bold")

    ax.broken_barh([(0.0, duration)], (traj_y, 0.5), facecolors="tab:gray", edgecolor="black", lw=0.8, alpha=0.55)
    ax.text(
        duration / 2.0,
        traj_y + 0.25,
        "trajectory (circle) — unchanged",
        ha="center",
        va="center",
        fontsize=9,
        bbox={"boxstyle": "round,pad=0.15", "fc": "white", "ec": "none", "alpha": 0.85},
    )

    # The marker spans the bars only, so it does not cut the phase row below.
    ax.plot([at, at], [traj_y - 0.1, 4.35], color="black", lw=1.3, ls="--", alpha=0.7, zorder=1)
    ax.text(
        at,
        4.3,
        f"replace-slot policy: {frm} → {to}\ntxn {record['phase']}\nsession unchanged",
        ha="center",
        va="bottom",
        fontsize=8,
        bbox={"boxstyle": "round", "fc": "white", "ec": "0.6", "alpha": 0.95},
    )

    # The transaction phase flow, drawn once: the same transaction as a
    # trajectory swap (one live binding change under a running consumer).
    xs = np.linspace(0.07, 0.93, len(PHASES))
    for k, (x, label) in enumerate(zip(xs, PHASES)):
        ax.text(
            x,
            0.30,
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
                xy=(xs[k + 1] - 0.08, 0.30),
                xytext=(x + 0.08, 0.30),
                xycoords="axes fraction",
                textcoords="axes fraction",
                arrowprops={"arrowstyle": "->", "color": "0.4", "lw": 1.1},
            )
    ax.text(
        0.5,
        0.10,
        "one Cake update transaction: EVT_TXN_CLOSURE_COMPUTED → … → EVT_TXN_GENERATIONS_RECLAIMED",
        ha="center",
        va="center",
        fontsize=7.5,
        transform=ax.transAxes,
        color="0.3",
    )

    ax.set_xlim(0.0, duration)
    ax.set_ylim(0.0, 4.8)
    ax.set_yticks([])
    ax.set_xlabel("t [s]")
    ax.set_title(f"resident session {record.get('session', '?')[:12]}… — never restarted", fontsize=10)
    ax.grid(True, axis="x", alpha=0.25)

    fig.suptitle("Microduck on the Cake plane: swapping the policy leaf while the walk continues", fontsize=12)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-dir", default="state/run-policy-swap", help="where policy_swap.json and trace.csv live")
    parser.add_argument("--out", default=None, help="output image (default: <run-dir>/microduck_cake_policy_swap.png)")
    args = parser.parse_args(argv)

    run_dir = Path(args.run_dir)
    record, rows = _load(run_dir)
    out = Path(args.out) if args.out else run_dir / "microduck_cake_policy_swap.png"
    print(f"wrote {plot(record, rows, out)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
