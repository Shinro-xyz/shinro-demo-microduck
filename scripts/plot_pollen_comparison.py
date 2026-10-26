#!/usr/bin/env python3
"""Draw the Pollen-vs-Cake policy-swap comparison from the committed evidence.

The claim this figure makes is narrow and checkable: replacing the network that is
*driving* costs one stack the gait and the other nothing. Three panels, one time axis:

    1. distance from the pose each duck held at the request — did it get anywhere?
    2. trunk tilt — is it still standing?
    3. vertical speed — and if it moved downward, how hard, and what picked it up?

Both inputs are trunk pose from each stack's own runtime:

    evidence/pollen-trunk-state.csv        `robotctl monitor --json`, slimmed:
                                           odom.position, safety.gravity_z, fallen, mode
    evidence/cake-trace-walk-velstand.csv  `make policy-swap` -> state/run-policy-swap/
                                           trace.csv, plus the recorded swap time in
                                           cake-swap-walk-velstand.json

Neither stream carries a trunk velocity, so both are differentiated here — the same
treatment, but theirs is decimated to ~32 Hz against our 50 Hz, and a derivative
amplifies the difference. That is why panel 1 is distance, not speed.

    python scripts/plot_pollen_comparison.py
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

POLLEN, CAKE = "#C8871A", "#0A9D55"
INK, MUTED, RULE = "#16211B", "#5A655E", "#E4E6E4"

HERE = Path(__file__).resolve().parent.parent
EVIDENCE = HERE / "docs" / "media" / "pollen" / "evidence"
DEFAULT_OUT = HERE / "docs" / "media" / "pollen" / "policy-swap-velocity.png"

GRID_S = 0.05   # common resample step
WIN_S = 0.15    # centred-difference window for the derivative
THEIR_COMPLETE_S = 2.0  # their HOME_RAMP timer, measured at 1.98-2.02 s


def _resample(t, xyz, tilt, t0, lo, hi):
    """Pose and attitude on a fixed grid, as ``(seconds since t0, xyz, tilt_deg)``."""
    out, k = [], 0
    for i in range(int((hi - lo) / GRID_S) + 1):
        tq = t0 + lo + i * GRID_S
        while k + 1 < len(t) and t[k + 1] < tq:
            k += 1
        if k + 1 >= len(t):
            break
        f = (tq - t[k]) / (t[k + 1] - t[k]) if t[k + 1] > t[k] else 0.0
        out.append((tq - t0,
                    [xyz[k][j] + (xyz[k + 1][j] - xyz[k][j]) * f for j in range(3)],
                    tilt[k] + (tilt[k + 1] - tilt[k]) * f))
    return out


def _derive(pts, home):
    """Distance from ``home`` (cm), tilt (deg), vertical speed (m/s).

    ``home`` is the trunk position the duck was standing at before the swap was
    requested — the first sample of its own run, not the first sample of the
    plotted window, so the number means "how far it walked from where it stood".
    """
    n = int(WIN_S / GRID_S) // 2
    rows = []
    for i, (t, p, tilt) in enumerate(pts):
        lo, hi = max(0, i - n), min(len(pts) - 1, i + n)
        dt = pts[hi][0] - pts[lo][0]
        vz = (pts[hi][1][2] - pts[lo][1][2]) / dt if dt > 0 else 0.0
        rows.append((t, 100 * math.dist(home[:2], p[:2]), tilt, vz))
    return rows


def _pollen(path: Path):
    rows = list(csv.DictReader(open(path)))
    t = [float(r["t"]) for r in rows]
    xyz = [[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows]
    # Their runtime reports the gravity vector in the trunk frame; tilt is the angle
    # between that and straight down.
    tilt = [math.degrees(math.acos(max(-1.0, min(1.0, -float(r["gravity_z"]))))) for r in rows]
    t0 = next(float(r["t"]) for r in rows if r["mode"] == "homing")
    home = xyz[0]
    return _derive(_resample(t, xyz, tilt, t0, -2.0, 6.5), home)


def _cake(trace: Path, swap: Path):
    rows = list(csv.DictReader(open(trace)))
    t = [float(r["t"]) for r in rows]
    xyz = [[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows]
    tilt = [float(r["tilt_deg"]) for r in rows]
    t0 = float(json.load(open(swap))["at_s"])
    home = xyz[0]
    return _derive(_resample(t, xyz, tilt, t0, -2.0, 6.5), home)


def _marks(ax, top_text: bool = False, y: float = 0.0) -> None:
    ax.axvline(0, color=MUTED, lw=0.9, ls=(0, (4, 3)))
    ax.axvline(THEIR_COMPLETE_S, color=POLLEN, lw=0.9, ls=(0, (2, 3)))
    if top_text:
        ax.text(0.06, y, "swap requested", fontsize=8.5, color=MUTED)
        ax.text(THEIR_COMPLETE_S + 0.06, y, "theirs COMPLETED", fontsize=8.5, color=POLLEN)


def plot(theirs, ours, out: Path) -> Path:
    fig, (ax1, ax2, ax3) = plt.subplots(
        3, 1, figsize=(10.6, 8.0), sharex=True,
        gridspec_kw={"hspace": 0.2, "height_ratios": [1.15, 1.0, 0.8]})

    # ---- 1. did it get anywhere ----
    ax1.plot([p[0] for p in theirs], [p[1] for p in theirs], color=POLLEN, lw=2)
    ax1.plot([p[0] for p in ours], [p[1] for p in ours], color=CAKE, lw=2)
    _marks(ax1, top_text=True, y=113)
    ax1.annotate(f"Pollen: {theirs[-1][1]:.0f} cm", xy=(6.2, theirs[-1][1]), xytext=(4.3, 6),
                 fontsize=9, color=POLLEN, arrowprops=dict(arrowstyle="->", color=POLLEN, lw=0.9))
    ax1.annotate(f"Cake: {ours[-1][1]:.0f} cm, straight through the swap",
                 xy=(4.6, 0.84 * ours[-1][1]), xytext=(2.6, 104), fontsize=9, color=CAKE,
                 arrowprops=dict(arrowstyle="->", color=CAKE, lw=0.9))
    ax1.annotate("ours COMPLETED here", xy=(0.0, 30), xytext=(0.4, 30), fontsize=8.5, color=CAKE,
                 arrowprops=dict(arrowstyle="->", color=CAKE, lw=0.9))
    ax1.set_ylim(0, 118)
    ax1.set_ylabel("distance from home (cm)", fontsize=9, color=MUTED)
    ax1.set_title("how far each duck walked from home", loc="left", fontsize=11, color=INK, pad=10)

    # ---- 2. is it still standing ----
    ax2.plot([p[0] for p in theirs], [p[2] for p in theirs], color=POLLEN, lw=2)
    ax2.plot([p[0] for p in ours], [p[2] for p in ours], color=CAKE, lw=1.6)
    _marks(ax2)
    ax2.axhline(90, color=MUTED, lw=0.7, ls=":")
    ax2.text(6.35, 92, "horizontal", fontsize=8, color=MUTED, ha="right")
    ax2.annotate("79-81°, and on\nthe floor for the\nrest of the ramp", xy=(0.32, 30),
                 xytext=(0.85, 52), fontsize=9, color=POLLEN,
                 arrowprops=dict(arrowstyle="->", color=POLLEN, lw=0.9))
    ax2.annotate("upright again at +3.4 s,\nwalking at +2.9 s", xy=(3.4, 2), xytext=(3.9, 34),
                 fontsize=9, color=POLLEN, arrowprops=dict(arrowstyle="->", color=POLLEN, lw=0.9))
    ax2.text(-1.9, 13, "Cake: under 10° throughout", fontsize=9, color=CAKE)
    ax2.set_ylim(-4, 100)
    ax2.set_yticks([0, 20, 40, 60, 80])
    ax2.set_ylabel("trunk tilt\n(0° = upright)", fontsize=9, color=MUTED)

    # ---- 3. how it moved vertically ----
    ax3.axhline(0, color=MUTED, lw=0.9, ls=(0, (4, 3)))
    ax3.plot([p[0] for p in theirs], [p[3] for p in theirs], color=POLLEN, lw=1.6)
    ax3.plot([p[0] for p in ours], [p[3] for p in ours], color=CAKE, lw=1.4)
    _marks(ax3)
    ax3.annotate("0.34 m/s down: a fall,\nnot a stop", xy=(0.5, -0.28), xytext=(1.0, -0.41),
                 fontsize=9, color=POLLEN, arrowprops=dict(arrowstyle="->", color=POLLEN, lw=0.9))
    ax3.annotate("0.37 m/s up: the new\nnetwork picking it up", xy=(2.15, 0.34), xytext=(2.75, 0.28),
                 fontsize=9, color=POLLEN, arrowprops=dict(arrowstyle="->", color=POLLEN, lw=0.9))
    ax3.text(-1.9, -0.13, "Cake: the step rhythm, unchanged", fontsize=9, color=CAKE)
    ax3.set_ylim(-0.45, 0.45)
    ax3.set_ylabel("up/down speed\n(m/s, + is up)", fontsize=9, color=MUTED)
    ax3.set_xlabel("seconds relative to the load request", fontsize=9, color=MUTED)

    for ax in (ax1, ax2, ax3):
        ax.grid(axis="y", color=RULE, lw=0.7)
        ax.set_axisbelow(True)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)

    fig.text(0.008, 0.008,
             "Same robot, same two networks, same 0.11 m/s command. Both traces come from each stack's own trunk pose — `robotctl monitor --json`\n"
             "(odom.position and safety.gravity, decimated to ~32 Hz) for Pollen, trace.csv at 50 Hz for Cake — resampled to 50 ms; tilt is the angle\n"
             "from upright, and the vertical speed is a 150 ms derivative of the same positions. Panel 1 is straight-line distance from home — the trunk\n"
             "position it was standing at when the run began — so a plateau means one thing: it stopped getting anywhere. Both swaps complete, and both\n"
             "sessions are unchanged, which is why the posture, not the transaction, is the evidence. The plateau ends at +2.1 s because `velstand`\n"
             "stood the duck up, not because the swap did.",
             fontsize=7.7, color=MUTED, ha="left")
    fig.subplots_adjust(left=0.105, right=0.985, top=0.945, bottom=0.175)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--evidence", default=str(EVIDENCE))
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    args = ap.parse_args(argv)
    ev = Path(args.evidence)
    theirs = _pollen(ev / "pollen-trunk-state.csv")
    ours = _cake(ev / "cake-trace-walk-velstand.csv", ev / "cake-swap-walk-velstand.json")
    out = plot(theirs, ours, Path(args.out))
    print(f"wrote {out}")
    print("pollen: max tilt %.0f deg, peak vertical %.2f m/s, final distance %.0f cm" % (
        max(p[2] for p in theirs), min(p[3] for p in theirs), theirs[-1][1]))
    print("cake:   max tilt %.0f deg, peak vertical %.2f m/s, final distance %.0f cm" % (
        max(p[2] for p in ours), min(p[3] for p in ours), ours[-1][1]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
