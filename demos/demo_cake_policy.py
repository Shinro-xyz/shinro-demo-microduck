"""Run the Microduck walk with the policy on the **Cake control plane**.

The bare-kernel demo (:mod:`demos.demo_compiled_policy`) reaches the policy the
Linux way: :class:`~shinro_demo_microduck.policy.MicroduckPolicy` ``dlopen``s
``lib_neural_network.so`` and calls ``shinro_step`` itself, owning the pthread
and the tick loop. This demo reaches the *same* policy through the **Cake
supervisory kernel** instead: the resident admits the policy as a Cake module,
the module owns its own thread, and it reads its observation from — and
publishes its action to — the control plane.

This process owns only the **plant**. It steps MuJoCo and relays proprioception
and the trunk pose, then the action, over a Unix-domain socket to the Cake module
on the other side:

    sim proprioception ─► 48 f64 ─┐
    sim pose (x,y,yaw,ω) ─► 4 f64 ─┴─► [ Vm.step, in a Cake module ] ─► 14 f64 ─► sim.step()
                                        ▲
                        13-D command ───┘ from the swappable trajectory module

The 13-D command block (the trajectory) is **not** built here: it is a separate
Cake module on the plane, so it can be swapped live with ``replace-slot`` while
the robot keeps walking. The 4-D pose is what lets that module run closed-loop
pure pursuit against its reference path. This client only runs the plant.

For the render, the client also mirrors each trajectory capsule's reference path
(through :mod:`shinro_demo_microduck.trajectory`, the module the capsule's baked
shapes were written to match) and measures the trunk's cross-track error each
tick, so the panel can show how well each module was tracked before and after a
swap.

Nothing Cake-shaped reaches this side. There is no ``cake_*`` header, no entry
symbol, no lifecycle callback, and no Cake source in this repository: the wire is
just two little-endian f64 arrays — the graph's own port layout.

Launch it from the repo root; ``make cake`` fetches the release, stages the
deployment, starts the resident, and drives this client:

    make cake                       # circle trajectory, 8 s
    make cake TRAJ=slalom SECS=8
    make cake SWAP=circle,eight,slalom   # switch trajectory live, no restart

Run this file directly only when the deployment is already up (the module has
bound its socket):

    python -m demos.demo_cake_policy --socket /tmp/cake-microduck.sock --seconds 8
"""

# pyright: reportAttributeAccessIssue=false, reportOptionalMemberAccess=false
#   The render panels and offscreen renderer are optional and typed as None by
#   Pyright (mujoco has no stubs), so their attributes are misreported. Runtime
#   behaviour is correct; this repo's lint gate is ruff, not Pyright.

from __future__ import annotations

import os

os.environ.setdefault("MUJOCO_GL", "egl")

import argparse
import json
import math
import socket
import struct
import sys
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from shinro_demo_microduck import contract
from shinro_demo_microduck.sim import CONTROL_DT, MicroduckSim
from shinro_demo_microduck.viz import GIF_PROFILES

if TYPE_CHECKING:
    from shinro_demo_microduck.trajectory import ReferencePath

#: The command block width (`twist(3)` + `head_pose(4)` + `body_pose(6)`); it is
#: the trajectory module's, not this process's.
N_COMMAND = 13
#: What crosses the socket: the observation minus the command block, plus the
#: 4-D trunk pose `(x, y, yaw, yaw_rate)` the trajectory needs for pure pursuit.
N_PROPRIO = contract.N_OBS - N_COMMAND
N_POSE = 4
N_SEND = N_PROPRIO + N_POSE
N_ACTIONS = contract.N_ACTIONS
SEND_STRUCT = struct.Struct(f"<{N_SEND}d")
ACT_STRUCT = struct.Struct(f"<{N_ACTIONS}d")

#: Cake trajectory capsule name -> the ``trajectory.py`` shape it was compiled
#: from (``demo/trajectory.cmdl`` bakes ``trajectory = circle|eight|slalom|
#: forward``). The client mirrors the reference here so it can measure how far the
#: trunk strayed from the path the *active* module was steering toward — the
#: capsule itself lives in the private tree, so the mirror is the only way to see
#: the reference from this side.
_CAKE_SHAPES = {
    "circle": ("circle", True),
    "eight": ("figure_eight", True),
    "slalom": ("slalom", False),
    "forward": ("straight", False),
}


def _reference_for(name: str) -> ReferencePath | None:
    """The reference path a Cake trajectory capsule was built from, or ``None``.

    Uses :mod:`shinro_demo_microduck.trajectory` (the documented mirror of the
    capsule's baked shapes — the module's defaults match the capsule's
    ``circle``/``eight``/``slalom``/``forward`` exactly). Imported lazily so the
    non-rendering client never pulls the framework in.
    """
    spec = _CAKE_SHAPES.get(name)
    if spec is None:
        return None
    from shinro_demo_microduck import trajectory as traj

    attr, closed = spec
    return traj.ReferencePath(getattr(traj, attr)(), closed=closed)


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _path_start_tangent(reference: ReferencePath) -> float:
    """The heading a reference path leaves its start with — what an anchored
    capsule rotates onto the robot's heading (circle +x, eight 45 deg, slalom
    ~54 deg)."""
    pts = reference.points
    return math.atan2(float(pts[1][1] - pts[0][1]), float(pts[1][0] - pts[0][0]))


def _anchor_points(reference: ReferencePath, anchor: tuple[float, float, float], theta: float) -> np.ndarray:
    """The reference path re-parented onto ``anchor`` with its start tangent
    rotated by ``theta`` — the world path an anchored capsule steers toward."""
    pts = np.asarray(reference.points, dtype=np.float64)[:, :2]
    cth, sth = math.cos(theta), math.sin(theta)
    world = np.empty_like(pts)
    world[:, 0] = cth * pts[:, 0] - sth * pts[:, 1] + anchor[0]
    world[:, 1] = sth * pts[:, 0] + cth * pts[:, 1] + anchor[1]
    return world


def _segment_at(segments: list[tuple[str, float]], t: float) -> int:
    """Index of the trajectory active at ``t`` seconds (mirrors ``SwapPanels``)."""
    idx = 0
    for i, (_name, start) in enumerate(segments):
        if t >= start:
            idx = i
    return idx


def _yaw(quat_wxyz: np.ndarray) -> float:
    """Heading (yaw) of a ``(w, x, y, z)`` quaternion, in radians."""
    w, x, y, z = (float(v) for v in quat_wxyz)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _state(sim: MicroduckSim) -> list[float]:
    """The bytes the module reads each tick: proprioception then the pose."""
    pos = sim.trunk_position
    return [
        *sim.observation()[:N_PROPRIO],
        float(pos[0]),
        float(pos[1]),
        _yaw(sim.trunk_quaternion),
        float(sim.yaw_rate),
    ]


def _recv_exact(conn: socket.socket, size: int) -> bytes | None:
    """Read exactly ``size`` bytes, or ``None`` at EOF."""
    buf = bytearray()
    while len(buf) < size:
        chunk = conn.recv(size - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def run(
    *,
    socket_path: str,
    seconds: float,
    use_bam: bool,
    csv_path: Path | None,
    render=None,
    out_dir: Path | None = None,
    segments: list[tuple[str, float]] | None = None,
    panel_seconds: float | None = None,
    anchor: bool = False,
    reference_traj: str | None = None,
    module_label: str = "trajectory module",
    video_name: str = "microduck_cake_walk.gif",
) -> dict:
    # The sim's own `command` field is unused: the trajectory comes from the
    # Cake plane. A zero command keeps the sim's reset state canonical.
    sim = MicroduckSim(use_bam=use_bam)
    print(f"sim: MuJoCo ready ({N_PROPRIO}-D proprio + {N_POSE}-D pose, {N_ACTIONS}-D action)")

    # The sim is the server: bind, listen, and wait for the Cake module to
    # connect on the other side of the plane.
    sock_path = Path(socket_path)
    if sock_path.exists():
        sock_path.unlink()
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(socket_path)
    server.listen(1)
    conn, _ = server.accept()
    print(f"sim: Cake module connected on {socket_path}")

    n_steps = max(1, int(round(seconds / CONTROL_DT)))
    start_xy = sim.trunk_position[:2].copy()
    rows: list[tuple[float, float, float, float]] = []
    ctes: list[float] = []
    last_action = np.zeros(N_ACTIONS)

    # The reference path each scheduled segment tracks, so the client can measure
    # cross-track error without reaching into the private capsule. A policy swap
    # schedules *policy* segments (stand/walk) under one trajectory, so `reference`
    # names the trajectory whose path every segment tracks.
    if segments and reference_traj:
        base = _reference_for(reference_traj)
        refs = [base] * len(segments)
    else:
        refs = [_reference_for(name) for name, _ in segments] if segments else None
    # Anchored metric state: the capture is per segment (see the loop).
    anchor_seg = -1
    anchor_pose = (0.0, 0.0, 0.0)
    anchor_theta = 0.0

    # Optional video: the 3-D walk plus a bird's-eye panel that colours the path
    # by the trajectory module driving it, plots the tracking error, and marks
    # each live swap.
    panels = None
    grabber = None
    if render is not None:
        from shinro_demo_microduck.viz import FrameGrabber, SwapPanels

        seg_list = segments or [("trajectory", 0.0)]
        seg_refs: list[np.ndarray | None] = (
            [None if ref is None else ref.points for ref in refs] if refs is not None else [None] * len(seg_list)
        )
        # The client is launched with a +40 s safety margin on the loop, so the
        # panel's time axis must follow the *intended* run, not the loop bound —
        # otherwise the walk only fills a third of the window.
        panels = SwapPanels(
            seg_list, panel_seconds if panel_seconds is not None else seconds, references=seg_refs, label=module_label
        )
        grabber = FrameGrabber(sim, render, panels)
        if not grabber.enabled:
            grabber = None
            panels = None

    # The module reads proprioception and the pose first; prime it with the
    # reset state.
    conn.sendall(SEND_STRUCT.pack(*_state(sim)))

    try:
        for step in range(n_steps):
            try:
                raw = _recv_exact(conn, ACT_STRUCT.size)
                if raw is None:
                    print("sim: the Cake module closed the socket; stopping")
                    break
                action = np.asarray(ACT_STRUCT.unpack(raw), dtype=np.float64)
                sim.step(action)
                conn.sendall(SEND_STRUCT.pack(*_state(sim)))
            except (ConnectionResetError, BrokenPipeError):
                print("sim: the Cake module connection went away; stopping")
                break
            last_action = action
            pos = sim.trunk_position
            now = step * CONTROL_DT
            # Cross-track error against the active module's reference: 0 means the
            # trunk rode the path; the jump across a swap is the transition the
            # robot has to make onto the new trajectory.
            seg = _segment_at(segments, now) if segments else 0
            cross_track = math.nan
            if refs is not None:
                reference = refs[seg]
                if reference is not None:
                    if anchor:
                        # An anchored capsule re-parents its path onto the pose it
                        # first reads, so mirror that: capture the anchor at the
                        # segment's first tick and measure the transformed pose.
                        if seg != anchor_seg:
                            anchor_seg = seg
                            anchor_pose = (float(pos[0]), float(pos[1]), _yaw(sim.trunk_quaternion))
                            anchor_theta = _wrap_pi(anchor_pose[2] - _path_start_tangent(reference))
                            if panels is not None:
                                panels.set_reference(seg, _anchor_points(reference, anchor_pose, anchor_theta))
                        dx = float(pos[0]) - anchor_pose[0]
                        dy = float(pos[1]) - anchor_pose[1]
                        cth, sth = math.cos(anchor_theta), math.sin(anchor_theta)
                        local = np.array([cth * dx + sth * dy, -sth * dx + cth * dy])
                        cross_track = float(reference.cross_track(local))
                    else:
                        cross_track = float(reference.cross_track(pos))
            rows.append((now, float(pos[0]), float(pos[1]), float(pos[2])))
            ctes.append(cross_track)
            if grabber is not None and grabber.due(step):
                panels.update(now, pos, seg, cross_track)
                grabber.capture(sim)
    finally:
        conn.close()
        server.close()
        sock_path.unlink(missing_ok=True)

    video = None
    if grabber is not None:
        video = grabber.save((out_dir or Path("build/cake/run")) / video_name)
        grabber.close()
        if video is not None:
            print(f"sim: wrote {video} ({video.stat().st_size / 1e6:.2f} MB)")

    finite_cte = [c for c in ctes if math.isfinite(c)]
    pos = sim.trunk_position
    travel = float(np.linalg.norm(pos[:2] - start_xy))
    metrics = {
        "steps": len(rows),
        "travel_m": travel,
        "mean_speed_mps": travel / (len(rows) * CONTROL_DT) if rows else 0.0,
        "final_trunk_z": float(pos[2]),
        "tilt_deg": float(np.degrees(sim.trunk_tilt)),
        "mean_abs_action": float(np.mean(np.abs(last_action))),
        "mean_cross_track_m": float(np.mean(finite_cte)) if finite_cte else None,
        "nan_obs": bool(not np.all(np.isfinite(sim.observation()))),
        "video": str(video) if video is not None else None,
    }

    if csv_path is not None:
        try:
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            with open(csv_path, "w") as fh:
                fh.write("t,x,y,z,cross_track\n")
                for (t, x, y, z), cte in zip(rows, ctes):
                    fh.write(f"{t:.4f},{x:.6f},{y:.6f},{z:.6f},{cte:.6f}\n")
        except OSError as exc:
            print(f"sim: could not write {csv_path}: {exc}", file=sys.stderr)
        else:
            print(f"sim: wrote {len(rows)} rows to {csv_path}")

    print("sim: metrics " + json.dumps(metrics))
    print("RESULT " + json.dumps(metrics))
    return metrics


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--socket", default="/tmp/cake-microduck.sock", help="Unix socket the Cake module connects to")
    parser.add_argument("--seconds", type=float, default=8.0, help="simulated seconds to run")
    parser.add_argument("--no-bam", action="store_true", help="use the MJCF placeholder PD gains, not BAM")
    parser.add_argument("--csv", default=None, help="write the trunk trajectory here")
    parser.add_argument(
        "--gif",
        choices=sorted(GIF_PROFILES),
        default=None,
        help="render a GIF of the walk (needs an offscreen GL backend)",
    )
    parser.add_argument("--out-dir", default=None, help="where to write the GIF")
    parser.add_argument(
        "--schedule",
        default=None,
        help="trajectory segments for the panel, e.g. circle@0,eight@3,slalom@6",
    )
    parser.add_argument(
        "--panel-duration",
        type=float,
        default=None,
        help="intended run length for the panel time axis (defaults to --seconds)",
    )
    parser.add_argument(
        "--anchor",
        action="store_true",
        help="the trajectory capsules re-parent their paths onto the duck; mirror that in the error metric",
    )
    parser.add_argument(
        "--reference",
        default=None,
        help="trajectory whose path every scheduled segment tracks (for a policy swap: --schedule stand@0,walk@2 --reference circle)",
    )
    parser.add_argument(
        "--module-label",
        default="trajectory module",
        help="what --schedule's segments are, for the panel titles (e.g. 'policy')",
    )
    parser.add_argument(
        "--video-name",
        default="microduck_cake_walk.gif",
        help="GIF file name under --out-dir",
    )
    args = parser.parse_args(argv)

    segments = None
    if args.schedule:
        segments = [(name, float(start)) for name, _, start in (part.partition("@") for part in args.schedule.split(","))]

    metrics = run(
        socket_path=args.socket,
        seconds=args.seconds,
        use_bam=not args.no_bam,
        csv_path=Path(args.csv) if args.csv else None,
        render=GIF_PROFILES[args.gif] if args.gif else None,
        out_dir=Path(args.out_dir) if args.out_dir else None,
        segments=segments,
        panel_seconds=args.panel_duration,
        anchor=args.anchor,
        reference_traj=args.reference,
        module_label=args.module_label,
        video_name=args.video_name,
    )
    if metrics["steps"] < 10:
        print("sim: too few steps for a verdict", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
