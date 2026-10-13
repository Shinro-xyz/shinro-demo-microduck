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

import numpy as np

from shinro_demo_microduck import contract
from shinro_demo_microduck.sim import CONTROL_DT, MicroduckSim

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
    last_action = np.zeros(N_ACTIONS)

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
            rows.append((step * CONTROL_DT, float(pos[0]), float(pos[1]), float(pos[2])))
    finally:
        conn.close()
        server.close()
        sock_path.unlink(missing_ok=True)

    pos = sim.trunk_position
    travel = float(np.linalg.norm(pos[:2] - start_xy))
    metrics = {
        "steps": len(rows),
        "travel_m": travel,
        "mean_speed_mps": travel / (len(rows) * CONTROL_DT) if rows else 0.0,
        "final_trunk_z": float(pos[2]),
        "tilt_deg": float(np.degrees(sim.trunk_tilt)),
        "mean_abs_action": float(np.mean(np.abs(last_action))),
        "nan_obs": bool(not np.all(np.isfinite(sim.observation()))),
    }

    if csv_path is not None:
        try:
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            with open(csv_path, "w") as fh:
                fh.write("t,x,y,z\n")
                for t, x, y, z in rows:
                    fh.write(f"{t:.4f},{x:.6f},{y:.6f},{z:.6f}\n")
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
    args = parser.parse_args(argv)

    metrics = run(
        socket_path=args.socket,
        seconds=args.seconds,
        use_bam=not args.no_bam,
        csv_path=Path(args.csv) if args.csv else None,
    )
    if metrics["steps"] < 10:
        print("sim: too few steps for a verdict", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
