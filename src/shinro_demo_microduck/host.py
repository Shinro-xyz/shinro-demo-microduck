"""Host-side driver: run the Microduck sim from the compiled policy kernel.

The same controller config is loaded twice through shinro's ``onnx_rl``
adapter, which has two interchangeable backends over one graph:

* **eager** (``model_path``) — the ONNX is imported and executed by the pure
  numpy interpreter, in-process;
* **compiled** (``artifact_dir``) — ``lib/lib_neural_network.so`` is dlopen'd and
  driven through the ``shinro_step`` C ABI, with the graph manifest next to it
  describing the port layout.

Both run the *same* graph, so the honest correctness check is **lockstep
parity**: each tick both see the identical observation, and the kernel's action
is compared to the interpreter's. The plant is advanced only by the compiled
kernel's action — the eager policy never drives the robot.
"""

from __future__ import annotations

import json
import tomllib
from collections.abc import Callable
from pathlib import Path

import numpy as np
from shinro.controllers.onnx_rl_adapter import OnnxRLAdapter

from shinro_demo_microduck.paths import DEFAULT_ARTIFACT, HERE
from shinro_demo_microduck.sim import CONTROL_DT, MicroduckSim

#: Kernel stem the onnx_rl compiled backend loads (shinro's KERNEL_FILENAME).
KERNEL_FILENAME = "lib_neural_network.so"

_REPO_ROOT = HERE.parent.parent


def resolve_repo_path(path: str | Path) -> Path:
    """Resolve a repo-relative path, preferring CWD over the source checkout."""
    p = Path(path)
    if p.is_absolute():
        return p
    if p.exists():
        return p.resolve()
    return (_REPO_ROOT / p).resolve()


def load_controller_config(path: str | Path) -> dict:
    """Read an onnx_rl controller TOML and make ``model_path`` absolute."""
    with open(resolve_repo_path(path), "rb") as f:
        cfg = tomllib.load(f)
    if "model_path" in cfg:
        cfg["model_path"] = str(resolve_repo_path(cfg["model_path"]))
    return cfg


def load_policy(controller_config: str | Path, *, artifact_dir: str | Path | None = None) -> OnnxRLAdapter:
    """Build an ``onnx_rl`` adapter from the config (eager, or compiled when given)."""
    cfg = load_controller_config(controller_config)
    if artifact_dir is not None:
        cfg["artifact_dir"] = str(resolve_repo_path(artifact_dir))
    else:
        cfg.pop("artifact_dir", None)
    return OnnxRLAdapter.from_config(cfg)


def manifest_for(artifact_dir: str | Path) -> dict | None:
    """Return the graph manifest for a built artifact, or ``None`` if not built."""
    root = resolve_repo_path(artifact_dir)
    manifest_path = root / "graph_data_manifest.json"
    if not manifest_path.exists() or not (root / "lib" / KERNEL_FILENAME).exists():
        return None
    return json.loads(manifest_path.read_text())


def kernel_info(artifact_dir: str | Path = DEFAULT_ARTIFACT) -> str:
    """A one-line description of the compiled artifact (size, nodes, ports)."""
    root = resolve_repo_path(artifact_dir)
    so = root / "lib" / KERNEL_FILENAME
    manifest = manifest_for(root)
    if manifest is None:
        raise FileNotFoundError(f"no compiled kernel at {so} — run `make compile` first")
    nodes = manifest.get("nodes_total")
    ops = manifest.get("op_histogram", {})
    op_brief = ", ".join(f"{k}x{v}" for k, v in sorted(ops.items()))
    return f"{so}  ({nodes} nodes: {op_brief} · {so.stat().st_size / 1024:.0f} KiB)"


def drive(
    *,
    command: np.ndarray | None = None,
    controller_config: str | Path,
    artifact_dir: str | Path = DEFAULT_ARTIFACT,
    duration_s: float = 5.0,
    use_bam: bool = True,
    compare_eager: bool = True,
    sim: MicroduckSim | None = None,
    on_tick: Callable[[int, float, MicroduckSim], None] | None = None,
) -> dict:
    """Drive the sim from the compiled kernel; return run + parity metrics.

    Args:
        command: 13-D command block (default: all-zero idle).
        controller_config: The onnx_rl controller TOML.
        artifact_dir: A ``make compile`` directory (``lib/lib_neural_network.so``
            + ``graph_data_manifest.json``).
        duration_s: Simulated seconds to run.
        use_bam: Drive the BAM actuator (default) or the MJCF PD gains.
        compare_eager: Also run the eager interpreter each tick and report the
            lockstep parity (the correctness claim).
        sim: Use a pre-built sim instead of constructing one.
        on_tick: Called after each tick with ``(step, t, sim)``.

    Returns:
        Metrics: ``steps``, ``parity`` (max |kernel - interpreter| over the run,
        NaN when not compared), ``final_trunk_z``, ``travel_xy``, ``mean_speed``,
        ``yaw_rate`` (mean |yaw rate| rad/s), ``tilt_deg``.
    """
    kernel = load_policy(controller_config, artifact_dir=artifact_dir)
    eager = load_policy(controller_config) if compare_eager else None
    if sim is None:
        sim = MicroduckSim(command=command, use_bam=use_bam)

    n_steps = max(1, int(round(duration_s / CONTROL_DT)))
    parity = 0.0
    yaw_delta = 0.0
    prev_yaw = _yaw(sim.trunk_quaternion)
    start_xy = sim.trunk_position[:2].copy()

    for step in range(n_steps):
        obs = sim.observation()
        action = np.asarray(kernel.compute(obs), dtype=np.float64).ravel()
        if eager is not None:
            reference = np.asarray(eager.compute(obs), dtype=np.float64).ravel()
            parity = max(parity, float(np.max(np.abs(action - reference))))
        sim.step(action)
        yaw = _yaw(sim.trunk_quaternion)
        # Unwrap so a full turn is not double-counted as a small angle.
        yaw_delta += (yaw - prev_yaw + np.pi) % (2 * np.pi) - np.pi
        prev_yaw = yaw
        if on_tick is not None:
            on_tick(step, step * CONTROL_DT, sim)

    travel = float(np.linalg.norm(sim.trunk_position[:2] - start_xy))
    return {
        "steps": n_steps,
        "parity": parity if eager is not None else float("nan"),
        "final_trunk_z": float(sim.trunk_position[2]),
        "travel_xy": travel,
        "mean_speed": travel / duration_s,
        "yaw_rate": abs(float(yaw_delta)) / duration_s,
        "tilt_deg": float(np.degrees(sim.trunk_tilt)),
    }


def _yaw(quat_wxyz: np.ndarray) -> float:
    """Heading (yaw) of a ``(w, x, y, z)`` quaternion, in radians."""
    w, x, y, z = (float(v) for v in quat_wxyz)
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
