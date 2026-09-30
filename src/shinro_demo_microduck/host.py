"""Host-side driver: run the Microduck sim from the compiled policy kernel.

Two things live here and they play different roles:

* **the actor** — :class:`~shinro_demo_microduck.policy.MicroduckPolicy`, the
  ctypes deployment host. It owns the tick loop: it preallocates buffers, calls
  the kernel's C ABI, and its action drives the plant. This is how a team loads
  the artifact.
* **the reference** — shinro's ``onnx_rl`` adapter in *eager* mode, which imports
  the ONNX and executes it with the pure-numpy interpreter. It never drives the
  robot; it exists so the kernel can be checked against an independent
  implementation of the same graph, every tick, on the same observation.

Parity is therefore **lockstep**: both see the identical 61-D observation and
the kernel's 14-D action is compared to the interpreter's. (shinro's adapter can
also *be* the actor — ``backend="shinro"`` — which is how the framework's own
compiled backend is A/B'd against the deployment host; it needs numpy to pack
port buffers and costs ~47 MB more RSS for identical arithmetic.)
"""

from __future__ import annotations

import json
import tomllib
from collections.abc import Callable
from pathlib import Path

import numpy as np
from shinro.controllers.onnx_rl_adapter import OnnxRLAdapter

from shinro_demo_microduck.paths import DEFAULT_ARTIFACT, KERNEL_FILENAME, resolve_repo_path
from shinro_demo_microduck.policy import MicroduckPolicy
from shinro_demo_microduck.sim import CONTROL_DT, MicroduckSim

BACKENDS = ("ctypes", "shinro")


def load_controller_config(path: str | Path) -> dict:
    """Read an onnx_rl controller TOML and make ``model_path`` absolute."""
    with open(resolve_repo_path(path), "rb") as f:
        cfg = tomllib.load(f)
    if "model_path" in cfg:
        cfg["model_path"] = str(resolve_repo_path(cfg["model_path"]))
    return cfg


def load_policy(controller_config: str | Path, *, artifact_dir: str | Path | None = None) -> OnnxRLAdapter:
    """Build shinro's ``onnx_rl`` adapter (eager, or its compiled backend when given)."""
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
    """A one-line description of the compiled artifact (path, nodes, size)."""
    return str(MicroduckPolicy(artifact_dir).info)


def drive(
    *,
    command: np.ndarray | None = None,
    controller_config: str | Path,
    artifact_dir: str | Path = DEFAULT_ARTIFACT,
    duration_s: float = 5.0,
    use_bam: bool = True,
    backend: str = "ctypes",
    compare_eager: bool = True,
    sim: MicroduckSim | None = None,
    on_tick: Callable[[int, float, MicroduckSim], None] | None = None,
) -> dict:
    """Drive the sim from the compiled kernel; return run + parity metrics.

    Args:
        command: 13-D command block (default: all-zero idle).
        controller_config: The onnx_rl controller TOML (source of the reference).
        artifact_dir: A ``make compile`` directory.
        duration_s: Simulated seconds to run.
        use_bam: Drive the BAM actuator (default) or the MJCF PD gains.
        backend: ``"ctypes"`` (default) drives with the deployment host;
            ``"shinro"`` drives with shinro's compiled adapter instead.
        compare_eager: Run the eager interpreter each tick and report lockstep
            parity (the correctness claim).
        sim: Use a pre-built sim instead of constructing one.
        on_tick: Called after each tick with ``(step, t, sim)``.

    Returns:
        Metrics: ``steps``, ``parity`` (max |kernel - interpreter| over the run,
        NaN when not compared), ``backend``, ``final_trunk_z``, ``travel_xy``,
        ``mean_speed``, ``yaw_rate`` (mean |yaw rate| rad/s), ``tilt_deg``.
    """
    if backend not in BACKENDS:
        raise ValueError(f"backend must be one of {BACKENDS}, got {backend!r}")

    kernel = MicroduckPolicy(artifact_dir) if backend == "ctypes" else load_policy(controller_config, artifact_dir=artifact_dir)
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
        # Both backends hand back a 14-D float64 action; the kernel's is a
        # zero-copy view over its preallocated output buffer.
        action = (
            np.frombuffer(kernel.step(obs), dtype=np.float64)
            if backend == "ctypes"
            else np.asarray(kernel.compute(obs), dtype=np.float64).ravel()
        )
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
        "backend": backend,
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
