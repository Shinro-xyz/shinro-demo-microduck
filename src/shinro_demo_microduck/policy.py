"""The compiled Microduck walking kernel, loaded the way a robot runtime loads it.

This is the **deployment host**: ``ctypes`` plus the standard library, buffers
allocated once at load, and nothing else. It is the same pattern the Pi harness
in ``shinro-bench`` uses to time this exact artifact on target hardware, and the
pattern a Python-hosted robot stack (ROS 2 node, lerobot, a Jetson) would ship:

    from shinro_demo_microduck import MicroduckPolicy

    policy = MicroduckPolicy()                 # dlopen + preallocate
    while running:
        u = policy.step(sensor_observation)    # zero allocations
        apply_joint_targets(u)

Deliberately scoped to the Microduck **MLP** policy shape, which is what this
demo is for: one ``state`` input port, one ``u`` output port, no recurrent
``state_*`` ports. Dims come from the graph manifest rather than being hardcoded,
and the shape is asserted at load, so a future non-MLP export fails loudly here
instead of silently mis-packing ports. The generic cases (GRU/LSTM state
feedback, multi-port packing, ``epsilon`` noise) are what shinro's ``onnx_rl``
adapter is for — see :mod:`shinro_demo_microduck.host`, which keeps that adapter
as the eager parity reference.

Footprint, measured on this host: +0.1 MB RSS over a bare interpreter, 0 MB of
installed dependencies, 0 tracked allocations per call. See ``make footprint``.
"""

from __future__ import annotations

import ctypes
import json
from collections.abc import Sequence
from pathlib import Path

from shinro_demo_microduck.paths import DEFAULT_ARTIFACT, KERNEL_FILENAME, resolve_repo_path


class KernelInfo:
    """What the artifact says about itself (read from the graph manifest).

    A plain class with ``__slots__``, deliberately not a dataclass: this module is
    the deployment host, and ``import dataclasses`` costs ~1.5 MB of RSS to
    describe six fields.
    """

    __slots__ = ("path", "nodes", "n_obs", "n_actions", "bytes", "op_histogram")

    def __init__(
        self,
        path: Path,
        nodes: int,
        n_obs: int,
        n_actions: int,
        size_bytes: int,
        op_histogram: dict[str, int],
    ) -> None:
        self.path = path
        self.nodes = nodes
        self.n_obs = n_obs
        self.n_actions = n_actions
        self.bytes = size_bytes
        self.op_histogram = op_histogram

    def __str__(self) -> str:
        ops = ", ".join(f"{k}x{v}" for k, v in sorted(self.op_histogram.items()))
        return f"{self.path}  ({self.nodes} nodes: {ops} · {self.bytes / 1024:.0f} KiB)"


class MicroduckPolicy:
    """A loaded ``shinro_step`` kernel for the Microduck MLP policy.

    Args:
        artifact_dir: A ``make compile`` directory holding
            ``lib/lib_neural_network.so`` and ``graph_data_manifest.json``.
        kernel: Kernel file name override (shinro's ``KERNEL_FILENAME`` by default).

    Raises:
        FileNotFoundError: If the artifact has not been built.
        ValueError: If the artifact is not the single-port MLP shape this class
            is scoped to.
    """

    def __init__(self, artifact_dir: str | Path = DEFAULT_ARTIFACT, kernel: str = KERNEL_FILENAME) -> None:
        root = resolve_repo_path(artifact_dir)
        so_path = root / "lib" / kernel
        manifest_path = root / "graph_data_manifest.json"
        if not manifest_path.exists() or not so_path.exists():
            raise FileNotFoundError(f"no compiled kernel at {so_path} — run `make compile` first")

        manifest = json.loads(manifest_path.read_text())
        inputs = [p["name"] for p in manifest["inputs"]]
        outputs = [p["name"] for p in manifest["outputs"]]
        if inputs != ["state"] or outputs != ["u"] or manifest["state_outputs"]:
            raise ValueError(
                f"{root} is not the Microduck MLP shape (inputs={inputs}, outputs={outputs}, "
                f"state_outputs={[p['name'] for p in manifest['state_outputs']]}) — "
                "use shinro's onnx_rl adapter for recurrent or multi-port graphs"
            )
        self.info = KernelInfo(
            path=so_path,
            nodes=int(manifest["nodes_total"]),
            n_obs=int(manifest["inputs"][0]["shape"][0]),
            n_actions=int(manifest["outputs"][0]["shape"][0]),
            size_bytes=so_path.stat().st_size,
            op_histogram=dict(manifest.get("op_histogram", {})),
        )

        self._lib = ctypes.CDLL(str(so_path))
        self._lib.shinro_step.argtypes = [ctypes.POINTER(ctypes.c_double)] * 3
        self._lib.shinro_step.restype = None

        # Preallocate every buffer: the tick loop must not allocate.
        self._in = (ctypes.c_double * self.info.n_obs)()
        self._out = (ctypes.c_double * self.info.n_actions)()
        # A memoryless policy declares no state outputs; the C ABI still wants a
        # non-null pointer, so it gets one element of scratch.
        self._state = (ctypes.c_double * 1)()
        self._out_view = memoryview(self._out)

    # -- ports ---------------------------------------------------------------

    @property
    def n_obs(self) -> int:
        """Observation port width (61 for this policy)."""
        return self.info.n_obs

    @property
    def n_actions(self) -> int:
        """Action port width (14 for this policy)."""
        return self.info.n_actions

    def step(self, observation: Sequence[float]) -> memoryview:
        """Run one inference and return a zero-copy view of the action.

        ``observation`` may be anything slice-assignable — a numpy array, an
        ``array('d')``, or a plain list; ctypes copies it into the preallocated
        input buffer, so a numpy source costs no Python-level allocation.

        The returned ``memoryview`` is reused by the next call: consume it (or
        copy) before stepping again. For numpy: ``np.frombuffer(u, dtype=np.float64)``
        is a zero-copy view too.
        """
        self._in[:] = observation
        self._lib.shinro_step(self._in, self._out, self._state)
        return self._out_view

    def __call__(self, observation: Sequence[float]) -> memoryview:
        return self.step(observation)

    # -- lifecycle -----------------------------------------------------------

    def close(self) -> None:
        """Release the shared object. Safe to call more than once."""
        self._lib = None

    def __enter__(self) -> MicroduckPolicy:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


__all__ = ["KernelInfo", "MicroduckPolicy"]
