"""Probe ONE policy backend in an isolated process and print its footprint.

Run via ``scripts/compare_backends.py`` (which spawns one process per backend so
the measurements do not contaminate each other); it is also usable directly::

    python scripts/backend_probe.py --backend onnxruntime
    python scripts/backend_probe.py --backend shinro-adapter
    python scripts/backend_probe.py --backend microduck-policy

Three backends, deliberately:

``onnxruntime``
    The reference runtime: ``onnxruntime.InferenceSession`` on the ``.onnx``.
``shinro-adapter``
    The compiled ``.so`` through shinro's ``onnx_rl`` adapter — the framework's
    generic compiled backend. It packs ports with numpy, and it is what the demo
    uses as the eager/compiled **parity reference**.
``microduck-policy``
    The compiled ``.so`` through :class:`shinro_demo_microduck.policy.MicroduckPolicy`
    — the **deployment host** this repo ships: ctypes + the standard library,
    preallocated buffers, zero dependencies. This is the row that says what the
    kernel itself costs.

Reported per backend: RSS at three points (process baseline / after loading /
after running), the file-backed shared objects mapped into the process, the time
from process start to first inference, and per-call latency. Everything is stdout
JSON on the last line so the parent can parse it.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from array import array
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
# Allow a source checkout without `pip install -e .` (the parent puts nothing on
# the path; the probe is spawned as a script, so sys.path[0] is scripts/).
sys.path.insert(0, str(REPO / "src"))

POLICY_ONNX = REPO / "models" / "microduck" / "BEST_alpha_walking.onnx"
ARTIFACT = REPO / "build" / "compiled_policy"
KERNEL = ARTIFACT / "lib" / "lib_neural_network.so"
MANIFEST = ARTIFACT / "graph_data_manifest.json"

#: Timestamp of the earliest moment this process could report on (module start),
#: so "time to first inference" includes the backend's import cost.
_T0 = time.perf_counter()

_N_OBS = 61


# ─── process footprint ──────────────────────────────────────────────────────


def rss_kb() -> int:
    """Resident set size of this process, in KiB (Linux ``/proc/self/status``)."""
    with open("/proc/self/status") as handle:
        for line in handle:
            if line.startswith("VmRSS:"):
                return int(line.split()[1])
    raise RuntimeError("VmRSS not found in /proc/self/status")


def mapped_objects() -> list[dict[str, int]]:
    """File-backed mappings in this process, deduplicated by path.

    Returns ``[{"path", "bytes"}…]`` — the shared objects (and data files) that
    are actually present in the address space, which is the "what did this
    backend drag in" question.
    """
    seen: dict[str, int] = {}
    with open("/proc/self/maps") as handle:
        for line in handle:
            parts = line.rstrip("\n").split(maxsplit=5)
            if len(parts) < 6:
                continue
            path = parts[5]
            if not path.startswith("/"):
                continue
            start, end = (int(v, 16) for v in parts[0].split("-"))
            seen[path] = seen.get(path, 0) + (end - start)
    return [{"path": path, "bytes": size} for path, size in sorted(seen.items(), key=lambda kv: -kv[1])]


def is_runtime_lib(path: str) -> bool:
    """True for a file this backend loaded that a *different* backend would not."""
    return any(marker in path for marker in ("onnxruntime", "libpython", "numpy", "scipy"))


# ─── backends ───────────────────────────────────────────────────────────────


def _prod(shape) -> int:
    total = 1
    for dim in shape or []:
        total *= int(dim)
    return total


def _make_input(n: int) -> list[float]:
    """A plausible standing observation, in the contract's order.

    gravity is [0, 0, -1]; a small nonzero command so nothing is a trivially
    zero vector (which some runtimes can shortcut).
    """
    obs = [0.0] * n
    obs[5] = -1.0
    obs[48] = 0.4  # twist vx
    return obs


def probe(backend: str, samples: int, warmup: int) -> dict:
    baseline = rss_kb()
    obs_list = _make_input(_N_OBS)

    if backend == "onnxruntime":
        import numpy as np
        import onnxruntime as ort

        rss_after_import = rss_kb()
        session = ort.InferenceSession(str(POLICY_ONNX), providers=["CPUExecutionProvider"])
        feed = {"obs": np.asarray(obs_list, dtype=np.float32).reshape(1, -1)}
        rss_loaded = rss_kb()

        def run():
            return session.run(None, feed)[0]

    elif backend == "shinro-adapter":
        import numpy as np
        from shinro.controllers.onnx_rl_adapter import OnnxRLAdapter

        rss_after_import = rss_kb()
        policy = OnnxRLAdapter.from_config({"artifact_dir": str(ARTIFACT), "action_space": "continuous", "deterministic": True})
        obs = np.asarray(obs_list, dtype=np.float64)
        rss_loaded = rss_kb()

        def run():
            return policy.compute(obs)

    elif backend == "microduck-policy":
        from shinro_demo_microduck.policy import MicroduckPolicy

        rss_after_import = rss_kb()
        policy = MicroduckPolicy(ARTIFACT)
        buf = array("d", obs_list)
        rss_loaded = rss_kb()

        def run():
            return policy.step(buf)

    else:  # pragma: no cover - argparse restricts the choices
        raise SystemExit(f"unknown backend {backend!r}")

    first = run()  # the first call: this is "time to ready"
    t_ready = time.perf_counter() - _T0
    results = [first]

    for _ in range(warmup):
        run()

    latencies_ns: list[int] = []
    for _ in range(samples):
        t0 = time.perf_counter_ns()
        results.append(run())
        latencies_ns.append(time.perf_counter_ns() - t0)

    rss_after = rss_kb()
    latencies_ns.sort()

    def pct(p: float) -> float:
        return latencies_ns[min(len(latencies_ns) - 1, int(p * len(latencies_ns)))]

    return {
        "backend": backend,
        "python": sys.version.split()[0],
        "samples": samples,
        "rss_baseline_kb": baseline,
        "rss_after_import_kb": rss_after_import,
        "rss_loaded_kb": rss_loaded,
        "rss_after_run_kb": rss_after,
        "time_to_ready_s": round(t_ready, 4),
        "latency_us_median": round(pct(0.5) / 1000.0, 3),
        "latency_us_p99": round(pct(0.99) / 1000.0, 3),
        "latency_us_min": round(latencies_ns[0] / 1000.0, 3),
        "throughput_hz": round(1e9 / (sum(latencies_ns) / len(latencies_ns)), 1),
        "mapped_total_mb": round(sum(m["bytes"] for m in mapped_objects()) / 1e6, 2),
        "mapped_runtime_mb": round(sum(m["bytes"] for m in mapped_objects() if is_runtime_lib(m["path"])) / 1e6, 2),
        "mapped_top": [m["path"] for m in mapped_objects()[:12]],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--backend", required=True, choices=["onnxruntime", "shinro-adapter", "microduck-policy"])
    parser.add_argument("--samples", type=int, default=2000)
    parser.add_argument("--warmup", type=int, default=200)
    args = parser.parse_args(argv)

    for path in (POLICY_ONNX, KERNEL, MANIFEST):
        if not path.exists():
            print(f"missing {path}", file=sys.stderr)
            return 2

    print(json.dumps(probe(args.backend, args.samples, args.warmup)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
