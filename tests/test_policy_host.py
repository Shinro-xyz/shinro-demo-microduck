"""``MicroduckPolicy`` — the deployment host.

This is the class a robot runtime loads, so the properties worth locking are the
ones a deployment depends on: dims come from the manifest (not hardcoded), the
non-MLP shape is rejected loudly, the tick loop allocates nothing, the C ABI
works from the standard library alone, and the arithmetic is bit-identical to an
independent implementation of the same graph.
"""

import subprocess
import sys
import textwrap
from array import array
from pathlib import Path

import numpy as np
import pytest

from shinro_demo_microduck.host import manifest_for
from shinro_demo_microduck.paths import DEFAULT_ARTIFACT
from shinro_demo_microduck.policy import MicroduckPolicy

pytestmark = pytest.mark.kernel


@pytest.fixture(scope="module")
def artifact() -> str:
    if manifest_for(DEFAULT_ARTIFACT) is None:
        pytest.skip(f"no compiled kernel at {DEFAULT_ARTIFACT} — run `make compile`")
    return str(DEFAULT_ARTIFACT)


def test_dims_come_from_the_manifest(artifact):
    policy = MicroduckPolicy(artifact)
    manifest = manifest_for(artifact)
    assert policy.n_obs == manifest["inputs"][0]["shape"][0] == 61
    assert policy.n_actions == manifest["outputs"][0]["shape"][0] == 14
    assert policy.info.nodes == manifest["nodes_total"]
    assert policy.info.bytes > 0
    assert "gemm" in policy.info.op_histogram
    assert "lib_neural_network.so" in str(policy.info)


def test_rejects_a_non_mlp_artifact(artifact, tmp_path):
    """A recurrent/multi-port graph must fail here, not mis-pack ports silently."""
    import json
    import shutil

    src = Path(artifact)
    (tmp_path / "lib").mkdir()
    shutil.copy2(src / "lib" / "lib_neural_network.so", tmp_path / "lib" / "lib_neural_network.so")
    manifest = json.loads((src / "graph_data_manifest.json").read_text())
    manifest["state_outputs"] = [{"name": "state_h_0", "shape": [64]}]
    (tmp_path / "graph_data_manifest.json").write_text(json.dumps(manifest))

    with pytest.raises(ValueError, match="not the Microduck MLP shape"):
        MicroduckPolicy(tmp_path)


def test_missing_artifact_says_how_to_build_it(tmp_path):
    with pytest.raises(FileNotFoundError, match="make compile"):
        MicroduckPolicy(tmp_path)


def test_step_is_zero_allocation(artifact):
    """The preallocated-buffer claim, measured rather than asserted.

    ``shinro-bench`` measures 4 KB of RSS growth over 100,000 ticks on the Pi
    precisely because of this property; on a laptop the equivalent check is that
    the tick loop allocates no Python objects at all.
    """
    import tracemalloc

    policy = MicroduckPolicy(artifact)
    obs = np.zeros(policy.n_obs)
    obs[5] = -1.0
    for _ in range(200):
        policy.step(obs)

    tracemalloc.start()
    tracemalloc.reset_peak()
    n = 5000
    for _ in range(n):
        policy.step(obs)
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    assert current / n <= 1.0, f"{current / n:.2f} B allocated per call is not a zero-allocation loop"
    assert peak / n <= 16.0, f"peak {peak / n:.2f} B per call"


def test_output_view_is_reused_not_allocated(artifact):
    """Documented behaviour: the returned view is valid until the next step."""
    policy = MicroduckPolicy(artifact)
    obs = np.zeros(policy.n_obs)
    first = policy.step(obs)
    second = policy.step(obs)
    assert first is second
    assert len(first) == policy.n_actions


def test_agrees_bit_for_bit_with_the_framework_adapter(artifact):
    """Same kernel bytes, two hosts: identical f64 arithmetic means identical output.

    This is the check that keeps the deployment host honest — if it packed ports
    differently, the numbers would differ here even though both call the same
    ``shinro_step``.
    """
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from shinro.controllers.onnx_rl_adapter import OnnxRLAdapter

    policy = MicroduckPolicy(artifact)
    adapter = OnnxRLAdapter.from_config({"artifact_dir": artifact, "action_space": "continuous", "deterministic": True})

    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(100):
        obs = rng.normal(0.0, 0.3, policy.n_obs)
        obs[3:6] = [0.0, 0.0, -1.0]
        a = np.frombuffer(policy.step(obs), dtype=np.float64)
        b = np.asarray(adapter.compute(obs), dtype=np.float64).ravel()
        worst = max(worst, float(np.max(np.abs(a - b))))
    assert worst == 0.0, f"hosts disagree by {worst:.3e} — same kernel, so they must be bit-identical"


def test_imports_and_steps_without_numpy(artifact):
    """The deployment claim: stdlib only, verified in a fresh interpreter.

    Importing the package must not pull the framework, numpy, or MuJoCo — the
    robot's loader should cost nothing but a ``dlopen``.
    """
    code = textwrap.dedent(
        f"""
        import sys
        from array import array
        from shinro_demo_microduck.policy import MicroduckPolicy

        policy = MicroduckPolicy({artifact!r})
        action = policy.step(array("d", [0.0] * policy.n_obs))
        assert len(action) == policy.n_actions, len(action)
        heavy = sorted({{m.split(".")[0] for m in sys.modules}} & {{"numpy", "scipy", "mujoco", "shinro", "onnxruntime", "torch"}})
        assert not heavy, f"deployment host pulled in {{heavy}}"
        print("OK")
        """
    )
    proc = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=False)
    assert proc.returncode == 0, proc.stderr[-2000:]
    assert "OK" in proc.stdout


def test_context_manager_closes_cleanly(artifact):
    with MicroduckPolicy(artifact) as policy:
        assert len(policy.step(array("d", [0.0] * policy.n_obs))) == policy.n_actions
    policy.close()  # idempotent
