"""The importer, not just the compiler: the imported graph is the same policy.

``shinro.codegen.onnx_import`` translates the ONNX into a shinro graph (folding
the observation encoder and the action post-processing into arithmetic on baked
constants). The lockstep parity test in ``test_kernel_parity.py`` proves the
kernel matches the interpreter — but both could be wrong together if the
*import* were wrong. This test pins the import against onnxruntime, the
reference implementation of the ONNX spec.

The gap is f32 vs f64 arithmetic: onnxruntime runs the ONNX graph in float32,
shinro in float64, so ~1e-6 is the expected agreement on actions of order 0.1.
onnxruntime is deliberately NOT a dependency of this demo (the point of the
compiled path is that it has none), so the test skips when it is absent.
"""

import numpy as np
import pytest

from shinro_demo_microduck import contract
from shinro_demo_microduck.host import load_policy
from shinro_demo_microduck.paths import CONTROLLER_CONFIG, POLICY_ONNX

#: f32 (ONNX) vs f64 (shinro) arithmetic over a 4-layer MLP.
F32_TOL = 1e-4


@pytest.fixture(scope="module")
def onnxruntime():
    return pytest.importorskip("onnxruntime", reason="onnxruntime is an optional cross-check only")


def test_imported_graph_matches_onnxruntime(onnxruntime):
    session = onnxruntime.InferenceSession(str(POLICY_ONNX), providers=["CPUExecutionProvider"])
    eager = load_policy(CONTROLLER_CONFIG)

    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(25):
        obs = rng.normal(0.0, 0.4, contract.N_OBS)
        obs[contract.OBS_PROJECTED_GRAVITY] = [0.0, 0.0, -1.0]  # keep it a plausible pose
        reference = session.run(None, {"obs": obs.reshape(1, -1).astype(np.float32)})[0].ravel()
        worst = max(worst, float(np.max(np.abs(np.asarray(eager.compute(obs)).ravel() - reference))))
    assert worst < F32_TOL, f"import disagrees with onnxruntime by {worst:.2e}"


def test_onnxruntime_sees_the_same_joint_and_command_metadata():
    import onnx

    meta = {p.key: p.value for p in onnx.load(str(POLICY_ONNX)).metadata_props}
    assert meta["joint_names"].split(",") == list(contract.JOINT_NAMES)
    assert float(meta["action_scale"]) == 1.0
