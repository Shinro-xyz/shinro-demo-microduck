"""Compiled kernel vs onnxruntime: the two independent runtimes agree.

``tests/test_import_fidelity.py`` checks the *imported graph*; this checks the
**compiled artifact** end to end — the Zig kernel called through the C ABI
against onnxruntime running the original ``.onnx``, on the same observations.
Together they close the loop: import is faithful, and the compiled form of that
import is faithful.

The measurement is shared with ``scripts/compare_backends.py`` (which also
reports footprint and latency) so there is one implementation of the comparison.
"""

import pytest

from shinro_demo_microduck.host import manifest_for
from shinro_demo_microduck.paths import DEFAULT_ARTIFACT

pytestmark = pytest.mark.kernel

#: f32 (ONNX runtime) vs f64 (kernel) over a 4-layer MLP; ~1e-6 observed.
F32_TOL = 1e-4


def test_compiled_kernel_agrees_with_onnxruntime():
    pytest.importorskip("onnxruntime", reason="onnxruntime is an optional cross-check only")
    if manifest_for(DEFAULT_ARTIFACT) is None:
        pytest.skip(f"no compiled kernel at {DEFAULT_ARTIFACT} — run `make compile`")

    from scripts.compare_backends import agreement

    worst = agreement(200)
    assert worst < F32_TOL, f"kernel and onnxruntime disagree by {worst:.2e}"
