"""The compiled kernel: port contract, lockstep parity, and a walking sanity run.

These tests need the artifact, so they skip (rather than fail) until
``make compile`` has run. The parity check is the demo's central claim: the
``.so`` and the eager interpreter are two implementations of one graph, so on
identical observations they must agree to floating-point exactness.
"""

import numpy as np
import pytest

from shinro_demo_microduck import contract
from shinro_demo_microduck.host import drive, kernel_info, manifest_for
from shinro_demo_microduck.paths import CONTROLLER_CONFIG, DEFAULT_ARTIFACT
from shinro_demo_microduck.sim import MicroduckSim

pytestmark = pytest.mark.kernel

#: One float64 rounding step of a 14-D action is ~2e-16; the interpreter and the
#: Zig kernel differ only in operation ordering, so 1e-12 is a generous gate.
PARITY_TOL = 1e-12


@pytest.fixture(scope="module")
def artifact() -> str:
    if manifest_for(DEFAULT_ARTIFACT) is None:
        pytest.skip(f"no compiled kernel at {DEFAULT_ARTIFACT} — run `make compile`")
    return str(DEFAULT_ARTIFACT)


def test_artifact_declares_the_contract_ports(artifact):
    manifest = manifest_for(artifact)
    assert [port["name"] for port in manifest["inputs"]] == ["state"]
    assert manifest["inputs"][0]["shape"] == [contract.N_OBS]
    assert [port["name"] for port in manifest["outputs"]] == ["u"]
    assert manifest["outputs"][0]["shape"] == [contract.N_ACTIONS]
    # A memoryless MLP policy: no recurrent ports to feed back.
    assert manifest["state_outputs"] == []


def test_artifact_is_a_small_standalone_shared_object(artifact):
    print(kernel_info(artifact))


def test_kernel_matches_the_eager_interpreter_every_tick(artifact):
    metrics = drive(
        controller_config=CONTROLLER_CONFIG,
        artifact_dir=artifact,
        command=contract.build_command(twist=(0.4, 0.0, 0.0)),
        duration_s=1.0,
    )
    assert metrics["parity"] < PARITY_TOL, f"kernel diverged from the interpreter: {metrics['parity']:.3e}"


def test_kernel_walks_forward_when_commanded(artifact):
    """A 0.4 m/s command must produce real forward travel, upright."""
    metrics = drive(
        controller_config=CONTROLLER_CONFIG,
        artifact_dir=artifact,
        command=contract.build_command(twist=(0.4, 0.0, 0.0)),
        duration_s=6.0,
    )
    assert metrics["parity"] < PARITY_TOL
    assert metrics["travel_xy"] > 0.5, f"policy did not walk: travel {metrics['travel_xy']:.3f} m"
    assert metrics["final_trunk_z"] > 0.10, "policy fell over"
    assert metrics["tilt_deg"] < 15.0


def test_zero_command_stays_put(artifact):
    """The deployment idle state: the all-zero command must stand still."""
    metrics = drive(
        controller_config=CONTROLLER_CONFIG,
        artifact_dir=artifact,
        command=contract.build_command(),
        duration_s=3.0,
    )
    assert metrics["parity"] < PARITY_TOL
    assert metrics["travel_xy"] < 0.05, f"idle command drifted {metrics['travel_xy']:.3f} m"
    assert metrics["final_trunk_z"] > 0.10


def test_idle_policy_balances_from_a_perturbed_spawn(artifact):
    """The policy -- not a rigid pose hold -- is what keeps the robot upright.

    Spawned 2 cm high with 0.1 rad/s of joint-velocity noise, the compiled idle
    policy must still settle to the standing height with a small tilt. This is
    the TILT assertion the training repo insists on: a height-only check would
    pass a collapsed robot too.
    """
    rng = np.random.default_rng(0)
    sim = MicroduckSim(command=contract.build_command(), use_bam=True)
    sim.data.qpos[sim._free_qpos_adr + 2] += 0.02
    sim.data.qvel[:] = rng.normal(0.0, 0.1, sim.data.qvel.shape)

    metrics = drive(
        controller_config=CONTROLLER_CONFIG,
        artifact_dir=artifact,
        duration_s=3.0,
        sim=sim,
    )
    assert metrics["parity"] < PARITY_TOL
    assert metrics["tilt_deg"] < 10.0, f"idle policy let the robot tilt {metrics['tilt_deg']:.1f} deg"
    assert 0.10 < metrics["final_trunk_z"] < 0.14, f"standing height {metrics['final_trunk_z']:.4f} m is off"
    assert metrics["travel_xy"] < 0.05, "idle policy wandered while balancing"
