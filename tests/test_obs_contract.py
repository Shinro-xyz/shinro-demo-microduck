"""The 61-D observation / 14-D action contract, checked against the ONNX itself.

These are the invariants a wrong number silently breaks: joint order, the
default pose, the command block layout, and the fact that the exporter's own
metadata agrees with the code that builds the observation.
"""

import numpy as np
import pytest

from shinro_demo_microduck import contract
from shinro_demo_microduck.paths import POLICY_ONNX


def test_onnx_metadata_matches_the_contract():
    """The policy's stamped joint order / default pose / terms == contract.py."""
    meta = contract.verify_against_onnx(POLICY_ONNX)
    assert meta["joint_names"].split(",") == list(contract.JOINT_NAMES)
    assert meta["command_names"].split(",") == list(contract.COMMAND_NAMES)
    assert float(meta["action_scale"]) == contract.ACTION_SCALE


def test_obs_layout_is_contiguous_and_61_dimensional():
    slices = [
        contract.OBS_BASE_ANG_VEL,
        contract.OBS_PROJECTED_GRAVITY,
        contract.OBS_JOINT_POS,
        contract.OBS_JOINT_VEL,
        contract.OBS_LAST_ACTION,
        contract.OBS_TWIST_CMD,
        contract.OBS_HEAD_CMD,
        contract.OBS_BODY_CMD,
    ]
    assert slices[0].start == 0
    for previous, current in zip(slices, slices[1:]):
        assert current.start == previous.stop, "observation blocks must be contiguous"
    assert slices[-1].stop == contract.N_OBS == 61


def test_command_block_order_is_twist_then_head_then_body():
    command = contract.build_command(twist=(0.3, -0.1, 0.5), head_pose=(1, 2, 3, 4), body_pose=(5, 6, 7, 8, 9, 10))
    assert list(command[:3]) == [0.3, -0.1, 0.5]
    assert list(command[3:7]) == [1, 2, 3, 4]
    assert list(command[7:13]) == [5, 6, 7, 8, 9, 10]
    assert command.shape == (13,)


def test_command_slices_point_into_the_obs_block():
    obs = np.zeros(contract.N_OBS)
    obs[contract.COMMAND_SLICES["twist"]] = [1.0, 2.0, 3.0]
    obs[contract.COMMAND_SLICES["head_pose"]] = [4.0, 5.0, 6.0, 7.0]
    obs[contract.COMMAND_SLICES["body_pose"]] = [8.0, 9.0, 10.0, 11.0, 12.0, 13.0]
    assert obs[contract.OBS_TWIST_CMD].tolist() == [1.0, 2.0, 3.0]
    assert obs[contract.OBS_HEAD_CMD].tolist() == [4.0, 5.0, 6.0, 7.0]
    assert obs[contract.OBS_BODY_CMD].tolist() == [8.0, 9.0, 10.0, 11.0, 12.0, 13.0]


def test_build_command_rejects_wrong_lengths():
    with pytest.raises(ValueError):
        contract.build_command(twist=(0.1, 0.2))


def test_quat_rotate_inverse_at_identity_is_identity():
    identity = np.array([1.0, 0.0, 0.0, 0.0])
    assert np.allclose(contract.quat_rotate_inverse(identity, [0.0, 0.0, -1.0]), [0.0, 0.0, -1.0])


def test_quat_rotate_inverse_holds_gravity_length():
    quat = np.array([0.7071, 0.0, 0.0, 0.7071])  # yaw 90 deg
    gravity = contract.quat_rotate_inverse(quat, [0.0, 0.0, -1.0])
    assert np.isclose(np.linalg.norm(gravity), 1.0)
