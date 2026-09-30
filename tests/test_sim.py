"""MuJoCo plant: the BAM mirror, the observation builder, and a settle check.

The settle check is the training repo's own lesson applied to this demo: a rest
pose must be a stable equilibrium, and "resting fine" must be judged on TILT,
not on height alone (a fallen robot also has a plausible trunk z).
"""

import numpy as np
import pytest

from shinro_demo_microduck import bam, contract
from shinro_demo_microduck.contract import quat_rotate_inverse
from shinro_demo_microduck.paths import MESH_DIR, SCENE_WALK
from shinro_demo_microduck.presets import load_scene_assets, microduck_preset
from shinro_demo_microduck.sim import MicroduckSim

pytestmark = pytest.mark.integration


def test_bam_constants_mirror_training():
    """The BAM settings the demo replays with must equal the training kwargs."""
    # robot/microduck_constants.py :: _BAM_ACTUATOR_KWARGS
    assert bam.BAM_MOTOR_NAME == "xl330"
    assert bam.BAM_MODEL == "m6"
    assert bam.BAM_KP_FW == 200.0
    assert bam.BAM_VIN_RANGE == (6.5, 8.2)
    assert bam.BAM_VIN_DROP_GAIN_RANGE == (0.0, 0.2)
    assert bam.BAM_VIN_MIN == 6.0
    assert bam.BAM_MAX_CURRENT is None
    # bam.mjlab.BamActuator stiff_frictionloss=True
    assert bam.BAM_STIFF_SOLREF_FRICTION == (-5.0e4, -2.0e2)
    assert bam.BAM_STIFF_SOLIMP_FRICTION == (0.99, 0.9999, 0.001, 0.5, 2.0)


def test_bam_model_actuator_is_a_voltage_bounded_motor():
    sim = MicroduckSim(use_bam=True)
    kt, r = sim.bam_model.kt.value, sim.bam_model.R.value
    force_limit = bam.DEFAULT_VIN * kt / r
    for i in range(sim.model.nu):
        assert sim.model.actuator_gaintype[i] == 0, "BAM actuator must be a plain motor"
        lo, hi = sim.model.actuator_forcerange[i]
        assert np.isclose(hi, force_limit)
        assert np.isclose(lo, -force_limit)


def test_bam_zeroes_the_driven_joint_damping_and_friction():
    """BAM rewrites damping + frictionloss every step; leaving the XML values in
    would double-count the friction budget training never had."""
    sim = MicroduckSim(use_bam=True)
    for adr in sim._joint_dof_adr:
        assert sim.model.dof_damping[adr] == 0.0
        assert sim.model.dof_frictionloss[adr] == 0.0


def test_plain_xml_path_uses_the_declared_position_actuators():
    sim = MicroduckSim(use_bam=False)
    assert sim.model.nu == contract.N_ACTIONS
    assert tuple(sim.actuator_names) == contract.JOINT_NAMES


def test_actuator_order_is_the_contract_joint_order():
    sim = MicroduckSim(use_bam=True)
    assert tuple(sim.actuator_names) == contract.JOINT_NAMES


def test_observation_layout_at_reset():
    sim = MicroduckSim(command=contract.build_command(twist=(0.4, 0.0, 0.0)))
    obs = sim.observation()
    assert obs.shape == (contract.N_OBS,)
    # At rest: no rotation, gravity straight down in the body frame,
    # joints exactly at the default pose, no velocity, no previous action.
    assert np.allclose(obs[contract.OBS_BASE_ANG_VEL], 0.0, atol=1e-9)
    assert np.allclose(obs[contract.OBS_PROJECTED_GRAVITY], [0.0, 0.0, -1.0], atol=1e-9)
    assert np.allclose(obs[contract.OBS_JOINT_POS], 0.0, atol=1e-9)
    assert np.allclose(obs[contract.OBS_JOINT_VEL], 0.0, atol=1e-9)
    assert np.allclose(obs[contract.OBS_LAST_ACTION], 0.0, atol=1e-9)
    assert np.allclose(obs[contract.OBS_TWIST_CMD], [0.4, 0.0, 0.0])
    assert np.allclose(obs[contract.OBS_HEAD_CMD], 0.0)
    assert np.allclose(obs[contract.OBS_BODY_CMD], 0.0)


def test_base_ang_vel_source_matches_the_imu_gyro():
    """``base_ang_vel`` is read from ``qvel[3:6]``; the ``imu_ang_vel`` gyro is
    the same quantity after ``mj_forward``. If this ever diverges, the policy is
    being fed a different frame than training's ``root_link_ang_vel_b``."""
    import mujoco

    sim = MicroduckSim(use_bam=True)
    sim.step(np.full(contract.N_ACTIONS, 0.2))
    mujoco.mj_forward(sim.model, sim.data)
    gyro = sim.data.sensordata[sim._gyro_adr : sim._gyro_adr + 3]
    assert np.allclose(sim.data.qvel[3:6], gyro, atol=1e-9)


def test_last_action_slot_tracks_the_applied_action():
    sim = MicroduckSim(use_bam=True)
    action = np.linspace(-0.1, 0.1, contract.N_ACTIONS)
    sim.step(action)
    assert np.allclose(sim.observation()[contract.OBS_LAST_ACTION], action)


def test_zero_action_hold_drifts_so_the_policy_does_the_balancing():
    """A rigid hold of the home pose is NOT a stable equilibrium — the policy is.

    Locking this in matters: the home pose puts the CoM ~5 mm ahead of the ankle
    axis, so a position target alone tips forward within a couple of seconds —
    with or without BAM, and independent of the perturbation magnitude.
    Anyone who "fixes" this by stiffening the hold has changed the physics the
    policy was trained against. The companion test — the idle policy holding the
    stand from a perturbed spawn — lives in tests/test_kernel_parity.py.
    """
    sim = MicroduckSim(use_bam=True)
    assert sim.command.tolist() == [0.0] * 13
    zeros = np.zeros(contract.N_ACTIONS)
    for _ in range(150):  # 3 s at 50 Hz of zero action == "hold the home pose"
        sim.step(zeros)
    assert sim.trunk_tilt > np.radians(10.0), (
        "an unforced home-pose hold now self-stabilizes; the sim no longer matches the trained physics"
    )


def test_physics_preset_builds_the_walk_model():
    import mujoco

    model = microduck_preset({})
    assert model.xml_string
    assert "assets/" in next(iter(k for k in model.assets if k.endswith(".stl")))
    compiled = mujoco.MjModel.from_xml_string(model.xml_string, assets=model.assets)
    assert compiled.nu == contract.N_ACTIONS
    assert compiled.nq == 7 + contract.N_JOINTS


def test_scene_assets_are_keyed_by_scene_relative_path():
    assets = load_scene_assets()
    assert "robot_walk.xml" in assets
    assert f"assets/{next(iter(p.name for p in MESH_DIR.iterdir() if p.suffix == '.stl'))}" in assets


def test_scene_file_is_vendored_verbatim():
    """The scene we replay on must be the training asset, unmodified."""
    assert SCENE_WALK.exists()
    text = SCENE_WALK.read_text()
    assert '<include file="robot_walk.xml" />' in text
    assert 'meshdir="assets"' in (SCENE_WALK.parent / "robot_walk.xml").read_text()


def test_gravity_projection_uses_the_trunk_frame():
    sim = MicroduckSim(use_bam=True)
    sim.data.qpos[sim._free_qpos_adr + 3 : sim._free_qpos_adr + 7] = [0.7071, 0.0, 0.0, 0.7071]
    sim.data.qpos[sim._free_qpos_adr + 2] = 0.2
    import mujoco

    mujoco.mj_forward(sim.model, sim.data)
    gravity = sim.observation()[contract.OBS_PROJECTED_GRAVITY]
    assert np.allclose(gravity, quat_rotate_inverse([0.7071, 0.0, 0.0, 0.7071], [0.0, 0.0, -1.0]), atol=1e-6)
