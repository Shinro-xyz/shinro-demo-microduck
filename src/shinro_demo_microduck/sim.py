"""MuJoCo plant for the Microduck walk model + the 61-D observation builder.

The sim is the *host* side of the demo: it produces the observation the policy
consumes and applies the action the kernel returns. Nothing here is compiled —
the point of the demo is that the control law between those two calls is a Zig
``.so``, not Python.

Physics mirror training: 2 ms MuJoCo timestep x 10 substeps = 50 Hz control,
the rate every policy in this family was trained at. Actuation is BAM by
default (see :mod:`shinro_demo_microduck.bam`) because that is the actuator the
policy was trained against.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from shinro_demo_microduck import bam as _bam
from shinro_demo_microduck import contract
from shinro_demo_microduck.paths import SCENE_WALK

#: Control rate: 50 Hz, as in training (physics 500 Hz / decimation 10).
CONTROL_DT = 0.02
PHYSICS_DT = 0.002
N_SUBSTEPS = int(round(CONTROL_DT / PHYSICS_DT))

#: Spawn height for the trunk free joint. The scene's STAND keyframe uses 0.12;
#: the CPU rehearsal in the training repo spawns at 0.125 (a 5 mm offset the
#: policy absorbs). Kept configurable so a test can assert the settle is stable.
DEFAULT_SPAWN_HEIGHT = 0.125

_WORLD_GRAVITY = np.array([0.0, 0.0, -1.0])


class MicroduckSim:
    """The walk scene, the BAM actuator, and the 61-D observation contract.

    Args:
        command: The 13-D command block (see :func:`contract.build_command`).
            Defaults to the all-zero idle command, which is the deployment's
            rest state.
        use_bam: Use the BAM M6 voltage actuator (default). ``False`` falls back
            to the MJCF's placeholder ``<position>`` gains -- a documented
            actuator mismatch used only to compare the two.
        control_dt / physics_dt: Control period and MuJoCo timestep.
        key: Keyframe to reset to.
        spawn_height: Trunk free-joint height at reset.
        bam_vin / bam_vin_drop_gain: Battery voltage and sag gain. Training
            randomizes these per environment; the demo pins deterministic
            mid-range values.
    """

    def __init__(
        self,
        command: np.ndarray | None = None,
        *,
        use_bam: bool = True,
        control_dt: float = CONTROL_DT,
        physics_dt: float = PHYSICS_DT,
        key: str = "STAND",
        spawn_height: float | None = None,
        bam_vin: float = _bam.DEFAULT_VIN,
        bam_vin_drop_gain: float = _bam.DEFAULT_VIN_DROP_GAIN,
    ) -> None:
        self.control_dt = control_dt
        self.physics_dt = physics_dt
        self.n_substeps = max(1, int(round(control_dt / physics_dt)))
        self.use_bam = use_bam
        self.key = key
        self.spawn_height = DEFAULT_SPAWN_HEIGHT if spawn_height is None else spawn_height
        self.command = np.zeros(13) if command is None else np.asarray(command, dtype=np.float64).ravel()
        if self.command.size != 13:
            raise ValueError(f"command block must be 13-D, got {self.command.size}")

        self.bam_model = None
        self._bam_ctrl = None
        if use_bam:
            self.bam_model = _bam.load_bam_model(vin=bam_vin)
            self.model, self.actuator_names = _bam.compile_bam_scene(SCENE_WALK, self.bam_model, physics_dt)
            if tuple(self.actuator_names) != contract.JOINT_NAMES:
                raise ValueError(f"actuator order {self.actuator_names} != contract {contract.JOINT_NAMES}")
            self.data = mujoco.MjData(self.model)
            self._bam_ctrl = _bam.MujocoController(
                self.bam_model,
                self.actuator_names,
                self.model,
                self.data,
                vin_drop_gain=bam_vin_drop_gain,
                vin_min=_bam.BAM_VIN_MIN,
            )
        else:
            self.model = mujoco.MjModel.from_xml_path(str(SCENE_WALK))
            self.model.opt.timestep = physics_dt
            self.actuator_names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(self.model.nu)]
            self.data = mujoco.MjData(self.model)

        self.default_pose = contract.DEFAULT_POSE.copy()
        self.action_scale = contract.ACTION_SCALE

        joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in contract.JOINT_NAMES]
        self._joint_qpos_adr = np.array([self.model.jnt_qposadr[j] for j in joint_ids])
        self._joint_dof_adr = np.array([self.model.jnt_dofadr[j] for j in joint_ids])
        free_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "trunk_base_freejoint")
        self._free_qpos_adr = int(self.model.jnt_qposadr[free_id])
        gyro_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_ang_vel")
        self._gyro_adr = int(self.model.sensor_adr[gyro_id])

        self._last_action = np.zeros(contract.N_ACTIONS, dtype=np.float64)
        self.reset()

    # -- state ---------------------------------------------------------------

    def reset(self, key: str | None = None) -> None:
        """Reset to a keyframe + the default pose, zero velocity and action."""
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, key or self.key)
        mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        self.data.qpos[self._free_qpos_adr + 2] = self.spawn_height
        self.data.qpos[self._joint_qpos_adr] = self.default_pose
        self.data.qvel[:] = 0.0
        if self._bam_ctrl is not None:
            self._bam_ctrl.q_target[:] = self.default_pose
            self._bam_ctrl.reset(self.data.qpos)
        else:
            self.data.ctrl[:] = self.default_pose
        self._last_action[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def place(self, x: float, y: float, yaw: float = 0.0) -> None:
        """Teleport the trunk to ``(x, y, yaw)`` with the home pose and zero velocity.

        Used to spawn the robot *on* a reference trajectory rather than at the
        scene origin, so a follower starts with zero initial error.
        """
        half = 0.5 * yaw
        start = self._free_qpos_adr
        self.data.qpos[start : start + 7] = [x, y, self.spawn_height, math.cos(half), 0.0, 0.0, math.sin(half)]
        self.data.qpos[self._joint_qpos_adr] = self.default_pose
        self.data.qvel[:] = 0.0
        if self._bam_ctrl is not None:
            self._bam_ctrl.q_target[:] = self.default_pose
            self._bam_ctrl.reset(self.data.qpos)
        self._last_action[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def observation(self) -> np.ndarray:
        """Assemble the 61-D observation, in :mod:`contract` order.

        ``base_ang_vel`` is the free joint's rotational velocity: for a MuJoCo
        free joint ``qvel[3:6]`` is already in the *body* frame, which is what
        training's ``root_link_ang_vel_b`` is and what the ``imu_ang_vel`` gyro
        reads (they agree exactly after ``mj_forward``; the sensor buffer is one
        step stale inside a step, so reading ``qvel`` avoids the lag).
        """
        quat = self.data.qpos[self._free_qpos_adr + 3 : self._free_qpos_adr + 7]
        base_ang_vel = self.data.qvel[3:6].copy()
        projected_gravity = contract.quat_rotate_inverse(quat, _WORLD_GRAVITY)
        joint_pos = self.data.qpos[self._joint_qpos_adr] - self.default_pose
        joint_vel = self.data.qvel[self._joint_dof_adr]
        return np.concatenate([base_ang_vel, projected_gravity, joint_pos, joint_vel, self._last_action, self.command]).astype(np.float64)

    def step(self, action: np.ndarray) -> None:
        """Apply a 14-D action and advance one control period.

        The action is a position offset: ``q_target = default_pose + action``.
        Under BAM the target goes to the firmware loop, which writes a torque
        and a friction budget each physics step; without it the target is
        MuJoCo's ``ctrl``.
        """
        action = np.asarray(action, dtype=np.float64).ravel()
        if action.size != contract.N_ACTIONS:
            raise ValueError(f"action must be {contract.N_ACTIONS}-D, got {action.size}")
        target = self.default_pose + action * self.action_scale
        if self._bam_ctrl is not None:
            self._bam_ctrl.q_target[:] = target
            for _ in range(self.n_substeps):
                self._bam_ctrl.update()
                mujoco.mj_step(self.model, self.data)
        else:
            self.data.ctrl[:] = target
            for _ in range(self.n_substeps):
                mujoco.mj_step(self.model, self.data)
        self._last_action = action.copy()

    # -- readouts for plots / tests -----------------------------------------

    @property
    def trunk_position(self) -> np.ndarray:
        """Trunk position ``(x, y, z)`` in the world frame."""
        return self.data.qpos[self._free_qpos_adr : self._free_qpos_adr + 3].copy()

    @property
    def trunk_quaternion(self) -> np.ndarray:
        """Trunk orientation as a ``(w, x, y, z)`` quaternion."""
        return self.data.qpos[self._free_qpos_adr + 3 : self._free_qpos_adr + 7].copy()

    @property
    def trunk_linear_velocity(self) -> np.ndarray:
        """Trunk world-frame linear velocity ``(3,)`` (free joint ``qvel[0:3]``)."""
        return self.data.qvel[0:3].copy()

    @property
    def yaw_rate(self) -> float:
        """Trunk yaw rate about world up, in rad/s."""
        quat = self.trunk_quaternion
        w, x, y, z = (float(v) for v in quat)
        # World-frame angular velocity from the body-frame qvel[3:6].
        world = (
            np.array(
                [
                    [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
                ]
            )
            @ self.data.qvel[3:6]
        )
        return float(world[2])

    @property
    def trunk_tilt(self) -> float:
        """Angle between the trunk's up axis and world up, in radians."""
        return float(np.arccos(np.clip(-contract.quat_rotate_inverse(self.trunk_quaternion, _WORLD_GRAVITY)[2], -1, 1)))

    @property
    def joint_positions(self) -> np.ndarray:
        return self.data.qpos[self._joint_qpos_adr].copy()
