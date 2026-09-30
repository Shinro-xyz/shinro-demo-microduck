"""The Microduck policy contract: a 61-D observation in, a 14-D action out.

Every Microduck RL policy — walking, standing, trick — shares this layout so a
runtime can hot-swap ONNX files without touching the glue. It is the interface
between the simulator and the compiled kernel, and it is what the tests lock.

Observation, 61 D, in this exact order::

    [ 0: 3]  base_ang_vel        IMU gyro, base frame            (3)
    [ 3: 6]  projected_gravity   gravity direction, base frame   (3)
    [ 6:20]  joint_pos           current - default pose, rad     (14)
    [20:34]  joint_vel           rad/s                           (14)
    [34:48]  actions             the previous action             (14)
    [48:51]  twist command       [vx, vy, wz]                    (3)
    [51:55]  head_pose command   [neck_pitch, head_pitch,
                                  head_yaw, head_roll]           (4)
    [55:61]  body_pose command   [x, y, z, roll, pitch, yaw]     (6)

Action, 14 D, in servo order::

    q_target = default_joint_pos + action * action_scale (= 1.0)

A command slot an env does not use is ZERO-PADDED, never removed — the policy
family depends on the offsets being identical everywhere. For the walking
policy only ``twist`` is live; ``head_pose`` is a small secondary objective and
``body_pose`` is trained at weight 0 (dead weights kept alive so a later
curriculum can use them).

Everything here is derived from / checked against the ONNX file's own metadata
by :func:`verify_against_onnx`, so a mismatch fails loudly instead of silently
driving the robot with the wrong joint order.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

#: Servo order. Index == ctrl index == joint index on the walk model.
JOINT_NAMES: tuple[str, ...] = (
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "left_ankle",
    "neck_pitch",
    "head_pitch",
    "head_yaw",
    "head_roll",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle",
)

#: HOME frame (STAND2): legs flexed, trunk shifted ~5 mm forward so the CoM sits
#: over the ankle axis. Actions are offsets from this pose and joint_pos is
#: measured relative to it. Must equal the ONNX metadata's default_joint_pos.
DEFAULT_POSE: np.ndarray = np.array(
    [
        0.0,
        -0.08726646259971647,
        -0.457924,
        -0.004940,
        0.452984,
        0.3490658503988659,
        0.3490658503988659,
        0.0,
        0.0,
        0.0,
        0.08726646259971647,
        0.457924,
        0.004940,
        -0.452984,
    ],
    dtype=np.float64,
)

N_JOINTS = len(JOINT_NAMES)
N_ACTIONS = N_JOINTS
N_OBS = 61

#: Observation slices — the single source of truth for the layout above.
OBS_BASE_ANG_VEL = slice(0, 3)
OBS_PROJECTED_GRAVITY = slice(3, 6)
OBS_JOINT_POS = slice(6, 6 + N_JOINTS)
OBS_JOINT_VEL = slice(6 + N_JOINTS, 6 + 2 * N_JOINTS)
OBS_LAST_ACTION = slice(6 + 2 * N_JOINTS, 6 + 3 * N_JOINTS)
OBS_TWIST_CMD = slice(6 + 3 * N_JOINTS, 6 + 3 * N_JOINTS + 3)
OBS_HEAD_CMD = slice(OBS_TWIST_CMD.stop, OBS_TWIST_CMD.stop + 4)
OBS_BODY_CMD = slice(OBS_HEAD_CMD.stop, OBS_HEAD_CMD.stop + 6)

#: The command block in the order the ONNX metadata declares it.
COMMAND_NAMES: tuple[str, ...] = ("twist", "head_pose", "body_pose")

#: Where each command lands **inside the 61-D observation** (the slots the actor
#: reads). This is the mapping to index an observation with.
COMMAND_SLICES: dict[str, slice] = {
    "twist": OBS_TWIST_CMD,
    "head_pose": OBS_HEAD_CMD,
    "body_pose": OBS_BODY_CMD,
}

#: Where each command lands **inside the 13-D command block** that
#: :func:`build_command` produces and that the demos write into ``sim.command``.
#: Distinct from :data:`COMMAND_SLICES` — same commands, different container.
COMMAND_BLOCK_SLICES: dict[str, slice] = {
    "twist": slice(0, 3),
    "head_pose": slice(3, 7),
    "body_pose": slice(7, 13),
}
N_COMMAND = 13

#: Bounds the command channels were trained over (velocity recipe cfg). Used by
#: demos to pick sane references; not enforced by the kernel.
TWIST_LIMITS = {"vx": (-0.4, 0.4), "vy": (-0.3, 0.3), "wz": (-1.0, 1.0)}
HEAD_POSE_LIMITS = 0.6  # rad, per channel
BODY_POSE_LIMITS = {"xyz": 0.03, "rpy": 0.3}

ACTION_SCALE = 1.0


def build_command(
    twist: tuple[float, float, float] = (0.0, 0.0, 0.0),
    head_pose: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0),
    body_pose: tuple[float, float, float, float, float, float] = (0.0,) * 6,
) -> np.ndarray:
    """Assemble the 13-D command block exactly as the policy expects it."""
    if len(twist) != 3 or len(head_pose) != 4 or len(body_pose) != 6:
        raise ValueError("twist must be 3, head_pose 4, body_pose 6")
    return np.concatenate([twist, head_pose, body_pose]).astype(np.float64)


def quat_rotate_inverse(quat_wxyz: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """Rotate a world vector into the body frame given a ``(w, x, y, z)`` quat.

    This is how ``projected_gravity`` is built: rotate [0, 0, -1] by the inverse
    of the base orientation, i.e. express gravity in the base frame.
    """
    w, x, y, z = (float(v) for v in quat_wxyz)
    # Rotation matrix from the quaternion, then its transpose (inverse for a
    # unit quaternion) applied to the world vector.
    r = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )
    return r.T @ np.asarray(vec, dtype=np.float64)


def verify_against_onnx(model_path: str | Path) -> dict:
    """Check this contract against an ONNX file's exporter metadata.

    The exporter stamps ``joint_names`` / ``default_joint_pos`` /
    ``observation_names`` / ``command_names`` / ``action_scale`` on the graph.
    Reading them back is what makes the demo self-checking: a joint-order or
    default-pose drift is caught here instead of showing up as a policy that
    twitches on hardware.

    Args:
        model_path: Path to the ``.onnx`` policy.

    Returns:
        The raw metadata mapping (so callers can report provenance).

    Raises:
        ValueError: If a name, pose, or dimension disagrees with this module.
    """
    import onnx

    meta = {p.key: p.value for p in onnx.load(str(model_path)).metadata_props}

    declared_joints = meta["joint_names"].split(",")
    if tuple(declared_joints) != JOINT_NAMES:
        raise ValueError(f"joint order mismatch:\n  ONNX: {declared_joints}\n  code: {list(JOINT_NAMES)}")

    declared_pose = np.array([float(v) for v in meta["default_joint_pos"].split(",")])
    # The exporter stamps the pose with %.3f, so 1e-3 is the tightest honest
    # tolerance: the code keeps the full-precision HOME frame the policy was
    # actually trained with, and the stamp is a lossy record of it.
    if declared_pose.shape != DEFAULT_POSE.shape or not np.allclose(declared_pose, DEFAULT_POSE, atol=1e-3):
        raise ValueError(f"default pose mismatch:\n  ONNX: {declared_pose}\n  code: {DEFAULT_POSE}")

    declared_commands = tuple(meta["command_names"].split(","))
    if declared_commands != COMMAND_NAMES:
        raise ValueError(f"command block mismatch: ONNX {declared_commands} vs code {COMMAND_NAMES}")

    declared_obs = tuple(meta["observation_names"].split(","))
    expected_terms = (
        "base_ang_vel",
        "projected_gravity",
        "joint_pos",
        "joint_vel",
        "actions",
        "command",
        "head_command",
        "body_command",
    )
    if declared_obs != expected_terms:
        raise ValueError(f"observation term mismatch: {declared_obs}")

    if float(meta["action_scale"]) != ACTION_SCALE:
        raise ValueError(f"action_scale mismatch: ONNX {meta['action_scale']} vs code {ACTION_SCALE}")

    return meta
