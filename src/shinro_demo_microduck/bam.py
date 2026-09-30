"""The BAM M6 actuator the Microduck policies were trained against.

The MJCF in ``assets/`` declares placeholder ``<position>`` actuators (kp 50 /
10 / 0.52 ...). Training never used them: every policy in this family was
trained under **BAM** (``better-actuator-models``), a voltage-controlled XL330
model whose firmware position loop and load-dependent friction live in the
actuator, not in MuJoCo's ``position`` gain.

Running the policy against the XML's PD gains is a *different actuator model* —
the walk comes out visibly sluggish (~0.1 m/s for a 0.25 m/s command instead of
~0.25 m/s). So BAM is the default in :class:`~shinro_demo_microduck.sim.MicroduckSim`
and ``--no-bam`` is the documented fallback.

The constants below mirror ``_BAM_ACTUATOR_KWARGS`` in the training repo's
``robot/microduck_constants.py`` and the CPU rehearsal path in its
``scripts/infer_policy.py``. They are deliberately duplicated (not imported):
the training repo drags in mjlab/torch/warp, and a demo should not depend on it.
``tests/test_bam_mirror.py`` locks the values.
"""

from __future__ import annotations

import mujoco
import numpy as np
from bam.model import Model, load_model
from bam.mujoco import MujocoController

#: Motor + model id in the BAM database.
BAM_MOTOR_NAME = "xl330"
BAM_MODEL = "m6"

#: Firmware position-loop stiffness. microduck preserves the stock kp_fw=200.
BAM_KP_FW = 200.0

#: Battery voltage sampled per-environment at startup in training (DR range).
BAM_VIN_RANGE = (6.5, 8.2)
#: Load-dependent sag: V_drop = gain * sum(|tau|), with a hard floor.
BAM_VIN_DROP_GAIN_RANGE = (0.0, 0.2)
BAM_VIN_MIN = 6.0
#: Training runs WITHOUT the firmware current limiter (None == disabled).
BAM_MAX_CURRENT = None

#: Stiff joint-friction constraint, copied from ``bam.mjlab.BamActuator``
#: (``stiff_frictionloss=True`` in training): warp has no noslip solver, so BAM
#: stiffens frictionloss so a statically-held joint does not creep. Mirrored
#: here so the CPU replay applies the same friction budget as warp training did.
BAM_STIFF_SOLREF_FRICTION = (-5.0e4, -2.0e2)
BAM_STIFF_SOLIMP_FRICTION = (0.99, 0.9999, 0.001, 0.5, 2.0)

#: Deterministic mid-range values for the demo (training randomized these).
DEFAULT_VIN = 7.35
DEFAULT_VIN_DROP_GAIN = 0.1


def load_bam_model(kp_fw: float = BAM_KP_FW, vin: float = DEFAULT_VIN, max_current=BAM_MAX_CURRENT) -> Model:
    """Build the BAM M6 model + XL330 voltage-controlled actuator."""
    bam_model = load_model(motor_name=BAM_MOTOR_NAME, model=BAM_MODEL)
    bam_model.actuator.kp = kp_fw
    bam_model.actuator.vin = vin
    bam_model.actuator.max_current = max_current if (max_current and max_current > 0) else None
    return bam_model


def compile_bam_scene(xml_path: str, bam_model: Model, timestep: float) -> tuple[mujoco.MjModel, list[str]]:
    """Compile an MJCF scene with BAM actuators, mirroring ``bam.mjlab.BamActuator.edit_spec``.

    Every non-``passive_`` actuator becomes a torque motor bounded by the
    voltage-implied force limit (``vin * kt / R``); the driven joint loses its
    damping/frictionloss (BAM rewrites both every step) and gains the stiff
    friction constraint used in training.

    Args:
        xml_path: Scene MJCF (the robot is included from it).
        bam_model: The BAM model whose ``kt`` / ``R`` / ``vin`` set the limit.
        timestep: Physics timestep for the compiled model.

    Returns:
        ``(model, actuator_names)`` — the names are the BAM controller's ctrl order.
    """
    kt = bam_model.kt.value
    r = bam_model.R.value
    force_limit = bam_model.actuator.vin * kt / r

    spec = mujoco.MjSpec.from_file(str(xml_path))
    names: list[str] = []
    for act in spec.actuators:
        target = act.target
        target_name = target.name if hasattr(target, "name") else str(target)
        if target_name.startswith("passive_"):
            continue
        act.set_to_motor()
        act.forcelimited = True
        act.forcerange = (-force_limit, force_limit)
        act.ctrllimited = False
        act.gear = [1.0, 0, 0, 0, 0, 0]
        names.append(act.name)
        for joint in spec.joints:
            if joint.name == target_name:
                joint.damping = np.zeros((3, 1))  # MjsJoint expects a (3, 1) array
                joint.frictionloss = 0.0
                joint.solref_friction = BAM_STIFF_SOLREF_FRICTION
                joint.solimp_friction = BAM_STIFF_SOLIMP_FRICTION
                break

    model = spec.compile()
    model.opt.timestep = timestep
    return model, names


__all__ = [
    "BAM_KP_FW",
    "BAM_MAX_CURRENT",
    "BAM_MODEL",
    "BAM_MOTOR_NAME",
    "BAM_STIFF_SOLIMP_FRICTION",
    "BAM_STIFF_SOLREF_FRICTION",
    "BAM_VIN_DROP_GAIN_RANGE",
    "BAM_VIN_MIN",
    "BAM_VIN_RANGE",
    "DEFAULT_VIN",
    "DEFAULT_VIN_DROP_GAIN",
    "MujocoController",
    "compile_bam_scene",
    "load_bam_model",
]
