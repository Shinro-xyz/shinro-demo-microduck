"""The trajectory layer as shinro components: registered, strict, pluggable.

The demo's reference path and path tracker are not demo-local helpers — they
subclass shinro's ``TrajectoryGenerator`` / ``Controller`` ABCs, declare frozen
``Config`` dataclasses as their TOML schema, and register themselves so the
framework's own factories can build them. These tests hold that contract, because
it is the thing that makes them reusable: rename a field and a scenario silently
stops parsing, forget the decorator and ``type = "..."`` stops resolving.
"""

import subprocess
import sys
import textwrap

import numpy as np
import pytest

from shinro_demo_microduck import tracker as _tracker  # noqa: F401  (registration side effect)
from shinro_demo_microduck.tracker import PurePursuitConfig, PurePursuitTracker
from shinro_demo_microduck.trajectory import (
    MicroduckLoop,
    MicroduckLoopConfig,
    ReferencePath,
    load_loop_config,
    load_trajectory,
    trajectory_config_path,
)

PRESETS = ("circle", "figure_eight", "straight", "slalom", "waypoints_example")


# ─── registration ───────────────────────────────────────────────────────────


def test_both_components_are_registered_with_shinro():
    """Importing this package's modules is what puts them in the registries."""
    from shinro.factories.registry import _CONTROLLER_REGISTRY, _TRAJECTORY_REGISTRY

    assert _TRAJECTORY_REGISTRY["microduck_loop"] is MicroduckLoop
    assert _CONTROLLER_REGISTRY["microduck_pure_pursuit"] is PurePursuitTracker
    # and the decorator stamped the name the strict parser validates `type` against
    assert MicroduckLoop._registry_name == "microduck_loop"
    assert PurePursuitTracker._registry_name == "microduck_pure_pursuit"


def test_registration_is_an_import_side_effect_not_a_package_import():
    """`import shinro_demo_microduck` must stay light, so the components register
    when their *modules* are imported — the contract a scenario meets with
    ``--import shinro_demo_microduck.tracker``."""
    code = textwrap.dedent(
        """
        import sys
        from shinro.factories.registry import _CONTROLLER_REGISTRY, _TRAJECTORY_REGISTRY

        assert "microduck_loop" not in _TRAJECTORY_REGISTRY
        import shinro_demo_microduck          # package import alone: still unregistered
        assert "microduck_loop" not in _TRAJECTORY_REGISTRY
        import shinro_demo_microduck.tracker  # <-- what --import does
        assert "microduck_loop" in _TRAJECTORY_REGISTRY
        assert "microduck_pure_pursuit" in _CONTROLLER_REGISTRY
        print("OK")
        """
    )
    proc = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=False)
    assert proc.returncode == 0, proc.stderr[-2000:]
    assert "OK" in proc.stdout


def test_importing_the_tracker_does_not_pull_the_simulation_stack():
    """A component module is framework code, not robot code: it must not import
    MuJoCo. (MicroduckPolicy stays the only thing that ships with zero imports.)"""
    code = textwrap.dedent(
        """
        import sys
        import shinro_demo_microduck.tracker
        assert "mujoco" not in sys.modules, "importing a component pulled MuJoCo in"
        print("OK")
        """
    )
    proc = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=False)
    assert proc.returncode == 0, proc.stderr[-2000:]


# ─── the framework's own construction path ──────────────────────────────────


@pytest.mark.parametrize("name", PRESETS)
def test_trajectory_factory_builds_the_registered_generator(name):
    from shinro.factories.trajectory_factory import TrajectoryFactory

    schedule = TrajectoryFactory(str(trajectory_config_path(name))).create()
    assert isinstance(schedule, np.ndarray)
    assert schedule.ndim == 2 and schedule.shape[1] == 2
    assert schedule.shape[0] >= 2
    # the schedule the factory emits is the same path load_trajectory wraps
    reference = load_trajectory(name)
    assert np.allclose(schedule, reference.points, atol=1e-9)


def test_controller_factory_builds_the_registered_tracker():
    from shinro.factories.controller_factory import ControllerFactory

    controller = ControllerFactory("configs/controllers/pure_pursuit.toml").create()
    assert isinstance(controller, PurePursuitTracker)
    assert controller.cfg.v_cmd == pytest.approx(0.35)
    # the config the factory parsed is the one the tracker will use
    assert controller.cfg.v_min_cmd == pytest.approx(0.30)


def test_controller_factory_accepts_an_inline_config_dict():
    """Inline config is how a scenario or an MCP call would pass it."""
    from shinro.factories.controller_factory import ControllerFactory

    controller = ControllerFactory(config={"type": "microduck_pure_pursuit", "lookahead": 0.5}).create()
    assert controller.cfg.lookahead == pytest.approx(0.5)
    assert controller.cfg.k_heading == pytest.approx(1.5), "unset fields keep the measured defaults"


# ─── the TrajectoryGenerator contract ───────────────────────────────────────


def test_loop_from_config_returns_a_schedule_and_the_instance_agrees():
    cfg = load_loop_config("circle")
    schedule = MicroduckLoop.from_config(cfg)
    loop = MicroduckLoop(cfg)
    assert schedule.shape == (int(round(loop.T / cfg.dt)) + 1, 2)
    # sampling the instance at the same times reproduces the schedule
    times = np.linspace(0.0, loop.T, len(schedule), endpoint=False)
    assert np.allclose(np.stack([loop.position_at(t)[0] for t in times]), schedule, atol=1e-9)


def test_position_at_is_periodic_for_a_closed_loop_and_clamped_for_an_open_one():
    loop = MicroduckLoop(load_loop_config("figure_eight"))
    assert np.allclose(loop.position_at(0.0)[0], loop.position_at(loop.T)[0], atol=1e-9)
    assert np.allclose(loop.position_at(1.3)[0], loop.position_at(1.3 + loop.T)[0], atol=1e-9)

    goal = MicroduckLoop(load_loop_config("straight"))
    assert np.allclose(goal.position_at(1e6)[0], goal.path.points[-1], atol=1e-9)


def test_position_at_velocity_matches_the_speed_and_the_tangent():
    loop = MicroduckLoop(load_loop_config("circle"))
    _, velocity, acceleration = loop.position_at(loop.T / 8)
    assert np.linalg.norm(velocity) == pytest.approx(loop.cfg.speed, rel=1e-6)
    assert np.allclose(acceleration, 0.0)


def test_generate_is_a_documented_no_op_for_a_loop():
    """The ABC models a start->end motion; a loop has neither, and says so
    rather than silently mis-using the caller's arguments."""
    loop = MicroduckLoop(load_loop_config("circle"))
    returned = loop.generate(start_position=np.array([9.0, 9.0]), end_position=np.array([-9.0, -9.0]), duration=1.0)
    assert returned is loop
    assert np.allclose(loop.path.points[0], [0.0, 0.0], atol=1e-6), "the loop ignored the caller's start"


# ─── strict config parsing ──────────────────────────────────────────────────


def test_unknown_config_keys_are_a_loud_error():
    from shinro_demo_microduck.trajectory import MicroduckLoopConfig

    with pytest.raises(ValueError, match="unknown key"):
        MicroduckLoop.parse_config({"type": "microduck_loop", "radius": 1.0, "raius": 2.0})
    with pytest.raises(ValueError, match="unknown key"):
        PurePursuitTracker.parse_config({"v_cmd": 0.35, "lookahed": 0.4})
    assert MicroduckLoopConfig is MicroduckLoop.Config


def test_wrong_type_key_is_rejected():
    with pytest.raises(ValueError, match="wrong file"):
        MicroduckLoop.parse_config({"type": "lissajous", "shape": "circle"})


def test_shape_and_waypoints_are_mutually_exclusive_and_shape_is_validated():
    with pytest.raises(ValueError, match="shape must be one of"):
        MicroduckLoopConfig(shape="spiral")
    with pytest.raises(ValueError, match="needs at least 2 waypoints"):
        MicroduckLoopConfig(shape="waypoints")
    with pytest.raises(ValueError, match="takes no explicit waypoints"):
        MicroduckLoopConfig(shape="circle", waypoints=[[0.0, 0.0], [1.0, 1.0]])
    with pytest.raises(ValueError, match="dt must be positive"):
        MicroduckLoopConfig(dt=0.0)
    with pytest.raises(ValueError, match="speed must be positive"):
        MicroduckLoopConfig(speed=-1.0)


def test_parse_config_accepts_a_dict_a_path_or_an_instance():
    as_dict = MicroduckLoop.parse_config({"type": "microduck_loop", "shape": "circle", "radius": 2.0})
    assert isinstance(as_dict, MicroduckLoopConfig)
    assert as_dict.radius == pytest.approx(2.0)
    assert MicroduckLoop.parse_config(as_dict) is as_dict
    as_path = MicroduckLoop.parse_config(str(trajectory_config_path("circle")))
    assert as_path.shape == "circle"


# ─── the Controller contract ────────────────────────────────────────────────


def test_compute_is_the_abc_entry_point_and_matches_command():
    tracker = PurePursuitTracker(PurePursuitConfig(), path=load_trajectory("circle"))
    state = np.array([0.3, 0.1, 0.05, 0.2])
    assert np.allclose(tracker.compute(state), tracker.command(state[0:2], 0.05, 0.2))


def test_compute_requires_a_reference():
    tracker = PurePursuitTracker(PurePursuitConfig())
    with pytest.raises(ValueError, match="set_reference"):
        tracker.compute([0.0, 0.0, 0.0, 0.0])


def test_reset_clears_the_run_state():
    tracker = PurePursuitTracker(PurePursuitConfig(), path=load_trajectory("circle"))
    tracker.command([0.0, 0.0], 0.0, 0.0)
    assert tracker.cross_track < float("inf")
    tracker.reset()
    assert tracker.cross_track == float("inf")
    assert not tracker.reached_goal


def test_target_state_is_accepted_and_documented_as_unused():
    """The ABC passes a reference through; here the reference is the path."""
    tracker = PurePursuitTracker(PurePursuitConfig(), path=load_trajectory("circle"))
    assert np.allclose(tracker.compute([0.1, 0.0, 0.0, 0.0], target_state=np.zeros(3)), tracker.compute([0.1, 0.0, 0.0, 0.0]))


# ─── pluggability: shinro's own generators drive the microduck tracker ──────


def test_a_shinro_builtin_trajectory_plugs_into_the_microduck_tracker():
    """The point of the ABC: `microduck_pure_pursuit` never sees a Microduck
    trajectory — it sees a ``(steps, 2)`` schedule, so any registered generator
    works. Here: shinro's own Lissajous figure."""
    from shinro.factories.registry import _TRAJECTORY_REGISTRY

    schedule = _TRAJECTORY_REGISTRY["lissajous"].from_config(
        {"type": "lissajous", "dt": 0.02, "start": [1.0, 0.5], "end": [-1.0, -0.5], "duration": 6.0, "k": [1.0, 1.0]}
    )
    assert schedule.shape[1] == 2
    reference = ReferencePath(schedule, closed=False)
    tracker = PurePursuitTracker(PurePursuitConfig(), path=reference)
    assert reference.length > 1.0

    # and it actually tracks it: drive the real plant around part of that figure
    from shinro_demo_microduck.host import KernelRunner, manifest_for
    from shinro_demo_microduck.paths import CONTROLLER_CONFIG, DEFAULT_ARTIFACT
    from shinro_demo_microduck.sim import MicroduckSim

    if manifest_for(DEFAULT_ARTIFACT) is None:
        pytest.skip("no compiled kernel — run `make compile`")
    sim = MicroduckSim(use_bam=True)
    origin, yaw = reference.start_pose()
    sim.place(float(origin[0]), float(origin[1]), float(yaw))
    runner = KernelRunner(CONTROLLER_CONFIG, DEFAULT_ARTIFACT, compare_eager=False)
    cross = []
    for _ in range(400):  # 8 s
        w, x, y, z = (float(v) for v in sim.trunk_quaternion)
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        sim.command[:] = tracker.command(sim.trunk_position[:2], yaw, sim.yaw_rate)
        sim.step(runner.act(sim.observation()))
        cross.append(tracker.cross_track)
    assert np.mean(cross) < 0.10, f"tracking a shinro built-in drifted to {np.mean(cross) * 1000:.0f} mm"
    assert np.degrees(sim.trunk_tilt) < 15.0
