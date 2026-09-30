"""Preset trajectories: path conditioning, follower limits, and actual tracking.

The follower is the layer a team writes, and it has two jobs that are easy to get
wrong on *this* policy: never ask for a speed inside the measured deadband (the
robot stands instead of crawling, so a "slow down to 0.05 m/s" command is really
a "stop" command), and never ask for turn-in-place (pure yaw produces ~0.02 rad/s
— the policy simply stands). Both are asserted here rather than assumed.

The closed-loop tests measure the real thing: cross-track error when the compiled
kernel walks a preset path.
"""

import numpy as np
import pytest

from shinro_demo_microduck import contract
from shinro_demo_microduck.host import KernelRunner, manifest_for
from shinro_demo_microduck.paths import CONTROLLER_CONFIG, DEFAULT_ARTIFACT
from shinro_demo_microduck.sim import MicroduckSim
from shinro_demo_microduck.tracker import PurePursuitConfig, PurePursuitTracker
from shinro_demo_microduck.trajectory import (
    POLICY_DEADBAND_CMD,
    PROGRESS_FRACTION,
    ReferencePath,
    circle,
    figure_eight,
    ground_speed_for_command,
    load_trajectory,
    progress_speed_for_command,
    resample,
    slalom,
    smooth,
    straight,
)

PRESETS = ("circle", "figure_eight", "straight", "slalom", "waypoints_example")


# ─── paths ──────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("name", PRESETS)
def test_every_preset_loads_and_starts_at_the_spawn_pose(name):
    traj = load_trajectory(name)
    assert traj.length > 0.5, f"{name} is too short to be interesting"
    # Every generated path starts at the origin heading +x, which is the pose the
    # sim spawns in — so the follower starts with zero initial error.
    origin, yaw = traj.start_pose()
    # The generators are anchored at the origin; conditioning a smoothed path can
    # pull the seam in slightly, so allow a small offset.
    assert np.linalg.norm(origin) < 0.1, f"{name} starts at {origin}"
    # The start heading is the path's own tangent at the first point (only the
    # origin-anchored generators happen to start pointing at +x)...
    d = traj.points[1] - traj.points[0]
    assert yaw == pytest.approx(float(np.arctan2(d[1], d[0]))), name
    # ...and spawning there puts the robot on the path with no initial error.
    assert traj.cross_track(origin) < 0.05, name


@pytest.mark.parametrize(
    ("name", "closed"),
    [("circle", True), ("figure_eight", True), ("straight", False), ("slalom", False), ("waypoints_example", True)],
)
def test_presets_declare_closedness(name, closed):
    """A closed path runs forever; an open one ends and the follower stands there."""
    assert load_trajectory(name).closed is closed


def test_closed_path_length_includes_the_closing_segment():
    assert load_trajectory("circle").length == pytest.approx(2 * np.pi, abs=0.01)
    open_straight = ReferencePath(straight(2.0), closed=False)
    closed_straight = ReferencePath(straight(2.0), closed=True)
    assert open_straight.length == pytest.approx(2.0, abs=1e-6)
    assert closed_straight.length == pytest.approx(4.0, abs=1e-6)


def test_unknown_trajectory_or_type_fails_loudly():
    with pytest.raises(FileNotFoundError, match="no trajectory"):
        load_trajectory("no_such_path")


def test_resample_densifies_and_preserves_the_line():
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    dense = resample(pts, spacing=0.1)
    assert len(dense) > 20
    assert np.allclose(dense[0], pts[0]) and np.allclose(dense[-1], pts[-1])
    # the resampled polyline stays on the original (axis-aligned) line
    on_horizontal = np.isclose(dense[:, 1], 0.0, atol=1e-9) & (dense[:, 0] <= 1.0 + 1e-9)
    on_vertical = np.isclose(dense[:, 0], 1.0, atol=1e-9) & (dense[:, 1] <= 1.0 + 1e-9)
    assert np.all(on_horizontal | on_vertical), "resampling left the polyline"


def test_smooth_rounds_a_sharp_corner():
    """The point of smoothing: a 90 deg corner is not trackable, so blunt it."""

    def max_turn(pts):
        d = np.diff(np.vstack([pts, pts[:1]]), axis=0)
        ang = np.arctan2(d[:, 1], d[:, 0])
        return np.degrees(np.max(np.abs((np.diff(ang) + np.pi) % (2 * np.pi) - np.pi)))

    corner = resample(np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]), spacing=0.05, closed=True)
    rounded = smooth(corner, window=7, iterations=2, closed=True)
    assert max_turn(corner) > 80.0
    assert max_turn(rounded) < 25.0


def test_follower_config_rejects_impossible_limits():
    with pytest.raises(ValueError, match="v_min_cmd"):
        PurePursuitConfig(v_cmd=0.2, v_min_cmd=0.3)
    with pytest.raises(ValueError, match="lookahead"):
        PurePursuitConfig(lookahead=0.0)


# ─── follower: the two deployment limits ────────────────────────────────────


def test_follower_never_commands_inside_the_deadband():
    """Commanded speed is either 0 (stand) or >= the deadband — never 'crawl'."""
    traj = ReferencePath(circle(1.0), closed=True)
    follower = PurePursuitTracker(PurePursuitConfig(), path=traj)
    rng = np.random.default_rng(0)
    for _ in range(500):
        xy = rng.uniform(-1.5, 1.5, 2)
        command = follower.command(xy, rng.uniform(-np.pi, np.pi), rng.uniform(-1.0, 1.0))
        vx = float(command[contract.COMMAND_BLOCK_SLICES["twist"]][0])
        assert vx == 0.0 or vx >= POLICY_DEADBAND_CMD - 1e-12, f"commanded {vx:.3f} is inside the deadband"
        assert abs(vx) <= 0.40 + 1e-12, "commanded outside the trained range"
        assert float(command[contract.COMMAND_BLOCK_SLICES["twist"]][1]) == 0.0, "the walker gets no lateral command here"


def test_follower_never_asks_for_turn_in_place():
    """A pure-yaw command makes this policy stand still, so the follower must
    always keep a forward component while it is steering."""
    traj = ReferencePath(circle(0.6), closed=True)
    follower = PurePursuitTracker(PurePursuitConfig(), path=traj)
    rng = np.random.default_rng(1)
    steering = 0
    for _ in range(500):
        command = follower.command(rng.uniform(-1.2, 1.2, 2), rng.uniform(-np.pi, np.pi), 0.0)
        cmd = command[contract.COMMAND_BLOCK_SLICES["twist"]]
        if abs(cmd[2]) > 1e-9:
            steering += 1
            assert abs(cmd[0]) >= POLICY_DEADBAND_CMD - 1e-12, "steering without a forward component"
    assert steering > 100, "the sweep never produced a steering command — test is not exercising it"


def test_follower_stops_at_the_end_of_an_open_path():
    traj = ReferencePath(straight(2.0), closed=False)
    follower = PurePursuitTracker(PurePursuitConfig(), path=traj)
    start = np.array([0.0, 0.0])
    assert abs(float(follower.command(start, 0.0, 0.0)[contract.COMMAND_BLOCK_SLICES["twist"]][0])) >= POLICY_DEADBAND_CMD
    assert not follower.reached_goal
    # standing inside the goal radius of the final point
    command = follower.command(np.array([1.95, 0.0]), 0.0, 0.0)
    assert follower.reached_goal
    assert np.allclose(command, 0.0), "should have commanded a full stand at the goal"


def test_follower_steers_toward_the_path_not_away():
    traj = ReferencePath(np.array([[0.0, 0.0], [10.0, 0.0]]), closed=False)
    follower = PurePursuitTracker(PurePursuitConfig(), path=traj)
    left = follower.command(np.array([1.0, 0.2]), 0.0, 0.0)[contract.COMMAND_BLOCK_SLICES["twist"]]
    right = follower.command(np.array([1.0, -0.2]), 0.0, 0.0)[contract.COMMAND_BLOCK_SLICES["twist"]]
    assert left[2] < 0.0 < right[2], f"steering signs wrong: left {left[2]:+.3f}, right {right[2]:+.3f}"


def test_yaw_rate_damping_opposes_rotation():
    traj = ReferencePath(straight(3.0), closed=False)
    follower = PurePursuitTracker(PurePursuitConfig(), path=traj)
    on_path = np.array([1.0, 0.0])
    still = float(follower.command(on_path, 0.0, 0.0)[contract.COMMAND_BLOCK_SLICES["twist"]][2])
    spinning = float(follower.command(on_path, 0.0, 1.0)[contract.COMMAND_BLOCK_SLICES["twist"]][2])
    assert spinning < still, "measured yaw rate must damp the yaw command"


def test_speed_models_are_monotonic_and_match_the_locked_points():
    """The models the demo uses to size a run — locked to the measurement."""
    assert ground_speed_for_command(0.30) == pytest.approx(0.158, abs=0.01)
    assert ground_speed_for_command(0.40) == pytest.approx(0.206, abs=0.01)
    assert ground_speed_for_command(0.35) < ground_speed_for_command(0.40)
    # Net along-path progress is a fraction of the instantaneous body speed, and
    # must be the smaller of the two or a "one lap" run falls short.
    assert progress_speed_for_command(0.35) == pytest.approx(0.109, abs=0.005)
    assert progress_speed_for_command(0.35) < ground_speed_for_command(0.35)
    assert 0.0 < PROGRESS_FRACTION < 1.0


def test_advance_counts_progress_not_wobble():
    """A backward jitter of the nearest path point contributes nothing."""
    traj = load_trajectory("straight")
    assert traj.advance(0.50, 0.62) == pytest.approx(0.12)
    assert traj.advance(0.50, 0.48) == 0.0, "backward jitter must not add distance"
    # On a closed path the coordinate wraps, and a step across the seam is progress.
    closed = load_trajectory("circle")
    assert closed.advance(closed.length - 0.05, 0.05) == pytest.approx(0.10, abs=1e-6)
    # A self-intersecting path can flip the nearest point between branches, which
    # would otherwise report metres of progress in one tick.
    assert closed.advance(0.5, closed.length / 2 + 0.5) == 0.0


def test_arc_position_is_monotonic_along_the_path():
    traj = load_trajectory("straight")
    xs = np.linspace(0.0, 2.0, 20)
    positions = [traj.arc_position(np.array([x, 0.0])) for x in xs]
    assert positions == sorted(positions)
    assert positions[0] == pytest.approx(0.0) and positions[-1] == pytest.approx(2.0, abs=0.02)


# ─── closed loop: the compiled kernel actually walks the path ───────────────


@pytest.fixture(scope="module")
def artifact() -> str:
    if manifest_for(DEFAULT_ARTIFACT) is None:
        pytest.skip(f"no compiled kernel at {DEFAULT_ARTIFACT} — run `make compile`")
    return str(DEFAULT_ARTIFACT)


def _walk(traj_name: str, artifact: str, steps: int):
    """Walk a preset path with the follower; return cross-track samples and metrics."""
    traj = load_trajectory(traj_name)
    follower = PurePursuitTracker(PurePursuitConfig(), path=traj)
    sim = MicroduckSim(use_bam=True)
    origin, yaw = traj.start_pose()
    sim.place(float(origin[0]), float(origin[1]), float(yaw))
    runner = KernelRunner(CONTROLLER_CONFIG, artifact, compare_eager=False)
    cross, visited = [], []
    for _ in range(steps):
        w, x, y, z = (float(v) for v in sim.trunk_quaternion)
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        sim.command[:] = follower.command(sim.trunk_position[:2], yaw, sim.yaw_rate)
        sim.step(runner.act(sim.observation()))
        cross.append(follower.cross_track)
        visited.append(sim.trunk_position[:2].copy())
    return np.array(cross), sim, np.array(visited)


def _coverage(traj, visited, limit: float) -> float:
    """Fraction of reference points the robot passed within ``limit`` of.

    Arc progress cannot answer "did it walk the whole path?" — nearest-point
    progress is confused by a self-intersecting path, and a lap count says
    nothing about a cut segment. Coverage can.
    """
    to_path = np.linalg.norm(traj.points[:, None, :] - visited[None, :, :], axis=2).min(axis=1)
    return float((to_path < limit).mean())


@pytest.mark.integration
@pytest.mark.kernel
def test_kernel_tracks_a_circle(artifact):
    """The headline number: a bounded cross-track error on a smooth closed path."""
    traj = load_trajectory("circle")
    cross, sim, visited = _walk("circle", artifact, steps=1400)  # 28 s ~ half a lap
    assert cross.mean() < 0.04, f"mean cross-track {cross.mean() * 1000:.0f} mm is not tracking"
    assert cross.max() < 0.10, f"max cross-track {cross.max() * 1000:.0f} mm — it left the path"
    assert np.degrees(sim.trunk_tilt) < 12.0, "it fell over instead of walking"
    # Half a lap should have covered roughly half the path, with no segment cut.
    coverage = _coverage(traj, visited, limit=0.05)
    assert 0.4 < coverage < 0.65, f"half a lap covered {coverage * 100:.0f}% of the path"


@pytest.mark.integration
@pytest.mark.kernel
def test_kernel_walks_straight_and_stops_at_the_goal(artifact):
    cross, sim, _visited = _walk("straight", artifact, steps=1100)  # 22 s > 2 m at 0.11 m/s net
    assert cross.mean() < 0.03
    # It should have arrived and stopped rather than walked past the end.
    assert np.linalg.norm(sim.trunk_position[:2] - np.array([2.0, 0.0])) < 0.30
    assert sim.trunk_linear_velocity[0] < 0.05, "did not stop at the end of the open path"


@pytest.mark.integration
def test_place_puts_the_robot_on_the_path_start():
    sim = MicroduckSim(use_bam=True)
    sim.place(0.4, -0.3, np.pi / 2)
    assert np.allclose(sim.trunk_position[:2], [0.4, -0.3], atol=1e-9)
    w, x, y, z = (float(v) for v in sim.trunk_quaternion)
    assert np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) == pytest.approx(np.pi / 2, abs=1e-6)
    assert np.allclose(sim.trunk_linear_velocity, 0.0, atol=1e-9)


def test_generators_produce_the_paths_their_names_promise():
    assert np.allclose(circle(1.0).max(axis=0) - circle(1.0).min(axis=0), [2.0, 2.0], atol=1e-6)
    eight = figure_eight(1.2)
    assert eight[:, 1].max() > 0 and eight[:, 1].min() < 0
    assert np.allclose(slalom(0.2, 1.0, 2.0)[:, 1].max(), 0.2, atol=0.01)
    assert np.allclose(straight(2.0)[:, 1], 0.0)
