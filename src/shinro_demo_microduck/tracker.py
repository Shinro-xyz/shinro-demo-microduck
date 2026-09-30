"""The Microduck path tracker, as a registered shinro controller.

This is a shinro **component**: it subclasses :class:`shinro.components.Controller`,
declares a frozen ``Config`` dataclass as its TOML schema, and registers itself
with ``@register_controller`` so a scenario can name it::

    [controller]
    type = "microduck_pure_pursuit"
    config = "configs/controllers/pure_pursuit.toml"

and build it through the framework's own factory::

    tracker = ControllerFactory("configs/controllers/pure_pursuit.toml").create()
    tracker.set_reference(path)
    twist = tracker.compute([x, y, yaw, yaw_rate])   # 13-D Microduck command block

**Why a controller and not a trajectory.** The trained policy *is* the
controller for the joints; this class produces the command that policy tracks, so
it is the layer between navigation and the policy — state in, command out. That is
exactly shinro's ``Controller`` role. It never emits joint actions and never
touches the plant.

**Why closed loop.** An open-loop schedule of ``(vx, wz)`` over time would be the
obvious "preset trajectory", and it does not work on this checkpoint: its yaw
response is **non-monotonic in the command** (measured, 16 s runs, BAM):

    wz command   +0.2   +0.4   +0.6   +0.8   +1.0
    yaw achieved +0.32  +0.54  +0.03  +0.29  +0.65   rad/s

Asking for 0.6 rad/s of turn gets you almost none while 0.4 gets you 0.54. The
signs are right and the robot never falls, but no open-loop heading profile
survives that. Measuring the heading error every tick and damping on the measured
yaw rate absorbs it: 13 mm mean / 25 mm max cross-track error on a 1 m circle.

The tracker also respects the policy's **deadband** — while tracking it never
commands a forward speed inside the unresponsive band, so the robot either walks
or stands, never crawls — and never asks for turn-in-place, which this policy
cannot do (pure yaw produces ~0.02 rad/s).
"""

from __future__ import annotations

import math
import tomllib
from dataclasses import dataclass

import numpy as np
from shinro.components import Controller
from shinro.factories.registry import register_controller

from shinro_demo_microduck import contract
from shinro_demo_microduck.paths import resolve_repo_path
from shinro_demo_microduck.trajectory import POLICY_DEADBAND_CMD, POLICY_VX_MAX_CMD, ReferencePath


@dataclass(frozen=True)
class PurePursuitConfig:
    """Strict TOML schema for the ``microduck_pure_pursuit`` tracker.

    Field docs are in :class:`PurePursuitTracker`; the defaults are the best row
    of a sweep scored across three paths at once (a 1 m circle, a smoothed
    5-waypoint loop, and a figure-eight) — a gain set tuned on the circle alone
    cuts the waypoint corners badly. Mean / max cross-track at these values:

        circle    8.6 / 19.4 mm    waypoints 14.0 / 52.8 mm    figure_eight 10.5 / 30.4 mm

    ``tests/test_trajectory.py`` re-measures the closed loop and locks them.

    Fields:
        v_cmd: Forward command while tracking, inside ``[v_min_cmd, v_max_cmd]``.
        lookahead: Lookahead distance along the reference, metres.
        k_heading: Proportional heading gain (rad/s per rad of heading error).
        k_yaw_rate: Damping gain on the *measured* yaw rate.
        w_max: Yaw-rate command ceiling (rad/s).
        v_min_cmd: Never command forward speed inside this band (the deadband).
        v_max_cmd: Ceiling of the trained forward-command range.
        slow_cos: Floor of the ``cos(heading error)`` speed reduction into a turn.
        goal_radius: Stop radius at the end of an open reference.
        name: Registered controller name (validated against the ``type`` key).
    """

    v_cmd: float = 0.35
    lookahead: float = 0.25
    k_heading: float = 1.5
    k_yaw_rate: float = 0.5
    w_max: float = 1.0
    v_min_cmd: float = POLICY_DEADBAND_CMD
    v_max_cmd: float = POLICY_VX_MAX_CMD
    slow_cos: float = 0.35
    goal_radius: float = 0.15
    name: str = "microduck_pure_pursuit"

    def __post_init__(self) -> None:
        if not 0.0 < self.v_min_cmd <= self.v_cmd <= self.v_max_cmd:
            raise ValueError(
                f"microduck_pure_pursuit: need 0 < v_min_cmd <= v_cmd <= v_max_cmd, got {self.v_min_cmd}/{self.v_cmd}/{self.v_max_cmd}"
            )
        if self.lookahead <= 0.0:
            raise ValueError(f"microduck_pure_pursuit: lookahead must be positive, got {self.lookahead}")


@register_controller("microduck_pure_pursuit")
class PurePursuitTracker(Controller):
    """Drive the walking policy along a reference path with a lookahead target.

    Each tick: find the nearest reference point, walk forward until the lookahead
    distance is reached, and steer toward that target with proportional heading
    feedback plus damping on the measured yaw rate.

    Args:
        cfg: A :class:`PurePursuitConfig`.
        path: Optional reference path; can also arrive later via
            :meth:`set_reference`.
        backend: Array backend (kept for the framework signature; this tracker is
            numpy-only).
    """

    Config = PurePursuitConfig

    def __init__(self, cfg: PurePursuitConfig, path: ReferencePath | None = None, backend=None) -> None:
        self.cfg = cfg
        self.bk = backend
        self.path = path
        self.cross_track = float("inf")
        self.reached_goal = False

    def set_reference(self, path: ReferencePath) -> None:
        """Attach the reference path to track."""
        self.path = path
        self.reset()

    def reset(self) -> None:
        """Clear per-run state."""
        self.cross_track = float("inf")
        self.reached_goal = False

    # -- Controller contract -------------------------------------------------

    def compute(self, state, target_state=None) -> np.ndarray:
        """The 13-D command block for the current state.

        Args:
            state: ``[x, y, yaw, yaw_rate]`` — trunk position, heading (rad) and
                measured yaw rate (rad/s).
            target_state: Unused, and deliberately so: the reference here is a
                geometric *path*, not a state-space target. The path arrives via
                :meth:`set_reference`; ``position_at(t)`` on the trajectory is the
                other supported way to drive this tracker.

        Returns:
            The 13-D Microduck command block (``[twist(3), head_pose(4), body_pose(6)]``).
        """
        state = np.asarray(state, dtype=np.float64).ravel()
        return self.command(state[0:2], float(state[2]), float(state[3]))

    # -- the control law -----------------------------------------------------

    def command(self, position_xy, yaw: float, yaw_rate: float) -> np.ndarray:
        """The 13-D command block for an explicit pose (the same law as ``compute``)."""
        if self.path is None:
            raise ValueError("no reference path — call set_reference() first")
        xy = np.asarray(position_xy, dtype=np.float64)[:2]
        self.cross_track = self.path.cross_track(xy)

        if not self.path.closed and np.linalg.norm(xy - self.path.points[-1]) <= self.cfg.goal_radius:
            self.reached_goal = True
        if self.reached_goal:
            return contract.build_command()  # stand

        target = self.target(xy)
        heading_des = math.atan2(target[1] - xy[1], target[0] - xy[0])
        heading_error = (heading_des - yaw + math.pi) % (2 * math.pi) - math.pi

        wz = float(
            np.clip(
                self.cfg.k_heading * heading_error - self.cfg.k_yaw_rate * yaw_rate,
                -self.cfg.w_max,
                self.cfg.w_max,
            )
        )
        # Slow into a turn, but never into the policy's deadband: the robot either
        # walks or stands, it cannot crawl.
        vx = float(
            np.clip(
                self.cfg.v_cmd * max(self.cfg.slow_cos, math.cos(heading_error)),
                self.cfg.v_min_cmd,
                self.cfg.v_max_cmd,
            )
        )
        return contract.build_command(twist=(vx, 0.0, wz))

    def target(self, xy) -> np.ndarray:
        """The lookahead point the tracker steers toward."""
        if self.path is None:
            raise ValueError("no reference path — call set_reference() first")
        points, n = self.path.points, len(self.path.points)
        index = self.path.nearest_index(xy)
        current = index
        for _ in range(n):
            nxt = (current + 1) % n
            if np.linalg.norm(points[nxt] - xy) >= self.cfg.lookahead:
                break
            if not self.path.closed and nxt == 0:
                break
            current = nxt
        return points[current % n]

    @classmethod
    def parse_config(cls, config) -> PurePursuitConfig:
        """Strict-parse a TOML dict, a config file path, or a Config instance."""
        if isinstance(config, str):
            with open(resolve_repo_path(config), "rb") as handle:
                config = tomllib.load(handle)
        return super().parse_config(config)

    @classmethod
    def from_config(cls, config, backend=None, path: ReferencePath | None = None) -> PurePursuitTracker:
        """Build the tracker from a TOML config dict, a config file path, or a Config.

        Args:
            config: TOML dict (with a ``type`` key), a config file path, or a
                :class:`PurePursuitConfig`.
            backend: Array backend, forwarded to the instance.
            path: Optional reference path to attach immediately.

        Returns:
            A :class:`PurePursuitTracker`.
        """
        return cls(cls.parse_config(config), path=path, backend=backend)


__all__ = ["PurePursuitConfig", "PurePursuitTracker"]
