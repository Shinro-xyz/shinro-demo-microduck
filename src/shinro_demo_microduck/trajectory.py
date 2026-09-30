"""The Microduck reference path, as a registered shinro trajectory generator.

This is a shinro **component**, not a demo-local helper: it subclasses
:class:`shinro.components.TrajectoryGenerator`, declares a frozen ``Config``
dataclass as its TOML schema (strict-parsed by ``ConfigDriven``), and registers
itself with ``@register_trajectory`` so a scenario can name it::

    [trajectory]
    type = "microduck_loop"
    config = "configs/trajectories/circle.toml"

and build it through the framework's own factory::

    schedule = TrajectoryFactory("configs/trajectories/circle.toml").create()

``from_config`` follows the framework convention — it returns the sampled
``(steps, 2)`` reference schedule — while the instance keeps the richer
:class:`ReferencePath` the tracker needs (arc length, nearest point, progress).

**Why a new generator type.** shinro's existing trajectories are rest-to-rest
motions from one configuration to another over a fixed duration. A locomotion
policy wants the opposite: an *endless* reference that closes on itself, since
the robot has to keep walking. ``microduck_loop`` is that missing piece — and
because the tracker only ever sees a ``(steps, 2)`` schedule, shinro's own
generators (``circular_arc``, ``lissajous``, ``bspline``, …) plug in unchanged;
see ``tests/test_trajectory.py``.
"""

from __future__ import annotations

import math
import pathlib
import tomllib
from dataclasses import dataclass, field

import numpy as np
from shinro.components import TrajectoryGenerator
from shinro.factories.registry import register_trajectory

from shinro_demo_microduck.paths import resolve_repo_path

# ─── the measured policy response (see docs/trajectories.md) ───────────────

#: Forward command below which the policy stands still. Never emit inside this band.
POLICY_DEADBAND_CMD = 0.30
#: The walking policy was trained on vx in [-0.4, 0.4]; stay inside it.
POLICY_VX_MAX_CMD = 0.40


def ground_speed_for_command(v_cmd: float) -> float:
    """Instantaneous body speed the policy reaches for a forward command."""
    return 0.48 * v_cmd + 0.014


#: Fraction of the instantaneous body speed that becomes *net progress along a
#: path*. The trunk wobbles side to side every step (the gyro and gravity signals
#: oscillate with the gait), so the body covers ~0.17 m/s while advancing ~0.11 m/s
#: along a straight line (measured: 0.105-0.115 m/s on the circle, straight and
#: waypoint paths, against 0.182 m/s predicted from the instantaneous model).
#: Sizing a run must use the progress figure, or a "one lap" run stops two-thirds
#: of the way round.
PROGRESS_FRACTION = 0.60


def progress_speed_for_command(v_cmd: float) -> float:
    """Net along-path speed to expect from a forward command — use this for timing."""
    return ground_speed_for_command(v_cmd) * PROGRESS_FRACTION


# ─── shape helpers (authoring convenience for the `shape` config field) ─────


def circle(radius: float = 1.0, n: int = 400) -> np.ndarray:
    """Circle through the origin with tangent +x, so the robot starts on-path."""
    t = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    return np.stack([radius * np.sin(t), radius - radius * np.cos(t)], axis=1)


def figure_eight(scale: float = 1.2, n: int = 400) -> np.ndarray:
    """Gerono lemniscate through the origin with tangent +x."""
    t = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    return np.stack([scale * np.sin(t), scale * np.sin(2 * t) / 2.0], axis=1)


def straight(length: float = 2.0, n: int = 200) -> np.ndarray:
    """A straight line along +x from the origin (open path — stops at the end)."""
    return np.stack([np.linspace(0.0, length, n), np.zeros(n)], axis=1)


def slalom(amplitude: float = 0.35, wavelength: float = 1.6, length: float = 4.0, n: int = 400) -> np.ndarray:
    """A sine-wave corridor along +x (open path)."""
    x = np.linspace(0.0, length, n)
    return np.stack([x, amplitude * np.sin(2 * math.pi * x / wavelength)], axis=1)


SHAPES = {"circle": circle, "figure_eight": figure_eight, "straight": straight, "slalom": slalom}

#: Whether a shape loops. A closed reference runs forever; an open one ends and
#: the tracker commands stand there. Overridable per config.
CLOSED_BY_DEFAULT = {"circle": True, "figure_eight": True, "straight": False, "slalom": False, "waypoints": False}


# ─── path conditioning ──────────────────────────────────────────────────────


def resample(points: np.ndarray, spacing: float = 0.05, closed: bool = False) -> np.ndarray:
    """Resample a polyline at a fixed arc-length spacing.

    Sparse waypoints from a navigation stack are far apart (metres) while a
    tracker's lookahead is tens of centimetres; without densification the
    nearest-point search jumps between vertices and the corner cutting is
    arbitrary rather than chosen.
    """
    pts = np.asarray(points, dtype=np.float64)
    if closed:
        pts = np.vstack([pts, pts[:1]])
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 0.0:
        return pts
    n = max(2, int(round(s[-1] / spacing)) + 1)
    out = np.linspace(0.0, s[-1], n)
    return np.stack([np.interp(out, s, pts[:, 0]), np.interp(out, s, pts[:, 1])], axis=1)


def smooth(points: np.ndarray, window: int = 7, iterations: int = 2, closed: bool = False) -> np.ndarray:
    """Repeated box filter along a resampled path — rounds corners, keeps the line.

    A sharp waypoint corner (a 68 deg turn over 0 m of arc) is not trackable by a
    velocity-commanded walking policy: the tracker either cuts it or oscillates.
    Smoothing is what a navigation stack does before handing a path down, and it
    is applied here *before* the tracker sees the path, so the reference drawn in
    the demo is the path actually being tracked.
    """
    if window < 3 or iterations <= 0:
        return np.asarray(points, dtype=np.float64)
    width = window if window % 2 else window + 1
    kernel = np.ones(width) / width
    out = np.asarray(points, dtype=np.float64)
    for _ in range(iterations):
        if closed:
            padded = np.vstack([out[-(width // 2) :], out, out[: width // 2]])
        else:
            padded = np.vstack([out[:1]] * (width // 2) + [out] + [out[-1:]] * (width // 2))
        out = np.stack(
            [np.convolve(padded[:, 0], kernel, mode="valid"), np.convolve(padded[:, 1], kernel, mode="valid")],
            axis=1,
        )
    return out


class ReferencePath:
    """A conditioned reference path: points in the world x-y plane, maybe closed.

    The value object the tracker tracks and the metrics are measured against.
    Built from a ``(steps, 2)`` schedule — what a registered shinro trajectory
    generator emits.
    """

    __slots__ = ("points", "closed", "_arc", "_extended")

    def __init__(self, points: np.ndarray, closed: bool = True) -> None:
        self.points = np.asarray(points, dtype=np.float64)
        if self.points.ndim != 2 or self.points.shape[1] != 2 or len(self.points) < 2:
            raise ValueError(f"reference points must be an (N, 2) array with N >= 2, got {self.points.shape}")
        self.closed = bool(closed)
        self._arc: np.ndarray | None = None
        self._extended: np.ndarray | None = None

    @property
    def length(self) -> float:
        """Path length in metres (including the closing segment when closed)."""
        return float(self.arc_lengths[-1])

    @property
    def extended_points(self) -> np.ndarray:
        """The points with the closing vertex repeated, so arc length and x/y line up."""
        if self._extended is None:
            self._extended = np.vstack([self.points, self.points[:1]]) if self.closed else self.points
        return self._extended

    @property
    def arc_lengths(self) -> np.ndarray:
        """Cumulative arc length at each (extended) path point, lazily computed."""
        if self._arc is None:
            self._arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(self.extended_points, axis=0), axis=1))])
        return self._arc

    def start_pose(self) -> tuple[np.ndarray, float]:
        """Position and heading at the path start (where the robot should spawn)."""
        origin, nxt = self.points[0], self.points[1]
        return origin, math.atan2(nxt[1] - origin[1], nxt[0] - origin[0])

    def cross_track(self, xy: np.ndarray) -> float:
        """Perpendicular distance from ``xy`` to the nearest point on the path."""
        return float(np.min(np.linalg.norm(self.points - np.asarray(xy, dtype=np.float64)[:2], axis=1)))

    def nearest_index(self, xy: np.ndarray) -> int:
        """Index of the nearest path point to ``xy``."""
        return int(np.argmin(np.linalg.norm(self.points - np.asarray(xy, dtype=np.float64)[:2], axis=1)))

    def arc_position(self, xy: np.ndarray) -> float:
        """Arc-length coordinate of the path point nearest ``xy`` (metres)."""
        return float(self.arc_lengths[self.nearest_index(xy)])

    def advance(self, previous: float, current: float, max_step: float = 0.25) -> float:
        """Forward progress from one arc position to the next, wobble-tolerant.

        The trunk oscillates laterally every step, so the nearest path point can
        jump backwards by a centimetre or two; counting only the positive part
        makes this net progress along the path rather than sum-of-wobble — the
        distinction matters, because on a straight 2 m path the naive per-step
        distance reads 1.98 m while the robot has actually advanced 1.36 m.

        ``max_step`` rejects implausible jumps. The nearest path point moves in
        whole samples, so one accepted step can be as large as the path's
        sampling spacing (5 cm for the resampled waypoint paths) — but a path that
        crosses itself can flip between branches and report *metres* of progress in
        a single 50 Hz tick. Anywhere between the two works; this is at the top.
        """
        if self.closed:
            half = 0.5 * self.length
            delta = (current - previous + half) % self.length - half
        else:
            delta = current - previous
        if delta > max_step:
            return 0.0
        return max(0.0, delta)


# ─── the registered component ───────────────────────────────────────────────


@dataclass(frozen=True)
class MicroduckLoopConfig:
    """Strict TOML schema for the ``microduck_loop`` reference.

    Fields:
        dt: Sampling period (s) of the emitted ``(steps, 2)`` schedule. The
            schedule is the *spatial* reference in arc-length order; ``dt`` sets
            how finely it is sampled for a consumer that indexes it by step.
        speed: Reference speed (m/s) the loop is traversed at. Sets the horizon
            ``T = length / speed``; the tracker does not use it to walk (it has
            its own command), it is what makes ``position_at`` well defined.
        shape: Authoring convenience — build the waypoint list from a named shape
            (``circle``, ``figure_eight``, ``straight``, ``slalom``). Mutually
            exclusive with ``waypoints``; use ``waypoints`` for what a navigation
            stack actually emits.
        radius / scale / amplitude / wavelength / length / samples: Parameters of
            the named shapes. Only the ones the chosen ``shape`` reads are used.
        waypoints: Explicit ``[[x, y], ...]`` list, for ``shape = "waypoints"``.
        closed: Whether the reference loops. Defaults per shape (a circle loops,
            a straight line does not).
        smooth_spacing / smooth_window / smooth_iterations: Path conditioning
            applied before the tracker sees it — resample to ``smooth_spacing``
            metres, then a ``smooth_window``-metre box filter ``smooth_iterations``
            times. All zero (the default) leaves the shape as given.
        name: Registered trajectory name (validated against the ``type`` key).
    """

    dt: float = 0.02
    speed: float = 0.35
    shape: str = "circle"
    radius: float = 1.0
    scale: float = 1.2
    amplitude: float = 0.35
    wavelength: float = 1.6
    length: float = 2.0
    samples: int = 400
    waypoints: list[list[float]] = field(default_factory=list)
    closed: bool | None = None
    smooth_spacing: float = 0.0
    smooth_window: float = 0.0
    smooth_iterations: int = 0
    name: str = "microduck_loop"

    def __post_init__(self) -> None:
        if self.dt <= 0.0:
            raise ValueError(f"microduck_loop: dt must be positive, got {self.dt}")
        if self.speed <= 0.0:
            raise ValueError(f"microduck_loop: speed must be positive, got {self.speed}")
        if self.shape not in (*SHAPES, "waypoints"):
            raise ValueError(f"microduck_loop: shape must be one of {sorted((*SHAPES, 'waypoints'))}, got {self.shape!r}")
        if self.shape == "waypoints" and len(self.waypoints) < 2:
            raise ValueError("microduck_loop: shape 'waypoints' needs at least 2 waypoints")
        if self.shape != "waypoints" and self.waypoints:
            raise ValueError(f"microduck_loop: shape {self.shape!r} takes no explicit waypoints")

    @property
    def is_closed(self) -> bool:
        """Whether the reference loops, resolving the shape default."""
        return CLOSED_BY_DEFAULT[self.shape] if self.closed is None else self.closed


@register_trajectory("microduck_loop")
class MicroduckLoop(TrajectoryGenerator):
    """An endless reference loop for a locomotion policy.

    shinro's other generators interpolate a plant from one configuration to
    another and stop. A walking policy needs a reference it can follow
    indefinitely, so this one is closed by construction (or explicitly open, for
    a walk-to-a-goal reference) and parameterized by arc length rather than by a
    rest-to-rest time law::

        path = MicroduckLoop.from_config(cfg_dict)   # (steps, 2) schedule
        loop = MicroduckLoop(cfg)                    # rich instance
        loop.position_at(3.0)                        # (pos, vel, acc) on the loop

    Args:
        cfg: A :class:`MicroduckLoopConfig`.
        backend: Array backend (kept for the framework signature; this generator
            is numpy-only because the tracker consumes numpy).
    """

    Config = MicroduckLoopConfig

    def __init__(self, cfg: MicroduckLoopConfig, backend=None) -> None:
        self.cfg = cfg
        self.bk = backend
        points = self._build_points(cfg)
        self.path = ReferencePath(points, closed=cfg.is_closed)
        #: Horizon of one traversal; ``position_at`` is periodic with this period.
        self.T = self.path.length / cfg.speed

    @staticmethod
    def _build_points(cfg: MicroduckLoopConfig) -> np.ndarray:
        if cfg.shape == "waypoints":
            points = np.asarray(cfg.waypoints, dtype=np.float64)
        else:
            kwargs = {"n": cfg.samples}
            if cfg.shape == "circle":
                kwargs["radius"] = cfg.radius
            elif cfg.shape == "figure_eight":
                kwargs["scale"] = cfg.scale
            elif cfg.shape == "slalom":
                kwargs.update(amplitude=cfg.amplitude, wavelength=cfg.wavelength, length=cfg.length)
            elif cfg.shape == "straight":
                kwargs["length"] = cfg.length
            points = SHAPES[cfg.shape](**kwargs)

        if cfg.smooth_iterations > 0 and cfg.smooth_spacing > 0.0:
            spacing = cfg.smooth_spacing
            window = max(3, int(round(cfg.smooth_window / spacing)) | 1) if cfg.smooth_window > 0.0 else 0
            points = smooth(
                resample(points, spacing=spacing, closed=cfg.is_closed),
                window=window,
                iterations=cfg.smooth_iterations,
                closed=cfg.is_closed,
            )
        return points

    # -- TrajectoryGenerator contract ---------------------------------------

    def generate(self, start_position=None, end_position=None, duration=None, *args, **kwargs) -> MicroduckLoop:
        """No-op, deliberately: this reference is fully determined by its config.

        The ABC models a rest-to-rest motion from a start configuration to an end
        one over a duration. A closed loop has neither — the caller's start/end
        are ignored rather than silently mis-used, and the loop is returned so
        ``generate(...)`` chains like the other generators do.
        """
        return self

    def position_at(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Reference position, velocity and acceleration at time ``t``.

        Periodic in ``T = length / speed`` for a closed loop; clamped to the end
        of an open one.

        Returns:
            ``(position, velocity, acceleration)``, each shape ``(2,)``.
        """
        period = self.T
        if self.path.closed:
            t = t % period
        else:
            t = min(max(t, 0.0), period)
        s = t * self.cfg.speed
        points, arc = self.path.extended_points, self.path.arc_lengths
        xy = np.stack([np.interp(s, arc, points[:, 0]), np.interp(s, arc, points[:, 1])])
        # The loop is parameterized by arc length at constant speed, so the
        # tangent is the unit direction of the current segment.
        index = int(np.searchsorted(arc, s, side="right") - 1)
        index = min(max(index, 0), len(points) - 2)
        segment = points[index + 1] - points[index]
        norm = float(np.linalg.norm(segment)) or 1.0
        velocity = segment / norm * self.cfg.speed
        return xy, velocity, np.zeros(2)

    @classmethod
    def from_config(cls, config, backend=None) -> np.ndarray:
        """Build the reference and emit the framework's ``(steps, 2)`` schedule.

        Args:
            config: TOML config dict, a :class:`MicroduckLoopConfig`, or a path to
                a TOML file.
            backend: Array backend, forwarded to the instance.

        Returns:
            Array of shape ``(steps, 2)`` — the reference path in arc-length order.
        """
        loop = cls(cls.parse_config(config), backend=backend)
        steps = max(2, int(round(loop.T / loop.cfg.dt)) + 1)
        times = np.linspace(0.0, loop.T, steps, endpoint=not loop.path.closed)
        return np.stack([loop.position_at(t)[0] for t in times])

    @classmethod
    def parse_config(cls, config) -> MicroduckLoopConfig:
        """Strict-parse a TOML dict, a config file path, or a Config instance."""
        if isinstance(config, str):
            with open(resolve_repo_path(config), "rb") as handle:
                config = tomllib.load(handle)
        return super().parse_config(config)


# ─── loading ────────────────────────────────────────────────────────────────

TRAJECTORY_DIR = resolve_repo_path("configs/trajectories")


def trajectory_config_path(name_or_path: str | pathlib.Path) -> pathlib.Path:
    """Resolve a trajectory name to its config file under ``configs/trajectories/``."""
    path = pathlib.Path(name_or_path)
    if not path.exists():
        path = TRAJECTORY_DIR / f"{name_or_path}.toml"
    if not path.exists():
        raise FileNotFoundError(f"no trajectory at {name_or_path} (looked in {TRAJECTORY_DIR})")
    return resolve_repo_path(path)


def load_loop_config(name_or_path: str | pathlib.Path) -> MicroduckLoopConfig:
    """Strict-parse a trajectory config (the registered generator's schema)."""
    with open(trajectory_config_path(name_or_path), "rb") as handle:
        return MicroduckLoop.parse_config(tomllib.load(handle))


def load_trajectory(name_or_path: str | pathlib.Path) -> ReferencePath:
    """Build a reference path from a trajectory config, through the framework.

    Goes via shinro's ``TrajectoryFactory`` -> the registered ``microduck_loop``
    generator, so the demo exercises the same construction path a scenario does.
    """
    from shinro.factories.trajectory_factory import TrajectoryFactory

    config_path = trajectory_config_path(name_or_path)
    with open(config_path, "rb") as handle:
        raw = tomllib.load(handle)
    schedule = TrajectoryFactory(str(config_path)).create()
    return ReferencePath(schedule, closed=MicroduckLoop.parse_config(raw).is_closed)


__all__ = [
    "CLOSED_BY_DEFAULT",
    "POLICY_DEADBAND_CMD",
    "POLICY_VX_MAX_CMD",
    "PROGRESS_FRACTION",
    "SHAPES",
    "MicroduckLoop",
    "MicroduckLoopConfig",
    "ReferencePath",
    "circle",
    "figure_eight",
    "ground_speed_for_command",
    "load_loop_config",
    "load_trajectory",
    "progress_speed_for_command",
    "resample",
    "slalom",
    "smooth",
    "straight",
    "trajectory_config_path",
]
