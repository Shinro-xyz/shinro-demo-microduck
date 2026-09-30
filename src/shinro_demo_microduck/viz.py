"""Composite visualization for the compiled-policy replay.

Each captured tick produces one composite RGB frame::

    +---------------------+---------------------+
    |  MuJoCo 3-D view    |  bird's-eye x-y     |
    |  (robot-tracking)   |  trunk path         |
    +---------------------+---------------------+
                          |  velocity tracking  |
                          |  cmd vs measured    |
                          +---------------------+

The x-y panel plots where the trunk actually went (colored by speed) against the
commanded direction, and the right panel plots the commanded twist against the
achieved one — the honest picture of what the policy tracked and what it did not.
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402  (use() must run before pyplot imports)
import numpy as np
from PIL import Image

from shinro_demo_microduck import contract


def follow_camera(camera, lookat, *, distance: float = 0.55, azimuth: float = 130.0, elevation: float = -18.0):
    """Point a MuJoCo camera at the robot from behind-left, at robot scale."""
    camera.azimuth = azimuth
    camera.elevation = elevation
    camera.distance = distance
    camera.lookat[:] = lookat
    return camera


def birdseye_camera(camera, lookat=(0.0, 0.0, 0.0), distance: float = 1.2):
    """Point a MuJoCo camera straight down (bird's-eye)."""
    camera.azimuth = 0.0
    camera.elevation = -89.0
    camera.distance = distance
    camera.lookat[:] = lookat
    return camera


def _resize_to_height(rgb: np.ndarray, height: int) -> np.ndarray:
    rgb = np.asarray(rgb)[..., :3]
    if rgb.shape[0] == height:
        return rgb
    img = Image.fromarray(rgb)
    width = max(1, round(img.width * height / img.height))
    return np.asarray(img.resize((width, height), Image.BILINEAR))


def compose_h(panels, height: int = 640) -> np.ndarray:
    """Horizontally stack RGB panels, each resized to a common height."""
    return np.concatenate([_resize_to_height(p, height) for p in panels], axis=1)


def _twist(command: np.ndarray) -> tuple[float, float, float]:
    c = np.asarray(command, dtype=np.float64).ravel()
    return float(c[0]), float(c[1]), float(c[2])


class ReplayPanels:
    """Bird's-eye trunk path + velocity-tracking panel for one replay run.

    Call :meth:`update` each tick and :meth:`frame` to grab the panel as an RGB
    array, ready to be composed with the 3-D MuJoCo render.

    Args:
        command: The 13-D command block driving the run (annotated on the plot).
        duration: Run length in seconds (x-limit of the velocity panel).
        path_wh / track_wh: Panel sizes in inches.
        dpi: Figure DPI.
    """

    def __init__(
        self,
        command: np.ndarray,
        duration: float,
        *,
        path_wh: tuple[float, float] = (4.6, 4.4),
        track_wh: tuple[float, float] = (4.6, 4.4),
        dpi: int = 100,
    ) -> None:
        self.command = np.asarray(command, dtype=np.float64).ravel()
        self.cmd_vx, self.cmd_vy, self.cmd_wz = _twist(self.command)
        self.path_wh, self.track_wh, self.dpi = path_wh, track_wh, dpi

        self._t: list[float] = []
        self._xy: list[np.ndarray] = []
        self._speed: list[float] = []
        self._vx: list[float] = []
        self._vy: list[float] = []
        self._wz: list[float] = []

        self.fig = plt.figure(figsize=(path_wh[0] + track_wh[0], path_wh[1]), dpi=dpi)
        gs = self.fig.add_gridspec(1, 2, width_ratios=[path_wh[0], track_wh[0]], wspace=0.32)
        self.ax_path = self.fig.add_subplot(gs[0, 0])
        self.ax_vel = self.fig.add_subplot(gs[0, 1])

        ax = self.ax_path
        ax.set_title("bird's-eye trunk path (colored by speed)", fontsize=11)
        (self.path_pts,) = ax.plot([], [], "-", color="tab:orange", lw=2.0, label="trunk path")
        (self.pt_now,) = ax.plot([], [], "o", color="tab:red", ms=8, zorder=5)
        (self.cmd_arrow,) = ax.plot([], [], "-", color="tab:blue", lw=1.6, label=f"cmd v=({self.cmd_vx:.2f},{self.cmd_vy:.2f})")
        ax.plot([0.0], [0.0], "ks", ms=7, label="start")
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8, framealpha=0.9)

        av = self.ax_vel
        (self.ln_vx,) = av.plot([], [], "-", color="tab:blue", lw=1.8, label="vx measured")
        (self.ln_vy,) = av.plot([], [], "-", color="tab:green", lw=1.6, label="vy measured")
        self._cmd_lines = (
            av.axhline(self.cmd_vx, color="tab:blue", ls="--", lw=1.2, alpha=0.7),
            av.axhline(self.cmd_vy, color="tab:green", ls="--", lw=1.2, alpha=0.7),
        )
        av.set_xlim(0.0, duration)
        av.set_xlabel("t [s]")
        av.set_ylabel("linear velocity [m/s]")
        av.set_title(self._vel_title(), fontsize=10)
        av.grid(True, alpha=0.3)
        av.legend(loc="best", fontsize=8, framealpha=0.9)

    def _vel_title(self) -> str:
        return f"velocity tracking · cmd=({self.cmd_vx:+.2f},{self.cmd_vy:+.2f}) m/s · wz={self.cmd_wz:+.2f} rad/s"

    def set_command(self, command: np.ndarray) -> None:
        """Re-annotate for a new command, for a run that switches mid-flight.

        The dashed reference lines, the bird's-eye command arrow and both titles
        follow the live command, so the showcase GIF's panels never advertise a
        command the robot is no longer tracking.
        """
        self.command = np.asarray(command, dtype=np.float64).ravel()
        self.cmd_vx, self.cmd_vy, self.cmd_wz = _twist(self.command)
        for line, value in zip(self._cmd_lines, (self.cmd_vx, self.cmd_vy)):
            line.set_ydata([value, value])
        self.cmd_arrow.set_label(f"cmd v=({self.cmd_vx:.2f},{self.cmd_vy:.2f})")
        self.ax_path.legend(loc="best", fontsize=8, framealpha=0.9)
        self.ax_vel.set_title(self._vel_title(), fontsize=10)

    def update(self, t: float, xy: np.ndarray, velocity: np.ndarray, yaw_rate: float) -> None:
        """Record one tick. ``velocity`` is the world-frame trunk linear velocity (3,)."""
        xy = np.asarray(xy, dtype=np.float64)[:2]
        vel = np.asarray(velocity, dtype=np.float64)[:2]
        self._t.append(t)
        self._xy.append(xy)
        self._speed.append(float(np.linalg.norm(vel)))
        self._vx.append(float(vel[0]))
        self._vy.append(float(vel[1]))
        self._wz.append(float(yaw_rate))

        pts = np.array(self._xy)
        self.path_pts.set_data(pts[:, 0], pts[:, 1])
        self.pt_now.set_data([xy[0]], [xy[1]])
        if len(self._xy) > 1:
            x_span = max(0.3, float(np.ptp(pts[:, 0]))) * 0.25 + 0.15
            y_span = max(0.3, float(np.ptp(pts[:, 1]))) * 0.25 + 0.15
            self.ax_path.set_xlim(pts[:, 0].min() - x_span, pts[:, 0].max() + x_span)
            self.ax_path.set_ylim(pts[:, 1].min() - y_span, pts[:, 1].max() + y_span)

        norm = max(0.25, self.cmd_vx, self.cmd_vy, np.hypot(self.cmd_vx, self.cmd_vy))
        self.cmd_arrow.set_data([xy[0], xy[0] + self.cmd_vx / norm * 0.35], [xy[1], xy[1] + self.cmd_vy / norm * 0.35])

        self.ln_vx.set_data(self._t, self._vx)
        self.ln_vy.set_data(self._t, self._vy)
        # Keep the command reference lines on-screen too — they move when a run
        # switches command mid-flight.
        span = max(
            0.45,
            max(map(abs, self._vx + self._vy)) * 1.2,
            abs(self.cmd_vx) * 1.25,
            abs(self.cmd_vy) * 1.25,
        )
        self.ax_vel.set_ylim(-span, span)

    def frame(self) -> np.ndarray:
        """Render the two panels to an RGB array."""
        self.fig.canvas.draw()
        return np.asarray(self.fig.canvas.buffer_rgba())[..., :3]

    def close(self) -> None:
        plt.close(self.fig)


def command_summary(command: np.ndarray) -> str:
    """One-line human summary of a command block (used in demo output)."""
    cmd = np.asarray(command).ravel()
    head = cmd[contract.OBS_HEAD_CMD]
    body = cmd[contract.OBS_BODY_CMD]
    return f"twist=({cmd[0]:+.2f},{cmd[1]:+.2f},{cmd[2]:+.2f}) head_norm={np.linalg.norm(head):.3f} body_norm={np.linalg.norm(body):.3f}"


class TrackingPanels:
    """Reference-vs-actual path, cross-track error, and what the follower asked for.

    The trajectory counterpart of :class:`ReplayPanels`. A constant-command run
    only needs "did it hold the speed"; a waypoint run needs "did it stay on the
    path", plus the twist the follower was emitting to make that happen — which
    is also the honest picture of the policy's deadband (the shaded band).

    Args:
        reference_xy: The reference path (N, 2) to draw.
        duration: Run length in seconds.
        path_wh / side_wh: Panel sizes in inches.
        dpi: Figure DPI.
    """

    def __init__(
        self,
        reference_xy: np.ndarray,
        duration: float,
        *,
        path_wh: tuple[float, float] = (4.4, 4.4),
        side_wh: tuple[float, float] = (4.4, 4.4),
        dpi: int = 100,
    ) -> None:
        self.reference = np.asarray(reference_xy, dtype=np.float64)[:, :2]
        self._t: list[float] = []
        self._xy: list[np.ndarray] = []
        self._xt: list[float] = []
        self._vx: list[float] = []
        self._wz: list[float] = []

        self.fig = plt.figure(figsize=(path_wh[0] + side_wh[0], path_wh[1]), dpi=dpi)
        outer = self.fig.add_gridspec(1, 2, width_ratios=[path_wh[0], side_wh[0]], wspace=0.32)
        self.ax_path = self.fig.add_subplot(outer[0, 0])
        right = outer[0, 1].subgridspec(2, 1, hspace=0.5)
        self.ax_err = self.fig.add_subplot(right[0, 0])
        self.ax_cmd = self.fig.add_subplot(right[1, 0])

        ax = self.ax_path
        ax.set_title("bird's-eye: reference path vs trunk", fontsize=11)
        ax.plot(self.reference[:, 0], self.reference[:, 1], "--", color="tab:blue", lw=1.6, label="reference path")
        (self.path_pts,) = ax.plot([], [], "-", color="tab:orange", lw=2.0, label="trunk path")
        (self.pt_now,) = ax.plot([], [], "o", color="tab:red", ms=8, zorder=5)
        ax.plot([self.reference[0, 0]], [self.reference[0, 1]], "ks", ms=7, label="start")
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8, framealpha=0.9)

        eax = self.ax_err
        (self.ln_xt,) = eax.plot([], [], "-", color="tab:red", lw=1.8, label="cross-track |Δ| [m]")
        eax.set_xlim(0.0, duration)
        eax.set_xlabel("t [s]")
        eax.set_ylabel("error [m]")
        eax.set_title("cross-track error", fontsize=10)
        eax.grid(True, alpha=0.3)
        eax.legend(loc="upper right", fontsize=8, framealpha=0.9)

        cax = self.ax_cmd
        (self.ln_vx,) = cax.plot([], [], "-", color="tab:blue", lw=1.6, label="vx command")
        (self.ln_wz,) = cax.plot([], [], "-", color="tab:purple", lw=1.6, label="wz command")
        cax.axhspan(-0.30, 0.30, color="tab:gray", alpha=0.18)
        cax.axhline(0.0, color="0.6", lw=0.8)
        cax.set_xlim(0.0, duration)
        cax.set_xlabel("t [s]")
        cax.set_ylabel("command")
        cax.set_title("twist the tracker asked for (shaded = deadband)", fontsize=9)
        cax.grid(True, alpha=0.3)
        cax.legend(loc="best", fontsize=8, framealpha=0.9)

    def update(self, t: float, xy: np.ndarray, cross_track: float, command: np.ndarray) -> None:
        """Record one tick: position, measured cross-track error, emitted command."""
        xy = np.asarray(xy, dtype=np.float64)[:2]
        cmd = np.asarray(command, dtype=np.float64).ravel()
        self._t.append(t)
        self._xy.append(xy)
        self._xt.append(float(cross_track))
        self._vx.append(float(cmd[0]))
        self._wz.append(float(cmd[2]))

        pts = np.array(self._xy)
        self.path_pts.set_data(pts[:, 0], pts[:, 1])
        self.pt_now.set_data([xy[0]], [xy[1]])
        pad = 0.25
        lo = np.minimum(pts.min(axis=0), self.reference.min(axis=0)) - pad
        hi = np.maximum(pts.max(axis=0), self.reference.max(axis=0)) + pad
        self.ax_path.set_xlim(lo[0], hi[0])
        self.ax_path.set_ylim(lo[1], hi[1])

        self.ln_xt.set_data(self._t, self._xt)
        self.ax_err.set_ylim(0.0, max(0.05, max(self._xt) * 1.2))
        self.ln_vx.set_data(self._t, self._vx)
        self.ln_wz.set_data(self._t, self._wz)
        span = max(0.45, max(map(abs, self._vx + self._wz)) * 1.15)
        self.ax_cmd.set_ylim(-span, span)

    def frame(self) -> np.ndarray:
        """Render the panels to an RGB array."""
        self.fig.canvas.draw()
        return np.asarray(self.fig.canvas.buffer_rgba())[..., :3]

    def close(self) -> None:
        plt.close(self.fig)
