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

# pyright: reportAttributeAccessIssue=false, reportOptionalMemberAccess=false
#   mujoco and Pillow ship no py.typed stubs, and matplotlib's Agg canvas
#   attributes are not declared on FigureCanvasBase, so Pyright misreports
#   accesses to their C-extension attributes (and types mujoco.Renderer as
#   None). The runtime behaviour is correct; this repo's lint gate is ruff.

from __future__ import annotations

import shutil
import subprocess
from dataclasses import dataclass

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


# --- offscreen frame capture -------------------------------------------------
#
# The 3-D MuJoCo view plus whatever panels a run supplies, composed and written
# as a GIF. It lives here so every demo that renders (the bare kernel and the
# Cake plane) shares one grabber. MuJoCo is imported lazily, so importing this
# module never needs a GL stack.


@dataclass(frozen=True)
class Render:
    """Frame capture settings. ``COMPACT`` targets a GIF small enough to commit."""

    frame_wh: tuple[int, int] = (512, 512)
    composite_width: int = 1024
    every: int = 4  # capture every N control ticks (12.5 fps at 50 Hz)
    colors: int = 128
    #: Percentage of a lossy colour merge applied to the written GIF when
    #: ImageMagick is installed (0 disables it). The follow-camera makes every
    #: frame's 3-D scene new, so a capture cannot lean on inter-frame
    #: compression; merging near colours is the lever left, and a low value
    #: roughly halves the file while the panels stay legible. Ignored when no
    #: converter is installed, and a re-encode that is not smaller is discarded.
    fuzz: int = 0

    @property
    def fps(self) -> float:
        from shinro_demo_microduck.sim import CONTROL_DT

        return 1.0 / (CONTROL_DT * self.every)


#: Camera framing: a 25 cm robot at 0.45 m fills a good fraction of the window.
FOLLOW_DISTANCE = 0.45

#: Small profile for the *walking* showcase GIF (12 s -> ~3 MB).
COMPACT = Render(frame_wh=(320, 320), composite_width=720, every=5, colors=96)

#: Profile for a whole-lap trajectory GIF (~27 s): fewer frames and colours so a
#: full closed loop still fits.
TRAJECTORY_GIF = Render(frame_wh=(256, 256), composite_width=576, every=6, colors=64)

#: Profile for the Cake swap showcase: long enough (~24 s) for several trajectory
#: segments and their swaps. The frame is deliberately larger than the other
#: profiles so the three-row tracking panel (path, cross-track error, timeline)
#: stays legible. The stride is 12, not 6: the composite panel redraw is heavy
#: enough that a stride-6 render cannot hold real time, and the runner times its
#: swaps by wall-clock, so a lagging render drifts from the sim-time trace.
#: `fuzz` re-encodes the written GIF with a lossy colour merge, roughly halving
#: it; the capture cannot lean on inter-frame compression because the
#: follow-camera makes every frame's 3-D scene new.
CAKE = Render(frame_wh=(360, 360), composite_width=1080, every=12, colors=64, fuzz=5)

#: Render profiles selectable with ``--gif``.
GIF_PROFILES = {"full": Render(), "compact": COMPACT, "trajectory": TRAJECTORY_GIF, "cake": CAKE}


def _optimize_gif(path, fuzz: int) -> None:
    """Re-encode a written GIF with a lossy colour merge, in place, if it helps.

    Guarded on both sides: a missing converter leaves the GIF exactly as the
    render wrote it, and a re-encode that does not come out smaller is dropped.
    The frames, size and delay are preserved (only the palette is re-merged).
    """
    if fuzz <= 0:
        return
    exe = shutil.which("magick") or shutil.which("convert")
    if exe is None:
        return
    tmp = path.with_name(path.stem + ".opt.gif")
    try:
        subprocess.run(
            [exe, str(path), "-fuzz", f"{fuzz}%", "-layers", "OptimizeTransparency", str(tmp)],
            check=True,
            capture_output=True,
        )
    except (OSError, subprocess.CalledProcessError):
        tmp.unlink(missing_ok=True)
        return
    if tmp.exists() and tmp.stat().st_size < path.stat().st_size:
        tmp.replace(path)
    else:
        tmp.unlink(missing_ok=True)


def _fit_width(rgb: np.ndarray, render: Render) -> np.ndarray:
    """Downscale a composite frame and quantize it, so the GIF stays small."""
    img = Image.fromarray(np.asarray(rgb)[..., :3])
    if img.width > render.composite_width:
        img = img.resize((render.composite_width, round(img.height * render.composite_width / img.width)), Image.BILINEAR)
    # An adaptive palette is what keeps a photographic render from blowing up
    # the file; the tracking plots stay perfectly readable quantized.
    return np.asarray(img.convert("P", palette=Image.ADAPTIVE, colors=render.colors).convert("RGB"))


def _make_renderer(model, render: Render):
    """Create an offscreen renderer, or ``None`` if no GL backend is available.

    MuJoCo's offscreen framebuffer defaults to 640x480, so a larger capture needs
    a bigger buffer than the scene declares. The vendored scene is left
    byte-identical to the training asset, so the size is raised on the compiled
    model here instead of in the XML.
    """
    import mujoco

    w, h = render.frame_wh
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), w)
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), h)
    try:
        return mujoco.Renderer(model, width=w, height=h)
    except Exception as exc:  # pragma: no cover - depends on the host GL stack
        import sys

        print(
            f"  (no offscreen renderer: {exc})\n"
            f"  -> running without video. On a desktop, try MUJOCO_GL=glfw; on a headless\n"
            f"     box, MUJOCO_GL=egl with a working EGL device.",
            file=sys.stderr,
        )
        return None


class FrameGrabber:
    """Composite-frame capture: the 3-D view plus whatever panels the run supplies.

    Owns only the renderer and the compositing; the panels object is injected, so
    the same grabber serves the constant-command demo (:class:`ReplayPanels`), the
    trajectory demo (:class:`TrackingPanels`) and the Cake plane.
    """

    def __init__(self, sim, render: Render, panels=None) -> None:
        import mujoco

        self.render = render
        self.panels = panels
        self.renderer = _make_renderer(sim.model, render)
        self.frames: list[np.ndarray] = []
        self.camera = None
        if self.renderer is not None:
            self.camera = mujoco.MjvCamera()
            follow_camera(self.camera, sim.trunk_position, distance=FOLLOW_DISTANCE)

    @property
    def enabled(self) -> bool:
        return self.renderer is not None

    def due(self, step: int) -> bool:
        """Whether this control step is a capture step."""
        return self.enabled and step % self.render.every == 0

    def capture(self, sim) -> None:
        """Render the current state and append a composed frame."""
        follow_camera(self.camera, sim.trunk_position, distance=FOLLOW_DISTANCE)
        self.renderer.update_scene(sim.data, camera=self.camera)
        scene = self.renderer.render()
        panels = [scene] if self.panels is None else [scene, self.panels.frame()]
        self.frames.append(_fit_width(compose_h(panels, height=self.render.frame_wh[1]), self.render))

    def save(self, path):
        """Write the captured frames as a GIF, or return ``None`` if nothing was captured."""
        if not self.frames:
            return None
        import imageio.v3 as iio

        path.parent.mkdir(parents=True, exist_ok=True)
        iio.imwrite(path, self.frames, duration=1000.0 / self.render.fps, loop=0)
        _optimize_gif(path, self.render.fuzz)
        return path

    def close(self) -> None:
        if self.panels is not None:
            self.panels.close()


# --- the Cake plane's panel --------------------------------------------------

#: Colours for consecutive trajectory segments.
_SEGMENT_COLORS = ("tab:orange", "tab:blue", "tab:green", "tab:purple", "tab:red", "tab:brown")


class SwapPanels:
    """Reference-vs-actual path, tracking error, and the active trajectory module.

    The Cake plane's panel. It shows the path the walk actually traced and, by
    colour, which trajectory module drove each stretch — the visual answer to
    "the resident kept running while the trajectory changed". A middle row plots
    the cross-track error the active module incurred (zero means the trunk rode
    its reference), and the bottom strip names the module active over time. Each
    swap is marked on the path and, with a dashed line, on both lower rows.

    Args:
        segments: ``[(name, start_seconds), ...]`` in order; the first starts at 0.
        duration: Run length in seconds (x-limits of the time panels).
        references: Optional per-segment reference paths, in the same order as
            ``segments`` (``None`` where a segment's reference is unknown). They
            are drawn dashed behind the trunk and are what the error row measures
            against; the caller computes the error, not this panel.
    """

    def __init__(
        self,
        segments,
        duration,
        *,
        references: list[np.ndarray | None] | None = None,
        wh: tuple[float, float] = (6.2, 6.6),
        dpi: int = 100,
    ) -> None:
        self.names = [name for name, _ in segments]
        self.starts = [float(start) for _, start in segments]
        self.duration = float(duration)
        self._xy: list[list[np.ndarray]] = [[] for _ in self.names]
        self._t_seg: list[list[float]] = [[] for _ in self.names]
        self._cte: list[list[float]] = [[] for _ in self.names]
        self._seg: list[int] = []
        ref_list: list[np.ndarray | None] = list(references) if references is not None else [None] * len(self.names)

        self.fig = plt.figure(figsize=wh, dpi=dpi)
        gs = self.fig.add_gridspec(3, 1, height_ratios=[3.0, 1.6, 0.7], hspace=0.5)
        self.ax = self.fig.add_subplot(gs[0, 0])
        self.ax.set_title("bird's-eye trunk path vs reference, coloured by trajectory module", fontsize=11)
        # The reference each module steers toward, dashed and without a legend
        # entry: the trunk line already names the module. Kept so an anchored run
        # can re-place a segment's reference once its anchor is known.
        self.ref_lines: list = [None] * len(self.names)
        for i, ref in enumerate(ref_list):
            if ref is None:
                continue
            ref = np.asarray(ref, dtype=np.float64)[:, :2]
            self.ref_lines[i] = self.ax.plot(
                ref[:, 0], ref[:, 1], "--", color=_SEGMENT_COLORS[i % len(_SEGMENT_COLORS)], lw=1.0, alpha=0.55, zorder=1
            )[0]
        self.lines = [
            self.ax.plot([], [], "-", color=_SEGMENT_COLORS[i % len(_SEGMENT_COLORS)], lw=2.3, label=name)[0]
            for i, name in enumerate(self.names)
        ]
        (self.pt_now,) = self.ax.plot([], [], "o", color="black", ms=7, zorder=6)
        self.ax.plot([0.0], [0.0], "ks", ms=7, label="start")
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc="best", fontsize=8, framealpha=0.9)

        self.axerr = self.fig.add_subplot(gs[1, 0])
        self.axerr.set_title("tracking error (cross-track |Δ|), coloured by trajectory module", fontsize=9)
        self.lines_err = [
            self.axerr.plot([], [], "-", color=_SEGMENT_COLORS[i % len(_SEGMENT_COLORS)], lw=1.8)[0] for i in range(len(self.names))
        ]
        for i, name in enumerate(self.names):
            start = self.starts[i]
            end = self.starts[i + 1] if i + 1 < len(self.names) else self.duration
            self.axerr.axvspan(start, end, color=_SEGMENT_COLORS[i % len(_SEGMENT_COLORS)], alpha=0.10)
            if i > 0:
                self.axerr.axvline(start, color="black", lw=1.2, ls="--")
        self.axerr.set_xlim(0.0, self.duration)
        self.axerr.set_ylim(0.0, 0.2)
        self.axerr.set_ylabel("error [m]")
        self.axerr.grid(True, alpha=0.3)

        self.axt = self.fig.add_subplot(gs[2, 0])
        for i, name in enumerate(self.names):
            start = self.starts[i]
            end = self.starts[i + 1] if i + 1 < len(self.names) else self.duration
            self.axt.axvspan(start, end, color=_SEGMENT_COLORS[i % len(_SEGMENT_COLORS)], alpha=0.35)
            self.axt.text((start + end) / 2.0, 0.5, name, ha="center", va="center", fontsize=9)
            if i > 0:
                self.axt.axvline(start, color="black", lw=1.2, ls="--")
        (self.cursor,) = self.axt.plot([0.0, 0.0], [0.0, 1.0], color="black", lw=1.5)
        self.axt.set_xlim(0.0, self.duration)
        self.axt.set_ylim(0.0, 1.0)
        self.axt.set_yticks([])
        self.axt.set_xlabel("t [s]")
        self.axt.set_title("trajectory module active over time (dashed = replace-slot swap)", fontsize=9)

    def segment_at(self, t: float) -> int:
        """Index of the trajectory active at ``t`` seconds."""
        idx = 0
        for i, start in enumerate(self.starts):
            if t >= start:
                idx = i
        return idx

    def set_reference(self, seg: int, points: np.ndarray) -> None:
        """Re-place one segment's dashed reference path.

        An anchored trajectory capsule re-parents its path onto the trunk's pose
        at the swap, so the line drawn at construction (the world-fixed shape)
        would be the wrong reference for that stretch and the traced path would
        look like it drifted from it. Call this as soon as a segment's anchor is
        known, with the path already transformed into the world.
        """
        pts = np.asarray(points, dtype=np.float64)[:, :2]
        if self.ref_lines[seg] is None:
            self.ref_lines[seg] = self.ax.plot(
                pts[:, 0],
                pts[:, 1],
                "--",
                color=_SEGMENT_COLORS[seg % len(_SEGMENT_COLORS)],
                lw=1.0,
                alpha=0.55,
                zorder=1,
            )[0]
        else:
            self.ref_lines[seg].set_data(pts[:, 0], pts[:, 1])

    def update(self, t: float, xy: np.ndarray, seg: int, cross_track: float | None = None) -> None:
        """Record one tick into segment ``seg`` and redraw.

        ``cross_track`` is the distance from the trunk to the active module's
        reference path. Pass ``None`` when no reference is known, so the error row
        stays empty rather than plotting a fake zero.
        """
        xy = np.asarray(xy, dtype=np.float64)[:2]
        self._seg.append(int(seg))
        self._xy[seg].append(xy)
        pts = np.array(self._xy[seg])
        self.lines[seg].set_data(pts[:, 0], pts[:, 1])
        self.pt_now.set_data([xy[0]], [xy[1]])

        allpts = np.array([p for seg_pts in self._xy for p in seg_pts])
        span = 0.22 * max(0.4, float(np.ptp(allpts[:, 0])), float(np.ptp(allpts[:, 1]))) + 0.2
        self.ax.set_xlim(allpts[:, 0].min() - span, allpts[:, 0].max() + span)
        self.ax.set_ylim(allpts[:, 1].min() - span, allpts[:, 1].max() + span)

        if cross_track is not None:
            self._t_seg[seg].append(float(t))
            self._cte[seg].append(float(cross_track))
            self.lines_err[seg].set_data(self._t_seg[seg], self._cte[seg])
            hi = max((max(v) for v in self._cte if v), default=0.0)
            self.axerr.set_ylim(0.0, max(0.05, hi * 1.2))

        # First tick of a new segment: mark the swap on the path.
        if len(self._seg) >= 2 and self._seg[-1] != self._seg[-2]:
            self.ax.plot([xy[0]], [xy[1]], "D", color="black", ms=6, zorder=7)

        self.cursor.set_xdata([t, t])

    def frame(self) -> np.ndarray:
        """Render the panel to an RGB array."""
        self.fig.canvas.draw()
        return np.asarray(self.fig.canvas.buffer_rgba())[..., :3]

    def close(self) -> None:
        plt.close(self.fig)
