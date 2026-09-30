"""Drive the Microduck walking simulation from the compiled policy kernel.

The control law is not Python. The ONNX policy was lowered to a dataflow graph
by ``shinro.codegen.onnx_import``, compiled to ``lib/lib_neural_network.so`` by
``shinro build``, and this demo drives MuJoCo from that kernel through the
``shinro_step`` C ABI — loaded by :class:`~shinro_demo_microduck.policy.MicroduckPolicy`,
the ctypes host a robot runtime would ship:

    sim.observation() ─► [state 61] ─► shinro_step(.so) ─► u ─► action 14 ─► sim.step()

Each tick the *eager* interpreter runs the same graph on the same observation, so
the run prints a lockstep parity number — the honest correctness check (the
kernel and the interpreter are two implementations of one graph).

Modes:

    make live                                 # REAL-TIME viewer window, keys switch command
    make gif                                  # small showcase GIF -> docs/media/
    make trajectory T=circle                  # follow a preset path -> build/demos/
    make demo                                 # per-command GIFs -> build/demos/
    python -m demos.demo_compiled_policy --live
    python -m demos.demo_compiled_policy --trajectory circle --laps 2
    python -m demos.demo_compiled_policy forward
    python -m demos.demo_compiled_policy --backend shinro   # A/B the framework's adapter

``--live`` opens a MuJoCo window and runs at the policy's 50 Hz: press ``0``–``4``
to switch command, ``R`` to reset, ``+``/``-`` to grow/shrink the window, and
``ESC`` to quit. The window needs a display (``MUJOCO_GL=glfw`` on a desktop,
``egl`` on a headless box); the GIF modes only need an offscreen GL context and
fall back to metrics-only without one.

Note on the checkpoint: this walking policy has a low-speed deadband — commands
below ~0.25 m/s produce a stand rather than a slow walk (the tracking reward's
velocity std is wide, so standing there is cheap), and pure turn-in-place was
rare in its experience. Commands here stay above that threshold; see README.md.
"""

import os

os.environ.setdefault("MUJOCO_GL", "egl")

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from shinro_demo_microduck import contract
from shinro_demo_microduck.host import BACKENDS, KernelRunner, drive, kernel_info

# Importing the tracker module registers both shinro components
# (`microduck_pure_pursuit`, and `microduck_loop` via its dependency on the
# trajectory module). A scenario built by shinro's CLI needs the same thing:
#     shinro build scenarios/<name>.toml --import shinro_demo_microduck.tracker
from shinro_demo_microduck import tracker as _tracker  # noqa: F401
from shinro_demo_microduck.paths import CONTROLLER_CONFIG, DEFAULT_ARTIFACT
from shinro_demo_microduck.sim import CONTROL_DT, MicroduckSim
from shinro_demo_microduck.trajectory import load_loop_config, load_trajectory, progress_speed_for_command
from shinro_demo_microduck.viz import ReplayPanels, TrackingPanels, compose_h, follow_camera

HERE = Path(__file__).parent.parent
OUT_DIR = HERE / "build" / "demos"
MEDIA_DIR = HERE / "docs" / "media"
#: The tracker is a registered shinro controller (see src/shinro_demo_microduck/tracker.py).
PURE_PURSUIT_CONFIG = HERE / "configs" / "controllers" / "pure_pursuit.toml"

Twist = tuple[float, float, float]

#: Commands the demo replays. Chosen above the policy's ~0.25 m/s deadband.
COMMANDS: dict[str, tuple[Twist, float]] = {
    "idle": ((0.0, 0.0, 0.0), 4.0),
    "forward": ((0.4, 0.0, 0.0), 8.0),
    "forward-turn": ((0.4, 0.0, 0.8), 8.0),
    "backward": ((-0.4, 0.0, 0.0), 8.0),
    "strafe": ((0.4, 0.3, 0.0), 8.0),
}

#: One continuous run that changes command mid-flight — the showcase GIF.
SEQUENCE: tuple[tuple[str, float], ...] = (("idle", 2.0), ("forward", 5.0), ("forward-turn", 5.0))

#: Live-viewer key bindings: glfw keycode -> command program. GLFW's digit codes are
#: their ASCII values, so ``ord('0') + i`` walks ``COMMANDS`` in order.
LIVE_KEYS: dict[int, str] = {ord("0") + i: name for i, name in enumerate(COMMANDS)}
LIVE_RESET_KEYS = (ord("R"), ord("r"))
LIVE_QUIT_KEY = 256  # GLFW_KEY_ESCAPE
#: Window resize keys. MuJoCo hardcodes the viewer window to 2/3 of the monitor's
#: video mode with no launch-time knob, so this is the only way to change it.
LIVE_BIGGER_KEYS = (ord("+"), ord("="))
LIVE_SMALLER_KEYS = (ord("-"), ord("_"))
LIVE_RESIZE_STEP = 1.25

#: Camera framing: a 25 cm robot at 0.45 m fills a good fraction of the window.
FOLLOW_DISTANCE = 0.45


@dataclass(frozen=True)
class Render:
    """Frame capture settings. ``COMPACT`` targets a GIF small enough to commit."""

    frame_wh: tuple[int, int] = (512, 512)
    composite_width: int = 1024
    every: int = 4  # capture every N control ticks (12.5 fps at 50 Hz)
    colors: int = 128

    @property
    def fps(self) -> float:
        return 1.0 / (CONTROL_DT * self.every)


#: Small profile for the *walking* showcase GIF (12 s -> ~3 MB).
COMPACT = Render(frame_wh=(320, 320), composite_width=720, every=5, colors=96)

#: Profile for the committed *trajectory* GIF, which has to fit a whole lap
#: (~27 s, not 12 s). GIF size is frames x pixels x palette, so both are trimmed
#: from COMPACT: 8.3 fps instead of 10, and 64 colours instead of 96. Choppier
#: than the walking GIF, but a full closed loop is worth more than smoothness —
#: and it stays under 5 MB.
TRAJECTORY_GIF = Render(frame_wh=(256, 256), composite_width=576, every=6, colors=64)

#: Render profiles selectable with ``--gif``.
GIF_PROFILES = {"full": Render(), "compact": COMPACT, "trajectory": TRAJECTORY_GIF}


def _fit_width(rgb: np.ndarray, render: Render) -> np.ndarray:
    """Downscale a composite frame and quantize it, so the GIF stays small."""
    from PIL import Image

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
    w, h = render.frame_wh
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), w)
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), h)
    try:
        return mujoco.Renderer(model, width=w, height=h)
    except Exception as exc:  # pragma: no cover - depends on the host GL stack
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
    the same grabber serves the constant-command demo (:class:`ReplayPanels`) and
    the trajectory demo (:class:`TrackingPanels`).
    """

    def __init__(self, sim: MicroduckSim, render: Render, panels=None) -> None:
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

    def capture(self, sim: MicroduckSim) -> None:
        """Render the current state and append a composed frame."""
        follow_camera(self.camera, sim.trunk_position, distance=FOLLOW_DISTANCE)
        self.renderer.update_scene(sim.data, camera=self.camera)
        scene = self.renderer.render()
        panels = [scene] if self.panels is None else [scene, self.panels.frame()]
        self.frames.append(_fit_width(compose_h(panels, height=self.render.frame_wh[1]), self.render))

    def save(self, path: Path) -> Path | None:
        """Write the captured frames as a GIF, or return ``None`` if nothing was captured."""
        if not self.frames:
            return None
        import imageio.v3 as iio

        path.parent.mkdir(parents=True, exist_ok=True)
        iio.imwrite(path, self.frames, duration=1000.0 / self.render.fps, loop=0)
        return path

    def close(self) -> None:
        if self.panels is not None:
            self.panels.close()


def replay(
    name: str,
    artifact: str,
    *,
    use_bam: bool = True,
    render: Render | None = None,
    backend: str = "ctypes",
    out_dir: Path = OUT_DIR,
) -> dict:
    """Run one commanded replay from the kernel; return its metrics (with video path)."""
    twist, duration = COMMANDS[name]
    command = contract.build_command(twist=twist)
    sim = MicroduckSim(command=command, use_bam=use_bam)
    grabber = FrameGrabber(sim, render, ReplayPanels(command, duration)) if render is not None else None

    def on_tick(step: int, t: float, s: MicroduckSim) -> None:
        if grabber is not None and grabber.due(step):
            grabber.panels.update(t, s.trunk_position, s.trunk_linear_velocity, s.yaw_rate)
            grabber.capture(s)

    metrics = drive(
        controller_config=CONTROLLER_CONFIG,
        artifact_dir=artifact,
        duration_s=duration,
        use_bam=use_bam,
        backend=backend,
        sim=sim,
        on_tick=on_tick,
    )

    video = grabber.save(out_dir / f"microduck_compiled_{name}.gif") if grabber is not None else None
    if grabber is not None:
        grabber.close()
    metrics["video"] = str(video) if video else None
    metrics["command"] = command
    return metrics


def sequence(
    artifact: str, *, use_bam: bool = True, render: Render | None = None, backend: str = "ctypes", out_dir: Path = OUT_DIR
) -> dict:
    """One continuous run that switches command partway — the showcase GIF."""
    total = sum(d for _, d in SEQUENCE)
    sim = MicroduckSim(command=contract.build_command(twist=COMMANDS[SEQUENCE[0][0]][0]), use_bam=use_bam)
    runner = KernelRunner(CONTROLLER_CONFIG, artifact, backend=backend, compare_eager=True)
    grabber = FrameGrabber(sim, render, ReplayPanels(sim.command.copy(), total)) if render is not None else None

    step = 0
    t = 0.0
    travelled = 0.0
    for name, duration in SEQUENCE:
        twist, _ = COMMANDS[name]
        sim.command[:] = contract.build_command(twist=twist)
        if grabber is not None and grabber.panels is not None:
            grabber.panels.set_command(sim.command)
        n = int(round(duration / CONTROL_DT))
        start_xy = sim.trunk_position[:2].copy()
        for _ in range(n):
            sim.step(runner.act(sim.observation()))
            t += CONTROL_DT
            step += 1
            if grabber is not None and grabber.due(step):
                grabber.panels.update(t, sim.trunk_position, sim.trunk_linear_velocity, sim.yaw_rate)
                grabber.capture(sim)
        phase_travel = float(np.linalg.norm(sim.trunk_position[:2] - start_xy))
        travelled += phase_travel
        print(f"  {name:<13} {duration:>4.1f} s  ->  {phase_travel / duration:.3f} m/s  trunk z={sim.trunk_position[2]:.3f} m")

    video = grabber.save(out_dir / "microduck_walk.gif") if grabber is not None else None
    if grabber is not None:
        grabber.close()
    print(f"  lockstep parity (kernel vs eager interpreter): {runner.parity:.3e}")
    if video:
        print(f"  video: {video}  ({video.stat().st_size / 1e6:.2f} MB, {grabber.render.fps:.0f} fps)")
    return {"video": str(video) if video else None, "parity": runner.parity, "travel_xy": travelled, "steps": step}


def trajectory(
    name: str,
    artifact: str,
    *,
    use_bam: bool = True,
    render: Render | None = None,
    backend: str = "ctypes",
    out_dir: Path = OUT_DIR,
    laps: float = 1.0,
    duration_s: float | None = None,
) -> dict:
    """Walk a preset reference path with the pure-pursuit follower and report the error.

    This is the deployment shape: navigation gives a path, the follower turns it
    into a twist command each tick, and the compiled policy executes it. The
    follower runs *outside* the kernel — the policy stays a velocity tracker.
    """
    # Both halves are shinro components, built through the framework's own
    # factories: TrajectoryFactory -> the registered `microduck_loop` generator
    # (a (steps, 2) reference schedule), ControllerFactory -> the registered
    # `microduck_pure_pursuit` tracker.
    from shinro.factories.controller_factory import ControllerFactory

    traj = load_trajectory(name)
    loop_cfg = load_loop_config(name)
    tracker = ControllerFactory(str(PURE_PURSUIT_CONFIG)).create()
    tracker.set_reference(traj)
    speed = progress_speed_for_command(tracker.cfg.v_cmd)
    if duration_s is None:
        # One lap (or `laps`) at the policy's *net progress* speed (not its
        # instantaneous body speed), plus a second of settle for open paths.
        duration_s = laps * traj.length / speed + (0.5 if traj.closed else 1.5)
    n_steps = int(round(duration_s / CONTROL_DT))

    sim = MicroduckSim(command=contract.build_command(), use_bam=use_bam)
    start_xy, start_yaw = traj.start_pose()
    sim.place(float(start_xy[0]), float(start_xy[1]), float(start_yaw))
    runner = KernelRunner(CONTROLLER_CONFIG, artifact, backend=backend, compare_eager=True)
    grabber = FrameGrabber(sim, render, TrackingPanels(traj.points, duration_s)) if render is not None else None

    arc = traj.arc_position(sim.trunk_position[:2])
    progress = 0.0
    cross: list[float] = []
    visited: list[np.ndarray] = []
    t = 0.0
    for step in range(n_steps):
        command = tracker.command(sim.trunk_position[:2], _yaw(sim), sim.yaw_rate)
        sim.command[:] = command
        sim.step(runner.act(sim.observation()))
        t = (step + 1) * CONTROL_DT
        here = sim.trunk_position[:2]
        arc_next = traj.arc_position(here)
        progress += traj.advance(arc, arc_next)
        arc = arc_next
        cross.append(tracker.cross_track)
        visited.append(np.array(here))
        if grabber is not None and grabber.due(step):
            grabber.panels.update(t, here, tracker.cross_track, command)
            grabber.capture(sim)

    cross_arr = np.array(cross)
    # Path coverage: the fraction of reference points the robot actually passed
    # close to. Arc progress alone cannot answer "did it walk the whole path?" —
    # a self-intersecting path confuses nearest-point progress, and a lap count
    # says nothing about whether a segment was cut.
    to_path = np.linalg.norm(traj.points[:, None, :] - np.array(visited)[None, :, :], axis=2).min(axis=1)
    coverage = {limit: float((to_path < limit).mean()) for limit in (0.05, 0.10)}
    video = grabber.save(out_dir / f"microduck_trajectory_{name}.gif") if grabber is not None else None
    if grabber is not None:
        grabber.close()

    print(
        f"  reference       {name} (microduck_loop/{loop_cfg.shape}): {len(traj.points)} points, {traj.length:.2f} m, closed={traj.closed}"
    )
    print(f"  tracker         microduck_pure_pursuit: v_cmd={tracker.cfg.v_cmd} -> ~{speed:.3f} m/s, lookahead={tracker.cfg.lookahead} m")
    print(
        f"  duration        {duration_s:.1f} s  ({progress:.2f} m of {traj.length:.2f} m path, "
        f"{progress / traj.length:.2f} laps, {progress / duration_s:.3f} m/s along-path)"
    )
    print(
        f"  cross-track     mean {cross_arr.mean() * 1000:.1f} mm   "
        f"p95 {np.percentile(cross_arr, 95) * 1000:.1f} mm   max {cross_arr.max() * 1000:.1f} mm"
    )
    print(f"  coverage        {coverage[0.05] * 100:.0f}% of path points passed within 50 mm   ({coverage[0.10] * 100:.0f}% within 100 mm)")
    print(f"  trunk           z={sim.trunk_position[2]:.3f} m   tilt={np.degrees(sim.trunk_tilt):.1f} deg")
    print(f"  lockstep parity {runner.parity:.3e}")
    if video:
        print(f"  video: {video}  ({video.stat().st_size / 1e6:.2f} MB, {grabber.render.fps:.0f} fps)")
    return {
        "video": str(video) if video else None,
        "parity": runner.parity,
        "cross_track_mean": float(cross_arr.mean()),
        "cross_track_p95": float(np.percentile(cross_arr, 95)),
        "cross_track_max": float(cross_arr.max()),
        "coverage_50mm": coverage[0.05],
        "coverage_100mm": coverage[0.10],
        "path_progress": progress,
        "tilt_deg": float(np.degrees(sim.trunk_tilt)),
    }


def _yaw(sim: MicroduckSim) -> float:
    """Trunk heading (yaw) in radians."""
    w, x, y, z = (float(v) for v in sim.trunk_quaternion)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _set_hud(viewer, command_name: str, sim: MicroduckSim) -> None:
    """Top-left overlay in the live viewer (best-effort: the API varies by version)."""
    try:
        viewer.set_texts(
            [
                (
                    mujoco.mjtFontScale.mjFONTSCALE_150,
                    mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    f"command: {command_name}",
                    f"z={sim.trunk_position[2]:.3f} m   tilt={np.degrees(sim.trunk_tilt):.1f} deg",
                )
            ]
        )
    except Exception:  # pragma: no cover - older/newer viewer signatures
        pass


def _resize_viewer(bigger: bool) -> str:
    """Grow/shrink the viewer window. Returns a human-readable result.

    MuJoCo sizes its viewer window at **2/3 of the monitor's video mode**, hardcoded
    in ``simulate/glfw_adapter.cc``, and exposes no setter — ``launch_passive`` has
    no size argument and ``vis.global_.off*`` is the offscreen buffer, not the
    window. This runs on the viewer's own thread (we are inside its key callback),
    which is the thread that owns the GLFW window, so it can ask GLFW directly.

    Returns an empty string when the window could not be resolved, so a viewer
    build that cannot do this degrades to "key does nothing" instead of an error.
    """
    try:
        import glfw

        window = glfw.get_current_context()
        if window is None:
            return ""
        width, height = glfw.get_window_size(window)
        scale = LIVE_RESIZE_STEP if bigger else 1.0 / LIVE_RESIZE_STEP
        glfw.set_window_size(window, max(320, int(width * scale)), max(240, int(height * scale)))
        return f"window -> {glfw.get_window_size(window)[0]}x{glfw.get_window_size(window)[1]}"
    except Exception:
        return ""


def live(artifact: str, *, use_bam: bool = True, backend: str = "ctypes", start: str = "idle") -> dict:
    """Watch the policy drive the robot in a real-time MuJoCo viewer.

    Runs at the policy's 50 Hz control rate. Keys (in the viewer window):
    ``0`` idle, ``1`` forward, ``2`` forward-turn, ``3`` backward, ``4`` strafe,
    ``R`` reset the robot, ``+``/``-`` grow/shrink the window, ``ESC`` quit.
    Mouse drag / right-drag / scroll orbit and zoom as usual.
    """
    import mujoco.viewer

    twist, _ = COMMANDS[start]
    sim = MicroduckSim(command=contract.build_command(twist=twist), use_bam=use_bam)
    runner = KernelRunner(CONTROLLER_CONFIG, artifact, backend=backend, compare_eager=True)
    state = {"name": start, "quit": False, "reset": False}

    def key_callback(keycode: int) -> None:
        if keycode == LIVE_QUIT_KEY:
            state["quit"] = True
        elif keycode in LIVE_RESET_KEYS:
            state["reset"] = True
        elif keycode in LIVE_BIGGER_KEYS or keycode in LIVE_SMALLER_KEYS:
            note = _resize_viewer(bigger=keycode in LIVE_BIGGER_KEYS)
            if note:
                print(f"  {note}", flush=True)
        elif keycode in LIVE_KEYS:
            state["name"] = LIVE_KEYS[keycode]
            twist_new, _ = COMMANDS[state["name"]]
            sim.command[:] = contract.build_command(twist=twist_new)
            print(f"  command -> {state['name']}  twist={twist_new}", flush=True)

    print("live viewer — keys: " + "  ".join(f"{chr(k)}={v}" for k, v in sorted(LIVE_KEYS.items())) + "   R=reset  +/-=window  ESC=quit")
    # launch_passive runs MuJoCo's render loop on a daemon thread. We must join it
    # before the interpreter exits: glfw.terminate runs as an atexit hook and would
    # otherwise tear GLFW down underneath the still-running C render loop, which
    # segfaults (mujoco/viewer.py:525).
    threads_before = set(threading.enumerate())
    with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=key_callback) as viewer:
        follow_camera(viewer.cam, sim.trunk_position, distance=FOLLOW_DISTANCE)
        next_t = time.perf_counter()
        while viewer.is_running() and not state["quit"]:
            if state["reset"]:
                sim.reset()
                state["reset"] = False
            sim.step(runner.act(sim.observation()))
            follow_camera(viewer.cam, sim.trunk_position, distance=FOLLOW_DISTANCE)
            _set_hud(viewer, state["name"], sim)
            viewer.sync()
            # Pace to the real control period; if we fall behind, resynchronise
            # instead of accumulating a backlog.
            next_t += CONTROL_DT
            remaining = next_t - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
            else:
                next_t = time.perf_counter()

    for thread in [t for t in threading.enumerate() if t not in threads_before]:
        thread.join(timeout=5.0)
    print(f"live viewer closed.  max lockstep parity: {runner.parity:.3e}")
    return {"parity": runner.parity, "backend": backend}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Replay the Microduck ONNX policy from its compiled kernel.")
    parser.add_argument("commands", nargs="*", choices=sorted(COMMANDS), help="command programs to replay (default: all)")
    parser.add_argument("--artifact", default=str(DEFAULT_ARTIFACT), help="compiled artifact directory")
    parser.add_argument("--no-bam", action="store_true", help="use the MJCF PD gains instead of BAM (actuator mismatch, comparison only)")
    parser.add_argument("--no-video", action="store_true", help="skip the GIF render")
    parser.add_argument("--live", action="store_true", help="open a real-time viewer window instead of rendering GIFs")
    parser.add_argument("--sequence", action="store_true", help="one continuous run that changes command mid-flight (showcase GIF)")
    parser.add_argument(
        "--trajectory", metavar="NAME", help="walk a preset reference path (circle, figure_eight, straight, slalom, waypoints_example)"
    )
    parser.add_argument("--laps", type=float, default=1.0, help="trajectory length in path-laps (default 1)")
    parser.add_argument("--duration", type=float, default=None, help="override the run length in seconds")
    parser.add_argument(
        "--gif",
        choices=sorted(GIF_PROFILES),
        default="full",
        help="render profile: 'full' (default), 'compact' for the walking showcase, 'trajectory' for a whole-lap trajectory",
    )
    parser.add_argument(
        "--out-dir", default=None, help=f"where to write GIFs (default {OUT_DIR}; a compact profile defaults to {MEDIA_DIR})"
    )
    parser.add_argument(
        "--backend",
        default="ctypes",
        choices=list(BACKENDS),
        help="host driving the kernel: 'ctypes' (deployment default) or 'shinro' (framework adapter)",
    )
    args = parser.parse_args(argv)

    try:
        print("kernel:", kernel_info(args.artifact))
    except FileNotFoundError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(f"host:   {args.backend}  (deployment host = ctypes; 'shinro' is the framework adapter, for A/B)")

    use_bam = not args.no_bam
    if args.live:
        try:
            live(args.artifact, use_bam=use_bam, backend=args.backend, start=(args.commands or ["idle"])[0])
        except ImportError as exc:  # pragma: no cover - no display / no viewer
            print(f"ERROR: cannot open the viewer: {exc}", file=sys.stderr)
            return 3
        return 0

    render = None if args.no_video else GIF_PROFILES[args.gif]
    out_dir = Path(args.out_dir) if args.out_dir else (MEDIA_DIR if args.gif != "full" else OUT_DIR)

    if args.trajectory:
        name = args.trajectory
        print(f"\n=== trajectory: {name} ({args.laps:g} lap, bam={use_bam}) ===")
        try:
            trajectory(
                name,
                args.artifact,
                use_bam=use_bam,
                render=render,
                backend=args.backend,
                out_dir=out_dir,
                laps=args.laps,
                duration_s=args.duration,
            )
        except FileNotFoundError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 2
        return 0

    if args.sequence:
        print(f"\n=== showcase sequence (one continuous run, {sum(d for _, d in SEQUENCE):.0f} s) ===")
        sequence(args.artifact, use_bam=use_bam, render=render, backend=args.backend, out_dir=out_dir)
        return 0

    names = args.commands or list(COMMANDS)
    rows = []
    for name in names:
        twist, duration = COMMANDS[name]
        print(f"\n=== {name}: twist={twist} ({duration:.0f} s, bam={use_bam}) ===")
        metrics = replay(name, args.artifact, use_bam=use_bam, render=render, backend=args.backend, out_dir=out_dir)
        rows.append((name, metrics))
        print(
            f"  speed={metrics['mean_speed']:.4f} m/s   yaw_rate={metrics['yaw_rate']:.4f} rad/s   "
            f"trunk_z={metrics['final_trunk_z']:.4f} m   tilt={metrics['tilt_deg']:.2f} deg"
        )
        print(f"  lockstep parity (kernel vs eager interpreter): {metrics['parity']:.3e}")
        if metrics["video"]:
            print(f"  video: {metrics['video']}")

    print("\n=== summary ===")
    header = f"{'command':<14}{'speed [m/s]':>12}{'yaw [rad/s]':>13}{'tilt [deg]':>12}{'parity':>12}"
    print(header)
    print("-" * len(header))
    for name, m in rows:
        print(f"{name:<14}{m['mean_speed']:>12.4f}{m['yaw_rate']:>13.4f}{m['tilt_deg']:>12.2f}{m['parity']:>12.2e}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
