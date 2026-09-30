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

    make compile                       # ONNX -> lib/lib_neural_network.so (+ oracle gate)
    python -m demos.demo_compiled_policy            # all commands
    python -m demos.demo_compiled_policy forward    # one command
    python -m demos.demo_compiled_policy --backend shinro   # A/B the framework's adapter

Output: one composite GIF per command under ``build/demos/`` (3-D view ·
bird's-eye trunk path · velocity tracking) plus a metrics table.

Note on the checkpoint: this walking policy has a low-speed deadband — commands
below ~0.25 m/s produce a stand rather than a slow walk (the tracking reward's
velocity std is wide, so standing there is cheap), and pure turn-in-place was
rare in its experience. The demo commands are chosen above that threshold; see
README.md.
"""

import os

os.environ.setdefault("MUJOCO_GL", "egl")

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np

from shinro_demo_microduck import contract
from shinro_demo_microduck.host import BACKENDS, drive, kernel_info
from shinro_demo_microduck.paths import CONTROLLER_CONFIG, DEFAULT_ARTIFACT
from shinro_demo_microduck.sim import CONTROL_DT, MicroduckSim
from shinro_demo_microduck.viz import ReplayPanels, compose_h, follow_camera

HERE = Path(__file__).parent.parent
OUT_DIR = HERE / "build" / "demos"

#: Commands the demo replays. Chosen above the policy's ~0.25 m/s deadband.
COMMANDS: dict[str, tuple[tuple[float, float, float], float]] = {
    "idle": ((0.0, 0.0, 0.0), 4.0),
    "forward": ((0.4, 0.0, 0.0), 8.0),
    "forward-turn": ((0.4, 0.0, 0.8), 8.0),
    "backward": ((-0.4, 0.0, 0.0), 8.0),
    "strafe": ((0.4, 0.3, 0.0), 8.0),
}

RENDER_EVERY = 4  # capture every N control ticks (12.5 fps at 50 Hz)
FRAME_WH = (512, 512)
COMPOSITE_WIDTH = 1024  # downscale before writing, so the GIF stays a few MB


def _fit_width(rgb: np.ndarray, width: int = COMPOSITE_WIDTH) -> np.ndarray:
    """Downscale a composite frame to a fixed width (keeps the GIF small)."""
    from PIL import Image

    img = Image.fromarray(np.asarray(rgb)[..., :3])
    if img.width > width:
        img = img.resize((width, round(img.height * width / img.width)), Image.BILINEAR)
    # 128-colour adaptive palette: photographic RGB frames make a needlessly
    # large GIF, and the tracking plots stay perfectly readable quantized.
    return np.asarray(img.convert("P", palette=Image.ADAPTIVE, colors=128).convert("RGB"))


def _make_renderer(model):
    """Create an offscreen renderer, or ``None`` if no GL backend is available.

    MuJoCo's offscreen framebuffer defaults to 640x480, so a 640x640 capture
    needs a bigger buffer than the scene declares. The vendored scene is left
    byte-identical to the training asset, so the size is raised on the compiled
    model here instead of in the XML.
    """
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), FRAME_WH[0])
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), FRAME_WH[1])
    try:
        return mujoco.Renderer(model, width=FRAME_WH[0], height=FRAME_WH[1])
    except Exception as exc:  # pragma: no cover - depends on the host GL stack
        print(
            f"  (no offscreen renderer: {exc})\n"
            f"  -> running without video. On a desktop, try MUJOCO_GL=glfw; on a headless\n"
            f"     box, MUJOCO_GL=egl with a working EGL device.",
            file=sys.stderr,
        )
        return None


def replay(name: str, artifact: str, *, use_bam: bool = True, render: bool = True, backend: str = "ctypes") -> dict:
    """Run one commanded replay from the kernel; return its metrics (with video path)."""
    twist, duration = COMMANDS[name]
    command = contract.build_command(twist=twist)
    sim = MicroduckSim(command=command, use_bam=use_bam)

    renderer = camera = panels = None
    frames: list[np.ndarray] = []
    if render:
        renderer = _make_renderer(sim.model)
    if renderer is not None:
        camera = mujoco.MjvCamera()
        follow_camera(camera, sim.trunk_position, distance=0.6)
        panels = ReplayPanels(command, duration)

    def on_tick(step: int, t: float, s: MicroduckSim) -> None:
        if renderer is None or step % RENDER_EVERY != 0:
            return
        follow_camera(camera, s.trunk_position, distance=0.6)
        renderer.update_scene(s.data, camera=camera)
        scene = renderer.render()
        panels.update(t, s.trunk_position, s.trunk_linear_velocity, s.yaw_rate)
        frames.append(_fit_width(compose_h([scene, panels.frame()], height=FRAME_WH[0])))

    metrics = drive(
        controller_config=CONTROLLER_CONFIG,
        artifact_dir=artifact,
        duration_s=duration,
        use_bam=use_bam,
        backend=backend,
        sim=sim,
        on_tick=on_tick,
    )

    video = None
    if frames:
        import imageio.v3 as iio

        OUT_DIR.mkdir(parents=True, exist_ok=True)
        video = OUT_DIR / f"microduck_compiled_{name}.gif"
        iio.imwrite(video, frames, duration=1000.0 * CONTROL_DT * RENDER_EVERY, loop=0)
    if panels is not None:
        panels.close()
    metrics["video"] = str(video) if video else None
    metrics["command"] = command
    return metrics


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Replay the Microduck ONNX policy from its compiled kernel.")
    parser.add_argument("commands", nargs="*", choices=sorted(COMMANDS), help="command programs to replay (default: all)")
    parser.add_argument("--artifact", default=str(DEFAULT_ARTIFACT), help="compiled artifact directory")
    parser.add_argument("--no-bam", action="store_true", help="use the MJCF PD gains instead of BAM (actuator mismatch, comparison only)")
    parser.add_argument("--no-video", action="store_true", help="skip the GIF render")
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

    names = args.commands or list(COMMANDS)
    rows = []
    for name in names:
        twist, duration = COMMANDS[name]
        print(f"\n=== {name}: twist={twist} ({duration:.0f} s, bam={not args.no_bam}) ===")
        metrics = replay(name, args.artifact, use_bam=not args.no_bam, render=not args.no_video, backend=args.backend)
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
