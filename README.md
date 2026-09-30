# shinro-demo-microduck

A **Microduck** reference-robot demo for the [shinro control framework](https://github.com/Shinro-xyz/shinro-python-modules):
a 14-servo biped whose walking policy was trained with PPO, exported to ONNX, and
here compiled to a tiny language-agnostic `.so` — then replayed in MuJoCo from
**that kernel**.

The point of this repo is shinro's **compiled inference** end to end on a real
robot model. The observation encoder, the network, and the action
post-processing are translated into a shinro dataflow graph, lowered, and
compiled with Zig to a shared object exposing one function:

```c
void shinro_step(const double* in, double* out, double* state);
```

Everything robot-specific the framework deliberately does not ship lives here —
the vendored MJCF + meshes, the policy ONNX, the BAM actuator mirror, the configs
and scenarios, the demos, and the integration tests.

The demo drives the robot from the kernel through
`shinro_demo_microduck.policy.MicroduckPolicy` — **ctypes + the standard
library**, preallocated buffers, no framework, no numpy, no MuJoCo. That is the
host a robot runtime would ship, and `make footprint` measures what it costs:
**+1.2 MB RSS and 0 MB of dependencies**, against onnxruntime's +44.8 MB and
132.8 MB.

## Watching it

![Microduck walking, driven by the compiled kernel](docs/media/microduck_walk.gif)

_Standing → walking → turning: one continuous 12 s run with the command switched
mid-flight. Left: MuJoCo. Middle: bird's-eye trunk path. Right: commanded vs
measured velocity. Frames are captured during a replay whose only control law is
the compiled `.so`._

Three ways, in increasing interactivity:

```bash
make compile && make demo     # per-command GIFs          -> build/demos/*.gif
make gif                      # the showcase GIF above    -> docs/media/
make trajectory T=circle      # follow a preset reference path (build/demos/)
make live                     # REAL-TIME viewer window, keys switch command
```

`make live` opens a MuJoCo window and runs at the policy's 50 Hz control rate:
keys `0`–`4` switch command (`idle`, `forward`, `forward-turn`, `backward`,
`strafe`), `R` resets the robot, `+`/`-` resize the window, `ESC` quits, and the
mouse orbits/zooms as usual. It needs a display — `MUJOCO_GL=glfw` on a desktop,
`egl` on a headless box. The GIF modes only need an offscreen GL context and
print metrics anyway if there isn't one.

> **The window size is MuJoCo's, not ours.** `simulate/glfw_adapter.cc` hardcodes
> it to **2/3 of the monitor's video mode** (1280x800 on this machine) and
> `launch_passive` takes no size argument; `vis.global_.off*` is the *offscreen*
> buffer, not the window. Verified by requesting 640×480 through 1600×1000 and
> watching the viewport stay at 740×720. `+`/`-` therefore resize it after
> launch, from inside the viewer's own thread — the only place the GLFW window
> handle is reachable (asking `glfw.get_current_context()` from the main thread
> returns a dangling pointer and aborts).

## Results at a glance

| stage | artifact | check | measured |
| ----- | -------- | ----- | -------- |
| import | ONNX → 26-node graph | imported graph vs **onnxruntime** (spec reference) | `9.1e-07` (f32 vs f64) |
| compile | `lib/lib_neural_network.so` · 790 KiB | `shinro build` oracle gate: `.so` vs interpreter | `3.3e-16` |
| replay | 50 Hz MuJoCo loop driven from the `.so` | **lockstep parity** per tick, kernel vs interpreter | `≤ 8.9e-16` |

The replay is driven *only* by the compiled kernel's action, through the ctypes
deployment host. The eager interpreter runs the same graph on the same
observation each tick purely as a reference — that lockstep comparison is the
honest correctness check, and it is floating-point exact.

Against onnxruntime, the deployment host is a **165× smaller install**, a
**15× faster cold start**, and slightly faster per call — see
[Footprint](#footprint-the-deployment-host-vs-onnxruntime).

### What the policy does (4 s–8 s runs, BAM actuator)

| command `(vx, vy, wz)` | mean speed | yaw rate | trunk tilt | parity |
| ---------------------- | ---------- | -------- | ---------- | ------ |
| `(0, 0, 0)` idle | 0.0005 m/s | 0.001 | 0.7° | 1.9e-16 |
| `(0.4, 0, 0)` forward | 0.172 m/s | 0.016 | 0.9° | 6.7e-16 |
| `(0.4, 0, 0.8)` forward-turn | 0.055 m/s | 0.527 | 2.8° | 8.9e-16 |
| `(-0.4, 0, 0)` backward | 0.156 m/s | 0.214 | 2.5° | 5.6e-16 |
| `(0.4, 0.3, 0)` strafe | 0.167 m/s | 0.017 | 2.5° | 7.8e-16 |

Read [Fidelity](#fidelity-what-this-replay-is-and-is-not) before quoting these
numbers: the policy has a low-speed deadband and cannot turn in place.

## Quickstart

```bash
make install         # the sibling shinro checkout + this repo's extras
make compile         # ONNX -> lib/lib_neural_network.so (runs the oracle gate)
make live            # watch it: real-time MuJoCo viewer, keys switch command
make gif             # the small showcase GIF the README embeds
make demo            # per-command GIFs (composite: 3-D view, path, velocity)
make footprint       # deployment host vs shinro adapter vs onnxruntime (+ on-Pi numbers)
make test            # contract, sim, host, parity, kernel
```

`make compile` is `shinro build scenarios/microduck_walking.toml --out build/compiled_policy`,
which traces → composes → lowers → `zig build` → **oracle-checks the `.so`
against the interpreter** → stamps and verifies the artifact. Nothing is
trusted until that gate passes.

Replay one command, or skip the renderer:

```bash
python -m demos.demo_compiled_policy forward
python -m demos.demo_compiled_policy --no-video            # metrics only
MUJOCO_GL=glfw python -m demos.demo_compiled_policy        # desktop GL
```

Output: `build/demos/microduck_compiled_<command>.gif` — a composite of the 3-D
view, a bird's-eye trunk path colored by speed, and the velocity-tracking panel.

## How compiled inference works here

```
   ┌─ MuJoCo (BAM actuator) ────────────────────────────────┐
   │                                                        │
   │   sim.observation() ──► [state: 61] ──┐                │
   │                                       ▼                │
   │                         shinro_step(lib/lib_neural_network.so)
   │                                       │                │
   │                                       ▼                │
   │   q_target = HOME + u ◄── [u: 14] ◄───┘                │
   │                                                        │
   └────────────────────────────────────────────────────────┘
   the eager interpreter sees the SAME [state: 61] each tick
   and its [u: 14] is compared to the kernel's — lockstep parity
```

The box that calls `shinro_step` is :class:`MicroduckPolicy` — ctypes and the
standard library, buffers allocated once at load. It reads the port layout from
the graph manifest rather than hardcoding 61/14, and asserts the single-port MLP
shape at load, so a recurrent export fails loudly instead of mis-packing ports.

The kernel is a **pure function of its ports** — no plant, no RNG, no I/O. This
policy is a memoryless MLP, so the manifest declares one input (`state`) and one
output (`u`) and no recurrent `state_*` ports:

```bash
$ shinro build scenarios/microduck_walking.toml --out build/compiled_policy
wrote build/compiled_policy/graph_data.zig (26 nodes)
inputs: ['state']
outputs: ['u']
state outputs: []
oracle B (.so vs interpret): 20 random inputs, max abs err 3.331e-16 ✓
```

The observation normalizer is **baked into the ONNX** by the training repo's
exporter, so the graph's only input port is the raw 61-D observation and the
host does no pre-processing. `configs/controllers/onnx_rl_microduck.toml` sets
`observation.normalize = false` for exactly that reason.

## The policy contract

Every Microduck policy shares this layout so a runtime can hot-swap ONNX files
without touching the glue. `src/shinro_demo_microduck/contract.py` is the
single source of truth, and `verify_against_onnx()` checks it against the ONNX
file's own exporter metadata (joint order, home pose, command block, action
scale) — a mismatch fails loudly instead of silently driving the robot with the
wrong joint order.

**Observation, 61 D:**

| slice | term | notes |
| ----- | ---- | ----- |
| `0:3` | `base_ang_vel` | IMU gyro, trunk frame |
| `3:6` | `projected_gravity` | gravity direction in the trunk frame |
| `6:20` | `joint_pos` | current − home pose, rad |
| `20:34` | `joint_vel` | rad/s |
| `34:48` | `actions` | the previous action |
| `48:51` | `twist` command | `[vx, vy, wz]` |
| `51:55` | `head_pose` command | `[neck_pitch, head_pitch, head_yaw, head_roll]` |
| `55:61` | `body_pose` command | `[x, y, z, roll, pitch, yaw]` |

**Action, 14 D:** `q_target = home_pose + action * action_scale`, with
`action_scale = 1.0` (read from the ONNX metadata, not assumed). Servo order is
`left_hip_yaw … left_ankle, neck_pitch … head_roll, right_hip_yaw … right_ankle`
— asserted against the model's actuator order at construction.

A command slot an env does not use is ZERO-PADDED, never removed. The walking
policy only rewards `twist`; `head_pose` is a small secondary objective and
`body_pose` was trained at weight 0 (kept alive so a later curriculum can use
it). The demo feeds zeros there and commands `twist`.

## Footprint: the deployment host vs onnxruntime

`make footprint` (or `python scripts/compare_backends.py`) probes three ways to
run this policy, each in its **own process**, pinned to one CPU:

| backend | what it is |
| ------- | ---------- |
| `onnxruntime` | the reference runtime, on the `.onnx` |
| `shinro-adapter` | the `.so` through shinro's `onnx_rl` adapter — the framework's generic backend, and this demo's parity reference |
| `microduck-policy` | the `.so` through `MicroduckPolicy` — **the host this repo ships** |

Measured on an Intel Core Ultra 5 125H (P-core), Python 3.12.3, 5000 calls × 3
repeats per backend.

### How a team would load it

```python
from shinro_demo_microduck.policy import MicroduckPolicy

policy = MicroduckPolicy()                     # dlopen + preallocate
while running:
    action = policy.step(sensor_observation)   # zero allocations
    apply_joint_targets(action)                # action_scale * a + home_pose
```

Two properties make that shippable, both locked by `tests/test_policy_host.py`:
importing the module pulls in **nothing** — no framework, no numpy, no MuJoCo
(the test boots a fresh interpreter and asserts it) — and the tick loop allocates
no Python objects at all (0.0 B/call traced over 5000 calls). `shinro-bench`
measures the same property on target: **4 KB of RSS growth over 100,000 ticks**.

### On-disk footprint — what must be installed

| backend | policy artifact | dependencies | installed total |
| ------- | --------------- | ------------ | --------------- |
| onnxruntime | 0.79 MB (`.onnx`) | 132.0 MB (onnxruntime + numpy + OpenBLAS) | **132.8 MB** |
| shinro-adapter | 0.81 MB (`.so`) | 68.2 MB (numpy + OpenBLAS) | **69.0 MB** |
| `microduck-policy` | 0.81 MB (`.so`) | 0 MB | **0.8 MB** |

### Process footprint

| backend | RSS after load | Δ over a bare interpreter | file-backed mappings |
| ------- | -------------- | ------------------------- | -------------------- |
| onnxruntime | 58.7 MB | **+44.8 MB** | 92.6 MB |
| shinro-adapter | 61.9 MB | **+47.9 MB** | 123.0 MB |
| `microduck-policy` | 15.2 MB | **+1.2 MB** | 18.4 MB |

The whole mapping delta for `microduck-policy` is one 0.8 MB `.so` plus the few
stdlib modules the class itself imports — everything else in that column is the
Python interpreter. In a C/C++/Zig/Rust daemon there is no interpreter at all,
so the artifact's true footprint is the 0.8 MB file.

Note the `shinro-adapter` row costs as much as onnxruntime: that is the
*adapter's* numpy port-packing, not the kernel. It earns its place by driving the
eager interpreter too (one config, two backends) and by handling recurrent and
multi-port graphs generically — which is exactly what the deployment host
scoped itself out of.

### Time to first inference and per-call latency

| backend | time to first inference | median / call | min / call (floor) |
| ------- | ----------------------- | ------------- | ------------------ |
| onnxruntime | 121.0 ms | 35.5 µs | 33.4 µs |
| shinro-adapter | 199.0 ms | 38.2 µs | 36.3 µs |
| `microduck-policy` | **7.9 ms** | **31.6 µs** | **30.4 µs** |

Cold start is **15× faster than onnxruntime** — the number a daemon pays when
hot-swapping a policy at startup. Per-call latency is within ~10% across all
three, and the deployment host is the fastest of them precisely because it
allocates nothing per tick. Don't over-read that: at 197 k parameters the
arithmetic (~30 µs) and the Python call overhead are the same order of
magnitude, so this is a tie in any practical sense. On a robot the laptop
numbers are irrelevant anyway — see below.

### On target: Raspberry Pi 3 B+

The same kernel cross-compiled to `aarch64` and run on a Pi 3 B+ by the
`shinro-bench` harness, 100,000 ticks against a 1 ms deadline
(`result/2026-09-24T2202_policy_microduck_timing.md`; recorded in
`docs/target_measurements.json` so `make footprint` can print it):

| metric | value |
| ------ | ----- |
| median / call | **749.7 µs** (mean 765.2, p99 856.4) |
| min / max | 733.3 µs / 1249.0 µs |
| jitter (stddev) | 30.7 µs |
| deadline misses | 0.066 % |
| RSS | 12.7 MB (peak 20.9 MB) |
| RSS growth over the run | **4 KB over 100,000 ticks** |
| page faults | 1 minor, 0 major |
| disk | 787 KB `.so` + 11.6 KB manifests |

Three things to take from this. First, the **23× gap** between laptop (31.6 µs) and
target (749.7 µs): laptop latency does not transfer, so measure on the board —
but 750 µs is ~4 % of a 20 ms control period, so there is plenty of headroom.
Second, the flat RSS curve and single page fault are the payoff of the
zero-allocation tick loop, and they are what makes the kernel safe to run for
hours. (The same policy measured 2960 µs on an earlier *debug* build —
`ReleaseFast` is ~4× faster.) Third, this is the artifact the repo ships: a
single 787 KB shared object with a manifest, copyable to any `aarch64` Linux
board.

> **Measurement note (laptop rows).** This host is an Intel P-core/E-core hybrid,
> and the scheduler migrates processes between cores whose clock differs ~2×.
> Unpinned, the *same* backend measured 37 µs and 67 µs on consecutive runs and
> the ranking flipped. `scripts/compare_backends.py` pins every probe with
> `taskset` and takes the minimum over repeats; treat any unpinned latency number
> from this machine as noise.

### Agreement

Kernel vs onnxruntime on the same 2000 observations: **max |Δaction| = 1.02e-06**
(f32 runtime vs f64 kernel). `tests/test_backend_parity.py` locks this in; the
per-tick *kernel vs shinro interpreter* parity is tighter still (`≤ 8.9e-16`), and
`tests/test_policy_host.py` checks the two hosts are **bit-identical** on the same
kernel.

## Preset trajectories

```bash
make trajectory T=circle LAPS=2 # circle | figure_eight | straight | slalom | waypoints_example
make trajectory-gif             # the short GIF below -> docs/media/
python -m demos.demo_compiled_policy --trajectory figure_eight --laps 2
```

This is the deployment shape: navigation produces waypoints, a tracker converts
them into a twist command each tick, and the compiled policy executes it. The
policy stays a velocity tracker — this layer runs *outside* the kernel and never
replaces it.

Both halves are **shinro components**, not demo-local helpers — each subclasses a
framework ABC, declares a frozen `Config` dataclass as its TOML schema, and
registers itself:

| component | ABC | registered as | config |
| --------- | --- | ------------- | ------ |
| `MicroduckLoop` | `TrajectoryGenerator` | `microduck_loop` | `configs/trajectories/*.toml` |
| `PurePursuitTracker` | `Controller` | `microduck_pure_pursuit` | `configs/controllers/pure_pursuit.toml` |

so they build through the framework's own factories and plug into a scenario:

```python
from shinro.factories.trajectory_factory import TrajectoryFactory
from shinro.factories.controller_factory import ControllerFactory

schedule = TrajectoryFactory("configs/trajectories/circle.toml").create()   # (steps, 2)
tracker  = ControllerFactory("configs/controllers/pure_pursuit.toml").create()
tracker.set_reference(ReferencePath(schedule))
twist = tracker.compute([x, y, yaw, yaw_rate])                              # 13-D command
```

Registration is an import side effect, so a scenario needs the component module —
the same contract every shinro plugin follows:

```bash
shinro build scenarios/<name>.toml --import shinro_demo_microduck.tracker
```

(`tracker` imports `trajectory`, so one `--import` registers both.) `tests/test_shinro_components.py`
holds this contract: both names resolve in the registries, unknown config keys and
a wrong `type` fail loudly, `from_config` emits the framework's `(steps, 2)`
schedule, and **shinro's own generators plug in unchanged** — the tracker only ever
sees a schedule, so its own `lissajous` figure drives the real robot in a test.

**Why a new generator type at all.** shinro's existing trajectories are rest-to-rest
motions from one configuration to another over a fixed duration. A walking policy
needs an *endless* reference that closes on itself, because the robot has to keep
walking. `microduck_loop` is that missing piece, and it says so: its `generate()`
is a documented no-op, since a loop has no start/end to interpolate.

![Microduck following a preset waypoint path](docs/media/microduck_trajectory_waypoints_example.gif)

_A full lap of the 5-waypoint loop (27 s, 8 fps): the path visibly closes. The
composite shows reference vs actual path, the cross-track error, and the twist the
follower asked for — with the policy's deadband shaded, so you can see the
follower never asks for a speed inside it._

Each preset has a full-lap GIF of its own — reference vs actual, cross-track
error, and the command:

| path | GIF | what it shows |
| ---- | --- | ------------- |
| `circle` | [`microduck_trajectory_circle.gif`](docs/media/microduck_trajectory_circle.gif) | constant curvature, the easy case |
| `figure_eight` | [`microduck_trajectory_figure_eight.gif`](docs/media/microduck_trajectory_figure_eight.gif) | yaw command reverses twice per lap |
| `straight` | [`microduck_trajectory_straight.gif`](docs/media/microduck_trajectory_straight.gif) | open reference: walks 2 m and stops |
| `slalom` | [`microduck_trajectory_slalom.gif`](docs/media/microduck_trajectory_slalom.gif) | continuous small steering |
| `waypoints_example` | [`microduck_trajectory_waypoints_example.gif`](docs/media/microduck_trajectory_waypoints_example.gif) | what a navigation stack emits |

### Why a tracker, not a recorded twist profile

The obvious reading of "preset trajectory" is an open-loop schedule of `(vx, wz)`
over time. It does not work on this checkpoint, because its **yaw response is
non-monotonic in the command** (16 s runs, last 8 s, BAM):

| wz command | +0.2 | +0.4 | +0.6 | +0.8 | +1.0 |
| ---------- | ---- | ---- | ---- | ---- | ---- |
| yaw rate achieved | +0.32 | +0.54 | **+0.03** | +0.29 | +0.65 | rad/s |

The signs are right and it never falls (tilt stays under 6°), but asking for
0.6 rad/s of turn gets you almost none while 0.4 gets you 0.54. You cannot
open-loop a heading through that; a tracker that measures the heading error every
tick absorbs it. The tracker also respects the deadband in the other direction —
while tracking it never commands a forward speed inside the unresponsive band — and
never asks for turn-in-place, which this policy cannot do.

### Measured tracking (compiled kernel, BAM, 50 Hz)

| path | cross-track mean | p95 | max | coverage @50 mm | along-path | parity |
| ---- | ---------------- | --- | --- | -------------- | ---------- | ------ |
| `circle` (r = 1 m) | 8.6 mm | 16.8 mm | 19.4 mm | 100% | 0.113 m/s | 5.6e-16 |
| `figure_eight` | 10.5 mm | 24.6 mm | 30.4 mm | 100% | 0.111 m/s | 6.7e-16 |
| `straight` (2 m, stops at the end) | 7.6 mm | 13.2 mm | 17.8 mm | 95% (98% @100 mm) | 0.094 m/s | 5.6e-16 |
| `slalom` (0.35 m amplitude) | 16.3 mm | 32.3 mm | 41.6 mm | 98% | 0.103 m/s | 5.6e-16 |
| `waypoints_example` (smoothed) | 14.0 mm | 37.6 mm | 52.8 mm | 100% | 0.110 m/s | 5.6e-16 |

**Coverage** — the fraction of reference points the robot actually passed close to —
is reported alongside the cross-track error because they answer different
questions. A mean cross-track error can look fine while a whole segment is cut
(`slalom` sits at 44 mm mean but only covers half the path at 50 mm), and a lap
count says nothing about *where* the lap went. `straight`'s 96% is the follower
stopping inside its 15 cm goal radius, as designed.

`PurePursuitConfig` defaults (`k_heading = 1.5`, `k_yaw_rate = 0.5`,
`lookahead = 0.25 m`, `v_cmd = 0.35`) are the best row of a sweep scored across
**three** paths at once. That matters: gains tuned on the circle alone
(`lookahead = 0.35`) track the circle at 13 mm and the waypoint loop at 40 mm,
because a long lookahead cuts corners. `tests/test_trajectory.py` re-measures the
closed loop and locks them. Because the tracker is its own component, per-path
overrides are a second controller config, not a field on the trajectory.

### Sparse waypoints need conditioning — and it is two separate steps

A 5-waypoint loop has ~68° corners over zero arc length, and the tracker's
nearest-point search jumps between vertices when the vertices are metres apart.
Measured on the same loop, same gains:

| conditioning | mean | p95 | max |
| ------------ | ---- | --- | --- |
| raw 5 waypoints | 148.0 mm | 251.5 mm | 276.6 mm |
| resampled to 5 cm | 22.6 mm | 44.3 mm | 70.4 mm |
| resampled + smoothed (shipped) | **18.2 mm** | **32.3 mm** | **44.9 mm** |

So **densification is the essential step** (148 → 23 mm) and the smoothing pass
then blunts the corner-cutting peaks (max 70 → 45 mm). Worth separating, because
an earlier version of this table credited smoothing with the whole 148 → 25 mm
improvement — it was mostly resampling. The drawn reference is the conditioned
path, i.e. the path actually being tracked.

### Two measurement traps this walked into

- **Summed per-step distance over-states progress.** The trunk wobbles laterally
  every tick, so on a straight 2 m path the naive per-step sum reads 1.98 m when
  the robot has advanced 1.36 m — and a run sized from it stops two-thirds of the
  way round. Runs are timed and reported on *net along-path progress*
  (`Trajectory.advance`, positive deltas only).
- **A self-intersecting path breaks nearest-point progress.** At the figure-eight's
  crossing the nearest point flips between branches and reports metres within one
  50 Hz tick, so the metric rejects any single-tick advance above 0.25 m.

## Fidelity: what this replay is and is not

This is a **rehearsal**, mirroring the training repo's own CPU path
(`scripts/infer_policy.py`). It is not a sim2real claim. What is matched:

- **Actuators are BAM** (`better-actuator-models`, pinned to `==1.0.1`, the
  version training resolved). The vendored MJCF declares placeholder `<position>`
  gains (kp 50 / 10 / 0.52 …) that training never used; every policy in this
  family was trained under a voltage-controlled XL330 with a firmware position
  loop. Running the XML's PD gains is a *different actuator model* — it changes
  the walk (0.127 m/s vs 0.095 m/s for the same command) — so BAM is the
  default and `--no-bam` exists only for comparison. `src/shinro_demo_microduck/bam.py`
  mirrors `_BAM_ACTUATOR_KWARGS`; `tests/test_sim.py` locks the values.
- **Timing**: 2 ms physics × 10 substeps = 50 Hz control, the rate every policy
  in this family was trained at.
- **`base_ang_vel`**: read from the free joint's rotational velocity, which for
  MuJoCo is already trunk-frame — the same quantity as training's
  `root_link_ang_vel_b`. `tests/test_sim.py` asserts it equals the `imu_ang_vel`
  gyro, so a frame convention change cannot slip through.
- **No action clipping**: training ran with `clip_actions = None`.

What this replay does *not* reproduce: BAM's domain randomization (battery
voltage, sag, encoder bias, IMU misalignment, actuator delay, observation
noise). The demo pins deterministic mid-range values. For transfer work use the
training repo's tooling.

### Three behaviours worth knowing (all measured, all genuine)

1. **Low-speed deadband.** A command below ~0.25 m/s produces a *stand*, not a
   slow walk: `vx = 0.2` → 0.0009 m/s, `vx = 0.4` → 0.173 m/s. The velocity
   tracking reward's std is wide (0.3 m/s), so standing at a small command is
   cheap — which is also why a runtime hot-swaps in a *stand* policy at low
   speed rather than asking the walker to crawl.
2. **No turn-in-place.** A pure yaw command (`vx = 0, wz = 1.0`) stands still;
   turning only happens while walking (`vx = 0.4, wz = 0.8` → 0.53 rad/s). This
   is the training repo's known turn-in-place gap, addressed there with an
   explicit command bucket — this checkpoint predates it.
3. **The home pose is not a self-stabilizing equilibrium.** Holding
   `q_target = home` with no policy tips the robot over within ~2 s (with or
   without BAM), because the home pose puts the CoM ~5 mm ahead of the ankle
   axis. The *policy* does the balancing: the compiled idle policy holds the
   stand from a 2 cm-high spawn with 0.1 rad/s of joint noise at 0.7° tilt.
   `tests/test_sim.py` pins the first fact and `tests/test_kernel_parity.py` the
   second, so nobody "fixes" the sim by stiffening the hold.

Also note the walk model is **feet-only** for collisions (`robot_walk.xml`): once
the robot tips past recovery it has nothing to rest on and passes through the
floor. That is intentional for a walking env — falls end the episode — but it
makes the walk scene useless for fall-recovery work; the ground-contact and
all-collision variants exist for that.

## Repo layout

```
configs/controllers/onnx_rl_microduck.toml   the policy as a shinro onnx_rl controller
configs/controllers/pure_pursuit.toml        tracker gains (registered microduck_pure_pursuit)
configs/trajectories/*.toml                  reference presets, in the microduck_loop schema
scenarios/microduck_walking.toml             policy-only compile scenario (n_x=61, n_u=14)
models/microduck/BEST_alpha_walking.onnx     the vendored walking policy
docs/target_measurements.json                the on-Pi numbers, with provenance
docs/media/microduck_walk.gif                the showcase GIF the README embeds
src/shinro_demo_microduck/
  policy.py     MicroduckPolicy — THE DEPLOYMENT HOST (ctypes + stdlib, no deps)
  trajectory.py microduck_loop — the registered TrajectoryGenerator (closed references)
  tracker.py    microduck_pure_pursuit — the registered Controller (the control law)
  contract.py   61-D obs / 14-D action contract + ONNX-metadata verification
  bam.py        the BAM M6 voltage actuator (mirrors the training kwargs)
  sim.py        MuJoCo walk scene, 61-D observation builder, 50 Hz stepping
  host.py       drives the sim from the kernel; eager adapter as parity reference
  presets.py    the [physics].preset = "microduck" seam (opt-in import)
  viz.py        composite GIF panels
  assets/microduck/   vendored scene/robot MJCF + meshes (verbatim from training)
demos/demo_compiled_policy.py                the replay demo
scripts/backend_probe.py                     one backend, one process: RSS, mappings, latency
scripts/compare_backends.py                  deployment host vs adapter vs onnxruntime (+ on-Pi)
tests/                                       contract, sim, host, import, parity, kernel, backends, trajectory, components
```

The package import is deliberately light: importing it must not drag the
framework, numpy, or MuJoCo into the process, because `policy.py` is what ships.
Everything else is re-exported lazily. One consequence: the
`[physics].preset = "microduck"` seam registers when
`shinro_demo_microduck.presets` is imported, so a scenario using it is built with
`shinro build … --import shinro_demo_microduck.presets` (no in-tree scenario
needs it — the compile scenario is policy-only).

The demo's *reference* is shinro's `onnx_rl` adapter in eager mode, which is how
the parity check gets an independent implementation of the graph instead of
comparing the kernel to itself.

## Provenance

`BEST_alpha_walking.onnx` is the ONNX export of the deployed Microduck walking
policy: Weights & Biases run `441tzs6d` @ `model_3750`, published at
[`xiaofengzi/microduck-walking-onnx`](https://huggingface.co/xiaofengzi/microduck-walking-onnx).
It is a 61→512→256→128→14 MLP (197,896 parameters including the baked
observation normalizer). The MJCF, meshes, and BAM settings are vendored
verbatim from the training repo (`mjlab-microduck`) so this demo stays
self-contained and the replay runs the same asset the policy was trained on.
