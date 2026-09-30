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

## Results at a glance

| stage | artifact | check | measured |
| ----- | -------- | ----- | -------- |
| import | ONNX → 26-node graph | imported graph vs **onnxruntime** (spec reference) | `9.1e-07` (f32 vs f64) |
| compile | `lib/lib_neural_network.so` · 790 KiB | `shinro build` oracle gate: `.so` vs interpreter | `3.3e-16` |
| replay | 50 Hz MuJoCo loop driven from the `.so` | **lockstep parity** per tick, kernel vs interpreter | `≤ 8.9e-16` |

The replay is driven *only* by the compiled kernel's action. The eager
interpreter runs the same graph on the same observation each tick purely as a
reference — that lockstep comparison is the honest correctness check, and it is
floating-point exact.

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
make demo            # replay every command, write composite GIFs
make test            # unit + integration + kernel parity
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
scenarios/microduck_walking.toml             policy-only compile scenario (n_x=61, n_u=14)
models/microduck/BEST_alpha_walking.onnx     the vendored walking policy
src/shinro_demo_microduck/
  contract.py   61-D obs / 14-D action contract + ONNX-metadata verification
  bam.py        the BAM M6 voltage actuator (mirrors the training kwargs)
  sim.py        MuJoCo walk scene, 61-D observation builder, 50 Hz stepping
  host.py       loads the kernel through shinro's onnx_rl adapter, drives the sim
  presets.py    the registered [physics].preset = "microduck" seam
  viz.py        composite GIF panels
  assets/microduck/   vendored scene/robot MJCF + meshes (verbatim from training)
demos/demo_compiled_policy.py                the replay demo
tests/                                       contract, sim, BAM mirror, import, kernel
```

The replay uses shinro's own `onnx_rl` adapter for both backends — `model_path`
(eager interpreter) and `artifact_dir` (the `.so`) — so the demo exercises the
framework's public seam rather than a private ctypes shim.

## Provenance

`BEST_alpha_walking.onnx` is the ONNX export of the deployed Microduck walking
policy: Weights & Biases run `441tzs6d` @ `model_3750`, published at
[`xiaofengzi/microduck-walking-onnx`](https://huggingface.co/xiaofengzi/microduck-walking-onnx).
It is a 61→512→256→128→14 MLP (197,896 parameters including the baked
observation normalizer). The MJCF, meshes, and BAM settings are vendored
verbatim from the training repo (`mjlab-microduck`) so this demo stays
self-contained and the replay runs the same asset the policy was trained on.
