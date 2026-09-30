*Part of [shinro-demo-microduck](../README.md).*

# Fidelity: what this replay is and is not

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

## Three behaviours worth knowing (all measured, all genuine)

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

---

The observation and action layout this replay has to match exactly is specified in
[policy-contract.md](policy-contract.md); the numbers behind the three behaviours
above were measured with the sweep described in [trajectories.md](trajectories.md).
