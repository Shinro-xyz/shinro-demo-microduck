# Pollen's stack, on the same duck, doing the same swap

The comparison is only worth anything if the other side was run as its authors
intend it. So it was: the real daemons (`robotd`, `configd`, `updaterd`, `tofd`)
from `pollen-robotics/microduck` @ `d7560ef` (0.17.4), against `duck-body` in
`pollen-robotics/microduck_rl` — the same MuJoCo body the `make cake` demo drives.

```
pollen_policy_load_walking.gif   the duck walking, the load landing, the duck going down
policy-swap-velocity.png         the figure to use: distance from the request pose (one line
                                 plateaus, one does not) with the trunk's vertical speed beneath,
                                 drawn by `scripts/plot_pollen_comparison.py` from `evidence/`
policy-swap-runs.png             every run as a dot, three groups, group means
policy-swap-runs.md              the per-run table behind that plot, including the runs where
                                 Cake's own swap puts the duck on its back
evidence/                        the raw records the figure was drawn from
```

## What was run

```sh
export DUCK_SIM_RL=/path/to/microduck_rl
scripts/duck-sim                      # 49.9 of 50.0 Hz, 0 missed
scripts/duck-sim drive 0.15 0          # walk forward for 8 s
# 4 s in, with the duck walking:
scripts/duck-sim ctl policy load walk /path/to/velstand.onnx
```

`alpha_walking.onnx` and `velstand.onnx` are both Pollen's own, from
`pollen-robotics/microduck-policies`; `velstand` is their default walking policy
since v5. This repo vendors the second one (`models/microduck/velstand.onnx`), so
the command is `… policy load walk models/microduck/velstand.onnx` when run from
the checkout root. The same pair is swapped on the Cake side
(`../cake/microduck_cake_policy_swap_walk_velstand.gif`, `make walk-swap`), so both
stacks replace the network that is driving, mid-walk.

## What `robotd` says about it

```
policy change replaces the network driving: going home before swapping  (in_place=false)
+0.72 s   the fall verdict changed fallen=true    gravity_z=-0.179  (~80 deg off vertical)
+2.00 s   at the home pose; the policy has the robot
+0.00 s   policy change complete
+2.9  s   walking again      +3.4 s  upright
```

Their own design rule explains it (`docs/design/policy-channel-design.md` in that
repo): *only a change to the network driving moves the robot.* The network driving
is the one being replaced — so the swap you actually need is the one that costs the
gait. The same command issued while the duck **stands** swaps in place and nothing
moves; a load of the file already running is a no-op and the duck keeps walking.
Both of those controls were run.

## Honest limits

- Simulated, in **their** body and their contact model, not on a robot.
- Two forced changes plus two controls; not a distribution. Run more before
  quoting a rate.
- `robotctl monitor` samples at ~9 Hz in its text mode; the JSON stream here ran at
~32 Hz decimated from the 50 Hz loop.
- The 80 deg is a *consequence of ramping home while walking* on this duck with
  this pair of networks. It is not a claim that their runtime is unsafe — parking
  first is a defensible choice, and cross-fading an unvalidated gait is the risk
  they are avoiding.

## Re-running it

The capture needs a display; on a headless box, `Xvfb :99 -screen 0 1280x800x24`
plus `env -u WAYLAND_DISPLAY -u XDG_SESSION_TYPE DISPLAY=:99` makes the MuJoCo
window real enough for `ffmpeg -f x11grab`. The GIF here was cut from
`x11grab` at 25 fps, cropped 330x430 at (380,160), 12 fps.
