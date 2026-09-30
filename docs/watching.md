*Part of [shinro-demo-microduck](../README.md).*

# Watching it

![Microduck walking, driven by the compiled kernel](media/microduck_walk.gif)

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

## What the policy does

| command `(vx, vy, wz)` | mean speed | yaw rate | trunk tilt | parity |
| ---------------------- | ---------- | -------- | ---------- | ------ |
| `(0, 0, 0)` idle | 0.0005 m/s | 0.001 | 0.7° | 1.9e-16 |
| `(0.4, 0, 0)` forward | 0.172 m/s | 0.016 | 0.9° | 6.7e-16 |
| `(0.4, 0, 0.8)` forward-turn | 0.055 m/s | 0.527 | 2.8° | 8.9e-16 |
| `(-0.4, 0, 0)` backward | 0.156 m/s | 0.214 | 2.5° | 5.6e-16 |
| `(0.4, 0.3, 0)` strafe | 0.167 m/s | 0.017 | 2.5° | 7.8e-16 |

Read [Fidelity](fidelity.md) before quoting these
numbers: the policy has a low-speed deadband and cannot turn in place.
