# The Cake control plane

Two ways to run the same compiled Microduck policy. The first is the Linux way —
the demo `dlopen`s `lib_neural_network.so` and calls `shinro_step` itself. The
second reaches the identical policy through the **Cake supervisory kernel**: the
resident admits the policy as a Cake module, the module owns its own thread, and
there is no `dlopen`, no `ctypes`, and no host-side tick loop at all.

| | bare kernel (`make live`, `make demo`) | Cake plane (`make cake`) |
| --- | --- | --- |
| artifact | `shinro_step` `.so` | a Cake capsule (`CAP_ELF` + descriptor) |
| symbol | `shinro_step` | `cake_module_entry_v1` |
| who loads it | `MicroduckPolicy` (ctypes) | the Cake resident |
| recurrence | the host's loop | the module's own thread; Cake drives the clock |
| plant | MuJoCo, in this process | MuJoCo, in this process, over a socket |

Both drive the **same graph**, imported from the same ONNX, so both produce the
same walk — the table at the bottom is the check.

## How it fits together

The Cake side is a **precompiled release**, not something this repository
builds. The resident and the policy capsule are compiled in the private Cake
tree and published per target; this repository only fetches and runs them.

```text
   shinro-demo-microduck (this repo, no Cake code)        Cake release (private build)
   ┌──────────────────────────────┐                       ┌──────────────────────────────┐
   │ demos/demo_cake_policy.py    │  61 f64  ─socket──►   │ bin/cake-resident            │
   │   MicroduckSim (MuJoCo)      │                       │   admits caps/microduck.cakecap
   │   obs ───────────────────────┼───────────────────────┼► recv obs → Vm.step → u      │
   │   step(u) ◄──────────────────┼─────── 14 f64 ────────┼── send u (one thread)        │
   └──────────────────────────────┘                       └──────────────────────────────┘
```

The demo side sees only "write 61 doubles, read 14 doubles": two little-endian
f64 arrays, which are exactly the graph's own port layout. There is no `cake_*`
header, no entry symbol, no lifecycle callback, and no Cake source in this
repository. The producer, the module and the descriptors live in the private
`shinro-bench/cake` tree; see its
[release README](../../shinro-bench/cake/release/README.md).

## Run it

```bash
make fetch-cake                 # download + verify the release for this target
make cake                       # forward walk, 8 simulated seconds
make cake CMD=forward-turn SECS=8
make cake CMD=backward
```

`make cake` stages a one-slot Plan from the fetched capsule, starts the resident
and this repo's MuJoCo client, lets the walk run, drains the resident, and prints
the trajectory. The trace lands in `build/cake/run/trace.csv`.

Both `x86_64` (a MuJoCo desktop) and `aarch64` (a 64-bit Pi) are published;
`fetch-release.sh` picks the tarball for `uname -m`. Override the location when
the bundle is elsewhere:

```bash
MD_RELEASE_DIR=/path/with/release make cake
cake/fetch-release.sh --from /path/to/tarballs --version v0.1.0
```

## Switch trajectories live, without turning off Cake

The trajectory is a **separate Cake module** the policy consumes, so it can be
swapped while the robot walks:

```bash
make cake SWAP=circle,eight,slalom SECS=9   # 3 s each, live
```

What happens: the initial Plan holds the policy and stages every trajectory
capsule as a slot (the resident admits each one at start). At each boundary the
runner sends `admin-probe … replace-slot` with the 48-byte payload (the slot uuid
plus the replacement capsule's content id), and the kernel's update transaction
publishes the new provider generation, **drains the old one (`prepare_quiesce`)**, rebinds
the policy, activates the new one and reclaims the old — all inside the running
resident.

The demo asserts the point: each `replace-slot` returns
transaction phase `COMPLETED`, the resident's **session identity is unchanged**
across the swaps (it never restarted), the trajectory slot uuid is unchanged, and
the walk never misses a tick (one continuous 61/14-D socket stream). The trace
shows it: the three 3-second segments turn by different amounts, because the
command came from a different module each time.

The policy's `command` requirement is declared `change_policy = "live_snapshot"`,
so it keeps running through the swap and reads the replaceable table under an
epoch guard (`recv obs → Vm.step → send action` stays one lockstep thread).

## See it: the render and the swap figure

`make cake` (default `SWAP=circle,eight,slalom SECS=24`) renders two artifacts into
`docs/media/cake/` (`GIF=cake|full|compact|trajectory`, or `GIF=` to skip):

- `microduck_cake_walk.gif` — the 3-D MuJoCo walk (offscreen EGL) beside a
  bird's-eye panel that **colours the path by the trajectory module driving it**,
  marks each swap on the path, and carries a timeline strip of the module active
  over time.
- `microduck_cake_swap.png` — how Cake ran each swap: the two module bands (the
  policy running throughout, the trajectory generations advancing), the swap
  annotations (`replace-slot → …`, `txn COMPLETED`, `session unchanged`), and the
  update transaction's phases as one flow — closure computed → drain the retiring
  generation → apply the `live_snapshot` row → publish the binding snapshot →
  reclaim the retired generation (`docs/event-code-registry.md`).

Both are drawn by the demo's own code (`demos/demo_cake_policy.py` +
`scripts/plot_cake_swap.py`), from the `trace.csv` and `swaps.json` the run
already writes.

## What the Cake path is, and is not

- It does **not** put the control law in Python. The module runs the VM; Python
  only steps MuJoCo and relays bytes.
- It does **not** build a Cake module. The capsule is compiled in the private
  tree and shipped; this repository has no ABI header or descriptor.
- It does **not** reimplement the plant. `MicroduckSim` and the BAM actuator are
  the same ones `make live` uses.

## Fidelity

The module runs `recv obs → Vm.step → send action` in one thread, so the loop is
lockstep and the arithmetic is identical to the bare-kernel harness. Measured
against `MicroduckPolicy` on the same commands (4 simulated seconds, BAM):

| command | bare kernel travel / speed | Cake plane travel / speed |
| --- | --- | --- |
| `forward` | 0.668 m / 0.167 m/s | 0.677 m / 0.167 m/s |
| `forward-turn` | 0.466 m / 0.117 m/s | 0.480 m / 0.118 m/s |

The remaining difference is sub-percent and is the module's own thread pacing,
not a change in the control law.

## Protocol

One Unix-domain stream socket, strict request/response. The module is the client;
this repo's client binds the path (the module reads it from its configuration
blob).

```text
module → client   14 f64   one action for the current observation
client → module   61 f64   the observation after applying it
```

The client sends the reset observation first; the module then loops, reading an
observation, running its VM, and writing the action. A failed read or write ends
the loop, so stopping the resident drains the module and the client leaves on EOF.
