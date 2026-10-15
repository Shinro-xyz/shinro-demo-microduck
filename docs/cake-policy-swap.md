# Swapping the policy live

`make cake` swaps the **trajectory** under a live policy. This page is the other
half: swapping the **policy** under a live clock — the same `replace-slot`
transaction, applied to the module that computes the robot's action.

```sh
make fetch-cake-policy-swap         # the policy-swap demo is its own release
make policy-swap                    # stand -> walk: the duck stands, then walks
make policy-swap FROM=walk TO=stand # the other direction
make policy-swap PSECS=6            # shorter run
```

It runs `bin/run-policy-swap.sh`, which fetches the same precompiled release and
stages four Cake modules in one resident:

| module | owns | provides | requires |
| --- | --- | --- | --- |
| `bridge` | the sim socket + the 50 Hz tick | `observation` (61), `pose` (4) | `command` (13), `control` (14) |
| `policy` | — | `control` (14) | `observation` (61) |
| `trajectory` | a thread | `command` (13) | `pose` (4) |
| `spare_policy` | — | the swap target | `observation` (61) |

`policy` starts as the standing network, `spare_policy` holds the walking one.
`replace-slot` names the `policy` slot's uuid and points it at the walking
capsule; the resident never restarts.

## Why the policy is a leaf, and why that is the point

The policy cannot own the clock. If it did — the way the *fused* module in
`docs/cake-control-plane.md` does — then swapping it drains the thread that owns
the sim socket, the plant link goes away, and the walk stops. The transaction
still completes; the robot just stops walking.

So the split puts the socket and the clock in a stable `bridge` and makes the
policy a **leaf** the bridge consumes with a `live_snapshot` binding. During the
swap the bridge keeps ticking, re-reads the policy's action table from the
epoch-guarded binding snapshot, and cross-fades from the outgoing network's
action to the incoming one over 25 ticks — a bumpless hand-over, in the one
place that sees both generations.

The observable difference:

| | fused policy swap | split policy swap (`make policy-swap`) |
| --- | --- | --- |
| transaction | COMPLETED | COMPLETED |
| session identity | unchanged | unchanged |
| plant link | **torn down** — the sim client exits | **held** — the walk continues |
| result | duck stops at the swap | duck stands, then walks |

`make policy-swap` writes two figures under `MEDIA_DIR` (the committed copies
live in `docs/media/cake/`):

* `microduck_cake_policy_swap.gif` — the run itself, with the same tracking
  panel the trajectory GIF uses but driven by the *policy* schedule: the path is
  coloured by which policy drove it, and the bottom strip, "policy active over
  time (dashed = replace-slot swap)", names `stand` then `walk` across the swap.
  The duck balances in place under the standing policy for the first two
  seconds, then walks the circle; and
* `microduck_cake_policy_swap.png` — the run drawn the way the trajectory swap's
  figure is (`scripts/plot_cake_policy_swap.py`): the trunk path coloured by
  **which policy** drove it, the trunk speed beside it (near zero while it
  balances, ~0.2 m/s once it walks), and the running graph — the stable
  `bridge` across the whole axis, only the `policy` leaf's generation moving,
  the trajectory unchanged — over the update transaction that moved it.

![The policy swap: the trunk path coloured by policy, the trunk speed, and the graph the transaction moved](media/cake/microduck_cake_policy_swap.png)

_The same figure the trajectory swap draws, for the policy swap instead: the path
is coloured by which policy drove it (`stand` before the dashed marker, `walk`
after), the speed beside it is the gait change itself, and the right panel shows
the stable `bridge` running throughout while only the `policy` leaf's generation
moves. One run of `make policy-swap`._

## Reading the result

A run prints the transaction phase, the session identity before and after, and
the walk's own metrics:

```text
microduck-cake: transaction_phase COMPLETED
walk: steps=511 travel=1.155 m speed=0.113 m/s tilt=1.5 deg z=0.114 m
microduck-cake: policy swap (stand -> walk): PASS — transaction COMPLETED,
  session unchanged, the bridge held the plant link through the swap
```

`PASS` is the swap claim: the transaction completed, the session identity did
not change, and the bridge held the plant link for the whole window. Whether the
duck stayed *upright* is a control outcome and is reported in the metrics, not
asserted.

## The honest caveat

`FROM=stand TO=walk` works: the standing network holds the duck while the
walking network takes over, and the duck walks. The reverse —
`FROM=walk TO=stand` — completes the transaction and holds the link just as
well, but the duck **falls** (tilt ≈ 175°). The standing network is not a
mid-gait recovery controller, so a naive swap into it from a walk does not catch
the robot. That is a property of the networks, not of the swap: the transaction
is identical in both directions. A genuinely bumpless walk→stand would need the
incoming network to be a recovery policy (or a longer, state-aware blend).

The **render profile** matters too, for a different reason. The split's bridge and
leaf each own a 50 Hz thread, so they are loosely coupled; the heavy `cake`
profile's 1080-px composite redraw can starve one of them and the duck can fall
*before* the swap (~2 of 3 runs — with `GIF=` and no render at all, none of the
runs fell). The default `compact` profile renders the same tracking panel at 720
px and holds, which is why `make policy-swap` uses it; `PGIF=cake` selects the
heavy one. That sensitivity is the price of the loose coupling — scheduler
lockstep would remove it.

## It is its own release

`make policy-swap` runs from a **separate Cake release** from `make cake`. The
policy-swap bundle carries the split — the stable `bridge`, the walking and
standing policy leaves, and the trajectories — and **no fused capsule**; the
trajectory bundle carries the fused policy and the trajectories, unchanged. Both
are versioned `v0.1.0`, under separate tags, so the policy-swap demo can be
released without touching the trajectory one.

```sh
make fetch-cake-policy-swap        # bin/fetch-release.sh --flavour policy-swap
```

The private bundle builder produces both flavours
(`release/build-bundle.sh --flavour trajectory|policy-swap`). To run against a
bundle you built yourself, fetch it with `--from`, or point `MD_RELEASE_DIR` at
its parent:

```sh
bin/fetch-release.sh --flavour policy-swap --from /path/to/release/out
MD_RELEASE_DIR=/path/to/release/out make policy-swap
```
