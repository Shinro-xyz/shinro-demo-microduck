# Committed Cake media

Regenerated deliberately, never as a side effect of a run: the runner writes to
`state/run*/` and only lands here when `MEDIA_DIR=docs/media/cake` is named.

| file | command |
| --- | --- |
| `microduck_cake_walk.gif` | `make cake MEDIA_DIR=docs/media/cake` |
| `microduck_cake_walk_anchored.gif` | `make cake ANCHOR=1 MEDIA_DIR=docs/media/cake` |
| `microduck_cake_swap.png` | `make cake MEDIA_DIR=docs/media/cake` (the figure) |
| `microduck_cake_swap_anchored.png` | `make cake ANCHOR=1 MEDIA_DIR=docs/media/cake` |
| `microduck_cake_policy_swap.gif` | `make policy-swap MEDIA_DIR=docs/media/cake` (stand -> walk) |
| `microduck_cake_policy_swap.png` | `make policy-swap MEDIA_DIR=docs/media/cake` (the figure) |
| `microduck_cake_policy_swap_walk_velstand.gif` | `make policy-swap FROM=walk TO=walk_velstand MEDIA_DIR=docs/media/cake` |
| `microduck_cake_policy_swap_walk_velstand.png` | same run, the figure |

The `walk_velstand` pair is the *like-for-like* swap: the network that is driving
is replaced by a genuinely different walking network (Pollen's `velstand`), mid-
walk, with the duck never leaving its gait. The comparison against the same swap
on Pollen's own runtime — which homes the robot first — is in
[`../pollen/`](../pollen/), including the measured figure.

## The trace carries the trunk's pose

`state/run*/trace.csv` now writes `t,x,y,z,cross_track,tilt_deg,yaw_deg`: a swap is
a claim about posture as well as position, and `tilt_deg` is what says whether the
duck stayed on its feet. It is the column the posture comparison in
[`../pollen/`](../pollen/) is drawn from.

## The capture must not perturb the run

A rendered run and a headless one have to start the same way. They did not: the
client used to build the GL context, compile shaders and construct the panels
*inside* the opening exchange with the plane, and the run that came out applied a
different first action than the headless one — a biped that has not settled turns
that into a fall, and the GIF showed a duck on the floor while the headless trace
was a clean walk. The renderer is now built `RENDER_AFTER_TICKS` (10) ticks in, and
four consecutive rendered walk→walk runs came out clean (1.25–1.37 m, end tilt
1.6–5.4°) where three of three fell before.

Residual, stated because it matters if you quote numbers from a GIF: rendered and
headless runs are now reliably good but still not the *same* trace, because the
first published action still differs. Rendering offline from a recorded state
stream would remove even that.

`light` (`--gif light`) is the cheapest profile — a long stride — for machines
that cannot hold 12.5 fps while steering the plant.
