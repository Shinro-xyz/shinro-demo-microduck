# Every run, not the good one

`policy-swap-runs.png` is the plot; this is the table behind it. For the single clearest
picture of the difference, `policy-swap-velocity.png` plots distance from the pose each
duck held at the request — one line plateaus, the other does not — with the trunk's
vertical speed beneath it. Same robot, same two
networks (Pollen's `alpha_walking` and `velstand`), same command (0.11 m/s, 0.11 rad/s,
confirmed from `move.applied`), same 50 Hz.

## Pollen: `robotctl policy load` while walking

`robotd` 0.17.4 + `duck-sim`, `alpha_walking` ↔ `velstand` alternating so every run is a
real change, load fired 1.5–6.5 s into an 8 s walk. State from
`robotctl monitor --json`.

| run | load at | max tilt | lowest trunk | `fallen` | walking again |
| --- | --- | --- | --- | --- | --- |
| 1 | 1.5 s | 103° | 2.0 cm | yes | +2.00 s |
| 2 | 2.5 s | 103° | 2.0 cm | yes | +2.00 s |
| 3 | 3.5 s | 103° | 1.9 cm | yes | +2.02 s |
| 4 | 4.5 s | 95° | 3.1 cm | yes | +2.02 s |
| 5 | 5.5 s | 102° | 1.1 cm | yes | +1.98 s |
| 6 | 6.5 s | 81° | 4.3 cm | yes | +2.02 s |

6 of 6 leave their gait, 6 of 6 trip their own fall verdict. The 2 s is the `HOME_RAMP`
timer, not a measurement of how long the parking takes. Controls: loading the file
already running is a no-op and the duck keeps walking; the same load issued while the
duck **stands** swaps in place (`in_place`) and nothing moves, because their rule is
*only a change to the network driving moves the robot*.

## Cake: `make policy-swap`, same pair as two capsules

| run | direction | max tilt | lowest trunk | path length | steps | transaction |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | walk → walk_velstand | 5.6° | 11.2 cm | 202 cm | 512 steps | COMPLETED |
| 2 | walk → walk_velstand | 11.6° | 10.9 cm | 202 cm | 509 steps | COMPLETED |
| 3 | walk → walk_velstand | 7.0° | 11.2 cm | 200 cm | 514 steps | COMPLETED |
| 4 | walk → walk_velstand | 7.0° | 11.2 cm | 200 cm | 513 steps | COMPLETED |
| 5 | walk → walk_velstand | 7.8° | 11.2 cm | 201 cm | 514 steps | COMPLETED |
| 6 | walk → stand | 178.8° | −10.4 cm | 274 cm | 513 steps | COMPLETED |
| 7 | walk → stand | 178.8° | −10.4 cm | 273 cm | 512 steps | COMPLETED |
| 8 | walk → stand | 179.1° | −10.3 cm | 274 cm | 514 steps | COMPLETED |
| 9 | walk → stand | 178.8° | −10.4 cm | 273 cm | 512 steps | COMPLETED |
| 10 | walk → stand | 179.1° | −10.3 cm | 274 cm | 513 steps | COMPLETED |

## What the table says, including the part that is not flattering

1. **A completed transaction is not a walking robot.** All three groups report
   `COMPLETED` (theirs: accepted, then homed) with the session identity unchanged.
   Their duck is on the floor; so is ours, in the walk → stand direction.
2. **The direction matters more than the stack.** Cake's walk → walk_velstand keeps the
   duck up in 5 of 5. Cake's walk → stand puts it on its back in 5 of 5 — the incoming
   network expects a static base and the cross-fade cannot invent physical continuity.
   The mirror of that is why Pollen's runtime parks: it will not cross-fade an
   unvalidated gait into a live walk.
3. **Below zero.** In our walk → stand runs the free joint ends under the floor plane
   (−10 cm). That is a violent fall rather than a collapse, and it is worth a look
   before anyone quotes those numbers: a body going through the floor in simulation is a
   sign of contact or torque blow-up as much as of the control failing.
4. **Not a rate, not a robot.** Simulated, in their body for their lane; small n (5–6 per
   group); one duck, one pair of networks, one command. Enough to state the shape of the
   two designs, not enough to promise a number.
