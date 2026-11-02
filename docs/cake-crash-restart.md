# Crashing the control plane: what Cake does, and what it does not

`make cake` shows that a **module** can be replaced under a running resident. This
one shows what happens when a module **dies** — and the answer is short, because a
Cake module is not a process.

```bash
make crash                          # SIGKILL the resident at t=5 s, restart it
make crash FAULT=module             # a trap in the module's own thread
make crash CRASHES=3                # three crashes in a row
make crash CRASH_SWAP=1             # commit a live swap, *then* crash
bin/run-crash.sh --seconds 11 --crash-at 5 --hold 3   # the same, spelled out
```

It runs `bin/run-crash.sh`, which stages exactly the deployment `make cake` stages
(policy + trajectory + a spare trajectory slot), starts the MuJoCo plant, crashes
the control plane, restarts the resident, and prints what it measured. Everything
below is one of those measurements, from this checkout on x86_64.

## The module is in the resident

The resident admits a capsule, maps its ELF out of an anonymous `memfd`, and calls
into it — in its own address space:

```text
$ /proc/<resident-pid>/maps
/memfd:microduck   r-xp ...
/memfd:trajectory  r-xp ...
/memfd:spare_eight r-xp ...
```

So there is no boundary for a fault to stop at. A trap inside the trajectory module
is a trap inside the resident: the process dies, the drain never runs, nothing is
written to the journal, and the plant's socket goes away. `bin/run-crash.sh` says so
from `/proc` rather than asserting it (`module code in the resident …`), then
delivers the fault and records the exit status:

| injected fault | where | what the resident did |
| --- | --- | --- |
| `--fault kill` (SIGKILL) | the process | died, exit 137, in 0.01 s; no drain line |
| `--fault module` (SIGILL, a `@trap()`) | the busiest module thread | died, exit 132, in 1.09 s; no drain line |

Both leave the admin socket file behind (the next start replaces it) and the
state-root lock file behind — with its `flock` released by the kernel, which is why
a restart can claim the root again.

Two things worth knowing if you inject faults at this layer. An asynchronous
`SIGSEGV` sent to a thread of a **Rust** process is *swallowed*: Rust installs a
SIGSEGV handler for its stack-guard page, an injected signal is not near that guard,
and the handler returns rather than terminating. And on Ubuntu a core-dumping
signal does not terminate anything until `core_pattern`'s pipe helper (`apport`) has
collected the core, which `RLIMIT_CORE` does not bound — that is the 1.09 s above,
not the resident. `--fault module` is for fidelity; `--fault kill` is the one to
time things with.

## The plant survives; the walk does not, without a clock

Nothing restarts a module. There is no per-module supervision, no heartbeat the
resident acts on, and `GetHealth` reports what a module *says* — it does not notice a
thread that has stopped saying anything. What Cake does guarantee is that a *new
process* starts against the same durable state, and that is what the demo leans on.

So the plant is the supervisor in this script. `demos/demo_cake_policy.py
--reconnect N` — new in this branch, and inert without the flag — keeps the socket
open for business when its peer dies, holds the last command, keeps stepping MuJoCo
on its own clock, and re-accepts a restarted resident's module. The resident is ready
again in **0.06–0.07 s**; the plant is blind for about that plus one accept poll:

| `--restart-after` | plant blind for | outcome |
| --- | --- | --- |
| 0.0 s | 3 ticks, 0.06 s | upright (tilt 3.0°) |
| 0.1 s | 7 ticks, 0.14 s | upright (tilt 3.4°) |
| 0.2 s | 12 ticks, 0.24 s | **fallen** (tilt 171°) |
| 0.3 s | 17 ticks, 0.34 s | **fallen** (tilt 169°) |
| 0.5 s | 28 ticks, 0.56 s | **fallen** (tilt 169°) |

A held command is not a control law. From the trace of the 0.5 s run, the trunk is
upright (2.7°) when the plane goes away, 20.9° over a third of a second later, and
92° by the time the new generation connects — at which point the plant is on its
side and no policy is going to catch it. The same run without a crash
(`bin/run-crash.sh --crashes 0`) stays at 2.7–4.3° for its whole length, so the fall
is the outage and not the run.

The useful number is the budget: **a control-plane restart has to put a policy back
on the socket within ~0.15 s** (7 control periods) for this plant, and the resident's
own restart is 0.07 s of it. Three crashes in a row at `--restart-after 0.05` leave
the walk upright throughout (1.0 m travelled, tilt 2.9°), so it is the *blackout* that
matters, not the number of crashes.

## The restart: what survives, what is refused

Start against a state root that was never committed to and the restart is clean:

```text
journal at start format version 1, records read 0, torn tail none, journal created, next sequence 1
durable start fresh install, no committed record to recover
plan active bindings=3 ... epoch=2 resources=0 slots=3
```

A **new session UUID** (`CAKE-REC-002`), the Plan re-activated from configuration,
all three slots `ACTIVE`, the plant re-acquires it. That is the case `make crash`
shows by default.

Crash *after a committed swap* and the restart is a different story
(`make crash CRASH_SWAP=1`):

```text
journal at start ... records read 2, torn tail none, journal read, next sequence 3
durable start committed records present and not recoverable, class 5: recovered slot
  37d79ec9…d29e names configuration 8a39178d…1367 and the durable configuration store
  holds no entry under it
plan active ... epoch=2 ...            <- the crashed process had committed epoch 3
```

Three things to read out of that:

1. **The recovery refuses, and the process starts anyway.** `DurableStart::NotRecoverable`
   is reported loudly and never substituted for — `plan_entry` documents this as
   deliberate ("whether an unrecoverable durable state should stop a start is a policy
   question intake 004 does not settle"). The refusal is a live-path refusal
   (`class 5`: a recovered slot names a configuration the durable store never received,
   because this deployment writes its Plan through `plan_artifact` + the content store,
   not through the transaction engine's durable Plan write).
2. **The committed epoch goes backwards: 3 → 2.** The fresh process activates from
   configuration at its own initial epoch, over a journal that already carries the
   crashed process's commit. The live graph is the pre-swap Plan; the swap that
   completed before the crash did not survive it.
3. **It then commits 3 again.** The `--swap` probe at the end of the run is a
   `replace-slot` against the restarted resident, and it completes `2 -> 3` — the same
   epoch number the crashed process had already committed, for a different binding.
   The journal's `next sequence 3` is reused by that commit.

There is no crash-loop gate on this path either: `cake-resident`'s `crash_loop::gate`
(`CRASH_LOOP_BOUND = 3`, safe-start mode) is not called from `run_process`, and a
crashed state root grows no `crash-loop.sidecar` — checked after three consecutive
crashes. Three crashes, three clean starts.

## The systemd route (the one Cake actually documents)

Cake does not leave process supervision undefined: `CAKE-FAIL-001` calls for
"fail-stop process termination followed by supervised restart", `packaging/systemd/
cake-resident.service` is the committed example (`Restart=on-abnormal`,
`RestartSec=2`), `demo/pi/cake-resident-user.service` is the same policy as a user
unit, and `docs/operator-runbook.md` makes installing it an operator step. Both
files say what they are: examples, linted as text (`tests/dod/gate1/T004/unit-lint.sh`
for the system unit; the Pi one is not linted at all) and **never copied, enabled or
started by any gate**. Nothing in this repository runs a Cake resident under systemd,
so the route had never been measured. `--supervisor systemd` measures it — the same
crash, with the relaunch left entirely to a transient user unit carrying the
committed example's own keys:

```bash
make crash SUPERVISOR=systemd                    # Restart=on-abnormal, RestartSec=2
make crash SUPERVISOR=systemd RESTART_SEC=0.05    # the same, with a tuned interval
```

| route | crash → policy back on the socket | plant blind for | walk |
| --- | --- | --- | --- |
| manual, immediate (`RESTART_AFTER=0`) | 0.06 s | 3 ticks, 0.06 s | survives |
| systemd, `RestartSec=0.05` | 0.08–0.16 s | 4–8 ticks, 0.08–0.16 s | survives (0.30 s and down once crashes repeat) |
| **systemd, the committed `RestartSec=2`** | **2.07–2.24 s** | **104–109 ticks, 2.08–2.18 s** | **on the floor for ~2 s before the policy returns** |

Two more properties of that unit, measured rather than read off the file:

- **`Restart=on-abnormal` does not restart a refusal.** A process that exits with a
  non-zero *code* is not "abnormal" to systemd: `/bin/sh -c 'exit 2'` under
  `on-abnormal` yields `NRestarts=0`; under `on-failure` the same command restarts
  (4 restarts observed). So a resident that refuses its start — a held state root, an
  unreadable journal, a Plan that does not validate — is left **dead**, with no retry,
  by the very unit the runbook installs. From the plant's side that is worse than a
  crash: a crash comes back.
- **systemd's start limit is the only crash-loop bound on this path.** With
  `RestartSec=0.05` and a crash every second, the unit reached
  `inactive` after four to five starts and never came back — default
  `DefaultStartLimitBurst=5` in `DefaultStartLimitIntervalSec=10`. The plant then
  waited 20.18 s (948 control periods) with nothing to reconnect to, and the duck
  was down. Cake's own crash-loop gate (`crash_loop::gate`, `CRASH_LOOP_BOUND = 3`,
  safe-start mode) is not called from `run_process`, so nothing else bounds the loop.

That is the honest state of the question "isn't Cake a systemd service?": the restart
is systemd's job by design, the shipped interval is roughly fourteen times what this
plant can survive, a refusal is not restarted at all, and the eventual give-up is a
rate limit rather than a safe-start decision about the committed Plan.

There is an in-tree precedent for the missing piece, and it is worth naming because
it shows the pattern is not unknown to Cake:
`crates/lekiwi-supervisor` is a **module** that spawns and supervises a **child
process** — restart bound 4, backoff 50 ms x n, `SIGTERM` then `SIGKILL` after 500 ms.
Cake has supervision for a child process a module owns. It has none for the modules
themselves, which is the gap this demo falls through.

## What this says about the control plane

- **Swappable is not survivable.** `replace-slot` moves a generation under a live
  consumer; nothing in the kernel notices when a generation *stops existing*. The
  consumer keeps reading the last table it was given, forever, which on a robot is a
  frozen command rather than a fault.
- **The stable-clock split is the right shape, and it is also the supervisor.**
  `make policy-swap` already separates the socket-and-clock owner (`bridge`) from the
  policy leaf. That is exactly the piece this demo makes explicit: the process that
  owns the socket is the only one that can notice the other end went away, and it is
  the one that must survive.
- **A durable commit is not a live graph after a crash.** The journal keeps the
  intent; the restart rebuilds from configuration; a swap that was committed is not
  in the rebuilt graph, and the epoch counter walks backwards over its own history.
  Anything that reads epochs as a monotonic ordering — recovery, an operator, a
  supervisor deciding it is looking at a newer state — is reading a number that went
  back.
- **systemd is the named answer, and at its shipped settings it is too slow and too
  narrow.** `RestartSec=2` is 14x the budget, `on-abnormal` skips refusals, and the
  give-up is a rate limit. The 0.05 s variant fits the budget; it is not what any
  committed unit says.
- **The plant is not the right place for the fix.** Re-connecting here was three pages
  of client code and no kernel change, which is exactly the criticism: it works
  because this plant has a socket and a supervisor-shaped loop. A module that hangs
  (rather than dying) would not even give the plant a closed socket.

## Reproducing

```bash
make fetch-cake                    # or make fetch-cake-policy-swap to crash the split
make crash                        # 1 SIGKILL, restart at once, walk continues, PASS
make crash FAULT=module           # the same, as a trap inside the module's thread
make crash CRASH_SWAP=1           # commit a swap first: watch the recovery refuse
make crash CRASHES=3              # no crash-loop gate: three crashes, three starts
make crash SUPERVISOR=systemd     # systemd restarts it: the committed unit's RestartSec=2
make crash SUPERVISOR=systemd RESTART_SEC=0.05 CRASHES=7   # ... until its start limit trips
bin/run-crash.sh --crashes 0      # the control run, no crash
for ra in 0.0 0.1 0.2 0.3; do      # the tolerance ladder
  bin/run-crash.sh --crash-at 5 --restart-after $ra --seconds 11 --hold 3
done
```

`make crash` restarts the resident as soon as the old one is gone (`RESTART_AFTER=0`),
which is why the default run walks through the crash: the plant is blind for 3 ticks.
Every other row of that ladder is the same crash with a slower restart.

Each run writes, under `state/run-crash/` (untracked):

| file | what it is |
| --- | --- |
| `crash.json` | one record per crash: the crash class, drain, leftovers, sessions, epochs, crash-to-ready time, recovery line, and the plant's outage window |
| `crashes.jsonl` | the same, one JSON object per line, as the runner produced them |
| `trace.csv` | the trunk path, with a `linked` column: `0` on ticks the plant spent holding a command with no control plane |
| `client.out`, `resident.out`, `resident.err` | what the plant and the resident said, unbuffered, including the outage as it happens |

The PASS at the end is an assertion about the *harness*, not a verdict about Cake: the
plant stayed up, it re-acquired a restarted control plane, the resident stopped
cleanly afterwards, and the metrics were produced. The limits are the table above.
