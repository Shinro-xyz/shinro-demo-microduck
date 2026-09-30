*Part of [shinro-demo-microduck](../README.md).*

# Repo layout

```
configs/controllers/onnx_rl_microduck.toml   the policy as a shinro onnx_rl controller
configs/controllers/pure_pursuit.toml        tracker gains (registered microduck_pure_pursuit)
configs/trajectories/*.toml                  reference presets, in the microduck_loop schema
scenarios/microduck_walking.toml             policy-only compile scenario (n_x=61, n_u=14)
models/microduck/BEST_alpha_walking.onnx     the vendored walking policy
docs/*.md                                    these documents
docs/target_measurements.json                the on-Pi numbers, with provenance
docs/media/*.gif                             the committed GIFs (the repo's bulk, ~34 MB)
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
interop/                                     the same .so through C, C++, Zig, Python (interop/README.md)
tests/                                       contract, sim, host, import, parity, kernel, backends, trajectory, components
```

Two rules hold the layout together: **`policy.py` is the only file that ships**
(everything else is a demo, a test, or a build input), and **nothing robot-specific
lives in the framework** — the MJCF, policy, actuator mirror, configs and scenarios
are all here.

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

## The framework gates

Three `shinro` verbs touch this artifact, and it is worth being precise about
which apply:

- `shinro build` (`make compile`) — traces → composes → lowers → `zig build` →
  oracle-checks `.so`-vs-interpreter (3.3e-16) → stamps. The gate that matters.
- `shinro verify` (`make verify`) — re-hashes the stamped inputs and outputs
  against the deployment record; a hand-edited `.so` or a moved config fails here.
- `shinro check` (`make check`) — constructs each component through its factory;
  `onnx_rl` and `microduck_pure_pursuit` both pass.
- `shinro trace` — the per-component op-coverage gate — does **not** apply to
  either one. The policy is imported as a whole graph by the `policy_only` recipe
  rather than traced component-by-component, and the tracker is a host-side law
  (nearest-point search, `atan2`) that cannot lower. That split *is* the design:
  the shipped kernel is the memoryless graph, and the part with data-dependent
  branching stays in Python beside it.

`make gates` runs the check + verify pair; CI runs it (and `make interop`) after
`make compile`. The `.so` is not a Python artifact: `make interop` drives the same
file from C, C++, Zig and Python with one 61-D input, and the four 14-D actions
agree bit for bit — see [`../interop/README.md`](../interop/README.md).
