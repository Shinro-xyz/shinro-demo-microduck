# shinro-demo-microduck — AGENTS.md

A **Microduck** reference-robot demo for the `shinro` control framework: a
14-servo biped whose PPO-trained walking policy is compiled from ONNX to a Zig
`.so`, then replayed in MuJoCo through the `shinro_step` C ABI.

This repo is a **consumer** of [`shinro-python-modules`](../shinro-python-modules).
It adds the robot-specific assets and demos; it does **not** reimplement
framework logic (build, trace, oracle, verify all live in the package).

## Orientation

- `shinro` is consumed from a **sibling checkout**, installed editable:
  `make install` → `pip install -e "../shinro-python-modules[onnx-rl]"` +
  `pip install -e ".[mujoco,media,dev]"`. Import as `shinro.*`.
- Python ≥ 3.12. `zig` must be on `PATH` to compile. `rustc` is optional (only
  the interop harness).
- Override the interpreter for any target when deps live in a venv:
  `make demo PYTHON=.venv/bin/python`.
- The compile scenario is `scenarios/microduck_walking.toml` — a `policy_only`
  recipe (no `[estimator]`, no `[plant]`). The kernel lands in
  `build/compiled_policy/`.

## Commands

| command | does |
| ------- | ---- |
| `make compile` | ONNX → `lib/lib_neural_network.so` (runs the oracle gate) |
| `make check` / `make verify` / `make gates` | component construction + artifact-drift gates |
| `make interop` | the same `.so` through C, C++, Zig, Rust, Python |
| `make live` | real-time MuJoCo viewer (`0`–`4` commands, `R` reset, `+`/`-` resize) |
| `make gif` / `make demo` / `make trajectory T=circle` | offline renders |
| `make footprint` | deployment host vs onnxruntime: size, RSS, latency (+ on-Pi) |
| `make test` / `make test-quick` | full suite / no-sim smoke check |

- Single test: `python -m pytest tests/test_x.py -v -k "name"`.
- Programmatic build+verify (consumer of the shinro API, no subprocess):
  `python -m shinro_demo_microduck.build_kernel [scenario] --out DIR --target TRIPLE`.

## Key files

| path | role |
| ---- | ---- |
| `src/shinro_demo_microduck/policy.py` | the **deployment host** — `MicroduckPolicy`, ctypes + stdlib only |
| `sim.py` / `contract.py` / `bam.py` | MuJoCo walk scene, 61-D obs contract, BAM actuator mirror |
| `host.py` | drives the sim from the kernel; shinro `onnx_rl` adapter = parity reference |
| `tracker.py` / `trajectory.py` | registered `Controller` + `TrajectoryGenerator` |
| `presets.py` | the `[physics].preset = "microduck"` seam (opt-in import) |
| `build_kernel.py` | programmatic build+verify (`shinro.codegen.*` directly) |
| `scenarios/`, `configs/`, `models/` | compile scenario, component configs, vendored ONNX |
| `src/shinro_demo_microduck/assets/microduck/` | vendored MJCF + meshes (verbatim from training) |
| `interop/` | C / C++ / Zig / Rust / Python hosts for the C ABI |
| `docs/` | compiled-inference, policy-contract, fidelity, footprint, trajectories, provenance |

## Build chain (where things happen)

`shinro build` = gen (trace/compose/lower, zig-free) → `zig build` → oracle →
stamp → verify. The ONNX conversion is `shinro/codegen/onnx_import.py`
(`import_onnx_policy`, reached via the `policy_only` recipe); the graph lowers in
`shinro/codegen/lower_zig.py` to `graph_data.zig`; the Zig VM is
`shinro/runtime/`. See `docs/compiled-inference.md`.

## Gotchas

- **`policy.py` is the only file that ships.** Importing `shinro_demo_microduck`
  must stay stdlib-only (PEP 562 lazy re-exports); a test boots a fresh
  interpreter to prove no framework/numpy/MuJoCo is pulled. Don't add eager
  imports to `__init__.py`.
- **Component registration is an import side effect.** A scenario using
  `microduck_pure_pursuit` / `microduck_loop` needs
  `--import shinro_demo_microduck.tracker` (which imports `trajectory`).
- **`shinro trace` does not apply here.** The policy is imported as a whole graph
  (`policy_only`), not traced component-by-component; the tracker is a host-side
  law (`atan2`, nearest-point search) that cannot lower. That split is the design.
- **Cross builds skip the host oracle.** `target != "native"` can't `dlopen` on
  the host; the record marks `oracle.status = "not_run"`. Build native for the
  oracle, and pass `--native-record` to link a cross build to it.
- **BAM is the faithful actuator** (what training used). `--no-bam` uses the
  MJCF's placeholder PD gains — a documented mismatch that changes the walk.
- **Policy constraints, all measured:** ~0.25 m/s deadband (slow = stand, not
  crawl), non-monotonic yaw command response, no turn-in-place, feet-only
  collision. The tracker respects the first two; see `docs/fidelity.md`.
- **MuJoCo assets are vendored verbatim** — don't edit `assets/microduck/`. The
  home pose is *not* self-stabilizing; the policy balances (a test pins this, so
  don't "fix" it by stiffening the hold).
- **Metrics:** use net `along-path` progress, not summed per-step distance (the
  trunk wobble over-states it ~1.5×); report path `coverage` alongside
  cross-track error. See `docs/trajectories.md`.
- **Viewer:** window size is hardcoded by MuJoCo (2/3 of the monitor) — resize
  from inside its key callback; `launch_passive` runs a daemon thread whose
  `glfw.terminate` atexit hook segfaults unless you join viewer threads. See
  `docs/watching.md`.
- `make verify` needs `--out build/compiled_policy` (the scenario sets no
  `[compile].out`).

## CI

`.github/workflows/ci.yml`: checkout shinro sibling → install Zig (PyPI
`ziglang` wheel, pinned) → install → `make compile` → `make gates` →
`make interop` → `make test`.

## Git

- **Never commit or push unless asked.** Finish a step, report the diff and
  verification, and stop.
- One commit per coherent change set (not per file). Conventional Commits
  (`feat:`, `fix:`, `docs:` …) — no history rewrites.
