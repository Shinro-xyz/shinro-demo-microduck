*Part of [shinro-demo-microduck](../README.md).*

# How compiled inference works here

```
   ┌─ MuJoCo (BAM actuator) ────────────────────────────────┐
   │                                                        │
   │   sim.observation() ──► [state: 61] ──┐                │
   │                                       ▼                │
   │                         shinro_step(lib/lib_neural_network.so)
   │                                       │                │
   │                                       ▼                │
   │   q_target = HOME + u ◄── [u: 14] ◄───┘                │
   │                                                        │
   └────────────────────────────────────────────────────────┘
   the eager interpreter sees the SAME [state: 61] each tick
   and its [u: 14] is compared to the kernel's — lockstep parity
```

The box that calls `shinro_step` is :class:`MicroduckPolicy` — ctypes and the
standard library, buffers allocated once at load. It reads the port layout from
the graph manifest rather than hardcoding 61/14, and asserts the single-port MLP
shape at load, so a recurrent export fails loudly instead of mis-packing ports.

The kernel is a **pure function of its ports** — no plant, no RNG, no I/O. This
policy is a memoryless MLP, so the manifest declares one input (`state`) and one
output (`u`) and no recurrent `state_*` ports:

```bash
$ shinro build scenarios/microduck_walking.toml --out build/compiled_policy
wrote build/compiled_policy/graph_data.zig (26 nodes)
inputs: ['state']
outputs: ['u']
state outputs: []
oracle B (.so vs interpret): 20 random inputs, max abs err 3.331e-16 ✓
```

The observation normalizer is **baked into the ONNX** by the training repo's
exporter, so the graph's only input port is the raw 61-D observation and the
host does no pre-processing. `configs/controllers/onnx_rl_microduck.toml` sets
`observation.normalize = false` for exactly that reason.

## The gates around the kernel

`shinro build` is the gate that matters — trace → compose → lower → `zig build` →
oracle-check `.so`-vs-interpreter → stamp. Two more `shinro` verbs apply here and
run in CI (`make gates`): `shinro verify` (`make verify`) re-hashes the stamped
inputs and outputs against the deployment record, and `shinro check`
(`make check`) constructs each component through its factory.

`shinro trace` — the per-component op-coverage gate — does **not** apply to either
component. The policy is imported as a whole graph by the `policy_only` recipe
rather than traced component-by-component, and the tracker is a host-side law
(nearest-point search, `atan2`) that cannot lower. That split is the design: the
kernel is the memoryless graph, and the part with data-dependent branching stays
in Python beside it.

The `.so` itself is not a Python artifact — `make interop` runs the same file
through C, C++, Zig and Python hosts and checks the four agree (see
[`../interop/README.md`](../interop/README.md)).

---

What that artifact costs at runtime — install size, RSS, latency, and the same
numbers measured on a Raspberry Pi — is in [footprint.md](footprint.md).
