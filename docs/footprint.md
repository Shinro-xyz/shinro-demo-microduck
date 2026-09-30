*Part of [shinro-demo-microduck](../README.md).*

# Footprint: the deployment host vs onnxruntime

`make footprint` (or `python scripts/compare_backends.py`) probes three ways to
run this policy, each in its **own process**, pinned to one CPU:

| backend | what it is |
| ------- | ---------- |
| `onnxruntime` | the reference runtime, on the `.onnx` |
| `shinro-adapter` | the `.so` through shinro's `onnx_rl` adapter — the framework's generic backend, and this demo's parity reference |
| `microduck-policy` | the `.so` through `MicroduckPolicy` — **the host this repo ships** |

Measured on an Intel Core Ultra 5 125H (P-core), Python 3.12.3, 5000 calls × 3
repeats per backend.

## How a team would load it

```python
from shinro_demo_microduck.policy import MicroduckPolicy

policy = MicroduckPolicy()                     # dlopen + preallocate
while running:
    action = policy.step(sensor_observation)   # zero allocations
    apply_joint_targets(action)                # action_scale * a + home_pose
```

Two properties make that shippable, both locked by `tests/test_policy_host.py`:
importing the module pulls in **nothing** — no framework, no numpy, no MuJoCo
(the test boots a fresh interpreter and asserts it) — and the tick loop allocates
no Python objects at all (0.0 B/call traced over 5000 calls). `shinro-bench`
measures the same property on target: **4 KB of RSS growth over 100,000 ticks**.

## On-disk footprint — what must be installed

| backend | policy artifact | dependencies | installed total |
| ------- | --------------- | ------------ | --------------- |
| onnxruntime | 0.79 MB (`.onnx`) | 132.0 MB (onnxruntime + numpy + OpenBLAS) | **132.8 MB** |
| shinro-adapter | 0.81 MB (`.so`) | 68.2 MB (numpy + OpenBLAS) | **69.0 MB** |
| `microduck-policy` | 0.81 MB (`.so`) | 0 MB | **0.8 MB** |

## Process footprint

| backend | RSS after load | Δ over a bare interpreter | file-backed mappings |
| ------- | -------------- | ------------------------- | -------------------- |
| onnxruntime | 58.7 MB | **+44.8 MB** | 92.6 MB |
| shinro-adapter | 61.9 MB | **+47.9 MB** | 123.0 MB |
| `microduck-policy` | 15.2 MB | **+1.2 MB** | 18.4 MB |

The whole mapping delta for `microduck-policy` is one 0.8 MB `.so` plus the few
stdlib modules the class itself imports — everything else in that column is the
Python interpreter. In a C/C++/Zig/Rust daemon there is no interpreter at all,
so the artifact's true footprint is the 0.8 MB file.

Note the `shinro-adapter` row costs as much as onnxruntime: that is the
*adapter's* numpy port-packing, not the kernel. It earns its place by driving the
eager interpreter too (one config, two backends) and by handling recurrent and
multi-port graphs generically — which is exactly what the deployment host
scoped itself out of.

## Time to first inference and per-call latency

| backend | time to first inference | median / call | min / call (floor) |
| ------- | ----------------------- | ------------- | ------------------ |
| onnxruntime | 121.0 ms | 35.5 µs | 33.4 µs |
| shinro-adapter | 199.0 ms | 38.2 µs | 36.3 µs |
| `microduck-policy` | **7.9 ms** | **31.6 µs** | **30.4 µs** |

Cold start is **15× faster than onnxruntime** — the number a daemon pays when
hot-swapping a policy at startup. Per-call latency is within ~10% across all
three, and the deployment host is the fastest of them precisely because it
allocates nothing per tick. Don't over-read that: at 197 k parameters the
arithmetic (~30 µs) and the Python call overhead are the same order of
magnitude, so this is a tie in any practical sense. On a robot the laptop
numbers are irrelevant anyway — see below.

## On target: Raspberry Pi 3 B+

The same kernel cross-compiled to `aarch64` and run on a Pi 3 B+ by the
`shinro-bench` harness, 100,000 ticks against a 1 ms deadline
(`result/2026-09-24T2202_policy_microduck_timing.md`; recorded in
`docs/target_measurements.json` so `make footprint` can print it):

| metric | value |
| ------ | ----- |
| median / call | **749.7 µs** (mean 765.2, p99 856.4) |
| min / max | 733.3 µs / 1249.0 µs |
| jitter (stddev) | 30.7 µs |
| deadline misses | 0.066 % |
| RSS | 12.7 MB (peak 20.9 MB) |
| RSS growth over the run | **4 KB over 100,000 ticks** |
| page faults | 1 minor, 0 major |
| disk | 787 KB `.so` + 11.6 KB manifests |

Three things to take from this. First, the **23× gap** between laptop (31.6 µs) and
target (749.7 µs): laptop latency does not transfer, so measure on the board —
but 750 µs is ~4 % of a 20 ms control period, so there is plenty of headroom.
Second, the flat RSS curve and single page fault are the payoff of the
zero-allocation tick loop, and they are what makes the kernel safe to run for
hours. (The same policy measured 2960 µs on an earlier *debug* build —
`ReleaseFast` is ~4× faster.) Third, this is the artifact the repo ships: a
single 787 KB shared object with a manifest, copyable to any `aarch64` Linux
board.

> **Measurement note (laptop rows).** This host is an Intel P-core/E-core hybrid,
> and the scheduler migrates processes between cores whose clock differs ~2×.
> Unpinned, the *same* backend measured 37 µs and 67 µs on consecutive runs and
> the ranking flipped. `scripts/compare_backends.py` pins every probe with
> `taskset` and takes the minimum over repeats; treat any unpinned latency number
> from this machine as noise.

## Agreement

Kernel vs onnxruntime on the same 2000 observations: **max |Δaction| = 1.02e-06**
(f32 runtime vs f64 kernel). `tests/test_backend_parity.py` locks this in; the
per-tick *kernel vs shinro interpreter* parity is tighter still (`≤ 8.9e-16`), and
`tests/test_policy_host.py` checks the two hosts are **bit-identical** on the same
kernel.
