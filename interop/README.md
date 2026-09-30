# Cross-language C-ABI interop

The compiled walking policy is a language-agnostic C-ABI artifact:

```c
void shinro_step(const double* in, double* out, double* state);
```

Nothing about it is Python-, MuJoCo- or shinro-specific. Any language that can
`dlopen` a shared library can drive it. This directory calls the **same** `.so`
through C, C++, Zig, and Python with an identical 61-D input and shows they agree
bit for bit — the empirical form of the repo's "0.8 MB, no interpreter" claim.
Everywhere else in this repo the kernel is loaded from Python; here the *host*
has no runtime dependency at all.

Build the artifact first (from the repo root):

```bash
make compile          # shinro build scenarios/microduck_walking.toml --out build/compiled_policy
```

Then:

```bash
make interop          # or: make -C interop
```

```text
one shinro_step through four languages — same .so, same 61-D input:
  C        ...14 values...
  C++      ...14 values...
  Zig      ...14 values...
  Python   ...14 values...
  => all four agree (bit-identical f64 across the C ABI)
```

The Python line also cross-checks `MicroduckPolicy` — the host this repo ships —
against the raw ABI and prints the agreement to stderr.

## Port layout

From `build/compiled_policy/graph_data_manifest.json`:

| Port  | Elements | Layout |
| ----- | -------- | ------ |
| in    | 61 | the raw observation (see [policy-contract.md](../docs/policy-contract.md)) |
| out   | 14 | `u` — joint-position offsets |
| state | 0  | none: this policy is a memoryless MLP, so the pointer is scratch |

A recurrent or multi-port export would declare `state_*` ports here; the C ABI is
unchanged, the host just feeds each `state_*` output back to its matching input on
the next tick. `MicroduckPolicy` asserts the single-port MLP shape at load and
points non-MLP exports at shinro's generic adapter.

Every language rebuilds the **same synthetic observation** — slot `i` is
`((i % 11) - 5) / 32`, exactly representable, so the inputs are bit-identical and
any output difference is the host's fault, not the input's.

## Files

| Language | File | Build |
| -------- | ---- | ----- |
| C      | `c/kernel_demo.c`         | `cc -O2 -o kernel_demo_c kernel_demo.c -ldl` |
| C++    | `cpp/kernel_demo.cpp`     | `c++ -std=c++17 -O2 -o kernel_demo_cpp kernel_demo.cpp -ldl` |
| Zig    | `zig/kernel_demo.zig`     | `zig build-exe -O ReleaseFast -lc -femit-bin=kernel_demo_zig kernel_demo.zig` |
| Python | `python/kernel_demo.py`   | `python3 python/kernel_demo.py <so>` |
