*Part of [shinro-demo-microduck](../README.md).*

# Targets

**This is a desktop simulation.** It runs on 64-bit Linux on `x86_64`, and that is the
only architecture the Cake release is published for.

| | |
| --- | --- |
| architecture | `x86_64` |
| libc | glibc >= 2.34 |
| display | not required — the default run renders offscreen |
| MuJoCo | 3.16.0, from the pinned extras |

`bin/doctor.sh` classifies the machine before anything else runs, and prints which of
these it found:

| classification | meaning |
| -------------- | ------- |
| `x86_64` | the release runs here, including the offscreen render |
| `*-untested-glibc` | right architecture, glibc below the floor — the binaries may not start |
| `unpublished-architecture` | no release is published for this machine |
| anything else | not a machine the release describes |

`bin/fetch-release.sh` resolves the target from `uname -m`, and refuses anything that is
not `x86_64` with an explanation rather than letting the download 404.

## What is deliberately not a target

**`aarch64`.** There is no Pi bundle and there will not be one: the Cake control plane is
demonstrated as a desktop simulation. A user on a Pi is told that plainly rather than
handed a 404.

That is a decision about the *Cake release*, not about the compiled kernel. The kernel
itself does cross-compile and is fast on small hardware — the bare `.so` path measures
**750 us/tick on a Raspberry Pi 3 B+** (see [footprint.md](footprint.md)):

```bash
make compile TARGET=aarch64-linux-gnu
```

That produces a `.so` and drives it through the demo's own host. It does not give you the
Cake release — the resident, the capsules and the update transaction — which is the part
this repository exists to demonstrate.
