# cake/ — the Microduck demo on the Cake control plane

This directory runs the Microduck walking policy as a **Cake module** instead of
through the Linux `dlopen` host: the Cake resident (Cake's control plane) admits
the policy capsule, the module owns its own thread, and this repository's MuJoCo
client is the plant across a Unix-domain socket.

It consumes a **precompiled Cake release**. Nothing here compiles a Cake module,
and no Cake source, ABI header or descriptor lives in this repository:

```
shinro-demo-microduck/cake/          shinro-bench/cake/ (private)        shinro-cake (private)
  fetch-release.sh     ──fetch──►    release/build-bundle.sh  ◄──build──  demo/build-public.sh
  run-microduck.sh                   src/sim_policy.zig                  (the resident rows)
  lib/common.sh                      demo/microduck.cmdl
  VERSION
```

## Use it

```bash
make fetch-cake          # download + verify the release for this target
make cake                # circle trajectory, 8 s
make cake TRAJ=slalom SECS=8
make cake SWAP=circle,eight,slalom SECS=9   # switch trajectory live, no restart
```

Or directly:

```bash
cake/fetch-release.sh            # detects x86_64 or aarch64
cake/run-microduck.sh --seconds 8 --command forward
```

Both targets are published — `x86_64` (a MuJoCo desktop) and `aarch64` (a
64-bit Pi). `fetch-release.sh` picks the tarball for `uname -m`.

The trajectory is a **swappable Cake module**: `make cake SWAP=circle,eight,slalom`
switches it live with `replace-slot` while the robot walks, and the resident never
restarts. `make cake` also renders a GIF of the walk (the path coloured by
trajectory module with its tracking error, swaps marked) and a figure of how Cake
ran each swap with the same cross-track error. See
[Switch trajectories live](../docs/cake-control-plane.md#switch-trajectories-live-without-turning-off-cake).

## The release, and why it is precompiled

The bundle contains the target-matched `cake-resident`, `admin-probe` and
`cake-pack`, plus `caps/microduck.cakecap` — the policy module compiled from the
same ONNX this repo already ships. The producer is the private
`shinro-bench/cake/release/build-bundle.sh`; the exact layout and the publishing
steps are in its [`README`](../../shinro-bench/cake/release/README.md).

The boundary this keeps:

| here (public) | the private Cake tree |
| --- | --- |
| fetch + verify the release | build the resident for each target |
| stage the Plan, start the resident, run MuJoCo | compile the policy capsule against the ABI |
| the MuJoCo client (`../demos/demo_cake_policy.py`) | the module, descriptors, `cake-pack` |

The wire between the module and the client is just two little-endian f64 arrays
— the graph's own port layout (61 f64 in, 14 f64 out). No `cake_*` header,
symbol or lifecycle callback reaches this repository, and no MuJoCo reaches the
private tree.

## Files

| path | role |
| --- | --- |
| `VERSION` | the release version this checkout fetches |
| `lib/common.sh` | target detection, bundle resolution, shared helpers |
| `fetch-release.sh` | download + verify + extract the bundle under `release/<slug>/` |
| `run-microduck.sh` | stage the deployment and run the walk |
| `keys/` | the release signing public key (added with the first signed release) |
| `releases/<version>/SHA256SUMS` | the committed checksum list (added at release) |
| `release/` | fetched bundles (gitignored) |

## Verifying a download

`fetch-release.sh` checks the tarball's SHA-256 against `SHA256SUMS`, and
compares `SHA256SUMS` against the copy committed under `releases/<version>/`
when this is a repository checkout. Once `keys/` carries the release key it also
verifies the GPG signature over `SHA256SUMS`. Nothing is extracted before those
checks pass.
