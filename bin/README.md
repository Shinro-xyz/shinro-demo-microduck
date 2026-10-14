# bin/ — the Microduck demo on the Cake control plane

This directory runs the Microduck walking policy as a **Cake module** instead of
through the Linux `dlopen` host: the Cake resident (Cake's control plane) admits
the policy capsule, the module owns its own thread, and this repository's MuJoCo
client is the plant across a Unix-domain socket.

It consumes a **precompiled Cake release**. Nothing here compiles a Cake module,
and no Cake source, ABI header or descriptor lives in this repository:

```
shinro-demo-microduck/bin/            shinro-bench/cake/ (private)      shinro-cake (private)
  fetch-release.sh     ──fetch──►     release/build-bundle.sh  ◄──build── demo/build-public.sh
  run-demo.sh                         src/sim_policy.zig                (the resident rows)
  doctor.sh  verify.sh
  lib/common.sh
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
bin/fetch-release.sh                          # detects x86_64 or aarch64
bin/doctor.sh                                # can this machine run the binaries?
bin/verify.sh                                # re-check the release, and gate it
bin/run-demo.sh --seconds 8 --trajectory circle
```

Both targets are published — `x86_64` (a MuJoCo desktop) and `aarch64` (a 64-bit
Pi). `fetch-release.sh` picks the tarball for `uname -m`.

The trajectory is a **swappable Cake module**: `make cake SWAP=circle,eight,slalom`
switches it live with `replace-slot` while the robot walks, and the resident never
restarts. `make cake` also renders a GIF of the walk (the path coloured by
trajectory module with its tracking error, swaps marked) and a figure of how Cake
ran each swap with the same cross-track error. See
[Switch trajectories live](../docs/cake-control-plane.md#switch-trajectories-live-without-turning-off-cake).

## The release, and why it is precompiled

The bundle contains the target-matched `cake-resident`, `admin-probe` and
`cake-pack`, plus `caps/microduck.cakecap` — the policy module compiled from the
same ONNX this repo already ships — and the trajectory capsules. The producer is
the private `shinro-bench/cake/release/build-bundle.sh`; the exact layout and the
publishing steps are in its README.

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
| `../VERSION` | the release version this checkout fetches |
| `lib/common.sh` | target detection, bundle resolution, bounded admin queries, shared helpers |
| `fetch-release.sh` | download + verify + extract the bundle under `releases/<slug>/` |
| `doctor.sh` | classify this machine against the published targets, and probe each binary |
| `verify.sh` | re-check a fetched release, and run the two checks below |
| `run-demo.sh` | stage the deployment and run the walk |
| `keys/` | the release signing public key (added with the first signed release) |
| `releases/` | fetched bundles (gitignored) |
| `state/` | run output: trace, swap records, media (gitignored) |

## Verifying a download

`fetch-release.sh` checks the tarball's SHA-256 against `SHA256SUMS` before
extracting anything. When `MD_RELEASE_KEY_FINGERPRINT` in `lib/common.sh` is set
and `keys/` carries the release key, it also verifies the GPG signature over
`SHA256SUMS` against that fingerprint. **If the fingerprint is unset it says so
loudly**, because a digest is only as trustworthy as the channel it came over —
and an unset fingerprint is a release blocker, not a default.

## The two checks that decide whether tarball is publishable

`bin/verify.sh` runs both. They are the reason this directory has a `verify`
script at all, and neither is visible from inside a build tree.

1. **The dependency floor.** `readelf -d bin/cake-resident | grep NEEDED` must
   name `libc.so.6` and the dynamic loader, and nothing else. An extra
   `libgcc_s.so.1` means the build did not go through the public release recipe.

2. **The leak scan.** No shipped binary or capsule may contain `crates/`,
   `fixtures/`, `vendor/`, `/home/`, `CARGO_MANIFEST`, `CAKE_REPO_ROOT` or
   `shinro-cake`. A binary that fails this exposes the machine it was built on
   — including the building user's home directory — to anyone who downloads it.

A tarball that fails either one **must not be published**. `bin/verify.sh`
refuses it and says so.
