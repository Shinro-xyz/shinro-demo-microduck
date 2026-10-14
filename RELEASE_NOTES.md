# v0.1.0 — the Cake bundle for the Microduck demo

The precompiled Cake release `shinro-demo-microduck` fetches. Two assets:

| asset | |
| --- | --- |
| `microduck-cake-v0.1.0-x86_64.tar.gz` | the bundle |
| `SHA256SUMS` | its digest, and the check `bin/fetch-release.sh` makes |

## What is in it

- `bin/cake-resident` — the control plane
- `bin/admin-probe` — the admin client the demo queries slots and sessions with
- `bin/cake-pack` — stages the Plan
- `caps/microduck.cakecap` — the walking policy, compiled from Pollen Robotics' published ONNX
- `caps/trajectory_{forward,circle,eight,slalom}.cakecap` and their four `_anchor` variants — the swappable trajectory modules
- `LICENSE`, `NOTICE-cake-binaries.txt`, `VERSION`

`x86_64` only. This demonstration is a **desktop simulation** by design, so there is no
`aarch64` bundle and the Raspberry Pi row of `docs/targets.md` does not apply to the Cake
release; `bin/doctor.sh` says so plainly on a Pi. (The bare compiled kernel does still
cross-compile to `aarch64`.)

## Try it

```sh
git clone https://github.com/Shinro-xyz/shinro-demo-microduck.git
cd shinro-demo-microduck
make install          # the pinned shinro wheel + the demo's extras
make fetch-cake       # this release
bin/doctor.sh         # can this machine run the binaries?
make cake             # circle -> eight -> slalom, swapped live, rendered
```

`make cake` should end in `microduck-cake: microduck Cake demo: PASS`, having replaced
the trajectory module twice mid-walk with the resident's session identity unchanged.

## Known deviations, stated rather than hidden

**The dependency floor.** The ratified target profile allows `libc.so.6` alone. These
three binaries also link `libgcc_s.so.1`, because they come from the ordinary host
build. They cannot yet be rebuilt through the public release recipe: that recipe's
`--remap-path-prefix` **moves the resident's target-profile digest**, and a remapped
resident is refused by these capsules with `plan-target-profile-mismatch`. Removing
`libgcc_s.so.1` therefore requires fixing the digest derivation first.

**Build-tree paths are redacted in place.** The ordinary build bakes the building
user's home directory and the private crate layout into panic-location strings —
42 in `cake-resident`, 15 in `cake-pack`, 8 in `admin-probe`. Those have been
**redacted in place, byte-for-byte**, preserving every length and offset, which
removes the paths **without** moving the target-profile digest. `NOTICE-cake-binaries.txt`
inside the tarball states this. The consequence, plainly: **this artifact is not
byte-reproducible from the ordinary build alone** — reproducing it needs this redaction
step. Neither the redaction nor the recipe's remapping is a security boundary; the
kernel's source stays private by policy.

**No signature.** `SHA256SUMS` is not GPG-signed: the release key's fingerprint is still
unset in `bin/lib/common.sh`, so `bin/fetch-release.sh` verifies the digest and
**prints a warning saying the signature was not checked**. Treat the digest as the
check, and the download channel as the trust.

**No `MANIFEST.txt`.** This release does not carry a per-file manifest, so it does not
record the private `source_revision` the binaries were built from. That is worth adding
for the next one.

## Attribution

The Microduck MJCF, mesh set and walking policy are **Pollen Robotics'** work, Apache
License 2.0, redistributed here with attribution. The Cake kernel itself is Shinro SAS's,
commercially licensed and closed source.

## Licence

Business Source License 1.1 — see `LICENSE`. Not an open-source licence. Personal,
educational and non-commercial use is granted; commercial use requires a separate
licence. Converts to Apache License 2.0 on **2030-10-14**.
