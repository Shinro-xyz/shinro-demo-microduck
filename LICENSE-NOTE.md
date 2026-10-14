<!-- Copyright 2026 Shinro SAS. Licensed under the Business Source License 1.1; see LICENSE. -->

# License note for maintainers

`LICENSE` is the Business Source License 1.1. Its three parameters were set by
Shinro SAS on 2026-10-14, replacing the Apache License 2.0 this repository
carried before its release.

| Parameter | Value | Constraint from the license's own Covenants |
| --- | --- | --- |
| Additional Use Grant | personal, educational and non-commercial use permitted; commercial use requires a separate license (full text in `LICENSE`) | must only add rights, never restrict the rights the base license already grants |
| Change Date | 2030-10-14 | a date; the license also converts automatically four years after first public distribution of a version, whichever comes first |
| Change License | Apache License, Version 2.0 | GPL Version 2.0 or any later version, or a license compatible with it |

## Why BSL and not Apache-2.0 — the reason is the releases, not the scripts

The demo scripts here are thin glue, and on their own Apache-2.0 would have been
a defensible, even generous choice. What decides it is that **this repository is
the distribution channel for the compiled Cake binaries and capsules.** The
`LICENSE` at the root of a repository is the grant under which its release
assets are offered. Under Apache-2.0 this repository would have been handing out
an unconditional, perpetual, royalty-free, commercial-use grant over the
compiled output of a closed, commercially licensed kernel. That is not a
licensing preference; it is the mechanism.

The sibling public demo, `shinro-dev/kiwi-cake-demo`, reached the same
conclusion for the same reason and is the precedent this file follows.

**If the team would rather this repository be Apache-2.0**, the correct change is
not to relicense this one — it is to move the release assets to a repository
whose licence is BSL, and let this one be permissive. Relicensing in place while
still publishing the capsules here would re-open exactly the hole this closes.

## Set the Change Date in the future, and check it at publication

The sibling demo's v0.1.0 shipped a Change Date **already in the past on the day
it was published**. Because the BSL converts on the Change Date or the fourth
anniversary, whichever comes first, a date in the past means the BSL terms had
already expired as published and the Additional Use Grant was void. It cost them
a release: every release tarball embeds `LICENSE`, so correcting the date alone
required a new version.

Before the first release, confirm:

- the Change Date is **in the future**, and no later than four years after this
  version's first public distribution under this License;
- every release tarball embeds this `LICENSE`, and the date inside the tarball
  matches the date here;
- `NOTICE` and `THIRD_PARTY_LICENSES/` still describe the bytes actually shipped.

## What this licence is not

- **It does not license the Cake kernel.** The kernel is closed source,
  commercially copyrighted, and distributed as-is; nothing here grants any right
  in it. See `NOTICE` §2 and https://shinro.dev/licensing.
- **It does not cover Pollen Robotics' material.** The Microduck MJCF, mesh set
  and walking policy are Apache-2.0 and are explicitly excluded from the
  Licensed Work above. See `NOTICE` §3. Nothing here restricts any right Apache-2.0
  already grants you in them.
- **It does not resolve the framework's licence.** `shinro-python-modules`, which
  the demo's install step depends on, still ships a **zero-byte `LICENSE`**, which
  means all rights reserved by default. That is a separate decision, on a
  different repository, with different reasoning — there is a written argument
  that MPL 2.0 fits a code generator that *emits source code* better than either
  Apache-2.0 or BSL, because it needs a generated-source exception either way.
  Until that is settled, a user following the demo's install step is installing a
  package nobody has been granted rights to use.

## Why Apache License 2.0 satisfies the Change License covenant

The covenant (`LICENSE`, "Covenants of Licensor," item 1) requires a license
"compatible with GPL Version 2.0 or a later version," where compatible means
software under the Change License can be included in a program with software
under GPLv2 or a later version. The Apache Software Foundation's own
compatibility statement (https://www.apache.org/licenses/GPL-compatibility.html)
confirms Apache License 2.0 code can be included in a GPLv3 program, though not a
GPLv2-only one. Because GPLv3 is "a later version" of GPL Version 2.0, that
one-way compatibility satisfies the covenant as written. Verified by fetching the
Apache Software Foundation's page directly rather than assumed.

The license text itself is reproduced verbatim from the MariaDB publication of
BSL 1.1 and must not be edited beyond the Parameters block.
