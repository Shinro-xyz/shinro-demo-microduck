#!/usr/bin/env bash
# Copyright 2026 Shinro SAS. Licensed under the Business Source License 1.1; see LICENSE.
#
# bin/doctor.sh: read the facts of this machine, classify it against the
# published targets, and probe each shipped binary rather than assuming it
# starts. Explain in plain language what will and will not run here.
#
# Exit codes: 0 a published target and every binary runs; 3 this machine is not
# one the release describes (informational); 4 a shipped binary did not start;
# 5 no release binaries found, or usage.
set -uo pipefail
# shellcheck source=lib/common.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

while [ $# -gt 0 ]; do
    case "$1" in
        --flavour) FLAVOUR="${2:?--flavour needs a value}"; shift 2 ;;
        -h | --help) echo "usage: bin/doctor.sh [--flavour trajectory|policy-swap]"; exit 0 ;;
        *) md_fail "unrecognised argument '$1'" 5 ;;
    esac
done
# Which fetched release to probe. Default `trajectory`; the policy-swap demo is
# its own release, so name it to check that bundle instead.
FLAVOUR="${FLAVOUR:-trajectory}"
md_flavour_tag "$FLAVOUR" "$MD_VERSION" >/dev/null ||
    md_fail "unknown --flavour '$FLAVOUR' (trajectory or policy-swap)" 5

md_need_tool timeout "it bounds each binary probe"

md_facts
CLASS="$(md_classify)"
echo "microduck-cake doctor"
echo "  architecture     $MD_ARCH"
echo "  operating system ${MD_OS:-unknown}"
echo "  kernel           $MD_KERNEL"
echo "  page size        $MD_PAGE_SIZE"
echo "  glibc            ${MD_GLIBC:-unknown}"
echo "  release target   ${MD_SLUG:-none published for this architecture}"
echo "  release flavour  $FLAVOUR"
echo "  classification   $CLASS"
echo
echo "This demonstration is a desktop simulation: x86_64 only, at a glibc floor of 2.34."
echo
case "$CLASS" in
    x86_64)
        echo "The full demo runs here, including the offscreen render." ;;
    aarch64)
        echo "A 64-bit Pi, or another aarch64 board. The Cake release is NOT published"
        echo "for it - this demonstration is a desktop simulation. The bare compiled"
        echo "kernel does cross-compile and runs well here (750 us/tick on a Pi 3 B+):"
        echo "  make compile TARGET=aarch64-linux-gnu" ;;
    *-untested-glibc)
        echo "This is a published architecture, but glibc ${MD_GLIBC:-unknown} is below the"
        echo "2.34 floor the release is built for. The binaries may not start." ;;
    unpublished-architecture)
        echo "No release is published for $MD_ARCH. The Cake release is x86_64 only; the"
        echo "compiled kernel also cross-compiles to aarch64." ;;
    *)
        echo "This machine is not one the release describes. See docs/targets.md." ;;
esac
echo

DIR="$(md_binaries_dir "$FLAVOUR")" || { echo "No $FLAVOUR release found for this architecture; run bin/fetch-release.sh --flavour $FLAVOUR first."; exit 5; }
if [ ! -d "$DIR" ]; then
    echo "No release found at $DIR; run bin/fetch-release.sh --flavour $FLAVOUR first."
    exit 5
fi
md_check_binaries "$DIR" "$FLAVOUR"
echo "Shipped binaries under $DIR (one bounded no-op run each):"
BAD=0
for b in cake-resident admin-probe cake-pack; do
    if md_probe_binary "$DIR/bin/$b"; then
        echo "  $b: starts and resolves under this machine's dynamic loader"
    else
        echo "  $b: DID NOT START (dynamic loader or signal failure)"
        BAD=$((BAD + 1))
    fi
done
echo
echo "Next: bin/verify.sh --flavour $FLAVOUR, then tests/smoke.sh."
# If the other release is also fetched, say how to probe it: one flavour per run.
for other in trajectory policy-swap; do
    [ "$other" = "$FLAVOUR" ] && continue
    odir="$(md_binaries_dir "$other" 2>/dev/null)" || continue
    [ -d "$odir" ] || continue
    echo "Also fetched: $other at $odir (bin/doctor.sh --flavour $other)."
done

[ "$BAD" -eq 0 ] || exit 4
[ "$CLASS" = "x86_64" ] || exit 3
exit 0
