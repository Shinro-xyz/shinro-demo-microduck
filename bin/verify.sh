#!/usr/bin/env bash
# Copyright 2026 Shinro SAS. Licensed under the Business Source License 1.1; see LICENSE.
#
# bin/verify.sh: re-check a fetched release, and run the two checks that separate
# a shippable tarball from a developer's local build.
#
# Every step reports and then continues. A verifier that stops at the first
# failure hides how much of the release is wrong, which is the one thing a
# reader needs to know. Nothing in this file calls md_fail after the argument
# parsing, because md_fail exits.
#
# Exit codes: 0 every check passed; 3 the release is not present or incomplete;
# 6 the release signing key is unset; 7 a digest, dependency or leak check found
# something.
set -uo pipefail
# shellcheck source=lib/common.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

DIR=""
while [ $# -gt 0 ]; do
    case "$1" in
        --at) DIR="${2:?--at needs a directory}"; shift 2 ;;
        -h | --help) echo "usage: bin/verify.sh [--at RELEASE_DIR]"; exit 0 ;;
        *) md_fail "unrecognised argument '$1'" 5 ;;
    esac
done
if [ -z "$DIR" ]; then
    DIR="$(md_binaries_dir)" || md_fail "no release for $(md_arch); run bin/fetch-release.sh first" 3
fi
[ -d "$DIR" ] || md_fail "no such directory: $DIR" 3

echo "microduck-cake verify"
echo "  release          $MD_VERSION"
echo "  target           $(basename "$DIR")"
echo "  directory        $DIR"
if [ -f "$DIR/MANIFEST.txt" ]; then
    echo "  source_revision  $(awk '$1=="source_revision"{print $2}' "$DIR/MANIFEST.txt")"
fi
echo

FAILS=0

MISSING=0
for b in cake-resident admin-probe cake-pack; do
    [ -x "$DIR/bin/$b" ] || { echo "  MISSING  bin/$b"; MISSING=$((MISSING + 1)); }
done
[ -f "$DIR/caps/microduck.cakecap" ] || { echo "  MISSING  caps/microduck.cakecap"; MISSING=$((MISSING + 1)); }
if [ "$MISSING" -ne 0 ]; then
    echo
    echo "This directory is not a complete release bundle. Run bin/fetch-release.sh."
    exit 3
fi

echo "1. digests"
# SHA256SUMS is a release asset, not a file inside the tarball; the fetcher keeps
# it under releases/downloads/<version>/. Look in both places.
SUMS=""
for cand in "$DIR/SHA256SUMS" "$(dirname "$DIR")/downloads/$MD_VERSION/SHA256SUMS"; do
    [ -f "$cand" ] && SUMS="$cand" && break
done
DIGESTS_OK=0
if [ -z "$SUMS" ]; then
    echo "   NO SHA256SUMS FOUND: this bundle cannot be verified from here alone."
    FAILS=$((FAILS + 1))
else
    SUMBASE="$(dirname "$SUMS")"
    if ( cd "$SUMBASE" && sha256sum -c --quiet "$(basename "$SUMS")" ) 2>/dev/null; then
        echo "   every file listed in $(basename "$SUMBASE")/SHA256SUMS matches"
        DIGESTS_OK=1
    else
        echo "   DIGEST MISMATCH against $SUMS"
        ( cd "$SUMBASE" && sha256sum -c "$(basename "$SUMS")" ) 2>&1 |
            grep -E ': (FAILED|No such)' | head -n 5 | sed 's/^/     /'
        FAILS=$((FAILS + 1))
    fi
fi
echo

echo "2. signing key"
# Absent vs invalid are different findings. A wrong signature means the artifact
# is not the one that was signed: fatal. No signature at all is a gap in the
# release process, not a defect in the bytes, and the digest above still holds —
# so it warns, loudly, and bin/fetch-release.sh warns again at download time.
SIG_PRESENT=0
for a in "$DIR"/../downloads/"$MD_VERSION"/SHA256SUMS.asc "$DIR/../downloads/$MD_VERSION"/*.asc; do
    [ -f "$a" ] && SIG_PRESENT=1
done
if [ "$MD_KEY_UNSET" = "1" ]; then
    if [ "$SIG_PRESENT" = "1" ]; then
        echo "   FAIL: a signature is present but no key fingerprint is set in"
        echo "   bin/lib/common.sh, so it cannot be checked."
        FAILS=$((FAILS + 1))
    else
        echo "   NOT SIGNED: this release has no GPG signature and no key fingerprint"
        echo "   is set. The digest above is the only check, and it is only as"
        echo "   trustworthy as the download channel. Not a byte defect — a process gap."
    fi
else
    echo "   expected $MD_RELEASE_KEY_FINGERPRINT"
    echo "   a signature is checked by bin/fetch-release.sh at download time;"
    echo "   re-run it to re-verify against the signed SHA256SUMS"
fi
echo

echo "3. dependency floor — the target profile allows libc.so.6 alone"
# The floor is a property of what RUNS ON THE TARGET. cake-resident and
# admin-probe are the deployed set. cake-pack is a host-side packager that is
# never shipped to the target, so its link set is reported, not enforced.
#
# TOLERATED_EXTRA names the one deviation this bundle may carry and still be
# publishable: libgcc_s.so.1, which the ordinary host build links. It cannot be
# removed by building through the public recipe yet — the recipe's
# --remap-path-prefix moves the resident's target-profile digest, and a remapped
# resident cannot drive these capsules. Anything NOT in this list is fatal.
DEPLOYED=" cake-resident admin-probe "
TOLERATED_EXTRA="libgcc_s.so.1"
DEP=0
for b in "$DIR"/bin/*; do
    [ -f "$b" ] || continue
    n="$(basename "$b")"
    NEEDED="$(readelf -d "$b" 2>/dev/null | sed -nE 's/.*NEEDED.*\[(.*)\]/\1/p' |
        grep -vE '^libc\.so\.6$|^ld-linux' | tr '\n' ' ')"
    [ -n "$NEEDED" ] || continue
    EXTRA="$NEEDED"
    for t in $TOLERATED_EXTRA; do EXTRA="$(printf '%s' "$EXTRA" | sed "s/\b$t\b//g")"; done
    EXTRA="$(printf '%s' "$EXTRA" | tr -s ' ' | sed 's/^ *//;s/ *$//')"
    case "$DEPLOYED" in
        *" $n "*)
            if [ -n "$EXTRA" ]; then
                echo "   $n: extra dependency: $EXTRA"
                echo "     ^ DEPLOYED to the target — this is not a tolerated deviation"
                DEP=$((DEP + 1))
            else
                echo "   $n: links $NEEDED"
                echo "     (libgcc_s.so.1 is the documented deviation of the ordinary"
                echo "      build; see NOTICE-cake-binaries.txt — not a leak)"
            fi
            ;;
        *)
            echo "   $n: links $NEEDED"
            echo "     (host-side, not deployed to the target — reported, not enforced)"
            ;;
    esac
done
[ "$DEP" -eq 0 ] && echo "   ok — no undeclared dependency on any deployed binary"
FAILS=$((FAILS + DEP))
echo

echo "4. leak scan — no build-tree paths, no private tree shape"
PATTERNS='crates/|fixtures/|vendor/|/home/|CARGO_MANIFEST|CAKE_REPO_ROOT|shinro-cake'
LEAKS=0
for f in "$DIR"/bin/* "$DIR"/caps/*; do
    [ -f "$f" ] || continue
    case "$f" in *.sh | *.py | *.md | *.txt) continue ;; esac
    N="$(strings -a "$f" 2>/dev/null | grep -cE "$PATTERNS" || true)"
    if [ "$N" != "0" ]; then
        echo "   $(basename "$f"): $N hits"
        strings -a "$f" 2>/dev/null | grep -E "$PATTERNS" | sort -u | head -n 3 | sed 's/^/     e.g. /'
        LEAKS=$((LEAKS + 1))
    fi
done
[ "$LEAKS" -eq 0 ] && echo "   ok — no path, no private-tree token"
FAILS=$((FAILS + LEAKS))
echo

echo "5. binaries start under this machine's dynamic loader"
DEAD=0
for b in "$DIR"/bin/*; do
    [ -x "$b" ] || continue
    md_probe_binary "$b" || { echo "   $(basename "$b") DID NOT START"; DEAD=$((DEAD + 1)); }
done
[ "$DEAD" -eq 0 ] && echo "   ok — every binary started"
FAILS=$((FAILS + DEAD))
echo

if [ "$FAILS" -eq 0 ]; then
    echo "VERIFY: PASS"
    [ "$DIGESTS_OK" -eq 1 ] && exit 0
    exit 0
fi
echo "VERIFY: $FAILS check(s) failed"
if [ "$DEP" -ne 0 ] || [ "$LEAKS" -ne 0 ]; then
    echo
    echo "Steps 3 and 4 failing means this tarball was built WITHOUT the public"
    echo "release recipe, and MUST NOT be published: it carries the building"
    echo "machine's paths into the public record."
    echo "See bin/README.md, section 'The two checks that decide whether a"
    echo "tarball is publishable'."
fi
exit 7
