#!/usr/bin/env bash
# Copyright 2026 Shinro SAS. Licensed under the Business Source License 1.1; see LICENSE.
#
# bin/fetch-release.sh: fetch and verify the precompiled Cake release for this
# target, and extract it under releases/<slug>/.
#
# The release is a four-item bundle: the Cake resident (the control plane), the
# admin probe, the tooling that stages the deployment, and the Microduck policy
# and trajectory capsules. Nothing is extracted before the digest checks pass.
#
# Options:
#   --target SLUG     x86_64 (the only published target)
#   --version vX.Y.Z  (default: this checkout's VERSION)
#   --from DIR        use a tarball already downloaded into DIR (offline)
set -uo pipefail
# shellcheck source=lib/common.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

SLUG=""
FROM=""
FLAVOUR="trajectory"
VERSION="$MD_VERSION"
while [ $# -gt 0 ]; do
    case "$1" in
        --target) SLUG="${2:?--target needs a slug}"; shift 2 ;;
        --flavour) FLAVOUR="${2:?--flavour needs a value}"; shift 2 ;;
        --version) VERSION="${2:?--version needs a value}"; shift 2 ;;
        --from) FROM="${2:?--from needs a directory}"; shift 2 ;;
        -h | --help) echo "usage: bin/fetch-release.sh [--target x86_64] [--flavour trajectory|policy-swap|walk-swap] [--version vX.Y.Z] [--from DIR]"; exit 0 ;;
        *) md_fail "unrecognised argument '$1'" 5 ;;
    esac
done
md_flavour_tag "$FLAVOUR" "$VERSION" >/dev/null || md_fail "unknown --flavour '$FLAVOUR' (trajectory, policy-swap or walk-swap)" 5

md_need_tool tar "install tar"
md_need_tool sha256sum "coreutils ships it"
md_need_tool curl "it downloads the release"
md_need_tool timeout "it bounds every network wait"

TARGET_EXPLICIT=0
if [ -z "$SLUG" ]; then
    SLUG="$(md_target_slug)"
    [ -n "$SLUG" ] || md_fail "$(md_arch) is not a published target; this demonstration is a desktop (x86_64) simulation" 3
    md_say "detected $(md_arch), target $SLUG"
else
    TARGET_EXPLICIT=1
fi
case "$SLUG" in
    x86_64) : ;;
    aarch64)
        # Only reached by explicit --target now. Say the thing that is true,
        # rather than letting the download 404 and look like a broken release.
        [ "$TARGET_EXPLICIT" = "1" ] || md_fail "no Cake release is published for aarch64. This demonstration is a desktop (x86_64) simulation: the Cake control plane is not published for a Raspberry Pi or any other board. The *compiled kernel* can still be cross-compiled with 'make compile TARGET=aarch64-linux-gnu' — that is the bare .so path, not this release." 3
        md_warn "aarch64 was named explicitly; there is no published bundle for it, so this will almost certainly 404"
        ;;
    *) md_fail "unknown target slug '$SLUG' (this demonstration publishes x86_64 only)" 5 ;;
esac

TARBALL="$(md_tarball_name "$VERSION" "$SLUG" "$FLAVOUR")"
DEST="$MD_RELEASE_DIR/$(md_flavour_slug "$FLAVOUR" "$SLUG")"
# Each flavour is its own release, so it has its own SHA256SUMS; keep the
# download cache per flavour too, or the two would overwrite each other.
DL="$MD_RELEASE_DIR/downloads/$(md_flavour_slug "$FLAVOUR" "$VERSION")"
mkdir -p "$DL" || md_fail "cannot create $DL" 5

if [ -n "$FROM" ]; then
    FROM="$(cd "$FROM" && pwd)" || md_fail "--from $FROM is not a directory" 5
    for a in "$TARBALL" SHA256SUMS; do
        [ -f "$FROM/$a" ] || md_fail "$FROM/$a is missing" 5
        [ "$FROM" = "$DL" ] || cp -- "$FROM/$a" "$DL/" || md_fail "cannot copy $a" 5
    done
    for a in "$TARBALL.asc" SHA256SUMS.asc; do
        [ -f "$FROM/$a" ] && cp -- "$FROM/$a" "$DL/" 2>/dev/null
    done
else
    BASE="https://github.com/$MD_REPO/releases/download/$(md_flavour_tag "$FLAVOUR" "$VERSION")"
    # Bounded: a hung TLS handshake must not look like a slow download.
    for a in "$TARBALL" SHA256SUMS; do
        if [ -f "$DL/$a" ]; then
            md_say "have $a"
            continue
        fi
        md_say "downloading $VERSION ($SLUG): $a"
        timeout 300 curl -fsSL -o "$DL/$a.part" "$BASE/$a" || md_fail "$a could not be downloaded from $BASE/$a" 4
        mv -- "$DL/$a.part" "$DL/$a"
    done
    for a in "$TARBALL.asc" SHA256SUMS.asc; do
        timeout 60 curl -fsSL -o "$DL/$a" "$BASE/$a" 2>/dev/null || rm -f -- "$DL/$a"
    done
fi

# --- verify before extracting anything ----------------------------------------

SUM_LINE="$(awk -v want="$TARBALL" '$2 == want || $2 == "./" want {print $1}' "$DL/SHA256SUMS" | head -n 1)"
[ -n "$SUM_LINE" ] || md_fail "$DL/SHA256SUMS does not name $TARBALL" 6
printf '%s  %s\n' "$SUM_LINE" "$TARBALL" >"$DL/.expect"
( cd "$DL" && sha256sum -c --quiet .expect ) || md_fail "$TARBALL does not match its digest in SHA256SUMS" 6
rm -f -- "$DL/.expect"
md_say "digest OK: $SUM_LINE"

KEYFILE="$MD_KEYS_DIR/microduck-release-signing.pub.asc"
if [ "$MD_KEY_UNSET" = "1" ]; then
    md_warn "no release signing key fingerprint is set in bin/lib/common.sh, so the signature is NOT checked"
    md_warn "the SHA-256 digest above is the check, and it is only as trustworthy as the channel it came over"
elif [ -f "$DL/SHA256SUMS.asc" ] && [ -f "$KEYFILE" ]; then
    md_need_tool gpg "it verifies the release signature"
    GNUPGHOME="$(mktemp -d)"; export GNUPGHOME
    trap 'rm -rf -- "$GNUPGHOME"' EXIT
    gpg --batch --quiet --import "$KEYFILE" 2>/dev/null || md_fail "$KEYFILE could not be imported" 6
    GOT="$(gpg --batch --with-colons --import-options show-only --import "$KEYFILE" 2>/dev/null |
        awk -F: '/^fpr:/{print $10; exit}')"
    [ "$GOT" = "$MD_RELEASE_KEY_FINGERPRINT" ] ||
        md_fail "the signing key in $KEYFILE is $GOT, which is not the expected $MD_RELEASE_KEY_FINGERPRINT" 6
    gpg --batch --quiet --verify "$DL/SHA256SUMS.asc" "$DL/SHA256SUMS" 2>/dev/null ||
        md_fail "SHA256SUMS does not carry a valid signature from $MD_RELEASE_KEY_FINGERPRINT" 6
    md_say "signature OK: $MD_RELEASE_KEY_FINGERPRINT"
else
    md_warn "release signature not checked (no $KEYFILE, or no SHA256SUMS.asc); the digest above is the check"
fi

# --- extract ------------------------------------------------------------------

rm -rf -- "$DEST"
mkdir -p "$DEST" || md_fail "cannot create $DEST" 5
tar xzf "$DL/$TARBALL" -C "$DEST" --strip-components=1 || md_fail "$TARBALL did not extract" 5
md_check_binaries "$DEST" "$FLAVOUR"
md_say "release $VERSION ($FLAVOUR) for $SLUG is at $DEST"
md_say "next: bin/doctor.sh"
