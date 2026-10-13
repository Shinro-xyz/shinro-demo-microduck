#!/usr/bin/env bash
# Copyright 2026 Shinro SAS. Licensed under the Business Source License 1.1; see LICENSE.
#
# cake/fetch-release.sh: fetch and verify the precompiled Cake release for this
# target, and extract it under cake/release/<slug>/.
#
# The release is the same four-binary bundle the LeKiwi demo (kiwi-cake-demo)
# publishes: the Cake resident (the control plane), the admin probe, and the
# tooling that stages the deployment, plus the Microduck policy capsule. Nothing
# is extracted before the digest checks pass.
#
# Options:
#   --target SLUG    x86_64 | aarch64 (default: detected)
#   --version vX.Y.Z (default: this checkout's VERSION)
#   --from DIR       use a tarball already downloaded into DIR (offline)
set -uo pipefail
# shellcheck source=lib/common.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

SLUG=""
FROM=""
VERSION="$MD_VERSION"
while [ $# -gt 0 ]; do
    case "$1" in
        --target) SLUG="${2:?--target needs a slug}"; shift 2 ;;
        --version) VERSION="${2:?--version needs a value}"; shift 2 ;;
        --from) FROM="${2:?--from needs a directory}"; shift 2 ;;
        -h | --help) echo "usage: cake/fetch-release.sh [--target x86_64|aarch64] [--version vX.Y.Z] [--from DIR]"; exit 0 ;;
        *) md_fail "unrecognised argument '$1'" 5 ;;
    esac
done

md_need_tool tar "install tar"
md_need_tool sha256sum "coreutils ships it"

if [ -z "$SLUG" ]; then
    SLUG="$(md_target_slug)"
    [ -n "$SLUG" ] || md_fail "$(md_arch) is not a published target; x86_64 and aarch64 are (pass --target if you know what you are doing)" 3
    md_say "detected $(md_arch), target $SLUG"
fi
case "$SLUG" in
    x86_64 | aarch64) : ;;
    *) md_fail "unknown target slug '$SLUG' (x86_64 or aarch64)" 5 ;;
esac

TARBALL="$(md_tarball_name "$VERSION" "$SLUG")"
DEST="$MD_RELEASE_DIR/$SLUG"
DL="$MD_RELEASE_DIR/downloads/$VERSION"
mkdir -p "$DL" || md_fail "cannot create $DL" 5

if [ -n "$FROM" ]; then
    FROM="$(cd "$FROM" && pwd)" || md_fail "--from $FROM is not a directory" 5
    for a in "$TARBALL" SHA256SUMS; do
        [ -f "$FROM/$a" ] || md_fail "$FROM/$a is missing" 5
        [ "$FROM" = "$DL" ] || cp -- "$FROM/$a" "$DL/" || md_fail "cannot copy $a" 5
    done
    [ -f "$FROM/$TARBALL.asc" ] && cp -- "$FROM/$TARBALL.asc" "$DL/" 2>/dev/null
    [ -f "$FROM/SHA256SUMS.asc" ] && cp -- "$FROM/SHA256SUMS.asc" "$DL/" 2>/dev/null
else
    BASE="https://github.com/$MD_REPO/releases/download/$VERSION"
    md_need_tool curl "install curl, or use --from DIR"
    md_say "downloading $VERSION ($SLUG)"
    for a in "$TARBALL" SHA256SUMS "$TARBALL.asc" SHA256SUMS.asc; do
        curl -fsSL --retry 3 -o "$DL/$a" "$BASE/$a" 2>/dev/null || md_warn "no asset $a in $VERSION"
    done
fi

[ -f "$DL/$TARBALL" ] || md_fail "$DL/$TARBALL was not downloaded" 2
[ -f "$DL/SHA256SUMS" ] || md_fail "$DL/SHA256SUMS was not downloaded" 2

# --- digest ---------------------------------------------------------------------
expected="$(awk -v f="$TARBALL" '$2 == f || $2 == "*" f { print $1; exit }' "$DL/SHA256SUMS")"
[ -n "$expected" ] || md_fail "SHA256SUMS carries no line for $TARBALL" 2
actual="$(sha256sum "$DL/$TARBALL" | cut -d' ' -f1)"
[ "$actual" = "$expected" ] || md_fail "digest MISMATCH for $TARBALL (expected $expected, got $actual)" 2
md_say "digest ok: $TARBALL"

# The committed checksum list, when this is a repository checkout, is a second
# copy to compare the download against.
COMMITTED="$MD_CAKE_DIR/releases/$VERSION/SHA256SUMS"
if [ -f "$COMMITTED" ]; then
    cmp -s -- "$DL/SHA256SUMS" "$COMMITTED" ||
        md_fail "SHA256SUMS differs from the copy committed under cake/releases/$VERSION" 2
    md_say "SHA256SUMS equals cake/releases/$VERSION/SHA256SUMS"
fi

# --- signature (optional until the release key is in place) ---------------------
KEYFILE="$MD_KEYS_DIR/shinro-release-signing.pub.asc"
if [ -f "$DL/SHA256SUMS.asc" ] && [ -f "$KEYFILE" ] && ! grep -q 'PLACEHOLDER' "$KEYFILE" 2>/dev/null; then
    md_need_tool gpg "install gnupg"
    GNUPGTMP="$(mktemp -d "${TMPDIR:-/tmp}/microduck-gpg.XXXXXX")" || md_fail "cannot create a temporary keyring" 5
    chmod 700 "$GNUPGTMP"
    trap 'rm -rf -- "$GNUPGTMP"' EXIT
    gpg --homedir "$GNUPGTMP" --batch --quiet --no-default-keyring \
        --keyring "$GNUPGTMP/release.kbx" --import "$KEYFILE" 2>/dev/null ||
        md_fail "the release key could not be imported" 4
    status="$(gpg --homedir "$GNUPGTMP" --batch --no-default-keyring --keyring "$GNUPGTMP/release.kbx" \
        --status-fd 1 --verify "$DL/SHA256SUMS.asc" "$DL/SHA256SUMS" 2>/dev/null)"
    printf '%s\n' "$status" | grep -q '^\[GNUPG:\] GOODSIG ' ||
        md_fail "the signature over SHA256SUMS is bad or missing" 3
    md_say "signature ok: SHA256SUMS"
else
    md_warn "release signature not checked (no keys/shinro-release-signing.pub.asc yet); the digest above is the check"
fi

# --- extract ---------------------------------------------------------------------
rm -rf "$DEST"
mkdir -p "$DEST" || md_fail "cannot create $DEST" 5
tar -xzf "$DL/$TARBALL" -C "$DEST" --strip-components=1 ||
    md_fail "could not extract $DL/$TARBALL" 2
for b in cake-resident admin-probe cake-pack; do
    [ -x "$DEST/bin/$b" ] || md_fail "the archive holds no bin/$b" 2
done
[ -f "$DEST/caps/microduck.cakecap" ] || md_fail "the archive holds no caps/microduck.cakecap" 2
md_say "extracted $VERSION ($SLUG) to $DEST"
md_say "FETCHED: $TARBALL"
