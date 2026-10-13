# shellcheck shell=bash
# Copyright 2026 Shinro SAS. Licensed under the Business Source License 1.1; see LICENSE.
#
# Shared helpers for the Microduck Cake demo scripts. Sourced by every script
# under cake/, in a checkout and in an extracted release archive alike; never
# run on its own.
#
# The demo consumes a **precompiled Cake release**: the resident (the control
# plane) and the policy capsule are built by the private Cake tree and shipped
# as signed tarballs, one per target. Nothing here compiles a Cake module, and
# no Cake source (or ABI header) lives in this repository.

MD_REPO="${MD_REPO:-Shinro-xyz/shinro-demo-microduck}"
MD_RELEASE_KEY_FINGERPRINT="${MD_RELEASE_KEY_FINGERPRINT:-PLACEHOLDER_FINGERPRINT_REPLACE_ME}"

MD_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MD_CAKE_DIR="$(dirname "$MD_LIB_DIR")"
MD_ROOT="$(dirname "$MD_CAKE_DIR")"

if [ -r "$MD_CAKE_DIR/VERSION" ]; then
    MD_VERSION="$(tr -d '[:space:]' <"$MD_CAKE_DIR/VERSION")"
else
    MD_VERSION=""
fi
case "$MD_VERSION" in
    v[0-9]*.[0-9]*.[0-9]*) : ;;
    *) printf 'microduck-cake: FAIL: %s/VERSION is missing or is not vX.Y.Z\n' "$MD_CAKE_DIR" >&2; exit 1 ;;
esac

MD_RELEASE_DIR="${MD_RELEASE_DIR:-$MD_CAKE_DIR/release}"
MD_KEYS_DIR="${MD_KEYS_DIR:-$MD_CAKE_DIR/keys}"
MD_STATE="${MD_STATE_DIR:-$MD_ROOT/build/cake}"

md_say() { printf 'microduck-cake: %s\n' "$1"; }
md_warn() { printf 'microduck-cake: WARNING: %s\n' "$1" >&2; }
md_fail() { printf 'microduck-cake: FAIL: %s\n' "$1" >&2; exit "${2:-1}"; }
md_need_tool() { command -v "$1" >/dev/null 2>&1 || md_fail "$1 is not on PATH; $2"; }

# --- the target ---------------------------------------------------------------

md_arch() { uname -m; }

md_target_slug() {
    # The release tarball slug for this machine. Two are published: x86_64 (the
    # MuJoCo desktop) and aarch64 (a 64-bit Pi).
    case "$(md_arch)" in
        x86_64 | amd64) echo x86_64 ;;
        aarch64 | arm64) echo aarch64 ;;
        *) echo "" ;;
    esac
}

md_tarball_name() {
    # md_tarball_name VERSION SLUG
    printf 'microduck-cake-%s-%s.tar.gz' "$1" "$2"
}

# --- the bundle ---------------------------------------------------------------

md_bundle_dir() {
    # Where the fetched or locally assembled release for this target lives.
    local slug="${MD_SLUG:-$(md_target_slug)}"
    [ -n "$slug" ] || md_fail "no Cake release is published for $(md_arch); x86_64 and aarch64 only" 3
    printf '%s' "$MD_RELEASE_DIR/$slug"
}

md_require_bundle() {
    # Prints the bundle dir, or fails telling the user to fetch it.
    local dir
    dir="$(md_bundle_dir)"
    for b in cake-resident admin-probe cake-pack; do
        [ -x "$dir/bin/$b" ] || md_fail "no Cake release at $dir (missing bin/$b); run cake/fetch-release.sh first" 5
    done
    [ -f "$dir/caps/microduck.cakecap" ] || md_fail "$dir holds no caps/microduck.cakecap" 5
    printf '%s' "$dir"
}

# --- process helpers ----------------------------------------------------------

md_wait_for_file() {
    # md_wait_for_file PATH TICKS: up to TICKS x 50 ms for PATH to appear.
    local path="$1" ticks="$2" _ 
    for _ in $(seq 1 "$ticks"); do
        [ -e "$path" ] && return 0
        sleep 0.05
    done
    return 1
}

md_grep_wait() {
    # md_grep_wait PATTERN FILE TICKS: up to TICKS x 50 ms for FILE to match.
    local pattern="$1" file="$2" ticks="$3" _
    for _ in $(seq 1 "$ticks"); do
        grep -qE "$pattern" "$file" 2>/dev/null && return 0
        sleep 0.05
    done
    return 1
}

md_version_ge() {
    # md_version_ge A B: dotted version A at or above B.
    [ "$(printf '%s\n%s\n' "$1" "$2" | sort -t. -k1,1n -k2,2n -k3,3n | head -n 1)" = "$2" ]
}

# --- the admin plane ----------------------------------------------------------

md_status_field() {
    # md_status_field PROBE ADMIN FIELD: one field of a get-status reply.
    local probe="$1" admin="$2" field="$3"
    "$probe" request "$admin" get-status 2>/dev/null |
        sed -nE "s/^$field (.*)$/\1/p" | head -n 1
}

md_slot_uuid() {
    # md_slot_uuid PROBE ADMIN SLOT_NAME: the uuid of the named slot, or empty.
    local probe="$1" admin="$2" name="$3"
    "$probe" request "$admin" list-slots 2>/dev/null | awk -v want="$name" '
        /^slot /{n="";u="";for(i=1;i<=NF;i++){if($i~/^name=/)n=substr($i,6);if($i~/^uuid=/)u=substr($i,6)} if(n==want) print u}'
}
