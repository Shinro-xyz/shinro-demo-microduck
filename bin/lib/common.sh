# shellcheck shell=bash
# Copyright 2026 Shinro SAS. Licensed under the Business Source License 1.1; see LICENSE.
#
# Shared helpers for the shinro-demo-microduck Cake demo scripts. Sourced by
# every script under bin/, in a checkout and in an extracted release archive
# alike; never run on its own.
#
# The demo consumes a **precompiled Cake release**: the resident (the control
# plane) and the policy and trajectory capsules are built by the private Cake
# tree and shipped as signed tarballs, one per target. Nothing here compiles a
# Cake module, and no Cake source (or ABI header) lives in this repository.

# The repository this demo fetches its release from. Override to test a fork.
MD_REPO="${MD_REPO:-Shinro-xyz/shinro-demo-microduck}"

# The release signing key fingerprint. EMPTY MEANS UNVERIFIED, and that is a
# release blocker rather than a default: bin/verify.sh exits non-zero on it and
# bin/fetch-release.sh prints a warning that the digest is the only check.
# Set this before the first release. See RELEASE-CHECKLIST.md.
if [ -z "${MD_RELEASE_KEY_FINGERPRINT:-}" ]; then
    MD_RELEASE_KEY_FINGERPRINT=""
    MD_KEY_UNSET=1
else
    MD_KEY_UNSET=0
fi

MD_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MD_BIN_DIR="$(dirname "$MD_LIB_DIR")"
MD_ROOT="$(dirname "$MD_BIN_DIR")"

if [ -r "$MD_ROOT/VERSION" ]; then
    MD_VERSION="$(tr -d '[:space:]' <"$MD_ROOT/VERSION")"
else
    MD_VERSION=""
fi
case "$MD_VERSION" in
    v[0-9]*.[0-9]*.[0-9]*) : ;;
    *) printf 'microduck-cake: FAIL: %s/VERSION is missing or is not vX.Y.Z\n' "$MD_ROOT" >&2; exit 1 ;;
esac

MD_RELEASE_DIR="${MD_RELEASE_DIR:-$MD_ROOT/releases}"
MD_KEYS_DIR="${MD_KEYS_DIR:-$MD_ROOT/keys}"
MD_STATE="${MD_STATE_DIR:-$MD_ROOT/state}"

md_say() { printf 'microduck-cake: %s\n' "$1"; }
md_warn() { printf 'microduck-cake: WARNING: %s\n' "$1" >&2; }
md_fail() { printf 'microduck-cake: FAIL: %s\n' "$1" >&2; exit "${2:-1}"; }
md_need_tool() { command -v "$1" >/dev/null 2>&1 || md_fail "$1 is not on PATH; $2"; }

# --- the target ---------------------------------------------------------------

md_arch() { uname -m; }

md_target_slug() {
    # The release tarball slug for this machine. This demonstration is a DESKTOP
    # simulation: x86_64 is the only published target. aarch64 is still resolved
    # so that the scripts say something useful on a Pi rather than building a URL
    # that 404s; fetch-release.sh refuses it with an explanation unless the
    # target is named explicitly.
    case "$(md_arch)" in
        x86_64 | amd64) echo x86_64 ;;
        aarch64 | arm64) echo aarch64 ;;
        *) echo "" ;;
    esac
}

md_tarball_name() {
    # md_tarball_name VERSION SLUG [FLAVOUR]: the release asset name. The
    # `trajectory` flavour (the default) keeps the bare name, so existing
    # releases resolve unchanged; `policy-swap` carries the flavour in the name.
    local suffix=""
    case "${3:-trajectory}" in
        trajectory) ;;
        policy-swap) suffix="-policyswap" ;;
        *) return 1 ;;
    esac
    printf 'microduck-cake%s-%s-%s.tar.gz' "$suffix" "$1" "$2"
}

# --- flavours -----------------------------------------------------------------
#
# Two bundles are published, both at this repository's VERSION, each serving one
# demo. `trajectory` (the default) is the fused policy + the swappable
# trajectories — the `make cake` demo. `policy-swap` is the split: the stable
# `bridge` + the swappable policy leaves + the trajectories, and carries no
# fused capsule — the `make policy-swap` demo. They live in separate releases,
# because a repository has one tag per version and the trajectory release owns
# the bare `vX.Y.Z`, so the policy-swap release namespaces it.

md_flavour_slug() {
    # md_flavour_slug FLAVOUR SLUG: the release directory for a flavour.
    case "$1" in
        trajectory) printf '%s' "$2" ;;
        policy-swap) printf '%s-policyswap' "$2" ;;
        *) return 1 ;;
    esac
}

md_flavour_tag() {
    # md_flavour_tag FLAVOUR VERSION: the release tag that carries a flavour.
    case "$1" in
        trajectory) printf '%s' "$2" ;;
        policy-swap) printf 'policyswap-%s' "$2" ;;
        *) return 1 ;;
    esac
}

# --- machine facts, for bin/doctor.sh ----------------------------------------

md_facts() {
    MD_ARCH="$(md_arch)"
    MD_KERNEL="$(uname -r)"
    MD_OS=""
    [ -r /etc/os-release ] && MD_OS="$(sed -nE 's/^PRETTY_NAME="(.*)"$/\1/p' /etc/os-release | head -n 1)"
    MD_GLIBC="$(ldd --version 2>/dev/null | sed -nE 's/.* ([0-9]+\.[0-9]+)$/\1/p' | head -n 1)"
    MD_PAGE_SIZE="$(getconf PAGE_SIZE 2>/dev/null || echo unknown)"
    MD_SLUG="$(md_target_slug)"
}

md_classify() {
    # A classification of this machine against the published targets. Facts
    # only; it does not read calibration, a display or a robot.
    if [ -z "$MD_SLUG" ]; then
        echo "unpublished-architecture"; return
    fi
    if [ -z "$MD_GLIBC" ] || ! md_version_ge "$MD_GLIBC" 2.34; then
        echo "$MD_SLUG-untested-glibc"; return
    fi
    echo "$MD_SLUG"
}

md_version_ge() {
    # md_version_ge A B: dotted version A at or above B.
    [ "$(printf '%s\n%s\n' "$1" "$2" | sort -t. -k1,1n -k2,2n -k3,3n | head -n 1)" = "$2" ]
}

# --- the bundle ---------------------------------------------------------------

md_binaries_dir() {
    # md_binaries_dir [FLAVOUR]: where the fetched release for this target and
    # flavour lives, or non-zero if absent.
    local slug="${MD_SLUG:-$(md_target_slug)}"
    [ -n "$slug" ] || return 1
    local fs
    fs="$(md_flavour_slug "${1:-trajectory}" "$slug")" || return 1
    printf '%s' "$MD_RELEASE_DIR/$fs"
}

md_check_binaries() {
    # md_check_binaries DIR [FLAVOUR]: every shipped binary is present and
    # executable, and the flavour's capsules are there.
    local dir="$1" flav="${2:-trajectory}" b
    for b in cake-resident admin-probe cake-pack; do
        [ -x "$dir/bin/$b" ] || md_fail "$dir is missing bin/$b (run bin/fetch-release.sh)" 5
    done
    case "$flav" in
        trajectory)
            [ -f "$dir/caps/microduck.cakecap" ] || md_fail "$dir holds no caps/microduck.cakecap" 5
            ;;
        policy-swap)
            for b in microduck_bridge microduck_policy microduck_policy_stand; do
                [ -f "$dir/caps/$b.cakecap" ] || md_fail "$dir holds no caps/$b.cakecap (the policy-swap bundle)" 5
            done
            ;;
        *) md_fail "unknown release flavour '$flav'" 5 ;;
    esac
}

md_require_bundle() {
    # md_require_bundle [FLAVOUR]
    local flav="${1:-trajectory}" dir
    dir="$(md_binaries_dir "$flav")" || md_fail "no Cake release is published for $(md_arch); this demonstration is a desktop (x86_64) simulation" 3
    md_check_binaries "$dir" "$flav"
    printf '%s' "$dir"
}

md_probe_binary() {
    # md_probe_binary PATH: one bounded no-op run; proves the dynamic loader
    # resolves it. Bounded by wall clock so a hang is a failure, not a wait.
    local path="$1"
    [ -x "$path" ] || return 1
    timeout 10 "$path" --help >/dev/null 2>&1
    local rc=$?
    # 0 = it started and answered; 124/137 = it hung and was killed.
    [ "$rc" -ne 124 ] && [ "$rc" -ne 137 ]
}

# --- digests ------------------------------------------------------------------

md_verify_digests() {
    # md_verify_digests DIR [SUMS]: check DIR against a SHA256SUMS.
    # Defaults to DIR/SHA256SUMS. Two statements, not one: inside a single
    # `local a=$1 b=$a/...`, `$a` is read before the assignment takes effect,
    # and under `set -u` that is an unbound variable rather than an empty string.
    local dir="$1"
    local sums="${2:-$dir/SHA256SUMS}"
    [ -f "$sums" ] || md_fail "$dir has no SHA256SUMS; the release is unverifiable" 5
    local base
    base="$(dirname "$sums")"
    ( cd "$base" && sha256sum -c --quiet "$(basename "$sums")" ) ||
        md_fail "$dir does not match $(basename "$sums")" 6
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

# --- the admin plane ----------------------------------------------------------
#
# Every query is bounded by wall clock. An unbounded admin query turns a wedged
# resident into a wedged test, which reports as a hang rather than a failure.

MD_ADMIN_TIMEOUT="${MD_ADMIN_TIMEOUT:-10}"

md_admin() {
    # md_admin PROBE ADMIN ARGS...: one bounded admin request, on stdout.
    local probe="$1" admin="$2"; shift 2
    timeout "$MD_ADMIN_TIMEOUT" "$probe" request "$admin" "$@" 2>/dev/null
}

md_status_field() {
    # md_status_field PROBE ADMIN FIELD: one field of a get-status reply.
    local probe="$1" admin="$2" field="$3"
    md_admin "$probe" "$admin" get-status |
        sed -nE "s/^$field (.*)$/\1/p" | head -n 1
}

md_slot_uuid() {
    # md_slot_uuid PROBE ADMIN SLOT_NAME: the uuid of the named slot, or empty.
    local probe="$1" admin="$2" name="$3"
    md_admin "$probe" "$admin" list-slots | awk -v want="$name" '
        /^slot /{n="";u="";for(i=1;i<=NF;i++){if($i~/^name=/)n=substr($i,6);if($i~/^uuid=/)u=substr($i,6)} if(n==want) print u}'
}
