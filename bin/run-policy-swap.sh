#!/usr/bin/env bash
# Copyright 2026 Shinro SAS. Licensed under the Business Source License 1.1; see LICENSE.
#
# bin/run-policy-swap.sh: swap the Microduck **policy** live, from a precompiled
# Cake release — the same `replace-slot` transaction bin/run-demo.sh uses to
# change the trajectory, applied to the policy instead.
#
# The policy is a **leaf**. The resident's stable `bridge` owns the sim socket
# and the 50 Hz tick and consumes the leaf with a live binding, so `replace-slot`
# on the policy slot swaps a standing control network for a walking one (or back)
# **without the plant link dropping**: the bridge keeps ticking through the
# transaction and the walk continues. Run the fused demo (`bin/run-demo.sh`) and
# the same swap would complete the transaction but stop the walk — that contrast
# is the point.
#
# Topology staged (four slots, one resident):
#
#   bridge      (owns socket + clock)   provides observation(61), pose(4)
#                                        requires command(13), control(14)
#   policy      (leaf, swapped)         provides control(14)
#                                        requires observation(61)
#   trajectory  (circle)                provides command(13); requires pose(4)
#   spare_policy (the swap target)      the other leaf
#
#   bin/fetch-release.sh
#   bin/run-policy-swap.sh [--seconds 10] [--from stand|walk] [--to stand|walk]
set -euo pipefail
# shellcheck source=lib/common.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

SECONDS_TO_RUN="10"
FROM="stand"
TO="walk"
TRAJECTORY="circle"
BAM_FLAG=""
# GIF profile (full|compact|trajectory|cake), or empty to skip rendering.
GIF_PROFILE="${GIF-compact}"
# Where the GIF goes; defaults to the run directory. `make policy-swap
# MEDIA_DIR=docs/media/cake` regenerates the committed media.
MEDIA_DIR="${MEDIA_DIR-}"
# How long the run must survive for the walk to count as not stopped.
MIN_STEPS_FRACTION=7
while [ $# -gt 0 ]; do
    case "$1" in
        --seconds) SECONDS_TO_RUN="$2"; shift 2 ;;
        --from) FROM="$2"; shift 2 ;;
        --to) TO="$2"; shift 2 ;;
        --trajectory) TRAJECTORY="$2"; shift 2 ;;
        --gif) GIF_PROFILE="$2"; shift 2 ;;
        --no-gif | --no-render) GIF_PROFILE=""; shift ;;
        --media-dir) MEDIA_DIR="$2"; shift 2 ;;
        --no-bam) BAM_FLAG="--no-bam"; shift ;;
        -h | --help) echo "usage: bin/run-policy-swap.sh [--seconds 10] [--from stand|walk] [--to stand|walk] [--no-render]"; exit 0 ;;
        *) md_fail "unrecognised argument '$1'" 5 ;;
    esac
done

cap_for() {
    case "$1" in
        stand) echo microduck_policy_stand ;;
        walk) echo microduck_policy ;;
        *) md_fail "--from/--to must be stand or walk, got '$1'" 5 ;;
    esac
}
FROM_CAP="$(cap_for "$FROM")"
TO_CAP="$(cap_for "$TO")"
[ "$FROM" != "$TO" ] || md_fail "--from and --to are both '$FROM'; nothing to swap" 5

SLUG="${MD_SLUG:-$(md_target_slug)}"
[ -n "$SLUG" ] || md_fail "no Cake release is published for $(md_arch); this demonstration is a desktop (x86_64) simulation" 3
BUNDLE="$(md_require_bundle)"
ADMIN_PROBE="$BUNDLE/bin/admin-probe"
CAKE_PACK="$BUNDLE/bin/cake-pack"
HOST_FLAG=""
[ "$SLUG" = "x86_64" ] && HOST_FLAG="--host"

# The split's capsules. A release that predates the policy-swap work ships only
# the fused `microduck.cakecap`, so say that plainly rather than failing obscurely.
for cap in microduck_bridge "$FROM_CAP" "$TO_CAP" "trajectory_$TRAJECTORY"; do
    [ -f "$BUNDLE/caps/$cap.cakecap" ] ||
        md_fail "the release has no caps/$cap.cakecap in $BUNDLE/caps (it predates the policy swap)" 5
done

if [ -n "${PYTHON:-}" ]; then
    CLIENT_PY="$PYTHON"
elif [ -x "$MD_ROOT/.venv/bin/python" ]; then
    CLIENT_PY="$MD_ROOT/.venv/bin/python"
else
    CLIENT_PY="python3"
fi

RUN_DIR="$MD_STATE/run-policy-swap"
STORE="$RUN_DIR/state"
PLAN="$RUN_DIR/policy-swap.cakepln"
SOCK="$RUN_DIR/microduck.sock"
CSV="$RUN_DIR/trace.csv"
BRIDGE_CFG="$RUN_DIR/bridge.cfg"
POLICY_CFG="$RUN_DIR/policy.cfg"
SPARE_CFG="$RUN_DIR/spare.cfg"
TRAJ_CFG="$RUN_DIR/trajectory.cfg"
CONF="$RUN_DIR/resident.conf"
ADMIN="$RUN_DIR/admin.sock"
CLIENT_LOG="$RUN_DIR/client.out"
PAYLOAD="$RUN_DIR/replace.bin"

rm -rf "$RUN_DIR"
mkdir -p "$RUN_DIR" "$STORE"
MEDIA_DIR="${MEDIA_DIR:-$RUN_DIR}"
mkdir -p "$MEDIA_DIR"
# The bridge's configuration is the socket path; a leaf's is an unused test
# signal (it reads the bridge's table once bound), so it is a zeroed
# `host_in_len`(61) f64 blob. A config schema is part of a slot's identity, so
# both leaves name the same schema.
printf '%s' "$SOCK" >"$BRIDGE_CFG"
head -c 488 /dev/zero >"$POLICY_CFG"
cp -- "$POLICY_CFG" "$SPARE_CFG"
printf '%s' "$TRAJECTORY" >"$TRAJ_CFG"

md_say "staging the split deployment ($SLUG, $FROM -> $TO)"
deploy_args=(deploy --state-root "$STORE" --plan-out "$PLAN")
[ -n "$HOST_FLAG" ] && deploy_args+=("$HOST_FLAG")
deploy_args+=(
    --slot "bridge:$BUNDLE/caps/microduck_bridge.cakecap:$BRIDGE_CFG"
    --slot "policy:$BUNDLE/caps/$FROM_CAP.cakecap:$POLICY_CFG"
    --slot "trajectory:$BUNDLE/caps/trajectory_${TRAJECTORY}.cakecap:$TRAJ_CFG"
    --slot "spare_policy:$BUNDLE/caps/$TO_CAP.cakecap:$SPARE_CFG"
)
# Explicit because two slots provide `control` (the policy leaf and its spare),
# so the bridge's action requirement is otherwise ambiguous.
deploy_args+=(
    --bind "bridge:0=trajectory"
    --bind "bridge:1=policy"
    --bind "policy:0=bridge"
    --bind "trajectory:0=bridge"
    --bind "spare_policy:0=bridge"
)
"$CAKE_PACK" "${deploy_args[@]}" | sed 's/^/microduck-cake: /'

cat >"$CONF" <<EOF
store_root = $STORE
admin_socket_path = $ADMIN
admin_socket_group = $(id -gn)
admin_socket_mode = 0660
admin_max_connections = 4
signature_policy = allow
trusted_keys =
flight_ring_capacity = 8192
target_profile = lekiwi-mvp
plan_artifact = $PLAN
EOF

cleanup() {
    [ -n "${CLIENT_PID:-}" ] && kill -TERM "$CLIENT_PID" 2>/dev/null || true
    [ -n "${RESIDENT_PID:-}" ] && kill -TERM "$RESIDENT_PID" 2>/dev/null || true
}
trap cleanup EXIT

md_say "starting the MuJoCo client on $SOCK"
gif_args=()
if [ -n "$GIF_PROFILE" ]; then
    schedule="${TRAJECTORY}@0.00"
    gif_args=(--gif "$GIF_PROFILE" --out-dir "$MEDIA_DIR" --schedule "$schedule"
        --panel-duration "$SECONDS_TO_RUN" --video-name "microduck_cake_policy_swap.gif")
fi
(cd "$MD_ROOT" && exec "$CLIENT_PY" -m demos.demo_cake_policy \
    --socket "$SOCK" --seconds "$((SECONDS_TO_RUN + 40))" --csv "$CSV" $BAM_FLAG "${gif_args[@]}") \
    >"$CLIENT_LOG" 2>&1 &
CLIENT_PID=$!
md_wait_for_file "$SOCK" 200 || { cat "$CLIENT_LOG"; md_fail "client never bound $SOCK"; }

md_say "starting $(basename "$BUNDLE/bin/cake-resident")"
"$BUNDLE/bin/cake-resident" --config "$CONF" >"$RUN_DIR/resident.out" 2>"$RUN_DIR/resident.err" &
RESIDENT_PID=$!
if ! md_grep_wait "ready pid=" "$RUN_DIR/resident.out" 400; then
    cat "$RUN_DIR/resident.out" "$RUN_DIR/resident.err" >&2
    md_fail "resident start failed"
fi

# Let the initial policy settle (the duck balances under the stand policy).
SETTLE_S=2
sleep "$SETTLE_S"

policy_uuid="$(md_slot_uuid "$ADMIN_PROBE" "$ADMIN" policy)"
[ -n "$policy_uuid" ] || { cat "$RUN_DIR/resident.out"; md_fail "no 'policy' slot in the plan"; }
session_before="$(md_status_field "$ADMIN_PROBE" "$ADMIN" session_uuid)"
md_say "policy slot $policy_uuid; session $session_before"
md_say "swapping policy ($FROM -> $TO)"
capsule="$BUNDLE/caps/$TO_CAP.cakecap"
digest="$(sha256sum "$capsule" | cut -d' ' -f1)"
"$CLIENT_PY" - "$PAYLOAD" "$policy_uuid" "$digest" <<'PY'
import sys
out, slot_uuid, digest = sys.argv[1], sys.argv[2], sys.argv[3]
open(out, "wb").write(bytes.fromhex(slot_uuid) + bytes.fromhex(digest))
PY
swap_uuid="$(head -c16 /dev/urandom | od -An -tx1 | tr -d ' \n')"
swap_out="$("$ADMIN_PROBE" request "$ADMIN" replace-slot --uuid "$swap_uuid" --payload "$PAYLOAD" 2>&1)"
printf '%s\n' "$swap_out" | sed 's/^/microduck-cake: /'
printf '%s\n' "$swap_out" | grep -q '^status ok' || md_fail "the policy replace-slot was refused"

# Read the post-swap facts while the resident is still up: once it stops, the
# admin plane is gone and a `get-status` returns nothing to compare.
phase="$(printf '%s\n' "$swap_out" | sed -nE 's/^transaction_phase (.*)$/\1/p' | head -1)"
session_after="$(md_status_field "$ADMIN_PROBE" "$ADMIN" session_uuid)"
policy_uuid_after="$(md_slot_uuid "$ADMIN_PROBE" "$ADMIN" policy)"

# Run out the rest of the window so the post-swap behaviour is measured too.
sleep "$(awk -v s="$SECONDS_TO_RUN" -v w="$SETTLE_S" 'BEGIN { printf "%.2f", s - w }')"

kill -TERM "$RESIDENT_PID" 2>/dev/null || true
wait "$RESIDENT_PID" 2>/dev/null || true
RESIDENT_PID=""
wait "$CLIENT_PID" || true
CLIENT_PID=""
trap - EXIT

echo "=== client ==="
cat "$CLIENT_LOG"
echo "=== resident ==="
cat "$RUN_DIR/resident.out"

grep -q "plan active" "$RUN_DIR/resident.out" || md_fail "no plan-active record"
grep -q "stopped" "$RUN_DIR/resident.out" || md_fail "the resident did not stop cleanly"

# The whole point: the policy transaction completed, the session identity is
# unchanged (the resident never restarted), and the walk did not stop — the
# bridge held the plant link through the swap.
phase_ok=1
[ "$phase" = "COMPLETED" ] || phase_ok=0
[ "$session_after" = "$session_before" ] || phase_ok=0
[ "$policy_uuid_after" = "$policy_uuid" ] || phase_ok=0
[ "$phase_ok" = "1" ] ||
    md_fail "the swap did not hold: phase='$phase' session='$session_before'->'$session_after' slot='$policy_uuid'->'$policy_uuid_after'"

min_steps="$((SECONDS_TO_RUN * 50 * MIN_STEPS_FRACTION / 10))"
"$CLIENT_PY" - "$CLIENT_LOG" "$min_steps" <<'PY' || exit 1
import json, sys
for line in open(sys.argv[1]):
    if line.startswith("RESULT "):
        m = json.loads(line[len("RESULT "):])
        print(f"walk: steps={m['steps']} travel={m['travel_m']:.3f} m "
              f"speed={m['mean_speed_mps']:.3f} m/s tilt={m['tilt_deg']:.1f} deg "
              f"z={m['final_trunk_z']:.3f} m")
        if m["steps"] < int(sys.argv[2]):
            sys.exit(f"walk: only {m['steps']} steps (want >= {sys.argv[2]}); "
                     "the plant link did not survive the policy swap")
        break
else:
    sys.exit("walk: the client produced no metrics")
PY

md_say "policy swap ($FROM -> $TO): PASS — transaction COMPLETED, session unchanged, the bridge held the plant link through the swap"
