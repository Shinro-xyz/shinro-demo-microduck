#!/usr/bin/env bash
# Copyright 2026 Shinro SAS. Licensed under the Business Source License 1.1; see LICENSE.
#
# bin/run-demo.sh: run the Microduck walking demo on the Cake control plane, from
# a precompiled Cake release, and optionally switch trajectories live.
#
# Two Cake modules, one resident: the **policy** (owns the sim socket, runs the
# VM in lockstep) and a **trajectory** provider (emits the 13-D command block the
# policy consumes). `--swap circle,eight,slalom` replaces the trajectory module
# mid-walk with `replace-slot` — the resident never restarts, the policy keeps
# ticking, and the consumer's read of the replaceable command table stays
# epoch-guarded. That is the swappability the demo is here to show.
#
#   bin/fetch-release.sh
#   bin/run-demo.sh [--seconds 8] [--trajectory circle] [--swap circle,eight,slalom]
set -euo pipefail
# shellcheck source=lib/common.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

SECONDS_TO_RUN="8"
TRAJECTORY="circle"
SWAP=""
BAM_FLAG=""
# GIF profile (full|compact|trajectory|cake), or empty to skip rendering.
GIF_PROFILE="${GIF-compact}"
# Where the GIF and the swap figure go; defaults to the run directory.
MEDIA_DIR="${MEDIA_DIR-}"
while [ $# -gt 0 ]; do
    case "$1" in
        --seconds) SECONDS_TO_RUN="$2"; shift 2 ;;
        --trajectory) TRAJECTORY="$2"; shift 2 ;;
        --swap) SWAP="$2"; shift 2 ;;
        --gif) GIF_PROFILE="$2"; shift 2 ;;
        --no-gif | --no-render) GIF_PROFILE=""; shift ;;
        --media-dir) MEDIA_DIR="$2"; shift 2 ;;
        --no-bam) BAM_FLAG="--no-bam"; shift ;;
        -h | --help) echo "usage: bin/run-demo.sh [--seconds 8] [--trajectory circle] [--swap circle,eight,slalom] [--no-render]"; exit 0 ;;
        *) md_fail "unrecognised argument '$1'" 5 ;;
    esac
done
[ -n "$SWAP" ] || SWAP="$TRAJECTORY"
IFS=',' read -r -a TRAJS <<<"$SWAP"

# The anchored trajectory capsules (`ANCHOR=1`) re-parent each path onto the duck
# at the swap, so the hand-over shows no cross-track step; the default capsules
# stay world-fixed at the origin.
TRAJ_SUFFIX=""
MEDIA_SUFFIX=""
if [ -n "${ANCHOR-}" ]; then
    TRAJ_SUFFIX="_anchor"
    MEDIA_SUFFIX="_anchored"
fi

# The trajectory schedule for the render panel: one module per equal segment.
n_traj=${#TRAJS[@]}
segment="$(awk -v s="$SECONDS_TO_RUN" -v n="$n_traj" 'BEGIN { printf "%.2f", s / n }')"
schedule=""
for idx in "${!TRAJS[@]}"; do
    start="$(awk -v seg="$segment" -v i="$idx" 'BEGIN { printf "%.2f", seg * i }')"
    schedule+="${schedule:+,}${TRAJS[$idx]}@$start"
done

SLUG="${MD_SLUG:-$(md_target_slug)}"
[ -n "$SLUG" ] || md_fail "no Cake release is published for $(md_arch)" 3
BUNDLE="$(md_require_bundle)"
ADMIN_PROBE="$BUNDLE/bin/admin-probe"
CAKE_PACK="$BUNDLE/bin/cake-pack"
HOST_FLAG=""
[ "$SLUG" = "x86_64" ] && HOST_FLAG="--host"

for t in "${TRAJS[@]}"; do
    [ -f "$BUNDLE/caps/trajectory_$t.cakecap" ] ||
        md_fail "the release has no trajectory '$t' (want circle/eight/slalom/forward) in $BUNDLE/caps" 5
done

# An explicit PYTHON wins over a .venv. The other order is a trap: a stale
# .venv silently shadows the interpreter the user installed into, and the run
# fails with ModuleNotFoundError on a package that was just installed
# successfully. Override with `make cake PYTHON=.venv/bin/python` when the deps
# really do live in the venv.
if [ -n "${PYTHON:-}" ]; then
    CLIENT_PY="$PYTHON"
elif [ -x "$MD_ROOT/.venv/bin/python" ]; then
    CLIENT_PY="$MD_ROOT/.venv/bin/python"
else
    CLIENT_PY="python3"
fi

RUN_DIR="$MD_STATE/run"
STORE="$RUN_DIR/state"
PLAN="$RUN_DIR/microduck.cakepln"
SOCK="$RUN_DIR/microduck.sock"
CSV="$RUN_DIR/trace.csv"
POLICY_CFG="$RUN_DIR/policy.cfg"
TRAJ_CFG="$RUN_DIR/trajectory.cfg"
CONF="$RUN_DIR/resident.conf"
ADMIN="$RUN_DIR/admin.sock"
CLIENT_LOG="$RUN_DIR/client.out"
PAYLOAD="$RUN_DIR/replace.bin"

rm -rf "$RUN_DIR"
mkdir -p "$RUN_DIR" "$STORE"
MEDIA_DIR="${MEDIA_DIR:-$RUN_DIR}"
mkdir -p "$MEDIA_DIR"
printf '%s' "$SOCK" >"$POLICY_CFG"
printf '%s' "${TRAJS[0]}" >"$TRAJ_CFG"

md_say "staging the deployment ($SLUG, trajectory ${TRAJS[0]})"
deploy_args=(deploy --state-root "$STORE" --plan-out "$PLAN")
[ -n "$HOST_FLAG" ] && deploy_args+=("$HOST_FLAG")
deploy_args+=(
    --slot "microduck:$BUNDLE/caps/microduck.cakecap:$POLICY_CFG"
    --slot "trajectory:$BUNDLE/caps/trajectory_${TRAJS[0]}${TRAJ_SUFFIX}.cakecap:$TRAJ_CFG"
)
# Stage every replacement capsule as an extra slot, so the resident admits it
# at start (there is no runtime AdmitCapsule, and a candidate naming a capsule
# the process never admitted is refused before PREPARED). `--bind` names the
# provider the policy actually binds, since several slots provide `command`.
for t in "${TRAJS[@]}"; do
    [ "$t" = "${TRAJS[0]}" ] && continue
    deploy_args+=(--slot "spare_$t:$BUNDLE/caps/trajectory_$t${TRAJ_SUFFIX}.cakecap:$TRAJ_CFG")
done
deploy_args+=(--bind "microduck:0=trajectory")
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
[ -n "$GIF_PROFILE" ] && gif_args=(--gif "$GIF_PROFILE" --out-dir "$MEDIA_DIR" --schedule "$schedule" --panel-duration "$SECONDS_TO_RUN")
# The anchored run is the same demo with paths re-parented onto the duck; keep
# its media beside the world-fixed one under a distinct name.
[ -n "${ANCHOR-}" ] && gif_args+=(--anchor)
[ -n "$MEDIA_SUFFIX" ] && gif_args+=(--video-name "microduck_cake_walk${MEDIA_SUFFIX}.gif")
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

# The slot's uuid is what replace-slot names; it does not change across a swap.
traj_uuid="$(md_slot_uuid "$ADMIN_PROBE" "$ADMIN" trajectory)"
[ -n "$traj_uuid" ] || md_fail "no 'trajectory' slot in the plan"
session_before="$(md_status_field "$ADMIN_PROBE" "$ADMIN" session_uuid)"
md_say "trajectory slot $traj_uuid; session $session_before"

i=0
swap_facts=()
for t in "${TRAJS[@]}"; do
    i=$((i + 1))
    [ "$i" -eq 1 ] && continue
    sleep "$segment"
    md_say "swapping trajectory -> $t"
    capsule="$BUNDLE/caps/trajectory_$t${TRAJ_SUFFIX}.cakecap"
    digest="$(sha256sum "$capsule" | cut -d' ' -f1)"
    "$CLIENT_PY" - "$PAYLOAD" "$traj_uuid" "$digest" <<'PY'
import sys
out, slot_uuid, digest = sys.argv[1], sys.argv[2], sys.argv[3]
open(out, "wb").write(bytes.fromhex(slot_uuid) + bytes.fromhex(digest))
PY
    swap_uuid="$(head -c16 /dev/urandom | od -An -tx1 | tr -d ' \n')"
    swap_out="$("$ADMIN_PROBE" request "$ADMIN" replace-slot --uuid "$swap_uuid" --payload "$PAYLOAD" 2>&1)"
    printf '%s\n' "$swap_out" | sed 's/^/microduck-cake: /'
    printf '%s\n' "$swap_out" | grep -q '^status ok' || md_fail "replace-slot to '$t' was refused"
    phase="$(printf '%s\n' "$swap_out" | sed -nE 's/^transaction_phase (.*)$/\1/p' | head -1)"
    [ -n "$phase" ] || phase="answered"
    at_s="$(awk -v seg="$segment" -v k="$((i - 1))" 'BEGIN { printf "%.2f", seg * k }')"
    swap_facts+=("{\"index\":$((i - 1)),\"from\":\"${TRAJS[$((i - 2))]}\",\"to\":\"$t\",\"at_s\":$at_s,\"phase\":\"$phase\"}")
done
sleep "$segment"

# The whole point: the session identity is the same before and after the
# swaps — the resident never restarted. The slot uuid is unchanged too.
session_after="$(md_status_field "$ADMIN_PROBE" "$ADMIN" session_uuid)"
[ -n "$session_after" ] || md_fail "get-status gave no session identity after the swaps"
[ "$session_after" = "$session_before" ] ||
    md_fail "the session identity changed across the swaps ($session_before -> $session_after): the resident restarted"
traj_uuid_after="$(md_slot_uuid "$ADMIN_PROBE" "$ADMIN" trajectory)"
[ "$traj_uuid_after" = "$traj_uuid" ] || md_fail "the trajectory slot uuid changed across the swaps"
md_say "session unchanged ($session_after); trajectory slot $traj_uuid unchanged"

# The facts behind the swap figure: the schedule and each swap's outcome.
if [ "${#swap_facts[@]}" -gt 0 ]; then
    "$CLIENT_PY" - "$RUN_DIR/swaps.json" "$SECONDS_TO_RUN" "$session_before" "$schedule" "${swap_facts[@]}" <<'PY'
import json
import sys
from pathlib import Path

out = sys.argv[1]
sched = [[p.split("@")[0], float(p.split("@")[1])] for p in sys.argv[4].split(",")]
record = {
    "duration_s": float(sys.argv[2]),
    "session": sys.argv[3],
    "schedule": sched,
    "swaps": [json.loads(f) for f in sys.argv[5:]],
}
try:
    Path(out).write_text(json.dumps(record, indent=2))
except OSError as exc:
    print(f"could not write {out}: {exc}", file=sys.stderr)
    raise SystemExit(1)
PY
fi

kill -TERM "$RESIDENT_PID" 2>/dev/null || true
wait "$RESIDENT_PID" 2>/dev/null || true
RESIDENT_PID=""
wait "$CLIENT_PID" || true
CLIENT_PID=""
trap - EXIT

# The swap-handling figure: the path by module, and the transaction Cake ran
# for each swap (phase by phase). Written next to the GIF.
if [ -n "$GIF_PROFILE" ] && [ -f "$RUN_DIR/swaps.json" ]; then
    "$CLIENT_PY" "$MD_ROOT/scripts/plot_cake_swap.py" --run-dir "$RUN_DIR" --out "$MEDIA_DIR/microduck_cake_swap${MEDIA_SUFFIX}.png" 2>&1 | sed 's/^/microduck-cake: /' || true
fi

echo "=== client ==="
cat "$CLIENT_LOG"
echo "=== resident ==="
cat "$RUN_DIR/resident.out"

grep -q "plan active" "$RUN_DIR/resident.out" || md_fail "no plan-active record"
expected_slots=$((1 + ${#TRAJS[@]}))
grep -q "drained=$expected_slots" "$RUN_DIR/resident.out" || md_fail "the drain did not retire all $expected_slots modules"
grep -q "stopped" "$RUN_DIR/resident.out" || md_fail "the resident did not stop cleanly"
grep -q "^RESULT " "$CLIENT_LOG" || md_fail "the client produced no metrics"
"$CLIENT_PY" - "$CLIENT_LOG" <<'PY' || exit 1
import json, sys
for line in open(sys.argv[1]):
    if line.startswith("RESULT "):
        m = json.loads(line[len("RESULT "):])
        cte = m.get("mean_cross_track_m")
        cte_note = f" cte={cte:.3f} m" if cte is not None else ""
        print(f"walk: steps={m['steps']} travel={m['travel_m']:.3f} m "
              f"speed={m['mean_speed_mps']:.3f} m/s tilt={m['tilt_deg']:.1f} deg "
              f"z={m['final_trunk_z']:.3f} m{cte_note}")
        if m["steps"] < 10:
            sys.exit("walk: too few steps for a verdict")
        break
PY
md_say "microduck Cake demo: PASS"
