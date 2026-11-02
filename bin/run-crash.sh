#!/usr/bin/env bash
# Copyright 2026 Shinro SAS. Licensed under the Business Source License 1.1; see LICENSE.
#
# bin/run-crash.sh: crash the Cake control plane under a live plant, restart it,
# and record what Cake actually does about it.
#
# `make cake` shows that a module can be *replaced* without the resident
# restarting. This script asks the harder question: what happens when a module
# **dies**? A Cake module is code mapped into the resident's own address space
# (`/memfd:<slot>` in `/proc/<pid>/maps`, executable), so a fault in a module is a
# fault in the resident: there is no process boundary to contain it, no health
# signal the resident acts on, and no per-module restart to fall back on. What
# survives is what Cake keeps in durable state, and what a *plant* is willing to
# do while its control plane is gone.
#
# So the plant is the supervisor here. It owns the socket and its own clock, and
# when the control plane's end of the socket dies it holds the last command,
# keeps stepping MuJoCo, and waits for a restarted resident's module to connect
# again (`demos/demo_cake_policy.py --reconnect`). Everything in between is
# measured: how long the plant was blind, what the death left behind, what the
# restart recovered (or refused to), whether the session identity changed, and
# whether the walk carried on.
#
#   bin/run-crash.sh                     # one SIGKILL at t=5 s, restart at once
#   bin/run-crash.sh --restart-after 0.3 # a slower restart: the plant walks 0.3 s blind
#   bin/run-crash.sh --fault module      # a fatal fault in the module's own thread
#   bin/run-crash.sh --crashes 3 --restart-after 0.3
#   bin/run-crash.sh --swap              # commit a live swap before crashing
#
# Findings and the mechanism behind them: docs/cake-crash-restart.md.
set -euo pipefail
# shellcheck source=lib/common.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

SECONDS_TO_RUN=""
CRASH_AT="5"
CRASHES="1"
RESTART_AFTER="0.0"
HOLD="2"
FAULT="kill"
SUPERVISOR="manual"
RESTART_SEC="2"
TRAJECTORY="circle"
BAM_FLAG=""
DO_SWAP=""
OUT=""
while [ $# -gt 0 ]; do
    case "$1" in
        --seconds) SECONDS_TO_RUN="$2"; shift 2 ;;
        --crash-at) CRASH_AT="$2"; shift 2 ;;
        --crashes) CRASHES="$2"; shift 2 ;;
        --restart-after) RESTART_AFTER="$2"; shift 2 ;;
        --hold) HOLD="$2"; shift 2 ;;
        --supervisor) SUPERVISOR="$2"; shift 2 ;;
        --restart-sec) RESTART_SEC="$2"; shift 2 ;;
        --fault) FAULT="$2"; shift 2 ;;
        --trajectory) TRAJECTORY="$2"; shift 2 ;;
        --swap) DO_SWAP="1"; shift ;;
        --no-bam) BAM_FLAG="--no-bam"; shift ;;
        --out) OUT="$2"; shift 2 ;;
        -h | --help)
            echo "usage: bin/run-crash.sh [--seconds S] [--crash-at S] [--crashes N] [--restart-after S] [--hold S]"
            echo "                        [--fault kill|module] [--supervisor manual|systemd] [--restart-sec S]"
            echo "                        [--trajectory circle] [--swap] [--no-bam] [--out DIR]"
            exit 0
            ;;
        *) md_fail "unrecognised argument '$1'" 5 ;;
    esac
done
case "$FAULT" in
    kill | module) : ;;
    *) md_fail "--fault must be 'kill' (SIGKILL the resident) or 'module' (a fatal fault in a module thread)" 5 ;;
esac
case "$SUPERVISOR" in
    manual | systemd) : ;;
    *) md_fail "--supervisor must be 'manual' (the runner relaunches the resident) or 'systemd' (a user unit does, from the committed example's Restart= settings)" 5 ;;
esac
case "$CRASHES" in
    '' | *[!0-9]*) md_fail "--crashes must be a non-negative integer" 5 ;;
esac

# Enough control periods for every crash, its restart, and the run afterwards.
if [ -z "$SECONDS_TO_RUN" ]; then
    SECONDS_TO_RUN="$(awk -v n="$CRASHES" -v at="$CRASH_AT" -v gap="$RESTART_AFTER" -v hold="$HOLD" \
        'BEGIN { printf "%.0f", (n == 0 ? 0 : n * (at + gap)) + hold + 2 }')"
fi

SLUG="${MD_SLUG:-$(md_target_slug)}"
[ -n "$SLUG" ] || md_fail "no Cake release is published for $(md_arch); this demonstration is a desktop (x86_64) simulation" 3
BUNDLE="$(md_require_bundle)"
ADMIN_PROBE="$BUNDLE/bin/admin-probe"
CAKE_PACK="$BUNDLE/bin/cake-pack"
RESIDENT_BIN="$BUNDLE/bin/cake-resident"
HOST_FLAG=""
if [ "$SLUG" = "x86_64" ]; then HOST_FLAG="--host"; fi
SPARE="eight"
if [ "$TRAJECTORY" = "eight" ]; then SPARE="slalom"; fi
for c in "trajectory_$TRAJECTORY" "trajectory_$SPARE"; do
    [ -f "$BUNDLE/caps/$c.cakecap" ] || md_fail "the release has no '$c' capsule in $BUNDLE/caps" 5
done

if [ -n "${PYTHON:-}" ]; then
    CLIENT_PY="$PYTHON"
elif [ -x "$MD_ROOT/.venv/bin/python" ]; then
    CLIENT_PY="$MD_ROOT/.venv/bin/python"
else
    CLIENT_PY="python3"
fi

RUN_DIR="${OUT:-$MD_STATE/run-crash}"
STORE="$RUN_DIR/state"
PLAN="$RUN_DIR/microduck.cakepln"
SOCK="$RUN_DIR/microduck.sock"
ADMIN="$RUN_DIR/admin.sock"
CSV="$RUN_DIR/trace.csv"
POLICY_CFG="$RUN_DIR/policy.cfg"
TRAJ_CFG="$RUN_DIR/trajectory.cfg"
CONF="$RUN_DIR/resident.conf"
CLIENT_LOG="$RUN_DIR/client.out"
RESIDENT_LOG="$RUN_DIR/resident.out"
RESIDENT_ERR="$RUN_DIR/resident.err"
PAYLOAD="$RUN_DIR/replace.bin"
CRASHES_FILE="$RUN_DIR/crashes.jsonl"
CRASH_JSON="$RUN_DIR/crash.json"

rm -rf "$RUN_DIR"
mkdir -p "$RUN_DIR" "$STORE"
: >"$CRASHES_FILE"
# systemd appends the resident's output to this file; creating it here keeps the
# ready-line counter from reading a path that does not exist yet.
: >"$RESIDENT_LOG"
: >"$RESIDENT_ERR"
printf '%s' "$SOCK" >"$POLICY_CFG"
printf '%s' "$TRAJECTORY" >"$TRAJ_CFG"

# The same deployment `make cake` stages: the policy (it owns the plant socket),
# the active trajectory, and the replacement trajectory staged as a spare slot —
# a resident admits no capsule at runtime, so a swap can only name one the
# starting Plan already holds.
md_say "staging the deployment ($SLUG, trajectory $TRAJECTORY, spare $SPARE)"
deploy_args=(deploy --state-root "$STORE" --plan-out "$PLAN")
if [ -n "$HOST_FLAG" ]; then deploy_args+=("$HOST_FLAG"); fi
deploy_args+=(
    --slot "microduck:$BUNDLE/caps/microduck.cakecap:$POLICY_CFG"
    --slot "trajectory:$BUNDLE/caps/trajectory_$TRAJECTORY.cakecap:$TRAJ_CFG"
    --slot "spare_$SPARE:$BUNDLE/caps/trajectory_$SPARE.cakecap:$TRAJ_CFG"
    --bind "microduck:0=trajectory"
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

CLIENT_PID=""
RESIDENT_PID=""
#: The transient user unit `--supervisor systemd` runs the resident under.
UNIT="md-crash-$$"

# `systemd` mode is the route packaging/systemd/cake-resident.service documents:
# `Restart=on-abnormal` plus `RestartSec=`, and no relaunch anywhere in this
# script — systemd owns the lifetime and this script only measures it.
if [ "$SUPERVISOR" = "systemd" ]; then
    md_need_tool systemctl "the systemd route needs the systemd user manager"
    md_need_tool systemd-run "the systemd route needs systemd-run"
    systemctl --user is-system-running >/dev/null 2>&1 ||
        md_fail "no systemd user manager is reachable (systemctl --user is-system-running); use --supervisor manual" 3
fi

stop_resident() {
    if [ "$SUPERVISOR" = "systemd" ]; then
        systemctl --user stop "$UNIT" >/dev/null 2>&1 || true
    elif [ -n "$RESIDENT_PID" ]; then
        kill -TERM "$RESIDENT_PID" 2>/dev/null || true
    fi
}
cleanup() {
    [ -n "$CLIENT_PID" ] && kill -TERM "$CLIENT_PID" 2>/dev/null || true
    stop_resident
}
trap cleanup EXIT

# The plant first: it binds the socket, so a resident started at any moment finds
# a plant to talk to. `--reconnect N` is the whole supervisor — N restarts' worth
# of waiting instead of stopping at the first lost connection.
md_say "starting the MuJoCo plant on $SOCK (it waits for up to $CRASHES restart(s))"
# Unbuffered: this run is watched while it happens, and the interesting lines are
# the ones printed *during* the outage.
(cd "$MD_ROOT" && exec env PYTHONUNBUFFERED=1 "$CLIENT_PY" -m demos.demo_cake_policy \
    --socket "$SOCK" --seconds "$SECONDS_TO_RUN" --csv "$CSV" $BAM_FLAG \
    --reconnect "$CRASHES") >"$CLIENT_LOG" 2>&1 &
CLIENT_PID=$!
md_wait_for_file "$SOCK" 200 || { cat "$CLIENT_LOG"; md_fail "the plant never bound $SOCK"; }

# Ready lines accumulate across restarts, so every wait counts them.
READY_LINES=0
START_TIMES=()
start_resident() {
    # start_resident [await]: `await` means "systemd is doing the restarting, wait
    # for its process to report ready" and starts nothing of its own.
    local mode="${1:-start}" started want _
    started="$(date +%s.%N)"
    READY_LINES=$((READY_LINES + 1))
    want="$READY_LINES"
    if [ "$SUPERVISOR" = "systemd" ] && [ "$mode" = "await" ]; then
        : # systemd owns the relaunch: Restart=on-abnormal, RestartSec=$RESTART_SEC
    elif [ "$SUPERVISOR" = "systemd" ]; then
        # The committed example's own restart policy, over a transient unit: the
        # same `Restart=on-abnormal` and `RestartSec=` keys, so what is measured is
        # what an operator installing the unit would get.
        systemd-run --user --unit "$UNIT" --collect \
            --property Type=simple \
            --property Restart=on-abnormal \
            --property "RestartSec=$RESTART_SEC" \
            --property "StandardOutput=append:$RESIDENT_LOG" \
            --property "StandardError=append:$RESIDENT_ERR" \
            -- "$RESIDENT_BIN" --config "$CONF" >/dev/null 2>&1 ||
            md_fail "systemd-run refused to start $UNIT"
        RESIDENT_PID=""
    else
        "$RESIDENT_BIN" --config "$CONF" >>"$RESIDENT_LOG" 2>>"$RESIDENT_ERR" &
        RESIDENT_PID=$!
    fi
    # The log is appended to across restarts, so "ready" is a count, not a match:
    # the Nth start is ready when the log holds N ready lines.
    for _ in $(seq 1 600); do
        if [ "$(grep -c 'ready pid=' "$RESIDENT_LOG" 2>/dev/null)" -ge "$want" ]; then
            START_TIMES+=("$(awk -v a="$started" -v b="$(date +%s.%N)" 'BEGIN { printf "%.2f", b - a }')")
            return 0
        fi
        sleep 0.05
    done
    cat "$RESIDENT_LOG" "$RESIDENT_ERR" >&2
    return 1
}

# Which thread is the module? Every thread in the resident is named
# `cake-resident` and every one of them sleeps between ticks, so the honest
# discriminator is work done: the module that owns the plant socket runs the
# policy at the control rate, the trajectory module updates a path, and the main
# thread only polls. Delivering a fatal signal to the busiest non-main thread is
# a fault raised on a thread executing the resident-mapped capsule — which is the
# point: here, that is not a boundary the kernel will keep apart from the rest.
#
# The signal is SIGILL, the shape a Zig `@trap()` (or any illegal instruction)
# raises. SIGSEGV was tried first and is useless as an injected fault: the
# resident is a Rust program, Rust installs a SIGSEGV handler for its stack-guard
# page, and an *asynchronous* SIGSEGV is not near that guard, so the handler
# returns and the signal is discarded — measured here as a process that stays up.
# On Ubuntu the core-dump helper (`core_pattern` is a pipe to apport, which is not
# bounded by RLIMIT_CORE) then spends about a second collecting the core before a
# core-dumping signal actually terminates anything, so `--fault module` reports a
# longer crash time than `--fault kill`. That is the crash, not the restart.
inject_fault() {
    local pid="$1" fault="$2"
    "$CLIENT_PY" - "$pid" "$fault" <<'PY'
import ctypes
import os
import sys

pid = int(sys.argv[1])
fault = sys.argv[2]
if fault == "kill":
    os.kill(pid, 9)
    print(f"SIGKILL to the resident (pid {pid})")
    raise SystemExit(0)


def cpu_ticks(tid: int) -> int:
    """utime + stime for one thread, from /proc/<pid>/task/<tid>/stat."""
    fields = open(f"/proc/{pid}/task/{tid}/stat").read().rsplit(")", 1)[1].split()
    return int(fields[11]) + int(fields[12])


# Field numbering: `state` is field 3 overall, so utime (14) and stime (15) are
# indexes 11 and 12 after the `(comm)` field is dropped.
tids = [int(t) for t in os.listdir(f"/proc/{pid}/task") if int(t) != pid]
busiest = max(tids, key=cpu_ticks)
SYS_TGKILL = {"x86_64": 234, "aarch64": 131}[os.uname().machine]
libc = ctypes.CDLL(None, use_errno=True)
rc = libc.syscall(
    ctypes.c_long(SYS_TGKILL), ctypes.c_long(pid), ctypes.c_long(busiest), ctypes.c_long(4)
)
if rc != 0:
    raise SystemExit(f"tgkill failed: {os.strerror(ctypes.get_errno())}")
print(
    f"SIGILL to tid {busiest} (cpu {cpu_ticks(busiest)} ticks), "
    f"a thread of the resident's own address space"
)
PY
}

# The module code is mapped into the resident, not run behind a boundary. Say so
# from the kernel's own record instead of asserting it.
module_mappings() {
    grep -o '/memfd:[^ ]*' "/proc/$1/maps" 2>/dev/null | sort -u | paste -sd, - || true
}

status_field() { md_status_field "$ADMIN_PROBE" "$ADMIN" "$1"; }
slot_states() {
    # Answers nothing (and fails nothing) when no resident is serving, which is
    # the state a systemd give-up leaves behind.
    md_admin "$ADMIN_PROBE" "$ADMIN" list-slots 2>/dev/null |
        sed -nE 's/.*name=([a-z_0-9]+) state=([A-Z]+).*/\1=\2/p' | paste -sd' ' - || true
}

drains_before=0
md_say "starting the resident ($SUPERVISOR)"
start_resident || md_fail "the resident never reported ready"
md_say "resident ready in ${START_TIMES[0]} s; $CRASHES crash(es), first at t=${CRASH_AT} s"

# Optionally commit a live swap first: a committed epoch is what makes the
# restart's durable-state path interesting, because the journal then carries
# authority that the fresh process has to reconcile with.
sleep "$CRASH_AT"
if [ -n "$DO_SWAP" ]; then
    traj_uuid="$(md_slot_uuid "$ADMIN_PROBE" "$ADMIN" trajectory || true)"
    [ -n "$traj_uuid" ] || md_fail "no 'trajectory' slot in the plan" 5
    digest="$(sha256sum "$BUNDLE/caps/trajectory_$SPARE.cakecap" | cut -d' ' -f1)"
    "$CLIENT_PY" - "$PAYLOAD" "$traj_uuid" "$digest" <<'PY'
import sys

open(sys.argv[1], "wb").write(bytes.fromhex(sys.argv[2]) + bytes.fromhex(sys.argv[3]))
PY
    md_say "committing a live swap to $SPARE before the crash"
    "$ADMIN_PROBE" request "$ADMIN" replace-slot \
        --uuid "$(head -c16 /dev/urandom | od -An -tx1 | tr -d ' \n')" --payload "$PAYLOAD" 2>&1 |
        sed 's/^/microduck-cake: /' | grep -E 'status |transaction_phase ' || true
    sleep 1
fi

# Under systemd the pid is the unit's, not this shell's child, and the restart is
# systemd's: read it rather than remembering it.
resident_pid() {
    if [ "$SUPERVISOR" = "systemd" ]; then
        systemctl --user show "$UNIT" -p ExecMainPID --value 2>/dev/null
    else
        printf '%s' "$RESIDENT_PID"
    fi
}
systemd_field() { systemctl --user show "$UNIT" -p "$1" --value 2>/dev/null; }

gave_up=0
for i in $(seq 1 "$CRASHES"); do
    if [ "$i" -gt 1 ]; then sleep "$CRASH_AT"; fi
    session_before="$(status_field session_uuid || true)"
    epoch_before="$(status_field epoch || true)"
    target_pid="$(resident_pid)"
    mappings="$(module_mappings "$target_pid")"
    drains_before="$(grep -c 'shutdown drain' "$RESIDENT_LOG" || true)"
    restarts_before=0
    if [ "$SUPERVISOR" = "systemd" ]; then restarts_before="$(systemd_field NRestarts)"; fi

    md_say "crash $i/$CRASHES: $FAULT (session ${session_before:0:16}…, epoch $epoch_before)"
    inject_fault "$target_pid" "$FAULT" | sed 's/^/microduck-cake: /'
    crash_started="$(date +%s.%N)"
    exit_code=0
    if [ "$SUPERVISOR" = "systemd" ]; then
        # systemd reaps the process, so there is no `wait` to take a status from.
        # Read the unit instead: `Result` (`signal`, `core-dump`, ...) is only the
        # crash's class while the unit sits in auto-restart, and a short RestartSec
        # can carry it past that window before this poll sees it, so what was read
        # and what was missed are both reported as such.
        exit_code="not captured (RestartSec=$RESTART_SEC is shorter than this read)"
        for _ in $(seq 1 300); do
            sub="$(systemd_field SubState)"
            case "$sub" in
                auto-restart | failed | dead)
                    exit_code="$(systemd_field Result)"
                    break
                    ;;
            esac
            if [ "$(systemd_field NRestarts)" != "$restarts_before" ]; then break; fi
            sleep 0.02
        done
        crash_s="$(awk -v a="$crash_started" -v b="$(date +%s.%N)" 'BEGIN { printf "%.2f", b - a }')"
    else
        wait "$RESIDENT_PID" 2>/dev/null || exit_code=$?
        RESIDENT_PID=""
        crash_s="$(awk -v a="$crash_started" -v b="$(date +%s.%N)" 'BEGIN { printf "%.2f", b - a }')"
    fi

    # What a crashed process leaves behind, as opposed to a stopped one.
    drains_after="$(grep -c 'shutdown drain' "$RESIDENT_LOG" || true)"
    drain_ran=false
    if [ "$drains_after" -gt "$drains_before" ]; then drain_ran=true; fi
    admin_left=false
    if [ -S "$ADMIN" ]; then admin_left=true; fi
    claim_left=false
    if [ -f "$STORE/.cake-resident.lock" ]; then claim_left=true; fi

    if [ "$SUPERVISOR" = "systemd" ]; then
        md_say "restart $i/$CRASHES: systemd (Restart=on-abnormal, RestartSec=$RESTART_SEC)"
        if ! start_resident await; then
            # The unit ran out of starts: systemd's own rate limit is the only
            # crash-loop bound on this path, and when it trips nothing retries.
            echo "systemd gave up on $UNIT: $(systemd_field ActiveState) / $(systemd_field Result)"
            gave_up=1
            break
        fi
    else
        sleep "$RESTART_AFTER"
        md_say "restart $i/$CRASHES"
        start_resident
    fi
    # The number that matters is crash -> policy back on the socket, whichever
    # supervisor did the relaunching.
    restart_ready_s="$(awk -v a="$crash_started" -v b="$(date +%s.%N)" 'BEGIN { printf "%.2f", b - a }')"
    journal_line="$(grep -E 'journal at start' "$RESIDENT_LOG" | tail -n 1 | sed 's/^plan-resident: //')"
    recovery_line="$(grep -E 'durable start' "$RESIDENT_LOG" | tail -n 1 | sed 's/^plan-resident: //')"
    session_after="$(status_field session_uuid || true)"
    epoch_after="$(status_field epoch || true)"

    # The plant is the supervisor: did it notice, hold, and re-acquire?
    md_grep_wait "restarted Cake control plane is back" "$CLIENT_LOG" 400 ||
        md_fail "the plant never re-acquired the restarted control plane (crash $i); see $CLIENT_LOG"
    lost_line="$(grep 'control plane went away' "$CLIENT_LOG" | tail -n 1 | sed 's/^sim: //')"
    back_line="$(grep 'restarted Cake control plane is back' "$CLIENT_LOG" | tail -n 1 | sed 's/^sim: //')"

    # One line per crash; crash.json is assembled from these, so the printed
    # summary and the machine-readable record cannot drift apart.
    "$CLIENT_PY" - "$CRASHES_FILE" "$i" "$FAULT" "$exit_code" "$crash_s" \
        "$drain_ran" "$admin_left" "$claim_left" "$mappings" \
        "$session_before" "$session_after" "$epoch_before" "$epoch_after" \
        "$restart_ready_s" "$journal_line" "$recovery_line" "$lost_line" "$back_line" <<'PY'
import json
import sys

(
    path,
    index,
    fault,
    exit_code,
    crash_s,
    drain_ran,
    admin_left,
    claim_left,
    mappings,
    session_before,
    session_after,
    epoch_before,
    epoch_after,
    restart_s,
    journal,
    recovery,
    lost,
    back,
) = sys.argv[1:19]
record = {
    "index": int(index),
    "fault": fault,
    "exit": exit_code,
    "crash_s": float(crash_s),
    "drain_after_crash": drain_ran == "true",
    "admin_socket_left": admin_left == "true",
    "claim_file_left": claim_left == "true",
    "module_mappings": mappings,
    "session_before": session_before,
    "session_after": session_after,
    "epoch_before": epoch_before,
    "epoch_after": epoch_after,
    "restart_ready_s": float(restart_s),
    "journal": journal,
    "recovery": recovery,
    "link_lost": lost,
    "link_back": back,
}
with open(path, "a") as fh:
    fh.write(json.dumps(record) + "\n")
PY

    echo
    echo "crash $i/$CRASHES ($FAULT) at t=${CRASH_AT} s"
    echo "  module code in the resident  ${mappings:-no /memfd mapping found}"
    if [ "$SUPERVISOR" = "systemd" ]; then
        echo "  resident died                systemd Result=$exit_code, seen after ${crash_s} s"
    else
        echo "  resident died                exit $exit_code after ${crash_s} s"
    fi
    echo "  graceful shutdown            $([ "$drain_ran" = true ] && echo 'drain line present' || echo 'no drain line: the shutdown never ran')"
    echo "  left behind                  admin socket $([ "$admin_left" = true ] && echo present || echo gone); state-root claim $([ "$claim_left" = true ] && echo 'lock file present, flock released by the kernel' || echo gone)"
    if [ "$SUPERVISOR" = "systemd" ]; then
        echo "  restart                      process back in ${restart_ready_s} s from the crash (systemd RestartSec=$RESTART_SEC), slots $(slot_states)"
    else
        echo "  restart                      ready in ${restart_ready_s} s from the crash, slots $(slot_states)"
    fi
    echo "  journal                      $journal_line"
    echo "  durable recovery             $recovery_line"
    echo "  session                      ${session_before:0:16}… -> ${session_after:0:16}… $([ "$session_before" = "$session_after" ] && echo UNCHANGED || echo '(new process, new session)')"
    echo "  committed epoch              $epoch_before -> $epoch_after$([ -n "$epoch_before" ] && [ "$epoch_before" != "$epoch_after" ] && echo '  (the fresh process committed over it)')"
    echo "  plant                        ${lost_line:-no link loss recorded}"
    echo "  plant                        ${back_line:-did not re-acquire}"
    echo
done

# Let the walk run for a while after the last restart before asking the state root
# anything: what the plant does *after* the control plane comes back is the half of
# the answer a restart that only reports "ready" would hide.
if [ "$CRASHES" -gt 0 ] && [ "$gave_up" = 0 ]; then
    md_say "holding $HOLD s of walk after the restart"
    sleep "$HOLD"
fi

# The crash landed on a state root whose journal already carries committed
# records. Committing again is what shows whether the fresh process continues that
# history or starts a second, conflicting one.
traj_uuid="$(md_slot_uuid "$ADMIN_PROBE" "$ADMIN" trajectory || true)"
epoch_pre_swap="$(status_field epoch || true)"
post_swap=""
if [ -n "$traj_uuid" ]; then
    digest="$(sha256sum "$BUNDLE/caps/trajectory_$SPARE.cakecap" | cut -d' ' -f1)"
    "$CLIENT_PY" - "$PAYLOAD" "$traj_uuid" "$digest" <<'PY'
import sys

open(sys.argv[1], "wb").write(bytes.fromhex(sys.argv[2]) + bytes.fromhex(sys.argv[3]))
PY
    post_swap="$("$ADMIN_PROBE" request "$ADMIN" replace-slot \
        --uuid "$(head -c16 /dev/urandom | od -An -tx1 | tr -d ' \n')" --payload "$PAYLOAD" 2>&1 |
        sed -nE 's/^transaction_phase (.*)$/\1/p' | head -1)"
    epoch_post_swap="$(status_field epoch || true)"
    echo "slots after the restart      $(slot_states)"
    echo "swap after the restart       ${post_swap:-refused} (epoch $epoch_pre_swap -> $epoch_post_swap)"
    echo
fi

# A clean stop after a crash is the other half of the picture: the drain that the
# crashed process never reached, on the process that replaced it.
stop_resident
if [ "$SUPERVISOR" = "manual" ] && [ -n "$RESIDENT_PID" ]; then
    wait "$RESIDENT_PID" 2>/dev/null || true
fi
RESIDENT_PID=""
wait "$CLIENT_PID" 2>/dev/null || true
CLIENT_PID=""
trap - EXIT

drain_line="$(grep 'shutdown drain' "$RESIDENT_LOG" 2>/dev/null | tail -n 1 | sed 's/^plan-resident: //' || true)"

"$CLIENT_PY" - "$CRASH_JSON" "$SECONDS_TO_RUN" "$CRASHES" "$CRASH_AT" "$FAULT" \
    "$post_swap" "$drain_line" "$CRASHES_FILE" <<'PY'
import json
import sys

out, seconds, crashes, crash_at, fault, post_swap, drain_line, crashes_file = sys.argv[1:9]
runs = []
try:
    with open(crashes_file) as fh:
        runs = [json.loads(line) for line in fh if line.strip()]
except OSError:
    pass
record = {
    "seconds": float(seconds),
    "crashes_requested": int(crashes),
    "crashes_observed": len(runs),
    "crash_at_s": float(crash_at),
    "fault": fault,
    "swap_after_restart": post_swap or "refused",
    "final_drain": drain_line,
    "runs": runs,
}
try:
    with open(out, "w") as fh:
        json.dump(record, fh, indent=2)
        fh.write("\n")
except OSError as exc:
    print(f"microduck-cake: could not write {out}: {exc}", file=sys.stderr)
PY

echo "=== plant (tail) ==="
tail -n 8 "$CLIENT_LOG"
echo "=== resident (tail) ==="
tail -n 3 "$RESIDENT_LOG"

if [ "$gave_up" = 1 ]; then
    echo
    echo "systemd stopped restarting: the unit reached its start limit and nothing retried."
    echo "  (Restart=on-abnormal + systemd's default DefaultStartLimitBurst=5 is the only"
    echo "   crash-loop bound on this path; Cake's own CRASH_LOOP_BOUND is not wired.)"
    echo
fi
grep -q '^RESULT ' "$CLIENT_LOG" || md_fail "the plant produced no metrics; the run did not survive"
if [ "$gave_up" = 0 ]; then
    [ -n "$drain_line" ] || md_fail "the final resident stop recorded no drain (see $RESIDENT_LOG)"
fi
[ -z "$post_swap" ] || [ "$post_swap" = "COMPLETED" ] || md_warn "the swap after the restart answered '$post_swap'"
md_say "crash/restart run: PASS (the plant survived $CRASHES crash(es); read the limits above)"
