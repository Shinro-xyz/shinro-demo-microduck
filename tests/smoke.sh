#!/usr/bin/env bash
# Copyright 2026 Shinro SAS. Licensed under the Business Source License 1.1; see LICENSE.
#
# tests/smoke.sh: the acceptance test a stranger runs cold. No robot, no display.
#
# It asserts, in order: the release is intact and shippable; this machine can run
# the binaries; the compiled kernel walks the reference path; and the trajectory
# module is replaced while the robot walks, with the session identity unchanged
# and no tick missed.
#
# Arguments are passed through to the runner (for example --seconds 12).
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp "${TMPDIR:-/tmp}/md-smoke.XXXXXX")"
trap 'rm -f -- "$OUT"' EXIT

FAILS=0
mark() { printf '\n== %s ==\n' "$*"; }
check() { # check DESCRIPTION COMMAND...
    local desc="$1"
    shift
    if "$@" >>"$OUT" 2>&1; then
        echo "  ok    $desc"
    else
        echo "  FAIL  $desc (exit $?)"
        tail -n 15 "$OUT" | sed 's/^/        /'
        FAILS=$((FAILS + 1))
    fi
}

mark "the release is intact and shippable"
"$ROOT/bin/verify.sh" >>"$OUT" 2>&1
RC=$?
if [ "$RC" -eq 0 ]; then
    echo "  ok    bin/verify.sh -> PASS"
else
    echo "  FAIL  bin/verify.sh exited $RC"
    echo "        step 3 or 4 failing means this tarball was built without the"
    echo "        public release recipe and must not be published."
    tail -n 20 "$OUT" | sed 's/^/        /'
    FAILS=$((FAILS + 1))
fi

mark "this machine can run the binaries"
check "bin/doctor.sh" "$ROOT/bin/doctor.sh"

mark "the compiled kernel walks the reference path"
check "6 s on the circle, no render" "$ROOT/bin/run-demo.sh" --seconds 6 --no-render --trajectory circle "$@"

mark "the trajectory module is replaced while the robot walks"
"$ROOT/bin/run-demo.sh" --swap circle,eight,slalom --seconds 9 --no-render "$@" >"$OUT" 2>&1
RC=$?
if [ "$RC" -eq 0 ]; then
    echo "  ok    the run completed"
else
    echo "  FAIL  the live-swap run exited $RC"
    tail -n 20 "$OUT" | sed 's/^/        /'
    FAILS=$((FAILS + 1))
fi

# The runner's own assertions, and then the two facts that are the entire point
# of this repository, read from what the run wrote rather than from what it said.
if grep -q "session unchanged" "$OUT"; then
    echo "  ok    the runner reports the session identity unchanged across the swaps"
else
    echo "  FAIL  the runner never reported an unchanged session identity"
    FAILS=$((FAILS + 1))
fi

SWAPS="$ROOT/state/run/swaps.json"
TRACE="$ROOT/state/run/trace.csv"
if [ -f "$SWAPS" ]; then
    N_SWAPS="$(grep -o '"phase"' "$SWAPS" 2>/dev/null | wc -l)"
    N_DONE="$(grep -o '"COMPLETED"' "$SWAPS" 2>/dev/null | wc -l)"
    if [ "$N_SWAPS" -gt 0 ] && [ "$N_SWAPS" = "$N_DONE" ]; then
        echo "  ok    every swap reached phase COMPLETED ($N_DONE of $N_SWAPS)"
    else
        echo "  FAIL  swaps reaching COMPLETED: $N_DONE of $N_SWAPS"
        FAILS=$((FAILS + 1))
    fi
    SESSIONS="$(sed -nE 's/.*"session": "([^"]+)".*/\1/p' "$SWAPS" | sort -u | wc -l)"
    if [ "$SESSIONS" = "1" ]; then
        echo "  ok    one session identity across the whole run"
    else
        echo "  FAIL  $SESSIONS distinct session identities: a restart is not a live swap"
        FAILS=$((FAILS + 1))
    fi
else
    echo "  FAIL  no swaps.json: the swap did not run"
    FAILS=$((FAILS + 1))
fi

if [ -f "$TRACE" ]; then
    echo "  ok    trace.csv written ($(($(wc -l <"$TRACE") - 1)) ticks)"
else
    echo "  FAIL  no trace.csv"
    FAILS=$((FAILS + 1))
fi

mark "result"
if [ "$FAILS" -eq 0 ]; then
    echo "MICRODUCK-CAKE SMOKE: PASS"
    exit 0
fi
echo "MICRODUCK-CAKE SMOKE: $FAILS failure(s)"
exit 1
