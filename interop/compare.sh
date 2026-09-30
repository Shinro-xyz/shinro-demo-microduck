#!/bin/sh
# Drive one shinro_step from each of the four language hosts and assert they agree.
#
# Usage: sh compare.sh <path/to/lib_neural_network.so> <bin-dir> [python] [repo-root]
set -eu

SO="${1:?usage: compare.sh <so> <bin-dir> [python] [repo-root]}"
BIN="${2:?usage: compare.sh <so> <bin-dir> [python] [repo-root]}"
PYTHON="${3:-python3}"
REPO_ROOT="${4:-.}"

# compare.sh runs with cwd=interop/ (make -C interop), so a relative interpreter
# path like `.venv/bin/python` must be anchored at the repo root. A bare name
# (`python3`) stays a PATH lookup.
case "$PYTHON" in
  /*) : ;;
  */*) PYTHON="$REPO_ROOT/$PYTHON" ;;
  *) : ;;
esac

if [ ! -f "$SO" ]; then
  echo "no $SO — build it from the repo root first:" >&2
  echo "  make compile" >&2
  exit 1
fi

C="$("$BIN/kernel_demo_c" "$SO")"
CPP="$("$BIN/kernel_demo_cpp" "$SO")"
ZIG="$(MICRODUCK_SO="$SO" "$BIN/kernel_demo_zig")"
PY="$("$PYTHON" python/kernel_demo.py "$SO" 2>/dev/null)"

echo "one shinro_step through four languages — same .so, same 61-D input:"
printf '  %-8s %s\n' C "$C"
printf '  %-8s %s\n' C++ "$CPP"
printf '  %-8s %s\n' Zig "$ZIG"
printf '  %-8s %s\n' Python "$PY"

if [ "$C" != "$CPP" ] || [ "$C" != "$ZIG" ] || [ "$C" != "$PY" ]; then
  echo "FAIL: the four languages disagree" >&2
  exit 1
fi
echo "  => all four agree (bit-identical f64 across the C ABI)"

# Re-run the Python host so its MicroduckPolicy cross-check prints to stderr.
"$PYTHON" python/kernel_demo.py "$SO" >/dev/null
