#!/bin/sh
# Drive one shinro_step from each language host and assert they agree.
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

echo "one shinro_step through the compiled kernel — same .so, same 61-D input:"
printf '  %-8s %s\n' C "$C"
printf '  %-8s %s\n' C++ "$CPP"
printf '  %-8s %s\n' Zig "$ZIG"

HOSTS=4  # C, C++, Zig, Python
RUST="$C"  # default: skipped, so it cannot affect the comparison below
if [ -x "$BIN/kernel_demo_rust" ]; then
  RUST="$("$BIN/kernel_demo_rust" "$SO")"
  HOSTS=5
  printf '  %-8s %s\n' Rust "$RUST"
else
  printf '  %-8s %s\n' Rust "(skipped — rustc not on PATH)"
fi
printf '  %-8s %s\n' Python "$PY"

if [ "$C" != "$CPP" ] || [ "$C" != "$ZIG" ] || [ "$C" != "$RUST" ] || [ "$C" != "$PY" ]; then
  echo "FAIL: the hosts disagree" >&2
  exit 1
fi
echo "  => all $HOSTS hosts agree (bit-identical f64 across the C ABI)"

# Re-run the Python host so its MicroduckPolicy cross-check prints to stderr.
"$PYTHON" python/kernel_demo.py "$SO" >/dev/null
