#!/bin/sh
# Build the Rust interop host, or skip it cleanly when rustc is absent so
# `make interop` still runs without a Rust toolchain.
#
# Usage: sh build_rust.sh [rustc] [output-binary]
set -eu

RUSTC="${1:-rustc}"
OUT="${2:-build/kernel_demo_rust}"

if ! command -v "$RUSTC" >/dev/null 2>&1; then
  echo "rustc not found — skipping the Rust host (install Rust or set RUSTC=)" >&2
  exit 0
fi

"$RUSTC" -O -o "$OUT" rust/kernel_demo.rs -l dl
