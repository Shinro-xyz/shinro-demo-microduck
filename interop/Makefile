# shellcheck disable=SC2046  # Makefile variables look like bare shell substitutions to shellcheck

# Cross-language C-ABI interop for the compiled Microduck policy kernel.
#
# Build the artifact first (from the repo root):
#   make compile          # == shinro build scenarios/microduck_walking.toml --out build/compiled_policy
#
# Then:   make -C interop          (or `make interop` from the repo root)
#
# The same .so is driven through C, C++, Zig, and Python with an identical 61-D
# input; the four printed 14-D action vectors must agree bit for bit.

SO  ?= ../build/compiled_policy/lib/lib_neural_network.so
CC  ?= cc
CXX ?= c++
ZIG ?= zig
PYTHON ?= python3
REPO_ROOT ?= ..

BIN := build

.PHONY: all run clean

all: run

$(BIN):
	mkdir -p $(BIN)

$(BIN)/kernel_demo_c: c/kernel_demo.c | $(BIN)
	$(CC) -O2 -o $(BIN)/kernel_demo_c c/kernel_demo.c -ldl

$(BIN)/kernel_demo_cpp: cpp/kernel_demo.cpp | $(BIN)
	$(CXX) -std=c++17 -O2 -o $(BIN)/kernel_demo_cpp cpp/kernel_demo.cpp -ldl

$(BIN)/kernel_demo_zig: zig/kernel_demo.zig | $(BIN)
	$(ZIG) build-exe -O ReleaseFast -femit-bin=$(BIN)/kernel_demo_zig zig/kernel_demo.zig -lc

run: $(BIN)/kernel_demo_c $(BIN)/kernel_demo_cpp $(BIN)/kernel_demo_zig
	@sh compare.sh "$(SO)" "$(BIN)" "$(PYTHON)" "$(REPO_ROOT)"

clean:
	rm -rf $(BIN)
