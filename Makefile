.PHONY: install test test-quick demo compile run

# Path to the shinro framework checkout (sibling by default).
SHINRO ?= ../shinro-python-modules
# Interpreter for the demo/tests. Override when the deps live in a venv:
#   make demo PYTHON=.venv/bin/python
PYTHON ?= python3

# Install the framework checkout + this repo's extras.
install:
	pip install -e "$(SHINRO)[onnx-rl]"
	pip install -e ".[mujoco,media,dev]"

# Compile the walking policy ONNX into a Zig kernel (`lib/lib_neural_network.so`
# + graph manifest). `shinro build` runs the oracle gate: the graph is lowered,
# cross-compiled, then checked `.so`-vs-interpreter before the artifact is stamped.
compile:
	shinro build scenarios/microduck_walking.toml --out build/compiled_policy

demo:
	$(PYTHON) -m demos.demo_compiled_policy

# Rebuild the kernel and immediately replay it in MuJoCo.
run: compile demo

# Full suite: contract, sim, import fidelity, and kernel lockstep parity.
test:
	$(PYTHON) -m pytest tests/ -v --tb=short

# Fast import/contract smoke check (no simulations, no kernel).
test-quick:
	$(PYTHON) -m pytest tests/ -v --tb=short -m "not integration and not kernel"
