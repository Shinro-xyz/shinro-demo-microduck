.PHONY: install test test-quick demo live gif trajectory trajectory-gifs compile run footprint

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

# Watch it: real-time MuJoCo viewer window, keys 0-4 switch the command,
# R resets, ESC quits. Needs a display (MUJOCO_GL=glfw on a desktop).
live:
	$(PYTHON) -m demos.demo_compiled_policy --live

# Render the small showcase GIF the README embeds (committed under docs/media/).
gif:
	$(PYTHON) -m demos.demo_compiled_policy --sequence --gif compact

# Walk a preset reference path: waypoints -> follower -> compiled policy.
#   make trajectory T=circle LAPS=2     (circle|figure_eight|straight|slalom|waypoints_example)
# Full-length runs land in build/demos/ (they get big); `trajectory-gif` renders the
# short one the README embeds.
T ?= circle
LAPS ?= 1
trajectory:
	$(PYTHON) -m demos.demo_compiled_policy --trajectory $(T) --laps $(LAPS)

# The committed trajectory GIFs in docs/media/ — ONE FULL LAP each, so every path
# visibly closes. The 'trajectory' render profile is trimmed (8.3 fps, 288 px, 64
# colours) because these run for a whole lap: 20-70 s, not the 12 s of the walking
# showcase. Runs in a loop so the total is measurable in one go.
TRAJECTORY_PRESETS = circle figure_eight straight slalom waypoints_example
trajectory-gifs:
	@for t in $(TRAJECTORY_PRESETS); do \
	  $(PYTHON) -m demos.demo_compiled_policy --trajectory $$t --laps 1 --gif trajectory --out-dir docs/media; \
	done

# Per-command GIFs (composite: 3-D view, bird's-eye path, velocity tracking).
demo:
	$(PYTHON) -m demos.demo_compiled_policy

# Compare the policy's runtimes (onnxruntime / shinro adapter / deployment host):
# installed footprint, process RSS, time to first inference, latency, agreement.
footprint:
	$(PYTHON) scripts/compare_backends.py --out build/backend_comparison.md

# Rebuild the kernel and immediately replay it in MuJoCo.
run: compile demo

test:
	$(PYTHON) -m pytest tests/ -v --tb=short

# Fast import/contract smoke check (no simulations, no kernel).
test-quick:
	$(PYTHON) -m pytest tests/ -v --tb=short -m "not integration and not kernel"
