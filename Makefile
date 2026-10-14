# shellcheck disable=SC1089,SC2046,SC2068,SC2145,SC2154,SC2276,SC2283,SC2034,SC1091
# (shellcheck parses this file as shell; the Makefile variables/loops below are not shell)

.PHONY: install test test-quick demo live gif media trajectory trajectory-gifs compile run cake fetch-cake footprint check verify gates interop

# The shinro framework is pinned to a PUBLISHED release so a fresh clone works
# with no sibling checkout. Bump SHINRO_VERSION to track a newer tag.
SHINRO_VERSION ?= v0.3.0
SHINRO_WHEEL ?= https://github.com/Shinro-xyz/shinro-python-modules/releases/download/$(SHINRO_VERSION)/shinro-$(SHINRO_VERSION:v%=%)-py3-none-any.whl
# Path to a local shinro checkout, for `make install-editable`.
SHINRO ?= ../shinro-python-modules
# Interpreter for install, the demo and the tests — one interpreter for all
# three. `pip` is deliberately NOT called directly: a bare `pip` is absent from
# PATH on many current distributions (and `python3 -m pip` can be missing too),
# and installing against one interpreter while running against another is the
# classic "ModuleNotFoundError on a package that just installed" trap.
#   make venv && make install PYTHON=.venv/bin/python
PYTHON ?= python3

# A virtualenv to install into. Use this when the machine has no usable pip.
.PHONY: venv
venv:
	python3 -m venv .venv
	.venv/bin/python -m pip install --upgrade pip
	@echo
	@echo "now run:   make install PYTHON=.venv/bin/python"
	@echo "then:      make fetch-cake PYTHON=.venv/bin/python && make cake PYTHON=.venv/bin/python"

# The documented install: the pinned release. No sibling checkout needed.
install:
	$(PYTHON) -m pip install "shinro[onnx-rl] @ $(SHINRO_WHEEL)"
	$(PYTHON) -m pip install -e ".[mujoco,media,dev]"

# Local development against a working tree instead of the pinned release.
install-editable:
	$(PYTHON) -m pip install -e "$(SHINRO)[onnx-rl]"
	$(PYTHON) -m pip install -e ".[mujoco,media,dev]"

# Compile the walking policy ONNX into a Zig kernel (`lib/lib_neural_network.so`
# + graph manifest). `shinro build` runs the oracle gate: the graph is lowered,
# cross-compiled, then checked `.so`-vs-interpreter before the artifact is stamped.
# Cross-compile with `make compile TARGET=aarch64-linux-gnu`; a non-native target
# skips the host oracle (see docs/compiled-inference.md).
TARGET ?= native
compile:
	shinro build scenarios/microduck_walking.toml --out build/compiled_policy --target $(TARGET)

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

# End to end: compile the kernel, then render every GIF the README/docs embed —
# the walking showcase plus one full lap per trajectory preset, all to docs/media/.
#   make media            (native kernel + all six GIFs)
#   make media TARGET=aarch64-linux-gnu
media: compile
	$(MAKE) gif
	$(MAKE) trajectory-gifs

# Per-command GIFs (composite: 3-D view, bird's-eye path, velocity tracking).
demo:
	$(PYTHON) -m demos.demo_compiled_policy

# Compare the policy's runtimes (onnxruntime / shinro adapter / deployment host):
# installed footprint, process RSS, time to first inference, latency, agreement.
footprint:
	$(PYTHON) scripts/compare_backends.py --out build/backend_comparison.md

# ─── framework gates ─────────────────────────────────────────────────────────
# The two `shinro` gates that apply to this repo. Both are cheap (no sim, no
# rebuild) and run in CI after `make compile`.

# Construction gate: every component this repo ships is built through shinro's
# own factory, and a bad config fails loudly here instead of at run time.
check:
	shinro check configs/controllers/onnx_rl_microduck.toml
	shinro check configs/controllers/pure_pursuit.toml --import shinro_demo_microduck.tracker

# Drift gate: re-hash the stamped artifacts against the deployment record that
# `make compile` wrote — a hand-edited `.so` or a moved config is caught here.
verify:
	shinro verify scenarios/microduck_walking.toml --out build/compiled_policy

gates: check verify

# Cross-language C-ABI interop: the same `.so` driven through C, C++, Zig, Rust,
# and Python with one input (see interop/README.md). Proves the kernel needs no
# Python; needs a C/C++ compiler and Zig on PATH (`rustc` is optional locally).
interop:
	$(MAKE) -C interop SO=$(abspath build/compiled_policy/lib/lib_neural_network.so) PYTHON=$(PYTHON) REPO_ROOT=$(CURDIR)

# Rebuild the kernel and immediately replay it in MuJoCo.
run: compile demo

# The same walk with the policy on the **Cake control plane** instead of the
# Linux dlopen host: the Cake resident admits the shipped policy capsule, the
# module owns its thread, and this repo's MuJoCo client is the plant across a
# socket. The resident and the capsule are a precompiled Cake release (built in
# the private Cake tree, published per target); no Cake source lives here.
#   make fetch-cake            # download + verify the release for this target
#   make cake                  # circle -> eight -> slalom live, 24 s, rendered
#   make cake SECS=9           # shorter
#   make cake SWAP=circle,eight SECS=8
TRAJ ?= circle
SWAP ?= circle,eight,slalom
SECS ?= 24
# GIF=cake|full|compact|trajectory, or GIF= to skip. The walk GIF and the
# swap-handling figure are written to MEDIA_DIR (committed under docs/media/cake/).
GIF ?= cake
MEDIA_DIR ?= docs/media/cake
# ANCHOR=1 selects the anchored trajectory capsules (each path re-parented onto
# the duck at its swap) instead of the world-fixed ones.
ANCHOR ?=
fetch-cake:
	bin/fetch-release.sh
# Is the fetched tarball shippable? Checks the dependency floor and scans for
# build-tree paths. A tarball that fails either one must not be published.
cake-verify:
	bin/verify.sh
cake-doctor:
	bin/doctor.sh
cake:
	MEDIA_DIR=$(MEDIA_DIR) GIF=$(GIF) ANCHOR=$(ANCHOR) bin/run-demo.sh --seconds $(SECS) --trajectory $(TRAJ) $(if $(SWAP),--swap $(SWAP))

test:
	$(PYTHON) -m pytest tests/ -v --tb=short

# Fast import/contract smoke check (no simulations, no kernel).
test-quick:
	$(PYTHON) -m pytest tests/ -v --tb=short -m "not integration and not kernel"
