"""The demo's CLI surface: watching it, and the pieces that need no GL.

The render/GL paths are exercised by ``make demo`` / ``make gif``; what is worth
locking here is the logic around them — the live key bindings, the compact render
profile actually being smaller, the showcase sequence naming real commands, and
the panels re-annotating when a run switches command mid-flight (a bug that made
the first showcase GIF advertise the idle command while the robot was walking).
"""

import numpy as np
import pytest

from demos.demo_compiled_policy import (
    COMMANDS,
    COMPACT,
    LIVE_KEYS,
    LIVE_QUIT_KEY,
    LIVE_RESET_KEYS,
    SEQUENCE,
    Render,
)


def test_live_keys_cover_every_command_in_order():
    assert LIVE_KEYS[ord("0")] == "idle"
    assert LIVE_KEYS[ord("1")] == "forward"
    assert sorted(LIVE_KEYS.values()) == sorted(COMMANDS)
    assert len(set(LIVE_KEYS)) == len(LIVE_KEYS), "key bindings must not collide"


def test_live_reset_and_quit_keys():
    assert LIVE_QUIT_KEY == 256  # GLFW_KEY_ESCAPE
    assert ord("R") in LIVE_RESET_KEYS and ord("r") in LIVE_RESET_KEYS


def test_sequence_names_real_commands_and_has_a_duration():
    assert SEQUENCE, "the showcase sequence must not be empty"
    assert all(name in COMMANDS for name, _ in SEQUENCE)
    assert sum(d for _, d in SEQUENCE) > 0


def test_compact_profile_is_smaller_than_the_default():
    default = Render()
    assert COMPACT.frame_wh[0] < default.frame_wh[0]
    assert COMPACT.composite_width < default.composite_width
    assert COMPACT.every > default.every
    assert COMPACT.colors <= default.colors
    # fps must stay ≥ ~8, or the walk reads as a slideshow.
    assert COMPACT.fps >= 8.0


def test_render_fps_follows_the_control_rate():
    assert Render(every=1).fps == pytest.approx(50.0)
    assert Render(every=5).fps == pytest.approx(10.0)


def test_panels_reannotate_when_the_command_changes():
    """A run that switches command mid-flight must not keep the stale annotation."""
    from shinro_demo_microduck import contract
    from shinro_demo_microduck.viz import ReplayPanels

    panels = ReplayPanels(contract.build_command(twist=(0.0, 0.0, 0.0)), duration=12.0)
    assert (panels.cmd_vx, panels.cmd_vy) == (0.0, 0.0)

    panels.set_command(contract.build_command(twist=(0.4, 0.0, 0.8)))
    assert (panels.cmd_vx, panels.cmd_vy, panels.cmd_wz) == (0.4, 0.0, 0.8)
    assert "+0.40" in panels.ax_vel.get_title()
    assert "+0.80" in panels.ax_vel.get_title()
    # the dashed reference lines follow the command too
    assert [float(line.get_ydata()[0]) for line in panels._cmd_lines] == [0.4, 0.0]
    panels.close()


def test_panels_keep_the_command_lines_on_screen():
    """The y-limits must accommodate the command, not only the measured trace."""
    from shinro_demo_microduck import contract
    from shinro_demo_microduck.viz import ReplayPanels

    panels = ReplayPanels(contract.build_command(twist=(0.4, 0.0, 0.0)), duration=5.0)
    panels.update(0.0, np.zeros(2), np.zeros(3), 0.0)  # nothing measured yet
    lo, hi = panels.ax_vel.get_ylim()
    assert lo <= -0.4 and hi >= 0.4, f"command line at ±0.4 outside ylim {(lo, hi)}"
    panels.close()
