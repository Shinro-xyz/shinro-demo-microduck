"""The demo's CLI surface: watching it, and the pieces that need no GL.

The render/GL paths are exercised by ``make demo`` / ``make gif``; what is worth
locking here is the logic around them — the live key bindings, the compact render
profile actually being smaller, the showcase sequence naming real commands, and
the panels re-annotating when a run switches command mid-flight (a bug that made
the first showcase GIF advertise the idle command while the robot was walking).
"""

# pyright: reportArgumentType=false, reportIndexIssue=false
#   matplotlib's stubs type Line2D.get_ydata() as a Buffer union, so indexing
#   the returned array and float()-ing an element is misreported here. The
#   assertions are correct at runtime; this repo's lint gate is ruff.

import numpy as np
import pytest

from demos.demo_compiled_policy import (
    COMMANDS,
    LIVE_KEYS,
    LIVE_QUIT_KEY,
    LIVE_RESET_KEYS,
    SEQUENCE,
)
from shinro_demo_microduck.viz import COMPACT, Render


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


def test_swap_panel_picks_the_active_segment():
    """The Cake panel colours by whichever trajectory module is active at t."""
    from shinro_demo_microduck.viz import SwapPanels

    panels = SwapPanels([("circle", 0.0), ("eight", 3.0), ("slalom", 6.0)], duration=9.0)
    assert panels.segment_at(0.0) == 0
    assert panels.segment_at(2.99) == 0
    assert panels.segment_at(3.0) == 1
    assert panels.segment_at(8.9) == 2
    panels.close()


def test_cake_reference_mirrors_the_capsule_shapes():
    """The client mirrors the reference each trajectory capsule was compiled from.

    The capsule bakes ``circle|eight|slalom|forward`` with the same parameter
    defaults this test pins, so the cross-track error is measured against the path
    the active module was actually steering toward.
    """
    from demos.demo_cake_policy import _reference_for

    circle = _reference_for("circle")
    assert circle is not None and circle.closed
    # Circle through the origin with radius 1: x in [-1, 1], y in [0, 2].
    assert np.allclose(circle.points.min(axis=0), [-1.0, 0.0], atol=1e-9)
    assert np.allclose(circle.points.max(axis=0), [1.0, 2.0], atol=1e-9)

    eight = _reference_for("eight")
    assert eight is not None and eight.closed
    assert np.isclose(np.abs(eight.points[:, 0]).max(), 1.2)

    slalom = _reference_for("slalom")
    assert slalom is not None and not slalom.closed
    assert np.isclose(slalom.points[:, 0].max(), 4.0)

    forward = _reference_for("forward")
    assert forward is not None and not forward.closed
    assert np.isclose(forward.points[:, 0].max(), 2.0)
    assert np.allclose(forward.points[:, 1], 0.0)

    # A schedule name with no baked reference (the bare default) is not an error.
    assert _reference_for("trajectory") is None


def test_swap_panel_tracks_cross_track_per_segment():
    """The error row follows the active module, not one global path."""
    from demos.demo_cake_policy import _reference_for
    from shinro_demo_microduck.viz import SwapPanels

    segments = [("circle", 0.0), ("eight", 3.0)]
    circle, eight = _reference_for("circle"), _reference_for("eight")
    assert circle is not None and eight is not None
    panels = SwapPanels(segments, duration=6.0, references=[circle.points, eight.points])
    panels.update(0.0, np.array([0.0, 0.0]), 0, 0.1)
    panels.update(3.0, np.array([1.0, 0.0]), 1, 0.2)
    assert panels._cte[0] == [0.1]
    assert panels._cte[1] == [0.2]
    assert list(panels.lines_err[0].get_ydata()) == [0.1]
    assert list(panels.lines_err[1].get_ydata()) == [0.2]
    # The y-limit opens up to the largest error seen.
    assert panels.axerr.get_ylim()[1] >= 0.2
    panels.close()


def test_swap_panel_tolerates_an_unknown_reference():
    """No reference known -> the error row stays empty, not a fake zero."""
    from shinro_demo_microduck.viz import SwapPanels

    panels = SwapPanels([("trajectory", 0.0)], duration=1.0)
    panels.update(0.0, np.array([0.0, 0.0]), 0, None)
    assert panels._cte[0] == []
    panels.close()
