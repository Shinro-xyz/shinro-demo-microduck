"""Microduck reference-robot demo for the shinro control framework.

The package import is deliberately **light**: it must not drag in the framework,
numpy, or MuJoCo, because the compiled-policy host —
:class:`shinro_demo_microduck.policy.MicroduckPolicy` — is the code that ships to
the robot and it is stdlib-only::

    from shinro_demo_microduck.policy import MicroduckPolicy   # ctypes + stdlib, no deps

Everything else is re-exported lazily (PEP 562), so ``import shinro_demo_microduck``
stays cheap and each name costs only what it needs. Two consequences:

* The simulation side wants the extras: ``pip install -e ".[mujoco,media,dev]"``.
* The ``[physics].preset = "microduck"`` seam lives in
  :mod:`shinro_demo_microduck.presets` and is registered by importing *that*
  module — pulling the framework in is precisely what this module avoids. A
  scenario that declares the preset is therefore built with
  ``shinro build … --import shinro_demo_microduck.presets``.
"""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING, Any

__all__ = [
    "ASSETS",
    "CONTROL_DT",
    "DEFAULT_ARTIFACT",
    "MESH_DIR",
    "POLICY_ONNX",
    "SCENE_WALK",
    "MicroduckPolicy",
    "MicroduckSim",
    "contract",
    "load_scene_assets",
]

#: name -> (module, attribute). ``None`` means "the module itself".
_LAZY: dict[str, tuple[str, str | None]] = {
    "ASSETS": ("shinro_demo_microduck.paths", "ASSETS"),
    "CONTROL_DT": ("shinro_demo_microduck.sim", "CONTROL_DT"),
    "DEFAULT_ARTIFACT": ("shinro_demo_microduck.paths", "DEFAULT_ARTIFACT"),
    "MESH_DIR": ("shinro_demo_microduck.paths", "MESH_DIR"),
    "POLICY_ONNX": ("shinro_demo_microduck.paths", "POLICY_ONNX"),
    "SCENE_WALK": ("shinro_demo_microduck.paths", "SCENE_WALK"),
    "MicroduckPolicy": ("shinro_demo_microduck.policy", "MicroduckPolicy"),
    "MicroduckSim": ("shinro_demo_microduck.sim", "MicroduckSim"),
    "contract": ("shinro_demo_microduck.contract", None),
    "load_scene_assets": ("shinro_demo_microduck.presets", "load_scene_assets"),
}


def __getattr__(name: str) -> Any:
    """Resolve a re-export on first access (PEP 562), then cache it."""
    entry = _LAZY.get(name)
    if entry is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module_name, attribute = entry
    module = importlib.import_module(module_name)
    value = module if attribute is None else getattr(module, attribute)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted({*globals(), *_LAZY})


if TYPE_CHECKING:  # static analysers and IDEs: real imports, no runtime cost
    from shinro_demo_microduck import contract as contract
    from shinro_demo_microduck.paths import ASSETS, DEFAULT_ARTIFACT, MESH_DIR, POLICY_ONNX, SCENE_WALK
    from shinro_demo_microduck.policy import MicroduckPolicy as MicroduckPolicy
    from shinro_demo_microduck.presets import load_scene_assets as load_scene_assets
    from shinro_demo_microduck.sim import CONTROL_DT as CONTROL_DT
    from shinro_demo_microduck.sim import MicroduckSim as MicroduckSim
