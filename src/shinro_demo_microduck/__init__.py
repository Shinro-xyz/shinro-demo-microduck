"""Microduck reference-robot demo for the shinro control framework.

Importing this package registers its ``[physics].preset = "microduck"`` with
shinro's physics-preset registry, so scenarios that declare the preset resolve
without an extra ``--import``. It also re-exports the asset paths, the policy
contract, and the simulation helpers the demos and tests share.
"""

from shinro_demo_microduck import contract
from shinro_demo_microduck import presets as _presets  # noqa: F401  (registers "microduck" on import)
from shinro_demo_microduck.paths import ASSETS, DEFAULT_ARTIFACT, MESH_DIR, POLICY_ONNX, SCENE_WALK
from shinro_demo_microduck.presets import load_scene_assets
from shinro_demo_microduck.sim import CONTROL_DT, MicroduckSim

__all__ = [
    "ASSETS",
    "CONTROL_DT",
    "DEFAULT_ARTIFACT",
    "MESH_DIR",
    "POLICY_ONNX",
    "SCENE_WALK",
    "MicroduckSim",
    "contract",
    "load_scene_assets",
]
