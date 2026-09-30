"""Filesystem locations of the bundled Microduck MuJoCo assets.

``HERE`` is this package directory, so the paths work both from a source
checkout and from an installed wheel (the ``assets/`` tree is package data).
"""

from pathlib import Path

HERE = Path(__file__).parent
ASSETS = HERE / "assets" / "microduck"
SCENE_WALK = ASSETS / "scene_walk.xml"
ROBOT_WALK = ASSETS / "robot_walk.xml"
MESH_DIR = ASSETS / "assets"

#: The trained walking policy, vendored so the compile is reproducible offline.
#: Provenance: HuggingFace ``xiaofengzi/microduck-walking-onnx`` == the ONNX
#: export of the deployed walk (wandb ``441tzs6d`` @ ``model_3750``).
POLICY_ONNX = HERE.parent.parent / "models" / "microduck" / "BEST_alpha_walking.onnx"

#: The controller config the compile scenario and the replay both read.
CONTROLLER_CONFIG = HERE.parent.parent / "configs" / "controllers" / "onnx_rl_microduck.toml"

#: Where ``make compile`` installs the kernel (``lib/lib_neural_network.so``).
DEFAULT_ARTIFACT = HERE.parent.parent / "build" / "compiled_policy"
