"""Registered ``[physics].preset`` factory for the Microduck walk model.

A preset is shinro's plugin seam for turning a scenario's ``[physics]`` section
into an engine model. Importing *this module* registers ``"microduck"`` with
``shinro.factories.registry``, so a scenario that declares
``[physics].preset = "microduck"`` is built with::

    shinro build scenarios/<name>.toml --import shinro_demo_microduck.presets

Registration deliberately does **not** happen on ``import shinro_demo_microduck``:
importing the framework costs ~58 MB RSS, and the compiled-policy host
(:mod:`shinro_demo_microduck.policy`) is the code that ships to the robot — it
must stay stdlib-only. See the package docstring.

The Microduck MJCF is a scene that ``<include>``s the robot and keeps its meshes
in a subdirectory (``<compiler meshdir="assets">``). MuJoCo's
``from_xml_string`` resolves both out of the ``assets`` mapping, so the keys must
be the *scene-relative* paths (``robot_walk.xml``, ``assets/foot_left.stl``),
not bare filenames.
"""

from __future__ import annotations

from pathlib import Path

from shinro.factories.registry import PhysicsModel, register_physics_preset

from shinro_demo_microduck.paths import ASSETS, MESH_DIR, ROBOT_WALK, SCENE_WALK


def load_scene_assets(assets_dir: Path = ASSETS, mesh_dir: Path = MESH_DIR) -> dict[str, bytes]:
    """Load the scene's include + mesh files keyed by their scene-relative path.

    Args:
        assets_dir: Directory holding the scene MJCF and its mesh subdirectory.
        mesh_dir: The ``meshdir`` subdirectory (``assets``).

    Returns:
        ``{relative_path: bytes}`` for the robot include and every mesh.
    """
    assets: dict[str, bytes] = {ROBOT_WALK.name: ROBOT_WALK.read_bytes()}
    mesh_prefix = mesh_dir.name
    if mesh_dir.exists():
        for fname in sorted(mesh_dir.iterdir()):
            if fname.suffix.lower() in (".stl", ".obj", ".msh"):
                assets[f"{mesh_prefix}/{fname.name}"] = fname.read_bytes()
    return assets


@register_physics_preset("microduck")
def microduck_preset(cfg: dict) -> PhysicsModel:
    """Build the Microduck walk scene from the bundled MJCF + meshes.

    Args:
        cfg: The scenario's ``[physics]`` section (unused — the walk scene has
            one robot and no configurable parts).

    Returns:
        :class:`~shinro.factories.registry.PhysicsModel` holding the scene MJCF
        text and the mesh assets.
    """
    del cfg  # the preset has nothing to vary
    return PhysicsModel(xml_string=SCENE_WALK.read_text(), assets=load_scene_assets())
