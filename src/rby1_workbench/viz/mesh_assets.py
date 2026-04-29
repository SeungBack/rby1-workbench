"""Helpers for optional RB-Y1 mesh visualization in Rerun."""

from __future__ import annotations

import logging
from pathlib import Path


def discover_default_mesh_dir(model_name: str) -> Path | None:
    """Find the SDK mesh directory for the selected robot model."""
    normalized_model = model_name.lower()
    candidates = [
        Path("/home/kimm/Workspaces/RBY1Teleop/urdf/rby1") / f"rby1{normalized_model}" / "urdf_new" / "meshes",
        Path("/home/kimm/Workspaces/rby1-sdk/models") / f"rby1{normalized_model}" / "urdf" / "meshes",
        Path(__file__).resolve().parents[4] / "rby1-sdk" / "models" / f"rby1{normalized_model}" / "urdf" / "meshes",
    ]

    for candidate in candidates:
        if candidate.is_dir() and any(candidate.glob("*.glb")):
            return candidate
    return None


def default_link_mesh_map(mesh_dir: Path) -> dict[str, Path]:
    """Map supported link names to local GLB mesh assets."""
    asset_names = {
        "base": "base.glb",
        "link_torso_0": "LINK_1.glb",
        "link_torso_1": "LINK_2.glb",
        "link_torso_2": "LINK_3.glb",
        "link_torso_3": "LINK_4.glb",
        "link_torso_4": "LINK_5.glb",
        "link_torso_5": "LINK_6.glb",
        "link_right_arm_0": "LINK_7.glb",
        "link_right_arm_1": "LINK_8.glb",
        "link_right_arm_2": "LINK_9.glb",
        "link_right_arm_3": "LINK_10.glb",
        "link_right_arm_4": "LINK_11.glb",
        "link_right_arm_5": "LINK_12.glb",
        "link_right_arm_6": "LINK_13.glb",
        "link_left_arm_0": "LINK_14.glb",
        "link_left_arm_1": "LINK_15.glb",
        "link_left_arm_2": "LINK_16.glb",
        "link_left_arm_3": "LINK_17.glb",
        "link_left_arm_4": "LINK_18.glb",
        "link_left_arm_5": "LINK_19.glb",
        "link_left_arm_6": "LINK_20.glb",
        "ee_right": "EE_BODY.glb",
        "ee_left": "EE_BODY.glb",
        "link_head_0": "PAN_TILT_1.glb",
        "link_head_1": "PAN_TILT_2.glb",
        "link_head_2": "PAN_TILT_3.glb",
    }

    mesh_paths: dict[str, Path] = {}
    for link_name, asset_name in asset_names.items():
        asset_path = mesh_dir / asset_name
        if asset_path.is_file():
            mesh_paths[link_name] = asset_path
        else:
            logging.debug("Mesh asset missing for %s: %s", link_name, asset_path)
    return mesh_paths
