"""Reusable joint presets referenced by control apps."""

from __future__ import annotations

from typing import Any

import numpy as np


_READY_POSE_RAD = {
    "A": {
        "torso": np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0]),
        "right_arm": np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
        "left_arm": np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
    },
    "M": {
        "torso": np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0]),
        "right_arm": np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
        "left_arm": np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
    },
    "UB": {
        "torso": np.deg2rad([10.0, 0.0]),
        "right_arm": np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
        "left_arm": np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
    },
}


def ready_pose_targets_for_model(model: Any) -> dict[str, np.ndarray]:
    """Return the SDK 22_joint_impedance_control ready pose for this model."""
    model_name = str(model.model_name).upper()
    pose = _READY_POSE_RAD.get(model_name)
    if pose is None:
        raise ValueError(f"Unsupported model for ready pose preset: {model_name}")

    targets: dict[str, np.ndarray] = {}
    expected_sizes = {
        "torso": len(model.torso_idx),
        "right_arm": len(model.right_arm_idx),
        "left_arm": len(model.left_arm_idx),
    }
    for component, expected_size in expected_sizes.items():
        values = np.asarray(pose[component], dtype=np.float64)
        if len(values) != expected_size:
            raise ValueError(
                f"Ready pose preset for {model_name}/{component} has {len(values)} joints, "
                f"but the model exposes {expected_size}"
            )
        targets[component] = values.copy()

    return targets
