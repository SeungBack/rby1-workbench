"""Minimal SE(3) helpers used across visualization and calibration code."""

from __future__ import annotations

import numpy as np


def identity() -> np.ndarray:
    """Return a 4x4 identity transform."""
    return np.eye(4, dtype=np.float64)


def compose(lhs: np.ndarray, rhs: np.ndarray) -> np.ndarray:
    """Compose two homogeneous transforms."""
    return np.asarray(lhs @ rhs, dtype=np.float64)


def inverse(transform: np.ndarray) -> np.ndarray:
    """Invert a homogeneous transform."""
    rotation = transform[:3, :3]
    translation = transform[:3, 3]

    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ translation
    return result


def relative_transform(parent_in_root: np.ndarray, child_in_root: np.ndarray) -> np.ndarray:
    """Compute the child pose expressed in the parent frame."""
    return compose(inverse(parent_in_root), child_in_root)
