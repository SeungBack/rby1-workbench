"""Explicit control target state containers."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass(slots=True)
class JointTargetState:
    """Named joint targets shared by streaming controllers and apps."""

    component_targets: dict[str, np.ndarray] = field(default_factory=dict)

    @classmethod
    def from_component_targets(
        cls,
        component_targets: dict[str, np.ndarray],
    ) -> "JointTargetState":
        return cls(
            component_targets={
                component: np.asarray(target, dtype=np.float64).copy()
                for component, target in component_targets.items()
            }
        )

    def copy(self) -> "JointTargetState":
        return JointTargetState.from_component_targets(self.component_targets)

    def get(self, component: str) -> np.ndarray | None:
        target = self.component_targets.get(component)
        return None if target is None else target.copy()

    def set(self, component: str, target: np.ndarray) -> None:
        self.component_targets[component] = np.asarray(target, dtype=np.float64).copy()

    def as_dict(self) -> dict[str, np.ndarray]:
        return {
            component: target.copy()
            for component, target in self.component_targets.items()
        }


@dataclass(slots=True)
class CartesianTargetState:
    """Named Cartesian targets shared by streaming controllers and apps."""

    arm_targets: dict[str, np.ndarray] = field(default_factory=dict)

    @classmethod
    def from_arm_targets(
        cls,
        arm_targets: dict[str, np.ndarray],
    ) -> "CartesianTargetState":
        return cls(
            arm_targets={
                arm: np.asarray(target, dtype=np.float64).copy()
                for arm, target in arm_targets.items()
            }
        )

    def copy(self) -> "CartesianTargetState":
        return CartesianTargetState.from_arm_targets(self.arm_targets)

    def get(self, arm: str) -> np.ndarray | None:
        target = self.arm_targets.get(arm)
        return None if target is None else target.copy()

    def set(self, arm: str, target: np.ndarray) -> None:
        self.arm_targets[arm] = np.asarray(target, dtype=np.float64).copy()

    def as_dict(self) -> dict[str, np.ndarray]:
        return {
            arm: target.copy()
            for arm, target in self.arm_targets.items()
        }
